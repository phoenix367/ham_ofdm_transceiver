#include <string.h>

#include "conv.h"

/* max trellis steps: extended data frames carry up to 255-byte payloads
 * (36 + 2040 + 6 = 2082 steps); sized with margin, checked at runtime.
 * MCU note: at this depth the dep buffer is ~25 KB and a caller's
 * traceback ~17 KB -- an MCU build without EXT frames can drop this
 * back to 272 (dep 3.2 KB, traceback 2.2 KB). */
#define CONV_MAX_STEPS 2112

static const int POLYS[CONV_SPEED] = { 0133, 0171, 0165 }; /* octal */

/* puncture masks: [stream][phase], periods per rate */
static const uint8_t MASK_R13[3][3] = { {1, 1, 1}, {1, 1, 1}, {1, 1, 1} };
static const uint8_t MASK_R12[3][3] = { {1, 1, 0}, {1, 0, 0}, {0, 1, 0} };
static const uint8_t MASK_R23[3][3] = { {1, 1, 0}, {1, 0, 0}, {0, 0, 0} };
static const uint8_t MASK_R34[3][3] = { {1, 1, 1}, {1, 0, 0}, {0, 0, 0} };

static const uint8_t (*rate_mask(cc_rate_t r))[3]
{
    switch (r) {
    case CC_R12: return MASK_R12;
    case CC_R23: return MASK_R23;
    case CC_R34: return MASK_R34;
    default: return MASK_R13;
    }
}

static int rate_period(cc_rate_t r)
{
    return r == CC_R34 ? 3 : (r == CC_R13 ? 1 : 2);
}

static int parity(uint32_t v)
{
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (int)(v & 1);
}

/* expected symbols per (state, input bit, poly): +1 = logical 1 */
static int8_t g_expected[CONV_STATES][2][CONV_SPEED];
static int g_expected_init;

static void expected_init(void)
{
    int s, b, p;
    if (g_expected_init)
        return;
    for (s = 0; s < CONV_STATES; s++)
        for (b = 0; b < 2; b++) {
            uint32_t reg = (((uint32_t)s << 1) | (uint32_t)b) & ((1u << CONV_K) - 1);
            for (p = 0; p < CONV_SPEED; p++)
                g_expected[s][b][p] = (int8_t)(2 * parity(reg & (uint32_t)POLYS[p]) - 1);
        }
    g_expected_init = 1;
}

int conv_cc_elements(cc_rate_t rate, int bits_count)
{
    const uint8_t (*mask)[3] = rate_mask(rate);
    int period = rate_period(rate);
    int ones = 0, p, c;
    for (p = 0; p < CONV_SPEED; p++)
        for (c = 0; c < period; c++)
            ones += mask[p][c];
    return ((bits_count + CONV_PAD + period - 1) / period) * ones;
}

int conv_encoded_len(cc_rate_t rate, int bits_count)
{
    const uint8_t (*mask)[3] = rate_mask(rate);
    int period = rate_period(rate);
    int e, p, n = 0;
    for (e = 0; e < bits_count + CONV_PAD; e++)
        for (p = 0; p < CONV_SPEED; p++)
            n += mask[p][e % period];
    return n;
}

void conv_encode(cc_rate_t rate, const uint8_t *bits, int bits_count,
                 uint8_t *out)
{
    const uint8_t (*mask)[3] = rate_mask(rate);
    int period = rate_period(rate);
    uint32_t state = 0;
    int e, p, k = 0;
    for (e = 0; e < bits_count + CONV_PAD; e++) {
        uint32_t bit = e < bits_count ? (bits[e] & 1) : 0; /* zero tail */
        state = ((state << 1) | bit) & ((1u << CONV_K) - 1);
        for (p = 0; p < CONV_SPEED; p++)
            if (mask[p][e % period])
                out[k++] = (uint8_t)parity(state & (uint32_t)POLYS[p]);
    }
    /* note: python punctures the full coded stream after encoding, but the
     * mask phase advances with the element index either way -- identical */
}

void conv_decode(cc_rate_t rate, const int64_t *soft, int soft_len,
                 int bits_count, uint8_t *out, uint8_t *work)
{
    const uint8_t (*mask)[3] = rate_mask(rate);
    int period = rate_period(rate);
    int total_steps = bits_count + CONV_PAD;
    int num_el = ((total_steps + period - 1) / period) * period;
    int navail = soft_len;

    /* depunctured soft bits: inputs are quantized LLRs (<= +-254 even
     * after HARQ combining), int32 is generous -- int64 was pure host
     * convenience at 2x the RAM */
    static int32_t dep[CONV_MAX_STEPS * CONV_SPEED];
    int64_t pm[CONV_STATES], pm_next[CONV_STATES];
    int e, p, s, step, k = 0;
    int state;

    expected_init();
    if (total_steps > CONV_MAX_STEPS)
        total_steps = CONV_MAX_STEPS; /* defensive; never hit in-protocol */

    memset(dep, 0, sizeof(dep));
    for (e = 0; e < num_el && e < CONV_MAX_STEPS; e++)
        for (p = 0; p < CONV_SPEED; p++)
            if (mask[p][e % period] && k < navail)
                dep[e * CONV_SPEED + p] = (int32_t)soft[k++];

    for (s = 0; s < CONV_STATES; s++)
        pm[s] = (int64_t)(-1073741824); /* int32 min / 2, as the model */
    pm[0] = 0;

    for (step = 0; step < total_steps; step++) {
        const int32_t *rx = &dep[step * CONV_SPEED];
        for (s = 0; s < CONV_STATES; s++) {
            int in_bit = s & 1;
            int prev0 = s >> 1;
            int prev1 = prev0 + (CONV_STATES >> 1);
            int64_t b0 = 0, b1 = 0;
            for (p = 0; p < CONV_SPEED; p++) {
                b0 += (int64_t)g_expected[prev0][in_bit][p] * rx[p];
                b1 += (int64_t)g_expected[prev1][in_bit][p] * rx[p];
            }
            {
                int64_t m0 = pm[prev0] + b0;
                int64_t m1 = pm[prev1] + b1;
                /* traceback stores 1 bit/state: which predecessor won
                 * (prev is (s>>1) or (s>>1)+32) -- 8x less RAM than a
                 * byte per state, bit-identical result */
                if (m0 >= m1) {
                    pm_next[s] = m0;
                    work[step * (CONV_STATES / 8) + (s >> 3)] &=
                        (uint8_t)~(1u << (s & 7));
                } else {
                    pm_next[s] = m1;
                    work[step * (CONV_STATES / 8) + (s >> 3)] |=
                        (uint8_t)(1u << (s & 7));
                }
            }
        }
        memcpy(pm, pm_next, sizeof(pm));
    }

    state = 0;
    for (s = 1; s < CONV_STATES; s++)
        if (pm[s] > pm[state])
            state = s; /* first max wins ties, as np.argmax */

    for (step = total_steps - 1; step >= 0; step--) {
        int hi = (work[step * (CONV_STATES / 8) + (state >> 3)]
                  >> (state & 7)) & 1;
        if (step < bits_count)
            out[step] = (uint8_t)(state & 1);
        state = (state >> 1) + (hi ? CONV_STATES / 2 : 0);
    }
}
