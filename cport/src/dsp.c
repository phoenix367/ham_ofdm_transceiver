#include "dsp.h"
#include "fxp.h"
#include "rom_tables.h"

#define CORDIC_ITERS 16

void hilbert_analytic(const int16_t *x, int n, samp_t *out_i, samp_t *out_q)
{
    /* direct-form FIR over a zero-prehistory, matching the Python model's
     * zero padding: q[m] = sum_k taps[k] * x[m - (N-1) + k] */
    int m, k;
    for (m = 0; m < n; m++) {
        int64_t acc = 0;
        for (k = 0; k < HILBERT_TAPS_N; k++) {
            int idx = m - (HILBERT_TAPS_N - 1) + k;
            if (idx >= 0)
                acc += (int64_t)HILBERT_TAPS[HILBERT_TAPS_N - 1 - k] * x[idx];
        }
        out_q[m] = rshift_round(acc, Q15);
        out_i[m] = (m >= HILBERT_DELAY) ? x[m - HILBERT_DELAY] : 0;
    }
}

/* --- stationary-carrier notch (dsp.h) --------------------------------- */

#define NOTCH_MERGE_WORD ((uint32_t)(20.0 * 4294967296.0 / 12000.0)) /* 20 Hz */

void notch_init(notch_t *f, uint32_t word)
{
    int32_t c = NCO_COS[word >> (PHASE_BITS - NCO_LUT_BITS)];  /* Q15 */
    f->b1 = -c;                                     /* -2c in Q14 */
    f->a1 = (int32_t)rshift_round((int64_t)NOTCH_R_Q14 * c, 14); /* 2rc, Q14 */
    f->a2 = NOTCH_R2_Q14;
    f->x1 = f->x2 = f->y1 = f->y2 = 0;
}

int16_t notch_step(notch_t *f, int16_t x)
{
    /* multiply, do not shift: "negative << n" is undefined behaviour in C
     * (UBSan flagged it on the first negative sample) */
    int64_t acc = (int64_t)x * 16384 + (int64_t)f->b1 * f->x1
                  + (int64_t)f->x2 * 16384 + (int64_t)f->a1 * f->y1
                  - (int64_t)f->a2 * f->y2;
    int64_t y = rshift_round(acc, 14);
    if (y > 32767)
        y = 32767;
    else if (y < -32768)
        y = -32768;
    f->x2 = f->x1;
    f->x1 = x;
    f->y2 = f->y1;
    f->y1 = (int32_t)y;
    return (int16_t)y;
}

void notch_bank_clear(notch_bank_t *b)
{
    int i;
    for (i = 0; i < NOTCH_MAX; i++) {
        b->active[i] = 0;
        b->idle[i] = 0;
        b->rej[i] = 0;
        b->word[i] = 0;
    }
    b->tot = 0;
    b->n_tick = 0;
}

int notch_bank_active(const notch_bank_t *b)
{
    int i, n = 0;
    for (i = 0; i < NOTCH_MAX; i++)
        n += b->active[i];
    return n;
}

int notch_bank_near(notch_bank_t *b, uint32_t word, uint32_t tol_word)
{
    int i;
    for (i = 0; i < NOTCH_MAX; i++) {
        uint32_t d;
        if (!b->active[i])
            continue;
        d = b->word[i] - word;
        if (d < tol_word || (uint32_t)(0u - d) < tol_word) {
            b->idle[i] = 0;
            return 1;
        }
    }
    return 0;
}

int notch_bank_request(notch_bank_t *b, uint32_t word)
{
    int i, slot = -1;
    for (i = 0; i < NOTCH_MAX; i++) {
        if (b->active[i]) {
            uint32_t d = b->word[i] - word;
            if (d < NOTCH_MERGE_WORD || (uint32_t)(0u - d) < NOTCH_MERGE_WORD) {
                b->idle[i] = 0;   /* refresh */
                return 1;
            }
        } else if (slot < 0) {
            slot = i;
        }
    }
    if (slot < 0)
        return 0;
    notch_init(&b->f[slot], word);
    b->word[slot] = word;
    b->active[slot] = 1;
    b->idle[slot] = 0;
    b->rej[slot] = 0;
    return 1;
}

int16_t notch_bank_push(notch_bank_t *b, int16_t x)
{
    int i;
    int16_t y = x;
    b->tot += (int64_t)x * x;
    for (i = 0; i < NOTCH_MAX; i++) {
        if (!b->active[i])
            continue;
        {
            int16_t in = y;
            y = notch_step(&b->f[i], in);
            b->rej[i] += (int64_t)(in - y) * (in - y);
        }
    }
    if (++b->n_tick >= NOTCH_TICK) {
        /* release a notch whose rejected component has fallen under 1/32
         * of the input power (the carrier is gone; the 30 Hz of wanted
         * signal it removes is ~1.4 % of the 2.1 kHz band) for
         * NOTCH_IDLE_TICKS in a row */
        for (i = 0; i < NOTCH_MAX; i++) {
            if (!b->active[i])
                continue;
            if (b->rej[i] * 32 <= b->tot || b->rej[i] <= NOTCH_MIN_REJ) {
                if (++b->idle[i] >= NOTCH_IDLE_TICKS)
                    b->active[i] = 0;
            } else {
                b->idle[i] = 0;
            }
            b->rej[i] = 0;
        }
        b->tot = 0;
        b->n_tick = 0;
    }
    return y;
}

void hilbert_analytic_notched(const int16_t *x, int n, notch_bank_t *b,
                              samp_t *out_i, samp_t *out_q)
{
    int16_t line[64];
    int m, k;
    for (m = 0; m < n; m++) {
        int64_t acc = 0;
        line[m & 63] = notch_bank_push(b, x[m]);
        for (k = 0; k < HILBERT_TAPS_N; k++) {
            int idx = m - (HILBERT_TAPS_N - 1) + k;
            if (idx >= 0)
                acc += (int64_t)HILBERT_TAPS[HILBERT_TAPS_N - 1 - k]
                       * line[idx & 63];
        }
        out_q[m] = rshift_round(acc, Q15);
        out_i[m] = (m >= HILBERT_DELAY) ? line[(m - HILBERT_DELAY) & 63] : 0;
    }
}

void nco_derotate(const samp_t *in_i, const samp_t *in_q, int n,
                  int64_t phase_word, uint32_t start_phase,
                  samp_t *out_i, samp_t *out_q)
{
    uint32_t phase = start_phase;
    uint32_t word = (uint32_t)phase_word; /* modulo 2^32, sign folds in */
    int m;
    for (m = 0; m < n; m++) {
        int idx = (int)(phase >> (PHASE_BITS - NCO_LUT_BITS));
        int64_t c = NCO_COS[idx];
        int64_t s = NCO_COS[(idx + 3 * NCO_LUT_N / 4) & (NCO_LUT_N - 1)];
        /* read both inputs BEFORE writing either output: this is called
         * in place (rx_stream's tone stage does), and writing out_i[m]
         * first would corrupt the in_i[m] that out_q[m] still needs */
        int64_t vi = in_i[m], vq = in_q[m];   /* widen for the products */
        out_i[m] = (samp_t)rshift_round(vi * c + vq * s, Q15);
        out_q[m] = (samp_t)rshift_round(vq * c - vi * s, Q15);
        phase += word;
    }
}

void cordic_atan2(int64_t y, int64_t x, int64_t *angle, int64_t *mag)
{
    int64_t a = 0;
    int i;
    if (x < 0) { /* rotate into the right half-plane */
        int64_t t;
        if (y >= 0) {
            t = x; x = y; y = -t;
            a = (int64_t)1 << (PHASE_BITS - 2);
        } else {
            t = x; x = -y; y = t;
            a = -((int64_t)1 << (PHASE_BITS - 2));
        }
    }
    for (i = 0; i < CORDIC_ITERS; i++) {
        int64_t xs = x >> i, ys = y >> i;
        if (y > 0) {
            x += ys;
            y -= xs;
            a += CORDIC_ATAN[i];
        } else {
            x -= ys;
            y += xs;
            a -= CORDIC_ATAN[i];
        }
    }
    *angle = a;
    *mag = (x * CORDIC_GAIN_Q15) >> Q15;
}
