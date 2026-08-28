#include "dsp.h"
#include "fxp.h"
#include "rom_tables.h"

#define CORDIC_ITERS 16

void hilbert_analytic(const int16_t *x, int n, int64_t *out_i, int64_t *out_q)
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

void nco_derotate(const int64_t *in_i, const int64_t *in_q, int n,
                  int64_t phase_word, uint32_t start_phase,
                  int64_t *out_i, int64_t *out_q)
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
        int64_t vi = in_i[m], vq = in_q[m];
        out_i[m] = rshift_round(vi * c + vq * s, Q15);
        out_q[m] = rshift_round(vq * c - vi * s, Q15);
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
