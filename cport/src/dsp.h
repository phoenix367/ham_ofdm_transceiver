/* FIR Hilbert transformer, NCO, CORDIC -- C twin of ofdm_phy/fixed/dsp.py.
 * CFO convention: 32-bit phase-increment word (one turn = 2^32). */
#ifndef OFDM_DSP_H
#define OFDM_DSP_H

#include <stdint.h>

#define PHASE_BITS 32
#define NCO_LUT_BITS 12
#define HILBERT_TAPS_N 63
#define HILBERT_DELAY 31

/* analytic signal: I = input delayed by the group delay, Q = FIR output.
 * out_i/out_q are n samples each (edges transient, as in hardware). */
void hilbert_analytic(const int16_t *x, int n, int64_t *out_i, int64_t *out_q);

/* multiply the stream by exp(-j*phase): compensates +word/sample CFO.
 * word is the signed phase increment; start_phase the initial accumulator. */
void nco_derotate(const int64_t *in_i, const int64_t *in_q, int n,
                  int64_t phase_word, uint32_t start_phase,
                  int64_t *out_i, int64_t *out_q);

/* vectoring-mode CORDIC: angle in signed phase-word units, |x + jy| */
void cordic_atan2(int64_t y, int64_t x, int64_t *angle, int64_t *mag);

#endif /* OFDM_DSP_H */
