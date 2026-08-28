/* FIR Hilbert transformer, NCO, CORDIC -- C twin of ofdm_phy/fixed/dsp.py.
 * CFO convention: 32-bit phase-increment word (one turn = 2^32). */
#ifndef OFDM_DSP_H
#define OFDM_DSP_H

#include <stdint.h>

/* Analytic sample type.
 *
 * These derive from int16 audio through a unity-gain Hilbert FIR;
 * measured peak across every suite is 43077 (17 bits), so int32 keeps a
 * ~49000x margin at half the storage.
 *
 * Products of TWO samp_t must be widened explicitly -- 43077 squared is
 * 1.9e9 and sits right against INT32_MAX. On Cortex-M7 the widening
 * multiply is SMULL/SMLAL, the same instruction count as MUL/MLA, so
 * being explicit costs nothing. */
#ifndef OFDM_SAMP_T
#define OFDM_SAMP_T
typedef int32_t samp_t;
#endif

#define PHASE_BITS 32
#define NCO_LUT_BITS 12
#define HILBERT_TAPS_N 63
#define HILBERT_DELAY 31

/* analytic signal: I = input delayed by the group delay, Q = FIR output.
 * out_i/out_q are n samples each (edges transient, as in hardware). */
void hilbert_analytic(const int16_t *x, int n, samp_t *out_i, samp_t *out_q);

/* multiply the stream by exp(-j*phase): compensates +word/sample CFO.
 * word is the signed phase increment; start_phase the initial accumulator. */
void nco_derotate(const samp_t *in_i, const samp_t *in_q, int n,
                  int64_t phase_word, uint32_t start_phase,
                  samp_t *out_i, samp_t *out_q);

/* vectoring-mode CORDIC: angle in signed phase-word units, |x + jy| */
void cordic_atan2(int64_t y, int64_t x, int64_t *angle, int64_t *mag);

#endif /* OFDM_DSP_H */
