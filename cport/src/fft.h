/* Radix-2 DIT fixed-point FFT with Q15 twiddle ROM -- C twin of
 * ofdm_phy/fixed/fft.py. Per-stage >>1 gives a built-in 1/N scaling.
 * Supported sizes: 128, 256, 512 (ROMs in rom_tables.h). In-place on
 * caller-provided int64_t buffers. */
#ifndef OFDM_FFT_H
#define OFDM_FFT_H

#include <stdint.h>

/* forward transform, 1/N scaling; re/im are modified in place */
int fft_fixed(int64_t *re, int64_t *im, int n);

/* inverse via the conjugate trick (matches numpy ifft for Q15 inputs) */
int ifft_fixed(int64_t *re, int64_t *im, int n);

/* block-floating-point forward: prescales to target_bits, returns the
 * applied exponent via *exp (spectrum = fft_fixed(input) * 2^exp) */
int fft_bfp(int64_t *re, int64_t *im, int n, int target_bits, int *exp);

#endif /* OFDM_FFT_H */
