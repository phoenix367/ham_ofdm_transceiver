/* Fixed-point primitives -- C twin of ofdm_phy/fixed/fxp.py.
 *
 * Conventions: Q15 coefficients, int64_t datapath, round-half-up via
 * add-before-shift, symmetric saturation.
 *
 * Portability note: arithmetic right shift of negative signed integers is
 * assumed (gcc/clang/armcc/ti-cgt all guarantee it); fxp_selftest() verifies
 * at startup. */
#ifndef OFDM_FXP_H
#define OFDM_FXP_H

#include <stdint.h>

#define Q15 15
#define Q15_ONE (1 << Q15)
#define Q15_MAX ((1 << Q15) - 1)

static inline int64_t rshift_round(int64_t x, int shift)
{
    /* arithmetic right shift with round-half-up */
    return (x + ((int64_t)1 << (shift - 1))) >> shift;
}

static inline int64_t sat_bits(int64_t x, int bits)
{
    const int64_t hi = ((int64_t)1 << (bits - 1)) - 1;
    const int64_t lo = -((int64_t)1 << (bits - 1));
    return x > hi ? hi : (x < lo ? lo : x);
}

static inline int16_t sat16(int64_t x)
{
    return (int16_t)sat_bits(x, 16);
}

/* block-floating-point prescale: shift so the block max fills target_bits
 * (RTL: priority encoder on the block max); negative = downshift */
static inline int block_exponent(const int64_t *re, const int64_t *im,
                                 int n, int target_bits)
{
    int64_t m = 1;
    int i, bl = 0;
    for (i = 0; i < n; i++) {
        int64_t a = re[i] < 0 ? -re[i] : re[i];
        int64_t b = im[i] < 0 ? -im[i] : im[i];
        if (a > m) m = a;
        if (b > m) m = b;
    }
    while (m > 0) { m >>= 1; bl++; }
    return target_bits - bl;
}

/* integer square root (RTL: non-restoring root extractor) */
static inline int64_t isqrt_i64(int64_t x)
{
    int64_t r = 0, bit;
    for (bit = (int64_t)1 << 62; bit > x; bit >>= 2)
        ;
    while (bit != 0) {
        if (x >= r + bit) {
            x -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
        bit >>= 2;
    }
    return r;
}

/* returns 0 on a conforming platform */
static inline int fxp_selftest(void)
{
    volatile int64_t neg = -8;
    return ((neg >> 1) == -4 && rshift_round(-3, 1) == -1) ? 0 : 1;
}

#endif /* OFDM_FXP_H */
