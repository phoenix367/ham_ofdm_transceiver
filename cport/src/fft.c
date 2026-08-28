#include "fft.h"
#include "fxp.h"
/* fft.c owns the shared NCO_COS ROM: it is linked into every binary
 * that uses the table (dsp.c never links without it) */
#define ROM_TABLES_DEFINE
#include "rom_tables.h"

static void bit_reverse(int64_t *re, int64_t *im, int n)
{
    int bits = 0, i, j;
    while ((1 << bits) < n)
        bits++;
    for (i = 0; i < n; i++) {
        int b;
        j = 0;
        for (b = 0; b < bits; b++)
            j |= ((i >> b) & 1) << (bits - 1 - b);
        if (j > i) {
            int64_t t;
            t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
}

int fft_fixed(int64_t *re, int64_t *im, int n)
{
    /* twiddles W_n^k = e^{-2pi i k/n} read straight from the NCO cosine
     * ROM at stride NCO_LUT_N/n: re = cos, im = -sin = -cos(x + 3pi/2) */
    int nst, half, start, k;

    if (n != 128 && n != 256 && n != 512)
        return -1;
    nst = NCO_LUT_N / n;
    bit_reverse(re, im, n);

    for (half = 1; half < n; half *= 2) {
        int step = n / (2 * half);
        for (start = 0; start < n; start += 2 * half) {
            for (k = 0; k < half; k++) {
                int i0 = start + k;
                int i1 = i0 + half;
                int wj = k * step * nst;
                int64_t wr = NCO_COS[wj];
                int64_t wi =
                    -(int64_t)NCO_COS[(wj + 3 * NCO_LUT_N / 4)
                                      & (NCO_LUT_N - 1)];
                int64_t tr = rshift_round(re[i1] * wr - im[i1] * wi, Q15);
                int64_t ti = rshift_round(re[i1] * wi + im[i1] * wr, Q15);
                int64_t r0 = re[i0], i0v = im[i0];
                re[i1] = rshift_round(r0 - tr, 1);
                im[i1] = rshift_round(i0v - ti, 1);
                re[i0] = rshift_round(r0 + tr, 1);
                im[i0] = rshift_round(i0v + ti, 1);
            }
        }
    }
    return 0;
}

int ifft_fixed(int64_t *re, int64_t *im, int n)
{
    int i, rc;
    for (i = 0; i < n; i++)
        im[i] = -im[i];
    rc = fft_fixed(re, im, n);
    for (i = 0; i < n; i++)
        im[i] = -im[i];
    return rc;
}

int fft_bfp(int64_t *re, int64_t *im, int n, int target_bits, int *exp)
{
    int e = block_exponent(re, im, n, target_bits);
    int i;
    if (e >= 0) {
        /* multiply, do not shift: BFP scaling runs over signed spectra
         * and "negative << n" is undefined behaviour in C (UBSan flags
         * it on every suite that touches the FFT). gcc emits an
         * arithmetic shift, so the result has always been right -- but
         * relying on UB is what makes host and target free to differ.
         * The multiply is well defined and bit-identical. */
        int64_t scale = (int64_t)1 << e;
        for (i = 0; i < n; i++) {
            re[i] *= scale;
            im[i] *= scale;
        }
    } else {
        for (i = 0; i < n; i++) {
            re[i] = rshift_round(re[i], -e);
            im[i] = rshift_round(im[i], -e);
        }
    }
    *exp = e;
    return fft_fixed(re, im, n);
}
