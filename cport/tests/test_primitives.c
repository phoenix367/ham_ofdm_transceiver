/* Golden-vector tests for the primitive modules: bit-exact against the
 * Python fixed-point model (no tolerances -- integer DSP has no excuse). */
#include <stdio.h>
#include <string.h>

#include "../src/fxp.h"
#include "../src/fft.h"
#include "../src/dsp.h"
#include "test_vectors.h"

static int g_pass, g_fail;

static void check(const char *name, int ok)
{
    printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (ok)
        g_pass++;
    else
        g_fail++;
}

static int arr_eq(const int64_t *a, const int64_t *b, int n)
{
    int i;
    for (i = 0; i < n; i++)
        if (a[i] != b[i])
            return 0;
    return 1;
}

#define FFT_CASE(N)                                                        \
    do {                                                                   \
        int64_t re[N], im[N];                                              \
        memcpy(re, FFT##N##_IN_RE, sizeof(re));                            \
        memcpy(im, FFT##N##_IN_IM, sizeof(im));                            \
        fft_fixed(re, im, N);                                              \
        check("fft_fixed " #N, arr_eq(re, FFT##N##_OUT_RE, N) &&           \
                               arr_eq(im, FFT##N##_OUT_IM, N));            \
    } while (0)

int main(void)
{
    check("platform arithmetic shift", fxp_selftest() == 0);

    FFT_CASE(128);
    FFT_CASE(256);
    FFT_CASE(512);

    {
        int64_t re[128], im[128];
        memcpy(re, IFFT128_IN_RE, sizeof(re));
        memcpy(im, IFFT128_IN_IM, sizeof(im));
        ifft_fixed(re, im, 128);
        check("ifft_fixed 128", arr_eq(re, IFFT128_OUT_RE, 128) &&
                                arr_eq(im, IFFT128_OUT_IM, 128));
    }

    {
        int64_t re[512], im[512];
        int e;
        memcpy(re, BFP_WEAK_IN_RE, sizeof(re));
        memcpy(im, BFP_WEAK_IN_IM, sizeof(im));
        fft_bfp(re, im, 512, 13, &e);
        check("fft_bfp weak block", e == BFP_WEAK_EXP &&
              arr_eq(re, BFP_WEAK_OUT_RE, 512) &&
              arr_eq(im, BFP_WEAK_OUT_IM, 512));
        memcpy(re, BFP_HOT_IN_RE, sizeof(re));
        memcpy(im, BFP_HOT_IN_IM, sizeof(im));
        fft_bfp(re, im, 512, 13, &e);
        check("fft_bfp hot block", e == BFP_HOT_EXP &&
              arr_eq(re, BFP_HOT_OUT_RE, 512) &&
              arr_eq(im, BFP_HOT_OUT_IM, 512));
    }

    {
        /* golden vectors are int64; the pipeline carries samples as
         * samp_t (int32) -- widen at the comparison boundary */
        static samp_t oi[2048], oq[2048];
        static int64_t wi[2048], wq[2048];
        int i;
        hilbert_analytic(HILBERT_IN, 2048, oi, oq);
        for (i = 0; i < 2048; i++) { wi[i] = oi[i]; wq[i] = oq[i]; }
        check("hilbert analytic", arr_eq(wi, HILBERT_OUT_I, 2048) &&
                                  arr_eq(wq, HILBERT_OUT_Q, 2048));
    }

    {
        static samp_t oi[1024], oq[1024], ini[1024], inq[1024];
        static int64_t wi[1024], wq[1024];
        int i;
        for (i = 0; i < 1024; i++) {
            ini[i] = (samp_t)NCO_IN_I[i];
            inq[i] = (samp_t)NCO_IN_Q[i];
        }
        nco_derotate(ini, inq, 1024, NCO_WORD,
                     (uint32_t)NCO_START, oi, oq);
        for (i = 0; i < 1024; i++) { wi[i] = oi[i]; wq[i] = oq[i]; }
        check("nco derotate", arr_eq(wi, NCO_OUT_I, 1024) &&
                              arr_eq(wq, NCO_OUT_Q, 1024));
        nco_derotate(ini, inq, 1024, NCO_WORD2, 7u, oi, oq);
        for (i = 0; i < 1024; i++) { wi[i] = oi[i]; wq[i] = oq[i]; }
        check("nco derotate (negative word)",
              arr_eq(wi, NCO_OUT2_I, 1024) && arr_eq(wq, NCO_OUT2_Q, 1024));
    }

    {
        int n = (int)(sizeof(CORDIC_Y) / sizeof(CORDIC_Y[0]));
        int i, ok = 1;
        for (i = 0; i < n; i++) {
            int64_t a, m;
            cordic_atan2(CORDIC_Y[i], CORDIC_X[i], &a, &m);
            if (a != CORDIC_ANG[i] || m != CORDIC_MAG[i]) {
                printf("  cordic[%d]: got (%lld,%lld) want (%lld,%lld)\n", i,
                       (long long)a, (long long)m,
                       (long long)CORDIC_ANG[i], (long long)CORDIC_MAG[i]);
                ok = 0;
            }
        }
        check("cordic atan2/mag", ok);
    }

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
