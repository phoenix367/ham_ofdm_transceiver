/* Golden-vector tests for the bit pipeline (CRC, scrambler, interleaver,
 * convolutional encode + integer Viterbi) -- bit-exact vs the Python model. */
#include <stdio.h>
#include <string.h>

#include "../src/bits.h"
#include "../src/conv.h"
#include "../src/ldpc.h"
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

static int u8_eq(const uint8_t *a, const uint8_t *b, int n)
{
    return memcmp(a, b, (size_t)n) == 0;
}

static int i64_eq(const int64_t *a, const int64_t *b, int n)
{
    int i;
    for (i = 0; i < n; i++)
        if (a[i] != b[i])
            return 0;
    return 1;
}

/* golden LLR vectors are int64; the decode path is llr_t (int32) */
static llr_t g_llr_in[8192];
static const llr_t *as_llr(const int64_t *v, int n)
{
    int i;
    for (i = 0; i < n && i < (int)(sizeof(g_llr_in) / sizeof(g_llr_in[0])); i++)
        g_llr_in[i] = (llr_t)v[i];
    return g_llr_in;
}

int main(void)
{
    check("crc8 lte", crc8_lte(CRC_BITS, 120) == CRC8_WANT);
    check("crc16 ccitt", crc16_ccitt(CRC_BITS, 120) == CRC16_WANT);

    {
        uint8_t out[200];
        scramble_bits(SCR_IN, 200, out);
        check("scramble", u8_eq(out, SCR_OUT, 200));
    }
    {
        /* the golden vectors are int64; the pipeline is llr_t (int32),
         * so convert at the boundary and comparevalue for value */
        llr_t in[200], out[200];
        int64_t wide[200];
        int i, ok = 1;
        for (i = 0; i < 200; i++)
            in[i] = (llr_t)DESCR_IN[i];
        descramble_llrs(in, 200, out);
        for (i = 0; i < 200; i++)
            wide[i] = out[i];
        ok = i64_eq(wide, DESCR_OUT, 200);
        check("descramble (soft)", ok);
    }
    {
        uint8_t out[93];
        interleave_u8(IL_IN, 93, 16, out);
        check("interleave", u8_eq(out, IL_OUT, 93));
    }
    {
        llr_t in[93], out[93];
        int64_t wide[93];
        int i;
        for (i = 0; i < 93; i++)
            in[i] = (llr_t)DL_IN[i];
        deinterleave_i64(in, 93, 16, out);
        for (i = 0; i < 93; i++)
            wide[i] = out[i];
        check("deinterleave", i64_eq(wide, DL_OUT, 93));
    }

#define CONV_CASE(RATE)                                                     \
    do {                                                                    \
        uint8_t coded[512], dec[100];                                       \
        static uint8_t work[CONV_STATES * 128];                             \
        int n = conv_encoded_len(CC_##RATE, 100);                           \
        conv_encode(CC_##RATE, CONV_PAYLOAD, 100, coded);                   \
        check("conv encode " #RATE,                                         \
              n == (int)sizeof(CONV_##RATE##_CODED) &&                      \
              u8_eq(coded, CONV_##RATE##_CODED, n));                        \
        conv_decode(CC_##RATE,                                              \
                    as_llr(CONV_##RATE##_NOISY,                             \
                           (int)(sizeof(CONV_##RATE##_NOISY) / 8)),         \
                    (int)(sizeof(CONV_##RATE##_NOISY) / 8), 100, dec, work);\
        check("viterbi " #RATE " (noisy LLRs)",                             \
              u8_eq(dec, CONV_##RATE##_DEC, 100));                          \
    } while (0)

    CONV_CASE(R13);
    CONV_CASE(R12);
    CONV_CASE(R23);
    CONV_CASE(R34);

    {
        static uint8_t coded[768], dec[256];
        int k = (int)sizeof(LDPC_INFO);
        int n = ldpc_cc_elements(k);
        ldpc_encode(LDPC_INFO, k, coded);
        check("ldpc encode (k=236, shortened)",
              n == (int)sizeof(LDPC_CODED) &&
              u8_eq(coded, LDPC_CODED, n));
        ldpc_decode_int(as_llr(LDPC_NOISY, (int)(sizeof(LDPC_NOISY) / 8)),
                        (int)(sizeof(LDPC_NOISY) / 8), k, dec);
        check("ldpc min-sum decode (noisy LLRs)",
              u8_eq(dec, LDPC_DEC, k));
    }

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
