/* Golden tests for packets.c + tx.c: full frames must match the Python
 * fixed transmitter bit-exactly (FNV-1a 64 hash over all int16 samples,
 * plus the first 64 samples for debuggability). */
#include <stdio.h>
#include <string.h>

#include "../src/packets.h"
#include "../src/tx.h"
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

static uint64_t fnv64(const int16_t *s, int n)
{
    uint64_t h = UINT64_C(14695981039346656037);
    int i;
    for (i = 0; i < n; i++) {
        h ^= (uint16_t)s[i];
        h *= UINT64_C(1099511628211);
    }
    return h;
}

static int16_t g_frame[600000];

#define TX_CASE(TAG)                                                        \
    do {                                                                    \
        int pkt_n = (int)sizeof(TX_##TAG##_PKT);                            \
        int n = tx_build_frame((link_mode_t)TX_##TAG##_MODE,                \
                               TX_##TAG##_PKT, pkt_n, PKT_TYP_DATA,         \
                               (mod_type_t)TX_##TAG##_MOD,                  \
                               (cc_rate_t)TX_##TAG##_SPD, g_frame);         \
        int head_ok = n >= 64 &&                                            \
            memcmp(g_frame, TX_##TAG##_HEAD, sizeof(TX_##TAG##_HEAD)) == 0; \
        if (!head_ok && n >= 64) {                                          \
            int i;                                                          \
            for (i = 0; i < 64; i++)                                        \
                if (g_frame[i] != TX_##TAG##_HEAD[i]) {                     \
                    printf("  " #TAG " head[%d]: got %d want %d\n", i,      \
                           g_frame[i], TX_##TAG##_HEAD[i]);                 \
                    break;                                                  \
                }                                                           \
        }                                                                   \
        check("tx frame " #TAG,                                             \
              n == TX_##TAG##_LEN && head_ok &&                             \
              fnv64(g_frame, n) == TX_##TAG##_HASH);                        \
    } while (0)

int main(void)
{
    {
        uint8_t hdr[HEADER_BITS];
        header_encode(1, PKT_TYP_DATA, 1, 2, 180, hdr);
        check("header encode",
              memcmp(hdr, HDR_BITS_WANT, HEADER_BITS) == 0);
    }
    {
        uint8_t bits[512];
        int n = data_encode(0x5A5A5, DATA_PAYLOAD,
                            (int)sizeof(DATA_PAYLOAD), bits);
        check("data packet encode",
              n == (int)sizeof(DATA_BITS_WANT) &&
              memcmp(bits, DATA_BITS_WANT, (size_t)n) == 0);
    }

    TX_CASE(NORM_BPSK);
    TX_CASE(NORM_QPSK);
    TX_CASE(NORM_QAM16);
    TX_CASE(ROBUST_BPSK);
    TX_CASE(EXTREME_BPSK);

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
