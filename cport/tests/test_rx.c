/* Genie-synced RX demod tests: frames rebuilt by the (bit-exact) C TX,
 * demodulated at the detection genie (start, cfo_word) dumped from the
 * Python model; decoded packet bits must match exactly. */
#include <stdio.h>
#include <string.h>

#include "../src/packets.h"
#include "../src/tx.h"
#include "../src/rx_demod.h"
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

static int16_t g_samples[600000];
static uint8_t g_bits[2048];

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

static uint64_t fnv64_llr(const int64_t *v, int n)
{
    uint64_t h = UINT64_C(14695981039346656037);
    int i;
    for (i = 0; i < n; i++) {
        h ^= (uint64_t)v[i];
        h *= UINT64_C(1099511628211);
    }
    return h;
}

#define RX_CASE(TAG)                                                        \
    do {                                                                    \
        rxd_t r;                                                            \
        rxd_header_t h;                                                     \
        int pkt_n = (int)sizeof(TX_##TAG##_PKT);                            \
        int n = tx_build_frame((link_mode_t)TX_##TAG##_MODE,                \
                               TX_##TAG##_PKT, pkt_n, PKT_TYP_DATA,         \
                               (mod_type_t)TX_##TAG##_MOD,                  \
                               (cc_rate_t)TX_##TAG##_SPD, g_samples + 700); \
        int rc, start = -1;                                                 \
        int64_t cfo = 0;                                                    \
        memset(g_samples, 0, 700 * sizeof(int16_t));                        \
        rxd_init(&r, (link_mode_t)TX_##TAG##_MODE);                         \
        rc = rxd_receive_genie(&r, g_samples, 700 + n, RX_##TAG##_START,    \
                               RX_##TAG##_CFO_WORD, &h, g_bits);            \
        check("rx genie " #TAG,                                             \
              rc == 0 && h.len == pkt_n &&                                  \
              h.mod == TX_##TAG##_MOD && h.spd == TX_##TAG##_SPD &&         \
              memcmp(g_bits, TX_##TAG##_PKT, (size_t)pkt_n) == 0);          \
        if (rc != 0)                                                        \
            printf("  " #TAG ": rc=%d\n", rc);                              \
        rc = rxd_receive(&r, g_samples, 700 + n, &h, g_bits, &start, &cfo); \
        check("rx detect+decode " #TAG,                                     \
              rc == 0 && start == RX_##TAG##_START &&                       \
              cfo == RX_##TAG##_CFO_WORD &&                                 \
              memcmp(g_bits, TX_##TAG##_PKT, (size_t)pkt_n) == 0);          \
        if (rc != 0 || start != RX_##TAG##_START ||                         \
            cfo != RX_##TAG##_CFO_WORD)                                     \
            printf("  " #TAG ": rc=%d start=%d want %d cfo=%lld want %lld\n",\
                   rc, start, (int)RX_##TAG##_START, (long long)cfo,        \
                   (long long)RX_##TAG##_CFO_WORD);                         \
    } while (0)

int main(void)
{
    RX_CASE(NORM_BPSK);
    RX_CASE(NORM_QPSK);
    RX_CASE(NORM_QAM16);
    RX_CASE(ROBUST_BPSK);
    RX_CASE(EXTREME_BPSK);

    /* streamed burst: detect + header once, then N blocks at deterministic
     * offsets with ZC resyncs in between */
    {
        rxd_t r;
        rxd_header_t h;
        int ok[TX_BURST_N], start = -1, n_resync = -1, rc, k, all = 1;
        int64_t cfo = 0;
        int n = tx_build_burst((link_mode_t)TX_BURST_MODE, TX_BURST_BITS,
                               TX_BURST_PKT_BITS, TX_BURST_N, PKT_TYP_DATA,
                               (mod_type_t)TX_BURST_MOD,
                               (cc_rate_t)TX_BURST_SPD, TX_BURST_RESYNC,
                               g_samples + 700);
        memset(g_samples, 0, 700 * sizeof(int16_t));
        rxd_init(&r, (link_mode_t)TX_BURST_MODE);
        rc = rxd_receive_burst(&r, g_samples, 700 + n, TX_BURST_N,
                               TX_BURST_RESYNC, &h, g_bits, ok, &start, &cfo,
                               &n_resync);
        for (k = 0; k < TX_BURST_N; k++)
            if (!ok[k])
                all = 0;
        check("rx burst: every block delivered",
              rc == TX_BURST_N && all && h.mod == TX_BURST_MOD &&
              h.spd == TX_BURST_SPD);
        check("rx burst: payload bits bit-exact",
              memcmp(g_bits, TX_BURST_BITS,
                     (size_t)(TX_BURST_N * TX_BURST_PKT_BITS)) == 0);
        check("rx burst: every ZC resync locked",
              n_resync == (TX_BURST_N - 1) / TX_BURST_RESYNC);
        if (rc != TX_BURST_N)
            printf("  burst: rc=%d start=%d resyncs=%d\n", rc, start,
                   n_resync);

        /* a truncated recording must lose only the tail blocks */
        rc = rxd_receive_burst(&r, g_samples, 700 + n / 2, TX_BURST_N,
                               TX_BURST_RESYNC, &h, g_bits, ok, &start, &cfo,
                               &n_resync);
        check("rx burst: truncated tail reported as misses",
              rc > 0 && rc < TX_BURST_N && ok[0] && !ok[TX_BURST_N - 1]);
    }

    {
        rxd_t r;
        rxd_header_t h;
        int n = (int)(sizeof(RX_NOISY_SAMPLES) / sizeof(int16_t));
        int rc, start = -1;
        int64_t cfo = 0;
        rxd_init(&r, MODE_NORMAL);
        rc = rxd_receive_genie(&r, RX_NOISY_SAMPLES, n, RX_NOISY_START,
                               RX_NOISY_CFO_WORD, &h, g_bits);
        check("rx genie noisy (-5 dB, CFO, multipath)",
              rc == 0 && h.len == (int)sizeof(RX_NOISY_PKT) &&
              memcmp(g_bits, RX_NOISY_PKT, sizeof(RX_NOISY_PKT)) == 0);
        if (rc != 0)
            printf("  noisy: rc=%d\n", rc);
        rc = rxd_receive(&r, RX_NOISY_SAMPLES, n, &h, g_bits, &start, &cfo);
        check("rx detect+decode noisy",
              rc == 0 && start == RX_NOISY_START &&
              cfo == RX_NOISY_CFO_WORD &&
              memcmp(g_bits, RX_NOISY_PKT, sizeof(RX_NOISY_PKT)) == 0);
        if (rc != 0 || start != RX_NOISY_START || cfo != RX_NOISY_CFO_WORD)
            printf("  noisy full: rc=%d start=%d want %d cfo=%lld want %lld\n",
                   rc, start, (int)RX_NOISY_START, (long long)cfo,
                   (long long)RX_NOISY_CFO_WORD);
    }

    /* LDPC (ver=2) frame: C TX hash + full C reception */
    {
        rxd_t r;
        rxd_header_t h;
        int pkt_n = (int)sizeof(TX_LDPC_PKT);
        int n = tx_build_frame_ex(MODE_NORMAL, TX_LDPC_PKT, pkt_n,
                                  PKT_TYP_DATA, MOD_BPSK, CC_R13, 1,
                                  g_samples + 700);
        int rc, start = -1;
        int64_t cfo = 0;
        check("tx frame LDPC (ver=2)",
              n == TX_LDPC_LEN &&
              memcmp(g_samples + 700, TX_LDPC_HEAD,
                     sizeof(TX_LDPC_HEAD)) == 0 &&
              fnv64(g_samples + 700, n) == TX_LDPC_HASH);
        memset(g_samples, 0, 700 * sizeof(int16_t));
        rxd_init(&r, MODE_NORMAL);
        rc = rxd_receive(&r, g_samples, 700 + n, &h, g_bits, &start, &cfo);
        check("rx detect+decode LDPC",
              rc == 0 && h.ver == 2 && start == RX_LDPC_START &&
              cfo == RX_LDPC_CFO_WORD &&
              memcmp(g_bits, TX_LDPC_PKT, (size_t)pkt_n) == 0);
        if (rc != 0)
            printf("  ldpc: rc=%d\n", rc);
    }

    /* HARQ chase combining on a calibrated RX: complementary erasures */
    {
        static int64_t llrs[1024];
        rxd_t r;
        rxd_header_t h;
        int pkt_n = (int)sizeof(HARQ_PKT);
        int n = tx_build_frame(MODE_NORMAL, HARQ_PKT, pkt_n, PKT_TYP_DATA,
                               MOD_BPSK, CC_R13, g_samples + 700);
        int rc, llr_n = 0, i;
        static int16_t sa[600000], sb[600000];

        memset(g_samples, 0, 700 * sizeof(int16_t));
        memcpy(sa, g_samples, sizeof(int16_t) * (size_t)(700 + n));
        memcpy(sb, g_samples, sizeof(int16_t) * (size_t)(700 + n));
        for (i = HARQ_EA0; i < HARQ_EA1; i++)
            sa[i] = 0;
        for (i = HARQ_EB0; i < HARQ_EB1; i++)
            sb[i] = 0;

        rxd_init(&r, MODE_NORMAL);
        r.calibrate = 1;
        rc = rxd_receive_genie_harq(&r, sa, 700 + n, HARQ_A_START,
                                    HARQ_A_CFO_WORD, &h, g_bits,
                                    0, 0, llrs, &llr_n);
        check("harq attempt A fails, calibrated LLRs exported",
              rc == -3 && llr_n == HARQ_LLR_N &&
              fnv64_llr(llrs, llr_n) == HARQ_LLR_HASH);
        rc = rxd_receive_genie_harq(&r, sb, 700 + n, HARQ_B_START,
                                    HARQ_B_CFO_WORD, &h, g_bits,
                                    llrs, llr_n, 0, 0);
        check("harq attempt B + stored combines, CRC-gated",
              rc == 0 && r.last_harq_combined == 1 &&
              memcmp(g_bits, HARQ_PKT, (size_t)pkt_n) == 0);
        if (rc != 0)
            printf("  harq B: rc=%d\n", rc);
    }

    /* calibrated-LLR mode + integer SNR estimate on the noisy frame */
    {
        static int64_t llrs[1024];
        rxd_t r;
        rxd_header_t h;
        int nsm = (int)(sizeof(RX_NOISY_SAMPLES) / sizeof(int16_t));
        int rc, llr_n = 0;
        double d;
        rxd_init(&r, MODE_NORMAL);
        r.calibrate = 1;
        rc = rxd_receive_genie_harq(&r, RX_NOISY_SAMPLES, nsm,
                                    RX_NOISY_START, RX_NOISY_CFO_WORD,
                                    &h, g_bits, 0, 0, llrs, &llr_n);
        check("calibrated decode (alpha fit + reliability ROM)",
              rc == 0 && llr_n == CAL_LLR_N &&
              fnv64_llr(llrs, llr_n) == CAL_LLR_HASH &&
              memcmp(g_bits, RX_NOISY_PKT, sizeof(RX_NOISY_PKT)) == 0);
        d = r.last_snr_db - CAL_SNR_DB;
        check("integer SNR estimate matches",
              (d < 0 ? -d : d) < 1e-9);
        if (rc != 0)
            printf("  cal: rc=%d snr=%f want %f\n", rc, r.last_snr_db,
                   (double)CAL_SNR_DB);
    }

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
