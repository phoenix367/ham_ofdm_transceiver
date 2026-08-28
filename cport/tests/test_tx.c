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

/* fnv64 as a running state, so a streamed waveform can be checked
 * against the golden hash without ever holding it whole */
static uint64_t fnv64_upd(uint64_t h, const int16_t *s, int n)
{
    int i;
    for (i = 0; i < n; i++) {
        h ^= (uint16_t)s[i];
        h *= UINT64_C(1099511628211);
    }
    return h;
}

/* Stream one frame/burst through txs_pull in small chunks, checking each
 * chunk against the frame-at-once output as it arrives and folding it
 * into a running hash. chunk deliberately does not divide a symbol, so
 * the generator is exercised across every internal boundary. */
static int stream_matches(link_mode_t mode, const uint8_t *blocks,
                          int pkt_bits_n, int n_blocks, int typ,
                          mod_type_t mod, cc_rate_t spd, int resync_every,
                          const int16_t *ref, int ref_n, uint64_t want_hash,
                          int chunk)
{
    static int16_t buf[4096];
    uint64_t h = UINT64_C(14695981039346656037);
    int total = 0, got, pos = 0;
    txs_t *t = txs_open(mode, blocks, pkt_bits_n, n_blocks, typ, mod, spd,
                        resync_every, 0, &total);

    if (!t || total != ref_n)
        return 0;
    if (chunk > (int)(sizeof(buf) / sizeof(buf[0])))
        chunk = (int)(sizeof(buf) / sizeof(buf[0]));
    while ((got = txs_pull(t, buf, chunk)) > 0) {
        if (pos + got > ref_n || memcmp(buf, ref + pos, (size_t)got * 2) != 0)
            return 0;
        h = fnv64_upd(h, buf, got);
        pos += got;
    }
    return pos == ref_n && h == want_hash;
}

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

    /* streamed burst: must match the fixed model sample for sample */
    {
        int want = tx_burst_len((link_mode_t)TX_BURST_MODE, TX_BURST_PKT_BITS,
                                (mod_type_t)TX_BURST_MOD,
                                (cc_rate_t)TX_BURST_SPD, TX_BURST_N,
                                TX_BURST_RESYNC);
        int n = tx_build_burst((link_mode_t)TX_BURST_MODE, TX_BURST_BITS,
                               TX_BURST_PKT_BITS, TX_BURST_N, PKT_TYP_DATA,
                               (mod_type_t)TX_BURST_MOD,
                               (cc_rate_t)TX_BURST_SPD, TX_BURST_RESYNC,
                               g_frame);
        int head_ok = n >= 64 && memcmp(g_frame, TX_BURST_HEAD,
                                        sizeof(TX_BURST_HEAD)) == 0;
        check("tx burst length prediction", want == TX_BURST_LEN);
        check("tx burst BURST bit-exact",
              n == TX_BURST_LEN && head_ok &&
              fnv64(g_frame, n) == TX_BURST_HASH);
    }
    /* a one-block burst with no resync IS a frame -- same samples */
    {
        static int16_t frame[600000];
        int a = tx_build_burst((link_mode_t)TX_BURST_MODE, TX_BURST_BITS,
                               TX_BURST_PKT_BITS, 1, PKT_TYP_DATA,
                               (mod_type_t)TX_BURST_MOD,
                               (cc_rate_t)TX_BURST_SPD, 0, g_frame);
        int b = tx_build_frame((link_mode_t)TX_BURST_MODE, TX_BURST_BITS,
                               TX_BURST_PKT_BITS, PKT_TYP_DATA,
                               (mod_type_t)TX_BURST_MOD,
                               (cc_rate_t)TX_BURST_SPD, frame);
        check("1-block burst == frame",
              a == b && a > 0 && memcmp(g_frame, frame,
                                        sizeof(int16_t) * (size_t)a) == 0);
    }

    /* the streaming transmitter must be bit-identical to the
     * frame-at-once path, checked chunk by chunk as it is pulled */
    {
        int n = tx_build_frame((link_mode_t)TX_NORM_QPSK_MODE,
                               TX_NORM_QPSK_PKT,
                               (int)sizeof(TX_NORM_QPSK_PKT), PKT_TYP_DATA,
                               (mod_type_t)TX_NORM_QPSK_MOD,
                               (cc_rate_t)TX_NORM_QPSK_SPD, g_frame);
        check("txs frame NORM_QPSK bit-exact (137-sample chunks)",
              stream_matches((link_mode_t)TX_NORM_QPSK_MODE,
                             TX_NORM_QPSK_PKT,
                             (int)sizeof(TX_NORM_QPSK_PKT), 1, PKT_TYP_DATA,
                             (mod_type_t)TX_NORM_QPSK_MOD,
                             (cc_rate_t)TX_NORM_QPSK_SPD, 0,
                             g_frame, n, TX_NORM_QPSK_HASH, 137));
    }
    {
        int n = tx_build_frame((link_mode_t)TX_EXTREME_BPSK_MODE,
                               TX_EXTREME_BPSK_PKT,
                               (int)sizeof(TX_EXTREME_BPSK_PKT), PKT_TYP_DATA,
                               (mod_type_t)TX_EXTREME_BPSK_MOD,
                               (cc_rate_t)TX_EXTREME_BPSK_SPD, g_frame);
        check("txs frame EXTREME_BPSK bit-exact (1-sample chunks)",
              stream_matches((link_mode_t)TX_EXTREME_BPSK_MODE,
                             TX_EXTREME_BPSK_PKT,
                             (int)sizeof(TX_EXTREME_BPSK_PKT), 1,
                             PKT_TYP_DATA,
                             (mod_type_t)TX_EXTREME_BPSK_MOD,
                             (cc_rate_t)TX_EXTREME_BPSK_SPD, 0,
                             g_frame, n, TX_EXTREME_BPSK_HASH, 1));
    }
    {
        int n = tx_build_burst((link_mode_t)TX_BURST_MODE, TX_BURST_BITS,
                               TX_BURST_PKT_BITS, TX_BURST_N, PKT_TYP_DATA,
                               (mod_type_t)TX_BURST_MOD,
                               (cc_rate_t)TX_BURST_SPD, TX_BURST_RESYNC,
                               g_frame);
        check("txs burst bit-exact (resync ZC crossed mid-chunk)",
              stream_matches((link_mode_t)TX_BURST_MODE, TX_BURST_BITS,
                             TX_BURST_PKT_BITS, TX_BURST_N, PKT_TYP_DATA,
                             (mod_type_t)TX_BURST_MOD,
                             (cc_rate_t)TX_BURST_SPD, TX_BURST_RESYNC,
                             g_frame, n, TX_BURST_HASH, 999));
    }

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
