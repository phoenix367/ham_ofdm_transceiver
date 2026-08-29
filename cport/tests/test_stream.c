/* Streaming-architecture validation: the same captures as the
 * frame-at-once receiver, fed in arbitrary chunks through the ring-buffer
 * state machine. The tone stage is causal (windowed alignment/median,
 * peak-commit), so start/bits must match the reference exactly and the
 * CFO word within a small tolerance (the tone residual sees a slightly
 * different window); everything downstream is the same arithmetic. */
#include <stdio.h>
#include <string.h>

#include "../src/packets.h"
#include "../src/rom_modes.h"
#include "../src/tx.h"
#include "../src/rx_stream.h"
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

/* feed in chunks: pattern 0 = fixed 160; pattern 1 = LCG 1..1500 */
static int run_stream(link_mode_t mode, const int16_t *s, int n, int pattern,
                      rxs_event_t *ev)
{
    rxs_t *r = rxs_open(mode, 0);
    uint32_t lcg = 12345;
    int pos = 0, got = 0;
    while (pos < n) {
        int c;
        if (pattern == 0) {
            c = 160;
        } else {
            lcg = lcg * 1103515245u + 12345u;
            c = 1 + (int)((lcg >> 16) % 1500u);
        }
        if (c > n - pos)
            c = n - pos;
        got = rxs_push(r, s + pos, c, ev);
        pos += c;
        if (got && ev->type == 1)
            break; /* negative events are acquisition retries */
    }
    if (!(got && ev->type == 1))
        got = rxs_flush(r, ev);
    printf("  ring hwm %s: %lld samples\n",
           mode == 0 ? "NORMAL" : (mode == 1 ? "ROBUST" : "EXTREME"),
           (long long)rxs_ring_hwm(r));
    if (rxs_ring_hwm(r) > RXS_RAW_RING_LEN) {
        printf("  RING OVERRUN: hwm %lld > capacity %d\n",
               (long long)rxs_ring_hwm(r), RXS_RAW_RING_LEN);
        return 0; /* fail the case: reads wrapped into overwritten data */
    }
    return got;
}

static void report_cfo(const char *tag, int64_t got, int64_t want)
{
    if (got != want)
        printf("  %s: cfo %lld want %lld (delta %lld words)\n", tag,
               (long long)got, (long long)want, (long long)(got - want));
}

#define CFO_TOL ((int64_t)1 << 21) /* ~5.9 Hz -- tracker absorbs far more */

#define STREAM_CASE(TAG)                                                    \
    do {                                                                    \
        rxs_event_t ev;                                                     \
        int pkt_n = (int)sizeof(TX_##TAG##_PKT);                            \
        int n = tx_build_frame((link_mode_t)TX_##TAG##_MODE,                \
                               TX_##TAG##_PKT, pkt_n, PKT_TYP_DATA,         \
                               (mod_type_t)TX_##TAG##_MOD,                  \
                               (cc_rate_t)TX_##TAG##_SPD, g_samples + 700); \
        int p, ok_all = 1;                                                  \
        memset(g_samples, 0, 700 * sizeof(int16_t));                        \
        for (p = 0; p < 2; p++) {                                           \
            int got = run_stream((link_mode_t)TX_##TAG##_MODE, g_samples,   \
                                 700 + n, p, &ev);                          \
            int64_t dc = ev.cfo_word - RX_##TAG##_CFO_WORD;                 \
            int ok = got == 1 && ev.type == 1 &&                            \
                     ev.start_abs == RX_##TAG##_START &&                    \
                     (dc < 0 ? -dc : dc) <= CFO_TOL &&                      \
                     memcmp(ev.bits, TX_##TAG##_PKT, (size_t)pkt_n) == 0;   \
            if (!ok) {                                                      \
                printf("  " #TAG " p%d: got=%d type=%d start=%d want %d\n", \
                       p, got, ev.type, ev.start_abs,                       \
                       (int)RX_##TAG##_START);                              \
                report_cfo(#TAG, ev.cfo_word, RX_##TAG##_CFO_WORD);         \
                ok_all = 0;                                                 \
            }                                                               \
        }                                                                   \
        check("stream " #TAG " (2 chunk patterns)", ok_all);                \
    } while (0)

int main(void)
{
    STREAM_CASE(NORM_BPSK);
    STREAM_CASE(NORM_QAM16);
    STREAM_CASE(ROBUST_BPSK);
    STREAM_CASE(EXTREME_BPSK);

    {
        rxs_event_t ev;
        int n = (int)(sizeof(RX_NOISY_SAMPLES) / sizeof(int16_t));
        int got = run_stream(MODE_NORMAL, RX_NOISY_SAMPLES, n, 1, &ev);
        int64_t dc = ev.cfo_word - RX_NOISY_CFO_WORD;
        check("stream noisy (-5 dB, CFO, multipath)",
              got == 1 && ev.type == 1 && ev.start_abs == RX_NOISY_START &&
              (dc < 0 ? -dc : dc) <= CFO_TOL &&
              memcmp(ev.bits, RX_NOISY_PKT, sizeof(RX_NOISY_PKT)) == 0);
        report_cfo("noisy", ev.cfo_word, RX_NOISY_CFO_WORD);
    }

    {
        rxs_event_t ev;
        int pkt_n = (int)sizeof(TX_LDPC_PKT);
        int n = tx_build_frame_ex(MODE_NORMAL, TX_LDPC_PKT, pkt_n,
                                  PKT_TYP_DATA, MOD_BPSK, CC_R13, 1,
                                  g_samples + 700);
        int got;
        memset(g_samples, 0, 700 * sizeof(int16_t));
        got = run_stream(MODE_NORMAL, g_samples, 700 + n, 0, &ev);
        check("stream LDPC (ver=2)",
              got == 1 && ev.type == 1 && ev.hdr.ver == 2 &&
              ev.start_abs == RX_LDPC_START &&
              memcmp(ev.bits, TX_LDPC_PKT, (size_t)pkt_n) == 0);
        report_cfo("ldpc", ev.cfo_word, RX_LDPC_CFO_WORD);
    }

    /* streamed burst through the streaming receiver: one preamble, then
     * rxs_continue_burst() walks the remaining blocks */
    {
        rxs_t *r;
        rxs_event_t ev;
        int n = tx_build_burst((link_mode_t)TX_BURST_MODE, TX_BURST_BITS,
                               TX_BURST_PKT_BITS, TX_BURST_N, PKT_TYP_DATA,
                               (mod_type_t)TX_BURST_MOD,
                               (cc_rate_t)TX_BURST_SPD, TX_BURST_RESYNC,
                               g_samples + 700);
        int pos = 0, blocks = 0, exact = 1, total = 700 + n + 700;
        memset(g_samples, 0, 700 * sizeof(int16_t));
        memset(g_samples + 700 + n, 0, 700 * sizeof(int16_t));
        r = rxs_open((link_mode_t)TX_BURST_MODE, 0);
        while (pos < total && blocks < TX_BURST_N) {
            int c = 160;
            if (c > total - pos)
                c = total - pos;
            if (rxs_push(r, g_samples + pos, c, &ev)) {
                if (ev.type == 1) {
                    if (memcmp(ev.bits,
                               TX_BURST_BITS
                                   + (size_t)blocks * TX_BURST_PKT_BITS,
                               TX_BURST_PKT_BITS) != 0)
                        exact = 0;
                    blocks++;
                    /* the link layer would read its streamed marker here */
                    if (blocks < TX_BURST_N)
                        rxs_continue_burst(r, TX_BURST_RESYNC);
                }
            }
            pos += c;
        }
        check("stream burst: all blocks decoded in order",
              blocks == TX_BURST_N);
        check("stream burst: payload bits bit-exact", exact);
        printf("  burst: %d/%d blocks, %d samples\n", blocks, TX_BURST_N, n);
    }

    /* A frame whose tone field starts EXACTLY on a detection-block
     * boundary. No golden vector: the payload is known, so this is a
     * self-consistency check -- and it is the case the corpus never had.
     * Leads of 0 and 512 (multiples of NORMAL's 256-sample block) made
     * the coarse mask-shift search miss by one bin, the lag-N residual
     * wrapped, and every modulation failed at -94.07 Hz; lead 700 was
     * fine. The frame-at-once detector resolves this with a second,
     * lag-N/2 correlation; the streaming commit never had it until the
     * analog loopback stand found the gap. */
    {
        static int16_t sig[24000];
        uint8_t pay[27], pkt[280];
        rxs_event_t ev;
        int pkt_n, total = 0, got, pos, i, leads[2] = { 512, 0 }, li;
        for (i = 0; i < 27; i++)
            pay[i] = (uint8_t)(0x41 + (i % 26));
        pkt_n = data_encode(7, pay, 27, pkt);
        for (li = 0; li < 2; li++) {
            txs_t *t;
            rxs_t *r;
            int lead = leads[li], n;
            char name[80];
            memset(sig, 0, sizeof(sig));
            t = txs_open(MODE_NORMAL, pkt, pkt_n, 1, PKT_TYP_DATA, MOD_QPSK,
                         CC_R12, 0, 0, &total);
            pos = lead;
            while (t && (got = txs_pull(t, sig + pos, 24000 - pos)) > 0)
                pos += got;
            n = pos + 1536;
            r = rxs_open(MODE_NORMAL, 0);
            got = 0;
            for (pos = 0; pos < n && !got; pos += 512) {
                int c = n - pos < 512 ? n - pos : 512;
                got = rxs_push(r, sig + pos, c, &ev);
            }
            if (!got)
                got = rxs_flush(r, &ev);
            snprintf(name, sizeof(name),
                     "stream NORMAL QPSK, tone field block-aligned (lead %d)",
                     lead);
            check(name, got && ev.type == 1 && ev.pkt_bits_n == pkt_n
                        && memcmp(ev.bits, pkt, (size_t)pkt_n) == 0
                        && ev.start_abs == lead + PREAMBLE_LEN_NORMAL + 30);
            if (got)
                printf("  lead %d: start %d cfo %+.2f Hz\n", lead, ev.start_abs,
                       (double)ev.cfo_word * 12000.0 / 4294967296.0);
        }
    }

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
