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
#include "../src/rom_tables.h"
#include "../src/broadcast.h"
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

    /* A stream of small broadcast groups must not lose one to its own
     * commit rule. This is bench/bc_soak.c's failing seed, verbatim: ten
     * 2-frame BCAST groups at rung 12 (QAM16 3/4), LCG-derived payloads,
     * chunk sizes and inter-group gaps, which places group 9's tone
     * field in the ~8-sample alignment window (mod 256) where the field
     * ENTERING the detection window scores one isolated above-threshold
     * evaluation (~10x thr^2 against ~1e5x aligned). Committing that
     * spike anchors two blocks early, and the failed ZC/header attempts
     * then consume the true peak, losing the group whole -- 0.76% of
     * all groups of the live voice campaign, byte-located in the
     * rx_*.bin dumps. WEAK_COMMIT_X gates the region-end commit so the
     * weak argmax waits instead, and the true peak displaces it; a
     * build with -DWEAK_COMMIT_X=1 (gate off) loses frames 18 and 19
     * here. The multi-group preamble history is load-bearing: the same
     * geometry as an isolated pair decodes fine, so do not "simplify"
     * this to two groups. */
    {
        rxs_event_t ev;
        uint32_t st = 20264;
        int g2, seq = 0, frames = 0, left = 0, pos, i, n, lead;
        #define SOAK_RND() ((st = st * 1664525u + 1013904223u) >> 8)
        lead = 300 + (int)(SOAK_RND() % 512u);
        memset(g_samples, 0, sizeof(g_samples));
        pos = lead;
        for (g2 = 0; g2 < 10; g2++) {
            uint8_t payload[26], blocks[2 * (36 + 8 * 26)];
            int chunk = 30 + (int)(SOAK_RND() % 2u) * 5, pkt_n = 0, nf;
            txs_t *t;
            int total = 0, got2;
            for (nf = 0; nf < 2; nf++) {
                int first = (nf == 0), take = first ? 23 : chunk - 23;
                int flags = first ? BC_SYNC : 0;
                if (!first && g2 == 9)
                    flags |= BC_EOS;
                memset(payload, 0, sizeof(payload));
                payload[0] = (uint8_t)(flags | ((seq + nf) & BC_SEQ_MASK));
                payload[1] = (uint8_t)take;
                if (first)
                    payload[2] = (uint8_t)((1 << 4) | 0x0F);
                for (i = 0; i < take; i++)
                    payload[(first ? 3 : 2) + i] =
                        (uint8_t)(SOAK_RND() & 0xFFu);
                pkt_n = data_encode(0, payload, 26,
                                    blocks + (size_t)nf * (36 + 8 * 26));
            }
            memmove(blocks + (size_t)pkt_n, blocks + (36 + 8 * 26),
                    (size_t)pkt_n);
            t = txs_open(MODE_NORMAL, blocks, pkt_n, 2, PKT_TYP_BCAST,
                         MOD_QAM16, CC_R34, 4, 0, &total);
            n = 0;
            while (t && (got2 = txs_pull(t, g_samples + pos + n,
                                         (int)(sizeof(g_samples) / 2)
                                             - pos - n - 20000)) > 0)
                n += got2;
            seq += 2;
            pos += n + 1000 + (int)(SOAK_RND() % 15000u);
        }
        n = pos + 2000;
        {
            rxs_t *r = rxs_open(MODE_NORMAL, 0);
            for (pos = 0; pos + 256 <= n; pos += 256) {
                if (!rxs_push(r, g_samples + pos, 256, &ev))
                    continue;
                if (ev.type == 1 && ev.hdr.typ == PKT_TYP_BCAST) {
                    int fl = 0;
                    for (i = 0; i < 8; i++)
                        fl = (fl << 1) | (ev.bits[20 + i] & 1);
                    frames++;
                    left = (fl & BC_SYNC) ? 1 : (left > 0 ? left - 1 : 0);
                    if (fl & BC_EOS)
                        left = 0;
                    if (left > 0 && !rxs_continue_burst(r, 4))
                        left = 0;
                }
            }
        }
        check("stream BCAST groups: weak-spike alignment loses nothing "
              "(soak seed 20264)", frames == 20);
        printf("  %d of 20 frames decoded\n", frames);
        #undef SOAK_RND
    }

    /* PREEMPTION (rx_stream.c): a weak stranger's frame whose header
     * decodes used to own the receiver for its whole data block, and a
     * stronger frame starting inside that block was lost -- its preamble
     * consumed by the decode in progress and never revisited. The
     * challenger tracker now abandons the decode for a later peak at
     * RXS_PREEMPT_LOG2_Q4 (3 dB) more tone-bin energy than the locked
     * one. Two NORMAL frames 20 dB apart under noise ~26 dB below the
     * strong one, so the weak one sits at ~+6 dB SNR -- above the 64x
     * strong-commit gate (a 0 dB NORMAL preamble scores ~1.6e8 against
     * the 1.7e8 gate and only commits by stability, too late to capture
     * anything) and with a header that decodes -- the second starting
     * 8000 samples into the first's data block:
     *   (a) stranger first: the strong frame must decode, 1 preempt;
     *   (b) strong first: the frame in hand must NOT be abandoned for a
     *       stranger 20 dB down -- 0 preempts, the strong frame decodes.
     * Noise is what makes the metrics differ: on a noise-free wire both
     * tone contrasts saturate at the regularizer and no ratio exists. */
    {
        static int16_t weak[60000], strong[60000];
        uint8_t pw[512], ps[512];   /* 27-byte packets: 252 bit-bytes */
        int nw, ns, pw_n, ps_n, k, which;
        uint32_t lcg = 987654321u;
        pw_n = data_encode(11, (const uint8_t *)"STRANGER 27-BYTE PAYLOAD XX",
                           27, pw);
        ps_n = data_encode(22, (const uint8_t *)"WANTED   27-BYTE PAYLOAD YY",
                           27, ps);
        nw = tx_build_frame(MODE_NORMAL, pw, pw_n, PKT_TYP_DATA, MOD_BPSK,
                            CC_R12, weak);
        ns = tx_build_frame(MODE_NORMAL, ps, ps_n, PKT_TYP_DATA, MOD_BPSK,
                            CC_R12, strong);
        for (which = 0; which < 2; which++) {
            int lead = 700, off2 = lead + 8000;
            int n = off2 + (which == 0 ? ns : nw) + 1536;
            int decoded = 0, other = 0, pos, got;
            rxs_event_t ev;
            rxs_t *r;
            for (k = 0; k < n; k++) {   /* uniform noise, rms ~700 */
                lcg = lcg * 1103515245u + 12345u;
                g_samples[k] = (int16_t)((int)((lcg >> 16) % 2400u) - 1200);
            }
            for (k = 0; k < (which == 0 ? nw : ns); k++)
                g_samples[lead + k] += which == 0 ? (int16_t)(weak[k] / 10)
                                                  : strong[k];
            for (k = 0; k < (which == 0 ? ns : nw); k++)
                g_samples[off2 + k] += which == 0 ? strong[k]
                                                  : (int16_t)(weak[k] / 10);
            r = rxs_open(MODE_NORMAL, 0);
            for (pos = 0; pos < n; pos += 256) {
                int c = n - pos < 256 ? n - pos : 256;
                got = rxs_push(r, g_samples + pos, c, &ev);
                if (got && ev.type == 1) {
                    if (ev.pkt_bits_n == ps_n
                        && memcmp(ev.bits, ps, (size_t)ps_n) == 0)
                        decoded++;
                    else
                        other++;
                }
            }
            if (rxs_flush(r, &ev) && ev.type == 1) {
                if (ev.pkt_bits_n == ps_n
                    && memcmp(ev.bits, ps, (size_t)ps_n) == 0)
                    decoded++;
                else
                    other++;
            }
            printf("  preempt case %d: strong %d other %d preempts %ld\n",
                   which, decoded, other, (long)rxs_preempts(r));
            if (which == 0)
                check("preempt: strong frame inside a weak stranger's data "
                      "block decodes (1 preempt)",
                      decoded == 1 && rxs_preempts(r) == 1);
            else
                check("preempt: a weak stranger inside our data block does "
                      "not preempt (0 preempts)",
                      decoded == 1 && rxs_preempts(r) == 0);
        }
    }

    /* STATIONARY-CARRIER EXCISION (rx_stream.c carrier_track + the global
     * notch bank): a CW carrier from the NCO ROM, amplitude 12000 (~-4 dB
     * against the frame), keyed 1.5 s before a NORMAL frame and through
     * it, then 2 s of silence. The rolling finder needs one EXTREME tone
     * field (61440 samples, 5.1 s) of history whatever the mode, so the
     * carrier is notched before the frame arrives; the frame must decode,
     * the bank must hold the notch while the carrier is on and release it
     * in the silence. */
    {
        uint8_t pkt[512];
        int pkt_n = data_encode(33, (const uint8_t *)"CARRIER  27-BYTE PAYLOAD ZZ",
                                27, pkt);
        int lead = 72000, n_fr, n, k, pos, got, decoded = 0, held = 0;
        uint32_t ph = 0, word = 537944654u, lcg = 4242u;   /* ~1503 Hz */
        rxs_event_t ev;
        rxs_t *r;
        n_fr = tx_build_frame(MODE_NORMAL, pkt, pkt_n, PKT_TYP_DATA, MOD_BPSK,
                              CC_R12, g_samples + lead);
        n = lead + n_fr + 24000;
        memset(g_samples, 0, (size_t)lead * sizeof(int16_t));
        memset(g_samples + lead + n_fr, 0, 24000 * sizeof(int16_t));
        for (k = 0; k < n; k++) {   /* a small noise floor (rms ~37): the
                                     * ROM tone's phase-truncation spurs at
                                     * +-288 Hz are -60 dBc and, on a truly
                                     * silent channel, become the next
                                     * stationary lines once the carrier is
                                     * notched -- legitimate, but not what
                                     * this case is about */
            int32_t v = g_samples[k];
            lcg = lcg * 1103515245u + 12345u;
            v += (int32_t)((lcg >> 16) % 128u) - 64;
            if (k < lead + n_fr) {
                v += (int32_t)(((int64_t)12000 * NCO_COS[ph >> 20]) >> 15);
                ph += word;
            }
            if (v > 32767) v = 32767;
            if (v < -32768) v = -32768;
            g_samples[k] = (int16_t)v;
        }
        r = rxs_open(MODE_NORMAL, 0);
        for (pos = 0; pos < n; pos += 256) {
            int c = n - pos < 256 ? n - pos : 256;
            got = rxs_push(r, g_samples + pos, c, &ev);
            if (pos <= lead && pos + c > lead) {
                held = rxs_notches();   /* engaged before the frame */
#ifdef STREAM_DEBUG
                { void rxs_dump_bank(void); rxs_dump_bank(); }
#endif
            }
            if (got && ev.type == 1 && ev.pkt_bits_n == pkt_n
                && memcmp(ev.bits, pkt, (size_t)pkt_n) == 0)
                decoded++;
        }
        printf("  carrier: notches at frame start %d, decoded %d, notches at end %d\n",
               held, decoded, rxs_notches());
#ifdef STREAM_DEBUG
        { void rxs_dump_bank(void); rxs_dump_bank(); }
#endif
        check("carrier: notched before the frame, frame decodes, released after",
              held >= 1 && decoded == 1 && rxs_notches() == 0);
    }

    /* The excision bank is global and the ring is shared: a NORMAL
     * instance listening beside an EXTREME one must NOT read the peer's
     * EXTREME tone comb (3.4 s of the same four tones) as a stationary
     * carrier and notch it out from under the EXTREME instance. Both
     * open, an EXTREME frame under a small noise floor: the EXTREME
     * instance decodes, the bank stays empty. */
    {
        uint8_t pkt[512];
        int pkt_n = data_encode(44, (const uint8_t *)"EXTREME  27-BYTE PAYLOAD QQ",
                                27, pkt);
        int lead = 3000, n_fr, n, k, pos, decoded = 0, max_notch = 0;
        uint32_t lcg = 777u;
        rxs_event_t ev;
        rxs_t *rn, *rx;
        n_fr = tx_build_frame(MODE_EXTREME, pkt, pkt_n, PKT_TYP_DATA, MOD_BPSK,
                              CC_R13, g_samples + lead);
        n = lead + n_fr + 12000;
        memset(g_samples, 0, (size_t)lead * sizeof(int16_t));
        memset(g_samples + lead + n_fr, 0, 12000 * sizeof(int16_t));
        for (k = 0; k < n; k++) {
            int32_t v = g_samples[k];
            lcg = lcg * 1103515245u + 12345u;
            v += (int32_t)((lcg >> 16) % 128u) - 64;
            if (v > 32767) v = 32767;
            if (v < -32768) v = -32768;
            g_samples[k] = (int16_t)v;
        }
        rn = rxs_open(MODE_NORMAL, 0);
        rx = rxs_open(MODE_EXTREME, 0);
        for (pos = 0; pos < n; pos += 512) {
            int c = n - pos < 512 ? n - pos : 512;
            (void)rxs_push(rn, g_samples + pos, c, &ev);   /* the bystander */
            if (rxs_push(rx, g_samples + pos, c, &ev) && ev.type == 1
                && ev.pkt_bits_n == pkt_n && memcmp(ev.bits, pkt, (size_t)pkt_n) == 0)
                decoded++;
            if (rxs_notches() > max_notch)
                max_notch = rxs_notches();
        }
        if (rxs_flush(rx, &ev) && ev.type == 1 && ev.pkt_bits_n == pkt_n
            && memcmp(ev.bits, pkt, (size_t)pkt_n) == 0)
            decoded++;
        printf("  shared ring: EXTREME decoded %d, max notches %d\n", decoded, max_notch);
        check("shared ring: a NORMAL instance never notches the EXTREME peer's comb",
              decoded == 1 && max_notch == 0);
    }

    /* EXTREME under a CW carrier 10 dB above the frame, frame at -14 dB
     * SNR, the carrier on from the start: the notch engages 5.1 s in,
     * the frame's tone field begins 5.5 s in. Pins the reset of the
     * search's pending region when a notch engages (rx_stream.c
     * carrier_track): without it the raw carrier's above-threshold
     * region commits by stability as the frame arrives and the frame,
     * 10 dB weaker, cannot preempt the false anchor -- measured 35-63 %
     * PER from ISR 0 dB up at EXTREME with the notch itself on
     * frequency to 0.05 Hz. */
    {
        uint8_t pkt[512];
        int pkt_n = data_encode(55, (const uint8_t *)"CW+EXTREME 27-BYTE PAYLOAD.",
                                27, pkt);
        int lead = 66000, n_fr, n, k, pos, decoded = 0;   /* 523984-sample frame: 598 k fits g_samples */
        uint32_t ph = 0, word = 186030711u, lcg = 31337u;   /* ~519.7 Hz */
        rxs_event_t ev;
        rxs_t *r;
        n_fr = tx_build_frame(MODE_EXTREME, pkt, pkt_n, PKT_TYP_DATA, MOD_BPSK,
                              CC_R13, g_samples + lead);
        n = lead + n_fr + 8000;
        memset(g_samples, 0, (size_t)lead * sizeof(int16_t));
        memset(g_samples + lead + n_fr, 0, 8000 * sizeof(int16_t));
        for (k = 0; k < n; k++) {
            /* frame /8 (rms ~1750), uniform noise +-15000 (rms 8660,
             * -14 dB), carrier amplitude 7800 (rms 5500, ISR +10 dB) */
            int32_t v = g_samples[k] / 8;
            lcg = lcg * 1103515245u + 12345u;
            v += (int32_t)((lcg >> 16) % 30000u) - 15000;
            v += (int32_t)(((int64_t)7800 * NCO_COS[ph >> 20]) >> 15);
            ph += word;
            if (v > 32767) v = 32767;
            if (v < -32768) v = -32768;
            g_samples[k] = (int16_t)v;
        }
        r = rxs_open(MODE_EXTREME, 0);
        for (pos = 0; pos < n; pos += 512) {
            int c = n - pos < 512 ? n - pos : 512;
            if (rxs_push(r, g_samples + pos, c, &ev) && ev.type == 1
                && ev.pkt_bits_n == pkt_n && memcmp(ev.bits, pkt, (size_t)pkt_n) == 0)
                decoded++;
        }
        if (rxs_flush(r, &ev) && ev.type == 1 && ev.pkt_bits_n == pkt_n
            && memcmp(ev.bits, pkt, (size_t)pkt_n) == 0)
            decoded++;
        printf("  EXTREME under CW +10 dB: decoded %d, notches %d\n", decoded, rxs_notches());
        check("EXTREME frame under a CW carrier 10 dB above it decodes", decoded == 1);
    }

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
