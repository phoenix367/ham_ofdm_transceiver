/* Adaptive broadcast under a FADING channel -- the host twin of the
 * firmware's block/stats/re-rung machinery (usb_radio_main.c), which
 * the two-board stand cannot exercise: its wire does not fade.
 *
 * The sender cuts the stream into ~45 s blocks exactly as the firmware
 * does, ends each with the dataless EOB group, and re-rungs on the
 * receiver's BCSTAT. The receiver derives its desired rung from its
 * own measured SNR through the ladder sensitivities (the firmware's
 * rule), and stays SILENT below BURST_MIN_RUNG (the window-fit rule).
 * The channel is AWGN whose SNR follows a V-shaped fade:
 *
 *     snr(t) = SNR_HI                          t < T0
 *              ramp down to SNR_LO             T0 .. T1
 *              SNR_LO                          T1 .. T2
 *              ramp back to SNR_HI             T2 .. T3
 *
 *   -fixed   disable adaptation (hold the starting rung): the control
 *            arm for the comparison
 *   -r N     starting rung (default 12)
 *   -len N   payload bytes (default 8192)
 *   -lo DB   fade floor (default -5)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "packets.h"
#include "link.h"
#include "station.h"
#include "broadcast.h"
#include "tx.h"
#include "rx_stream.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define BC_GROUP 4
#define BC_FRAME 26
#define BLOCK_AIR_S 45.0
#define BLOCK_AIR_SHORT_S 20.0   /* after trouble: re-probe quickly */
#define MARGIN 2.5

static int RUNG = 12, SRC_LEN = 8192, FIXED = 0;
static double SNR_HI = 20.0, SNR_LO = -5.0;
static double T0 = 40.0, T1 = 80.0, T2 = 140.0, T3 = 180.0;

static uint8_t g_src[65536];
static int g_off, g_seq, g_rung;
static double g_block_air, g_clock;
static uint8_t g_blocks[BC_GROUP * (36 + 8 * BC_FRAME)];
static int16_t g_wave[600000];
static txs_t *g_txs;

static double fade_snr(double t)
{
    if (t < T0) return SNR_HI;
    if (t < T1) return SNR_HI + (SNR_LO - SNR_HI) * (t - T0) / (T1 - T0);
    if (t < T2) return SNR_LO;
    if (t < T3) return SNR_LO + (SNR_HI - SNR_LO) * (t - T3) / (T2 - T3);
    return SNR_HI;
}

/* build one group (or the EOB marker) at g_rung; returns samples */
static int build_group(int eob, int *total_out)
{
    uint8_t payload[BC_FRAME];
    int pkt_n = 0, nf = 0, first = 1, cap0 = BC_FRAME - 3,
        cap = BC_FRAME - 2, total = 0;

    if (eob) {
        memset(payload, 0, sizeof(payload));
        payload[0] = (uint8_t)(BC_SYNC | (g_seq & BC_SEQ_MASK));
        payload[1] = 0;
        payload[2] = (uint8_t)BC_PT_OPAQUE;
        pkt_n = data_encode(0, payload, BC_FRAME, g_blocks);
        g_seq++;
        nf = 1;
    } else {
        while (nf < BC_GROUP && g_off < SRC_LEN) {
            int take = first ? cap0 : cap;
            int flags = first ? BC_SYNC : 0;
            if (take > SRC_LEN - g_off)
                take = SRC_LEN - g_off;
            if (g_off + take >= SRC_LEN)
                flags |= BC_EOS;
            memset(payload, 0, sizeof(payload));
            payload[0] = (uint8_t)(flags | (g_seq & BC_SEQ_MASK));
            payload[1] = (uint8_t)take;
            if (first) {
                int gc = 0, g = BC_GROUP;
                while (g > 1) { g >>= 1; gc++; }
                payload[2] = (uint8_t)((gc << 4) | BC_PT_OPAQUE);
            }
            memcpy(payload + (first ? 3 : 2), g_src + g_off, (size_t)take);
            pkt_n = data_encode(0, payload, BC_FRAME,
                                g_blocks + (size_t)nf * (36 + 8 * BC_FRAME));
            g_off += take;
            g_seq++;
            nf++;
            first = 0;
        }
        {
            int i;
            for (i = 1; i < nf; i++)
                memmove(g_blocks + (size_t)i * pkt_n,
                        g_blocks + (size_t)i * (36 + 8 * BC_FRAME),
                        (size_t)pkt_n);
        }
    }
    g_txs = txs_open(ladder_mode(g_rung), g_blocks, pkt_n, nf,
                     PKT_TYP_BCAST, ladder_mod(g_rung), ladder_spd(g_rung),
                     BURST_STREAM_RESYNC, 0, total_out);
    if (!g_txs)
        return -1;
    {
        int got, pos = 0;
        while ((got = txs_pull(g_txs, g_wave + pos,
                               (int)(sizeof(g_wave) / 2) - pos)) > 0)
            pos += got;
        g_txs = 0;
        return pos;
    }
}

/* receiver state (the firmware's bc walk, condensed) */
static rxs_t *g_rx;
static int r_left, r_last_seq = -1, r_frames, r_lost, r_bytes, r_eos;
static int r_frames0, r_lost0;   /* snapshot at the last stats reply */
static double r_snr = 20.0;
static uint8_t r_out[65536];

static void rx_event(const rxs_event_t *ev, int *stat_due)
{
    const uint8_t *b = ev->bits;
    int plen, j, v, flags, seq, dlen, head = 2;

    if (ev->type == -3 && ev->hdr.typ == PKT_TYP_BCAST && r_left > 0) {
        r_left--;
        r_lost++;
        if (r_left > 0 && !rxs_continue_burst(g_rx, BURST_STREAM_RESYNC))
            r_left = 0;
        return;
    }
    if (ev->type != 1 || ev->hdr.typ != PKT_TYP_BCAST)
        return;
    plen = (ev->pkt_bits_n - 36) / 8;
    for (j = 0, v = 0; j < 8; j++)
        v = (v << 1) | (b[20 + j] & 1);
    flags = v & ~BC_SEQ_MASK;
    seq = v & BC_SEQ_MASK;
    for (j = 0, v = 0; j < 8; j++)
        v = (v << 1) | (b[28 + j] & 1);
    dlen = v;
    if (flags & BC_SYNC) {
        /* group size from the DESCRIPTOR, as the firmware reads it --
         * hardcoding BC_GROUP here made this harness chase three ghost
         * blocks after every single-frame EOB group and report 58%
         * loss into a +20 dB channel */
        int t = 0, q, grp;
        head = 3;
        for (q = 0; q < 8; q++)
            t = (t << 1) | (b[36 + q] & 1);
        grp = 1 << (t >> 4);
        if (grp < 1 || grp > BURST_STREAM_MAX)
            grp = BC_GROUP;
        r_left = grp - 1;
        if (dlen == 0 && !(flags & BC_EOS))
            *stat_due = 1;               /* the EOB marker */
    } else if (r_left > 0) {
        r_left--;
    }
    if (r_last_seq >= 0) {
        int gap = (seq - r_last_seq - 1) & BC_SEQ_MASK;
        if (gap > 0 && gap < 32)
            r_lost += gap;
    } else if (seq > 0 && seq < 32) {
        r_lost += seq;
    }
    r_last_seq = seq;
    r_frames++;
    r_snr = ev->snr_db;
    if (dlen > plen - head)
        dlen = plen - head;
    for (j = 0; j < dlen && r_bytes < (int)sizeof(r_out); j++) {
        int bb, val = 0;
        for (bb = 0; bb < 8; bb++)
            val = (val << 1) | (b[20 + 8 * (head + j) + bb] & 1);
        r_out[r_bytes++] = (uint8_t)val;
    }
    if (flags & BC_EOS)
        r_eos = 1;
    if (r_left > 0 && !(flags & BC_EOS)
        && !rxs_continue_burst(g_rx, BURST_STREAM_RESYNC))
        r_left = 0;
}

static uint32_t g_lcg = 20260831;
static double gauss(void)
{
    double u1, u2;
    g_lcg = g_lcg * 1103515245u + 12345u;
    u1 = ((g_lcg >> 8) & 0xFFFFFF) / 16777216.0 + 1e-12;
    g_lcg = g_lcg * 1103515245u + 12345u;
    u2 = ((g_lcg >> 8) & 0xFFFFFF) / 16777216.0;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

int main(int argc, char **argv)
{
    static int16_t air[620000];
    int i, stat_due = 0, min_rung = 99, rerungs = 0;
    double block_len = BLOCK_AIR_S;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-fixed")) FIXED = 1;
        else if (!strcmp(argv[i], "-r")) RUNG = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-len")) SRC_LEN = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-lo")) SNR_LO = atof(argv[++i]);
    }
    for (i = 0; i < SRC_LEN; i++)
        g_src[i] = (uint8_t)(i * 41 + 7);
    g_rung = RUNG;
    g_rx = rxs_open(MODE_NORMAL, 0);

    printf("adaptive=%s start rung %d, fade %+.0f -> %+.0f -> %+.0f dB\n",
           FIXED ? "OFF" : "on", RUNG, SNR_HI, SNR_LO, SNR_HI);
    while (g_off < SRC_LEN && g_clock < 900.0) {
        int eob = 0, n, k;
        double snr_now = fade_snr(g_clock), sig_rms;
        if (!FIXED && g_block_air >= block_len
            && g_off < SRC_LEN) {
            eob = 1;
            g_block_air = 0.0;
        }
        n = build_group(eob, &k);
        if (n <= 0) { printf("build failed\n"); return 1; }
        /* the wire: signal RMS fixed by the modem; noise sized so the
         * delivered SNR follows the fade profile */
        {
            double acc = 0.0, sigma;
            for (k = 0; k < n; k++)
                acc += (double)g_wave[k] * g_wave[k];
            sig_rms = sqrt(acc / n);
            sigma = sig_rms / pow(10.0, snr_now / 20.0);
            memset(air, 0, 700 * sizeof(int16_t));
            for (k = 0; k < n; k++) {
                double v = g_wave[k] + sigma * gauss();
                air[700 + k] = (int16_t)(v > 32767 ? 32767
                                        : v < -32768 ? -32768 : v);
            }
            for (k = 0; k < 700; k++)
                air[700 + n + k] = (int16_t)(sigma * gauss());
        }
        for (k = 0; k + 256 <= n + 1400; k += 256) {
            rxs_event_t ev;
            if (rxs_push(g_rx, air + k, 256, &ev))
                rx_event(&ev, &stat_due);
        }
        g_clock += (double)n / 12000.0;
        g_block_air += (double)n / 12000.0;
        if (eob) {
            int replied = 0;
            if (stat_due) {
                /* Desired rung: the SNR-derived rung, PULLED DOWN by
                 * the loss rate since the last stats reply. The SNR
                 * alone is survivorship-biased -- it is measured only
                 * on frames strong enough to decode, and this harness
                 * read +22 dB inside a -5 dB fade. Losses are the
                 * fade's real signature. */
                int r, bd = (r_frames - r_frames0) + (r_lost - r_lost0);
                double lf = bd > 0
                            ? (double)(r_lost - r_lost0) / bd : 0.0;
                for (r = ladder_n() - 1; r > 0; r--)
                    if (r_snr >= ladder_sens_db(r) + MARGIN)
                        break;
                /* A LOSSY block anchors on the rung that produced the
                 * losses, never on the SNR: the estimate is measured
                 * only on frames that survived, and read +22 dB inside
                 * a -5 dB fade while 41% of the block died. Severe
                 * loss goes straight to the floor. */
                if (lf > 0.40)
                    r = BURST_MIN_RUNG;
                else if (lf > 0.25)
                    r = g_rung - 4;
                else if (lf > 0.05)
                    r = g_rung - 2;
                if (r < 0) r = 0;
                stat_due = 0;
                if (r >= BURST_MIN_RUNG) {
                    replied = 1;
                    printf("t=%5.1f s  snr %+6.1f dB  stats: %d ok %d "
                           "lost (blk loss %2.0f%%, rx snr %+5.1f) asks "
                           "rung %d", g_clock, snr_now, r_frames, r_lost,
                           lf * 100.0, r_snr, r);
                    r_frames0 = r_frames;
                    r_lost0 = r_lost;
                    /* a lossy block shortens the next one: the probe
                     * cadence must track trouble, not the calendar */
                    block_len = lf > 0.05 ? BLOCK_AIR_SHORT_S
                                          : BLOCK_AIR_S;
                    if (r != g_rung) {
                        g_rung = r;
                        rerungs++;
                        printf("  -- re-rung");
                    }
                    printf("\n");
                    g_clock += 2.0;
                } else {
                    printf("t=%5.1f s  snr %+6.1f dB  stats: receiver "
                           "SILENT (needs rung %d)\n", g_clock, snr_now,
                           r);
                    g_clock += 4.0;
                }
            } else {
                g_clock += 4.0;      /* window held, marker lost */
            }
            if (!replied)
                block_len = BLOCK_AIR_SHORT_S;
            if (!replied && g_rung > BURST_MIN_RUNG) {
                /* No reply where one was promised IS the feedback: the
                 * EOB (or the reply) died in whatever is eating the
                 * stream, or the receiver went silent below NORMAL.
                 * Either way, down. */
                g_rung -= 2;
                if (g_rung < BURST_MIN_RUNG)
                    g_rung = BURST_MIN_RUNG;
                rerungs++;
                printf("t=%5.1f s  snr %+6.1f dB  no stats reply -- "
                       "stepping down to rung %d\n", g_clock, snr_now,
                       g_rung);
            }
        }
        if (g_rung < min_rung)
            min_rung = g_rung;
    }
    printf("RESULT adaptive=%s: %d/%d payload bytes delivered, "
           "%d frames ok, %d lost, %.1f s air, min rung %d, %d re-rung(s)"
           "%s\n", FIXED ? "OFF" : "on", r_bytes, SRC_LEN,
           r_frames, r_lost, g_clock, min_rung, rerungs,
           r_eos ? "" : " [no EOS seen]");
    return 0;
}
