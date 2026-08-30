/* Host reproduction of the two-board streamed-burst failure.
 *
 * Builds the SAME waveform the radio firmware transmits (station-style
 * EXT_DATA blocks, streaming generator, 8 blocks, resync 4), applies the
 * board's impairments one at a time, and walks it with the firmware's
 * burst_advance logic through the real streaming receiver.
 *
 * Impairments, cumulative flags:
 *   -q   12-bit DAC at 3/4 scale re-read by a 16-bit ADC (the wire)
 *   -d   mid-rail DC offset (-143 LSB, measured on the stand)
 *   ppm  sample-rate offset between TX and RX clocks (linear interp)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "packets.h"
#include "link.h"
#include "station.h"
#include "tx.h"
#include "rx_stream.h"

#define RESYNC BURST_STREAM_RESYNC
static int N_BLOCKS = 8;
static int FS = 25;   /* -f N */
static int RUNG = 6;  /* -r N */

static int16_t g_wave[400000];
static int16_t g_air[400000];
static uint8_t g_blocks[BURST_STREAM_MAX * (36 + 8 * (BURST_SUBHDR + 253))];

static int build_burst_wave(int *pkt_n_out)
{
    uint8_t payload[BURST_SUBHDR + 253];
    lc_word_t lc;
    int pkt_n = 36 + 8 * (BURST_SUBHDR + FS);
    int k, j, total = 0, got, pos = 0;
    txs_t *t;

    memset(&lc, 0, sizeof(lc));
    lc.seq = 1; lc.ack = 0; lc.req_rung = 12; lc.snr_db = 10.0;
    lc.freq_corr_hz = 0.0; lc.flags = FLAG_BURST_DATA;
    for (k = 0; k < N_BLOCKS; k++) {
        payload[0] = (uint8_t)(k | BURST_SUB_STREAMED);
        payload[1] = (uint8_t)(((k == N_BLOCKS - 1 || k == 0) ? 0x80 : 0)
                               | 11);
        payload[2] = FS;
        for (j = 0; j < FS; j++)
            payload[BURST_SUBHDR + j] = (uint8_t)(k * 37 + j * 11 + 5);
        if (data_encode(lc_pack(&lc), payload, BURST_SUBHDR + FS,
                        g_blocks + (size_t)k * pkt_n) != pkt_n)
            return -1;
    }
    t = txs_open(ladder_mode(RUNG), g_blocks, pkt_n, N_BLOCKS,
                 PKT_TYP_EXT_DATA, ladder_mod(RUNG), ladder_spd(RUNG),
                 RESYNC, 0, &total);
    if (!t || total <= 0 || total > (int)(sizeof(g_wave) / 2))
        return -1;
    while ((got = txs_pull(t, g_wave + pos, (int)(sizeof(g_wave)/2) - pos)) > 0)
        pos += got;
    *pkt_n_out = pkt_n;
    return pos == total ? pos : -1;
}

/* the wire: DAC 12-bit at 3/4 scale -> ADC 16-bit, plus DC */
static int16_t wire(int16_t s, int quant, int dc)
{
    int32_t v = s;
    if (quant) {
        uint32_t d = (uint32_t)(2048 + ((v * 3) >> 6));   /* firmware ISR */
        v = (int32_t)(d << 4) - 32768;                    /* ADC reads it */
        /* undo the 3/4 analog gain so the receiver sees comparable
         * amplitude to the digital case; keep the quantization */
        v = (v * 85) >> 6;   /* x64/48 ~ x85/64 */
        if (v > 32767) v = 32767;
        if (v < -32768) v = -32768;
    }
    return (int16_t)(v + (dc ? -143 : 0));
}

/* Replicate the firmware's DAC-FIFO pull pattern: tx_fill() tops a
 * 2048-sample ring while an "ISR" drains 1..256 samples at a time, and
 * every drained sample is compared against the flat single-pull
 * waveform. tx_short in the firmware checks only the COUNT -- this
 * checks the VALUES, which is the part a chunking bug in the streaming
 * generator's burst path would corrupt while the count still matched. */
static int fifo_check(const uint8_t *blocks, int pkt_n, const int16_t *ref,
                      int ref_n)
{
    static int16_t fifo[2048];
    uint32_t w = 0, r = 0, lcg = 99;
    int total = 0, out = 0;
    txs_t *t = txs_open(ladder_mode(RUNG), blocks, pkt_n, N_BLOCKS,
                        PKT_TYP_EXT_DATA, ladder_mod(RUNG),
                        ladder_spd(RUNG), RESYNC, 0, &total);
    if (!t || total != ref_n) { printf("fifo: open/len mismatch\n"); return -1; }
    while (out < ref_n) {
        /* top up exactly as tx_fill does */
        while (t && (uint32_t)(w - r) < 2048u) {
            int wi = (int)(w & 2047u);
            int room = (int)(2048u - (w - r));
            int lin = 2048 - wi;
            int got;
            if (room > lin) room = lin;
            if (room <= 0) break;
            got = txs_pull(t, fifo + wi, room);
            if (got <= 0) { t = 0; break; }
            w += (uint32_t)got;
        }
        if (r == w) break;
        /* drain a random 1..256 like the tick would over time */
        lcg = lcg * 1103515245u + 12345u;
        {
            int m = 1 + (int)((lcg >> 16) % 256u);
            while (m-- > 0 && r != w) {
                int16_t v = fifo[r & 2047u];
                if (v != ref[out]) {
                    printf("fifo: DIVERGES at sample %d: %d != ref %d\n",
                           out, v, ref[out]);
                    return -1;
                }
                r++; out++;
            }
        }
    }
    printf("fifo: %d/%d samples identical to the flat pull\n", out, ref_n);
    return out == ref_n ? 0 : -1;
}

int main(int argc, char **argv)
{
    int quant = 0, dc = 0, i, do_fifo = 0, do_tx = 0;
    double ppm = 0.0;
    int pkt_n = 0, n, air_n, lead = 700;
    rxs_t *rx[3];
    int burst_left = 0, burst_miss = 0, ev_count = 0;
    int got_blocks[BURST_STREAM_MAX];

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-f")) FS = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-r")) RUNG = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-n")) N_BLOCKS = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-q")) quant = 1;
        else if (!strcmp(argv[i], "-d")) dc = 1;
        else if (!strcmp(argv[i], "-fifo")) do_fifo = 1;
        else if (!strcmp(argv[i], "-tx")) do_tx = 1;
        else ppm = atof(argv[i]);
    }
    memset(got_blocks, 0, sizeof(got_blocks));

    n = build_burst_wave(&pkt_n);
    if (n <= 0) { printf("build failed\n"); return 1; }
    if (do_fifo)
        return fifo_check(g_blocks, pkt_n, g_wave, n) ? 1 : 0;

    /* resample by ppm (RX clock faster: reads the waveform slower) and
     * push through the wire model */
    {
        double step = 1.0 / (1.0 + ppm * 1e-6), src = 0.0;
        int k = 0;
        memset(g_air, 0, (size_t)lead * 2);
        while (src < n - 1 && lead + k < (int)(sizeof(g_air)/2) - 8) {
            int i0 = (int)src;
            double fr = src - i0;
            double v = g_wave[i0] * (1.0 - fr) + g_wave[i0 + 1] * fr;
            g_air[lead + k++] = wire((int16_t)(v + (v >= 0 ? 0.5 : -0.5)),
                                     quant, dc);
            src += step;
        }
        air_n = lead + k + 700;
    }

    rx[0] = rxs_open(MODE_NORMAL, 0);
    rx[1] = rxs_open(MODE_ROBUST, 0);
    rx[2] = rxs_open(MODE_EXTREME, 0);

    for (i = 0; i + 256 <= air_n; i += 256) {
        int m;
        /* -tx: emulate the station preparing (then abandoning) a
         * transmission between pushes mid-walk -- the arena traffic the
         * firmware generates that no test exercises */
        if (do_tx && burst_left > 0 && (i & 0x1FFF) == 0) {
            int tot = 0;
            static int16_t junk[512];
            uint8_t pkt[64];
            int pn = data_encode(7, (const uint8_t *)"LINK", 4, pkt);
            txs_t *t = txs_open(MODE_NORMAL, pkt, pn, 1, PKT_TYP_DATA,
                                MOD_BPSK, ladder_spd(4), 0, 0, &tot);
            if (t)
                txs_pull(t, junk, 512);   /* generator state in the arena */
        }
        for (m = 0; m < 3; m++) {
            rxs_event_t ev;
            if (!rx[m] || !rxs_push(rx[m], g_air + i, 256, &ev))
                continue;
            ev_count++;
            if (m == 0) {
                int streamed = 0, ackreq = 0;
                if (ev.type == 1) {
                    /* read the marker exactly as the firmware does */
                    lc_word_t lc; uint32_t res = 0; int b, v = 0;
                    for (b = 0; b < 20; b++)
                        res = (res << 1) | (ev.bits[b] & 1);
                    lc_unpack(res, &lc);
                    for (b = 0; b < 8; b++)
                        v = (v << 1) | (ev.bits[20 + b] & 1);
                    streamed = lc.flags == FLAG_BURST_DATA
                               && (v & BURST_SUB_STREAMED);
                    ackreq = ev.bits[20 + 8] & 1;
                    if (streamed) {
                        int idx = v & 0x7F;
                        if (idx >= 0 && idx < N_BLOCKS)
                            got_blocks[idx] = 1;
                    }
                }
                printf("  event: type %d%s%s\n", ev.type,
                       streamed ? " STREAMED" : "",
                       streamed && ackreq ? " ackreq" : "");
                if (streamed && burst_left == 0) {
                    burst_left = BURST_STREAM_MAX - 1; burst_miss = 0;
                } else if (streamed) {
                    burst_miss = 0;
                    if (ackreq) burst_left = 0; else burst_left--;
                } else if (burst_left > 0) {
                    if (++burst_miss >= 2) burst_left = 0;
                    else burst_left--;
                }
                if (burst_left > 0 && !rxs_continue_burst(rx[0], RESYNC))
                    burst_left = 0;
            }
        }
    }
    {
        int ok = 0;
        for (i = 0; i < N_BLOCKS; i++)
            ok += got_blocks[i];
        printf("RESULT ppm=%g quant=%d dc=%d : %d/%d blocks decoded\n",
               ppm, quant, dc, ok, N_BLOCKS);
        return ok == N_BLOCKS ? 0 : 1;
    }
}
