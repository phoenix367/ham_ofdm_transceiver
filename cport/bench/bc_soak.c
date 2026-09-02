/* Soak the firmware's broadcast build-and-walk path at the VOICE shape:
 * 2-frame groups of ~30 B, random payload, random inter-group gaps,
 * random sample-phase lead, thousands of groups. The question is binary:
 * does the ~0.76% group loss measured on the two-board stand reproduce
 * in the code path, or is it the air?
 *
 *   -r N      rung (default 12, the voice rung)
 *   -t N      trials (default 100)
 *   -g N      groups per trial (default 18, one 20 s transmission)
 *   -q        the 12-bit DAC / 16-bit ADC wire
 *   -all      open NORMAL+EXTREME receivers (the board's mask 5)
 *   -seed N
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "packets.h"
#include "link.h"
#include "station.h"
#include "broadcast.h"
#include "tx.h"
#include "rx_stream.h"

#define BC_FRAME 26

static int RUNG = 12, TRIALS = 100, GROUPS = 18, QUANT = 0, ALL_MODES = 0,
           VERBOSE = 0, SWEEP = 0;
static unsigned SEED = 12345;

static uint8_t g_bc_blocks[2 * (36 + 8 * BC_FRAME)];
static int16_t g_wave[80000];
static int16_t g_air[2200000];

static unsigned rng_state;
static unsigned rnd(void) {
    rng_state = rng_state * 1664525u + 1013904223u;
    return rng_state >> 8;
}

static int16_t wire(int16_t s)
{
    int32_t v = s;
    if (QUANT) {
        uint32_t d = (uint32_t)(2048 + ((v * 3) >> 6));
        v = (int32_t)(d << 4) - 32768;
        v = (v * 85) >> 6;
    }
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

/* build ONE 2-frame group carrying `chunk` bytes (23 in frame 0, the
 * rest in frame 1), seq numbering continuous across the stream */
static int build_group(int seq0, int chunk, int eos, int16_t *dst, int cap)
{
    uint8_t payload[BC_FRAME];
    int pkt_n = 0, nf, n = 0, got;
    txs_t *txs; int tx_total = 0;

    for (nf = 0; nf < 2; nf++) {
        int first = (nf == 0);
        int take = first ? 23 : chunk - 23;
        int flags = first ? BC_SYNC : 0;
        int j;
        if (!first && eos) flags |= BC_EOS;
        memset(payload, 0, sizeof(payload));
        payload[0] = (uint8_t)(flags | ((seq0 + nf) & BC_SEQ_MASK));
        payload[1] = (uint8_t)take;
        if (first) payload[2] = (uint8_t)((1 << 4) | 0x0F); /* log2(2), opaque */
        for (j = 0; j < take; j++)
            payload[(first ? 3 : 2) + j] = (uint8_t)(rnd() & 0xFF);
        pkt_n = data_encode(0, payload, BC_FRAME,
                            g_bc_blocks + (size_t)nf * (36 + 8 * BC_FRAME));
    }
    memmove(g_bc_blocks + (size_t)pkt_n, g_bc_blocks + (36 + 8 * BC_FRAME),
            (size_t)pkt_n);
    txs = txs_open(ladder_mode(RUNG), g_bc_blocks, pkt_n, 2, PKT_TYP_BCAST,
                   ladder_mod(RUNG), ladder_spd(RUNG), BURST_STREAM_RESYNC,
                   0, &tx_total);
    if (!txs) return -1;
    while ((got = txs_pull(txs, g_wave + n,
                           (int)(sizeof(g_wave) / 2) - n)) > 0)
        n += got;
    if (n != tx_total) { printf("SHORT PULL %d/%d\n", n, tx_total); return -1; }
    if (n > cap) return -1;
    { int i; for (i = 0; i < n; i++) dst[i] = wire(g_wave[i]); }
    return n;
}

static rxs_t *g_rxs[3];
static int g_bc_left[3];
static int g_last_seq, g_frames, g_lost, g_neg;
static uint8_t g_seen[64];                  /* seqs decoded this trial */
static int g_gapb[40], g_chunk[40], g_gstart[40];   /* per-group build info */

static void on_event(int m, const rxs_event_t *ev)
{
    if (ev->type == -3 && ev->hdr.typ == PKT_TYP_BCAST && g_bc_left[m] > 0) {
        g_bc_left[m]--;
        g_lost++;
        if (g_bc_left[m] > 0
            && !rxs_continue_burst(g_rxs[m], BURST_STREAM_RESYNC))
            g_bc_left[m] = 0;
        return;
    }
    if (ev->type != 1 || ev->hdr.typ != PKT_TYP_BCAST) {
        g_neg++;
        if (VERBOSE)
            printf("      EV m%d type %d typ %d start %lld snr %+.1f\n",
                   m, ev->type, ev->hdr.typ, (long long)ev->start_abs,
                   ev->snr_db);
        return;
    }
    {
        const uint8_t *b = ev->bits;
        int j, v, flags, seq;
        for (j = 0, v = 0; j < 8; j++) v = (v << 1) | (b[20 + j] & 1);
        flags = v & ~BC_SEQ_MASK; seq = v & BC_SEQ_MASK;
        if (flags & BC_SYNC) g_bc_left[m] = 1;   /* group of 2 */
        else if (g_bc_left[m] > 0) g_bc_left[m]--;
        if (g_last_seq >= 0) {
            int gap = (seq - g_last_seq - 1) & BC_SEQ_MASK;
            if (gap > 0 && gap < 32) g_lost += gap;
        }
        g_last_seq = seq;
        if (seq >= 0 && seq < 64) g_seen[seq] = 1;
        g_frames++;
        if (VERBOSE)
            printf("      ok m%d seq %d start %lld\n", m, seq,
                   (long long)ev->start_abs);
        if (flags & BC_EOS) g_bc_left[m] = 0;
        if (g_bc_left[m] > 0
            && !rxs_continue_burst(g_rxs[m], BURST_STREAM_RESYNC))
            g_bc_left[m] = 0;
    }
}

int main(int argc, char **argv)
{
    int i, t, tot_frames = 0, tot_lost = 0, tot_groups = 0, bad_trials = 0;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-r")) RUNG = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-t")) TRIALS = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-g")) GROUPS = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-q")) QUANT = 1;
        else if (!strcmp(argv[i], "-all")) ALL_MODES = 1;
        else if (!strcmp(argv[i], "-seed")) SEED = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "-v")) VERBOSE = 1;
        else if (!strcmp(argv[i], "-sweep")) SWEEP = atoi(argv[++i]);
    }

    if (SWEEP == 1) TRIALS = 512;
    if (SWEEP == 2) TRIALS = 1024;
    for (t = 0; t < TRIALS; t++) {
        int pos, g, seq = 0, m, air_n, lead;
        rng_state = SEED + (SWEEP ? 0u : (unsigned)t * 7919u);
        lead = SWEEP == 1 ? 300 + t
                     : 300 + (int)(rnd() % 512);  /* sweep block alignment */
        if (SWEEP == 2) lead = 300;               /* fixed; the GAP sweeps */
        memset(g_air, 0, sizeof(*g_air) * (size_t)lead);
        pos = lead;
        for (g = 0; g < GROUPS; g++) {
            int chunk = 30 + (int)(rnd() % 2) * 5;   /* 30 or 35 B */
            int n;
            g_gstart[g] = pos; g_chunk[g] = chunk;
            n = build_group(seq, chunk, g == GROUPS - 1,
                                g_air + pos,
                                (int)(sizeof(g_air) / 2) - pos - 30000);
            if (n < 0) { printf("build failed t=%d g=%d\n", t, g); return 2; }
            seq += 2;
            pos += n;
            /* inter-group gap: the sender re-keys on its own schedule */
            { int gap = SWEEP == 2 ? 1000 + t : 1000 + (int)(rnd() % 15000);
              g_gapb[g + 1 < 40 ? g + 1 : 39] = gap;
              memset(g_air + pos, 0, sizeof(*g_air) * (size_t)gap);
              pos += gap; }
        }
        g_gapb[0] = lead;
        air_n = pos + 2000;

        for (m = 0; m < 3; m++) { g_bc_left[m] = 0; }
        g_last_seq = -1; g_frames = 0; g_lost = 0;
        memset(g_seen, 0, sizeof(g_seen));
        g_rxs[0] = rxs_open(MODE_NORMAL, 0);
        if (ALL_MODES) g_rxs[2] = rxs_open(MODE_EXTREME, 0);
        for (i = 0; i + 256 <= air_n; i += 256) {
            for (m = 0; m < 3; m++) {
                rxs_event_t ev;
                if (!g_rxs[m] || !rxs_push(g_rxs[m], g_air + i, 256, &ev))
                    continue;
                on_event(m, &ev);
            }
        }
        /* rxs_open is keyed by mode on a static pool: reopening next
         * trial reinitialises the slot, so nothing to free here */
        for (m = 0; m < 3; m++) g_rxs[m] = 0;

        tot_frames += g_frames; tot_lost += g_lost;
        tot_groups += GROUPS;
        if (g_frames != 2 * GROUPS || g_lost) {
            bad_trials++;
            printf(SWEEP ? "MISS t=%1$d lead %2$d (mod 256: %5$d) frames %3$d/%4$d\n"
                         : "trial %1$3d: lead %2$d  frames %3$d/%4$d  lost %6$d\n",
                   t, lead, g_frames, 2 * GROUPS, lead % 256, g_lost);
            if (!SWEEP) for (g = 0; g < GROUPS; g++) {
                int miss0 = !g_seen[2 * g], miss1 = !g_seen[2 * g + 1];
                if (miss0 || miss1)
                    printf("      group %2d (seq %2d,%2d): %s%s  "
                           "start %d (mod B: %d)  gap-before %d  chunk %d\n",
                           g, 2 * g, 2 * g + 1,
                           miss0 ? "SYNC-missed " : "",
                           miss1 ? "second-missed" : "",
                           g_gstart[g], g_gstart[g] % 512,
                           g_gapb[g], g_chunk[g]);
            }
        }
        if ((t + 1) % 20 == 0)
            printf("  ... %d trials, %d frames, %d lost\n",
                   t + 1, tot_frames, tot_lost);
    }
    printf("SOAK rung=%d quant=%d all=%d: %d trials, %d groups, "
           "%d frames decoded, %d lost, %d bad trial(s)\n",
           RUNG, QUANT, ALL_MODES, TRIALS, tot_groups,
           tot_frames, tot_lost, bad_trials);
    printf("group-equivalent loss rate: %.3f %%  (stand measured 0.76 %%)\n",
           tot_frames + tot_lost > 0
               ? 100.0 * tot_lost / (tot_frames + tot_lost) : 0.0);
    return bad_trials ? 1 : 0;
}
