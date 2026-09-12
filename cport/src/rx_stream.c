#include <string.h>
#ifdef RXS_TRACE
#include <stdio.h>
#endif

#include "rx_stream.h"
#include "rx_internal.h"
#include "rx_detect.h"
#include "packets.h"
#include "conv.h"
#include "ldpc.h"
#include "fxp.h"
#include "fft.h"
#include "dsp.h"
#include "rom_tables.h"
#include "rom_modes.h"


#define MAX_SHIFTS 33                    /* +-16 mask-roll grid, EXTREME */
/* Summary ring per instance, sized by that instance's OWN mode: it only
 * ever scans its own tone window (15 / 30 / 120 blocks). Giving all
 * three the EXTREME count cost 110 kB for nothing. Caps are powers of
 * two so the index stays a mask. */
#define BLK_CAP 128                      /* EXTREME: >= 120 blocks */
#ifndef WEAK_COMMIT_X      /* -DWEAK_COMMIT_X=1 disables the gate (A/B) */
#define WEAK_COMMIT_X 64   /* region-end commit needs metric >= 64x thr^2 */
#endif
#define BLK_CAP_NORMAL 16                /* >= 15  */
#define BLK_CAP_ROBUST 32                /* >= 30  */
#define BLK_TOTAL (BLK_CAP_NORMAL + BLK_CAP_ROBUST + BLK_CAP)
#define DECLINE_BLOCKS 3
/* PREEMPTION. The tone stage keeps evaluating windows while the header
 * and data states run, as a CHALLENGER tracked by the same region /
 * stability rules as the search. A challenger that would commit and
 * whose preamble arrives with more tone-bin energy than the one this
 * frame was committed on (by RXS_PREEMPT_LOG2_Q4) aborts the decode
 * and is committed in its place.
 *
 * Why: a same-mode frame 20 dB below ours (10 dB under the noise at
 * +10 dB SNR) still detects -- its tone comb has ~+9 dB per-bin SNR --
 * and its header often decodes, after which this receiver used to spend
 * the stranger's whole data block (2.1 s NORMAL, 32 s EXTREME) deaf;
 * our own preamble inside that window was consumed and, the search
 * being newest-block-only, never revisited. Measured 18-28 % PER at
 * NORMAL +10 dB from ISR -20 dB up against a frame-at-once model that
 * lost nothing (experiments/interference.py). The ratio, not a plain
 * "any new peak": an interferer at or above our own power destroys the
 * frame in progress anyway (ISR 0 dB is ~100 % PER in the sweep), so
 * switching to the stronger signal is right on average, and the 3 dB
 * margin keeps an equal-power stranger, which may or may not kill the
 * frame in hand, from taking it for certain.
 * -DRXS_PREEMPT=0 builds the old behaviour for A/B. */
#ifndef RXS_PREEMPT
#define RXS_PREEMPT 1
#endif
/* The bar is the challenger's TONE-BIN ENERGY against the locked
 * frame's, as log2 in Q4 (16 = one octave = 3 dB), not the metric
 * ratio: the contrast metric saturates against its 1 % floor
 * regularizer once the per-bin SNR passes ~20 dB, so two strong
 * preambles 12 dB apart score the same and the ratio cannot order
 * them. Tone-bin energy is what "stronger" means and does not
 * saturate. Our own data symbols in the challenger's window put only
 * n_mask/N_DATA_CARRIERS of our power in the tone bins (~-8 dB), below
 * the bar even before the challenger has to pass the commit rule. */
#ifndef RXS_PREEMPT_LOG2_Q4
#define RXS_PREEMPT_LOG2_Q4 16
#endif
/* ZC search margin either side of the tone field's end, in blocks.
 *
 * Counted in BLOCKS, not samples, so it tracks B -- which is what the
 * candidate's error tracks: cs_abs is a block boundary, so it is wrong
 * by up to a block from quantisation alone, plus whatever the tone
 * arg-max adds. Measured total error is well under one block in every
 * mode (+37 / -219 / -220 samples).
 *
 * TUNED, and bounded on BOTH sides, only a factor of two apart:
 *   too small (4) -- the window stops covering the ZC and acquisitions
 *     are lost;
 *   too large (16) -- NORMAL's tone field is only 3840 samples, so
 *     16*256 exceeds it, the anchor clamps to 0, and the window grows
 *     wide enough to reach past the preamble into data symbols, where a
 *     correlator will eventually find a spurious peak.
 * Both fail test_stream. Re-run `make zcab` if you move this. */

#define ZC_ANCHOR_MARGIN_BLK 8
#define ZC_WIN_MAX 71000
#define STREAM_MAX_SYM (CP_LEN + 64 * FFT_BINS)
#define RXS_MAX_INST 3

typedef struct {
    int64_t band;
    int64_t dot[2][MAX_SHIFTS];
    int exp;
} blk_sum_t;

/* STATIONARY-CARRIER EXCISION, streaming twin of rx_find_tones (reasoning
 * in ofdm.py, FullOFDMModem.IND_X). Each instance keeps a rolling history
 * of its blocks' ON bins, EXC_HIST_SAMPLES deep = one EXTREME tone
 * field (3 x 160 x 128 samples, 5.1 s) WHATEVER ITS OWN MODE, so that a
 * bin lit by a tone comb -- ours, a stranger's preamble train, or a
 * peer's EXTREME preamble seen by the NORMAL instance -- is on for at
 * most 2/3 of the history and never reaches the 85 % stationarity
 * fraction, while a carrier does. The bank is GLOBAL and filters the
 * SHARED ring, so a NORMAL instance sized for its own 0.32 s window
 * would notch the EXTREME peer's comb out from under the EXTREME
 * instance (measured: it did, 3 notches per frame on the other-mode
 * preamble arm of the sweep). A carrier's frequency comes from the
 * inter-block phase advance over the summary window, and the request
 * goes to that one bank on the push path: every consumer (tone stage,
 * ZC, demod, every instance) then reads the notched samples. The bank
 * releases a notch by its own monitor once the carrier is gone
 * (dsp.c). 5.1 s of latency at onset is the price of never notching a
 * preamble; the sweep gives the streaming receiver that lead.
 *
 * A stationary group is judged from per-bin running averages of the
 * bin/mean ratio (one division per bin per block, tau 64 blocks): the
 * averaged-ratio and comb tests of rx_internal.h need nothing else.
 * Only a candidate that passes them has its FREQUENCY measured, once,
 * by re-reading the last DET_FIND_BLOCKS blocks of the raw ring as REAL
 * samples (the positive bins of a real block's FFT are the analytic
 * signal's, no Hilbert needed) for the bin's phase advance between
 * consecutive blocks. A first version re-read the ring to judge every
 * group: during an EXTREME frame all 23 subcarriers are stationary,
 * each its own group, and that was 1472 FFTs per 43 ms block -- hours
 * under QEMU, impossible on the part. Once a notch is up the ring
 * holds notched samples, so a bin that already has a notch within a
 * bin's width only refreshes it. */
#define HIST_MAX 256
#define EXC_HIST_SAMPLES (3 * 160 * FFT_BINS)   /* EXTREME tone field */

/* Raw (NOT derotated) lag-FFT_BINS correlation per block, including the
 * products that straddle the previous block. Summing these over the tone
 * segment replaces RE-READING it: derotating by a constant per-sample
 * phase multiplies every lag-L product by the same e^{-jTL}, so the
 * coarse word comes off as one angle SUBTRACTION at commit instead of a
 * pass over 40960 samples. Verified against the old path on a real
 * EXTREME preamble across +-120 Hz: worst disagreement 316 word units =
 * 0.00088 Hz, against a 93.75 Hz coarse bin. This is what stops the raw
 * ring having to reach back to cs_abs.
 *
 * It lives in its OWN ring, not in blk_sum_t, and that is the whole
 * point: the tone detector commits about TWO tone windows after the
 * peak, so at commit the segment's blocks are already out of g_blk,
 * which only holds one window. At 16 bytes per block this ring can
 * afford to hold two (plus slack) where g_blk's ~544 bytes could not. */
#define LAG_N FFT_BINS
#define LAG_CAP_NORMAL   64      /* >= 2*15  + slack */
#define LAG_CAP_ROBUST  128      /* >= 2*30  + slack */
#define LAG_CAP_EXTREME 256      /* >= 2*120 + slack */
#define LAG_TOTAL (LAG_CAP_NORMAL + LAG_CAP_ROBUST + LAG_CAP_EXTREME)

typedef struct {
    int64_t re, im;      /* lag N   = FFT_BINS   -- the fine estimate */
    int64_t re2, im2;    /* lag N/2 = FFT_BINS/2 -- resolves its wrap */
} lag_sum_t;
#define LAG_N2 (FFT_BINS / 2)

static int64_t div_round_signed(int64_t a, int64_t b)
{
    return a >= 0 ? (a + b / 2) / b : -((-a + b / 2) / b);
}

/* tone-peak tracker: the argmax over one contiguous above-threshold
 * region of window offsets, plus the decline count that ends it. One
 * for the search, one for the challenger that runs during a decode. */
typedef struct {
    int crossed, decline;
    int64_t best_metric;
    int64_t best_energy;  /* tone-bin energy at the argmax, log2 Q4 */
    int64_t best_off_blk, min_blk; /* min_blk: earliest window START */
    int best_shift;
    int64_t best_eval_blk; /* when the argmax was last improved */
} peak_t;

struct rxs_state {
    int inst;
    rxd_t demod;
    link_mode_t mode;
    int B, T, max_shift, thr_q10, n_mask_bins;
    int64_t word_per_bin;
    const uint8_t *mask0, *mask1;
    int tone0, tone1, total_blocks;
    int zc_win, zc_anchor;

    int64_t abs_n; /* samples consumed so far */

    /* state machine */
    enum { S_SEARCH, S_ZC_WAIT, S_HEADER, S_DATA } st;
    int active;               /* 0 = consume samples, skip the work */
    peak_t pk;                /* the search */
    peak_t ch;                /* the challenger, live during a decode */
    int64_t locked_metric;    /* what the frame in progress committed on */
    int64_t locked_energy;    /* its tone-bin energy, log2 Q4 */
    /* rolling carrier finder (see HIST_MAX): per block one bit per bin
     * ("on" = above 1.5x the in-band mean), a ring hist_len blocks deep,
     * and the per-bin count of set bits in the ring */
    uint8_t ind[EXC_HIST_SAMPLES / 16];   /* hist_len blocks x B/16 bytes = 3840 */
    uint8_t hits[512 / 2];
    uint16_t ravg[512 / 2];   /* per-bin EWMA of pow/mean, Q8, tau 64 blocks */
    int hist_len, hist_fill, hist_pos;
    int64_t preempts;         /* decodes abandoned for a stronger peak */
    int64_t cs_abs, start_abs, cfo_word;
    int64_t cw; /* coarse tone word */
    int sym_idx, n_hdr, n_data, mu, cap, use_ldpc;
    /* streamed bursts: data_base is the first symbol of the CURRENT
     * block, so continuing a burst is a matter of stepping it on */
    int64_t data_base;
    int blk_idx, burst_resync;
    int64_t burst_resume_abs;
    int64_t last_eval_blk;
    int blk_base, blk_mask;   /* this instance's slice of g_blk */
    int lag_base, lag_mask;   /* and of g_lag */
    int64_t ring_hwm; /* deepest raw lookback (incl. FIR history) */
    int64_t ring_miss; /* reads refused because the samples were gone */
    rxd_header_t hdr;
    int hdr_scale, data_scale;
    uint8_t hdr_bits[HEADER_BITS];
    int pkt_bits_count; /* PKT_BITS_FROM_HDR(hdr.typ, hdr.len) */
};

static struct rxs_state g_pool[RXS_MAX_INST];
/* persistent per-instance state */
/* ONE shared raw ring for all instances: every receiver is fed the same
 * audio (mode self-labeling by preamble root), so concurrent instances
 * write identical values -- idempotent. Instances fed different streams
 * must not be live concurrently (sequential reuse is fine: an instance
 * only reads samples it wrote itself). The analytic signal is
 * reconstructed on extraction (Hilbert-on-read), which costs one 63-tap
 * FIR per extracted sample and removes the per-instance analytic rings
 * entirely (2 B/sample raw vs 8 B/sample analytic per instance). */
static int16_t g_raw[RXS_RAW_RING_LEN];
/* the notch bank in front of the shared ring, and the last sample index
 * it has processed: instances push the same samples, the first to push
 * an index filters and writes it, the others skip (idempotent ring) */
static notch_bank_t g_bank;
static int64_t g_notch_done = -1;
static blk_sum_t g_blk[BLK_TOTAL];
static lag_sum_t g_lag[LAG_TOTAL];
static const int BLK_CAP_OF[3] = { BLK_CAP_NORMAL, BLK_CAP_ROBUST, BLK_CAP };
static const int BLK_OFF_OF[3] = { 0, BLK_CAP_NORMAL,
                                   BLK_CAP_NORMAL + BLK_CAP_ROBUST };
static const int LAG_CAP_OF[3] = { LAG_CAP_NORMAL, LAG_CAP_ROBUST,
                                   LAG_CAP_EXTREME };
static const int LAG_OFF_OF[3] = { 0, LAG_CAP_NORMAL,
                                   LAG_CAP_NORMAL + LAG_CAP_ROBUST };
static llr_t g_h64[RXS_MAX_INST][MAX_LLRS], g_d64[RXS_MAX_INST][MAX_LLRS];
static int g_hexps[RXS_MAX_INST][MAX_SYMS], g_dexps[RXS_MAX_INST][MAX_SYMS];
static rxd_wacc_t g_wacc[RXS_MAX_INST];   /* interference weighting, per block */
/* per-call scratch (never live across rxs_push calls) */
/* No segment scratch: the tone stage's residual and the ZC scan both
 * pull through zc_ring_fetch, which reads the shared raw ring directly
 * and derotates on the way out. This pair held 1.1 MB. */

/* decode phase, arena slot 0 (see rx_internal.h) */
static llr_t *const g_q64 = (llr_t *)rx_arena;
/* One symbol's samples, shared by the header and data demod states.
 *
 * Both are call-scoped -- ring_copy fills it and rxd_demod_symbol
 * consumes it within the same step, nothing survives the return -- and
 * the states are mutually exclusive: the header completes before the
 * first data symbol. Two separate copies cost 128 kB to hold the same
 * thing at different times. */
/* also from the shared arena: demod runs only after detection has
 * finished with it (see rx_internal.h) */
typedef char sym_arena_fits[4 * STREAM_MAX_SYM * sizeof(samp_t)
                            <= RX_ARENA_BYTES ? 1 : -1];
static samp_t *const g_sym_i = (samp_t *)rx_arena;
static samp_t *const g_sym_q = (samp_t *)(rx_arena + STREAM_MAX_SYM * sizeof(samp_t));

/* analytic extraction: recompute the streaming Hilbert from the raw
 * ring, bit-identical to the former write-time FIR (zero prehistory;
 * i[n] = x[n-31] pairs with the group-delayed q[n]) */
/* Is every raw sample this call will touch still in the ring?
 *
 * The ring holds only the last RXS_RAW_RING_LEN samples written, and
 * ring_copy addresses it by modulo alone -- so a request that has fallen
 * off the back does not fail, it silently returns samples from the
 * PREVIOUS wrap. Those decode to a CRC failure indistinguishable from
 * noise, which is the worst possible way for a receiver to report that
 * it could not keep up. The FIR reaches HILBERT_TAPS_N-1 samples behind
 * abs_start, so that history has to be resident too.
 *
 * This does not fire in normal operation: the deepest lookback measured
 * on a heavily-lagged receiver was 124478 of 147456. It is here so that
 * if it ever does, it is visible as a miss rather than as mystery
 * corruption. */
static void rearm(rxs_t *r, int64_t guard_abs);

static int ring_resident(const rxs_t *r, int64_t abs_start, int n)
{
    if (r->abs_n - abs_start + (HILBERT_TAPS_N - 1) > RXS_RAW_RING_LEN)
        return 0;                     /* overwritten since it was written */
    if (abs_start + n > r->abs_n)
        return 0;                     /* not written yet */
    return 1;
}

static int ring_copy(rxs_t *r, int64_t abs_start, int n,
                     samp_t *di, samp_t *dq)
{
    int k, j;
    int ok = ring_resident(r, abs_start, n);

    if (r->abs_n - abs_start + (HILBERT_TAPS_N - 1) > r->ring_hwm)
        r->ring_hwm = r->abs_n - abs_start + (HILBERT_TAPS_N - 1);
    if (!ok)
        r->ring_miss++;   /* copied anyway -- the caller decides */
    for (k = 0; k < n; k++) {
        int64_t abs = abs_start + k;
        int64_t acc = 0;
        int idx = (int)(abs % RXS_RAW_RING_LEN);
        int nt = abs + 1 < HILBERT_TAPS_N ? (int)(abs + 1)
                                          : HILBERT_TAPS_N;
        for (j = 0; j < nt; j++) {
            acc += (int64_t)HILBERT_TAPS[j] * g_raw[idx];
            if (--idx < 0)
                idx += RXS_RAW_RING_LEN;
        }
        dq[k] = (samp_t)rshift_round(acc, Q15);
        di[k] = abs >= HILBERT_DELAY
                    ? (samp_t)g_raw[(int)((abs - HILBERT_DELAY)
                                          % RXS_RAW_RING_LEN)]
                    : 0;
    }
    return ok ? 0 : -1;
}

/* ZC scan source: the samples are already in the shared raw ring, so
 * materialising a derotated int64 copy of the whole search window cost
 * 1.1 MB to hold data we had. The scan's live span is only
 * preamble_len+1 samples (see zc_src_t), so it pulls what it needs and
 * we derotate on the way out.
 *
 * The derotation phase is a pure function of the index -- phase =
 * k * cw, mod 2^32 -- which is what makes random access possible. */
typedef struct {
    rxs_t *r;
    int64_t base_abs, cw;
} zc_ring_ctx_t;

static void zc_ring_fetch(void *ctx, int k, int n, samp_t *di, samp_t *dq)
{
    zc_ring_ctx_t *z = (zc_ring_ctx_t *)ctx;
    ring_copy(z->r, z->base_abs + k, n, di, dq);
    nco_derotate(di, dq, n, z->cw, (uint32_t)((int64_t)(uint32_t)z->cw * k),
                 di, dq);
}

rxs_t *rxs_open(link_mode_t mode, int calibrate)
{
    /* keyed by MODE, not round-robin: the summary slice below is sized
     * for this mode's tone window, so the two must agree. One live
     * instance per mode, which is how the stack uses it (demoapp opens
     * exactly one receiver per link mode). */
    rxs_t *r = &g_pool[(int)mode];
    int inst = (int)mode;
    static const struct {
        int B, T, max_shift, thr, nmask;
        int64_t wpb;
        const uint8_t *m0, *m1;
    } D[3] = {
        { DET_B_NORMAL, DET_T_NORMAL, DET_MAX_SHIFT_NORMAL,
          DET_NEWMAN_THR_Q10_NORMAL, DET_N_MASK_BINS_NORMAL,
          DET_WORD_PER_BIN_NORMAL, DET_MASK0_NORMAL, DET_MASK1_NORMAL },
        { DET_B_ROBUST, DET_T_ROBUST, DET_MAX_SHIFT_ROBUST,
          DET_NEWMAN_THR_Q10_ROBUST, DET_N_MASK_BINS_ROBUST,
          DET_WORD_PER_BIN_ROBUST, DET_MASK0_ROBUST, DET_MASK1_ROBUST },
        { DET_B_EXTREME, DET_T_EXTREME, DET_MAX_SHIFT_EXTREME,
          DET_NEWMAN_THR_Q10_EXTREME, DET_N_MASK_BINS_EXTREME,
          DET_WORD_PER_BIN_EXTREME, DET_MASK0_EXTREME, DET_MASK1_EXTREME },
    };
    memset(r, 0, sizeof(*r));
    r->inst = inst;
    /* the shared raw ring is NOT cleared: another instance may be live,
     * and a fresh instance only ever reads samples it has written */
    rxd_init(&r->demod, mode);
    r->demod.calibrate = calibrate;
    r->mode = mode;
    r->B = D[mode].B;
    r->T = D[mode].T;
    r->max_shift = D[mode].max_shift;
    r->thr_q10 = D[mode].thr;
    r->n_mask_bins = D[mode].nmask;
    r->word_per_bin = D[mode].wpb;
    r->mask0 = D[mode].m0;
    r->mask1 = D[mode].m1;
    r->tone0 = 2 * r->T * FFT_BINS / r->B;
    r->tone1 = r->T * FFT_BINS / r->B;
    r->total_blocks = r->tone0 + r->tone1;
    r->blk_base = BLK_OFF_OF[(int)mode];
    r->blk_mask = BLK_CAP_OF[(int)mode] - 1;
    r->lag_base = LAG_OFF_OF[(int)mode];
    r->lag_mask = LAG_CAP_OF[(int)mode] - 1;
    if (r->total_blocks > BLK_CAP_OF[(int)mode])
        return 0;   /* window does not fit its slice -- caps are wrong */
    /* The ZC sits at the END of the tone field, and cs_abs is accurate to
     * a few hundred samples (measured +37 / -219 / -220 for the three
     * modes). Scanning from cs_abs across the whole tone field therefore
     * searched ~60000 offsets to find something within a block or two of
     * a known place -- and, worse, anchored the read at cs_abs, which is
     * two tone fields behind the write head by the time the detector
     * commits. Anchor at the tone field's end instead, with a margin of
     * four blocks either way. */
#ifdef ZC_ANCHOR_LEGACY
    /* pre-anchoring behaviour, kept buildable so the two can be A/B'd on
     * identical waveforms -- see bench/zc_anchor_ab.c */
    r->zc_anchor = 0;
    r->zc_win = 3 * r->T * FFT_BINS + r->demod.symbol_len + 4 * FFT_BINS
                + r->B;
#else
    r->zc_anchor = 3 * r->T * FFT_BINS - ZC_ANCHOR_MARGIN_BLK * r->B;
    if (r->zc_anchor < 0)
        r->zc_anchor = 0;
    r->zc_win = r->demod.symbol_len + 2 * ZC_ANCHOR_MARGIN_BLK * r->B;
#endif
    r->hist_len = EXC_HIST_SAMPLES / r->B;   /* 240 / 120 / 120 blocks */
    if (r->hist_len > HIST_MAX)
        r->hist_len = HIST_MAX;
    /* one bank for the shared ring; a fresh instance starts a fresh
     * stream (every live instance is opened before the first push) */
    notch_bank_clear(&g_bank);
    g_notch_done = -1;
    r->st = S_SEARCH;
    r->active = 1;
    r->pk.best_metric = -1;
    r->ch.best_metric = -1;
    r->locked_metric = -1;
    r->last_eval_blk = -1;
    r->ring_hwm = 0;
    r->ring_miss = 0;
    return r;
}

int64_t rxs_ring_hwm(const rxs_t *r)
{
    return r->ring_hwm;
}

int64_t rxs_ring_miss(const rxs_t *r)
{
    return r->ring_miss;
}

int64_t rxs_preempts(const rxs_t *r)
{
    return r->preempts;
}

int rxs_notches(void)
{
    return notch_bank_active(&g_bank);
}

#ifdef STREAM_DEBUG
#include <stdio.h>
void rxs_dump_bank(void)
{
    int i;
    for (i = 0; i < NOTCH_MAX; i++)
        fprintf(stderr, "bank slot %d: active %d word %u idle %d\n", i,
                g_bank.active[i], (unsigned)g_bank.word[i], g_bank.idle[i]);
}
#endif

#ifdef STREAM_DEBUG
#include <stdio.h>
#define SDBG(...) fprintf(stderr, __VA_ARGS__)
#else
#define SDBG(...)
#endif

/* rolling stationary-carrier finder (see HIST_MAX): fold this block's ON
 * bits into the history and, for any bin ON in 85 % of the last
 * hist_len blocks, judge the group from the raw ring and request a
 * notch */
static void carrier_track(rxs_t *r, int64_t blk_idx, const uint8_t *bits)
{
    int H = r->hist_len, half = r->B / 2, S = r->B / FFT_BINS, k, byte;
    int nbytes = half / 8;
    uint8_t *slot = r->ind + r->hist_pos * nbytes;
    if (r->hist_fill == H) {   /* drop the oldest block */
        const uint8_t *old = slot;
        for (k = 1; k < half; k++)
            if (old[k >> 3] & (1u << (k & 7)))
                r->hits[k]--;
    } else {
        r->hist_fill++;
    }
    for (byte = 0; byte < nbytes; byte++)
        slot[byte] = bits[byte];
    for (k = 1; k < half; k++)
        if (bits[k >> 3] & (1u << (k & 7)))
            r->hits[k]++;
    r->hist_pos = (r->hist_pos + 1) % H;
    if (r->hist_fill < H)
        return;
    for (k = 1; k < half; k++) {
        int j, q, best, comb = 0;
        int64_t sr = 0, si = 0, pr = 0, pi = 0, ang, mag, b, first;
        int K = r->hist_len < DET_FIND_BLOCKS ? r->hist_len : DET_FIND_BLOCKS;
        if (r->hits[k] * DET_STAT_DEN < DET_STAT_NUM * H)
            continue;
        j = k;
        while (j + 1 < half && r->hits[j + 1] * DET_STAT_DEN >= DET_STAT_NUM * H)
            j++;
        /* the strongest bin of the group, by running average */
        best = k;
        for (q = k; q <= j; q++)
            if (r->ravg[q] > r->ravg[best])
                best = q;
        if (r->ravg[best] < DET_AVG_Q8) {
            k = j;
            continue;
        }
        /* a modulated comb is not a carrier (rx_internal.h) */
        for (q = best - S; q <= best + S; q += 2 * S)
            if (q >= 1 && q < half && r->hits[q] * 2 >= H
                && (int64_t)r->ravg[q] * DET_COMB_RATIO >= r->ravg[best])
                comb = 1;
        if (comb) {
            k = j;
            continue;
        }
        /* a notch already within a bin's width of it: refresh, no
         * re-estimate (the ring is notched there by now) */
        if (notch_bank_near(&g_bank, (uint32_t)(((int64_t)best << 32) / r->B),
                            (uint32_t)(((int64_t)1 << 32) / r->B))) {
            k = j;
            continue;
        }
        /* its frequency: the bin's phase advance over the last K blocks
         * of the raw ring, real input */
        first = blk_idx - K + 1;
        if (first < 0)
            first = 0;
        for (b = first; b <= blk_idx; b++) {
            int64_t re[512], im[512];
            int t, exp;
            for (t = 0; t < r->B; t++) {
                re[t] = g_raw[(int)((b * r->B + t) % RXS_RAW_RING_LEN)];
                im[t] = 0;
            }
            fft_bfp(re, im, r->B, 13, &exp);
            if (b > first) {
                sr += pr * re[best] + pi * im[best];
                si += pr * im[best] - pi * re[best];
            }
            pr = re[best];
            pi = im[best];
        }
        if (sr != 0 || si != 0) {
            uint32_t word;
            cordic_atan2(si, sr, &ang, &mag);
            word = (uint32_t)((((int64_t)best << 32) + ang + r->B / 2)
                              / r->B);
            SDBG("carrier: bin %d word %u -> notch\n", best, word);
            if (notch_bank_request(&g_bank, word) == 2) {
                /* A carrier just got its notch: whatever tone region
                 * the search has been tracking was built on the raw
                 * carrier, not on a preamble. Measured at EXTREME:
                 * the carrier sat in a mask bin of one CFO shift for
                 * the 5.1 s before the notch engaged, its region's
                 * stability commit fired 123 blocks later -- as the
                 * frame arrived -- and the ZC and header attempts on
                 * that anchor held the receiver through the frame's
                 * tone field, which at 10 dB below the carrier could
                 * not preempt them. Only windows starting after this
                 * block count from here (NORMAL never noticed: its
                 * window is 15 blocks, the false attempt is over long
                 * before a frame can arrive). */
                if (r->pk.best_metric < (int64_t)r->thr_q10 * r->thr_q10
                                            * WEAK_COMMIT_X) {
                    /* ...but only a WEAK region: a carrier's is (7x the
                     * threshold on the traced failure), a genuine peak's
                     * is above the 64x strong-commit bar even at the
                     * knee. A borderline carrier engages its notch at a
                     * random moment, mid-frame included, and dropping a
                     * strong region then cost 68 % of the frames at ISR
                     * -3 dB at EXTREME. */
                    r->pk.crossed = 0;
                    r->pk.decline = 0;
                    r->pk.best_metric = -1;
                    r->pk.min_blk = blk_idx + 1;
                }
                r->ch.crossed = 0;
                r->ch.decline = 0;
                r->ch.best_metric = -1;
            }
        }
        k = j;
    }
}

/* per-block summary: BFP spectrum -> band power + mask dots per shift */
static void block_summary(rxs_t *r, int64_t blk_idx)
{
    int64_t re[512], im[512], pow_[512];
    blk_sum_t *bs = &g_blk[r->blk_base + (int)(blk_idx & r->blk_mask)];
    int B = r->B, k, sh, exp;

    /* Status ignored on purpose. The detection stages validate
     * themselves -- a stale read fails the ZC correlation threshold or
     * the header CRC and costs one acquisition attempt. Refusing them
     * instead was measured to throw away GOOD acquisitions: with a ring
     * deliberately undersized to 8192, aborting here took a receiver
     * that decoded 16 of 16 blocks down to 0. */
    {   /* the ring yields samp_t; the FFT works in int64 */
        samp_t sre[512 + LAG_N], sim[512 + LAG_N];   /* DET_B max + lag */
        int t;
        int64_t base = blk_idx * B - LAG_N;
        int hist = LAG_N;

        if (base < 0) {           /* first block: no history to correlate */
            base = 0;
            hist = 0;
        }
        ring_copy(r, base, B + hist, sre, sim);
        for (t = 0; t < B; t++) {
            re[t] = sre[hist + t];
            im[t] = sim[hist + t];
        }
        {   /* Raw correlations at BOTH lags, same term order as
             * lag_words_src(). Two lags for the reason rx_detect.c gives
             * and CLAUDE.md insists on: the lag-N phase wraps at
             * +-fs/2N = +-46.9 Hz, exactly one coarse bin, and the
             * coarse mask-shift search is a near-tie between adjacent
             * bins -- so a one-bin miss makes lag-N wrap and land
             * 93.75 Hz out, which kills the frame. Lag N/2 is
             * unambiguous over +-fs/N and picks the right cycle.
             *
             * The streaming receiver never had the second lag: its
             * commit used lag-N alone, and the frame-at-once path's
             * unwrap was never carried across. Found on the analog test
             * stand, then reproduced digitally: a NORMAL frame whose
             * tone field starts exactly on a block boundary (lead 0 or
             * 512) failed at -94.07 Hz in every modulation, while lead
             * 700 decoded -- on every receiver back to before the
             * acquisition changes. */
            lag_sum_t *ls = &g_lag[r->lag_base
                                   + (int)(blk_idx & r->lag_mask)];
            ls->re = 0;  ls->im = 0;
            ls->re2 = 0; ls->im2 = 0;
            for (t = (hist ? 0 : LAG_N); t < B; t++) {
                int a = hist + t - LAG_N, b = hist + t;
                ls->re += (int64_t)sre[a] * sre[b] + (int64_t)sim[a] * sim[b];
                ls->im += (int64_t)sre[a] * sim[b] - (int64_t)sim[a] * sre[b];
            }
            for (t = (hist ? 0 : LAG_N2); t < B; t++) {
                int a = hist + t - LAG_N2, b = hist + t;
                ls->re2 += (int64_t)sre[a] * sre[b] + (int64_t)sim[a] * sim[b];
                ls->im2 += (int64_t)sre[a] * sim[b] - (int64_t)sim[a] * sre[b];
            }
        }
    }
    fft_bfp(re, im, B, 13, &exp);
    bs->exp = exp;
    bs->band = 0;
    for (k = 0; k < B; k++)
        pow_[k] = re[k] * re[k] + im[k] * im[k];
    for (k = 1; k < B / 2; k++)
        bs->band += pow_[k];
    {   /* carrier finder: which bins are above 1.5x the in-band mean,
         * and each bin's running average of its ratio to the mean */
        uint8_t bits[512 / 16];
        int64_t mean = bs->band / (B / 2 - 1);
        memset(bits, 0, sizeof bits);
        for (k = 1; k < B / 2; k++) {
            int64_t rq = mean > 0 ? (pow_[k] << 8) / mean : 0;
            if (rq > 65535)
                rq = 65535;
            if (pow_[k] * DET_IND_DEN > DET_IND_NUM * mean)
                bits[k >> 3] |= (uint8_t)(1u << (k & 7));
            r->ravg[k] = (uint16_t)(r->ravg[k] + ((rq - r->ravg[k]) >> 6));
        }
        carrier_track(r, blk_idx, bits);
    }
    for (sh = -r->max_shift; sh <= r->max_shift; sh++) {
        int64_t s0 = 0, s1 = 0;
        for (k = 0; k < B; k++) {
            int src = (k - sh) & (B - 1);
            if (r->mask0[src])
                s0 += pow_[k];
            if (r->mask1[src])
                s1 += pow_[k];
        }
        bs->dot[0][sh + r->max_shift] = s0;
        bs->dot[1][sh + r->max_shift] = s1;
    }
}

/* running tone contrast for the window ending at blk_idx (windowed
 * exponent alignment + windowed median floor -- the causal divergence) */
static void eval_tone_window(rxs_t *r, int64_t blk_idx, int64_t *metric_out,
                             int *shift_out, int64_t *energy_out)
{
    int64_t band_al[BLK_CAP];
    int64_t off0 = blk_idx - r->total_blocks + 1;
    int e_min = 1 << 30, sh, b;
    int64_t floor_v = 0, best = -1, best_sig = 0;
    int best_sh = 0;
    int n_band_bins = r->B / 2 - 1;

    for (b = 0; b < r->total_blocks; b++) {
        int e = g_blk[r->blk_base + (int)((off0 + b) & r->blk_mask)].exp;
        if (e < e_min)
            e_min = e;
    }
    for (b = 0; b < r->total_blocks; b++) {
        blk_sum_t *bs = &g_blk[r->blk_base + (int)((off0 + b) & r->blk_mask)];
        int s2 = 2 * (bs->exp - e_min);
        if (s2 > 62)
            s2 = 62;
        band_al[b] = bs->band >> s2;
        /* causal floor: the frame-at-once model regularizes with 1% of the
         * whole capture's MEDIAN block power, which is acausal (it counts
         * the frame's loud data blocks). The causal analogue with the same
         * intent is 1% of the window's MAXIMUM block power: tone-bearing
         * windows self-regularize, so partial-overlap windows on a quiet
         * channel cannot saturate the contrast and out-score the true
         * alignment (measured failure without this). */
        if (band_al[b] > floor_v)
            floor_v = band_al[b];
    }

    for (sh = 0; sh < 2 * r->max_shift + 1; sh++) {
        int64_t sig0 = 0, sig1 = 0, band0 = 0, band1 = 0;
        int64_t rest0, rest1, c0, c1, metric;
        for (b = 0; b < r->tone0; b++) {
            blk_sum_t *bs = &g_blk[r->blk_base + (int)((off0 + b) & r->blk_mask)];
            int s2 = 2 * (bs->exp - e_min);
            if (s2 > 62)
                s2 = 62;
            sig0 += bs->dot[0][sh] >> s2;
            band0 += band_al[b];
        }
        for (b = r->tone0; b < r->total_blocks; b++) {
            blk_sum_t *bs = &g_blk[r->blk_base + (int)((off0 + b) & r->blk_mask)];
            int s2 = 2 * (bs->exp - e_min);
            if (s2 > 62)
                s2 = 62;
            sig1 += bs->dot[1][sh] >> s2;
            band1 += band_al[b];
        }
        rest0 = band0 - sig0;
        rest1 = band1 - sig1;
        if (rest0 < 1)
            rest0 = 1;
        if (rest1 < 1)
            rest1 = 1;
        rest0 += (floor_v * r->tone0) / 100;
        rest1 += (floor_v * r->tone1) / 100;
        c0 = (sig0 * n_band_bins * 1024) / (rest0 * r->n_mask_bins);
        c1 = (sig1 * n_band_bins * 1024) / (rest1 * r->n_mask_bins);
        metric = c0 * c1;
        if (metric > best) {
            best = metric;
            best_sh = sh - r->max_shift;
            best_sig = sig0 + sig1;
        }
    }
    *metric_out = best;
    *shift_out = best_sh;
    /* absolute tone-bin energy: the window's mantissas are aligned to
     * e_min, and a BFP exponent counts headroom LEFT-shifts, so the true
     * energy is the aligned sum >> 2*e_min (fft_bfp, rx_demod.c) */
    *energy_out = (best_sig > 0 ? rxd_log2_q4(best_sig) : 0)
                  - (int64_t)32 * e_min;
}


/* commit the tone peak: coarse CFO word from shift + lag-N residual */
static void tone_commit(rxs_t *r)
{
    int seg_n = 2 * r->T * FFT_BINS;
    SDBG("tone_commit: off_blk=%lld cs=%lld shift=%d metric=%lld energy=%lld\n",
         (long long)r->pk.best_off_blk, (long long)(r->pk.best_off_blk * r->B),
         r->pk.best_shift, (long long)r->pk.best_metric,
         (long long)r->pk.best_energy);
    r->cs_abs = r->pk.best_off_blk * r->B;
    r->cw = (int64_t)r->pk.best_shift * r->word_per_bin;
    /* arm the challenger: only windows starting AFTER this peak, and
     * this peak's metric is the bar it has to clear (x RXS_PREEMPT_X) */
    r->locked_metric = r->pk.best_metric;
    r->locked_energy = r->pk.best_energy;
    r->ch.crossed = 0;
    r->ch.decline = 0;
    r->ch.best_metric = -1;
    r->ch.min_blk = r->pk.best_off_blk + 1;
    {   /* Summed from the per-block summaries -- the tone segment is NOT
         * re-read. The blocks were correlated raw, so the coarse word is
         * removed here as an angle subtraction (blk_sum_t). Before this,
         * the re-read anchored at cs_abs and single-handedly set the raw
         * ring's size. */
        int64_t rr = 0, ri = 0, rr2 = 0, ri2 = 0, ang, mag;
        int nb = seg_n / r->B, b;
        for (b = 0; b < nb; b++) {
            const lag_sum_t *ls =
                &g_lag[r->lag_base
                       + (int)((r->pk.best_off_blk + b) & r->lag_mask)];
            rr += ls->re;   ri += ls->im;
            rr2 += ls->re2; ri2 += ls->im2;
        }
        cordic_atan2(ri, rr, &ang, &mag);
        ang = (int64_t)(int32_t)(uint32_t)(ang - r->cw * LAG_N);
        {
            int64_t fine = div_round_signed(ang, LAG_N);
            int64_t coarse, ambig = ((int64_t)1 << PHASE_BITS) / FFT_BINS, k;
            /* the lag-N/2 estimate, same coarse-word subtraction, and
             * the unwrap exactly as rx_residual_word_src() does it */
            cordic_atan2(ri2, rr2, &ang, &mag);
            ang = (int64_t)(int32_t)(uint32_t)(ang - r->cw * LAG_N2);
            coarse = div_round_signed(ang, LAG_N2);
            k = div_round_signed(coarse - fine, ambig);
            fine += k * ambig;
#ifdef LAG_AB
            {   /* the old path, for comparison only */
                zc_ring_ctx_t z; zc_src_t src; int64_t ref;
                z.r = r; z.base_abs = r->cs_abs; z.cw = r->cw;
                src.ctx = &z; src.fetch = zc_ring_fetch;
                ref = rx_lag_n_word_src(&src, seg_n);
                fprintf(stderr,
                        "[lag] mode=%d nb=%d off_blk=%lld  old=%lld new=%lld"
                        "  diff=%lld (%.4f Hz)\n",
                        (int)r->mode, nb, (long long)r->pk.best_off_blk,
                        (long long)ref, (long long)fine,
                        (long long)(fine - ref),
                        (double)(fine - ref) * 12000.0 / 4294967296.0);
            }
#endif
            r->cw += fine;
        }
    }
    r->st = S_ZC_WAIT;
}

static void rearm(rxs_t *r, int64_t guard_abs)
{
    int64_t newest = r->abs_n / r->B - 1;
    r->st = S_SEARCH;
    if (r->ch.crossed
        && newest - r->ch.best_eval_blk <= r->total_blocks + DECLINE_BLOCKS
        && r->ch.best_metric >= (int64_t)r->thr_q10 * r->thr_q10
                                    * WEAK_COMMIT_X) {
        /* A challenger region is still open as the decode ends: carry it
         * into the search instead of restarting past the frame. Measured
         * need: a weak stranger's frame FAILED its data CRC three blocks
         * after our +10 dB preamble had become the challenger's argmax
         * (metric 3.5e9, energy 67 Q4 over the bar) but before its region
         * had ended, and the old reset put the search's floor past our
         * preamble -- the next commits were garbage on our own data
         * symbols. The recency test keeps a stale region from an unmute
         * (rxs_set_active) or a long-finished decode out of it, and the
         * strength gate keeps out the data-hover regions that only ever
         * commit by stability: carrying those cost one failed header per
         * decoded frame on a busy channel (measured 3 -> 6 commits per
         * 10 s of a stranger's traffic) for nothing. */
        r->pk = r->ch;
    } else {
        r->pk.crossed = 0;
        r->pk.decline = 0;
        r->pk.best_metric = -1;
        r->pk.min_blk = (guard_abs + r->B - 1) / r->B;
    }
    r->ch.crossed = 0;
    r->ch.decline = 0;
    r->ch.best_metric = -1;
}

/* Fold one window evaluation into a peak tracker, S_SEARCH's rule for
 * both the search and the challenger. Returns 1 when the tracker's
 * commit rule fires: the above-threshold region ended on a STRONG
 * argmax, or the argmax has held for a full window span of newer
 * evaluations (see the S_SEARCH comment for why each clause exists). */
static int track_peak(const rxs_t *r, peak_t *p, int64_t blk,
                      int64_t metric, int shift, int64_t energy)
{
    int64_t thr2 = (int64_t)r->thr_q10 * r->thr_q10;
    int64_t off = blk - r->total_blocks + 1;
    if (metric > thr2 && off >= p->min_blk) {
        p->crossed = 1;
        p->decline = 0;
        if (metric > p->best_metric) {
            p->best_metric = metric;
            p->best_energy = energy;
            p->best_off_blk = off;
            p->best_shift = shift;
            p->best_eval_blk = blk;
        }
    } else if (p->crossed) {
        p->decline++;
    }
    return p->crossed
           && ((p->decline >= DECLINE_BLOCKS
                && p->best_metric >= thr2 * WEAK_COMMIT_X)
               || blk - p->best_eval_blk >= r->total_blocks + DECLINE_BLOCKS);
}

/* The challenger: evaluate the newest window while a decode is in
 * progress and, if a later peak commits with RXS_PREEMPT_LOG2_Q4 more
 * tone-bin energy than this frame was committed on, abandon the frame
 * for it.
 * Returns 1 when the state machine has been redirected. */
static int challenge(rxs_t *r)
{
#if RXS_PREEMPT
    int64_t blk = r->abs_n / r->B - 1, metric, energy;
    int shift;
    if (blk < r->total_blocks - 1 || blk == r->last_eval_blk
        || blk - r->total_blocks + 1 < r->ch.min_blk)
        return 0;
    r->last_eval_blk = blk;
    eval_tone_window(r, blk, &metric, &shift, &energy);
    if (metric > (int64_t)r->thr_q10 * r->thr_q10 || r->ch.crossed) {
        SDBG("challenge: st=%d blk=%lld metric=%lld sh=%d e=%lld crossed=%d "
             "decline=%d best=%lld off=%lld\n", (int)r->st, (long long)blk,
             (long long)metric, shift, (long long)energy, r->ch.crossed,
             r->ch.decline, (long long)r->ch.best_metric,
             (long long)r->ch.best_off_blk);
    }
    if (!track_peak(r, &r->ch, blk, metric, shift, energy))
        return 0;
    if (r->ch.best_energy < r->locked_energy + RXS_PREEMPT_LOG2_Q4) {
        /* not worth the frame in hand: drop this region, keep listening */
        r->ch.crossed = 0;
        r->ch.decline = 0;
        r->ch.best_metric = -1;
        return 0;
    }
    SDBG("preempt: st=%d locked=%lld/e%lld challenger=%lld/e%lld off_blk=%lld\n",
         (int)r->st, (long long)r->locked_metric, (long long)r->locked_energy,
         (long long)r->ch.best_metric, (long long)r->ch.best_energy,
         (long long)r->ch.best_off_blk);
    r->preempts++;
    r->pk = r->ch;
    r->burst_resume_abs = 0;   /* a burst walk cannot continue from here */
    tone_commit(r);
    return 1;
#else
    (void)r;
    return 0;
#endif
}

/* finish the data block: quantize/calibrate, decode, CRC, SNR estimate */
static int finish_frame(rxs_t *r, rxs_event_t *ev)
{
    static uint8_t coded[MAX_LLRS];
    static int8_t ref[MAX_LLRS];
    int n_llr = r->n_data * r->cap;
    int e_min, s, k, ok;

    e_min = g_dexps[r->inst][0];
    for (s = 1; s < r->n_data; s++)
        if (g_dexps[r->inst][s] < e_min)
            e_min = g_dexps[r->inst][s];
    for (s = 0; s < r->n_data; s++) {
        int sh = 2 * (g_dexps[r->inst][s] - e_min);
        if (sh > 31)   /* see rx_demod: exact for values that fit int32 */
            sh = 31;
        for (k = 0; k < r->cap; k++)
            g_d64[r->inst][s * r->cap + k] >>= sh;
    }
    r->data_scale = 2 * e_min;
    rxd_wacc_apply(&g_wacc[r->inst], g_d64[r->inst], r->mu);

    if (r->demod.calibrate && r->mu <= 2) {
        int fit_shift = 0;
        int64_t alpha;
        int ncod = conv_encoded_len(CC_R13, HEADER_BITS);
        conv_encode(CC_R13, r->hdr_bits, HEADER_BITS, coded);
        rxd_known_ref(coded, ncod, N_DATA_CARRIERS, ref);
        alpha = rxd_fit_alpha_q12(g_h64[r->inst], ref, ncod, &fit_shift);
        if (alpha > 0)
            rxd_calibrated_llrs(g_d64[r->inst], n_llr, r->data_scale, alpha,
                                r->hdr_scale + fit_shift, g_q64);
        else
            rxd_quantize(g_d64[r->inst], n_llr, r->mu == 4 ? 8 : 6, g_q64);
    } else {
        rxd_quantize(g_d64[r->inst], n_llr, r->mu == 4 ? 8 : 6, g_q64);
    }

    rxd_decode_block(g_q64, n_llr, (cc_rate_t)r->hdr.spd, (int)r->use_ldpc,
                     r->pkt_bits_count, ev->bits);
    ok = data_check_crc(ev->bits, r->pkt_bits_count) == 0;

    ev->hdr = r->hdr;
    ev->pkt_bits_n = r->pkt_bits_count;
    ev->start_abs = (int)r->start_abs;
    ev->cfo_word = r->cfo_word;
    ev->snr_db = -30.0;
    if (!ok) {
        ev->type = -3;
        return 1;
    }

    {
        int64_t num = 0, den = 0, bn, bd;
        int ncod = conv_encoded_len(CC_R13, HEADER_BITS);
        int nref;
        conv_encode(CC_R13, r->hdr_bits, HEADER_BITS, coded);
        nref = rxd_known_ref(coded, ncod, N_DATA_CARRIERS, ref);
        if (rxd_snr_block_moments(g_h64[r->inst], ref, nref, N_DATA_CARRIERS,
                                  &bn, &bd) == 0) {
            num += bn;
            den += bd;
        }
        if (r->mu <= 2) {
            int ncod_d;
            if (r->use_ldpc) {
                ldpc_encode(ev->bits, r->pkt_bits_count, coded);
                ncod_d = ldpc_cc_elements(r->pkt_bits_count);
            } else {
                conv_encode((cc_rate_t)r->hdr.spd, ev->bits,
                            r->pkt_bits_count, coded);
                ncod_d = conv_encoded_len((cc_rate_t)r->hdr.spd,
                                          r->pkt_bits_count);
            }
            nref = rxd_known_ref(coded, ncod_d, r->cap, ref);
            if (rxd_snr_block_moments(g_d64[r->inst], ref, nref, r->cap, &bn, &bd)
                == 0) {
                num += bn;
                den += bd;
            }
        }
        if (num > 0 && den > 0) {
            int l2 = rxd_log2_q4(num) - rxd_log2_q4(den);
            ev->snr_db = rxd_snr_map(r->mode, r->mu, (double)l2 / 16.0 * TEN_LOG10_2
                         
                         - rxd_tile_db(r->mode) + SNR_CAL_DB);
        }
    }
    ev->type = 1;
    return 1;
}

/* advance the state machine as far as the ring allows; 1 = event filled */
static int advance(rxs_t *r, rxs_event_t *ev)
{
    for (;;) {
        switch (r->st) {
        case S_SEARCH: {
            int64_t blk = r->abs_n / r->B - 1; /* newest complete block */
            int64_t metric, energy;
            int shift;
            if (blk < r->total_blocks - 1 || blk < r->pk.min_blk
                || blk == r->last_eval_blk)
                return 0;
            r->last_eval_blk = blk;
            eval_tone_window(r, blk, &metric, &shift, &energy);
            /* partial tone overlap already crosses the threshold ~a full
             * window before the true peak, so a decline-from-best rule
             * commits too early (measured). Instead: track the argmax over
             * the whole contiguous above-threshold REGION and commit when
             * the metric falls back below threshold -- causal, and the
             * true (fully-aligned) window is guaranteed to have been
             * evaluated (track_peak).
             *
             * Commit when the above-threshold region ends, OR when the
             * argmax has been stable for a full window span of newer
             * evaluations (quiet channels: data symbols can hover at the
             * threshold so the region never cleanly ends, but the aligned
             * tone peak scores far above them and stays the argmax).
             *
             * The region-end commit is gated on the metric being STRONG
             * (>= 64x the threshold). A tone field ENTERING the window
             * can produce an isolated above-threshold spike -- one or
             * two evaluations at ~10x threshold, at an ~8-sample window
             * of the frame's start offset mod 256 -- whose region dies
             * immediately, and committing it anchors ~2 blocks before
             * the REAL preamble: the failed ZC/header attempts then
             * consume the true peak's samples, which the search (newest
             * block only, summary ring one window deep) never revisits,
             * and the frame is lost whole. That was the entire 0.76%
             * "clean-wire" broadcast group loss of the voice campaign
             * (make bcsoak reproduces it: 28/720 frames). A WEAK argmax
             * instead takes the patient path: evaluation continues, the
             * true peak arrives within the next window, replaces the
             * argmax, and its own strong region-end commit fires.
             * Genuine peaks measure 1e4..1e6 x threshold at NORMAL and
             * ~1e3x at the EXTREME knee, so 64x costs a real frame
             * nothing but the stability wait it already tolerates on
             * quiet channels -- and a first fix that instead REWOUND
             * the search and recomputed summaries from the raw ring
             * melted both boards (cap_overruns in the millions: every
             * failed EXTREME acquisition re-scanned its excursion, on
             * noise, forever). Prefer not committing garbage over
             * recovering from having committed it. */
            if (track_peak(r, &r->pk, blk, metric, shift, energy)) {
                tone_commit(r);
                continue;
            }
            return 0;
        }
        case S_ZC_WAIT: {
            int ft;
            int64_t fw;
            /* the challenger runs here too: the ZC wait is ~25 blocks at
             * NORMAL, longer than the 15-block tone window, so a stronger
             * preamble landing inside it would otherwise be seen only by
             * its partial-overlap tail windows once the header started
             * (measured: a residual 8 % loss at ISR -20 dB without this) */
            if (challenge(r))
                continue;
            if (r->abs_n < r->cs_abs + r->zc_anchor + r->zc_win)
                return 0;
            {
                zc_ring_ctx_t z;
                zc_src_t src;
                z.r = r;
                z.base_abs = r->cs_abs + r->zc_anchor;
                z.cw = r->cw;
                src.ctx = &z;
                src.fetch = zc_ring_fetch;
                ft = 0;
                fw = 0;
                if (rx_detect_zc_src(r->mode, &src, r->zc_win, &ft, &fw)
                    != 0) {
                    SDBG("zc: no lock at cs=%lld\n", (long long)r->cs_abs);
                    rearm(r, r->cs_abs + r->B);
                    continue;
                }
            }
            if (0) {
                SDBG("zc: no lock at cs=%lld\n", (long long)r->cs_abs);
                rearm(r, r->cs_abs + r->B); /* false tone hit */
                continue;
            }
            SDBG("zc: ft=%d fw=%lld -> start=%lld\n", ft, (long long)fw,
                 (long long)(r->cs_abs + ft));
            r->start_abs = r->cs_abs + r->zc_anchor + ft;
            r->cfo_word = r->cw + fw;
            r->n_hdr = (conv_cc_elements(CC_R13, HEADER_BITS)
                        + N_DATA_CARRIERS - 1) / N_DATA_CARRIERS;
            r->sym_idx = 0;
            r->demod.last_hyp = -1;
            rxd_wacc_reset(&g_wacc[r->inst]);
            r->st = S_HEADER;
            continue;
        }
        case S_HEADER: {
            samp_t *si = g_sym_i, *sq = g_sym_q;
            int64_t pos = r->start_abs + (int64_t)r->sym_idx
                          * r->demod.symbol_len;
            int win_buf[5], win_n = 0;
            const int *window = 0;
            if (challenge(r))
                continue;
            if (r->abs_n < pos + r->demod.symbol_len)
                return 0;
            if (r->demod.last_hyp >= 0) {
                int lo = r->demod.last_hyp - 2 < 0 ? 0 : r->demod.last_hyp - 2;
                int hi = r->demod.last_hyp + 2 >= r->demod.n_words
                             ? r->demod.n_words - 1 : r->demod.last_hyp + 2;
                int k;
                for (k = lo; k <= hi; k++)
                    win_buf[win_n++] = k;
                window = win_buf;
            }
            if (ring_copy(r, pos, r->demod.symbol_len, si, sq) != 0) {
                /* stale samples here decode to a CRC failure that is
                 * indistinguishable from noise -- say so instead */
                rearm(r, pos + r->demod.symbol_len);
                continue;
            }
            {
                int64_t rho[N_DATA_CARRIERS];
                g_hexps[r->inst][r->sym_idx] = rxd_demod_symbol(
                    &r->demod, si, sq, (int)pos, r->cfo_word, 1, window, win_n,
                    g_h64[r->inst] + r->sym_idx * N_DATA_CARRIERS, rho);
                rxd_wacc_add(&g_wacc[r->inst], rho, g_hexps[r->inst][r->sym_idx]);
            }
            r->sym_idx++;
            if (r->sym_idx < r->n_hdr)
                continue;
            {
                int e_min = g_hexps[r->inst][0], s, k;
                for (s = 1; s < r->n_hdr; s++)
                    if (g_hexps[r->inst][s] < e_min)
                        e_min = g_hexps[r->inst][s];
                for (s = 0; s < r->n_hdr; s++) {
                    int sh = 2 * (g_hexps[r->inst][s] - e_min);
                    if (sh > 31)   /* see rx_demod */
                        sh = 31;
                    for (k = 0; k < N_DATA_CARRIERS; k++)
                        g_h64[r->inst][s * N_DATA_CARRIERS + k] >>= sh;
                }
                r->hdr_scale = 2 * e_min;
            }
            rxd_wacc_apply(&g_wacc[r->inst], g_h64[r->inst], 1);
            rxd_quantize(g_h64[r->inst], r->n_hdr * N_DATA_CARRIERS, 6, g_q64);
            rxd_decode_block(g_q64, r->n_hdr * N_DATA_CARRIERS, CC_R13, 0,
                             HEADER_BITS, r->hdr_bits);
            if (header_decode(r->hdr_bits, &r->hdr.ver, &r->hdr.typ,
                              &r->hdr.mod, &r->hdr.spd, &r->hdr.len) != 0) {
                ev->type = -1;
                ev->start_abs = (int)r->start_abs;
                ev->cfo_word = r->cfo_word;
                /* false anchor: resume the search just past it so the true
                 * preamble (still in the ring) can be re-acquired */
                rearm(r, r->cs_abs + r->B);
                return 1;
            }
            if ((r->hdr.ver != 1 && r->hdr.ver != 2)
                || (r->hdr.typ == PKT_TYP_EXT_DATA && r->hdr.ver == 2)) {
                ev->type = -2;
                rearm(r, r->start_abs + r->n_hdr * r->demod.symbol_len);
                return 1;
            }
            r->use_ldpc = r->hdr.ver == 2;
            r->mu = r->hdr.mod == 2 ? 4 : (r->hdr.mod == 1 ? 2 : 1);
            r->cap = N_DATA_CARRIERS * r->mu;
            {
                int coded_len;
                r->pkt_bits_count = PKT_BITS_FROM_HDR(r->hdr.typ, r->hdr.len);
                coded_len = r->use_ldpc
                                    ? ldpc_cc_elements(r->pkt_bits_count)
                                    : conv_cc_elements((cc_rate_t)r->hdr.spd,
                                                       r->pkt_bits_count);
                r->n_data = (coded_len + r->cap - 1) / r->cap;
            }
            /* A frame the buffers cannot hold must be REFUSED here, not
             * discovered later: nothing downstream checks, so an
             * oversized header walks g_d64[inst] straight into the
             * neighbouring instance's rows and g_dexps past its row --
             * and decodes garbage that fails CRC in a way that looks
             * like a bad channel. Found on the two-board stand: the
             * radio firmware shipped with -DMAX_LLRS=1024 (copied from
             * the small-frame bench builds) and every burst engage with
             * frag_size >= 100 produced exactly that signature -- host
             * repro decodes 0/3 blocks at MAX_LLRS=1024 and 3/3 at the
             * default. The reject takes the bad-version exit: the
             * header WAS valid, but this receiver build cannot decode
             * the frame it announces, which is the same contract. */
            if (r->n_data > MAX_SYMS || r->n_data * r->cap > MAX_LLRS) {
                ev->type = -2;
                rearm(r, r->start_abs + r->n_hdr * r->demod.symbol_len);
                return 1;
            }
            r->sym_idx = 0;
            r->data_base = r->start_abs
                           + (int64_t)r->n_hdr * r->demod.symbol_len;
            r->blk_idx = 0;
            r->burst_resume_abs = 0;
            rxd_wacc_reset(&g_wacc[r->inst]);
            r->st = S_DATA;
            continue;
        }
        case S_DATA: {
            samp_t *si = g_sym_i, *sq = g_sym_q;
            int64_t pos = r->data_base
                          + (int64_t)r->sym_idx * r->demod.symbol_len;
            int win_buf[5], win_n = 0;
            const int *window = 0;
            int k;
            if (challenge(r))
                continue;
            if (r->abs_n < pos + r->demod.symbol_len)
                return 0;
            if (r->demod.last_hyp >= 0) {
                int lo = r->demod.last_hyp - 2 < 0 ? 0 : r->demod.last_hyp - 2;
                int hi = r->demod.last_hyp + 2 >= r->demod.n_words
                             ? r->demod.n_words - 1 : r->demod.last_hyp + 2;
                for (k = lo; k <= hi; k++)
                    win_buf[win_n++] = k;
                window = win_buf;
            }
            if (ring_copy(r, pos, r->demod.symbol_len, si, sq) != 0) {
                /* abandon the frame rather than decode stale ring
                 * contents into a CRC failure that looks like noise */
                rearm(r, pos + r->demod.symbol_len);
                continue;
            }
            {
                int64_t rho[N_DATA_CARRIERS];
                g_dexps[r->inst][r->sym_idx] = rxd_demod_symbol(
                    &r->demod, si, sq, (int)pos, r->cfo_word, r->mu, window,
                    win_n, g_d64[r->inst] + r->sym_idx * r->cap, rho);
                rxd_wacc_add(&g_wacc[r->inst], rho, g_dexps[r->inst][r->sym_idx]);
            }
            r->sym_idx++;
            if (r->sym_idx < r->n_data)
                continue;
            {
                int rc = finish_frame(r, ev);
                int64_t end = r->data_base
                              + (int64_t)r->n_data * r->demod.symbol_len;
                /* remember where this block ended: if the caller
                 * recognises the frame as part of a streamed burst it
                 * calls rxs_continue_burst() before pushing more samples,
                 * and the next block is decoded from here without
                 * re-detecting a preamble */
#ifdef RXS_TRACE
                fprintf(stderr, "[rxs] block done: blk_idx %d base %lld "
                        "end %lld rc %d type %d  lag %lld hwm %lld/%d%s\n",
                        r->blk_idx, (long long)r->data_base, (long long)end,
                        rc, ev->type, (long long)(r->abs_n - r->data_base),
                        (long long)r->ring_hwm, RXS_RAW_RING_LEN,
                        r->ring_hwm > RXS_RAW_RING_LEN ? "  RING OVERRUN" : "");
#endif
                r->burst_resume_abs = end;
                rearm(r, end);
                return rc;
            }
        }
        }
    }
}

void rxs_set_active(rxs_t *r, int active)
{
    if (!r || r->active == !!active)
        return;
    r->active = !!active;
    if (r->active)
        rearm(r, r->abs_n);   /* start looking from now, not from then */
}

int rxs_active(const rxs_t *r) { return r ? r->active : 0; }

int rxs_push(rxs_t *r, const int16_t *chunk, int n, rxs_event_t *ev)
{
    int got = 0, m;
    arena_claim(ARENA_RX); /* half-duplex arena: see arena.h */
    for (m = 0; m < n; m++) {
        /* the ring write and the abs_n advance happen even when muted:
         * the ring is shared and indexed by abs_n, so an instance that
         * stopped counting would corrupt the others' history */
        if (r->abs_n > g_notch_done) {   /* first instance to see this index */
            g_raw[(int)(r->abs_n % RXS_RAW_RING_LEN)] =
                notch_bank_push(&g_bank, chunk[m]);
            g_notch_done = r->abs_n;
        }
        r->abs_n++;
        if (!r->active)
            continue;
        if ((r->abs_n % r->B) == 0) {
            block_summary(r, r->abs_n / r->B - 1);
            if (!got)
                got = advance(r, ev);
        }
    }
    if (r->active && !got)
        got = advance(r, ev);
    return got;
}

int rxs_flush(rxs_t *r, rxs_event_t *ev)
{
    static const int16_t zeros[512];
    int left = r->demod.symbol_len, got = 0;
    if (!r->active)
        return 0;
    arena_claim(ARENA_RX); /* half-duplex arena: see arena.h */
    while (left > 0 && !got) {
        int n = left > 512 ? 512 : left;
        got = rxs_push(r, zeros, n, ev);
        left -= n;
    }
    return got;
}

int rxs_continue_burst(rxs_t *r, int resync_every)
{
    int64_t base;
    arena_claim(ARENA_RX); /* half-duplex arena: see arena.h */

    if (r->burst_resume_abs <= 0)
        return 0;
#ifdef RXS_TRACE
    fprintf(stderr, "[rxs] continue: blk_idx %d -> %d  resume_abs %lld  "
            "skip_zc %d  n_data %d symlen %d\n", r->blk_idx, r->blk_idx + 1,
            (long long)r->burst_resume_abs,
            (resync_every > 0 && (r->blk_idx + 1) % resync_every == 0),
            r->n_data, r->demod.symbol_len);
#endif
    base = r->burst_resume_abs;
    r->burst_resume_abs = 0;
    r->blk_idx++;
    /* the transmitter inserts its ZC block before every resync_every-th
     * block; step over it. This receiver does not re-lock on it the way
     * the frame-at-once path does -- one more documented divergence,
     * benign here because bursts are NORMAL-only (BURST_MIN_RUNG) and an
     * open-loop NORMAL stream was measured to hold far longer than any
     * burst lasts. */
    if (resync_every > 0 && r->blk_idx % resync_every == 0)
        base += r->demod.symbol_len;
    r->data_base = base;
    r->burst_resync = resync_every;
    r->sym_idx = 0;
    rxd_wacc_reset(&g_wacc[r->inst]);
    r->st = S_DATA;
    return 1;
}
