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
#define BLK_CAP 128                      /* >= tone window blocks (120) */
#define DECLINE_BLOCKS 3
#define ZC_WIN_MAX 71000
#define STREAM_MAX_SYM (CP_LEN + 64 * FFT_BINS)
#define RXS_MAX_INST 3

typedef struct {
    int64_t band;
    int64_t dot[2][MAX_SHIFTS];
    int exp;
} blk_sum_t;

struct rxs_state {
    int inst;
    rxd_t demod;
    link_mode_t mode;
    int B, T, max_shift, thr_q10, n_mask_bins;
    int64_t word_per_bin;
    const uint8_t *mask0, *mask1;
    int tone0, tone1, total_blocks;
    int zc_win;

    int64_t abs_n; /* samples consumed so far */

    /* state machine */
    enum { S_SEARCH, S_ZC_WAIT, S_HEADER, S_DATA } st;
    int crossed, decline;
    int64_t best_metric;
    int64_t best_off_blk, min_blk;
    int best_shift;
    int64_t cs_abs, start_abs, cfo_word;
    int64_t best_eval_blk; /* when the argmax was last improved */
    int64_t cw; /* coarse tone word */
    int sym_idx, n_hdr, n_data, mu, cap, use_ldpc;
    /* streamed bursts: data_base is the first symbol of the CURRENT
     * block, so continuing a burst is a matter of stepping it on */
    int64_t data_base;
    int blk_idx, burst_resync;
    int64_t burst_resume_abs;
    int64_t last_eval_blk;
    int64_t ring_hwm; /* deepest raw lookback (incl. FIR history) */
    rxd_header_t hdr;
    int hdr_scale, data_scale;
    uint8_t hdr_bits[HEADER_BITS];
    int pkt_bits_count; /* PKT_BITS_FROM_HDR(hdr.typ, hdr.len) */
};

static struct rxs_state g_pool[RXS_MAX_INST];
static int g_pool_next;
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
static blk_sum_t g_blk[RXS_MAX_INST][BLK_CAP];
static int64_t g_h64[RXS_MAX_INST][MAX_LLRS], g_d64[RXS_MAX_INST][MAX_LLRS];
static int g_hexps[RXS_MAX_INST][MAX_SYMS], g_dexps[RXS_MAX_INST][MAX_SYMS];
/* per-call scratch (never live across rxs_push calls) */
static int64_t g_seg_i[ZC_WIN_MAX], g_seg_q[ZC_WIN_MAX];
static int64_t g_q64[MAX_LLRS];

/* analytic extraction: recompute the streaming Hilbert from the raw
 * ring, bit-identical to the former write-time FIR (zero prehistory;
 * i[n] = x[n-31] pairs with the group-delayed q[n]) */
static void ring_copy(rxs_t *r, int64_t abs_start, int n,
                      int64_t *di, int64_t *dq)
{
    int k, j;
    if (r->abs_n - abs_start + (HILBERT_TAPS_N - 1) > r->ring_hwm)
        r->ring_hwm = r->abs_n - abs_start + (HILBERT_TAPS_N - 1);
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
        dq[k] = rshift_round(acc, Q15);
        di[k] = abs >= HILBERT_DELAY
                    ? g_raw[(int)((abs - HILBERT_DELAY)
                                  % RXS_RAW_RING_LEN)]
                    : 0;
    }
}

rxs_t *rxs_open(link_mode_t mode, int calibrate)
{
    rxs_t *r = &g_pool[g_pool_next % RXS_MAX_INST];
    int inst = g_pool_next % RXS_MAX_INST;
    g_pool_next++;
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
    r->zc_win = 3 * r->T * FFT_BINS + r->demod.symbol_len + 4 * FFT_BINS
                + r->B;
    r->st = S_SEARCH;
    r->best_metric = -1;
    r->last_eval_blk = -1;
    r->ring_hwm = 0;
    return r;
}

int64_t rxs_ring_hwm(const rxs_t *r)
{
    return r->ring_hwm;
}

/* per-block summary: BFP spectrum -> band power + mask dots per shift */
static void block_summary(rxs_t *r, int64_t blk_idx)
{
    int64_t re[512], im[512], pow_[512];
    blk_sum_t *bs = &g_blk[r->inst][blk_idx & (BLK_CAP - 1)];
    int B = r->B, k, sh, exp;

    ring_copy(r, blk_idx * B, B, re, im);
    fft_bfp(re, im, B, 13, &exp);
    bs->exp = exp;
    bs->band = 0;
    for (k = 0; k < B; k++)
        pow_[k] = re[k] * re[k] + im[k] * im[k];
    for (k = 1; k < B / 2; k++)
        bs->band += pow_[k];
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
                             int *shift_out)
{
    int64_t band_al[BLK_CAP];
    int64_t off0 = blk_idx - r->total_blocks + 1;
    int e_min = 1 << 30, sh, b;
    int64_t floor_v = 0, best = -1;
    int best_sh = 0;
    int n_band_bins = r->B / 2 - 1;

    for (b = 0; b < r->total_blocks; b++) {
        int e = g_blk[r->inst][(off0 + b) & (BLK_CAP - 1)].exp;
        if (e < e_min)
            e_min = e;
    }
    for (b = 0; b < r->total_blocks; b++) {
        blk_sum_t *bs = &g_blk[r->inst][(off0 + b) & (BLK_CAP - 1)];
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
            blk_sum_t *bs = &g_blk[r->inst][(off0 + b) & (BLK_CAP - 1)];
            int s2 = 2 * (bs->exp - e_min);
            if (s2 > 62)
                s2 = 62;
            sig0 += bs->dot[0][sh] >> s2;
            band0 += band_al[b];
        }
        for (b = r->tone0; b < r->total_blocks; b++) {
            blk_sum_t *bs = &g_blk[r->inst][(off0 + b) & (BLK_CAP - 1)];
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
        }
    }
    *metric_out = best;
    *shift_out = best_sh;
}

#ifdef STREAM_DEBUG
#include <stdio.h>
#define SDBG(...) fprintf(stderr, __VA_ARGS__)
#else
#define SDBG(...)
#endif

/* commit the tone peak: coarse CFO word from shift + lag-N residual */
static void tone_commit(rxs_t *r)
{
    int seg_n = 2 * r->T * FFT_BINS;
    SDBG("tone_commit: off_blk=%lld cs=%lld shift=%d metric=%lld\n",
         (long long)r->best_off_blk, (long long)(r->best_off_blk * r->B),
         r->best_shift, (long long)r->best_metric);
    r->cs_abs = r->best_off_blk * r->B;
    r->cw = (int64_t)r->best_shift * r->word_per_bin;
    ring_copy(r, r->cs_abs, seg_n, g_seg_i, g_seg_q);
    nco_derotate(g_seg_i, g_seg_q, seg_n, r->cw, 0u, g_seg_i, g_seg_q);
    r->cw += rx_lag_n_word(g_seg_i, g_seg_q, seg_n);
    r->st = S_ZC_WAIT;
}

static void rearm(rxs_t *r, int64_t guard_abs)
{
    r->st = S_SEARCH;
    r->crossed = 0;
    r->decline = 0;
    r->best_metric = -1;
    r->min_blk = (guard_abs + r->B - 1) / r->B;
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
        for (k = 0; k < r->cap; k++)
            g_d64[r->inst][s * r->cap + k] >>= sh;
    }
    r->data_scale = 2 * e_min;

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
            ev->snr_db = (double)l2 / 16.0 * TEN_LOG10_2
                         - rxd_tile_db(r->mode) + SNR_CAL_DB;
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
            int64_t metric;
            int shift;
            if (blk < r->total_blocks - 1 || blk < r->min_blk
                || blk == r->last_eval_blk)
                return 0;
            r->last_eval_blk = blk;
            eval_tone_window(r, blk, &metric, &shift);
            /* partial tone overlap already crosses the threshold ~a full
             * window before the true peak, so a decline-from-best rule
             * commits too early (measured). Instead: track the argmax over
             * the whole contiguous above-threshold REGION and commit when
             * the metric falls back below threshold -- causal, and the
             * true (fully-aligned) window is guaranteed to have been
             * evaluated. */
            {
                int64_t off = blk - r->total_blocks + 1;
                if (metric > (int64_t)r->thr_q10 * r->thr_q10
                    && off >= r->min_blk) {
                    r->crossed = 1;
                    r->decline = 0;
                    if (metric > r->best_metric) {
                        r->best_metric = metric;
                        r->best_off_blk = off;
                        r->best_shift = shift;
                        r->best_eval_blk = blk;
                    }
                } else if (r->crossed) {
                    r->decline++;
                }
            }
            /* commit when the above-threshold region ends, OR when the
             * argmax has been stable for a full window span of newer
             * evaluations (quiet channels: data symbols can hover at the
             * threshold so the region never cleanly ends, but the aligned
             * tone peak scores far above them and stays the argmax) */
            if (r->crossed
                && (r->decline >= DECLINE_BLOCKS
                    || blk - r->best_eval_blk
                           >= r->total_blocks + DECLINE_BLOCKS)) {
                tone_commit(r);
                continue;
            }
            return 0;
        }
        case S_ZC_WAIT: {
            int ft;
            int64_t fw;
            if (r->abs_n < r->cs_abs + r->zc_win)
                return 0;
            ring_copy(r, r->cs_abs, r->zc_win, g_seg_i, g_seg_q);
            nco_derotate(g_seg_i, g_seg_q, r->zc_win, r->cw, 0u,
                         g_seg_i, g_seg_q);
            if (rx_detect_zc_window(r->mode, g_seg_i, g_seg_q, r->zc_win,
                                    &ft, &fw) != 0) {
                SDBG("zc: no lock at cs=%lld\n", (long long)r->cs_abs);
                rearm(r, r->cs_abs + r->B); /* false tone hit */
                continue;
            }
            SDBG("zc: ft=%d fw=%lld -> start=%lld\n", ft, (long long)fw,
                 (long long)(r->cs_abs + ft));
            r->start_abs = r->cs_abs + ft;
            r->cfo_word = r->cw + fw;
            r->n_hdr = (conv_cc_elements(CC_R13, HEADER_BITS)
                        + N_DATA_CARRIERS - 1) / N_DATA_CARRIERS;
            r->sym_idx = 0;
            r->demod.last_hyp = -1;
            r->st = S_HEADER;
            continue;
        }
        case S_HEADER: {
            static int64_t si[STREAM_MAX_SYM], sq[STREAM_MAX_SYM];
            int64_t pos = r->start_abs + (int64_t)r->sym_idx
                          * r->demod.symbol_len;
            int win_buf[5], win_n = 0;
            const int *window = 0;
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
            ring_copy(r, pos, r->demod.symbol_len, si, sq);
            g_hexps[r->inst][r->sym_idx] = rxd_demod_symbol(
                &r->demod, si, sq, (int)pos, r->cfo_word, 1, window, win_n,
                g_h64[r->inst] + r->sym_idx * N_DATA_CARRIERS);
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
                    for (k = 0; k < N_DATA_CARRIERS; k++)
                        g_h64[r->inst][s * N_DATA_CARRIERS + k] >>= sh;
                }
                r->hdr_scale = 2 * e_min;
            }
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
            r->sym_idx = 0;
            r->data_base = r->start_abs
                           + (int64_t)r->n_hdr * r->demod.symbol_len;
            r->blk_idx = 0;
            r->burst_resume_abs = 0;
            r->st = S_DATA;
            continue;
        }
        case S_DATA: {
            static int64_t si[STREAM_MAX_SYM], sq[STREAM_MAX_SYM];
            int64_t pos = r->data_base
                          + (int64_t)r->sym_idx * r->demod.symbol_len;
            int win_buf[5], win_n = 0;
            const int *window = 0;
            int k;
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
            ring_copy(r, pos, r->demod.symbol_len, si, sq);
            g_dexps[r->inst][r->sym_idx] = rxd_demod_symbol(
                &r->demod, si, sq, (int)pos, r->cfo_word, r->mu, window,
                win_n, g_d64[r->inst] + r->sym_idx * r->cap);
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

int rxs_push(rxs_t *r, const int16_t *chunk, int n, rxs_event_t *ev)
{
    int got = 0, m;
    for (m = 0; m < n; m++) {
        g_raw[(int)(r->abs_n % RXS_RAW_RING_LEN)] = chunk[m];
        r->abs_n++;
        if ((r->abs_n % r->B) == 0) {
            block_summary(r, r->abs_n / r->B - 1);
            if (!got)
                got = advance(r, ev);
        }
    }
    if (!got)
        got = advance(r, ev);
    return got;
}

int rxs_flush(rxs_t *r, rxs_event_t *ev)
{
    static const int16_t zeros[512];
    int left = r->demod.symbol_len, got = 0;
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
    r->st = S_DATA;
    return 1;
}
