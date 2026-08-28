#include <string.h>

#include "rx_detect.h"
#include "fxp.h"
#include "fft.h"
#include "dsp.h"
#include "rom_tables.h"
#include "rom_modes.h"
#include "rx_internal.h"

#define DET_B_MAX 512
#define MAX_BLOCKS 2200          /* RXD_MAX_SAMPLES / 256 */
#define ZC_WIN_MAX 71000         /* detect window, EXTREME worst case */
#define ZC_KLEN_MAX (4 * FFT_BINS)
/* cc[m] gathers mag[m + g*klen] for g < ng, so the scan needs
 * (ng-1)*klen+1 magnitudes resident: EXTREME 15*512+1 = 7681. */
#define ZC_MAG_RING (15 * ZC_KLEN_MAX + 1)
#define ZC_PREAMBLE_MAX (64 * FFT_BINS)   /* zc_L, EXTREME */
#define LAG_CHUNK 512
#define LAG_HIST FFT_BINS
#define ZC_SLIDE_BLK 2048
#define ZC_SLIDE_W (ZC_PREAMBLE_MAX + ZC_SLIDE_BLK)

/* carved from the shared arena -- see rx_internal.h for why this is
 * safe; the offsets below must stay within RX_ARENA_I64 */
#define DET_OFF_WI   0
#define DET_OFF_WQ   (DET_OFF_WI + ZC_SLIDE_W)
#define DET_OFF_MAG  (DET_OFF_WQ + ZC_SLIDE_W)
#define DET_OFF_KR   (DET_OFF_MAG + ZC_MAG_RING)
#define DET_OFF_KI   (DET_OFF_KR + ZC_KLEN_MAX)
#define DET_OFF_RRE  (DET_OFF_KI + ZC_KLEN_MAX)
#define DET_OFF_RIM  (DET_OFF_RRE + ZC_KLEN_MAX)
#define DET_OFF_BI   (DET_OFF_RIM + ZC_KLEN_MAX)
#define DET_OFF_BQ   (DET_OFF_BI + LAG_CHUNK + LAG_HIST)
#define DET_OFF_END  (DET_OFF_BQ + LAG_CHUNK + LAG_HIST)
typedef char det_arena_fits[DET_OFF_END <= RX_ARENA_I64 ? 1 : -1];

typedef struct {
    int B, T, max_shift, thr_q10, n_mask_bins;
    int64_t word_per_bin;
    const uint8_t *mask0, *mask1;
    int zc_L, zc_G, zc_groups, zc_pre_block_len, zc_thr_q10;
    int64_t zc_ref_energy;
    int n_zc_hyps;
    const int32_t *zc_hyp_words;
    const int16_t *zc_rom_re, *zc_rom_im;
    int sym_tile;
} det_mode_t;

#define DET_ENTRY(NM, TILE)                                                  \
    { DET_B_##NM, DET_T_##NM, DET_MAX_SHIFT_##NM, DET_NEWMAN_THR_Q10_##NM,   \
      DET_N_MASK_BINS_##NM, DET_WORD_PER_BIN_##NM, DET_MASK0_##NM,           \
      DET_MASK1_##NM, ZC_L_##NM, ZC_G_##NM, ZC_GROUPS_##NM,                  \
      ZC_PRE_BLOCK_LEN_##NM, ZC_THR_Q10_##NM, ZC_REF_ENERGY_##NM,            \
      N_ZC_HYPS_##NM, ZC_HYP_WORDS_##NM, ZC_ROM_RE_##NM, ZC_ROM_IM_##NM,     \
      TILE }

static const det_mode_t DET[3] = {
    DET_ENTRY(NORMAL, SYM_TILE_NORMAL),
    DET_ENTRY(ROBUST, SYM_TILE_ROBUST),
    DET_ENTRY(EXTREME, SYM_TILE_EXTREME),
};

static int64_t div_round_signed(int64_t a, int64_t b)
{
    return a >= 0 ? (a + b / 2) / b : -((-a + b / 2) / b);
}

/* phase estimate of a derotated segment at an arbitrary lag */
static int64_t lag_word(const int64_t *di, const int64_t *dq, int n, int lag)
{
    int64_t rr = 0, ri = 0, ang, mag;
    int k;
    for (k = 0; k < n - lag; k++) {
        rr += di[k] * di[k + lag] + dq[k] * dq[k + lag];
        ri += di[k] * dq[k + lag] - dq[k] * di[k + lag];
    }
    cordic_atan2(ri, rr, &ang, &mag);
    return div_round_signed(ang, lag);
}

/* lag-N phase estimate of a derotated segment -> residual phase word */

/* Both lag correlations in ONE pass over a pulled source.
 *
 * The lags are FFT_BINS and FFT_BINS/2 -- 128 and 64 -- not a fraction
 * of the segment, so the correlation never needs more than a 128-sample
 * delay line however long the segment is. Materialising the segment for
 * it cost 640 kB at EXTREME to hold 40960 samples that are read exactly
 * twice each.
 *
 * Accumulates the same terms in the same order as lag_word(), so the
 * cordic input is identical and the result is bit-exact. */

static void lag_words_src(const zc_src_t *src, int n, int want_coarse,
                          int64_t *fine_out, int64_t *coarse_out)
{
    int64_t *const bi = rx_arena + DET_OFF_BI;
    int64_t *const bq = rx_arena + DET_OFF_BQ;
    int64_t rr1 = 0, ri1 = 0, rr2 = 0, ri2 = 0, ang, mag;
    const int L1 = FFT_BINS, L2 = FFT_BINS / 2;
    int pos = 0, hist = 0;

    while (pos + hist < n) {
        int want = LAG_CHUNK;
        int total, j, j0;
        if (pos + hist + want > n)
            want = n - pos - hist;
        src->fetch(src->ctx, pos + hist, want, bi + hist, bq + hist);
        total = hist + want;
        /* absolute index a = pos + j; a-L1 must still be in the buffer */
        j0 = pos < L1 ? L1 - pos : L1;
        for (j = j0; j < total; j++) {
            rr1 += bi[j - L1] * bi[j] + bq[j - L1] * bq[j];
            ri1 += bi[j - L1] * bq[j] - bq[j - L1] * bi[j];
            if (want_coarse) {
                rr2 += bi[j - L2] * bi[j] + bq[j - L2] * bq[j];
                ri2 += bi[j - L2] * bq[j] - bq[j - L2] * bi[j];
            }
        }
        if (total > LAG_HIST) {
            memmove(bi, bi + total - LAG_HIST,
                    (size_t)LAG_HIST * sizeof(int64_t));
            memmove(bq, bq + total - LAG_HIST,
                    (size_t)LAG_HIST * sizeof(int64_t));
            pos += total - LAG_HIST;
            hist = LAG_HIST;
        } else {
            hist = total;
        }
    }
    cordic_atan2(ri1, rr1, &ang, &mag);
    *fine_out = div_round_signed(ang, L1);
    if (want_coarse) {
        cordic_atan2(ri2, rr2, &ang, &mag);
        *coarse_out = div_round_signed(ang, L2);
    }
}

/* array + derotation, and source + derotation: the phase at index k is
 * k*word, so a fetch can start anywhere */
typedef struct {
    const int64_t *i_arr, *q_arr;
    int off;
    int64_t w;
} zc_rot_ctx_t;

static void zc_rot_fetch(void *ctx, int k, int n, int64_t *di, int64_t *dq)
{
    const zc_rot_ctx_t *a = (const zc_rot_ctx_t *)ctx;
    memcpy(di, a->i_arr + a->off + k, (size_t)n * sizeof(int64_t));
    memcpy(dq, a->q_arr + a->off + k, (size_t)n * sizeof(int64_t));
    nco_derotate(di, dq, n, a->w,
                 (uint32_t)((int64_t)(uint32_t)a->w * k), di, dq);
}

typedef struct {
    const zc_src_t *inner;
    int off;
    int64_t w;
} zc_rot2_ctx_t;

static void zc_rot2_fetch(void *ctx, int k, int n, int64_t *di, int64_t *dq)
{
    const zc_rot2_ctx_t *a = (const zc_rot2_ctx_t *)ctx;
    a->inner->fetch(a->inner->ctx, a->off + k, n, di, dq);
    nco_derotate(di, dq, n, a->w,
                 (uint32_t)((int64_t)(uint32_t)a->w * k), di, dq);
}

int64_t rx_lag_n_word_src(const zc_src_t *src, int n)
{
    int64_t fine = 0, coarse = 0;
    lag_words_src(src, n, 0, &fine, &coarse);
    return fine;
}

int64_t rx_residual_word_src(const zc_src_t *src, int n)
{
    int64_t fine = 0, coarse = 0, ambig = (int64_t)1 << PHASE_BITS, k;
    lag_words_src(src, n, 1, &fine, &coarse);
    ambig /= FFT_BINS;
    k = div_round_signed(coarse - fine, ambig);
    return fine + k * ambig;
}

int64_t rx_lag_n_word(const int64_t *di, const int64_t *dq, int n)
{
    return lag_word(di, dq, n, FFT_BINS);
}

/* Residual CFO with the lag-N cycle resolved by a shorter lag.
 *
 * The lag-N phase is precise but wraps beyond +-fs/2N = +-46.9 Hz, which
 * is exactly one coarse bin -- and the coarse mask-shift search is a
 * near-tie between adjacent bins (measured 4% apart), so noise picks the
 * neighbour often enough to matter. When it does, the residual must
 * supply more than 46.9 Hz, wraps, and the total lands a full bin
 * (93.75 Hz) out. That cyclically shifts the Zadoff-Chu correlation by
 * ~34 samples, past the 32-sample cyclic prefix, and the frame dies of
 * ISI: measured at ~8% of ALL acquisitions before this. The lag-N/2
 * phase is unambiguous over +-fs/N, wide enough to cover a one-bin
 * coarse error, so it picks the right cycle. (The float chain does the
 * same job with a full-block FFT peak; this is the integer-cheap twin.)
 * Keep this in step with ofdm_phy/fixed/rx.py::_detect_newman. */

static int64_t g_pows[MAX_BLOCKS * DET_B_MAX];
/* No detection scratch here any more: the ZC scan pulls through
 * zc_src_t and both lag correlations run off a 128-sample delay line, so
 * neither the search window nor the coarse segment is ever resident. */


static int cmp_i64(const void *a, const void *b)
{
    int64_t x = *(const int64_t *)a, y = *(const int64_t *)b;
    return x < y ? -1 : (x > y ? 1 : 0);
}

#include <stdlib.h> /* qsort for the median (host reference) */

/* tone stage: returns 0 + (sample_index, cfo_word), -1 on no lock */
static int detect_newman(const det_mode_t *d, const int64_t *i_arr,
                         const int64_t *q_arr, int n, int *start_out,
                         int64_t *word_out)
{
    static int64_t block_band[MAX_BLOCKS], band0[MAX_BLOCKS], band1[MAX_BLOCKS];
    static int64_t sorted[MAX_BLOCKS];
    int B = d->B, T = d->T;
    int num_blocks = n / B;
    int tone0 = 2 * T * FFT_BINS / B;
    int tone1 = T * FFT_BINS / B;
    int total = tone0 + tone1;
    int n_off, b, k, sh, exp;
    int exps[MAX_BLOCKS], e_min;
    int64_t floor_v, best_metric = -1;
    int best_off = 0, best_shift = 0;

    if (num_blocks < total || num_blocks > MAX_BLOCKS)
        return -1;

    for (b = 0; b < num_blocks; b++) {
        int64_t re[DET_B_MAX], im[DET_B_MAX];
        memcpy(re, i_arr + b * B, sizeof(int64_t) * (size_t)B);
        memcpy(im, q_arr + b * B, sizeof(int64_t) * (size_t)B);
        fft_bfp(re, im, B, 13, &exp);
        exps[b] = exp;
        for (k = 0; k < B; k++)
            g_pows[b * B + k] = re[k] * re[k] + im[k] * im[k];
    }
    e_min = exps[0];
    for (b = 1; b < num_blocks; b++)
        if (exps[b] < e_min)
            e_min = exps[b];
    for (b = 0; b < num_blocks; b++) {
        int s2 = 2 * (exps[b] - e_min);
        if (s2 > 62)
            s2 = 62;
        for (k = 0; k < B; k++)
            g_pows[b * B + k] >>= s2;
    }

    /* per-block in-band power + median floor (int(np.median): the sum of
     * two mid elements halves exactly in floor arithmetic for nonneg) */
    for (b = 0; b < num_blocks; b++) {
        int64_t s = 0;
        for (k = 1; k < B / 2; k++)
            s += g_pows[b * B + k];
        block_band[b] = s;
        sorted[b] = s;
    }
    qsort(sorted, (size_t)num_blocks, sizeof(int64_t), cmp_i64);
    floor_v = (num_blocks & 1)
                  ? sorted[num_blocks / 2]
                  : (sorted[num_blocks / 2 - 1] + sorted[num_blocks / 2]) / 2;

    n_off = num_blocks - total + 1;
    /* prefix sums of per-block band power for the window band totals */
    {
        static int64_t pre[MAX_BLOCKS + 1];
        pre[0] = 0;
        for (b = 0; b < num_blocks; b++)
            pre[b + 1] = pre[b] + block_band[b];
        for (k = 0; k < n_off; k++) {
            band0[k] = pre[k + tone0] - pre[k];
            band1[k] = pre[k + tone0 + tone1] - pre[k + tone0];
        }
    }

    for (sh = -d->max_shift; sh <= d->max_shift; sh++) {
        static int64_t dot0[MAX_BLOCKS], dot1[MAX_BLOCKS];
        static int64_t p0[MAX_BLOCKS + 1], p1[MAX_BLOCKS + 1];
        int n_band_bins = B / 2 - 1;
        for (b = 0; b < num_blocks; b++) {
            int64_t s0 = 0, s1 = 0;
            for (k = 0; k < B; k++) {
                int src = k - sh; /* rolled mask: m[k] = mask[(k - sh) mod B] */
                src &= B - 1;
                if (d->mask0[src])
                    s0 += g_pows[b * B + k];
                if (d->mask1[src])
                    s1 += g_pows[b * B + k];
            }
            dot0[b] = s0;
            dot1[b] = s1;
        }
        p0[0] = p1[0] = 0;
        for (b = 0; b < num_blocks; b++) {
            p0[b + 1] = p0[b] + dot0[b];
            p1[b + 1] = p1[b] + dot1[b];
        }
        for (k = 0; k < n_off; k++) {
            int64_t sig0 = p0[k + tone0] - p0[k];
            int64_t sig1 = p1[k + tone0 + tone1] - p1[k + tone0];
            int64_t rest0 = band0[k] - sig0;
            int64_t rest1 = band1[k] - sig1;
            int64_t c0, c1, metric;
            if (rest0 < 1)
                rest0 = 1;
            if (rest1 < 1)
                rest1 = 1;
            rest0 += (floor_v * tone0) / 100;
            rest1 += (floor_v * tone1) / 100;
            c0 = (sig0 * n_band_bins * 1024) / (rest0 * d->n_mask_bins);
            c1 = (sig1 * n_band_bins * 1024) / (rest1 * d->n_mask_bins);
            metric = c0 * c1;
            if (metric > best_metric) {
                best_metric = metric;
                best_off = k;
                best_shift = sh;
            }
        }
    }

    if (best_metric < (int64_t)d->thr_q10 * d->thr_q10)
        return -1;

    {
        int sample_index = best_off * B;
        int64_t coarse_word = (int64_t)best_shift * d->word_per_bin;
        int seg_n = 2 * T * FFT_BINS;
        zc_rot_ctx_t rc;
        zc_src_t rs;
        rc.i_arr = i_arr;
        rc.q_arr = q_arr;
        rc.off = sample_index;
        rc.w = coarse_word;   /* python derotates the slice from phase 0 */
        rs.ctx = &rc;
        rs.fetch = zc_rot_fetch;
        *start_out = sample_index;
        *word_out = coarse_word + rx_residual_word_src(&rs, seg_n);
    }
    return 0;
}

/* ZC stage on a (derotated) window: fine timing + CFO */

/* Bounded slice of the scan window.
 *
 * The scan's live span is preamble_len+1 samples however long the search
 * is (see zc_src_t), so this holds that plus a slide block and refills as
 * the scan advances. Each sample is fetched exactly once per pass: the
 * retained tail is moved down rather than re-read. 164 kB at EXTREME,
 * against 1.1 MB to materialise the window. */

static int64_t *const g_wi = rx_arena + DET_OFF_WI;
static int64_t *const g_wq = rx_arena + DET_OFF_WQ;
static int g_wbase, g_wfill;

static void win_open(const zc_src_t *src, int n)
{
    g_wbase = 0;
    g_wfill = n < ZC_SLIDE_W ? n : ZC_SLIDE_W;
    if (g_wfill > 0)
        src->fetch(src->ctx, 0, g_wfill, g_wi, g_wq);
}

/* make absolute index hi readable, keeping at least `back` behind it */
static void win_need(const zc_src_t *src, int n, int hi, int back)
{
    int new_base, shift, add;

    if (hi < g_wbase + g_wfill || g_wbase + g_wfill >= n)
        return;
    new_base = hi - back;
    if (new_base < 0)
        new_base = 0;
    if (new_base < g_wbase)
        new_base = g_wbase;
    shift = new_base - g_wbase;
    if (shift > 0) {
        int keep = g_wfill - shift;
        if (keep < 0)
            keep = 0;
        memmove(g_wi, g_wi + shift, (size_t)keep * sizeof(int64_t));
        memmove(g_wq, g_wq + shift, (size_t)keep * sizeof(int64_t));
        g_wbase = new_base;
        g_wfill = keep;
    }
    add = ZC_SLIDE_W - g_wfill;
    if (g_wbase + g_wfill + add > n)
        add = n - g_wbase - g_wfill;
    if (add > 0) {
        src->fetch(src->ctx, g_wbase + g_wfill, add,
                   g_wi + g_wfill, g_wq + g_wfill);
        g_wfill += add;
    }
}

#define WI(a) g_wi[(a) - g_wbase]
#define WQ(a) g_wq[(a) - g_wbase]

static int detect_zc(const det_mode_t *d, const zc_src_t *src,
                     int n, int *time_out,
                     int64_t *word_out)
{
    /* Streaming ZC scan: memory is a function of the PREAMBLE, not of the
     * search window. The window is 70688 samples at EXTREME (5.9 s of
     * audio) purely because 64x tiling stretches the tone stage's timing
     * uncertainty; the information being extracted is a 512-sample kernel
     * and a running peak.
     *
     *   ecs[] (a prefix sum of energy) is only ever read as
     *     ecs[m+preamble_len] - ecs[m] -- a sliding window sum, and
     *     i_arr/q_arr are still here, so it is a running scalar.
     *   cc[] is consumed immediately by the argmax; only cc at the
     *     winning index and the total survive the loop.
     *   mag[] genuinely needs a window, because cc[m] gathers
     *     mag[m + g*klen] for g < ng -- but that is (ng-1)*klen+1
     *     entries, not the whole span. */
    int64_t *const mag = rx_arena + DET_OFF_MAG;
    int64_t *const kr = rx_arena + DET_OFF_KR;
    int64_t *const ki = rx_arena + DET_OFF_KI;
    int64_t *const rom_re = rx_arena + DET_OFF_RRE;
    int64_t *const rom_im = rx_arena + DET_OFF_RIM;
    int klen = d->zc_G * FFT_BINS;
    int ng = d->zc_groups;
    int preamble_len = d->zc_L * FFT_BINS;
    int h, m, g, k;
    int64_t best_metric = -1, best_w = 0;
    int best_idx = 0;

    if (n < preamble_len + CP_LEN)
        return -1;

    for (k = 0; k < klen; k++) {
        rom_re[k] = d->zc_rom_re[k];
        rom_im[k] = d->zc_rom_im[k];
    }

    for (h = 0; h < d->n_zc_hyps; h++) {
        int64_t w = d->zc_hyp_words[h];
        int n_corr = n - klen + 1;
        int n_cc = n_corr - (ng - 1) * klen;
        int n_pos = n - preamble_len + 1;
        int64_t csum = 0, floor_v, ptf_q4, thr, metric2;
        int j;

        int span = (ng - 1) * klen;   /* mag lookahead cc[m] needs */
        int64_t we = 0, cc_j = 0, bm = -1;
        int p;

        nco_derotate(rom_re, rom_im, klen, -w, 0u, kr, ki);

        if (n_pos > n_cc)
            n_pos = n_cc;

        /* energy of the first candidate window, [0, preamble_len) */
        win_open(src, n);
        win_need(src, n, preamble_len - 1, preamble_len);
        for (k = 0; k < preamble_len; k++)
            we += WI(k) * WI(k) + WQ(k) * WQ(k);

        j = 0;
        for (p = 0; p < n_corr; p++) {
            int64_t a = 0, b2 = 0, aa, bb;
            win_need(src, n, p + klen < n ? p + klen : n - 1, preamble_len);
            for (k = 0; k < klen; k++) {
                a += WI(p + k) * kr[k] + WQ(p + k) * ki[k];
                b2 += WQ(p + k) * kr[k] - WI(p + k) * ki[k];
            }
            a = rshift_round(a, Q15);
            b2 = rshift_round(b2, Q15);
            aa = a < 0 ? -a : a;
            bb = b2 < 0 ? -b2 : b2;
            mag[p % ZC_MAG_RING] =
                (aa > bb ? aa : bb) + ((aa < bb ? aa : bb) >> 1);

            /* mag[m .. m+span] are now resident, so cc[m] is complete */
            m = p - span;
            if (m < 0 || m >= n_cc)
                continue;
            {
                int64_t sc = 0;
                for (g = 0; g < ng; g++)
                    sc += mag[(m + g * klen) % ZC_MAG_RING];
                csum += sc;
                if (m < n_pos) {
                    int64_t nm = (sc >> 5) * (sc >> 5);
                    int64_t dn = (((we >> 8)
                                   * ((ng * d->zc_ref_energy) >> 12)) >> 15);
                    int64_t m2;
                    if (dn < 1)
                        dn = 1;
                    m2 = (nm << 10) / dn;
                    if (m2 > bm) {
                        bm = m2;
                        j = m;
                        cc_j = sc;    /* only the winner survives the loop */
                    }
                    /* slide the energy window to [m+1, m+1+preamble_len) */
                    if (m + 1 < n_pos)
                        we += WI(m + preamble_len) * WI(m + preamble_len)
                              + WQ(m + preamble_len) * WQ(m + preamble_len)
                              - WI(m) * WI(m) - WQ(m) * WQ(m);
                }
            }
        }
        metric2 = bm;

        /* floor = int(np.mean(cc)) + 1: mirror the double division */
        floor_v = (int64_t)((double)csum / (double)n_cc) + 1;
        ptf_q4 = (cc_j << 4) / floor_v;
        thr = d->zc_thr_q10;
        if (ptf_q4 < 56) {
            int64_t t2 = thr + (thr * (56 - ptf_q4)) / 32;
            int64_t cap = (thr * 9) / 4;
            thr = t2 < cap ? t2 : cap;
        }
        if (metric2 >= thr * thr && metric2 > best_metric) {
            best_metric = metric2;
            best_idx = j;
            best_w = w;
        }
    }

    if (best_metric < 0)
        return -1;

    /* revisit the winner by pulling its preamble again, derotated by the
     * winning hypothesis -- nothing of the scan was kept */
    {
        zc_rot2_ctx_t rc;
        zc_src_t rs;
        rc.inner = src;
        rc.off = best_idx;
        rc.w = best_w;
        rs.ctx = &rc;
        rs.fetch = zc_rot2_fetch;
        *word_out = best_w + rx_lag_n_word_src(&rs, preamble_len);
    }
    *time_out = (best_idx - CP_LEN) + d->zc_pre_block_len;
    return 0;
}

/* array-backed source, so callers that already hold the window are
 * unchanged (the frame-at-once path, and the golden-vector tests) */
typedef struct {
    const int64_t *i_arr, *q_arr;
} zc_arr_ctx_t;

static void zc_arr_fetch(void *ctx, int k, int n, int64_t *di, int64_t *dq)
{
    const zc_arr_ctx_t *a = (const zc_arr_ctx_t *)ctx;
    memcpy(di, a->i_arr + k, (size_t)n * sizeof(int64_t));
    memcpy(dq, a->q_arr + k, (size_t)n * sizeof(int64_t));
}

int rx_detect(link_mode_t mode, const int64_t *i_arr, const int64_t *q_arr,
              int n, int *start, int64_t *cfo_word)
{
    const det_mode_t *d = &DET[mode];
    int cs, ft, win;
    int64_t cw, fw;

    if (detect_newman(d, i_arr, q_arr, n, &cs, &cw) != 0)
        return -1;

    win = 3 * d->T * FFT_BINS + (CP_LEN + d->sym_tile * FFT_BINS)
          + 4 * FFT_BINS + d->B;
    if (cs + win > n)
        win = n - cs;
    {
        static int64_t wi[ZC_WIN_MAX], wq[ZC_WIN_MAX];
        uint32_t sp = 0u; /* python derotates the slice from phase 0 */
        nco_derotate(i_arr + cs, q_arr + cs, win, cw, sp, wi, wq);
        {
            zc_arr_ctx_t a;
            zc_src_t src;
            a.i_arr = wi;
            a.q_arr = wq;
            src.ctx = &a;
            src.fetch = zc_arr_fetch;
            if (detect_zc(d, &src, win, &ft, &fw) != 0)
                return -1;
        }
        if (0)
            return -1;
    }
    *start = cs + ft;
    *cfo_word = cw + fw;
    return 0;
}

int rx_detect_zc_window(link_mode_t mode, const int64_t *i_arr,
                        const int64_t *q_arr, int n, int *time_out,
                        int64_t *word_out)
{
    zc_arr_ctx_t a;
    zc_src_t src;
    a.i_arr = i_arr;
    a.q_arr = q_arr;
    src.ctx = &a;
    src.fetch = zc_arr_fetch;
    return detect_zc(&DET[mode], &src, n, time_out, word_out);
}

int rx_detect_zc_src(link_mode_t mode, const zc_src_t *src, int n,
                     int *time_out, int64_t *word_out)
{
    return detect_zc(&DET[mode], src, n, time_out, word_out);
}
