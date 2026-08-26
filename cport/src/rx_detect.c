#include <string.h>

#include "rx_detect.h"
#include "fxp.h"
#include "fft.h"
#include "dsp.h"
#include "rom_tables.h"
#include "rom_modes.h"

#define DET_B_MAX 512
#define MAX_BLOCKS 2200          /* RXD_MAX_SAMPLES / 256 */
#define ZC_WIN_MAX 71000         /* detect window, EXTREME worst case */
#define ZC_KLEN_MAX (4 * FFT_BINS)

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
static int64_t rx_residual_word(const int64_t *di, const int64_t *dq, int n)
{
    int64_t fine = lag_word(di, dq, n, FFT_BINS);
    int64_t coarse = lag_word(di, dq, n, FFT_BINS / 2);
    int64_t ambig = (int64_t)1 << PHASE_BITS;
    int64_t k;
    ambig /= FFT_BINS;                    /* one lag-N cycle */
    k = div_round_signed(coarse - fine, ambig);
    return fine + k * ambig;
}

static int64_t g_pows[MAX_BLOCKS * DET_B_MAX];
static int64_t g_seg_i[ZC_WIN_MAX], g_seg_q[ZC_WIN_MAX];

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
        nco_derotate(i_arr + sample_index, q_arr + sample_index, seg_n,
                     coarse_word,
                     0u /* python derotates the slice from phase 0 */,
                     g_seg_i, g_seg_q);
        *start_out = sample_index;
        *word_out = coarse_word + rx_residual_word(g_seg_i, g_seg_q, seg_n);
    }
    return 0;
}

/* ZC stage on a (derotated) window: fine timing + CFO */
static int detect_zc(const det_mode_t *d, const int64_t *i_arr,
                     const int64_t *q_arr, int n, int *time_out,
                     int64_t *word_out)
{
    static int64_t ecs[ZC_WIN_MAX + 1];
    static int64_t mag[ZC_WIN_MAX], cc[ZC_WIN_MAX];
    static int64_t kr[ZC_KLEN_MAX], ki[ZC_KLEN_MAX];
    static int64_t rom_re[ZC_KLEN_MAX], rom_im[ZC_KLEN_MAX];
    int klen = d->zc_G * FFT_BINS;
    int ng = d->zc_groups;
    int preamble_len = d->zc_L * FFT_BINS;
    int h, m, g, k;
    int64_t best_metric = -1, best_w = 0;
    int best_idx = 0;

    if (n < preamble_len + CP_LEN)
        return -1;

    ecs[0] = 0;
    for (k = 0; k < n; k++)
        ecs[k + 1] = ecs[k] + i_arr[k] * i_arr[k] + q_arr[k] * q_arr[k];

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

        nco_derotate(rom_re, rom_im, klen, -w, 0u, kr, ki);

        for (m = 0; m < n_corr; m++) {
            int64_t a = 0, b2 = 0, aa, bb;
            for (k = 0; k < klen; k++) {
                a += i_arr[m + k] * kr[k] + q_arr[m + k] * ki[k];
                b2 += q_arr[m + k] * kr[k] - i_arr[m + k] * ki[k];
            }
            a = rshift_round(a, Q15);
            b2 = rshift_round(b2, Q15);
            aa = a < 0 ? -a : a;
            bb = b2 < 0 ? -b2 : b2;
            mag[m] = (aa > bb ? aa : bb) + ((aa < bb ? aa : bb) >> 1);
        }
        for (m = 0; m < n_cc; m++) {
            int64_t s = 0;
            for (g = 0; g < ng; g++)
                s += mag[g * klen + m];
            cc[m] = s;
            csum += s;
        }
        if (n_pos > n_cc)
            n_pos = n_cc;

        j = 0;
        {
            int64_t bm = -1;
            for (m = 0; m < n_pos; m++) {
                int64_t we = ecs[preamble_len + m] - ecs[m];
                int64_t nm = (cc[m] >> 5) * (cc[m] >> 5);
                int64_t dn = (((we >> 8) * ((ng * d->zc_ref_energy) >> 12)) >> 15);
                int64_t m2;
                if (dn < 1)
                    dn = 1;
                m2 = (nm << 10) / dn;
                if (m2 > bm) {
                    bm = m2;
                    j = m;
                }
            }
            metric2 = bm;
        }

        /* floor = int(np.mean(cc)) + 1: mirror the double division */
        floor_v = (int64_t)((double)csum / (double)n_cc) + 1;
        ptf_q4 = (cc[j] << 4) / floor_v;
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

    nco_derotate(i_arr + best_idx, q_arr + best_idx, preamble_len, best_w,
                 0u, g_seg_i, g_seg_q);
    *word_out = best_w + rx_lag_n_word(g_seg_i, g_seg_q, preamble_len);
    *time_out = (best_idx - CP_LEN) + d->zc_pre_block_len;
    return 0;
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
        if (detect_zc(d, wi, wq, win, &ft, &fw) != 0)
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
    return detect_zc(&DET[mode], i_arr, q_arr, n, time_out, word_out);
}
