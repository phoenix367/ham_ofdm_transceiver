#include <string.h>

#include "rx_demod.h"
#include "rx_detect.h"
#include "rx_internal.h"
#include "ldpc.h"
#include "fxp.h"
#include "fft.h"
#include "dsp.h"
#include "bits.h"
#include "conv.h"
#include "packets.h"
#include "rom_tables.h"
#include "rom_modes.h"

#define MAX_SEARCH 275
#define MAX_WINDOW 40        /* top-3 coarse windows of 11 */
#define RXD_MAX_SAMPLES 560000

static int64_t g_i[RXD_MAX_SAMPLES], g_q[RXD_MAX_SAMPLES];

static const double TILE_DBS[3] = { TILE_DB_NORMAL, TILE_DB_ROBUST,
                                    TILE_DB_EXTREME };

double rxd_tile_db(link_mode_t mode)
{
    return TILE_DBS[mode];
}

void rxd_init(rxd_t *r, link_mode_t mode)
{
    static const struct {
        const int32_t *words;
        int n;
        int tile;
    } M[3] = {
        { SEARCH_WORDS_NORMAL, N_SEARCH_NORMAL, SYM_TILE_NORMAL },
        { SEARCH_WORDS_ROBUST, N_SEARCH_ROBUST, SYM_TILE_ROBUST },
        { SEARCH_WORDS_EXTREME, N_SEARCH_EXTREME, SYM_TILE_EXTREME },
    };
    r->mode = mode;
    r->sym_tile = M[mode].tile;
    r->symbol_len = CP_LEN + M[mode].tile * FFT_BINS;
    r->search_words = M[mode].words;
    r->n_words = M[mode].n;
    r->last_hyp = -1;
    r->coarse_enabled = 1;
    r->coarse_gate_q4 = COARSE_GATE_Q4_C;
    r->calibrate = 0;
    r->last_snr_db = 0.0;
    r->last_harq_combined = 0;
}

/* derotate [pos, pos+span) by cfo_word + words[k], accumulate ct tiles
 * after the CP, BFP-FFT; returns in-band energy, spectrum optional */
static int64_t eval_hyp(const rxd_t *r, const int64_t *si, const int64_t *sq,
                        int pos, int64_t cfo_word, int k,
                        int ct, int *exp_out, int64_t *spec_re,
                        int64_t *spec_im)
{
    static int64_t di[CP_LEN + 64 * FFT_BINS], dq[CP_LEN + 64 * FFT_BINS];
    int64_t acc_i[FFT_BINS], acc_q[FFT_BINS];
    int64_t word = cfo_word + r->search_words[k];
    uint32_t start_phase = (uint32_t)((uint64_t)(uint32_t)word * (uint64_t)pos);
    int span = CP_LEN + ct * FFT_BINS;
    int t, n, exp;
    int64_t e = 0;

    nco_derotate(si, sq, span, word, start_phase, di, dq);
    memset(acc_i, 0, sizeof(acc_i));
    memset(acc_q, 0, sizeof(acc_q));
    for (t = 0; t < ct; t++)
        for (n = 0; n < FFT_BINS; n++) {
            acc_i[n] += di[CP_LEN + t * FFT_BINS + n];
            acc_q[n] += dq[CP_LEN + t * FFT_BINS + n];
        }
    fft_bfp(acc_i, acc_q, FFT_BINS, 13, &exp);
    for (n = 0; n < N_CHANNEL; n++) {
        int ci = CHANNEL_IDX[n];
        e += acc_i[ci] * acc_i[ci] + acc_q[ci] * acc_q[ci];
    }
    *exp_out = exp;
    if (spec_re) {
        memcpy(spec_re, acc_i, sizeof(acc_i));
        memcpy(spec_im, acc_q, sizeof(acc_q));
    }
    return e;
}

/* energy comparison on a common exponent: a > b in true scale?
 * (BFP exponent counts headroom left-shifts: true = e >> 2*exp) */
static int energy_gt(int64_t ea, int xa, int64_t eb, int xb)
{
    int m = xa < xb ? xa : xb;
    int sa = 2 * (xa - m), sb = 2 * (xb - m);
    int64_t a = ea >> (sa > 62 ? 62 : sa);
    int64_t b = eb >> (sb > 62 ? 62 : sb);
    return a > b;
}

/* coarse pass of the gated two-stage search; fills window[], returns the
 * window length, or 0 = below the contrast gate (run the full grid) */
static int coarse_window(rxd_t *r, const int64_t *si, const int64_t *sq,
                         int pos, int64_t cfo_word, int *window)
{
    int64_t vals[(MAX_SEARCH + 3) / 4];
    int exps[(MAX_SEARCH + 3) / 4], ks[(MAX_SEARCH + 3) / 4];
    int64_t aligned[(MAX_SEARCH + 3) / 4];
    int order[(MAX_SEARCH + 3) / 4];
    uint8_t mark[MAX_SEARCH];
    int ct = r->sym_tile / 4 > 4 ? r->sym_tile / 4 : 4;
    int nc = 0, k, i, j, e_min = 0, wn = 0;
    int64_t median;

    for (k = 0; k < r->n_words; k += 4) {
        vals[nc] = eval_hyp(r, si, sq, pos, cfo_word, k, ct, &exps[nc], 0, 0);
        ks[nc] = k;
        nc++;
    }
    e_min = exps[0];
    for (i = 1; i < nc; i++)
        if (exps[i] < e_min)
            e_min = exps[i];
    for (i = 0; i < nc; i++) {
        int s = 2 * (exps[i] - e_min);
        aligned[i] = vals[i] >> (s > 62 ? 62 : s);
        order[i] = i;
    }
    /* sort descending by (aligned, k) -- python tuple compare on ties */
    for (i = 1; i < nc; i++) {
        int o = order[i];
        for (j = i - 1; j >= 0; j--) {
            int p = order[j];
            if (aligned[p] > aligned[o] ||
                (aligned[p] == aligned[o] && ks[p] > ks[o]))
                break;
            order[j + 1] = p;
        }
        order[j + 1] = o;
    }
    median = aligned[order[nc / 2]];
    if (median < 1)
        median = 1;
    if ((aligned[order[0]] << 4) / median < r->coarse_gate_q4)
        return 0;

    memset(mark, 0, sizeof(mark));
    for (i = 0; i < 3 && i < nc; i++) {
        int kc = ks[order[i]];
        int lo = kc - 5 < 0 ? 0 : kc - 5;
        int hi = kc + 5 >= r->n_words ? r->n_words - 1 : kc + 5;
        for (k = lo; k <= hi; k++)
            mark[k] = 1;
    }
    for (k = 0; k < r->n_words; k++)
        if (mark[k])
            window[wn++] = k;
    return wn;
}

/* one symbol: hypothesis search + channel estimate + matched-filter LLRs.
 * llr receives capacity = N_DATA_CARRIERS * mu values; returns the BFP exp */
int rxd_demod_symbol(rxd_t *r, const int64_t *seg_i, const int64_t *seg_q,
                     int pos, int64_t cfo_word, int mu,
                     const int *window, int win_n, int64_t *llr)
{
    static int win_buf[MAX_WINDOW];
    int64_t spec_re[FFT_BINS], spec_im[FFT_BINS];
    int64_t hp_re[N_PILOTS], hp_im[N_PILOTS];
    int64_t h_re[N_CHANNEL], h_im[N_CHANNEL];
    int64_t best_e = 0;
    int best_exp = 0, best_k = -1, exp, i, k;

    if (window == 0 && r->coarse_enabled && r->n_words > 9) {
        int wn = coarse_window(r, seg_i, seg_q, pos, cfo_word, win_buf);
        if (wn > 0) {
            window = win_buf;
            win_n = wn;
        }
    }

    for (i = 0; i < (window ? win_n : r->n_words); i++) {
        int64_t e;
        k = window ? window[i] : i;
        e = eval_hyp(r, seg_i, seg_q, pos, cfo_word, k, r->sym_tile, &exp, 0, 0);
        if (best_k < 0 || energy_gt(e, exp, best_e, best_exp)) {
            best_e = e;
            best_exp = exp;
            best_k = k;
        }
    }
    r->last_hyp = best_k;
    eval_hyp(r, seg_i, seg_q, pos, cfo_word, best_k, r->sym_tile, &exp, spec_re, spec_im);

    for (k = 0; k < N_PILOTS; k++) {
        int64_t yr = spec_re[PILOT_CARRIERS[k]];
        int64_t yi = spec_im[PILOT_CARRIERS[k]];
        hp_re[k] = rshift_round(yr * PILOT_CONJ_RE[k] - yi * PILOT_CONJ_IM[k], Q15);
        hp_im[k] = rshift_round(yr * PILOT_CONJ_IM[k] + yi * PILOT_CONJ_RE[k], Q15);
    }
    for (k = 0; k < N_CHANNEL; k++) {
        int lo = INTERP_LO[k], hi = INTERP_HI[k];
        int64_t w = INTERP_W[k];
        h_re[k] = (hp_re[lo] * (Q15_ONE - w) + hp_re[hi] * w) >> Q15;
        h_im[k] = (hp_im[lo] * (Q15_ONE - w) + hp_im[hi] * w) >> Q15;
    }

    {
        int64_t li[N_DATA_CARRIERS], lq[N_DATA_CARRIERS];
        for (k = 0; k < N_DATA_CARRIERS; k++) {
            int ci = CHANNEL_IDX[DATA_LOCAL_IDX[k]];
            int64_t yr = spec_re[ci], yi = spec_im[ci];
            int64_t hr = h_re[DATA_LOCAL_IDX[k]], hi2 = h_im[DATA_LOCAL_IDX[k]];
            li[k] = yr * hr + yi * hi2;
            lq[k] = yi * hr - yr * hi2;
        }
        if (mu == 1) {
            for (k = 0; k < N_DATA_CARRIERS; k++)
                llr[k] = li[k];
        } else if (mu == 2) {
            for (k = 0; k < N_DATA_CARRIERS; k++) {
                llr[2 * k] = li[k];
                llr[2 * k + 1] = lq[k];
            }
        } else { /* 16-QAM: sign bits + inner bits vs per-symbol amp ref */
            int64_t ssum = 0, h2sum = 0, ratio_q8;
            int64_t h2[N_DATA_CARRIERS];
            for (k = 0; k < N_DATA_CARRIERS; k++) {
                int64_t hr = h_re[DATA_LOCAL_IDX[k]], hi2 = h_im[DATA_LOCAL_IDX[k]];
                h2[k] = hr * hr + hi2 * hi2;
                ssum += (li[k] < 0 ? -li[k] : li[k]) +
                        (lq[k] < 0 ? -lq[k] : lq[k]);
                h2sum += h2[k];
            }
            if (h2sum < 1)
                h2sum = 1;
            ratio_q8 = (ssum << 8) / (2 * h2sum);
            for (k = 0; k < N_DATA_CARRIERS; k++) {
                int64_t t = (h2[k] * ratio_q8) >> 8;
                int64_t ai = li[k] < 0 ? -li[k] : li[k];
                int64_t aq = lq[k] < 0 ? -lq[k] : lq[k];
                llr[4 * k] = li[k];
                llr[4 * k + 1] = t - ai;
                llr[4 * k + 2] = lq[k];
                llr[4 * k + 3] = t - aq;
            }
        }
    }
    return exp;
}

/* n_syms symbols with the slew-limited tracker; exponent-aligned stream */
static int demod_block(rxd_t *r, int start, int64_t cfo_word, int n_syms,
                       int mu, int64_t *arr)
{
    int exps[MAX_SYMS] = { 0 }; /* n_syms >= 1 always writes exps[0] */
    int cap = N_DATA_CARRIERS * mu;
    int win_buf[5];
    int s, k, e_min;

    for (s = 0; s < n_syms; s++) {
        const int *window = 0;
        int win_n = 0;
        if (r->last_hyp >= 0) {
            int lo = r->last_hyp - 2 < 0 ? 0 : r->last_hyp - 2;
            int hi = r->last_hyp + 2 >= r->n_words ? r->n_words - 1
                                                   : r->last_hyp + 2;
            for (k = lo; k <= hi; k++)
                win_buf[win_n++] = k;
            window = win_buf;
        }
        {
            int p = start + s * r->symbol_len;
            exps[s] = rxd_demod_symbol(r, g_i + p, g_q + p, p, cfo_word, mu,
                                       window, win_n, arr + s * cap);
        }
    }
    e_min = exps[0];
    for (s = 1; s < n_syms; s++)
        if (exps[s] < e_min)
            e_min = exps[s];
    for (s = 0; s < n_syms; s++) {
        int sh = 2 * (exps[s] - e_min);
        for (k = 0; k < cap; k++)
            arr[s * cap + k] >>= sh;
    }
    return 2 * e_min; /* scale_log2, for the calibrate path later */
}

void rxd_quantize(const int64_t *arr, int n, int target_bits, int64_t *out)
{
    int64_t peak = 0;
    int i, bl = 0, shift;
    for (i = 0; i < n; i++) {
        int64_t a = arr[i] < 0 ? -arr[i] : arr[i];
        if (a > peak)
            peak = a;
    }
    while (peak > 0) {
        peak >>= 1;
        bl++;
    }
    shift = bl - (target_bits - 1);
    if (shift < 0)
        shift = 0;
    for (i = 0; i < n; i++) {
        int64_t v = arr[i] >> shift;
        int64_t lim = (1 << (target_bits - 1)) - 1;
        out[i] = v > lim ? lim : (v < -lim ? -lim : v);
    }
}

/* descramble -> deinterleave -> crop -> Viterbi / LDPC min-sum */
void rxd_decode_block(const int64_t *llrs, int n_total, cc_rate_t rate,
                      int use_ldpc, int bits_count, uint8_t *out)
{
    static int64_t descr[MAX_LLRS], deint[MAX_LLRS];
    static uint8_t work[CONV_STATES / 8 * CONV_MAX_STEPS_PUB];

    descramble_llrs(llrs, n_total, descr);
    deinterleave_i64(descr, n_total, N_DATA_CARRIERS, deint);
    if (use_ldpc) {
        int coded_len = ldpc_cc_elements(bits_count);
        ldpc_decode_int(deint, coded_len, bits_count, out);
    } else {
        int coded_len = conv_cc_elements(rate, bits_count);
        conv_decode(rate, deint, coded_len, bits_count, out, work);
    }
}

/* re-encoded known bits -> +-1 reference in the received stream's order */
int rxd_known_ref(const uint8_t *coded, int n, int cap, int8_t *ref)
{
    static uint8_t padded[MAX_LLRS], il[MAX_LLRS], scr[MAX_LLRS];
    int n_syms = (n + cap - 1) / cap;
    int total = n_syms * cap;
    int i;

    memcpy(padded, coded, (size_t)n);
    memset(padded + n, 0, (size_t)(total - n));
    interleave_u8(padded, total, N_DATA_CARRIERS, il);
    scramble_bits(il, total, scr);
    for (i = 0; i < total; i++)
        ref[i] = (int8_t)(2 * scr[i] - 1);
    return total;
}

static int bit_length_i64(int64_t v)
{
    int bl = 0;
    while (v > 0) {
        v >>= 1;
        bl++;
    }
    return bl;
}

static int64_t peak_abs(const int64_t *a, int n)
{
    int64_t p = 0;
    int i;
    for (i = 0; i < n; i++) {
        int64_t v = a[i] < 0 ? -a[i] : a[i];
        if (v > p)
            p = v;
    }
    return p;
}

/* header-based integer temperature fit; returns alpha_q12 or 0 on a
 * degenerate fit; *fit_shift receives the h64 alignment shift */
int64_t rxd_fit_alpha_q12(const int64_t *h64, const int8_t *ref, int n,
                          int *fit_shift)
{
    int64_t m = 0, ssq = 0, den;
    int sh = bit_length_i64(peak_abs(h64, n)) - 20;
    int i;
    if (sh < 0)
        sh = 0;
    *fit_shift = sh;
    for (i = 0; i < n; i++) {
        int64_t lx = (h64[i] >> sh) * ref[i];
        m += lx;
        ssq += lx * lx;
    }
    den = (int64_t)n * ssq - m * m;
    if (den <= 0 || m <= 0)
        return 0;
    {
        int64_t a = ((2 * m * (int64_t)n) << 12) / den;
        return a < 1 ? 1 : a;
    }
}

/* calibrated-domain data LLRs through the reliability ROM */
void rxd_calibrated_llrs(const int64_t *d64, int n, int scale_d,
                         int64_t alpha_q12, int hdr_scale_fit, int64_t *out)
{
    int shift = scale_d - hdr_scale_fit;
    int i;
    for (i = 0; i < n; i++) {
        int64_t d_hf = shift >= 0 ? (d64[i] >> shift) : (d64[i] << -shift);
        int64_t l_cal_q2 = (alpha_q12 * d_hf) >> 10;
        int64_t a = l_cal_q2 < 0 ? -l_cal_q2 : l_cal_q2;
        int64_t idx = a >> 2;
        int sgn = l_cal_q2 > 0 ? 1 : (l_cal_q2 < 0 ? -1 : 0);
        if (idx > 31)
            idx = 31;
        out[i] = sgn * (int64_t)RECAL_ROM[idx];
    }
}

/* gain-weighted per-column moments of one LLR block (fading-robust) */
int rxd_snr_block_moments(const int64_t *arr, const int8_t *ref, int n,
                          int cap, int64_t *num_out, int64_t *den_out)
{
    static int64_t hf[MAX_LLRS];
    static int64_t g[MAX_SYMS];
    int rows = n / cap;
    int sh = bit_length_i64(peak_abs(arr, n)) - 10;
    int t, c, kept = 0;
    int64_t w = 0, sum_p = 0, num = 0;

    if (sh < 0)
        sh = 0;
    for (t = 0; t < rows; t++) {
        int64_t s = 0;
        for (c = 0; c < cap; c++) {
            hf[t * cap + c] = arr[t * cap + c] >> sh;
            s += hf[t * cap + c] < 0 ? -hf[t * cap + c] : hf[t * cap + c];
        }
        g[t] = s / cap;
        if (g[t] > 0)
            kept++;
    }
    if (kept < 2)
        return -1;
    for (c = 0; c < cap; c++) {
        int64_t s_c = 0, p_c = 0;
        for (t = 0; t < rows; t++) {
            if (g[t] <= 0)
                continue;
            s_c += g[t] * hf[t * cap + c] * ref[t * cap + c];
            p_c += hf[t * cap + c] * hf[t * cap + c];
        }
        num += s_c * s_c;
        sum_p += p_c;
    }
    for (t = 0; t < rows; t++)
        if (g[t] > 0)
            w += g[t] * g[t];
    *num_out = num;
    *den_out = w * sum_p - num;
    return (*num_out > 0 && *den_out > 0) ? 0 : -1;
}

int rxd_log2_q4(int64_t v)
{
    int bl = bit_length_i64(v);
    int mant = bl >= 5 ? (int)((v >> (bl - 5)) & 0xF)
                       : (int)((v << (5 - bl)) & 0xF);
    return ((bl - 1) << 4) + LOG2_FRAC_Q4[mant];
}

/* demod + decode with g_i/g_q already filled and tail-padded */
static int receive_common(rxd_t *r, int start, int64_t cfo_word,
                          rxd_header_t *hdr, uint8_t *pkt_bits,
                          const int64_t *prev_llrs, int prev_n,
                          int64_t *llrs_out, int *llrs_n_out)
{
    static int64_t h64[MAX_LLRS], d64[MAX_LLRS], q64[MAX_LLRS];
    static int64_t comb[MAX_LLRS];
    static int8_t ref[MAX_LLRS];
    static uint8_t coded[MAX_LLRS];
    uint8_t hdr_bits[HEADER_BITS];
    int n_hdr, n_data, cap, mu, coded_len, pos, use_ldpc, n_llr;
    int bits_count, scale_h, scale_d, ok, combined = 0;

    r->last_hyp = -1;
    r->last_harq_combined = 0;

    n_hdr = (conv_cc_elements(CC_R13, HEADER_BITS) + N_DATA_CARRIERS - 1)
            / N_DATA_CARRIERS;
    scale_h = demod_block(r, start, cfo_word, n_hdr, 1, h64);
    rxd_quantize(h64, n_hdr * N_DATA_CARRIERS, 6, q64);
    rxd_decode_block(q64, n_hdr * N_DATA_CARRIERS, CC_R13, 0, HEADER_BITS,
                 hdr_bits);
    if (header_decode(hdr_bits, &hdr->ver, &hdr->typ, &hdr->mod, &hdr->spd,
                      &hdr->len) != 0)
        return -1;
    if (hdr->ver != 1 && hdr->ver != 2)
        return -2;
    use_ldpc = hdr->ver == 2;
    if (hdr->typ == PKT_TYP_EXT_DATA && use_ldpc)
        return -2; /* EXT frames are conv-only */
    bits_count = PKT_BITS_FROM_HDR(hdr->typ, hdr->len);

    mu = hdr->mod == 2 ? 4 : (hdr->mod == 1 ? 2 : 1);
    cap = N_DATA_CARRIERS * mu;
    coded_len = use_ldpc ? ldpc_cc_elements(bits_count)
                         : conv_cc_elements((cc_rate_t)hdr->spd, bits_count);
    n_data = (coded_len + cap - 1) / cap;
    n_llr = n_data * cap;
    pos = start + n_hdr * r->symbol_len;

    scale_d = demod_block(r, pos, cfo_word, n_data, mu, d64);

    if (r->calibrate && mu <= 2) {
        int fit_shift = 0;
        int64_t alpha;
        int ncod = conv_encoded_len(CC_R13, HEADER_BITS);

        conv_encode(CC_R13, hdr_bits, HEADER_BITS, coded);
        rxd_known_ref(coded, ncod, N_DATA_CARRIERS, ref);
        alpha = rxd_fit_alpha_q12(h64, ref, ncod, &fit_shift);
        if (alpha > 0)
            rxd_calibrated_llrs(d64, n_llr, scale_d, alpha,
                            scale_h + fit_shift, q64);
        else
            rxd_quantize(d64, n_llr, mu == 4 ? 8 : 6, q64);
    } else {
        rxd_quantize(d64, n_llr, mu == 4 ? 8 : 6, q64);
    }

    if (llrs_out) {
        memcpy(llrs_out, q64, sizeof(int64_t) * (size_t)n_llr);
        if (llrs_n_out)
            *llrs_n_out = n_llr;
    }

    rxd_decode_block(q64, n_llr, (cc_rate_t)hdr->spd, use_ldpc, bits_count,
                 pkt_bits);
    ok = data_check_crc(pkt_bits, bits_count) == 0;
    if (!ok && prev_llrs && prev_n == n_llr) {
        int i;
        for (i = 0; i < n_llr; i++)
            comb[i] = q64[i] + prev_llrs[i];
        rxd_decode_block(comb, n_llr, (cc_rate_t)hdr->spd, use_ldpc,
                     bits_count, pkt_bits);
        ok = data_check_crc(pkt_bits, bits_count) == 0;
        combined = ok;
    }
    if (!ok)
        return -3;
    r->last_harq_combined = combined;

    /* data-aided SNR estimate over every symbol with known bits */
    {
        int64_t num = 0, den = 0, bn, bd;
        int ncod_h = conv_encoded_len(CC_R13, HEADER_BITS);
        int nref;

        conv_encode(CC_R13, hdr_bits, HEADER_BITS, coded);
        nref = rxd_known_ref(coded, ncod_h, N_DATA_CARRIERS, ref);
        if (rxd_snr_block_moments(h64, ref, nref, N_DATA_CARRIERS, &bn, &bd) == 0) {
            num += bn;
            den += bd;
        }
        if (mu <= 2) {
            int ncod_d;
            if (use_ldpc) {
                ldpc_encode(pkt_bits, bits_count, coded);
                ncod_d = ldpc_cc_elements(bits_count);
            } else {
                conv_encode((cc_rate_t)hdr->spd, pkt_bits, bits_count, coded);
                ncod_d = conv_encoded_len((cc_rate_t)hdr->spd, bits_count);
            }
            nref = rxd_known_ref(coded, ncod_d, cap, ref);
            if (rxd_snr_block_moments(d64, ref, nref, cap, &bn, &bd) == 0) {
                num += bn;
                den += bd;
            }
        }
        if (num > 0 && den > 0) {
            int l2 = rxd_log2_q4(num) - rxd_log2_q4(den);
            r->last_snr_db = (double)l2 / 16.0 * TEN_LOG10_2
                             - TILE_DBS[r->mode] + SNR_CAL_DB;
        } else {
            r->last_snr_db = -30.0;
        }
    }
    return 0;
}

static void fill_analytic(rxd_t *r, const int16_t *samples, int n_samples)
{
    hilbert_analytic(samples, n_samples, g_i, g_q);
    /* tail pad tolerates a small positive timing slip, as the model */
    memset(g_i + n_samples, 0, sizeof(int64_t) * (size_t)r->symbol_len);
    memset(g_q + n_samples, 0, sizeof(int64_t) * (size_t)r->symbol_len);
}

int rxd_receive_genie(rxd_t *r, const int16_t *samples, int n_samples,
                      int start, int64_t cfo_word,
                      rxd_header_t *hdr, uint8_t *pkt_bits)
{
    fill_analytic(r, samples, n_samples);
    return receive_common(r, start, cfo_word, hdr, pkt_bits, 0, 0, 0, 0);
}

int rxd_receive_genie_harq(rxd_t *r, const int16_t *samples, int n_samples,
                           int start, int64_t cfo_word,
                           rxd_header_t *hdr, uint8_t *pkt_bits,
                           const int64_t *prev_llrs, int prev_n,
                           int64_t *llrs_out, int *llrs_n_out)
{
    fill_analytic(r, samples, n_samples);
    return receive_common(r, start, cfo_word, hdr, pkt_bits,
                          prev_llrs, prev_n, llrs_out, llrs_n_out);
}

int rxd_receive(rxd_t *r, const int16_t *samples, int n_samples,
                rxd_header_t *hdr, uint8_t *pkt_bits,
                int *start_out, int64_t *cfo_word_out)
{
    int start;
    int64_t cfo_word;

    hilbert_analytic(samples, n_samples, g_i, g_q);
    if (rx_detect(r->mode, g_i, g_q, n_samples, &start, &cfo_word) != 0)
        return -4;
    memset(g_i + n_samples, 0, sizeof(int64_t) * (size_t)r->symbol_len);
    memset(g_q + n_samples, 0, sizeof(int64_t) * (size_t)r->symbol_len);
    if (start_out)
        *start_out = start;
    if (cfo_word_out)
        *cfo_word_out = cfo_word;
    return receive_common(r, start, cfo_word, hdr, pkt_bits, 0, 0, 0, 0);
}
