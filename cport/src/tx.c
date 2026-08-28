#include <string.h>

#include "tx.h"
#include "fxp.h"
#include "fft.h"
#include "bits.h"
#include "ldpc.h"
#include "packets.h"
#include "rom_tables.h"
#include "rom_modes.h"

#define QPSK_AMP 23170   /* round(Q15_MAX / sqrt(2)) */
#define QAM16_AMP 10362  /* round(Q15_MAX / sqrt(10)) */
#define OUTPUT_GAIN_SHIFT 3
#define TX_LPF_N 33

/* worst case: EXTREME, 255-bit packet at BPSK 1/3 (host reference keeps the
 * whole frame; an MCU TX streams symbol by symbol instead) */
#define TX_MAX_SAMPLES 540672
#define TX_MAX_CODED 8448

static int64_t g_sig[TX_MAX_SAMPLES];

typedef struct {
    /* preamble as its unique periodic blocks (see gen_vectors.py):
     * tone field A = 2*tone_tiles FFT frames of a 32-sample block,
     * tone field B = tone_tiles frames of a 64-sample block,
     * ZC symbol    = CP (tile tail) + sym_tile copies of a 128 tile */
    const int16_t *tone_a, *tone_b, *zc;
    int tone_tiles;
    int preamble_len;
    int sym_tile;
} tx_mode_t;

static const tx_mode_t MODES[3] = {
    { PRE_TONEA_NORMAL, PRE_TONEB_NORMAL, PRE_ZC_NORMAL,
      PRE_TONE_TILES_NORMAL, PREAMBLE_LEN_NORMAL, SYM_TILE_NORMAL },
    { PRE_TONEA_ROBUST, PRE_TONEB_ROBUST, PRE_ZC_ROBUST,
      PRE_TONE_TILES_ROBUST, PREAMBLE_LEN_ROBUST, SYM_TILE_ROBUST },
    { PRE_TONEA_EXTREME, PRE_TONEB_EXTREME, PRE_ZC_EXTREME,
      PRE_TONE_TILES_EXTREME, PREAMBLE_LEN_EXTREME, SYM_TILE_EXTREME },
};

static int mod_mu(mod_type_t mod)
{
    return mod == MOD_QAM16 ? 4 : (mod == MOD_QPSK ? 2 : 1);
}

/* Gray per axis: (0,0)->-3 (0,1)->-1 (1,1)->+1 (1,0)->+3 */
static int qam16_level(int b0, int b1)
{
    static const int lvl[4] = { -3, -1, 3, 1 }; /* index = (b0<<1)|b1 */
    return lvl[(b0 << 1) | b1];
}

static void map_bits(const uint8_t *bits, mod_type_t mod, int n_carriers,
                     int64_t *re, int64_t *im)
{
    int c;
    for (c = 0; c < n_carriers; c++) {
        switch (mod) {
        case MOD_BPSK:
            re[c] = bits[c] ? Q15_MAX : -Q15_MAX;
            im[c] = 0;
            break;
        case MOD_QPSK:
            re[c] = QPSK_AMP * (2 * bits[2 * c] - 1);
            im[c] = QPSK_AMP * (2 * bits[2 * c + 1] - 1);
            break;
        default: /* MOD_QAM16 */
            re[c] = QAM16_AMP * qam16_level(bits[4 * c], bits[4 * c + 1]);
            im[c] = QAM16_AMP * qam16_level(bits[4 * c + 2], bits[4 * c + 3]);
            break;
        }
    }
}

/* one tiled OFDM symbol (CP + tile * FFT_BINS) appended at *pos */
static void modulate_symbol(const uint8_t *row, mod_type_t mod, int sym_tile,
                            int64_t *sig, int *pos)
{
    int64_t sub_re[N_CHANNEL], sub_im[N_CHANNEL];
    int64_t sym_re[FFT_BINS], sym_im[FFT_BINS];
    int64_t d_re[N_DATA_CARRIERS], d_im[N_DATA_CARRIERS];
    int k, t;

    memset(sub_re, 0, sizeof(sub_re));
    memset(sub_im, 0, sizeof(sub_im));
    map_bits(row, mod, N_DATA_CARRIERS, d_re, d_im);
    for (k = 0; k < N_DATA_CARRIERS; k++) {
        sub_re[DATA_LOCAL_IDX[k]] = d_re[k];
        sub_im[DATA_LOCAL_IDX[k]] = d_im[k];
    }
    for (k = 0; k < N_PILOTS; k++) {
        sub_re[PILOT_LOCAL_IDX[k]] = PILOT_RE[k];
        sub_im[PILOT_LOCAL_IDX[k]] = PILOT_IM[k];
    }

    memset(sym_re, 0, sizeof(sym_re));
    memset(sym_im, 0, sizeof(sym_im));
    for (k = 0; k < N_CHANNEL; k++) {
        int ci = CHANNEL_IDX[k];
        sym_re[ci] = sub_re[k];
        sym_im[ci] = sub_im[k];
        sym_re[FFT_BINS - ci] = sub_re[k]; /* Hermitian mirror */
        sym_im[FFT_BINS - ci] = -sub_im[k];
    }

    ifft_fixed(sym_re, sym_im, FFT_BINS); /* real output in sym_re */

    /* CP = last CP_LEN samples of the (identical) tiles */
    for (k = 0; k < CP_LEN; k++)
        sig[(*pos)++] = sym_re[FFT_BINS - CP_LEN + k];
    for (t = 0; t < sym_tile; t++)
        for (k = 0; k < FFT_BINS; k++)
            sig[(*pos)++] = sym_re[k];
}

/* FEC-encode (conv or LDPC), pad to whole symbols, interleave, scramble;
 * returns symbol count, rows in `rows` (n_syms * capacity bits) */
static int encode_block(const uint8_t *bits, int bits_n, cc_rate_t rate,
                        int use_ldpc, int capacity, uint8_t *rows)
{
    static uint8_t coded[TX_MAX_CODED], il[TX_MAX_CODED];
    int n = use_ldpc ? ldpc_cc_elements(bits_n) : conv_encoded_len(rate, bits_n);
    int n_syms = (n + capacity - 1) / capacity;
    int total = n_syms * capacity;

    if (use_ldpc)
        ldpc_encode(bits, bits_n, coded);
    else
        conv_encode(rate, bits, bits_n, coded);
    memset(coded + n, 0, (size_t)(total - n));
    interleave_u8(coded, total, N_DATA_CARRIERS, il);
    scramble_bits(il, total, rows);
    return n_syms;
}

static int data_syms(int pkt_bits_n, mod_type_t mod, cc_rate_t spd,
                     int use_ldpc)
{
    int capacity = N_DATA_CARRIERS * mod_mu(mod);
    int n = use_ldpc ? ldpc_cc_elements(pkt_bits_n)
                     : conv_encoded_len(spd, pkt_bits_n);
    return (n + capacity - 1) / capacity;
}

int tx_frame_len_ex(link_mode_t mode, int pkt_bits_n, mod_type_t mod,
                    cc_rate_t spd, int use_ldpc)
{
    const tx_mode_t *mc = &MODES[mode];
    int symbol_len = CP_LEN + mc->sym_tile * FFT_BINS;
    int n_hdr = (conv_encoded_len(CC_R13, HEADER_BITS) + N_DATA_CARRIERS - 1)
                / N_DATA_CARRIERS;
    return mc->preamble_len
           + (n_hdr + data_syms(pkt_bits_n, mod, spd, use_ldpc)) * symbol_len;
}

int tx_frame_len(link_mode_t mode, int pkt_bits_n, mod_type_t mod,
                 cc_rate_t spd)
{
    return tx_frame_len_ex(mode, pkt_bits_n, mod, spd, 0);
}

int tx_burst_len(link_mode_t mode, int pkt_bits_n, mod_type_t mod,
                 cc_rate_t spd, int n_blocks, int resync_every)
{
    const tx_mode_t *mc = &MODES[mode];
    int symbol_len = CP_LEN + mc->sym_tile * FFT_BINS;
    int n_resync = resync_every > 0 ? (n_blocks - 1) / resync_every : 0;

    if (n_blocks < 1)
        return -1;
    return tx_frame_len(mode, pkt_bits_n, mod, spd)
           + (n_blocks - 1) * data_syms(pkt_bits_n, mod, spd, 0) * symbol_len
           + n_resync * symbol_len;
}

/* the ZC symbol alone (CP = tile tail, then sym_tile tiles) -- the same
 * ROM block that closes the preamble, re-emitted as a resync marker */
static void emit_zc(const tx_mode_t *mc, int64_t *sig, int *pos)
{
    int s, k;
    for (k = FFT_BINS - CP_LEN; k < FFT_BINS; k++)
        sig[(*pos)++] = mc->zc[k];
    for (s = 0; s < mc->sym_tile; s++)
        for (k = 0; k < FFT_BINS; k++)
            sig[(*pos)++] = mc->zc[k];
}

/* clip at RMS + ~6 dB, Q15 low-pass, static x8 gain -- over the WHOLE
 * waveform (the clip threshold is a signal-wide RMS, so a burst is not
 * the concatenation of separately finished frames) */
static void finish(int total, int16_t *out)
{
    int64_t sum_sq = 0, mean_sq, thr;
    int k;

    for (k = 0; k < total; k++)
        sum_sq += g_sig[k] * g_sig[k];
    mean_sq = sum_sq / total;
    thr = 2 * isqrt_i64(mean_sq);
    for (k = 0; k < total; k++) {
        if (g_sig[k] > thr)
            g_sig[k] = thr;
        else if (g_sig[k] < -thr)
            g_sig[k] = -thr;
    }

    for (k = 0; k < total; k++) {
        int64_t acc = 0;
        int j;
        for (j = 0; j < TX_LPF_N; j++) {
            int idx = k + (TX_LPF_N - 1) / 2 - j;
            if (idx >= 0 && idx < total)
                acc += (int64_t)TX_LPF_TAPS[j] * g_sig[idx];
        }
        /* multiply, do not shift: acc is routinely negative here and
         * "negative << n" is undefined behaviour in C (UBSan flags it).
         * gcc happens to emit an arithmetic shift, which is why this has
         * always worked -- but UB is exactly what differs between hosts
         * and targets. The multiply is well defined and identical. */
        out[k] = sat16(rshift_round(acc, Q15) * (1 << OUTPUT_GAIN_SHIFT));
    }
}

/* preamble: tone field A (2*tiles frames of a 32-sample block), tone
 * field B (tiles frames of a 64-sample block), then the ZC symbol */
static void emit_preamble(const tx_mode_t *mc, int64_t *sig, int *pos)
{
    int k;
    for (k = 0; k < 2 * mc->tone_tiles * FFT_BINS; k++)
        sig[(*pos)++] = mc->tone_a[k & 31];
    for (k = 0; k < mc->tone_tiles * FFT_BINS; k++)
        sig[(*pos)++] = mc->tone_b[k & 63];
    emit_zc(mc, sig, pos);
}

int tx_build_burst(link_mode_t mode, const uint8_t *blocks, int pkt_bits_n,
                   int n_blocks, int typ, mod_type_t mod, cc_rate_t spd,
                   int resync_every, int16_t *out)
{
    static uint8_t rows[TX_MAX_CODED];
    uint8_t hdr_bits[HEADER_BITS];
    const tx_mode_t *mc = &MODES[mode];
    int total = tx_burst_len(mode, pkt_bits_n, mod, spd, n_blocks,
                             resync_every);
    int capacity = N_DATA_CARRIERS * mod_mu(mod);
    int pos = 0, s, b, n_syms;

    {
        int max_bits = typ == PKT_TYP_EXT_DATA ? 36 + 8 * 255 : 255;
        if (n_blocks < 1 || pkt_bits_n > max_bits || total < 0
            || total > TX_MAX_SAMPLES)
            return -1;
    }

    emit_preamble(mc, g_sig, &pos);

    /* one header for the whole burst: every block shares typ/len/mod/spd */
    header_encode(1, typ, (int)mod, (int)spd,
                  typ == PKT_TYP_EXT_DATA ? (pkt_bits_n - 36) / 8 : pkt_bits_n,
                  hdr_bits);
    n_syms = encode_block(hdr_bits, HEADER_BITS, CC_R13, 0, N_DATA_CARRIERS,
                          rows);
    for (s = 0; s < n_syms; s++)
        modulate_symbol(rows + s * N_DATA_CARRIERS, MOD_BPSK, mc->sym_tile,
                        g_sig, &pos);

    for (b = 0; b < n_blocks; b++) {
        if (resync_every > 0 && b > 0 && b % resync_every == 0)
            emit_zc(mc, g_sig, &pos);
        n_syms = encode_block(blocks + (size_t)b * pkt_bits_n, pkt_bits_n,
                              spd, 0, capacity, rows);
        for (s = 0; s < n_syms; s++)
            modulate_symbol(rows + s * capacity, mod, mc->sym_tile, g_sig,
                            &pos);
    }

    finish(total, out);
    return total;
}

int tx_build_frame_ex(link_mode_t mode, const uint8_t *pkt_bits,
                      int pkt_bits_n, int typ, mod_type_t mod, cc_rate_t spd,
                      int use_ldpc, int16_t *out)
{
    static uint8_t rows[TX_MAX_CODED];
    uint8_t hdr_bits[HEADER_BITS];
    const tx_mode_t *mc = &MODES[mode];
    int total = tx_frame_len_ex(mode, pkt_bits_n, mod, spd, use_ldpc);
    int capacity = N_DATA_CARRIERS * mod_mu(mod);
    int pos = 0, s, n_syms;

    {
        int max_bits = typ == PKT_TYP_EXT_DATA ? 36 + 8 * 255 : 255;
        if (pkt_bits_n > max_bits || total > TX_MAX_SAMPLES)
            return -1;
        if (typ == PKT_TYP_EXT_DATA && use_ldpc)
            return -1; /* EXT frames are conv-only (LDPC K=256) */
    }

    emit_preamble(mc, g_sig, &pos);

    header_encode(use_ldpc ? 2 : 1, typ, (int)mod, (int)spd,
                  typ == PKT_TYP_EXT_DATA ? (pkt_bits_n - 36) / 8
                                          : pkt_bits_n,
                  hdr_bits);
    n_syms = encode_block(hdr_bits, HEADER_BITS, CC_R13, 0, N_DATA_CARRIERS,
                          rows);
    for (s = 0; s < n_syms; s++)
        modulate_symbol(rows + s * N_DATA_CARRIERS, MOD_BPSK, mc->sym_tile,
                        g_sig, &pos);

    n_syms = encode_block(pkt_bits, pkt_bits_n, spd, use_ldpc, capacity, rows);
    for (s = 0; s < n_syms; s++)
        modulate_symbol(rows + s * capacity, mod, mc->sym_tile, g_sig, &pos);

    finish(total, out);
    return total;
}

int tx_build_frame(link_mode_t mode, const uint8_t *pkt_bits, int pkt_bits_n,
                   int typ, mod_type_t mod, cc_rate_t spd, int16_t *out)
{
    return tx_build_frame_ex(mode, pkt_bits, pkt_bits_n, typ, mod, spd, 0,
                             out);
}
