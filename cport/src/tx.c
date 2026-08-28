#include <string.h>

#include "tx.h"
#include "fxp.h"
#include "fft.h"
#include "bits.h"
#include "ldpc.h"
#include "packets.h"
#include "rom_tables.h"
#include "rom_modes.h"
#include "arena.h"

#define QPSK_AMP 23170   /* round(Q15_MAX / sqrt(2)) */
#define QAM16_AMP 10362  /* round(Q15_MAX / sqrt(10)) */
#define OUTPUT_GAIN_SHIFT 3
#define TX_LPF_N 33

/* worst case: EXTREME, 255-bit packet at BPSK 1/3 (host reference keeps the
 * whole frame; an MCU TX streams symbol by symbol instead) */
#define TX_MAX_SAMPLES 540672
#define TX_MAX_CODED 8448

/* Transmit-side layout of the shared arena (arena.h). The FEC scratch
 * leads because its offsets must be known here, above the generator
 * state whose size closes the region (TX_OFF_END, asserted below). Both
 * multiples of 8, so the state keeps the arena's int64 alignment. */
#define TX_OFF_CODED 0
#define TX_OFF_IL    (TX_OFF_CODED + TX_MAX_CODED)
#define TX_OFF_STATE (TX_OFF_IL + TX_MAX_CODED)

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
/* One symbol's 128-sample tile: map, place carriers, IFFT. A tiled OFDM
 * symbol is just this block repeated sym_tile times behind a CP that is
 * its own tail -- which is what lets the streaming transmitter hold a
 * symbol in 1 kB instead of buffering the frame. */
static void symbol_tile(const uint8_t *row, mod_type_t mod, int64_t *sym_re)
{
    int64_t sub_re[N_CHANNEL], sub_im[N_CHANNEL];
    int64_t sym_im[FFT_BINS];
    int64_t d_re[N_DATA_CARRIERS], d_im[N_DATA_CARRIERS];
    int k;

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

    memset(sym_re, 0, FFT_BINS * sizeof(*sym_re));
    memset(sym_im, 0, sizeof(sym_im));
    for (k = 0; k < N_CHANNEL; k++) {
        int ci = CHANNEL_IDX[k];
        sym_re[ci] = sub_re[k];
        sym_im[ci] = sub_im[k];
        sym_re[FFT_BINS - ci] = sub_re[k]; /* Hermitian mirror */
        sym_im[FFT_BINS - ci] = -sub_im[k];
    }

    ifft_fixed(sym_re, sym_im, FFT_BINS); /* real output in sym_re */
}

/* sample k of a tiled symbol: CP (the tile's tail) then sym_tile copies */
static int64_t tile_sample(const int64_t *tile, int k)
{
    return k < CP_LEN ? tile[FFT_BINS - CP_LEN + k]
                      : tile[(k - CP_LEN) % FFT_BINS];
}

static void modulate_symbol(const uint8_t *row, mod_type_t mod, int sym_tile,
                            int64_t *sig, int *pos)
{
    int64_t sym_re[FFT_BINS];
    int n = CP_LEN + sym_tile * FFT_BINS, k;

    symbol_tile(row, mod, sym_re);
    for (k = 0; k < n; k++)
        sig[(*pos)++] = tile_sample(sym_re, k);
}

/* FEC-encode (conv or LDPC), pad to whole symbols, interleave, scramble;
 * returns symbol count, rows in `rows` (n_syms * capacity bits) */
static int encode_block(const uint8_t *bits, int bits_n, cc_rate_t rate,
                        int use_ldpc, int capacity, uint8_t *rows)
{
    uint8_t *const coded = (uint8_t *)(ofdm_arena + TX_OFF_CODED);
    uint8_t *const il = (uint8_t *)(ofdm_arena + TX_OFF_IL);
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

/* ------------------------------------------------------------------ *
 * Streaming transmitter: generate the waveform on demand
 *
 * The frame-at-once path holds every sample of a frame in g_sig -- 4.3 MB
 * at EXTREME, the largest buffer in the port, for a waveform that is
 * nothing but short periodic blocks repeated: tone A is 32 samples, tone
 * B is 64, ZC is 128, and a data symbol is one 128-sample IFFT tile
 * repeated sym_tile times behind a CP that is its own tail. So the
 * generator needs one tile (1 kB), not one frame.
 *
 * The one thing that genuinely needs the whole waveform is the clip
 * threshold, which is 2x its RMS. Rather than buffer for it, the
 * generator runs TWICE: once to accumulate the energy, then again to
 * emit. It is a pure function of its inputs, so the second pass
 * reproduces the first exactly -- and the result is bit-identical to
 * tx_build_frame/tx_build_burst, which the suites assert.
 * ------------------------------------------------------------------ */

enum { G_TONE_A, G_TONE_B, G_ZC_PRE, G_HDR, G_RESYNC, G_BLK, G_DONE };

struct txs_state {
    int mode, typ, mod, spd, resync_every, use_ldpc;
    const uint8_t *blocks;
    int pkt_bits_n, n_blocks, capacity, total;
    int64_t thr;
    /* generator cursor */
    int stage, seg_k, blk, sym, n_syms;
    int64_t tile[FFT_BINS];
    uint8_t rows[TX_MAX_CODED];
    uint8_t hdr_bits[HEADER_BITS];
    /* output pipeline: clipped samples awaiting the 33-tap LPF */
    int gen_idx, out_idx;
    int64_t ring[TXS_RING];
};

/* The generator state is live across txs_pull calls; encode_block's
 * scratch is not, but it runs *inside* a pull, so the two need separate
 * room. rows[] lives inside the state and is aliased with neither. */
#define TX_OFF_END (TX_OFF_STATE + (int)sizeof(struct txs_state))
typedef char tx_arena_fits[TX_OFF_END <= OFDM_ARENA_BYTES ? 1 : -1];

#define g_txs (*(struct txs_state *)(void *)(ofdm_arena + TX_OFF_STATE))

/* Set when a pull found the arena in someone else's hands. It cannot
 * live in the arena for the obvious reason. */
static int g_txs_fault;

static int txs_sym_len(const struct txs_state *t)
{
    return CP_LEN + MODES[t->mode].sym_tile * FFT_BINS;
}

static void txs_rewind(struct txs_state *t)
{
    t->stage = G_TONE_A;
    t->seg_k = 0;
    t->blk = 0;
    t->sym = 0;
}

/* next raw (pre-clip, pre-filter) sample of the waveform */
static int64_t txs_gen(struct txs_state *t)
{
    const tx_mode_t *mc = &MODES[t->mode];
    int sym_len = txs_sym_len(t);

    for (;;) {
        switch (t->stage) {
        case G_TONE_A:
            if (t->seg_k < 2 * mc->tone_tiles * FFT_BINS)
                return mc->tone_a[t->seg_k++ & 31];
            t->seg_k = 0;
            t->stage = G_TONE_B;
            continue;
        case G_TONE_B:
            if (t->seg_k < mc->tone_tiles * FFT_BINS)
                return mc->tone_b[t->seg_k++ & 63];
            t->seg_k = 0;
            t->stage = G_ZC_PRE;
            continue;
        case G_ZC_PRE:
        case G_RESYNC:
            if (t->seg_k < sym_len) {
                int k = t->seg_k++;
                return k < CP_LEN ? mc->zc[FFT_BINS - CP_LEN + k]
                                  : mc->zc[(k - CP_LEN) % FFT_BINS];
            }
            t->seg_k = 0;
            if (t->stage == G_ZC_PRE) {
                t->n_syms = encode_block(t->hdr_bits, HEADER_BITS, CC_R13, 0,
                                         N_DATA_CARRIERS, t->rows);
                t->sym = 0;
                t->stage = G_HDR;
                symbol_tile(t->rows, MOD_BPSK, t->tile);
            } else {
                t->n_syms = encode_block(t->blocks
                                             + (size_t)t->blk * t->pkt_bits_n,
                                         t->pkt_bits_n, (cc_rate_t)t->spd,
                                         t->use_ldpc, t->capacity, t->rows);
                t->sym = 0;
                t->stage = G_BLK;
                symbol_tile(t->rows, (mod_type_t)t->mod, t->tile);
            }
            continue;
        case G_HDR:
            if (t->seg_k < sym_len)
                return tile_sample(t->tile, t->seg_k++);
            t->seg_k = 0;
            if (++t->sym < t->n_syms) {
                symbol_tile(t->rows + (size_t)t->sym * N_DATA_CARRIERS,
                            MOD_BPSK, t->tile);
                continue;
            }
            t->blk = 0;
            t->stage = G_RESYNC;   /* block 0 never carries a resync ZC */
            t->seg_k = sym_len;
            continue;
        case G_BLK:
            if (t->seg_k < sym_len)
                return tile_sample(t->tile, t->seg_k++);
            t->seg_k = 0;
            if (++t->sym < t->n_syms) {
                symbol_tile(t->rows + (size_t)t->sym * t->capacity,
                            (mod_type_t)t->mod, t->tile);
                continue;
            }
            if (++t->blk >= t->n_blocks) {
                t->stage = G_DONE;
                continue;
            }
            t->stage = G_RESYNC;
            /* skip the ZC unless this block boundary carries one */
            t->seg_k = (t->resync_every > 0
                        && t->blk % t->resync_every == 0) ? 0 : sym_len;
            continue;
        default:
            return 0;
        }
    }
}

txs_t *txs_open(link_mode_t mode, const uint8_t *blocks, int pkt_bits_n,
                int n_blocks, int typ, mod_type_t mod, cc_rate_t spd,
                int resync_every, int use_ldpc, int *total_out)
{
    struct txs_state *t = &g_txs;
    int64_t sum_sq = 0;
    int max_bits = typ == PKT_TYP_EXT_DATA ? 36 + 8 * 255 : 255;
    int k;

    if (n_blocks < 1 || pkt_bits_n > max_bits)
        return 0;
    if (typ == PKT_TYP_EXT_DATA && use_ldpc)
        return 0;

    /* claim before the first write: pass 1 below already runs the
     * generator, and encode_block writes its own arena slots */
    arena_claim(ARENA_TX);
    g_txs_fault = 0;

    memset(t, 0, sizeof(*t));
    t->mode = mode;
    t->blocks = blocks;
    t->pkt_bits_n = pkt_bits_n;
    t->n_blocks = n_blocks;
    t->typ = typ;
    t->mod = mod;
    t->spd = spd;
    t->resync_every = resync_every;
    t->use_ldpc = use_ldpc;
    t->capacity = N_DATA_CARRIERS * mod_mu(mod);
    t->total = n_blocks == 1 && resync_every == 0
                   ? tx_frame_len_ex(mode, pkt_bits_n, mod, spd, use_ldpc)
                   : tx_burst_len(mode, pkt_bits_n, mod, spd, n_blocks,
                                  resync_every);
    if (t->total < 0)
        return 0;
    header_encode(use_ldpc ? 2 : 1, typ, (int)mod, (int)spd,
                  typ == PKT_TYP_EXT_DATA ? (pkt_bits_n - 36) / 8 : pkt_bits_n,
                  t->hdr_bits);

    /* pass 1: energy only, samples discarded */
    txs_rewind(t);
    for (k = 0; k < t->total; k++) {
        int64_t v = txs_gen(t);
        sum_sq += v * v;
    }
    t->thr = 2 * isqrt_i64(sum_sq / t->total);

    /* pass 2 starts clean */
    txs_rewind(t);
    t->gen_idx = 0;
    t->out_idx = 0;
    if (total_out)
        *total_out = t->total;
    return t;
}

int txs_total(const txs_t *t)
{
    if (!t || !arena_held_by(ARENA_TX))
        return -1;
    return t->total;
}

int txs_faulted(void)
{
    return g_txs_fault;
}

int txs_pull(txs_t *t, int16_t *out, int max)
{
    int half = (TX_LPF_N - 1) / 2, n = 0;

    if (!t || !out || max <= 0)
        return 0;
    /* Half duplex is the contract that lets the generator share the
     * receiver's arena (arena.h). If a receive phase ran since txs_open,
     * everything below would be reading its leftovers: stop instead, and
     * say so through txs_faulted(). */
    if (!arena_held_by(ARENA_TX)) {
        g_txs_fault = 1;
        return 0;
    }
    while (n < max && t->out_idx < t->total) {
        int64_t acc = 0;
        int j;
        /* keep the filter's right half populated */
        while (t->gen_idx <= t->out_idx + half && t->gen_idx < t->total) {
            int64_t v = txs_gen(t);
            if (v > t->thr)
                v = t->thr;
            else if (v < -t->thr)
                v = -t->thr;
            t->ring[t->gen_idx & (TXS_RING - 1)] = v;
            t->gen_idx++;
        }
        for (j = 0; j < TX_LPF_N; j++) {
            int idx = t->out_idx + half - j;
            if (idx >= 0 && idx < t->total)
                acc += (int64_t)TX_LPF_TAPS[j] * t->ring[idx & (TXS_RING - 1)];
        }
        out[n++] = sat16(rshift_round(acc, Q15) * (1 << OUTPUT_GAIN_SHIFT));
        t->out_idx++;
    }
    return n;
}
