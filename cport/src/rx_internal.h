/* Internal receiver primitives shared between the frame-at-once reference
 * (rx_demod.c) and the MCU streaming architecture (rx_stream.c). Not part
 * of the public API. */
#ifndef OFDM_RX_INTERNAL_H
#define OFDM_RX_INTERNAL_H

#include <stdint.h>
#include "dsp.h"
#include "rx_demod.h"
#include "conv.h"

/* LLR sample type. These hold demodulated soft values, measured peak
 * 1211062 across the suites -- 21 bits, so int32 keeps a ~1770x margin
 * while halving the largest remaining buffers in a streaming receiver
 * (448 KB of them). FEASIBILITY.md called this out as available
 * headroom; this takes it.
 *
 * Products of two llr_t values MUST be widened explicitly: 1.2e6 squared
 * is 1.4e12 and overflows 32 bits. On Cortex-M7 the widening multiply is
 * SMULL/SMLAL, the same instruction count as MUL/MLA, so it is free. */
#ifndef OFDM_LLR_T
#define OFDM_LLR_T
typedef int32_t llr_t;
#endif

/* Shared scratch arena.
 *
 * These phases are strictly sequential within one rxs_push, and every
 * buffer below is CALL-SCOPED -- none survives the return:
 *
 *   detect  g_wi/g_wq slide window, mag ring, ZC kernel, lag delay line
 *   demod   one symbol's samples
 *   decode  quantised LLRs, descramble/deinterleave/HARQ
 *
 * Detection finishes before the first symbol is demodulated, and demod
 * before decode, so they share storage. Sized by the largest phase
 * (detect, 31489 int64) rather than their sum (514568 B).
 *
 * What must NOT live here: g_raw, g_blk, g_h64/g_d64 all carry state
 * across pushes. */
/* Sized in BYTES, because the phases mix sample (samp_t), LLR (llr_t)
 * and accumulator (int64) widths. Detect is the largest once samples are
 * int32: its mag ring stays int64.
 *
 *   detect 156680   demod 131584   decode 131072
 *
 * The union forces int64 alignment for the accumulator views. */
#define RX_ARENA_BYTES 156680
extern union rx_arena_u {
    int64_t align;
    unsigned char b[RX_ARENA_BYTES];
} rx_arena_store;
#define rx_arena (rx_arena_store.b)

/* Sized for a 255-byte EXT frame at BPSK 1/3. A build without
 * PKT_TYP_EXT_DATA can set this to 1024 (-DMAX_LLRS=1024), which is a
 * FEATURE trade, not a mode trade. */
#ifndef MAX_LLRS
#define MAX_LLRS 8192
#endif
#define MAX_SYMS 400

/* one symbol at seg (>= symbol_len samples), absolute position `pos` for
 * the CFO phase reference; window = NULL -> coarse/full search. llr
 * receives N_DATA_CARRIERS * mu values; returns the BFP exponent. */
int rxd_demod_symbol(rxd_t *r, const samp_t *seg_i, const samp_t *seg_q,
                     int pos, int64_t cfo_word, int mu,
                     const int *window, int win_n, llr_t *llr);

void rxd_quantize(const llr_t *arr, int n, int target_bits, llr_t *out);
void rxd_decode_block(const llr_t *llrs, int n_total, cc_rate_t rate,
                      int use_ldpc, int bits_count, uint8_t *out);
int rxd_known_ref(const uint8_t *coded, int n, int cap, int8_t *ref);
int64_t rxd_fit_alpha_q12(const llr_t *h64, const int8_t *ref, int n,
                          int *fit_shift);
void rxd_calibrated_llrs(const llr_t *d64, int n, int scale_d,
                         int64_t alpha_q12, int hdr_scale_fit, llr_t *out);
int rxd_snr_block_moments(const llr_t *arr, const int8_t *ref, int n,
                          int cap, int64_t *num_out, int64_t *den_out);
int rxd_log2_q4(int64_t v);
double rxd_tile_db(link_mode_t mode);

#endif /* OFDM_RX_INTERNAL_H */
