/* Internal receiver primitives shared between the frame-at-once reference
 * (rx_demod.c) and the MCU streaming architecture (rx_stream.c). Not part
 * of the public API. */
#ifndef OFDM_RX_INTERNAL_H
#define OFDM_RX_INTERNAL_H

#include <stdint.h>
#include "rx_demod.h"
#include "conv.h"

#define MAX_LLRS 8192 /* 255-byte EXT frame at BPSK 1/3 */
#define MAX_SYMS 400

/* one symbol at seg (>= symbol_len samples), absolute position `pos` for
 * the CFO phase reference; window = NULL -> coarse/full search. llr
 * receives N_DATA_CARRIERS * mu values; returns the BFP exponent. */
int rxd_demod_symbol(rxd_t *r, const int64_t *seg_i, const int64_t *seg_q,
                     int pos, int64_t cfo_word, int mu,
                     const int *window, int win_n, int64_t *llr);

void rxd_quantize(const int64_t *arr, int n, int target_bits, int64_t *out);
void rxd_decode_block(const int64_t *llrs, int n_total, cc_rate_t rate,
                      int use_ldpc, int bits_count, uint8_t *out);
int rxd_known_ref(const uint8_t *coded, int n, int cap, int8_t *ref);
int64_t rxd_fit_alpha_q12(const int64_t *h64, const int8_t *ref, int n,
                          int *fit_shift);
void rxd_calibrated_llrs(const int64_t *d64, int n, int scale_d,
                         int64_t alpha_q12, int hdr_scale_fit, int64_t *out);
int rxd_snr_block_moments(const int64_t *arr, const int8_t *ref, int n,
                          int cap, int64_t *num_out, int64_t *den_out);
int rxd_log2_q4(int64_t v);
double rxd_tile_db(link_mode_t mode);

#endif /* OFDM_RX_INTERNAL_H */
