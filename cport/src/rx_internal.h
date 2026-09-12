/* Internal receiver primitives shared between the frame-at-once reference
 * (rx_demod.c) and the MCU streaming architecture (rx_stream.c). Not part
 * of the public API. */
#ifndef OFDM_RX_INTERNAL_H
#define OFDM_RX_INTERNAL_H

#include <stdint.h>
#include "dsp.h"
#include "rx_demod.h"
#include "conv.h"
#include "arena.h"
#include "rom_modes.h"

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

/* Shared scratch arena -- see arena.h for the contract. The receiver's
 * three phases are strictly sequential within one rxs_push, and every
 * buffer below is CALL-SCOPED, so none survives the return:
 *
 *   detect  g_wi/g_wq slide window, mag ring, ZC kernel, lag delay line
 *   demod   one symbol's samples
 *   decode  quantised LLRs, descramble/deinterleave/HARQ
 *
 * Detection finishes before the first symbol is demodulated, and demod
 * before decode, so they share storage: 131584 B rather than 514568.
 * Demod is the largest, not detect -- eval_hyp's derotation scratch is
 * live *while* the symbol samples still are.
 *
 * What must NOT live here: g_raw, g_blk, g_h64/g_d64 all carry state
 * across pushes. The transmitter's generator state does too, but it is
 * live only while transmitting, which is why it can share (arena.h). */
#define RX_ARENA_BYTES OFDM_ARENA_BYTES
#define rx_arena ofdm_arena

/* Sized for a 255-byte EXT frame at BPSK 1/3. A build without
 * PKT_TYP_EXT_DATA can set this to 1024 (-DMAX_LLRS=1024), which is a
 * FEATURE trade, not a mode trade. */
#ifndef MAX_LLRS
#define MAX_LLRS 8192
#endif
#ifndef MAX_SYMS
#define MAX_SYMS 400   /* the radio build passes 280: its largest EXT frame is 276 symbols */
#endif

/* one symbol at seg (>= symbol_len samples), absolute position `pos` for
 * the CFO phase reference; window = NULL -> coarse/full search. llr
 * receives N_DATA_CARRIERS * mu values; returns the BFP exponent. */
/* rho (N_DATA_CARRIERS, may be NULL): the symbol's interference residual
 * per data carrier, scale 2^exp -- see rxd_wacc_t */
int rxd_demod_symbol(rxd_t *r, const samp_t *seg_i, const samp_t *seg_q,
                     int pos, int64_t cfo_word, int mu,
                     const int *window, int win_n, llr_t *llr, int64_t *rho);

/* INTERFERENCE WEIGHTING of a block's LLRs (the float model's
 * Transceiver.block_weights, the fixed model's WeightAcc -- bit-exact).
 * A carrier or a voice harmonic on one subcarrier is ~17 dB above the
 * per-carrier signal there and its LLRs are large, confident and wrong;
 * a syllable does the same to a run of symbols. Each symbol's residual
 * per carrier (decision-directed error of Y*conj(H) over |H|: the
 * noise-plus-interference amplitude in received units, scale 2^exp) is
 * accumulated per carrier on a running exponent and per symbol with its
 * own; once the block is complete every LLR is scaled by
 * min(1, (median/sum)^2) of its carrier and of its symbol, Q15, floored
 * at 1/64 -- only ever down, never to zero. */
typedef struct {
    int64_t acc[N_DATA_CARRIERS];
    int acc_e;
    int64_t sym[MAX_SYMS];
    int sym_e[MAX_SYMS];
    int n;
} rxd_wacc_t;
void rxd_wacc_reset(rxd_wacc_t *w);
void rxd_wacc_add(rxd_wacc_t *w, const int64_t *rho, int exp);
/* arr: n symbols x N_DATA_CARRIERS*mu LLRs, exponent-aligned */
void rxd_wacc_apply(const rxd_wacc_t *w, llr_t *arr, int mu);

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

/* EXCESS BINS of one detection block (the float model's
 * FullOFDMModem.EXC_X, the fixed model's block_excess): the top DET_N_EXC
 * in-band bins above DET_EXC_X x mean(in-band bins) (floor division by
 * B/2-1), and their excess over that clamp; the top bins are kept by an
 * ascending scan with strict replacement (ties -> lower bin). The stationary-carrier finders
 * (rx_find_tones here, the rolling one in rx_stream.c) count how often a
 * bin is in excess. Shared so the twins stay bit-exact. `pow` is one
 * block's power spectrum (any common scale); returns the count found. */
#define DET_EXC_X 4
#define DET_N_EXC 4
#define DET_MIN_TONE_BLOCKS 8
int det_block_excess(const int64_t *pow, int B, int *bins, int64_t *exc);

/* stationary carriers of a whole recording (frame at once): up to
 * NOTCH_MAX phase words, from the real samples' block spectra */
int rx_find_tones(link_mode_t mode, const int16_t *x, int n, uint32_t *words);
/* hilbert_analytic with those carriers notched out first */
void rx_excise_analytic(link_mode_t mode, const int16_t *x, int n,
                        samp_t *out_i, samp_t *out_q);
double rxd_tile_db(link_mode_t mode);
/* per-(mode, mu) output map for the SNR estimate: integer-estimator dB
 * in, float-reference dB out (rom_modes.h knots; see fixed/rx.py
 * SNR_MAP for the measurement story) */
double rxd_snr_map(link_mode_t mode, int mu, double est);

#endif /* OFDM_RX_INTERNAL_H */
