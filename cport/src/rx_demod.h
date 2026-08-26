/* Genie-synced receiver demodulator -- C twin of the demod half of
 * ofdm_phy/fixed/rx.py (detection lands with rx_detect.c). Includes the
 * gated two-stage frequency search, the slew-limited tracker, pilot
 * channel estimation, BPSK/QPSK/16-QAM matched-filter LLRs, the
 * per-modulation quantizers and the conv decode path (LDPC pending). */
#ifndef OFDM_RX_DEMOD_H
#define OFDM_RX_DEMOD_H

#include <stdint.h>
#include "tx.h" /* link_mode_t, mod_type_t */

typedef struct {
    link_mode_t mode;
    int sym_tile;
    int symbol_len;
    const int32_t *search_words;
    int n_words;
    int last_hyp; /* -1 = tracker unset (first symbol of a frame) */
    int coarse_enabled;
    int coarse_gate_q4;
    int calibrate; /* header alpha fit + reliability ROM (MU<=2) */
    /* per-frame outputs of the last successful receive */
    double last_snr_db;
    int last_harq_combined;
} rxd_t;

typedef struct {
    int ver, typ, mod, spd, len;
} rxd_header_t;

void rxd_init(rxd_t *r, link_mode_t mode);

/* demodulate + decode a frame at a known (start, cfo_word): int16 audio in,
 * packet bits out (hdr->len of them). Returns 0, or -1 on header CRC,
 * -2 on unsupported ver, -3 on data CRC. */
int rxd_receive_genie(rxd_t *r, const int16_t *samples, int n_samples,
                      int start, int64_t cfo_word,
                      rxd_header_t *hdr, uint8_t *pkt_bits);

/* full receive: detection (rx_detect.c) + demod + decode. Returns the
 * genie codes above, or -4 on no preamble lock. start/cfo_word report
 * the detection result (may be NULL). */
int rxd_receive(rxd_t *r, const int16_t *samples, int n_samples,
                rxd_header_t *hdr, uint8_t *pkt_bits,
                int *start_out, int64_t *cfo_word_out);

/* genie receive with HARQ chase combining: prev_llrs (from a previous
 * failed attempt) are LLR-summed on a fresh-decode CRC failure; the fresh
 * quantized data LLRs are exported via llrs_out/llrs_n (also on failure,
 * for the next attempt). Any of the extra pointers may be NULL. */
int rxd_receive_genie_harq(rxd_t *r, const int16_t *samples, int n_samples,
                           int start, int64_t cfo_word,
                           rxd_header_t *hdr, uint8_t *pkt_bits,
                           const int64_t *prev_llrs, int prev_n,
                           int64_t *llrs_out, int *llrs_n_out);

/* streamed burst receive -- twin of tx_build_burst (docs/phy.md).
 * Detects and decodes the header ONCE, then walks n_blocks data blocks at
 * deterministic offsets, re-locking timing and CFO on the ZC symbols the
 * transmitter interleaves every `resync_every` blocks (0 = none).
 *
 *   pkt_bits : n_blocks * PKT_BITS_FROM_HDR(typ, len) bits, block-major
 *   ok_flags : per-block CRC result (1 = delivered); may be NULL
 *
 * Returns the number of blocks delivered (0..n_blocks), or -1 header CRC,
 * -2 unsupported ver (bursts are conv-only), -4 no preamble lock. A block
 * that fails costs only itself -- the next offset is deterministic.
 * No HARQ yet: per-block chase combining lands with the link-layer
 * integration (the float chain already carries it, see BlockStats.llrs). */
int rxd_receive_burst(rxd_t *r, const int16_t *samples, int n_samples,
                      int n_blocks, int resync_every, rxd_header_t *hdr,
                      uint8_t *pkt_bits, int *ok_flags, int *start_out,
                      int64_t *cfo_word_out, int *n_resync_out);

#endif /* OFDM_RX_DEMOD_H */
