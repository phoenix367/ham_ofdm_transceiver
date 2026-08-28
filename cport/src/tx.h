/* Fixed-point transmitter -- C twin of ofdm_phy/fixed/tx.py.
 * Convolutional frames (ver=1); LDPC (ver=2) lands with ldpc.c. */
#ifndef OFDM_TX_H
#define OFDM_TX_H

#include <stdint.h>
#include "conv.h"

typedef enum { MODE_NORMAL = 0, MODE_ROBUST = 1, MODE_EXTREME = 2 } link_mode_t;
typedef enum { MOD_BPSK = 0, MOD_QPSK = 1, MOD_QAM16 = 2 } mod_type_t;

/* frame length in samples for a pkt_bits_n-bit packet (buffer sizing) */
int tx_frame_len(link_mode_t mode, int pkt_bits_n, mod_type_t mod,
                 cc_rate_t spd);
int tx_frame_len_ex(link_mode_t mode, int pkt_bits_n, mod_type_t mod,
                    cc_rate_t spd, int use_ldpc);

/* build a complete frame: preamble + BPSK/R13 header + data block.
 * pkt_bits: encoded packet (packets.h), typ: PKT_TYP_*. use_ldpc codes
 * the data block with the IRA LDPC (header ver=2). Returns the sample
 * count (== tx_frame_len_ex) or -1 on a bad argument. */
int tx_build_frame(link_mode_t mode, const uint8_t *pkt_bits, int pkt_bits_n,
                   int typ, mod_type_t mod, cc_rate_t spd, int16_t *out);
int tx_build_frame_ex(link_mode_t mode, const uint8_t *pkt_bits,
                      int pkt_bits_n, int typ, mod_type_t mod, cc_rate_t spd,
                      int use_ldpc, int16_t *out);

/* --- streamed bursts (docs/phy.md "Streaming bursts") -------------------
 * One preamble and one header for n_blocks equal-size packets, with the
 * preamble's ZC block re-emitted every `resync_every` blocks (0 = never)
 * to refresh timing and residual CFO. Amortizes the fixed cost that a
 * per-frame preamble pays every time: 1.73x for 20 x 27-byte NORMAL
 * packets, at 0.08 dB. Conv FEC only -- LDPC bursts have no use case
 * (the block sizes that make streaming pay are above LDPC's K=256). */
int tx_burst_len(link_mode_t mode, int pkt_bits_n, mod_type_t mod,
                 cc_rate_t spd, int n_blocks, int resync_every);

/* blocks: n_blocks * pkt_bits_n packet bits, contiguous, all the same
 * type and size (that is what lets one header describe them all).
 * Returns the sample count (== tx_burst_len) or -1 on a bad argument. */
int tx_build_burst(link_mode_t mode, const uint8_t *blocks, int pkt_bits_n,
                   int n_blocks, int typ, mod_type_t mod, cc_rate_t spd,
                   int resync_every, int16_t *out);

/* ---------------- streaming transmitter ----------------
 * Generates the waveform on demand instead of buffering a frame: the
 * caller pulls chunks and transmits them. Bit-identical to
 * tx_build_frame/tx_build_burst (asserted by the suites).
 *
 * resync_every == 0 with n_blocks == 1 builds a plain frame. */
#define TXS_RING 64

typedef struct txs_state txs_t;

txs_t *txs_open(link_mode_t mode, const uint8_t *blocks, int pkt_bits_n,
                int n_blocks, int typ, mod_type_t mod, cc_rate_t spd,
                int resync_every, int use_ldpc, int *total_out);
int txs_total(const txs_t *t);
/* Pull up to max samples; returns the count written, 0 when finished. */
int txs_pull(txs_t *t, int16_t *out, int max);

#endif /* OFDM_TX_H */
