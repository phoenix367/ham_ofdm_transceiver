/* MCU streaming receiver architecture (plan §4): bounded ring buffer,
 * per-block tone summaries with a running contrast metric and causal
 * peak-commit, bounded ZC stage, symbol-by-symbol demodulation -- no
 * frame-at-once buffering.
 *
 * Divergence from the frame-at-once reference, by necessity (causality):
 * the tone stage aligns block exponents and takes its median floor over
 * the sliding detection window instead of the whole capture, and commits
 * on a local metric peak instead of a global argmax. Everything after the
 * tone hit (ZC timing, CFO refinement, demod, decode) is the same
 * arithmetic as rx_demod/rx_detect. Validated by result equivalence on
 * the golden corpus. Single instance (module-static buffers). */
#ifndef OFDM_RX_STREAM_H
#define OFDM_RX_STREAM_H

#include <stdint.h>
#include "rx_demod.h"

typedef struct {
    int type; /* 1 = frame decoded; -1 header CRC, -2 bad ver, -3 data CRC */
    rxd_header_t hdr;
    uint8_t bits[2600]; /* up to 2076 bits for a 255-byte EXT frame */
    int pkt_bits_n;      /* decoded packet bit count (type-resolved) */
    int start_abs;
    int64_t cfo_word;
    double snr_db;
} rxs_event_t;

typedef struct rxs_state rxs_t; /* opaque; single instance */

/* Shared raw sample ring: ONE int16 ring serves every instance (all
 * receivers listen to the same audio; each push writes the same values
 * idempotently).
 *
 * What sets the size is NOT the preamble -- it is how late the tone
 * detector commits. Both consumers of the tone peak (the lag-N residual
 * in tone_commit, and the ZC scan in S_ZC_WAIT) anchor at `cs_abs`, the
 * START of the tone field; and on a channel where the metric never
 * falls cleanly back below threshold the commit waits for the argmax to
 * be stable for a full window span past the best block (rx_stream.c,
 * "commit when the above-threshold region ends, OR ..."). The best block
 * is already one window past cs_abs, so the write head ends up TWO tone
 * fields ahead of the sample the receiver then reaches back for:
 *
 *   lookback = (2 * 3*T*FFT_BINS/B + DECLINE_BLOCKS) * B
 *              + (HILBERT_TAPS_N - 1)
 *
 * which is exact for all three modes -- 8510 / 32318 / 124478, matching
 * rxs_ring_hwm to the sample (test_stream prints it). EXTREME therefore
 * wants 124478, and 147456 is that with 18% margin: deliberately not a
 * power of two, since 2^17 would leave only 5%.
 *
 * So the ring is 288 kB because of a detector TIMEOUT, not because a
 * frame is long. Halving it means making the lag-N estimate incremental
 * (it is a re-read of the tone field, which the block summaries already
 * traverse) and anchoring the ZC scan near the tone field's END rather
 * than its start -- both touch acquisition sensitivity, which is why
 * neither has been done casually. Overridable meanwhile. */
#ifndef RXS_RAW_RING_LEN
#define RXS_RAW_RING_LEN 147456
#endif

/* diagnostic: deepest ring lookback observed (samples behind the write
 * head) -- the empirical minimum ring size for this mode/traffic */
int64_t rxs_ring_hwm(const rxs_t *r);

/* Reads refused because the samples had already been overwritten (or had
 * not been written yet). Always 0 on a receiver that keeps up; any
 * non-zero value means decodes were abandoned rather than run on stale
 * ring contents, and the ring wants sizing up for this workload. */
int64_t rxs_ring_miss(const rxs_t *r);

rxs_t *rxs_open(link_mode_t mode, int calibrate);

/* feed samples in arbitrary chunks; returns 1 when ev was filled */
int rxs_push(rxs_t *r, const int16_t *chunk, int n, rxs_event_t *ev);

/* end-of-stream: pad one symbol of silence (the model's tail-slip pad) */
int rxs_flush(rxs_t *r, rxs_event_t *ev);

/* streamed bursts: after an event whose packet says more blocks follow
 * (the link layer's marker -- the PHY does not read packet payloads),
 * arm the decoder to take the next block from the deterministic offset
 * after the one just finished instead of hunting for a preamble.
 * resync_every must match the transmitter's. Returns 0 if there is no
 * block boundary to continue from. */
int rxs_continue_burst(rxs_t *r, int resync_every);

#endif /* OFDM_RX_STREAM_H */
