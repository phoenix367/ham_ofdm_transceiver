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
 * What sets the size is how far back the receiver reaches when the tone
 * detector finally commits -- which is about TWO tone fields after the
 * peak, because the detector takes the argmax over the whole
 * above-threshold region and has to see that region end.
 *
 * Both consumers of the tone peak USED to anchor at `cs_abs`, the START
 * of the tone field, which made the reach-back two full tone fields:
 *
 *   was:  (2 * 3*T*FFT_BINS/B + DECLINE_BLOCKS) * B + (HILBERT_TAPS_N-1)
 *         = 8510 / 32318 / 124478 samples
 *
 * Neither does any more. The lag-N residual is accumulated per block
 * during the tone scan and summed at commit (see `lag_sum_t`), so the
 * tone field is never re-read; and the ZC scan is anchored at the tone
 * field's END, where the ZC actually is, rather than searching forward
 * from cs_abs across the whole field. What remains is one tone field
 * plus the search margin:
 *
 *   now:  3*T*FFT_BINS + ZC_ANCHOR_MARGIN_BLK*B + (DECLINE_BLOCKS+1)*B
 *                      + (HILBERT_TAPS_N - 1)
 *         = 6718 / 21054 / 67134 samples   (measured, test_stream prints
 *                                           rxs_ring_hwm per mode)
 *
 * 81920 is EXTREME's 67134 with 22 % margin. Deliberately not a power of
 * two only by accident now -- it is 80*1024, which simply fits.
 *
 * The other reason this ring exists is capacity, not lookback: the
 * acquisition burst does not run in real time on a 480 MHz M7, so the
 * receiver falls behind and catches up from here. That lag ADDS to the
 * reach-back above; see FEASIBILITY.md, which measures both. */
#ifndef RXS_RAW_RING_LEN
#define RXS_RAW_RING_LEN 81920
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
