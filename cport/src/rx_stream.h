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

/* shared raw sample ring: ONE int16 ring serves every instance (all
 * receivers listen to the same audio; each push writes the same values
 * idempotently). Sized ~1.18x the measured worst-case lookback
 * (rxs_ring_hwm: 124416 + 62 FIR history, EXTREME). Overridable, and
 * deliberately not a power of two -- 2^17 leaves only 5% margin. */
#ifndef RXS_RAW_RING_LEN
#define RXS_RAW_RING_LEN 147456
#endif

/* diagnostic: deepest ring lookback observed (samples behind the write
 * head) -- the empirical minimum ring size for this mode/traffic */
int64_t rxs_ring_hwm(const rxs_t *r);

rxs_t *rxs_open(link_mode_t mode, int calibrate);

/* feed samples in arbitrary chunks; returns 1 when ev was filled */
int rxs_push(rxs_t *r, const int16_t *chunk, int n, rxs_event_t *ev);

/* end-of-stream: pad one symbol of silence (the model's tail-slip pad) */
int rxs_flush(rxs_t *r, rxs_event_t *ev);

#endif /* OFDM_RX_STREAM_H */
