/* Broadcast (non-ARQ) mode -- C twin of ofdm_phy/broadcast.py.
 *
 * A broadcast is a streamed burst with the acknowledgment machinery
 * subtracted: no ack request, no window, no selective repeat, no reply
 * timer. What it adds is a way for a receiver that was not listening at
 * the start to join -- every GROUP re-sends the preamble and opens with
 * a SYNC frame carrying the stream descriptor.
 *
 *   [preamble][header][f0 SYNC][f1]..[fN]  [preamble][header][f0 SYNC]..
 *
 * The transmitter needs no new code: one group IS tx_build_burst() over
 * PKT_TYP_DATA frames. Only the receive walk is new.
 *
 * Framing, two bytes per frame:
 *   byte 0  bit 7 SYNC   opens a group; the descriptor byte follows the
 *                        length
 *           bit 6 EOS    last frame of the broadcast
 *           bits 5-0     sequence mod 64 -- loss statistics ONLY, there
 *                        is no retransmission
 *   byte 1           valid payload bytes in THIS frame
 *
 * The per-frame length costs ~4% at 26-byte frames and makes every frame
 * self-delimiting; carrying it only on the EOS frame would mis-size the
 * payload whenever that one frame is the one lost.
 */
#ifndef OFDM_BROADCAST_H
#define OFDM_BROADCAST_H

#include <stdint.h>
#include "rx_demod.h"

#define BC_SYNC 0x80
#define BC_EOS 0x40
#define BC_SEQ_MASK 0x3F
#define BC_SEQ_MOD 64
#define BC_HEAD 2       /* flags+seq, length */

/* payload types carried in the SYNC descriptor */
#define BC_PT_TELEMETRY 0
#define BC_PT_CODEC2_700 1
#define BC_PT_CODEC2_450 2
#define BC_PT_OPAQUE 15

#ifndef BC_MAX_GROUP
#define BC_MAX_GROUP 8  /* frames per group (sizes a static buffer) */
#endif

/* What a receiver can say about a broadcast it heard -- in non-ARQ mode
 * this is the entire feedback path.
 *
 * frames_lost counts SEQUENCE GAPS between decoded frames, so it covers
 * the acquired span only: groups missed before the first acquisition or
 * after the last are invisible here. The sender knows what it sent and
 * can compare. */
typedef struct {
    int groups;       /* stream groups acquired */
    int frames_ok;    /* frames decoded */
    int frames_lost;  /* inferred from sequence gaps */
    int bytes_out;
    int ptype;        /* payload type from the SYNC descriptor, -1 unknown */
    int group;        /* frames per group, read from the same descriptor */
    int saw_eos;
    double snr_sum;   /* over decoded frames; divide by frames_ok */
} bc_stats_t;

/* Decode every group in a recording and reassemble the payload.
 *
 *   samples/n  : int16 audio
 *   group      : frames per group, as the transmitter built them
 *   out/out_cap: reassembled payload
 * Returns bytes written, or -1 on a bad argument.
 *
 * The detector takes a GLOBAL argmax over whatever slice it is given, so
 * this walks the recording with slices bounded to hold exactly one
 * preamble. Handing it more lets it lock the strongest preamble rather
 * than the first, silently losing every group before that one. */
int bc_receive(link_mode_t mode, const int16_t *samples, int n, int group,
               uint8_t *out, int out_cap, bc_stats_t *st);

#endif /* OFDM_BROADCAST_H */
