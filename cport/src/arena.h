/* One scratch arena for the whole modem -- receiver AND transmitter.
 *
 * Two facts make the sharing safe, and they are the entire contract:
 *
 *   - the receiver's detect / demod / decode scratch is CALL-SCOPED.
 *     Nothing in it survives the return from rxs_push (or rxd_receive),
 *     which is why those three phases already overwrote each other.
 *   - the link is HALF DUPLEX. A station transmits or listens, never
 *     both. The streaming transmitter's generator state is the one
 *     thing here that IS live across calls -- from txs_open until the
 *     last txs_pull -- and half duplex is exactly the guarantee that no
 *     RX phase runs inside that span.
 *
 * So the arena is sized by the largest single phase, not by the sum.
 * Today that is the receiver's demod phase at 131584 B; the whole
 * transmitter needs 27000 B of it. Each region asserts its own fit at
 * compile time (rx_internal.h, rx_detect.c, tx.c) -- add a region and
 * the build tells you if it no longer fits.
 *
 * The half-duplex assumption is CHECKED, not trusted. Every receive
 * entry point stamps the owner tag; txs_pull refuses to continue a
 * generator whose state a receive phase has walked over, and reports it
 * through txs_faulted(). Aliasing bugs in this codebase have a habit of
 * looking like "the waveform quietly went wrong" (see FEASIBILITY.md,
 * "Narrowing a type is where the bugs live"), and a transmission that
 * stops is a far cheaper failure than one that transmits noise.
 *
 * The check is one integer compare per pull call, not per sample.
 */
#ifndef OFDM_ARENA_H
#define OFDM_ARENA_H

#include <stdint.h>

/* Sized in BYTES, because the phases mix sample (samp_t), LLR (llr_t)
 * and accumulator (int64) widths. The union forces int64 alignment for
 * the accumulator views.
 *
 *   rx detect 125956   rx demod 131584   rx decode 131072   tx 27000
 */
/* Overridable, because the right size is "the largest region this image
 * actually links". A transmit-only build compiles none of the receiver's
 * regions and needs 27000; leaving it at the receiver's figure would
 * cost such a build 100 kB it never touches. The knob is safe because
 * every region asserts its own fit at compile time -- set it too small
 * and the build fails, naming the region that no longer fits. */
#ifndef OFDM_ARENA_BYTES
#define OFDM_ARENA_BYTES 131584
#endif

extern union ofdm_arena_u {
    int64_t align;
    unsigned char b[OFDM_ARENA_BYTES];
} ofdm_arena_store;
#define ofdm_arena (ofdm_arena_store.b)

enum { ARENA_NOBODY = 0, ARENA_RX, ARENA_TX };

extern int ofdm_arena_owner;

/* Stamp the arena's current owner. Receive paths call this on entry;
 * the transmitter calls it in txs_open and re-checks on every pull. */
static inline void arena_claim(int who) { ofdm_arena_owner = who; }
static inline int arena_held_by(int who) { return ofdm_arena_owner == who; }

#endif /* OFDM_ARENA_H */
