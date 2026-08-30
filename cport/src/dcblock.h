#ifndef OFDM_DCBLOCK_H
#define OFDM_DCBLOCK_H

/* One-pole DC blocker for the receive front end.
 *
 * The receiver's demodulation is DC-blind (bin 0 is unused), but
 * everything AROUND it was not: energy carrier sense reads mean square,
 * so a DC operating point -- a parked peer DAC, a bias network, a
 * ground offset -- lands straight in the busy logic. The 8 kB stress
 * campaign measured the damage: a peer DAC parked 0.83 V off mid-rail
 * put a constant 2.7e8 on carrier sense while frames decoded fine
 * through it. The front end must make DC someone else's non-problem:
 * filter ONCE at the input, and every consumer (capture, carrier
 * sense) sees a zero-mean signal whatever the analog side does.
 *
 * Structure: a leaky integrator tracks the DC, the output is input
 * minus the track -- a first-order high-pass.
 *
 *     dc  += (x - dc) >> DCBLOCK_SHIFT      (in Q-extended precision)
 *     y    = x - dc
 *
 * DCBLOCK_SHIFT 8 gives a time constant of 256 samples = 21.3 ms at
 * 12 kHz, i.e. a -3 dB corner near fs/(2*pi*256) = 7.5 Hz. The OFDM
 * band starts two decades above, so the passband cost is nil, and a
 * DC step (a peer re-parking between frames) is absorbed in ~100 ms.
 * The tracker state is Q(DCBLOCK_SHIFT) so the subtraction cancels the
 * DC exactly in the steady state instead of leaving an LSB limit
 * cycle.
 *
 * Integer-only, two adds and two shifts per sample: fits the 12 kHz
 * ISR with nothing to measure. */

#include <stdint.h>

#define DCBLOCK_SHIFT 8

typedef struct {
    int32_t dc_q; /* DC estimate in Q(DCBLOCK_SHIFT) */
} dcblock_t;

static inline void dcblock_init(dcblock_t *b) { b->dc_q = 0; }

static inline int16_t dcblock_step(dcblock_t *b, int16_t x)
{
    int32_t y;
    b->dc_q += (((int32_t)x << DCBLOCK_SHIFT) - b->dc_q) >> DCBLOCK_SHIFT;
    y = (int32_t)x - (b->dc_q >> DCBLOCK_SHIFT);
    if (y > 32767)
        y = 32767;
    if (y < -32768)
        y = -32768;
    return (int16_t)y;
}

#endif /* OFDM_DCBLOCK_H */
