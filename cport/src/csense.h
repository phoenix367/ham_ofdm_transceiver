#ifndef OFDM_CSENSE_H
#define OFDM_CSENSE_H

/* Energy carrier sense, extracted from the radio firmware so its state
 * machine can be TESTED on the host. Every constant here has a scar
 * attached (the 8 kB stress campaign):
 *
 *  - the floor's climb rate is per 40 ms WINDOW, not per call: at the
 *    firmware's 1 kHz call rate a per-call climb rose 40x too fast;
 *  - the warm-up gate covers the CONSUMER: evaluating the zeroed mean
 *    during ADC settling snapped the floor to its clamp and latched
 *    busy on a quiet wire until the rebase -- both boards' first-ever
 *    key-ups landed at CS_REBASE_MS to the millisecond;
 *  - CS_REBASE_MS must exceed the longest frame the STATION can emit
 *    (300 s > the 255-byte EXTREME worst case), not demoapp's 38 s;
 *  - feed it DC-FREE samples (dcblock.h): cs is mean SQUARE, and a
 *    peer DAC parked 0.83 V off mid-rail reads as 2.7e8 of permanent
 *    carrier while frames decode fine through it.
 *
 * The caller owns time: pass milliseconds to cs_busy(). cs_feed() is
 * ISR-safe (integer ring + one published mean). */

#include <stdint.h>

#define CS_WIN 480               /* 40 ms at 12 kHz */
#define CS_RATIO_SQ 9.0          /* busy when mean > 9x floor */
#define CS_REBASE_MS 300000u     /* > the longest emittable frame */
#define CS_WARMUP_MS 1000u       /* no verdicts before the ring is real */
#define CS_CLIMB_MS 40u          /* floor climb cadence (0.05%/window) */

typedef struct {
    int16_t ring[CS_WIN];
    int pos;
    int64_t acc;
    volatile uint32_t mean;      /* published by cs_feed for cross-context reads */
    double floor_;
    uint32_t busy_since_ms;      /* 0 = not in a busy run */
    uint32_t climb_ms;
} csense_t;

void cs_init(csense_t *c);
void cs_feed(csense_t *c, int16_t v);          /* one sample, ISR-safe */
int cs_busy(csense_t *c, uint32_t now_ms);     /* verdict + floor update */

#endif /* OFDM_CSENSE_H */
