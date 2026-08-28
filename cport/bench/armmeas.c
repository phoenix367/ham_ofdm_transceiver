/* The RAM figure in FEASIBILITY.md, made reproducible.
 *
 * Every static buffer in the port is sized for a worst case, so what an
 * image actually costs depends entirely on which entry points it
 * references: --gc-sections then drops the rest. A build that touches
 * the frame-at-once path carries megabytes the streaming path never
 * needs. Measuring by hand therefore drifts -- the number in the report
 * stopped matching what anyone could re-derive -- so the reference main
 * lives here instead of in a shell history.
 *
 * It references exactly what an MCU station does, which is what
 * demoapp/app.c references: the streaming transmitter, the streaming
 * receiver, and the station protocol on top of them. Nothing is
 * expected to decode -- the calls exist to hold the symbols live
 * against the linker.
 *
 *     make armmeas                    # as built: EXT frames enabled
 *     make armmeas ARMMEAS_DEFS=-DMAX_LLRS=1024
 *     make armmeas ARMMEAS_DEFS='-DARMMEAS_TX_ONLY -DOFDM_ARENA_BYTES=27000'
 *
 * The last one is the transmit-only image (a beacon, a telemetry
 * sender). It links none of the receiver, so the shared arena can shrink
 * to the transmit region -- see arena.h; the fit assertions make the
 * override safe.
 */
#include <stdio.h>

#include "station.h"
#include "rx_stream.h"
#include "tx.h"

#ifndef ARMMEAS_TX_ONLY
static station_t g_st;
static station_phy_t g_phy;
#endif

int main(void)
{
    int16_t buf[64];
    txs_t *t;
    int total = 0;

#ifndef ARMMEAS_TX_ONLY
    {
        rxs_event_t ev;
        rxs_t *r;

        station_init(&g_st, &g_phy, 1);
        station_submit(&g_st, (const uint8_t *)"x", 1, 0);
        station_poll_tx(&g_st, 0.0, 0, buf, 64);
        station_on_decoded(&g_st, (const uint8_t *)buf, 8, 0.0, 0.0, 0, 0.0);

        r = rxs_open(MODE_NORMAL, 0);
        rxs_push(r, buf, 64, &ev);
        rxs_flush(r, &ev);
        rxs_continue_burst(r, 0);
    }
#endif

    t = txs_open(MODE_NORMAL, (const uint8_t *)buf, 8, 1, 1,
                 MOD_BPSK, CC_R12, 0, 0, &total);
    txs_pull(t, buf, 64);

    printf("%d\n", total);
    return 0;
}
