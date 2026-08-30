/* Host scenario tests for carrier sense -- every scenario here is a
 * failure that actually happened on the two-board stand, replayed
 * against the real csense.c + dcblock.h. The decode path proved robust
 * to input abuse (burst_repro's matrix decodes 8/8 bare); carrier
 * sense is where every real failure lived, so carrier sense is what
 * gets the regression suite. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "csense.h"
#include "dcblock.h"

static int g_pass, g_fail;
static void check(const char *name, int ok)
{
    printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (ok) g_pass++; else g_fail++;
}

/* drive n_ms of a given sample generator at 12 kHz + 1 kHz cs_busy
 * calls, mirroring the firmware's cadence. Returns ms spent busy. */
typedef int16_t (*gen_t)(long n, void *ctx);
static uint32_t drive(csense_t *c, dcblock_t *b, uint32_t *ms, gen_t gen,
                      void *ctx, uint32_t n_ms, int *last_busy)
{
    uint32_t busy_ms = 0;
    long i;
    for (i = 0; i < (long)n_ms * 12; i++) {
        int16_t v = gen(i, ctx);
        if (b)
            v = dcblock_step(b, v);
        if (*ms > 500)           /* the firmware's producer warm-up */
            cs_feed(c, v);
        if (i % 12 == 11) {
            (*ms)++;
            *last_busy = cs_busy(c, *ms);
            if (*last_busy)
                busy_ms++;
        }
    }
    return busy_ms;
}

static int16_t gen_quiet(long n, void *ctx)
{ (void)ctx; return (int16_t)(-143 + (n * 1103515245 >> 16) % 61 - 30); }
static int16_t gen_park(long n, void *ctx)
{ (void)n; return (int16_t)(intptr_t)ctx; }
static int16_t gen_frame(long n, void *ctx)
{ (void)ctx; return (int16_t)(16000.0 * sin(2 * M_PI * 1500.0 * n / 12000.0)
                              * (0.7 + 0.3 * sin(2 * M_PI * 37.0 * n / 12000.0))); }

int main(void)
{
    csense_t cs;
    dcblock_t db;
    uint32_t ms;
    int busy;

    /* 1. boot latch: garbage warm-up then a quiet wire must read idle.
     * The measured failure: the zeroed mean snapped the floor to its
     * clamp and a quiet wire read busy for exactly CS_REBASE_MS. */
    cs_init(&cs); dcblock_init(&db); ms = 0; busy = 1;
    drive(&cs, &db, &ms, gen_quiet, 0, 5000, &busy);
    check("boot into quiet wire reads idle within 5 s", busy == 0);

    /* 2. the parked peer DAC (measured -16577 = 0.81 V) THROUGH the
     * blocker: must read idle -- the whole point of dcblock. */
    cs_init(&cs); dcblock_init(&db); ms = 0; busy = 1;
    drive(&cs, &db, &ms, gen_park, (void *)(intptr_t)-16577, 5000, &busy);
    check("parked DC through dcblock reads idle", busy == 0);

    /* 3. the same park BARE latches busy until the rebase -- the
     * documented reason the blocker is not optional. */
    cs_init(&cs); ms = 0; busy = 0;
    drive(&cs, 0, &ms, gen_quiet, 0, 3000, &busy);       /* settle quiet */
    {
        uint32_t busy_ms = drive(&cs, 0, &ms, gen_park,
                                 (void *)(intptr_t)-16577, 30000, &busy);
        check("bare parked DC latches busy (why dcblock exists)",
              busy_ms > 25000);
    }

    /* 4. a real frame must read BUSY promptly and STAY busy through
     * 45 s -- BURST_FRAG_MAX_AIR_S, the longest frame the station may
     * now emit. (The floor climb crosses the 9x threshold at ~176 s of
     * continuous signal: that is WHY frames are capped at 45 s.) */
    cs_init(&cs); dcblock_init(&db); ms = 0; busy = 0;
    drive(&cs, &db, &ms, gen_quiet, 0, 3000, &busy);
    {
        uint32_t busy_ms = drive(&cs, &db, &ms, gen_frame, 0, 45000, &busy);
        check("45 s frame stays busy end to end (>= 44 s busy)",
              busy_ms >= 44000 && busy == 1);
    }
    drive(&cs, &db, &ms, gen_quiet, 0, 2000, &busy);
    check("idle returns within 2 s of the frame ending", busy == 0);

    /* 5. frame/gap cycling: busy tracks the duty cycle, no ratchet. */
    cs_init(&cs); dcblock_init(&db); ms = 0; busy = 0;
    drive(&cs, &db, &ms, gen_quiet, 0, 3000, &busy);
    {
        int k, ok = 1;
        for (k = 0; k < 5; k++) {
            uint32_t bm = drive(&cs, &db, &ms, gen_frame, 0, 19000, &busy);
            if (bm < 18000) ok = 0;            /* frame must read busy */
            drive(&cs, &db, &ms, gen_quiet, 0, 5000, &busy);
            if (busy) ok = 0;                  /* gap must read idle */
        }
        check("5x 19 s frame + 5 s gap cycles track busy/idle", ok);
    }

    /* 6. a DC step (peer re-parks, bias cap charging) costs at most a
     * brief transient, never a latch. */
    cs_init(&cs); dcblock_init(&db); ms = 0; busy = 0;
    drive(&cs, &db, &ms, gen_quiet, 0, 3000, &busy);
    {
        uint32_t busy_ms = drive(&cs, &db, &ms, gen_park,
                                 (void *)(intptr_t)14000, 10000, &busy);
        check("DC step: transient busy < 2 s, then idle",
              busy_ms < 2000 && busy == 0);
    }

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
