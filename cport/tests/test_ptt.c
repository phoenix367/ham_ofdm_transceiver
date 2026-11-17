/* PTT keying (src/ptt.h): lead, tail and the stall watchdog, driven
 * through a model of the radio firmware's transmit path -- a FIFO the
 * main loop pre-fills and tops up (tx_fill) and a 12 kHz ISR that drains
 * it into the DAC (tim6_isr) -- so the tests exercise the same call
 * order usb_radio_main.c uses, and the pin is recorded per tick. */
#include <stdio.h>
#include <string.h>

#define PTT_PIN_HOOK test_pin
#include "../src/ptt.h"

static int g_pass, g_fail;
static void check(const char *name, int ok)
{
    printf("  %-66s %s\n", name, ok ? "ok" : "FAIL");
    if (ok) g_pass++; else g_fail++;
}

/* ---- the pin ---- */
static int g_pin = -1, g_pin_writes;
void test_pin(int on) { g_pin = on; g_pin_writes++; }

/* ---- the firmware's transmit path, in miniature ---- */
#define TXF_N 2048
#define OUT_MAX 200000
static int16_t g_txf[TXF_N];
static uint32_t g_txf_w, g_txf_r;
static int g_tx_on;
static ptt_t g_ptt;

static int g_gen_left, g_gen_next, g_gen_open;  /* generator: 1, 2, 3, ... */
static long g_pulled;

static int16_t g_out[OUT_MAX];   /* DAC, one entry per keyed tick */
static int g_out_n, g_keyed_ticks, g_underruns;

static void tx_fill(uint32_t slack)
{
    while (g_gen_open && (uint32_t)(g_txf_w - g_txf_r) + slack < TXF_N) {
        int wi = (int)(g_txf_w & (TXF_N - 1));
        int room = (int)(TXF_N - (g_txf_w - g_txf_r));
        int lin = TXF_N - wi, got, i;
        if (room > lin)
            room = lin;
        if (room <= 0)
            break;
        got = g_gen_left < room ? g_gen_left : room;
        if (got <= 0) {
            int fl = ptt_tail(&g_ptt, g_txf + wi, room);
            if (fl > 0) {
                g_txf_w += (uint32_t)fl;
                continue;
            }
            g_gen_open = 0;
            break;
        }
        for (i = 0; i < got; i++)
            g_txf[wi + i] = (int16_t)(g_gen_next++);
        g_gen_left -= got;
        g_txf_w += (uint32_t)got;
        g_pulled += got;
    }
}

static void isr_tick(void)
{
    if (!g_tx_on)
        return;
    g_keyed_ticks += g_pin == 1;
    if (ptt_in_lead(&g_ptt)) {
        g_out[g_out_n++] = 0;                 /* mid-rail */
    } else if (g_txf_r != g_txf_w) {
        g_out[g_out_n++] = g_txf[g_txf_r & (TXF_N - 1)];
        g_txf_r++;
        ptt_fed(&g_ptt);
    } else {
        g_out[g_out_n++] = 0;
        g_underruns++;
        ptt_starved(&g_ptt);
    }
}

/* one transmission of n samples; the main loop runs between ticks */
static void transmit(int n)
{
    g_gen_left = n;
    g_gen_next = 1;
    g_gen_open = 1;
    g_pulled = 0;
    g_out_n = g_keyed_ticks = g_underruns = 0;
    g_txf_r = g_txf_w = 0;          /* tx_begin() */
    ptt_arm(&g_ptt);
    tx_fill(0);                     /* pre-fill */
    ptt_key_up(&g_ptt);             /* tx_key_up() */
    g_tx_on = 1;
    while (g_tx_on && g_out_n < OUT_MAX - 1) {
        isr_tick();
        if (g_gen_open)
            tx_fill(256);
        if (!g_gen_open && g_txf_r == g_txf_w) {
            g_tx_on = 0;            /* every sample has left the DAC */
            ptt_key_down(&g_ptt);
        }
    }
}

/* the output is exactly `lead` mid-rail ticks, the frame 1..n, then
 * `tail` mid-rail ticks */
static int shape_ok(int lead, int n, int tail)
{
    int i;
    if (g_out_n != lead + n + tail)
        return 0;
    for (i = 0; i < lead; i++)
        if (g_out[i] != 0) return 0;
    for (i = 0; i < n; i++)
        if (g_out[lead + i] != (int16_t)(i + 1)) return 0;
    for (i = 0; i < tail; i++)
        if (g_out[lead + n + i] != 0) return 0;
    return 1;
}

int main(void)
{
    printf("test_ptt: PB14 keying, lead/tail and the stall watchdog\n");

    ptt_init(&g_ptt);
    ptt_pin_init();
    check("init drives the RELEASED level", g_pin == 0 && g_pin_writes == 1);

    /* ---- defaults: the pin follows the frame, nothing else changes ---- */
    transmit(5000);
    check("default 0/0: DAC output is bit-identical to the frame",
          shape_ok(0, 5000, 0) && g_underruns == 0);
    check("default 0/0: keyed for exactly the frame's ticks, released after",
          g_keyed_ticks == 5000 && g_pin == 0 && g_ptt.keyups == 1);
    check("default 0/0: the firmware's own keyed-time clock reads 5000",
          g_ptt.last_on_ticks == 5000);

    /* ---- lead: the frame starts later, it is not cut ---- */
    ptt_config(&g_ptt, 200, 0);
    transmit(5000);
    check("lead 200 ms: 2400 mid-rail ticks, then the WHOLE frame",
          shape_ok(2400, 5000, 0) && g_underruns == 0);
    check("lead 200 ms: keyed 2400 + 5000 ticks, no underrun counted",
          g_keyed_ticks == 7400 && g_pin == 0);

    /* ---- tail: mid-rail after the frame, not counted as generator ---- */
    ptt_config(&g_ptt, 0, 50);
    transmit(5000);
    check("tail 50 ms: the frame, then 600 mid-rail ticks",
          shape_ok(0, 5000, 600));
    check("tail samples are not generator samples (tx_short stays honest)",
          g_pulled == 5000);

    /* a frame shorter than the FIFO is exhausted INSIDE the pre-fill,
     * before key-up: the tail must already be owed by then */
    transmit(300);
    check("short frame (exhausted in the pre-fill) still gets its tail",
          shape_ok(0, 300, 600) && g_pin == 0);

    ptt_config(&g_ptt, 150, 40);
    transmit(9000);
    check("lead 150 + tail 40 around a 9000-sample frame",
          shape_ok(1800, 9000, 480) && g_keyed_ticks == 1800 + 9000 + 480);
    check("keyed-time clock reads lead + frame + tail",
          g_ptt.last_on_ticks == 1800 + 9000 + 480);

    /* ---- config clamps and round trip ---- */
    ptt_config(&g_ptt, -5, 99999);
    check("config clamps to 0..2000 ms",
          ptt_lead_ms(&g_ptt) == 0 && ptt_tail_ms(&g_ptt) == PTT_MAX_MS);
    ptt_config(&g_ptt, 250, 75);
    check("config round-trips in ms",
          ptt_lead_ms(&g_ptt) == 250 && ptt_tail_ms(&g_ptt) == 75);
    ptt_config(&g_ptt, 0, 0);

    /* ---- stall watchdog ---- */
    {
        int i, released_at = -1;
        uint32_t cuts0 = g_ptt.cuts;
        g_txf_r = g_txf_w = 0;
        ptt_arm(&g_ptt);
        ptt_key_up(&g_ptt);
        g_tx_on = 1;
        g_out_n = g_underruns = 0;
        /* the main loop has died with the FIFO empty */
        for (i = 0; i < 2 * (int)PTT_STALL_TICKS && i < OUT_MAX - 1; i++) {
            isr_tick();
            if (released_at < 0 && g_pin == 0)
                released_at = i + 1;
        }
        check("stall: PTT dropped after exactly 1 s of starved DAC",
              released_at == (int)PTT_STALL_TICKS
              && g_ptt.cuts == cuts0 + 1);
        /* feeding resumes: the release holds until the NEXT key-up */
        g_txf[0] = 7;
        g_txf_w = g_txf_r + 1;
        isr_tick();
        check("stall: stays released though samples arrive again",
              g_pin == 0 && g_ptt.cuts == cuts0 + 1);
        g_tx_on = 0;
        ptt_key_down(&g_ptt);
        transmit(1000);
        check("stall: the next transmission keys normally",
              shape_ok(0, 1000, 0) && g_keyed_ticks == 1000
              && g_ptt.cuts == cuts0 + 1);
    }
    {
        /* a starved stretch one tick short of the limit is NOT a stall,
         * and feeding resets the count */
        int i;
        uint32_t cuts0 = g_ptt.cuts;
        g_txf_r = g_txf_w = 0;
        ptt_key_up(&g_ptt);
        g_tx_on = 1;
        g_out_n = 0;
        for (i = 0; i < (int)PTT_STALL_TICKS - 1; i++)
            isr_tick();
        g_txf[g_txf_w & (TXF_N - 1)] = 1;
        g_txf_w++;
        isr_tick();                       /* fed: count resets */
        for (i = 0; i < (int)PTT_STALL_TICKS - 1; i++)
            isr_tick();
        check("starved 1 tick short of 1 s, twice, fed between: still keyed",
              g_pin == 1 && g_ptt.cuts == cuts0);
        g_tx_on = 0;
        ptt_key_down(&g_ptt);
    }

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
