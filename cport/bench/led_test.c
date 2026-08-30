/* Is the LED on PA0? A RAM-resident blink sketch that answers it.
 *
 * Runs from RAM over JTAG, so the flashed radio firmware is untouched
 * and a `make reset` (or a power-cycle) brings it back.
 *
 * The pattern is deliberately NOT a plain blink: a plain blink cannot
 * be told from a power LED, a heartbeat someone else is driving, or a
 * pin that happens to float at the right level. Four phases repeat,
 * and the beacon says which one is running right now, so what the eye
 * sees can be checked against what the board says it is doing:
 *
 *   phase 0   6 s   1 Hz             unmistakably deliberate
 *   phase 1   3 s   SOLID ON         the pin can hold a level
 *   phase 2   3 s   SOLID OFF        and the other one
 *   phase 3   6 s   5 Hz             and switch fast
 *
 * Before any of that it probes the pin as an input, once with a
 * pull-up and once with a pull-down, and records both. That does not
 * identify an LED (a 40 k internal pull cannot light one, and a
 * floating pin follows the pull either way) but it does catch the case
 * that matters: a pin something ELSE is driving disagrees with the
 * pull, and must not be driven push-pull.
 *
 * Beacon at 0x20000000, read with `mdw` while it runs -- mdw does not
 * halt the core.
 */
#include <stdint.h>

#include "led.h"

#define RCC_CFGR (*(volatile uint32_t *)0x58024410u)
#define SYST_CSR (*(volatile uint32_t *)0xE000E010u)
#define SYST_RVR (*(volatile uint32_t *)0xE000E014u)
#define SYST_CVR (*(volatile uint32_t *)0xE000E018u)
#define SYST_COUNTFLAG (1u << 16)

#define LED_BEACON_MAGIC 0x11EDBEEFu

typedef struct {
    uint32_t magic;        /* 0x11EDBEEF once main runs */
    uint32_t sws;          /* RCC_CFGR SWS: 3 = PLL1, 0 = HSI */
    uint32_t cpu_khz;      /* what the delays are calibrated against */
    uint32_t pin;          /* port letter in ASCII << 8 | pin: 0x4100 = PA0 */
    uint32_t idr_pullup;   /* pin as input, pull-up:   0 or 1 */
    uint32_t idr_pulldown; /* pin as input, pull-down: 0 or 1 */
    uint32_t phase;        /* 0..3, see above */
    uint32_t cycles;       /* full four-phase repetitions */
    uint32_t ms;           /* milliseconds since start -- climbs = alive */
    uint32_t edges;        /* LED state changes */
    uint32_t level;        /* last level written (1 = lit) */
    uint32_t moder;        /* GPIO MODER readback: proves the config took */
} led_beacon_t;

volatile led_beacon_t g_beacon __attribute__((section(".results"), used));

static void tick_init(void)
{
    /* A RAM image inherits the clock of whatever ran before it. After a
     * SYSRESETREQ the flashed firmware has already switched sysclk to
     * PLL1 at 400 MHz by the time the loader halts the core; on a bare
     * board it would still be HSI at 64 MHz. Getting this wrong only
     * changes the blink RATE, but a 6x-fast blink is exactly the kind
     * of thing that gets read as "the pin is not the LED". */
    uint32_t sws = (RCC_CFGR >> 3) & 7u;
    uint32_t hz = sws == 3u ? 400000000u : 64000000u;
    g_beacon.sws = sws;
    g_beacon.cpu_khz = hz / 1000u;
    SYST_RVR = hz / 1000u - 1u;      /* 1 ms; 400000 fits the 24-bit reload */
    SYST_CVR = 0;
    SYST_CSR = 5u;                   /* enable, core clock, no interrupt */
}

static void delay_ms(uint32_t ms)
{
    while (ms--) {
        while (!(SYST_CSR & SYST_COUNTFLAG))
            ;
        g_beacon.ms++;
    }
}

static void set(int on)
{
    if ((uint32_t)on != g_beacon.level)
        g_beacon.edges++;
    g_beacon.level = (uint32_t)on;
    led_set(on);
}

/* n blinks of `half` ms on and `half` ms off */
static void blink(int n, uint32_t half)
{
    while (n-- > 0) {
        set(1);
        delay_ms(half);
        set(0);
        delay_ms(half);
    }
}

static void probe_pin(void)
{
    LED_AHB4ENR |= (1u << LED_RCC_BIT);
    (void)LED_AHB4ENR;
    LED_MODER &= ~(3u << (2 * LED_PIN));            /* input */
    LED_PUPDR = (LED_PUPDR & ~(3u << (2 * LED_PIN)))
                | (1u << (2 * LED_PIN));            /* pull-up */
    tick_init();
    delay_ms(20);
    g_beacon.idr_pullup = (LED_IDR >> LED_PIN) & 1u;
    LED_PUPDR = (LED_PUPDR & ~(3u << (2 * LED_PIN)))
                | (2u << (2 * LED_PIN));            /* pull-down */
    delay_ms(20);
    g_beacon.idr_pulldown = (LED_IDR >> LED_PIN) & 1u;
}

int main(void)
{
    g_beacon.pin = (uint32_t)(('A' + (LED_PORT_BASE - 0x58020000u) / 0x400u)
                             << 8) | (uint32_t)LED_PIN;
    g_beacon.magic = LED_BEACON_MAGIC;

    probe_pin();
    led_init();
    g_beacon.moder = LED_MODER;

    for (;;) {
        g_beacon.phase = 0;
        blink(6, 500);              /* 6 s at 1 Hz */

        g_beacon.phase = 1;
        set(1);
        delay_ms(3000);             /* 3 s solid on */

        g_beacon.phase = 2;
        set(0);
        delay_ms(3000);             /* 3 s solid off */

        g_beacon.phase = 3;
        blink(30, 100);             /* 6 s at 5 Hz */

        g_beacon.cycles++;
    }
}
