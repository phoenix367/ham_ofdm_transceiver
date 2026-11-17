/* Transmitter keying (PTT) -- one GPIO pin, and the timing around it.
 *
 * The radio firmware drives PB14 ACTIVE for exactly as long as it
 * transmits: from key-up until the last sample has left the DAC, plus an
 * optional TAIL of mid-rail samples. A radio needs time after its PTT
 * closes before it is on frequency at full power, so an optional LEAD
 * holds the DAC at mid-rail for that long BEFORE the frame's first
 * sample -- the frame is never shortened, it starts later. Both default
 * to 0: the cable stand has no radio, and every figure measured on it
 * (results/board_bench.*) was taken without them. Set them with
 * `config ptt_lead <ms>` / `config ptt_tail <ms>` (UP_CFG 15/16).
 *
 * Measured on the stand (2026-11-15, tools/baofeng/ptt_carrier_check.py):
 * PB14 HIGH on board1 (JTAG stmA) keys a Baofeng through its PTT driver,
 * heard by a HackRF at +27.8 dB in 30 kHz through a 60 dB pad, and
 * nothing while low -- active-high. PB14 is free in this firmware: USB is
 * OTG_FS on PA11/PA12, and PB14/PB15 are taken only when the USB core is
 * OTG_HS (USB_RHPORT=1), which the radio image never is. PB3/PB4 (JTAG)
 * share the port, so every MODER/OTYPER/... write is read-modify-write.
 *
 * Measured through the firmware (2026-11-15, Baofeng on board1's PB14,
 * HackRF behind 60 dB, carrier edges in SAMPLE time): the pin is held
 * exactly lead + frame + tail -- beacon ptt_last_ticks 9280 / 11680 /
 * 11680 for a 9280-sample frame at lead/tail 0/0, 200/0, 0/200, zero
 * drain latency -- and the carrier follows: +197 ms for the lead,
 * +200 ms for the tail. The RADIO then stays on the air 0.20-0.36 s
 * past release (0.354-0.358 s after 1 s frames, 0.200 s after a 25 s
 * one); its PTT-to-RF delay also wandered by ~58 ms in one run. Those
 * are the radio's numbers, not the pin's. The tail is not dead carrier:
 * it carries a 56 Hz tone at 0.35-0.7 kHz deviation that starts as the
 * pin drops (2026-11-17, two key-ups, IQ demodulated) -- the signature
 * of the Baofeng's squelch-tail-elimination (STE) burst. But STE is NOT
 * what holds the carrier: with STE off (two more key-ups) the 56 Hz tone
 * is gone (~1 Hz deviation) and the carrier is just as long -- 3.97 s
 * against 3.94-3.98 s with it on, for the same ~3.63 s of pin. The radio
 * stays keyed ~0.35 s regardless; STE only fills that time.
 *
 * TAIL vs THE PEER: a station keys its reply turnaround/2 = 150 ms after
 * decoding, and we cannot hear while keyed. A tail longer than the
 * peer's 150 ms + its own lead puts the start of its reply in our deaf
 * window: measured, tail 200 against a peer at lead 0 lost the ack of
 * every frame and walked the ladder to EXTREME; with the peer at lead
 * 300 the same tail delivered 7 of 7 with no retransmission. With radios
 * at BOTH ends the radio's own release time counts the same way, so
 * give each station a lead of ~0.4 s and leave the tail at 0.
 *
 * Safety: a keyed radio whose firmware stopped is a carrier on the air
 * until the radio's own time-out timer. Two releases cover that:
 *   STALL  the ISR drops PTT after PTT_STALL_TICKS consecutive ticks of
 *          empty FIFO while keyed -- the main loop stopped feeding the
 *          DAC and the frame is lost anyway. It stays released until the
 *          next key-up; `cuts` counts it (beacon ptt_cuts).
 *   FAULT  startup_flash.c's Default_Handler calls fault_safe_state()
 *          (weak), which the radio firmware defines to drop the pin.
 *
 * Who touches what: the main loop calls ptt_arm() when it starts
 * pre-filling a transmission, ptt_tail() from tx_fill once the generator
 * is exhausted, ptt_key_up() immediately before raising g_tx_on and
 * ptt_key_down() when the FIFO has drained. The 12 kHz ISR calls
 * ptt_in_lead() / ptt_fed() / ptt_starved() while g_tx_on is set.
 * key_up writes the lead BEFORE the caller raises g_tx_on, and the ISR
 * is on the same core, so it never sees a half-armed transmission.
 *
 * Header-only, like led.h. The pin is written through BSRR (atomic, so
 * the ISR may release it). Host tests define PTT_PIN_HOOK to a function
 * of their own and the GPIO code is not compiled.
 */
#ifndef OFDM_PTT_H
#define OFDM_PTT_H

#include <stdint.h>
#include <string.h>

#define PTT_FS          12000u          /* the converter tick */
#define PTT_MAX_MS      2000            /* clamp for lead and tail */
#define PTT_STALL_TICKS 12000u          /* 1 s of starved DAC while keyed */

/* ---- the pin -------------------------------------------------------- */
#ifdef PTT_PIN_HOOK
void PTT_PIN_HOOK(int on);
static inline void ptt_pin_write(int on) { PTT_PIN_HOOK(on); }
static inline void ptt_pin_init(void) { PTT_PIN_HOOK(0); }
#else
#ifndef PTT_PORT_BASE
#define PTT_PORT_BASE 0x58020400u       /* GPIOB */
#endif
#ifndef PTT_RCC_BIT
#define PTT_RCC_BIT 1                   /* RCC_AHB4ENR: GPIOB = bit 1 */
#endif
#ifndef PTT_PIN
#define PTT_PIN 14                      /* PB14 */
#endif
#ifndef PTT_ACTIVE_LOW
#define PTT_ACTIVE_LOW 0                /* MEASURED active-high (above) */
#endif

#define PTT_AHB4ENR (*(volatile uint32_t *)0x580244E0u)
#define PTT_REG(off) (*(volatile uint32_t *)(PTT_PORT_BASE + (off)))

static inline void ptt_pin_write(int on)
{
#if PTT_ACTIVE_LOW
    PTT_REG(0x18u) = on ? (1u << (PTT_PIN + 16)) : (1u << PTT_PIN);
#else
    PTT_REG(0x18u) = on ? (1u << PTT_PIN) : (1u << (PTT_PIN + 16));
#endif
}

/* The pin floats (analog, the reset state) until this runs, so call it
 * first thing in main. The released level is driven BEFORE the pin
 * becomes an output, so switching the mode cannot key the radio. */
static inline void ptt_pin_init(void)
{
    PTT_AHB4ENR |= (1u << PTT_RCC_BIT);
    (void)PTT_AHB4ENR;                           /* let the enable land */
    ptt_pin_write(0);
    PTT_REG(0x04u) &= ~(1u << PTT_PIN);           /* push-pull */
    PTT_REG(0x08u) &= ~(3u << (2 * PTT_PIN));     /* slowest edge */
    PTT_REG(0x0Cu) &= ~(3u << (2 * PTT_PIN));     /* no pull */
    PTT_REG(0x00u) = (PTT_REG(0x00u) & ~(3u << (2 * PTT_PIN)))
                     | (1u << (2 * PTT_PIN));     /* output */
}
#endif

/* ---- the timing ----------------------------------------------------- */
typedef struct ptt {
    uint32_t lead_ticks, tail_ticks;  /* configured, in converter ticks */
    volatile uint32_t lead_left;      /* ISR: mid-rail ticks still owed */
    volatile uint32_t starve;         /* ISR: consecutive empty-FIFO ticks */
    uint32_t tail_left;               /* main loop: tail samples owed */
    volatile int keyed;               /* the pin as last driven */
    uint32_t keyups;                  /* transmissions keyed */
    volatile uint32_t cuts;           /* stall-watchdog releases */
    /* how long the pin was ACTUALLY held, by the converter clock: ISR
     * ticks from key-up to key-down, the main loop's drain latency
     * included. Should read lead + frame + tail; the beacon carries the
     * last one so a radio's carrier can be compared with the pin. */
    volatile uint32_t on_ticks;
    uint32_t last_on_ticks;
} ptt_t;

static inline void ptt_init(ptt_t *p)
{
    memset((void *)p, 0, sizeof(*p));
}

static inline uint32_t ptt_ms_to_ticks(int ms)
{
    if (ms < 0) ms = 0;
    if (ms > PTT_MAX_MS) ms = PTT_MAX_MS;
    return (uint32_t)ms * (PTT_FS / 1000u);
}

/* takes effect at the next transmission */
static inline void ptt_config(ptt_t *p, int lead_ms, int tail_ms)
{
    p->lead_ticks = ptt_ms_to_ticks(lead_ms);
    p->tail_ticks = ptt_ms_to_ticks(tail_ms);
}

static inline int ptt_lead_ms(const ptt_t *p)
{
    return (int)(p->lead_ticks / (PTT_FS / 1000u));
}

static inline int ptt_tail_ms(const ptt_t *p)
{
    return (int)(p->tail_ticks / (PTT_FS / 1000u));
}

/* main loop, as the FIFO pre-fill of a transmission starts: the tail is
 * owed from here, because a short frame can exhaust its generator inside
 * the pre-fill -- before key-up */
static inline void ptt_arm(ptt_t *p)
{
    p->tail_left = p->tail_ticks;
}

/* main loop, immediately before raising g_tx_on */
static inline void ptt_key_up(ptt_t *p)
{
    p->lead_left = p->lead_ticks;
    p->starve = 0;
    p->on_ticks = 0;
    p->keyed = 1;
    p->keyups++;
    ptt_pin_write(1);
}

/* main loop, once the last sample has left the DAC */
static inline void ptt_key_down(ptt_t *p)
{
    p->last_on_ticks = p->on_ticks;
    p->keyed = 0;
    p->lead_left = 0;
    p->tail_left = 0;
    ptt_pin_write(0);
}

/* main loop, from tx_fill once the generator (and any impairment flush)
 * is exhausted: write up to `room` mid-rail samples of tail, return how
 * many. Not generator output, so not counted in g_tx_pulled. */
static inline int ptt_tail(ptt_t *p, int16_t *dst, int room)
{
    int n = (int)p->tail_left;
    if (n > room)
        n = room;
    if (n <= 0)
        return 0;
    memset(dst, 0, (size_t)n * sizeof(*dst));
    p->tail_left -= (uint32_t)n;
    return n;
}

/* ISR, every tick while transmitting: 1 = still in the lead, hold the
 * DAC at mid-rail and do not touch the FIFO. Also the keyed-time clock:
 * it is called exactly once per tick while g_tx_on is set. */
static inline int ptt_in_lead(ptt_t *p)
{
    uint32_t l = p->lead_left;
    p->on_ticks++;
    if (!l)
        return 0;
    p->lead_left = l - 1;
    return 1;
}

/* ISR: a sample went out */
static inline void ptt_fed(ptt_t *p)
{
    p->starve = 0;
}

/* ISR: the FIFO was empty this tick */
static inline void ptt_starved(ptt_t *p)
{
    if (p->keyed && ++p->starve >= PTT_STALL_TICKS) {
        p->keyed = 0;
        p->cuts++;
        ptt_pin_write(0);
    }
}

#endif /* OFDM_PTT_H */
