/* Two boards linked by an audio channel: board A's DAC -> wire -> board
 * B's ADC. One OFDM frame, over copper, between two independent clocks.
 *
 * This is the step analog_loop.c could not take. There, one board played
 * and recorded through the same TIM6, so playback and capture shared a
 * time base and the sample-rate offset was exactly zero -- the one
 * impairment a real link always has and a loopback never does. Here the
 * two boards run their own crystals, their own PLLs and their own TIM6,
 * so the receiver sees a genuine sample-rate offset (tens of ppm) it has
 * to absorb, and both boards report their MEASURED sample rate so the
 * offset can be computed rather than assumed.
 *
 * One image, two roles, chosen by the host over JTAG before the CPU is
 * released (g_beacon.role at 0x20000004, preserved across main()'s
 * clear). A station is half duplex and the transmitter shares the
 * receiver's arena, so no board ever does both:
 *
 *   role TX: build one NORMAL QPSK frame, then play it to DAC1_OUT1
 *            (PA4) REPS times back to back, ~24 s of air.
 *   role RX: record ADC1 channel 3 (PA6) for CAP_N samples, then run
 *            the streaming receiver over the recording and count how
 *            many frames came back bit-exact.
 *
 * The host starts TX first and RX second, so the whole capture window
 * sits inside the transmission -- no trigger, no handshake, no clock
 * shared between the boards even to start them. The recording is left
 * in place for the host to dump, so the channel can be characterised
 * independently of whether the decoder liked it.
 *
 * Wiring: board A PA4 -> board B PA6, and a GROUND wire between the two
 * boards. Both grounds also meet at the host's USB, but that is a long
 * and noisy path; without the direct wire the ADC reads the difference
 * between two supplies rather than the signal.
 *
 * Runs from RAM over the JTAG probe; neither board's flash is touched.
 */

#include <stdint.h>
#include <string.h>

#include "tx.h"
#include "packets.h"
#include "rx_stream.h"
#include "vectors.h"

/* --- registers, from stm32h743xx.h ---------------------------------- */
#define RCC_BASE      0x58024400u
#define RCC_D3CCIPR   (*(volatile uint32_t *)(RCC_BASE + 0x58u))
#define RCC_AHB1ENR   (*(volatile uint32_t *)(RCC_BASE + 0xD8u))
#define RCC_AHB4ENR   (*(volatile uint32_t *)(RCC_BASE + 0xE0u))
#define RCC_APB1LENR  (*(volatile uint32_t *)(RCC_BASE + 0xE8u))
#define RCC_AHB1ENR_ADC12EN   (1u << 5)
#define RCC_AHB4ENR_GPIOAEN   (1u << 0)
#define RCC_APB1LENR_TIM6EN   (1u << 4)
#define RCC_APB1LENR_DAC12EN  (1u << 29)
#define RCC_D3CCIPR_ADCSEL_Pos 16

#define TIM6_BASE     0x40001000u
#define TIM6_CR1      (*(volatile uint32_t *)(TIM6_BASE + 0x00u))
#define TIM6_DIER     (*(volatile uint32_t *)(TIM6_BASE + 0x0Cu))
#define TIM6_SR       (*(volatile uint32_t *)(TIM6_BASE + 0x10u))
#define TIM6_EGR      (*(volatile uint32_t *)(TIM6_BASE + 0x14u))
#define TIM6_CNT      (*(volatile uint32_t *)(TIM6_BASE + 0x24u))
#define TIM6_PSC      (*(volatile uint32_t *)(TIM6_BASE + 0x28u))
#define TIM6_ARR      (*(volatile uint32_t *)(TIM6_BASE + 0x2Cu))
#define TIM6_DAC_IRQn 54

#define DAC1_BASE     0x40007400u
#define DAC_CR        (*(volatile uint32_t *)(DAC1_BASE + 0x00u))
#define DAC_DHR12R1   (*(volatile uint32_t *)(DAC1_BASE + 0x08u))
#define DAC_MCR       (*(volatile uint32_t *)(DAC1_BASE + 0x3Cu))
#define DAC_CR_EN1    (1u << 0)

#define ADC1_BASE     0x40022000u
#define ADC_ISR       (*(volatile uint32_t *)(ADC1_BASE + 0x00u))
#define ADC_CR        (*(volatile uint32_t *)(ADC1_BASE + 0x08u))
#define ADC_CFGR      (*(volatile uint32_t *)(ADC1_BASE + 0x0Cu))
#define ADC_SMPR1     (*(volatile uint32_t *)(ADC1_BASE + 0x14u))
#define ADC_PCSEL     (*(volatile uint32_t *)(ADC1_BASE + 0x1Cu))
#define ADC_SQR1      (*(volatile uint32_t *)(ADC1_BASE + 0x30u))
#define ADC_DR        (*(volatile uint32_t *)(ADC1_BASE + 0x40u))
#define ADC_CCR       (*(volatile uint32_t *)(ADC1_BASE + 0x308u))
#define ADC_ISR_ADRDY  (1u << 0)
#define ADC_ISR_EOC    (1u << 2)
#define ADC_ISR_LDORDY (1u << 12)
#define ADC_CR_ADEN    (1u << 0)
#define ADC_CR_ADSTART (1u << 2)
#define ADC_CR_BOOST_Pos 8
#define ADC_CR_ADVREGEN (1u << 28)
#define ADC_CR_DEEPPWD  (1u << 29)
#define ADC_CR_ADCAL    (1u << 31)
#define ADC_CCR_PRESC_Pos 18
#define ADC_CHANNEL    3               /* ADC12_INP3 = PA6 */

#define NVIC_ISER_R ((volatile uint32_t *)0xE000E100u)
#define DEMCR    (*(volatile uint32_t *)0xE000EDFCu)
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYC  (*(volatile uint32_t *)0xE0001004u)

#ifndef OFDM_CPU_HZ
#define OFDM_CPU_HZ 400000000u
#endif
#define FS_HZ 12000u

#define ROLE_TX 1u
#define ROLE_RX 2u

/* NORMAL QPSK 1/2, 27-byte payload: 16896 samples. LEAD of silence at
 * both ends, so replaying the buffer end to end leaves a 2*LEAD gap
 * between frames -- the receiver needs quiet before a preamble. */
#define FRAME_MAX 17408
#define LEAD      512
#define PLAY_MAX  (LEAD + FRAME_MAX + LEAD)
#define REPS      14                   /* ~24 s of transmission */

/* 4.5 s: at least two COMPLETE frame periods land inside it wherever
 * the capture happens to start relative to the transmitter. */
#define CAP_N     54000

#define D2_BSS __attribute__((section(".d2_bss")))

/* One buffer for both roles -- a board is either playing or recording,
 * never both, so they cannot collide. Sized by the larger use. */
static int16_t g_buf[CAP_N] D2_BSS;
static uint8_t g_pkt[280];
static int g_pkt_n;
static int g_play_n;

/* --- beacon, read back over JTAG at 0x20000000 ---------------------- */
#define BEACON_MAGIC 0xA10C11A4u

typedef struct {
    uint32_t magic;
    uint32_t role;            /* written by the HOST before resume */
    uint32_t stage;
    uint32_t tim_hz;          /* measured TIM6 input clock */
    uint32_t fs_mhz;          /* achieved sample rate, mHz -- the ppm
                               * offset between the boards comes from
                               * comparing the two */
    uint32_t adc_ldo, adc_cal, adc_rdy;
    uint32_t adc_conv_ns;
    uint32_t n_play, n_cap, reps_done, isr_count, isr_max_cyc;
    int32_t  cap_min, cap_max, cap_mean;
    uint32_t buf_addr;
    int32_t  n_events, n_decoded, n_ok;   /* over the whole recording */
    int32_t  ev_start, ev_cfo;            /* of the first good frame */
    uint32_t iser0, iser1, iser2, iser3;  /* NVIC INHERITED at entry */
    uint32_t early_stray_irq, early_stray_n;  /* strays during bring-up */
    uint32_t fault, icsr, cfsr, hfsr, stray_irq, stray_count;
    /* start of every decoded frame: consecutive spacings measure the
     * sample-rate offset, which comparing the two TIM6 clocks cannot --
     * both boards derive TIM6 and DWT from the SAME PLL, so that ratio
     * is exact on each board and identical between them however far
     * apart the crystals are. */
    int32_t  starts[8];
} beacon_t;

beacon_t g_beacon __attribute__((section(".results"), used));

enum { ST_ENTER = 1, ST_ROLE, ST_TX_BUILT, ST_PERIPH, ST_TIMER,
       ST_RUNNING, ST_RAN, ST_DECODED };

/* --- ISR ------------------------------------------------------------ */
static volatile uint32_t g_idx, g_reps;
static volatile int g_running;
static uint32_t g_role;

static inline uint32_t adc_convert(void)
{
    ADC_CR |= ADC_CR_ADSTART;
    while (!(ADC_ISR & ADC_ISR_EOC))
        ;
    return ADC_DR;                      /* reading DR clears EOC */
}

static void tim6_isr(void)
{
    uint32_t t0 = DWT_CYC, i = g_idx, d;
    TIM6_SR = 0;                        /* clear UIF */
    if (!g_running)
        return;
    if (g_role == ROLE_TX) {
        /* 3/4 scale: the DAC's output buffer cannot reach the rails and
         * the transmitter reaches int16 full scale. */
        int32_t s = g_buf[i];
        DAC_DHR12R1 = (uint32_t)(2048 + ((s * 3) >> 6));
        if (++i >= (uint32_t)g_play_n) {
            i = 0;
            if (++g_reps >= REPS) {
                DAC_DHR12R1 = 2048;
                g_running = 0;
            }
        }
        g_idx = i;
    } else {
        if (i < CAP_N) {
            g_buf[i] = (int16_t)(adc_convert() - 32768u);
            g_idx = i + 1;
        } else {
            g_running = 0;
        }
    }
    g_beacon.isr_count++;
    d = DWT_CYC - t0;
    if (d > g_beacon.isr_max_cyc)
        g_beacon.isr_max_cyc = d;
}

/* --- bring-up ------------------------------------------------------- */
static int wait_bit(volatile uint32_t *reg, uint32_t mask, int want_set,
                    uint32_t max_cyc)
{
    uint32_t t0 = DWT_CYC;
    for (;;) {
        uint32_t v = *reg & mask;
        if ((want_set && v) || (!want_set && !v))
            return 1;
        if (DWT_CYC - t0 > max_cyc)
            return 2;
    }
}

static void adc_init(void)
{
    volatile uint32_t *isr = (volatile uint32_t *)(ADC1_BASE + 0x00u);
    volatile uint32_t *cr  = (volatile uint32_t *)(ADC1_BASE + 0x08u);

    RCC_AHB1ENR |= RCC_AHB1ENR_ADC12EN;
    RCC_D3CCIPR = (RCC_D3CCIPR & ~(3u << RCC_D3CCIPR_ADCSEL_Pos))
                  | (2u << RCC_D3CCIPR_ADCSEL_Pos);   /* per_ck = HSI */
    ADC_CCR = (4u << ADC_CCR_PRESC_Pos);              /* CKMODE 0, /8 */

    ADC_CR &= ~ADC_CR_DEEPPWD;
    ADC_CR |= ADC_CR_ADVREGEN;
    g_beacon.adc_ldo = wait_bit(isr, ADC_ISR_LDORDY, 1, 4000000u);
    {   volatile int k = 20000; while (k--) ; }

    ADC_CR |= (1u << ADC_CR_BOOST_Pos);
    ADC_CR |= ADC_CR_ADCAL;
    g_beacon.adc_cal = wait_bit(cr, ADC_CR_ADCAL, 0, 40000000u);

    ADC_CFGR = 0;                       /* 16-bit, single, sw trigger */
    ADC_PCSEL = 1u << ADC_CHANNEL;      /* H7: preselect or read garbage */
    ADC_SQR1 = (uint32_t)ADC_CHANNEL << 6;
    ADC_SMPR1 = 5u << (3 * ADC_CHANNEL);          /* 64.5 cycles */

    ADC_ISR = ADC_ISR_ADRDY;
    ADC_CR |= ADC_CR_ADEN;
    g_beacon.adc_rdy = wait_bit(isr, ADC_ISR_ADRDY, 1, 4000000u);

    {   uint32_t t0 = DWT_CYC;
        (void)adc_convert();
        g_beacon.adc_conv_ns = (uint32_t)((uint64_t)(DWT_CYC - t0)
                                          * 1000000000ull / OFDM_CPU_HZ);
    }
}

static void dac_init(void)
{
    RCC_APB1LENR |= RCC_APB1LENR_DAC12EN;
    DAC_MCR = 0;                        /* normal mode, buffer ON, to pin */
    DAC_DHR12R1 = 2048;
    DAC_CR = DAC_CR_EN1;
    {   volatile int k = 20000; while (k--) ; }    /* buffer wake-up */
}

/* Measure TIM6's input clock against DWT, then set it to FS_HZ. Each
 * board does this for itself; the two answers are what make the link
 * asynchronous, and their ratio is the sample-rate offset. */
static void tim6_init(void)
{
    uint32_t c0, c1, d0, d1, arr;
    uint64_t hz;

    RCC_APB1LENR |= RCC_APB1LENR_TIM6EN;
    TIM6_CR1 = 0;
    TIM6_PSC = 1999;
    TIM6_ARR = 0xFFFF;
    TIM6_EGR = 1;
    TIM6_SR = 0;
    TIM6_CR1 = 1;
    c0 = TIM6_CNT; d0 = DWT_CYC;
    while (DWT_CYC - d0 < OFDM_CPU_HZ / 20u)   /* 50 ms */
        ;
    c1 = TIM6_CNT; d1 = DWT_CYC;
    TIM6_CR1 = 0;
    hz = (uint64_t)((c1 - c0) & 0xFFFFu) * 2000ull * OFDM_CPU_HZ / (d1 - d0);
    g_beacon.tim_hz = (uint32_t)hz;

    arr = (uint32_t)((hz + FS_HZ / 2) / FS_HZ);
    TIM6_PSC = 0;
    TIM6_ARR = arr - 1;
    TIM6_EGR = 1;
    TIM6_SR = 0;
    g_beacon.fs_mhz = (uint32_t)(hz * 1000ull / arr);
    TIM6_DIER = 1;
    vectors_set(TIM6_DAC_IRQn, tim6_isr);
    vectors_irq_enable(TIM6_DAC_IRQn, 0x40);
}

/* --- the frame ------------------------------------------------------ */
/* Both roles build the PACKET (the receiver needs it to check the bits
 * against); only the transmitter turns it into samples. */
static void build_packet(void)
{
    uint8_t payload[27];
    int i;
    for (i = 0; i < 27; i++)
        payload[i] = (uint8_t)(0x41 + (i % 26));       /* "ABC..." */
    g_pkt_n = data_encode(7, payload, 27, g_pkt);
}

static int build_frame(void)
{
    txs_t *t;
    int total = 0, got, pos = LEAD;

    t = txs_open(MODE_NORMAL, g_pkt, g_pkt_n, 1, PKT_TYP_DATA, MOD_QPSK,
                 CC_R12, 0, 0, &total);
    if (!t || total <= 0 || total > FRAME_MAX)
        return -1;
    memset(g_buf, 0, sizeof(g_buf));
    while ((got = txs_pull(t, g_buf + pos, PLAY_MAX - pos)) > 0)
        pos += got;
    if (pos - LEAD != total)
        return -1;
    g_play_n = LEAD + total + LEAD;
    return total;
}

static void decode_capture(int n)
{
    rxs_t *r;
    rxs_event_t ev;
    int pos;
    int64_t sum = 0;
    int32_t mn = 32767, mx = -32768, mean;
    int i;

    for (i = 0; i < n; i++) {
        int32_t v = g_buf[i];
        sum += v;
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    mean = (int32_t)(sum / n);
    g_beacon.cap_min = mn + 32768;      /* report raw ADC units */
    g_beacon.cap_max = mx + 32768;
    g_beacon.cap_mean = mean + 32768;
    /* the recording rides on the transmitter's 1.65 V mid-rail plus
     * whatever the two boards' grounds differ by: remove it */
    for (i = 0; i < n; i++)
        g_buf[i] = (int16_t)(g_buf[i] - mean);

    r = rxs_open(MODE_NORMAL, 0);
    if (!r)
        return;
    /* the whole recording, not just up to the first hit: the window
     * holds two or three frames and each one is a trial */
    for (pos = 0; pos < n; pos += 512) {
        int c = n - pos < 512 ? n - pos : 512;
        if (rxs_push(r, g_buf + pos, c, &ev)) {
            g_beacon.n_events++;
            if (ev.type == 1) {
                g_beacon.n_decoded++;
                if (ev.pkt_bits_n == g_pkt_n
                    && memcmp(ev.bits, g_pkt, (size_t)g_pkt_n) == 0) {
                    if (g_beacon.n_ok == 0) {
                        g_beacon.ev_start = ev.start_abs;
                        g_beacon.ev_cfo = (int32_t)ev.cfo_word;
                    }
                    if (g_beacon.n_ok < 8)
                        g_beacon.starts[g_beacon.n_ok] = ev.start_abs;
                    g_beacon.n_ok++;
                }
            }
        }
    }
}

int main(void)
{
    uint32_t role;

    /* the host wrote role while we were halted -- keep it */
    /* Mask interrupts for the whole of bring-up. This image inherits the
     * NVIC of the firmware it displaced, so a source can be enabled and
     * asserting before a single instruction here runs; taking it while
     * the vector table still reads "stray everywhere" gets that source
     * masked for good. Measured exactly that: TIM6's first update
     * landed in the stray handler, which cleared ISER bit 54, and the
     * receiver then sat in its run loop with isr_count 0 forever. */
    __asm__ volatile("cpsid i");

    role = g_beacon.role;
    memset((void *)&g_beacon, 0, sizeof(g_beacon));
    g_beacon.magic = BEACON_MAGIC;
    g_beacon.role = role;
    g_beacon.stage = ST_ENTER;
    g_beacon.buf_addr = (uint32_t)(uintptr_t)g_buf;
    g_role = role;

    g_beacon.iser0 = NVIC_ISER_R[0];      /* what we inherited */
    g_beacon.iser1 = NVIC_ISER_R[1];
    g_beacon.iser2 = NVIC_ISER_R[2];
    g_beacon.iser3 = NVIC_ISER_R[3];

    DEMCR |= (1u << 24);
    DWT_CYC = 0;
    DWT_CTRL |= 1u;
    vectors_install();

    if (role != ROLE_TX && role != ROLE_RX) {
        g_beacon.stage = 0xE0;          /* host did not set a role */
        for (;;) ;
    }
    g_beacon.stage = ST_ROLE;

    build_packet();
    if (role == ROLE_TX) {
        if (build_frame() < 0) {
            g_beacon.stage = 0xEE;      /* generator refused */
            for (;;) ;
        }
        g_beacon.n_play = (uint32_t)g_play_n;
        g_beacon.stage = ST_TX_BUILT;
    }

    RCC_AHB4ENR |= RCC_AHB4ENR_GPIOAEN;  /* PA4/PA6 stay analog (reset) */
    if (role == ROLE_TX)
        dac_init();
    else
        adc_init();
    g_beacon.stage = ST_PERIPH;

    tim6_init();
    /* a stray here is the failure this bench already hit once -- record
     * it while it still means something, not at the end we never reach */
    g_beacon.early_stray_irq = g_vectors_status.stray_irq;
    g_beacon.early_stray_n = g_vectors_status.stray_count;
    g_beacon.stage = ST_TIMER;

    g_idx = 0;
    g_reps = 0;
    g_running = 1;
    __asm__ volatile("cpsie i");
    TIM6_CR1 = 1;
    g_beacon.stage = ST_RUNNING;
    while (g_running)
        ;
    TIM6_CR1 = 0;
    __asm__ volatile("cpsid i");
    g_beacon.reps_done = g_reps;
    g_beacon.stage = ST_RAN;

    if (role == ROLE_RX) {
        g_beacon.n_cap = g_idx;
        decode_capture((int)g_idx);
    }
    g_beacon.fault = g_vectors_status.fault;
    g_beacon.icsr = g_vectors_status.icsr;
    g_beacon.cfsr = g_vectors_status.cfsr;
    g_beacon.hfsr = g_vectors_status.hfsr;
    g_beacon.stray_irq = g_vectors_status.stray_irq;
    g_beacon.stray_count = g_vectors_status.stray_count;
    g_beacon.stage = ST_DECODED;

    __asm__ volatile("bkpt #0");
    for (;;)
        ;
}
