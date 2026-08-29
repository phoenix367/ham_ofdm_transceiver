/* Analog loopback test stand: DAC out -> wire -> ADC in, on one board.
 *
 * The first step towards two boards linked by an audio channel. A
 * station is half duplex and the transmitter shares the receiver's
 * scratch arena, so this does NOT transmit and receive at once: it
 * generates a whole NORMAL frame into a buffer first, then a 12 kHz
 * timer plays that buffer to DAC1_OUT1 (PA4) while recording ADC1
 * (channel 3, PA6) sample for sample, and only then runs the streaming
 * receiver over the recording. Short PA4 to PA6 with a wire and the
 * frame has to survive a real analog path: 12-bit DAC, output buffer,
 * copper, ADC input network, 16-bit ADC.
 *
 * Nothing here is DMA. The timer's update interrupt writes one DAC
 * sample, starts one ADC conversion, waits for it (~9 us of an 83 us
 * period) and stores it. That costs a little CPU during playback --
 * which is idle anyway, since the receiver runs afterwards -- and in
 * exchange needs no DMA streams, no trigger-source tables from a
 * reference manual this project does not have, and no ambiguity about
 * which ADC sample belongs to which DAC sample.
 *
 * Every peripheral fact is from ST's own header (stm32h743xx.h) or the
 * datasheet; every clock is MEASURED on the part rather than derived:
 * the timer's input against DWT, the ADC's conversion time against DWT.
 * The recording is left in place for the host to dump over JTAG, so the
 * analog path can be characterised (gain, delay, noise) independently
 * of whether the decoder liked it.
 *
 * Runs from RAM over the JTAG probe; the board's flash is untouched.
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
#define DAC_DOR1      (*(volatile uint32_t *)(DAC1_BASE + 0x2Cu))
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

#define DEMCR    (*(volatile uint32_t *)0xE000EDFCu)
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYC  (*(volatile uint32_t *)0xE0001004u)

#ifndef OFDM_CPU_HZ
#define OFDM_CPU_HZ 400000000u
#endif
#define FS_HZ 12000u

/* --- the frame ------------------------------------------------------ */
/* NORMAL QPSK 1/2, 27-byte payload: 16896 samples. Small enough that
 * both buffers live off the receiver's AXI-SRAM -- playback in D2,
 * the recording in DTCM. */
#define FRAME_MAX 17408
#define LEAD      512                  /* silence before and after */
#define CAP_N     (LEAD + FRAME_MAX + LEAD)

#define D2_BSS   __attribute__((section(".d2_bss")))
#define DTCM_BSS __attribute__((section(".dtcm_bss")))

static int16_t g_play[CAP_N] D2_BSS;
static int16_t g_cap[CAP_N]  DTCM_BSS;
static uint8_t g_pkt[280];
static int g_pkt_n;

/* --- beacon, read back over JTAG at 0x20000000 ---------------------- */
#define BEACON_MAGIC 0xA10C0DE5u

typedef struct {
    uint32_t magic, stage;
    uint32_t tim_hz;          /* measured TIM6 input clock */
    uint32_t fs_mhz;          /* achieved sample rate, mHz */
    uint32_t adc_ldo, adc_cal, adc_rdy;   /* 1 = ok, 2 = timed out */
    uint32_t adc_conv_ns;     /* measured conversion time */
    uint32_t n_play, n_cap, isr_count, isr_max_cyc;
    int32_t  cap_min, cap_max, cap_mean;  /* raw ADC, 16-bit unsigned */
    uint32_t play_addr, cap_addr;         /* for the host to dump */
    int32_t  ev_type;         /* rxs event: 1 = decoded; 0 = none */
    int32_t  ev_start, ev_cfo, bits_ok;
    uint32_t fault, icsr, cfsr, hfsr, stray_irq, stray_count;
} beacon_t;

beacon_t g_beacon __attribute__((section(".results"), used));

enum { ST_ENTER = 1, ST_TX_DONE, ST_PERIPH, ST_TIMER, ST_PLAYING,
       ST_PLAYED, ST_DECODED };

/* --- ISR: one sample out, one sample in ----------------------------- */
static volatile uint32_t g_idx;
static volatile int g_running;

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
    if (i < CAP_N) {
        /* 3/4 scale: the DAC's output buffer cannot reach the rails and
         * the transmitter reaches int16 full scale. 2048 + s*3/64 maps
         * -32768..32767 onto 512..3583 of 0..4095. */
        int32_t s = g_play[i];
        DAC_DHR12R1 = (uint32_t)(2048 + ((s * 3) >> 6));
        /* let the buffer settle, then sample */
        {
            volatile int k = 400;       /* ~1 us at 400 MHz */
            while (k--)
                ;
        }
        g_cap[i] = (int16_t)(adc_convert() - 32768u);
        g_idx = i + 1;
    } else {
        DAC_DHR12R1 = 2048;
        g_running = 0;
    }
    g_beacon.isr_count++;
    d = DWT_CYC - t0;
    if (d > g_beacon.isr_max_cyc)
        g_beacon.isr_max_cyc = d;
}

/* --- bring-up --------------------------------------------------------- */
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
    /* kernel clock: per_ck (HSI, 64 MHz) -- known to be running, unlike
     * PLL2. Async mode, prescaler /8. The actual conversion time is
     * measured below rather than assumed. */
    RCC_D3CCIPR = (RCC_D3CCIPR & ~(3u << RCC_D3CCIPR_ADCSEL_Pos))
                  | (2u << RCC_D3CCIPR_ADCSEL_Pos);
    ADC_CCR = (4u << ADC_CCR_PRESC_Pos);          /* CKMODE 0, /8 */

    ADC_CR &= ~ADC_CR_DEEPPWD;
    ADC_CR |= ADC_CR_ADVREGEN;
    g_beacon.adc_ldo = wait_bit(isr, ADC_ISR_LDORDY, 1, 4000000u); /* 10 ms */
    {   volatile int k = 20000; while (k--) ; }    /* and a fixed margin */

    ADC_CR |= (1u << ADC_CR_BOOST_Pos);           /* 6.25-12.5 MHz band */
    ADC_CR |= ADC_CR_ADCAL;                        /* single-ended cal */
    g_beacon.adc_cal = wait_bit(cr, ADC_CR_ADCAL, 0, 40000000u);

    ADC_CFGR = 0;                                  /* 16-bit, single, sw trigger */
    ADC_PCSEL = 1u << ADC_CHANNEL;                 /* H7: preselect or read garbage */
    ADC_SQR1 = (uint32_t)ADC_CHANNEL << 6;         /* L=0: one conversion */
    ADC_SMPR1 = 5u << (3 * ADC_CHANNEL);           /* 64.5 cycles on ch 3 */

    ADC_ISR = ADC_ISR_ADRDY;
    ADC_CR |= ADC_CR_ADEN;
    g_beacon.adc_rdy = wait_bit(isr, ADC_ISR_ADRDY, 1, 4000000u);

    {   /* measure the conversion time */
        uint32_t t0 = DWT_CYC;
        (void)adc_convert();
        g_beacon.adc_conv_ns = (uint32_t)((uint64_t)(DWT_CYC - t0) * 1000000000ull
                                          / OFDM_CPU_HZ);
    }
}

static void dac_init(void)
{
    RCC_APB1LENR |= RCC_APB1LENR_DAC12EN;
    DAC_MCR = 0;                        /* normal mode, buffer ON, to pin */
    DAC_DHR12R1 = 2048;                 /* mid-scale before enabling */
    DAC_CR = DAC_CR_EN1;                /* no trigger: DHR -> DOR at once */
    {   volatile int k = 20000; while (k--) ; }    /* buffer wake-up */
}

/* Measure TIM6's input clock against DWT, then set it to FS_HZ. The
 * timer clock depends on APB prescalers this image has not read, and
 * DWT on a sysclk this project has only inferred -- so the sample rate
 * is reported as MEASURED and the host can check it against wall time
 * through the ISR count. */
static void tim6_init(void)
{
    uint32_t c0, c1, d0, d1, arr;
    uint64_t hz;

    RCC_APB1LENR |= RCC_APB1LENR_TIM6EN;
    TIM6_CR1 = 0;
    TIM6_PSC = 1999;                    /* /2000: no 16-bit wrap in 50 ms */
    TIM6_ARR = 0xFFFF;
    TIM6_EGR = 1;                       /* load PSC */
    TIM6_SR = 0;
    TIM6_CR1 = 1;                       /* CEN */
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
    TIM6_DIER = 1;                      /* UIE */
    vectors_set(TIM6_DAC_IRQn, tim6_isr);
    vectors_irq_enable(TIM6_DAC_IRQn, 0x40);
}

/* --- the frame ------------------------------------------------------ */
static int build_frame(void)
{
    uint8_t payload[27];
    txs_t *t;
    int total = 0, got, pos = LEAD, i;

    for (i = 0; i < 27; i++)
        payload[i] = (uint8_t)(0x41 + (i % 26));       /* "ABC..." */
    g_pkt_n = data_encode(7, payload, 27, g_pkt);
    t = txs_open(MODE_NORMAL, g_pkt, g_pkt_n, 1, PKT_TYP_DATA, MOD_QPSK,
                 CC_R12, 0, 0, &total);
    if (!t || total <= 0 || total > FRAME_MAX)
        return -1;
    memset(g_play, 0, sizeof(g_play));
    while ((got = txs_pull(t, g_play + pos, CAP_N - pos)) > 0)
        pos += got;
    return pos - LEAD == total ? total : -1;
}

static void decode_capture(int n)
{
    rxs_t *r = rxs_open(MODE_NORMAL, 0);
    rxs_event_t ev;
    int pos, got = 0;
    int64_t sum = 0;
    int32_t mn = 32767, mx = -32768, mean;
    int i;

    for (i = 0; i < n; i++) {
        int32_t v = g_cap[i];
        sum += v;
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    mean = (int32_t)(sum / n);
    g_beacon.cap_min = mn + 32768;      /* report raw ADC units */
    g_beacon.cap_max = mx + 32768;
    g_beacon.cap_mean = mean + 32768;
    /* the recording rides on the DAC's 1.65 V mid-rail: remove it */
    for (i = 0; i < n; i++)
        g_cap[i] = (int16_t)(g_cap[i] - mean);

    if (!r)
        return;
    for (pos = 0; pos < n && !got; pos += 512) {
        int c = n - pos < 512 ? n - pos : 512;
        got = rxs_push(r, g_cap + pos, c, &ev);
    }
    if (!got)
        got = rxs_flush(r, &ev);
    if (got) {
        g_beacon.ev_type = ev.type;
        g_beacon.ev_start = ev.start_abs;
        g_beacon.ev_cfo = (int32_t)ev.cfo_word;
        g_beacon.bits_ok = (ev.type == 1 && ev.pkt_bits_n == g_pkt_n
                            && memcmp(ev.bits, g_pkt, (size_t)g_pkt_n) == 0);
    }
}

int main(void)
{
    int total;

    memset((void *)&g_beacon, 0, sizeof(g_beacon));
    g_beacon.magic = BEACON_MAGIC;
    g_beacon.stage = ST_ENTER;
    g_beacon.play_addr = (uint32_t)(uintptr_t)g_play;
    g_beacon.cap_addr = (uint32_t)(uintptr_t)g_cap;

    DEMCR |= (1u << 24);
    DWT_CYC = 0;
    DWT_CTRL |= 1u;
    vectors_install();

    total = build_frame();
    if (total < 0) {
        g_beacon.stage = 0xEE;          /* generator refused */
        for (;;) ;
    }
    g_beacon.n_play = (uint32_t)(LEAD + total + LEAD);
    g_beacon.stage = ST_TX_DONE;

    RCC_AHB4ENR |= RCC_AHB4ENR_GPIOAEN;  /* PA4/PA6 stay analog (reset) */
    dac_init();
    adc_init();
    g_beacon.stage = ST_PERIPH;

    tim6_init();
    g_beacon.stage = ST_TIMER;

    g_idx = 0;
    g_running = 1;
    __asm__ volatile("cpsie i");
    TIM6_CR1 = 1;
    g_beacon.stage = ST_PLAYING;
    while (g_running)
        ;
    TIM6_CR1 = 0;
    __asm__ volatile("cpsid i");
    g_beacon.n_cap = g_idx;
    g_beacon.stage = ST_PLAYED;

    decode_capture((int)g_idx);
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
