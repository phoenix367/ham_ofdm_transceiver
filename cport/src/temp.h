/* Internal temperature sensor -- STM32H743, ADC3.
 *
 * Header-only for the same reason as led.h: it is a property of the
 * PART rather than of the code, and a bench should be able to drive it
 * exactly as the firmware does. Only one translation unit includes it
 * (the firmware's main), which is why the state below may be static.
 *
 * WHICH CONVERTER. Not the audio one: on this part VSENSE is bonded to
 * ADC3_INP18 (VBAT/4 on INP17, VREFINT on INP19) and ADC1/2 cannot
 * reach it at all. ADC3 sits in the D3 domain behind its own AHB4
 * clock gate, so it costs the 12 kHz capture path nothing -- the two
 * converters run independently. The kernel-clock select (D3CCIPR
 * ADCSEL) is COMMON to all three ADCs and adc_init() owns it, so call
 * temp_init() after adc_init() and do not set it here.
 *
 * WHERE IT RUNS. From the main loop at 1 Hz, never the ISR: the sensor
 * needs a long sampling time (810.5 cycles, ~100 us at the 8 MHz ADC
 * clock this firmware runs) and the ISR has a 83 us budget.
 *
 * THE ARITHMETIC. Two factory calibration points in system memory,
 * READ OFF THIS BOARD over JTAG rather than assumed: TS_CAL1 = 12420
 * at 30 C, TS_CAL2 = 16374 at 110 C (49.4 LSB/C), VREFINT_CAL = 24275.
 * The raw reading is first referred back to the calibration's 3.3 V
 * VDDA through VREFINT -- the boards run off a USB regulator, and
 * assuming 3.3 V exactly would fold the supply's error straight into
 * the temperature. All integer, no division wider than 32 bits: the
 * intermediates peak around 6e8.
 *
 * A wrong address would otherwise produce a confident wrong number, so
 * temp_init() sanity-checks the calibration pair and reports the
 * sensor as absent (TEMP_Q8_NONE) if it is not ordered and plausible.
 */
#ifndef OFDM_TEMP_H
#define OFDM_TEMP_H

#include <stdint.h>

/* "no reading": the protocol's up_status_t carries this verbatim, and
 * both consoles print it as n/a rather than as -128 C. */
#define TEMP_Q8_NONE ((int16_t)-32768)

#ifndef TEMP_ADC_BASE
#define TEMP_ADC_BASE 0x58026000u        /* ADC3 */
#endif
#ifndef TEMP_RCC_AHB4ENR_ADDR
#define TEMP_RCC_AHB4ENR_ADDR 0x580244E0u
#endif
#ifndef TEMP_CAL1_ADDR
#define TEMP_CAL1_ADDR 0x1FF1E820u       /* 30 C  */
#endif
#ifndef TEMP_CAL2_ADDR
#define TEMP_CAL2_ADDR 0x1FF1E840u       /* 110 C */
#endif
#ifndef TEMP_VREFCAL_ADDR
#define TEMP_VREFCAL_ADDR 0x1FF1E860u
#endif

#define TEMP_R(off)   (*(volatile uint32_t *)(uintptr_t)(TEMP_ADC_BASE + (off)))
#define TEMP_ISR_R    TEMP_R(0x00u)
#define TEMP_CR       TEMP_R(0x08u)
#define TEMP_CFGR     TEMP_R(0x0Cu)
#define TEMP_SMPR2    TEMP_R(0x18u)
#define TEMP_PCSEL    TEMP_R(0x1Cu)
#define TEMP_SQR1     TEMP_R(0x30u)
#define TEMP_DR       TEMP_R(0x40u)
#define TEMP_CCR      TEMP_R(0x308u)
#define TEMP_AHB4ENR  (*(volatile uint32_t *)(uintptr_t)TEMP_RCC_AHB4ENR_ADDR)

#define TEMP_CH_VSENSE 18
#define TEMP_CH_VREF   19

static int      g_temp_ok;               /* init succeeded AND cal sane */
static int32_t  g_temp_cal1, g_temp_cal2, g_temp_vrefcal;

static inline int32_t temp_cal_at(uint32_t addr)
{
    return (int32_t)(*(volatile uint16_t *)(uintptr_t)addr);
}

/* One software-triggered conversion. Returns -1 rather than spinning
 * forever if the converter never reports end-of-conversion. */
static inline int32_t temp_convert(int ch)
{
    uint32_t guard;
    TEMP_SQR1 = (uint32_t)ch << 6;               /* L=1, SQ1 = ch */
    TEMP_ISR_R = (1u << 2);                      /* clear EOC */
    TEMP_CR |= (1u << 2);                        /* ADSTART */
    for (guard = 0; guard < 4000000u; guard++)
        if (TEMP_ISR_R & (1u << 2))
            return (int32_t)(TEMP_DR & 0xFFFFu);
    return -1;
}

static inline int temp_init(void)
{
    uint32_t guard;

    g_temp_cal1 = temp_cal_at(TEMP_CAL1_ADDR);
    g_temp_cal2 = temp_cal_at(TEMP_CAL2_ADDR);
    g_temp_vrefcal = temp_cal_at(TEMP_VREFCAL_ADDR);
    /* 80 C apart, and this part reads ~49 LSB/C: anything outside a
     * wide band around that is a bad address, not a hot chip. */
    if (g_temp_cal2 - g_temp_cal1 < 800 || g_temp_cal2 - g_temp_cal1 > 12000
        || g_temp_vrefcal < 8000 || g_temp_vrefcal > 40000)
        return 0;

    TEMP_AHB4ENR |= (1u << 24);                  /* ADC3EN */
    TEMP_CCR = (4u << 18)                        /* async kernel clock /8 */
             | (1u << 22)                        /* VREFEN  */
             | (1u << 23);                       /* VSENSEEN */
    TEMP_CR &= ~(1u << 29);                      /* DEEPPWD off */
    TEMP_CR |= (1u << 28);                       /* ADVREGEN */
    for (guard = 0; guard < 4000000u; guard++)
        if (TEMP_ISR_R & (1u << 12))             /* LDORDY */
            break;
    for (guard = 0; guard < 200000u; guard++)    /* t_START, as adc_init */
        __asm__ volatile("nop");
    TEMP_CR |= (1u << 8);                        /* BOOST */
    TEMP_CR |= (1u << 31);                       /* ADCAL */
    for (guard = 0; guard < 40000000u; guard++)
        if (!(TEMP_CR & (1u << 31)))
            break;
    TEMP_CFGR = 0;                               /* 16-bit, single, sw */
    TEMP_PCSEL = (1u << TEMP_CH_VSENSE) | (1u << TEMP_CH_VREF);
    /* SMPR2 covers channels 10..19; 7 = 810.5 cycles, the longest --
     * the sensor's own settling time is the reason this is not in the
     * ISR. */
    TEMP_SMPR2 = (7u << (3 * (TEMP_CH_VSENSE - 10)))
               | (7u << (3 * (TEMP_CH_VREF - 10)));
    TEMP_ISR_R = 1u;                             /* clear ADRDY */
    TEMP_CR |= 1u;                               /* ADEN */
    for (guard = 0; guard < 4000000u; guard++)
        if (TEMP_ISR_R & 1u) {
            g_temp_ok = 1;
            return 1;
        }
    return 0;
}

/* Degrees C in Q8, or TEMP_Q8_NONE. ~200 us (two conversions). */
static inline int temp_read_q8(void)
{
    int32_t ts, vref, span;

    if (!g_temp_ok)
        return TEMP_Q8_NONE;
    ts = temp_convert(TEMP_CH_VSENSE);
    vref = temp_convert(TEMP_CH_VREF);
    if (ts < 0 || vref <= 0)
        return TEMP_Q8_NONE;

    /* refer the reading back to the calibration's VDDA */
    ts = ts * g_temp_vrefcal / vref;
    span = g_temp_cal2 - g_temp_cal1;            /* checked non-zero */
    /* 30 C + (ts - cal1) * 80 C / span, in Q8. Peak intermediate is
     * about 80*256*30000 = 6.1e8, inside int32. */
    return (30 << 8) + (80 * 256 * (ts - g_temp_cal1)) / span;
}

#endif /* OFDM_TEMP_H */
