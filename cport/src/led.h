/* Board LED -- one pin, no dependencies.
 *
 * Header-only so the RAM bench (`bench/led_test.c`) and the flashed
 * firmware drive the same pin the same way. Everything is a macro
 * override, because the pin is a property of the BOARD, not of the
 * code: this stand's boards are STM32H743VIT6 in LQFP100 (read from
 * SYSCFG_PKGR, 0x0), where port H is bonded out as PH0/PH1 only --
 * both taken by the 25 MHz crystal -- so anything on PH2..PH15 is a
 * pin that does not exist. PA1 is the LED (found by blinking it); PA0
 * is unconnected. Both are free of the firmware's other uses: PA4
 * (DAC out), PA6 (ADC in), PA11/PA12 (USB) and PA13-PA15 + PB3 (JTAG,
 * and driving those kills the probe).
 *
 * Cost: one bit in one register per state change (BSRR is atomic, so
 * an ISR may drive it while the main loop drives another pin of the
 * same port).
 */
#ifndef OFDM_LED_H
#define OFDM_LED_H

#include <stdint.h>

#ifndef LED_PORT_BASE
#define LED_PORT_BASE 0x58020000u   /* GPIOA */
#endif
#ifndef LED_RCC_BIT
#define LED_RCC_BIT 0               /* RCC_AHB4ENR: GPIOA = bit 0 */
#endif
#ifndef LED_PIN
#define LED_PIN 1                   /* PA1 -- MEASURED (make run-led): PA0
                                     * floats and lights nothing. The
                                     * first flashed build defaulted to
                                     * 0 while the bench was run with
                                     * LED_PIN=1: the state machine ran
                                     * perfectly on a pin with no LED. */
#endif
#ifndef LED_ACTIVE_LOW
#define LED_ACTIVE_LOW 0            /* 1 = the LED is wired to VDD */
#endif

#define LED_AHB4ENR (*(volatile uint32_t *)0x580244E0u)
#define LED_REG(off) (*(volatile uint32_t *)(LED_PORT_BASE + (off)))
#define LED_MODER   LED_REG(0x00u)
#define LED_OTYPER  LED_REG(0x04u)
#define LED_OSPEEDR LED_REG(0x08u)
#define LED_PUPDR   LED_REG(0x0Cu)
#define LED_IDR     LED_REG(0x10u)
#define LED_ODR     LED_REG(0x14u)
#define LED_BSRR    LED_REG(0x18u)

static inline void led_set(int on)
{
#if LED_ACTIVE_LOW
    LED_BSRR = on ? (1u << (LED_PIN + 16)) : (1u << LED_PIN);
#else
    LED_BSRR = on ? (1u << LED_PIN) : (1u << (LED_PIN + 16));
#endif
}

static inline void led_init(void)
{
    LED_AHB4ENR |= (1u << LED_RCC_BIT);
    (void)LED_AHB4ENR;                       /* let the enable settle */
    led_set(0);                              /* drive the OFF level first,
                                              * so switching MODER cannot
                                              * flash the LED */
    LED_OTYPER &= ~(1u << LED_PIN);          /* push-pull */
    LED_OSPEEDR &= ~(3u << (2 * LED_PIN));   /* slowest: it is an LED */
    LED_PUPDR &= ~(3u << (2 * LED_PIN));     /* no pull */
    LED_MODER = (LED_MODER & ~(3u << (2 * LED_PIN)))
                | (1u << (2 * LED_PIN));     /* general-purpose output */
}

#endif /* OFDM_LED_H */
