/* STM32H743 bring-up for USB device mode, register level.
 *
 * Everything here has to happen before TinyUSB can see a core, and
 * every value is from ST's own headers (cmsis_device_h7) rather than
 * from memory:
 *
 *   PWR_CR3       USB33DEN bit 24, USBREGEN bit 25, USB33RDY bit 26
 *   RCC_CR        HSI48ON bit 12, HSI48RDY bit 13
 *   RCC_D2CCIP2R  USBSEL bits 21:20, 11 = HSI48
 *   RCC_AHB1ENR   USB2OTGFSEN bit 27, USB1OTGHSEN bit 25
 *   USB2_OTG_FS   0x40080000, OTG_FS_IRQn = 101
 *
 * The 48 MHz comes from HSI48, not a PLL. Deliberately: a PLL-derived
 * USB clock depends on the crystal, and this board's HSE frequency is
 * not knowable from software -- the clock tree only says sysclk =
 * HSE x 16. HSI48 is an on-chip RC trimmed by CRS against the host's
 * start-of-frame, which is what it exists for, and it makes the
 * firmware correct on any H743 board whatever its crystal.
 */

#include <stdint.h>

#define PWR_BASE   0x58024800u
#define PWR_CR3    (*(volatile uint32_t *)(PWR_BASE + 0x0Cu))
#define PWR_CR3_USB33DEN (1u << 24)
#define PWR_CR3_USBREGEN (1u << 25)
#define PWR_CR3_USB33RDY (1u << 26)

#define RCC_BASE   0x58024400u
#define RCC_CR         (*(volatile uint32_t *)(RCC_BASE + 0x00u))
#define RCC_D2CCIP2R   (*(volatile uint32_t *)(RCC_BASE + 0x54u))
#define RCC_AHB1ENR    (*(volatile uint32_t *)(RCC_BASE + 0xD8u))
#define RCC_AHB4ENR    (*(volatile uint32_t *)(RCC_BASE + 0xE0u))
#define RCC_APB1HENR   (*(volatile uint32_t *)(RCC_BASE + 0xF4u))
#define RCC_CR_HSI48ON  (1u << 12)
#define RCC_CR_HSI48RDY (1u << 13)
#define RCC_USBSEL_HSI48 (3u << 20)
#define RCC_AHB1ENR_USB2OTGFSEN (1u << 27)
#define RCC_AHB1ENR_USB1OTGHSEN (1u << 25)
#define RCC_AHB4ENR_GPIOAEN (1u << 0)
#define RCC_AHB4ENR_GPIOBEN (1u << 1)
#define RCC_APB1HENR_CRSEN  (1u << 6)

#define CRS_BASE   0x40008400u
#define CRS_CR     (*(volatile uint32_t *)(CRS_BASE + 0x00u))
#define CRS_CR_AUTOTRIMEN (1u << 6)
#define CRS_CR_CEN        (1u << 5)

#define GPIOA_BASE 0x58020000u
#define GPIOB_BASE 0x58020400u
#define GPIO_MODER(b)   (*(volatile uint32_t *)((b) + 0x00u))
#define GPIO_OSPEEDR(b) (*(volatile uint32_t *)((b) + 0x08u))
#define GPIO_PUPDR(b)   (*(volatile uint32_t *)((b) + 0x0Cu))
#define GPIO_AFRH(b)    (*(volatile uint32_t *)((b) + 0x24u))

#define NVIC_ISER ((volatile uint32_t *)0xE000E100u)
#define NVIC_ICER ((volatile uint32_t *)0xE000E180u)
#define NVIC_IPR  ((volatile uint8_t  *)0xE000E400u)
#define OTG_FS_IRQn 101
#define OTG_HS_IRQn 77

/* CMSIS global, normally from ST's system_stm32h7xx.c which this build
 * does not include. TinyUSB's dwc2 port uses it in exactly one place --
 * a 1 ms busy-loop for remote wakeup, which a device-only vendor-class
 * configuration never takes -- so the value affects nothing here. It is
 * the sysclk derived from the live part: RCC reads SWS=PLL1, DIVM1=5,
 * DIVN1=160, DIVP1=/2, i.e. HSE x 16, and 25 MHz is the usual crystal. */
uint32_t SystemCoreClock = 400000000u;

static void pin_af(uint32_t base, int pin, int af)
{
    int s2 = 2 * pin, s4 = 4 * (pin - 8);
    GPIO_MODER(base) = (GPIO_MODER(base) & ~(3u << s2)) | (2u << s2);
    GPIO_OSPEEDR(base) |= (3u << s2);   /* very high speed for 12 Mbit/s */
    GPIO_PUPDR(base) &= ~(3u << s2);    /* no pull: the PHY drives */
    GPIO_AFRH(base) = (GPIO_AFRH(base) & ~(0xFu << s4)) | ((uint32_t)af << s4);
}

/* Ask for the USB supply, and work out which kind of board this is.
 *
 * A board can source VDD33USB two ways, and they need OPPOSITE settings:
 *
 *   externally, from its own 3.3 V rail -- set USB33DEN (the supply
 *     level detector) and nothing else;
 *   internally, from VDD50USB (the VBUS pin) -- also set USBREGEN to
 *     start the on-chip regulator.
 *
 * Enabling USBREGEN on a board of the first kind does not merely waste
 * a regulator: USB33RDY stays LOW and the device never comes up.
 * Measured on the part -- PWR_CR3 read 0x03000042 with both enables set
 * and ready clear, and 0x05000042 (ready) the instant USBREGEN was
 * cleared. So this tries the external case first and falls back, which
 * makes the firmware correct on either board without being told.
 *
 * Bounded, never a spin-wait. An unbounded `while` here hangs before
 * reaching anything that could report why, which on the bench looked
 * exactly like a crash: stage stuck at 1, loops at 0. */
static uint32_t s_supply_spins;
static int s_used_regulator;

#ifndef OFDM_USB_REG_FALLBACK_SPINS
#define OFDM_USB_REG_FALLBACK_SPINS 400000u
#endif

int ofdm_usb_bsp_supply_ready(void)
{
    PWR_CR3 |= PWR_CR3_USB33DEN;
    if (PWR_CR3 & PWR_CR3_USB33RDY)
        return 1;
    if (!s_used_regulator && ++s_supply_spins > OFDM_USB_REG_FALLBACK_SPINS) {
        s_used_regulator = 1;          /* try the internal regulator */
        PWR_CR3 |= PWR_CR3_USBREGEN;
    }
    return 0;
}

/* 1 if the internal regulator was needed -- i.e. VDD33USB comes from
 * VBUS on this board. Reported in the beacon so the board's wiring is
 * recorded rather than guessed at next time. */
int ofdm_usb_bsp_used_regulator(void)
{
    return s_used_regulator;
}

void ofdm_usb_bsp_init(int rhport)
{
    /* 48 MHz from the on-chip RC, trimmed against the host's SOF */
    RCC_CR |= RCC_CR_HSI48ON;
    while (!(RCC_CR & RCC_CR_HSI48RDY))
        ;   /* bounded in practice: an on-chip RC with no external
             * dependency, and measured ready on this part */
    RCC_D2CCIP2R = (RCC_D2CCIP2R & ~(3u << 20)) | RCC_USBSEL_HSI48;
    RCC_APB1HENR |= RCC_APB1HENR_CRSEN;
    CRS_CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;

    /* 3. pins. AF10 for OTG_FS on PA11/PA12, AF12 for OTG_HS on
     *    PB14/PB15. Only D+/D- are driven; VBUS sensing is left off in
     *    the core, because a board that waits for a VBUS it cannot see
     *    never enumerates -- and that failure looks exactly like a dead
     *    cable, which is an expensive thing to misdiagnose. */
    if (rhport == 0) {
        RCC_AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
        pin_af(GPIOA_BASE, 11, 10);
        pin_af(GPIOA_BASE, 12, 10);
        RCC_AHB1ENR |= RCC_AHB1ENR_USB2OTGFSEN;
    } else {
        RCC_AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
        pin_af(GPIOB_BASE, 14, 12);
        pin_af(GPIOB_BASE, 15, 12);
        RCC_AHB1ENR |= RCC_AHB1ENR_USB1OTGHSEN;
    }
    (void)RCC_AHB1ENR;   /* let the enable land before the core is poked */
}

void ofdm_usb_bsp_irq_enable(int rhport)
{
    int irq = rhport == 0 ? OTG_FS_IRQn : OTG_HS_IRQn;
    NVIC_IPR[irq] = 0x80;
    NVIC_ISER[irq >> 5] = 1u << (irq & 31);
}

/* The counterpart, and the one the RAM-resident image actually needs.
 *
 * TinyUSB enables the OTG interrupt in the NVIC from inside dcd_init()
 * -- dwc2_dcd_int_enable() calls NVIC_EnableIRQ. In a flashed build
 * that is right. In a RAM-resident image there is NO VECTOR TABLE:
 * VTOR still points at whatever was there before, so the first USB
 * interrupt vectors into another program's handler. That presents as a
 * hang with no output, which this project has already paid for once.
 *
 * So the image polls tud_int_handler() and masks the line instead. */
void ofdm_usb_bsp_irq_disable(int rhport)
{
    int irq = rhport == 0 ? OTG_FS_IRQn : OTG_HS_IRQn;
    NVIC_ICER[irq >> 5] = 1u << (irq & 31);
}
