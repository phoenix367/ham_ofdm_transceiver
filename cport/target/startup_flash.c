/* Cold-boot startup for a FLASH-resident image on the STM32H743.
 *
 * Everything the RAM-resident images got for free from the firmware that
 * was already running -- the 400 MHz clock tree, the FPU, a vector table
 * -- this does itself, because at reset the part runs from HSI at 64 MHz
 * with no PLL and VTOR at 0.
 *
 * The clock values are not derived; they were READ OFF THIS PART over
 * JTAG while its resident firmware ran. HSE 25 MHz / 5 * 160 / 2 =
 * 400 MHz, VOS1, flash latency 2. A different crystal needs different
 * DIVN/DIVM, which is why the reset path can MEASURE the result (DWT vs
 * a known timer) rather than trusting it; the applications that report a
 * clock do exactly that in their beacon.
 */

#include <stdint.h>

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern int main(void);
void Reset_Handler(void);
void Default_Handler(void);

#define RCC_CR        (*(volatile uint32_t *)0x58024400u)
#define RCC_CFGR      (*(volatile uint32_t *)0x58024410u)
#define RCC_D1CFGR    (*(volatile uint32_t *)0x58024418u)
#define RCC_D2CFGR    (*(volatile uint32_t *)0x5802441Cu)
#define RCC_D3CFGR    (*(volatile uint32_t *)0x58024420u)
#define RCC_PLLCKSELR (*(volatile uint32_t *)0x58024428u)
#define RCC_PLLCFGR   (*(volatile uint32_t *)0x5802442Cu)
#define RCC_PLL1DIVR  (*(volatile uint32_t *)0x58024430u)
#define PWR_D3CR      (*(volatile uint32_t *)0x58024818u)
#define FLASH_ACR     (*(volatile uint32_t *)0x52002000u)
#define SCB_VTOR      (*(volatile uint32_t *)0xE000ED08u)
#define CPACR         (*(volatile uint32_t *)0xE000ED88u)

#define RCC_CR_HSEON   (1u << 16)
#define RCC_CR_HSERDY  (1u << 17)
#define RCC_CR_PLL1ON  (1u << 24)
#define RCC_CR_PLL1RDY (1u << 25)
#define PWR_D3CR_VOSRDY (1u << 13)

static void clock_init(void)
{
    /* VOS1 for 400 MHz, then wait for the regulator. */
    PWR_D3CR = (PWR_D3CR & ~(3u << 14)) | (3u << 14);
    while (!(PWR_D3CR & PWR_D3CR_VOSRDY))
        ;
    /* HSE (25 MHz crystal). */
    RCC_CR |= RCC_CR_HSEON;
    while (!(RCC_CR & RCC_CR_HSERDY))
        ;
    /* PLL1, verbatim from the part: DIVM1 5, DIVN1 160, DIVP1 2. */
    RCC_PLLCKSELR = 0x00000052u;
    RCC_PLLCFGR   = 0x00070009u;
    RCC_PLL1DIVR  = 0x7f03029fu;
    RCC_CR |= RCC_CR_PLL1ON;
    while (!(RCC_CR & RCC_CR_PLL1RDY))
        ;
    /* Flash latency BEFORE raising the clock. */
    FLASH_ACR = 0x00000032u;
    while ((FLASH_ACR & 0xFu) != 2u)
        ;
    /* Domain prescalers, then switch sysclk to PLL1. */
    RCC_D1CFGR = 0x00000048u;
    RCC_D2CFGR = 0x00000440u;
    RCC_D3CFGR = 0x00000050u;
    RCC_CFGR = (RCC_CFGR & ~7u) | 3u;
    while ((RCC_CFGR & (7u << 3)) != (3u << 3))
        ;
}

void Reset_Handler(void)
{
    uint32_t *src, *dst;

    SCB_VTOR = 0x08000000u;           /* our table, in flash */
    CPACR |= (0xFu << 20);            /* FPU: before any float prologue */
    __asm__ volatile("dsb; isb");

    clock_init();

    for (src = &_sidata, dst = &_sdata; dst < &_edata; )
        *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss; )
        *dst++ = 0;

    __asm__ volatile("dsb; isb");
    main();
    for (;;)
        ;
}

void Default_Handler(void)
{
    for (;;)
        ;
}

#define ALIAS __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void) ALIAS;
void HardFault_Handler(void) ALIAS;
void MemManage_Handler(void) ALIAS;
void BusFault_Handler(void) ALIAS;
void UsageFault_Handler(void) ALIAS;
void SVC_Handler(void) ALIAS;       /* 11 */
void PendSV_Handler(void) ALIAS;    /* 14 */
void SysTick_Handler(void) ALIAS;   /* 15 -- an app that uses SysTick overrides it */
void OTG_FS_Handler(void) ALIAS;    /* IRQ 101 */
void OTG_HS_Handler(void) ALIAS;    /* IRQ 77  */

/* A NULL vector is not the default handler -- an interrupt through it
 * jumps to 0 and faults. Every position an app might enable must name a
 * handler, so SysTick (15) and the OTG lines are filled here even though
 * this file leaves them weak; the app provides the strong definition. */
__attribute__((section(".isr_vector"), used))
void (*const g_vectors[16 + 102])(void) = {
    (void (*)(void))&_estack,
    Reset_Handler,
    NMI_Handler, HardFault_Handler, MemManage_Handler,
    BusFault_Handler, UsageFault_Handler,
    [11] = SVC_Handler,
    [14] = PendSV_Handler,
    [15] = SysTick_Handler,
    [16 + 77] = OTG_HS_Handler,
    [16 + 101] = OTG_FS_Handler
};
