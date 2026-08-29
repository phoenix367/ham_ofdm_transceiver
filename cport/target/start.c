/* Entry point for the RAM-resident bench.
 *
 * OpenOCD sets SP and PC and resumes, so there is no vector table, no
 * .data copy (sections are loaded at their run addresses) and no
 * semihosting. Only two things have to happen before main: zero .bss,
 * which the loader does not do, and turn the caches on so the numbers
 * describe a realistic deployment rather than a stalled one. */

#include <stdint.h>

extern unsigned char _bss_start[], _bss_end[];
extern int main(void);

#define SCB_CCR   (*(volatile uint32_t *)0xE000ED14u)
#define SCB_CCSIDR (*(volatile uint32_t *)0xE000ED80u)
#define SCB_CSSELR (*(volatile uint32_t *)0xE000ED84u)
#define SCB_ICIALLU (*(volatile uint32_t *)0xE000EF50u)
#define SCB_DCISW  (*(volatile uint32_t *)0xE000EF60u)
#define CCR_IC (1u << 17)
#define CCR_DC (1u << 16)

static void dcache_invalidate_all(void)
{
    uint32_t ccsidr, sets, ways, s, w;
    SCB_CSSELR = 0;                 /* select L1 data cache */
    __asm__ volatile("dsb");
    ccsidr = SCB_CCSIDR;
    sets = ((ccsidr >> 13) & 0x7FFFu);
    ways = ((ccsidr >> 3) & 0x3FFu);
    for (s = 0; s <= sets; s++)
        for (w = 0; w <= ways; w++)
            SCB_DCISW = (w << 30) | (s << 5);
    __asm__ volatile("dsb; isb");
}

void _start(void)
{
    unsigned char *p;

    for (p = _bss_start; p < _bss_end; p++)
        *p = 0;

    /* Invalidate before enabling: the lines are unknown at this point
     * (another image was running a moment ago) and enabling a cache full
     * of someone else's tags would hand us their data. */
    SCB_ICIALLU = 0;
    dcache_invalidate_all();
    __asm__ volatile("dsb; isb");
    SCB_CCR |= CCR_IC | CCR_DC;
    __asm__ volatile("dsb; isb");

    main();
    for (;;)
        ;
}
