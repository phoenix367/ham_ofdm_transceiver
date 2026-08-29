/* Minimal flash-boot proof, NO DWT. Flashed to 0x08000000, it must run
 * on its own after a reset with nothing loaded. The magic appearing
 * proves Reset_Handler ran main (.bss, this beacon included, is zeroed
 * first); sws proves clock_init switched sysclk to PLL1; loops climbing
 * proves the core is executing.
 *
 * Deliberately DWT-free: DWT's cycle counter is not clocked unless a
 * debugger is attached, so a standalone image that reads DWT_CYC stalls
 * the bus. The first version did, and hung on that exact ldr -- which is
 * why the flashed USB firmware clocks its station off SysTick, not DWT.
 */
#include <stdint.h>

#define RCC_CFGR (*(volatile uint32_t *)0x58024410u)

typedef struct {
    uint32_t magic;      /* 0x0B007ED0 once main runs */
    uint32_t sws;        /* RCC_CFGR SWS: 3 = PLL1 is sysclk */
    uint32_t loops;      /* climbs -> core is executing */
    uint32_t vtor;       /* 0x08000000 */
} hb_t;

volatile hb_t g_beacon __attribute__((section(".results"), used));

int main(void)
{
    g_beacon.sws = (RCC_CFGR >> 3) & 7u;
    g_beacon.vtor = *(volatile uint32_t *)0xE000ED08u;
    g_beacon.magic = 0x0B007ED0u;
    for (;;)
        g_beacon.loops++;
}
