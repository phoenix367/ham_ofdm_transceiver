/* Vector table for RAM-resident images.
 *
 * A RAM-resident image has no table of its own, so VTOR still points at
 * whatever firmware was resident before -- and any interrupt or fault
 * then vectors into ANOTHER PROGRAM's handler and vanishes. The USB
 * bring-up paid for that lesson twice. This installs a table in RAM,
 * points VTOR at it, and applies the rule that lesson produced:
 *
 *   a genuine fault stops and reports;
 *   an unexpected interrupt is masked, counted, and survived.
 *
 * The second half matters. The first table treated every vector as
 * fatal and died within a minute on an unhandled SysTick nobody had
 * switched off -- ICSR 0x80F, CFSR and HFSR both zero, not a fault at
 * all. Treating every stray source as fatal makes a dead device out of
 * a stray timer. */

#include "vectors.h"

#define SCB_VTOR  (*(volatile uint32_t *)0xE000ED08u)
#define ICSR      (*(volatile uint32_t *)0xE000ED04u)
#define CFSR      (*(volatile uint32_t *)0xE000ED28u)
#define HFSR      (*(volatile uint32_t *)0xE000ED2Cu)
#define SYST_CSR  (*(volatile uint32_t *)0xE000E010u)
#define NVIC_ISER ((volatile uint32_t *)0xE000E100u)
#define NVIC_ICER ((volatile uint32_t *)0xE000E180u)
#define NVIC_ICPR ((volatile uint32_t *)0xE000E280u)
#define SCB_CCR   (*(volatile uint32_t *)0xE000ED14u)
#define SCB_DCCMVAC (*(volatile uint32_t *)0xE000EF68u)
#define SCB_CCR_DC (1u << 16)
#define NVIC_IPR  ((volatile uint8_t  *)0xE000E400u)

#define VECT_N 166            /* 16 system + 150 external on an H743 */

static vect_fn_t g_table[VECT_N] __attribute__((aligned(1024)));
volatile vectors_status_t g_vectors_status;

/* Push vector-table writes out to memory.
 *
 * The table lives in ordinary cached SRAM, and on a Cortex-M7 an
 * exception vector fetch reads MEMORY, not the D-cache -- so an entry
 * written here sits in a dirty cache line while the hardware still
 * fetches the old one. DSB does not help: it orders accesses, it does
 * not clean.
 *
 * This is not theoretical. It cost a full debug cycle on the two-board
 * link: the receiver installed its TIM6 handler, started the timer, and
 * the very first interrupt vectored to the STRAY handler, which masked
 * IRQ 54 (ISER1 = 0, ISPR1 = 0x400000 pending forever, isr_count 0).
 * Reading the table over JTAG showed the correct handler -- the line
 * had been evicted by then. The transmitter role never saw it, because
 * build_frame() runs between install and first interrupt and evicts the
 * line in time. Same binary, same board: only the eviction timing
 * differed.
 *
 * Cleaning with the cache disabled is harmless, so this is unconditional
 * except for the cheap check that keeps it a no-op when DC is off.
 */
static void vect_flush(const void *addr, uint32_t n)
{
    uint32_t a, end;
    if (!(SCB_CCR & SCB_CCR_DC))
        return;
    a = (uint32_t)(uintptr_t)addr & ~31u;      /* 32-byte lines on M7 */
    end = (uint32_t)(uintptr_t)addr + n;
    __asm__ volatile("dsb");
    for (; a < end; a += 32u)
        SCB_DCCMVAC = a;
    __asm__ volatile("dsb; isb");
}

static void fault_handler(void)
{
    uint32_t vect = ICSR & 0x1FFu;

    if (vect < 16u) {
        g_vectors_status.icsr = ICSR;
        g_vectors_status.cfsr = CFSR;
        g_vectors_status.hfsr = HFSR;
        if (vect == 15u) {            /* SysTick: silence it and go on */
            SYST_CSR = 0;
            g_vectors_status.stray_irq = 15u;
            g_vectors_status.stray_count++;
            return;
        }
        g_vectors_status.fault = 1;
        for (;;)
            ;
    }
    {
        uint32_t irq = vect - 16u;
        g_vectors_status.stray_irq = irq;
        g_vectors_status.stray_count++;
        NVIC_ICER[irq >> 5] = 1u << (irq & 31u);
    }
}

void vectors_install(void)
{
    int i;
    for (i = 0; i < VECT_N; i++)
        g_table[i] = fault_handler;
    g_table[0] = (vect_fn_t)0x20020000u;   /* initial SP; unused, we are running */
    vect_flush(g_table, sizeof(g_table));
    __asm__ volatile("dsb");
    SCB_VTOR = (uint32_t)(void *)g_table;
    __asm__ volatile("dsb; isb");
    SYST_CSR = 0;                          /* nothing here wants a tick */
}

void vectors_set(int irq, vect_fn_t fn)
{
    g_table[16 + irq] = fn;
    vect_flush(&g_table[16 + irq], sizeof(g_table[0]));
}

void vectors_irq_enable(int irq, uint8_t prio)
{
    NVIC_IPR[irq] = prio;
    /* Drop anything already pending BEFORE enabling. A RAM image
     * inherits the NVIC of the firmware it displaced -- measured: an
     * image loaded over the USB firmware starts with ISER3 bit 5
     * (OTG_FS) still set -- so a line can be asserted, and its enable
     * still set, from before this image existed. Enabling on top of
     * that takes the stale interrupt as the first one, which lands in
     * the stray handler and gets the source MASKED. */
    NVIC_ICPR[irq >> 5] = 1u << (irq & 31);
    NVIC_ISER[irq >> 5] = 1u << (irq & 31);
}
