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
#define NVIC_IPR  ((volatile uint8_t  *)0xE000E400u)

#define VECT_N 166            /* 16 system + 150 external on an H743 */

static vect_fn_t g_table[VECT_N] __attribute__((aligned(1024)));
volatile vectors_status_t g_vectors_status;

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
    __asm__ volatile("dsb");
    SCB_VTOR = (uint32_t)(void *)g_table;
    __asm__ volatile("dsb; isb");
    SYST_CSR = 0;                          /* nothing here wants a tick */
}

void vectors_set(int irq, vect_fn_t fn)
{
    g_table[16 + irq] = fn;
    __asm__ volatile("dsb; isb");
}

void vectors_irq_enable(int irq, uint8_t prio)
{
    NVIC_IPR[irq] = prio;
    NVIC_ISER[irq >> 5] = 1u << (irq & 31);
}
