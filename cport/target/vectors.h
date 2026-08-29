/* A vector table for RAM-resident images -- see vectors.c. */
#ifndef OFDM_TARGET_VECTORS_H
#define OFDM_TARGET_VECTORS_H
#include <stdint.h>

typedef void (*vect_fn_t)(void);

/* Installs a table in RAM and points VTOR at it. Every entry is the
 * fault handler until vectors_set() replaces it. */
void vectors_install(void);
void vectors_set(int irq, vect_fn_t fn);   /* external IRQ number */
void vectors_irq_enable(int irq, uint8_t prio);

/* what the fault handler recorded; all zero on a healthy run */
typedef struct {
    uint32_t fault, icsr, cfsr, hfsr;   /* a genuine fault: stopped here */
    uint32_t stray_irq, stray_count;    /* unexpected IRQs: masked, went on */
} vectors_status_t;
extern volatile vectors_status_t g_vectors_status;

#endif
