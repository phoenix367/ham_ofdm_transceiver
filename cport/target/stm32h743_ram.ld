/* Link the on-target bench to run entirely from RAM.
 *
 * Nothing here is flashed: OpenOCD loads the sections straight to their
 * run addresses, sets SP and PC, and resumes. The board keeps whatever
 * firmware it already had, which matters when the only probe is a
 * bit-banger and reflashing would take minutes.
 *
 * Code goes in ITCM because that is where a real deployment would put it
 * (64-bit fetch, zero wait states, FEASIBILITY.md "Target fit") -- the
 * point is to measure the algorithms, not the AXI bus.
 *
 * Because OpenOCD loads the image directly there is no separate LMA and
 * no .data copy; startup only has to zero .bss. */

MEMORY
{
    ITCM  (rx)  : ORIGIN = 0x00000000, LENGTH = 64K
    DTCM  (rw)  : ORIGIN = 0x20000000, LENGTH = 128K
    AXI   (rw)  : ORIGIN = 0x24000000, LENGTH = 512K
    D2    (rw)  : ORIGIN = 0x30000000, LENGTH = 288K
}

/* Fixed, so the host can dump results without parsing the ELF. */
_results_addr = 0x20000000;
_stack_top    = ORIGIN(DTCM) + LENGTH(DTCM);

ENTRY(_start)

SECTIONS
{
    /* Code only. .rodata deliberately does NOT live here: a data read
     * from ITCM contends with instruction fetch on the same TCM port,
     * and the hot loops read tables (HILBERT_TAPS, NCO_COS) on every
     * iteration. Measured cost of getting this wrong is in
     * FEASIBILITY.md. */
    .text : {
        KEEP(*(.text._start))
        *(.text*)
        . = ALIGN(8);
    } > ITCM

    /* results first in DTCM, at the fixed address above */
    .results _results_addr : {
        KEEP(*(.results))
        . = ALIGN(8);
    } > DTCM

    /* const tables and initialised data in DTCM: zero wait states, and
     * a separate port from the instruction side */
    .rodata : {
        *(.rodata*)
        . = ALIGN(8);
    } > DTCM

    .data : {
        *(.data*)
        . = ALIGN(8);
    } > DTCM

    /* explicitly DTCM-resident scratch, for the memory-vs-compute A/B */
    .dtcm_bss (NOLOAD) : {
        *(.dtcm_bss*)
        . = ALIGN(8);
    } > DTCM

    /* D2 SRAM1+2+3. Clock-gated: the bench turns it on before use.
     * g_raw is 288 kB and lands here BY NAME -- with -fdata-sections it
     * has its own .bss.g_raw section, so no source change is needed.
     * It has to leave AXI: g_raw + g_blk + the arena is 522 kB against
     * AXI's 512, and D2 is exactly 288 kB, which g_raw exactly fills. */
    .d2_bss (NOLOAD) : {
        *(.bss.g_raw)
        *(.d2_bss*)
        . = ALIGN(8);
    } > D2

    .bss (NOLOAD) : {
        _bss_start = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(8);
        _bss_end = .;
    } > AXI

    /DISCARD/ : { *(.ARM.exidx*) *(.comment) }
}
