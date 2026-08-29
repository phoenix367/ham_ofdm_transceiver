/* Flash-resident image on the STM32H743. Code and rodata in bank-1 flash
 * from reset; .data copied to DTCM and .bss zeroed by Reset_Handler.
 * Vector table at 0x08000000, where hardware fetches SP and reset PC. */

MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K   /* sector 0 */
    DTCM  (rw) : ORIGIN = 0x20000000, LENGTH = 128K
    AXI   (rw) : ORIGIN = 0x24000000, LENGTH = 512K
    D2    (rw) : ORIGIN = 0x30000000, LENGTH = 288K
}

_estack = ORIGIN(DTCM) + LENGTH(DTCM);
ENTRY(Reset_Handler)

SECTIONS
{
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
    .text : { *(.text*) *(.rodata*) . = ALIGN(4); } > FLASH

    _sidata = LOADADDR(.data);
    .data : { . = ALIGN(4); _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > DTCM AT > FLASH

    /* g_beacon lands here by its own section, at a fixed DTCM address so
     * the host can still read it. It is zeroed as part of .bss. */
    .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; *(.results) *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > AXI

    .d2_bss   (NOLOAD) : { *(.bss.g_raw) *(.d2_bss*)  . = ALIGN(8); } > D2
    .dtcm_bss (NOLOAD) : { *(.dtcm_bss*) . = ALIGN(8); } > DTCM

    /DISCARD/ : { *(.ARM.exidx*) }
}
