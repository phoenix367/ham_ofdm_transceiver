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

    /* D2 and DTCM MUST be placed before .bss: the linker assigns input
     * sections to the first output section that matches, and .bss's
     * `*(.bss*)` matches `.bss.g_raw` too. With .bss first, the two
     * lines below matched nothing, g_raw sat in AXI, and adding the
     * receiver to an image overflowed AXI by 100 kB while D2 stayed
     * completely empty. (stm32h743_usb.ld always had this order, which
     * is why the RAM benches were unaffected.)
     *
     * The receiver's two big rings go here by name -- -fdata-sections
     * gives every object its own .bss.<symbol>, so no source change is
     * needed: the raw sample ring (160 kB) and the per-block summary
     * ring (94 kB). Both are written before they are read on any path
     * that matters, but they are zeroed anyway -- see below. */
    .d2_bss (NOLOAD) : {
        . = ALIGN(8); _sd2bss = .;
        *(.bss.g_raw) *(.bss.g_blk) *(.d2_bss*)
        . = ALIGN(8); _ed2bss = .;
    } > D2

    .dtcm_bss (NOLOAD) : {
        . = ALIGN(8); _sdtcmbss = .;
        /* the receiver's data-LLR store: 3 x MAX_LLRS x 4 B. At the
         * radio build's MAX_LLRS=4416 that is 53 kB, which AXI cannot
         * take on top of everything else but DTCM can -- and the LLR
         * store is the hottest large object in the demod, so DTCM is
         * where it wants to be anyway. */
        *(.bss.g_d64)
        /* the carrier-sense ring: touched every 12 kHz tick from the
         * ISR, and the 960 B that put AXI 504 B over when its call
         * site became real. DTCM is where an ISR-hot ring belongs. */
        *(.bss.g_bring)
        *(.dtcm_bss*)
        . = ALIGN(8); _edtcmbss = .;
    } > DTCM

    /* .results first, so the beacon keeps its fixed address at the base
     * of AXI (0x24000000) whatever else lands in .bss. */
    .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; *(.results) *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > AXI

    /DISCARD/ : { *(.ARM.exidx*) }
}
