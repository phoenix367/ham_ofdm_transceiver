/* QEMU mps2-an500 (Cortex-M7): code in ZBT SSRAM1 at 0, data in the
 * 16 MB PSRAM at 0x60000000. The suites' static buffers are host-sized
 * (see FEASIBILITY.md for the per-mode MCU budget), so the data region
 * is deliberately generous -- this runs the tests, it does not model a
 * part. Even so, test_rx and test_link need ~29 MB and do not fit. */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 4M
  RAM   (rwx) : ORIGIN = 0x60000000, LENGTH = 16M
}
ENTRY(Reset_Handler)
SECTIONS
{
  .text : { KEEP(*(.isr_vector)) *(.text*) *(.rodata*) } > FLASH
  .ARM.exidx : { *(.ARM.exidx*) } > FLASH
  . = ALIGN(4);
  __etext = .;
  .data : AT (__etext) { __data_start__ = .; *(.data*) . = ALIGN(4); __data_end__ = .; } > RAM
  .bss  : { __bss_start__ = .; *(.bss*) *(COMMON) . = ALIGN(4); __bss_end__ = .; } > RAM
  . = ALIGN(8);
  __heap_start__ = .;
  /* 64K of stack. modulate_symbol alone holds a few kB of int64 locals
   * and the call depth adds more, so 64K is comfortable without being
   * arbitrary. (An earlier comment here blamed a 16K stack for a
   * test_tx lockup -- that was wrong: the dump showed SP one frame below
   * _estack, i.e. an EMPTY stack, and the real cause was undefined
   * behaviour in fft.c/tx.c. Kept as a caution against reading QEMU's
   * post-lockup register dump as the faulting context.) */
  __heap_end__ = ORIGIN(RAM) + LENGTH(RAM) - 64K;
  _estack = ORIGIN(RAM) + LENGTH(RAM);
}
