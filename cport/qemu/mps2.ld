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
  __heap_end__ = ORIGIN(RAM) + LENGTH(RAM) - 16K;
  _estack = ORIGIN(RAM) + LENGTH(RAM);
}
