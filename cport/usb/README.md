# TinyUSB integration for the OFDM modem

Brings the modem up as its own USB device on an STM32H743, using
TinyUSB's dwc2 port. `../src/usb_proto.c` and `../src/usb_desc.h` are
shared with the host driver, so device and host cannot drift apart.

    tusb_config.h      one vendor interface, 2 bulk endpoints, nothing else
    usb_descriptors.c  TinyUSB callbacks fed from usb_desc.h
    bsp_stm32h7.c      PWR / clocks / pins / NVIC, register level
    usb_main.c         RAM-resident bring-up image with a progress beacon

## Dependencies, and why they are not vendored

TinyUSB's STM32 port needs ST's CMSIS device headers, which need ARM's
CMSIS core headers. That is ~150 MB of third-party source to hold a
dependency-free C port hostage, so they are fetched rather than
committed:

    git clone --depth 1 https://github.com/hathach/tinyusb            /tmp/tinyusb
    git clone --depth 1 https://github.com/STMicroelectronics/cmsis_device_h7 /tmp/cmsis_h7
    git clone --depth 1 https://github.com/ARM-software/CMSIS_5       /tmp/cmsis5
    make -C cport usbfw TINYUSB=/tmp/tinyusb CMSIS_H7=/tmp/cmsis_h7 CMSIS5=/tmp/cmsis5

Nothing else in `cport/` depends on any of it.

## Running it

The image runs from RAM -- loaded over JTAG, flash never written, undone
by a reset:

    make -C cport usbfw ...
    openocd -f tools/esp32-probe/stm32h7-rbb.cfg \
      -c init -c halt -c "load_image cport/build/usb_bringup.elf" \
      -c "reg sp 0x20020000" -c "reg pc 0x00000000" -c resume -c exit

Progress is a struct at **0x20000000**, readable while the core runs:

    magic 0x05BEAC01 | stage | mounted | rx | tx | frames_in | frames_out
                     | loops | suspended | resumed | supply_waits
                     | used_regulator

    stage 1 entered   2 waiting for VDD33USB   3 BSP done
          4 tusb_init 5 looping                6 mounted (enumerated)

`loops` advancing with `stage` stuck is a device waiting for something,
not a crash -- which is the distinction the first version of this could
not make.

## Two things that cost real time

**VDD33USB has two sources and they need opposite settings.** A board
that feeds it from its own 3.3 V rail wants `USB33DEN` alone; a board
that derives it from VBUS also wants `USBREGEN`. Setting `USBREGEN` on
the first kind does not just waste a regulator -- `USB33RDY` stays low
and the device never comes up. Measured on the part: `PWR_CR3` read
`0x03000042` with both enables set and ready clear, then `0x05000042`
the instant `USBREGEN` was cleared. `ofdm_usb_bsp_supply_ready()` now
tries external first and falls back, and records which worked.

**A RAM-resident image has no vector table.** TinyUSB enables the OTG
interrupt in the NVIC from inside `dcd_init()`; with no table installed,
the first USB interrupt vectors through whatever VTOR still points at.
So the bring-up image masks the line and polls `tud_int_handler()`
instead. A flashed build should install a table and use
`ofdm_usb_bsp_irq_enable()`.

## Status

Software chain verified on a live STM32H743, all read back from the
part: HSI48 selected as the USB clock (`RCC_D2CCIP2R` USBSEL=3),
OTG_FS clocked (`RCC_AHB1ENR` bit 27), PA11/PA12 in AF10, `GCCFG`
PWRDWN set and VBDEN clear, `DCTL` soft-disconnect released, supply
ready, firmware looping.

**Not yet enumerated by a host.** With the above all correct and a cable
attached, no device appeared on the bus -- which points at D+/D- from
the connector not reaching PA11/PA12 on this particular board rather
than at anything above. Worth checking, in order: that the cable is a
data cable and not charge-only; that the connector used is the MCU's and
not an on-board debugger's; and what the board's schematic actually
routes to PA11/PA12.

The OTG_HS alternative (PB14/PB15, `-DOFDM_USB_RHPORT=1`) was tried and
is **inconclusive**: that core came back with `GCCFG` = 0 (PHY powered
down) and `DCTL` soft-disconnect asserted, i.e. it never initialised, so
it was not a fair test of that wiring. Making the HS port come up is the
next thing to try if the schematic says the connector goes there.
