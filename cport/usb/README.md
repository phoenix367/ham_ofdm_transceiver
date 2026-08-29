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

## Status: enumerates

Confirmed on a live STM32H743 with the image running from RAM:

    Bus 001 Device 035: ID 1209:0001 Generic pid.codes Test PID
      bDeviceClass          255 Vendor Specific Class
      iProduct                2 OFDM Modem
      iSerial                 3 240041000551333438363436
      bInterfaceClass       255 Vendor Specific Class
      bInterfaceSubClass     79 (0x4F)
      bInterfaceProtocol      1
        bEndpointAddress     0x01  EP 1 OUT
        bEndpointAddress     0x81  EP 1 IN

and device-side `stage` = 6 (mounted), `used_regulator` = 0. The serial
is the part's own 96-bit unique ID, which is the whole point: two modems
on one host are distinguishable, and `/dev/ofdm-modem-<serial>` is
stable.

Remaining to talk to it: the raw node comes up `root:root`, so libusb
cannot open it as a user until the rule is installed --

    sudo cp host/99-ofdm-modem.rules /etc/udev/rules.d/
    sudo udevadm control --reload && sudo udevadm trigger

## Three things that cost real time

**A USB-C-to-C cable will not work on most dev boards.** They omit the
CC pull-down resistors (Rd, 5.1 kOhm) that mark the board as a device,
so a C-to-C host never detects an attach and never sources VBUS.
Nothing appears on the bus and every register on the device reads
correct, which is an expensive combination to debug. A C-to-A cable
needs no CC negotiation and works. This is what the first failure was.

**VDD33USB has two sources needing OPPOSITE settings.** A board feeding
it from its own 3.3 V rail wants `USB33DEN` alone; one deriving it from
VBUS also wants `USBREGEN`. Setting `USBREGEN` on the first kind does
not just waste a regulator -- `USB33RDY` stays low and the device never
comes up. Measured: `PWR_CR3` = `0x03000042` with both enables set and
ready clear, then `0x05000042` the instant `USBREGEN` was cleared.
`ofdm_usb_bsp_supply_ready()` tries external first and falls back,
recording which worked. The board under test uses the external supply.

**A RAM-resident image has no vector table, and faults vanish into the
previous firmware.** TinyUSB enables the OTG interrupt inside
`dcd_init()`, so the bring-up image masks the line and polls
`tud_int_handler()`. But that only covers interrupts: after poking
`DCTL` over JTAG and restarting in place, the image faulted and `PC`
came back as `0x080051be` -- the *resident* firmware's HardFault
handler, reached through the stale VTOR. The beacon showed `stage` 4,
`loops` 1 and nothing else, which looks like a hang. Restart from a
clean core (`SYSRESETREQ` via `0xE000ED0C = 0x05FA0004`) rather than
in place; `reset halt` does not work here because NRST is not wired to
the probe. A flashed build should install its own table and report
faults.

## Status of the station binding: INCOMPLETE

`usb_main.c` now runs a real `station_t` behind the endpoints via
`usb_modem.c`, with a stub PHY (no codec on this board, so the link
layer's timers and rate ladder run for real while the samples go
nowhere).

It **enumerates correctly** -- `stage` 6, `mounted` 1, the host driver
opens it by serial -- but the data path **stalls**: the device answers,
sends exactly 549 bytes in 16 frames, and then both directions stop
while the main loop keeps running. Reproduced identically across four
builds, so it is deterministic rather than a race.

Ruled out, each by measurement rather than by argument:

- **the diagnostic firehose.** A station with no radio times out
  constantly and every event was a frame. Gating the stream behind
  `UP_CFG_DIAG_STREAM` (now off by default, and dropped above a
  half-full queue) is right on its own merits -- a debug stream must
  never crowd out command replies -- but the stall was unchanged.
- **loop starvation.** With interrupts masked, `tud_int_handler()` runs
  only as often as the loop does, and calling `station_poll_tx()` every
  iteration cut the loop from 653 M to 6.6 M. Rate-limiting the station
  restored it to 14.5 M; the stall was unchanged.
- **cache versus DMA.** The buffers are in AXI-SRAM with the D-cache on,
  which would be a real bug if the dwc2 core were mastering them. It is
  not: `CFG_TUD_DWC2_DMA_ENABLE` defaults to 0 and slave mode to 1, so
  the CPU copies every byte.
- **the clock.** `s_cycles` reads 21.0 G, i.e. 52.6 s at 400 MHz, so
  the station's timers are being driven correctly.

Two genuine bugs were found and fixed on the way, neither of which was
the cause: the drain loop called `usb_modem_poll()` (which REMOVES
bytes) before checking `tud_vendor_write_available()`, discarding data
and desynchronising the host's parser when the endpoint was briefly
full; and `station_on_tx_end()` was never called, so the station
believed it was permanently on the air.

549 bytes is suspiciously close to `CFG_TUD_VENDOR_TX_BUFSIZE` (512),
which points at the endpoint buffer filling and never draining. The next
diagnostic is to put `txq_len`, `dropped` and
`tud_vendor_write_available()` into the beacon and watch them at the
moment it stops -- reading them from guessed struct offsets produced
garbage and should not be repeated.

Until that is resolved, the **stub** bring-up image (commit 74a3936) is
the one that demonstrably exchanges data over USB end to end.
