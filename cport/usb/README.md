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

## Status of the station binding: WORKING

`usb_main.c` runs a real `station_t` behind the endpoints through
`usb_modem.c`, interrupt-driven with its own vector table, with a stub
PHY (no codec on this board, so the link layer's timers and rate ladder
run for real while the samples go nowhere). Verified on the part:

    open #1  drained 1924 stale bytes   ping ok   submit ok   resyncs 0
    open #2  drained  185               ping ok   submit ok   resyncs 0
    open #3  drained  111               ping ok   submit ok   resyncs 0
    killed mid-read at 3 s, reopened:
             drained   37               ping ok   submit ok   resyncs 0

    beacon after: isr_count 293, fault 0, dropped 0, write_avail 401

### What the stall was

For most of a day the station image enumerated, answered nothing, and
went quiet after one packet. The instrumentation that finally settled
it read, at the moment of the stall:

    txq_len       0        staging queue empty
    write_avail 150        of a 512-byte TinyUSB TX FIFO
    tx_bytes    399

362 bytes were sitting INSIDE TinyUSB's transmit FIFO, unsent -- and
362 = 29 (the `RSP_INFO` reply) + 9 x 37 (status frames), exactly, while
399 - 362 = 37 = precisely one status frame ever reached the wire. An
earlier reading of the same numbers as "the device stops producing" was
wrong: it was producing fine, and TinyUSB was accepting the frames into
a FIFO it never drained. The endpoint was wedged after its first
transfer, and `tud_vendor_write_flush()` refuses to start another while
it believes one is in flight.

The cause is an interaction between an ordinary host habit and TinyUSB's
clear-stall handling, confirmed from the source:

1. The device pushes status unprompted every 0.5 s, so by the time a
   host opens it, EP_IN is already **armed** in hardware with a frame
   waiting for an IN token. (The stub image never spoke first, which is
   the only reason it worked.)
2. The host driver called `clear_halt(EP_IN)` on open -- a common
   recovery idiom, and one I had added myself three fixes earlier.
3. `usbd_edpt_clear_stall` (`usbd.c:1698`) clears its software BUSY flag
   *unconditionally* -- its own comment calls this "long-standing
   behavior" -- while the dwc2 `dcd_edpt_clear_stall` (`dcd_dwc2.c:715`)
   only clears the STALL bit and resets the PID. **Nothing disarms the
   hardware.**
4. The next flush sees "not busy" and re-arms the endpoint on top of the
   live transfer: DIEPTSIZ rewritten with EPENA already set, which the
   dwc2 core does not define. One packet escapes; the completion no
   longer matches what TinyUSB thinks it started; BUSY never clears.
5. Side signature: `isr_count` climbs to tens of millions, because the
   TX-FIFO-empty interrupt is level-triggered and stays asserted.

Tested by prediction, both ways. Gating status frames until the host had
spoken (so EP_IN was idle at open) made the first open work and -- as
the mechanism predicts -- every later open still wedged, since by then
the device was pushing again. That is what turned a hypothesis into a
cause.

### The fix

Host-side: **consume the armed transfer instead of resetting the
endpoint.** `_UsbTransport` reads EP_IN with short timeouts until it
goes quiet and discards what it finds (the parser resyncs past stale
frames anyway). `clear_halt` is kept for EP_OUT only, where three
consecutive opens against an armed endpoint were measured harmless.
The device is unchanged in behaviour: it pushes status whenever mounted,
which is the right thing for a modem to do, and is the case a driver
must handle.

The device is not made robust to `clear_halt(EP_IN)` itself; that would
mean changing TinyUSB's clear-stall path, which is its business and not
this project's. Any other host driver for this device must drain, not
reset.

### Five hypotheses disproven on the way, each by measurement

- the diagnostic firehose (gated behind `UP_CFG_DIAG_STREAM`, off by
  default, and shed under backpressure -- right on its own merits, but
  not the cause);
- loop starvation under polling (rate-limiting the station restored
  the loop from 6.6 M to 14.5 M iterations; stall unchanged);
- cache versus dwc2 DMA (`CFG_TUD_DWC2_DMA_ENABLE` defaults to 0 --
  slave mode, the CPU copies every byte);
- polling instead of interrupts (a real hypothesis in slave mode; made
  genuinely interrupt-driven, 12.4 M ISRs, no faults, stall unchanged);
- the clock (`s_cycles` read 52.6 s of real time).

Three unrelated bugs fixed on the way: a drain loop that removed bytes
from the queue before checking the endpoint had room (discarding data
mid-frame); `station_on_tx_end()` never being called; and an unhandled
SysTick that a catch-all fault handler treated as fatal (`ICSR` 0x80F,
`CFSR`/`HFSR` zero) -- genuine faults now stop and report, unexpected
interrupts are masked, counted and survived.
