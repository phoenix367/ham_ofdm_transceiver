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

## Status of the station binding: INCOMPLETE, but correctly narrowed

`usb_main.c` runs a real `station_t` behind the endpoints through
`usb_modem.c`, with a stub PHY (no codec on this board, so the link
layer's timers and rate ladder run for real while the samples go
nowhere). It **enumerates** and stays mounted; the data path does not
work: it emits ~11-15 frames and then goes quiet, and never answers
`CMD_INFO` at all.

The image is now **interrupt-driven** with its own vector table (VTOR),
which is better engineering regardless of this bug -- see below.

Instrumenting the stall settled what it is NOT. At the moment it is
stuck:

    txq_len      0     the staging queue is EMPTY
    dropped      0     nothing was discarded
    write_avail  150   the endpoint has room
    last_poll    0     there is nothing to send
    mounted_now  1     still enumerated

So the device is not blocked by flow control, a full FIFO, or a lost
frame. It simply stops PRODUCING. The earlier reading -- that 549 bytes
was suspiciously close to `CFG_TUD_VENDOR_TX_BUFSIZE` (512) -- was a
coincidence, and chasing it cost time. The search belongs in
`usb_modem.c`'s command path: `rx_bytes` shows the five bytes of
`CMD_INFO` arriving, and no `RSP_INFO` ever comes back.

Hypotheses tested and DISPROVEN, each by measurement:

- **the diagnostic firehose** -- gated behind `UP_CFG_DIAG_STREAM` (off
  by default now, and dropped above a half-full queue, which is right on
  its own merits). Stall unchanged.
- **loop starvation under polling** -- `station_poll_tx()` every
  iteration cut the loop from 653 M to 6.6 M; rate-limiting restored it.
  Stall unchanged.
- **cache versus dwc2 DMA** -- `CFG_TUD_DWC2_DMA_ENABLE` defaults to 0
  and slave mode to 1, so the CPU copies every byte and the D-cache is
  irrelevant.
- **polling instead of interrupts** -- a real hypothesis, since in slave
  mode the driver refills the IN FIFO from the TX-FIFO-empty interrupt.
  Now genuinely interrupt-driven, `isr_count` 12.4 M, no faults. Stall
  unchanged.
- **the clock** -- `s_cycles` reads 52.6 s of real time at 400 MHz.

Three real bugs were found and fixed along the way, none of them the
cause:

- the drain loop called `usb_modem_poll()`, which REMOVES bytes, before
  checking `tud_vendor_write_available()` -- a briefly full endpoint
  discarded data mid-frame and desynchronised the host's parser;
- `station_on_tx_end()` was never called, so the station believed it was
  permanently transmitting;
- installing a vector table immediately caught `ICSR = 0x80F`, an
  unhandled **SysTick** left running, with `CFSR` and `HFSR` both zero.
  Treating every vector as fatal turns any stray source into a dead
  device, so a genuine fault now stops and reports while an unexpected
  interrupt is masked, counted and survived.

The **stub** image (commit 74a3936) remains the one that demonstrably
exchanges data over USB end to end.
