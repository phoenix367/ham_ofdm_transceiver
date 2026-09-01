---
name: test-usb
description: Build, enumerate and talk to the modem as a USB device - the TinyUSB firmware on the STM32H743, the host driver, and the hardware-free emulator. Use after any change under cport/src/usb_*, cport/usb/, or host/.
---

# The modem as a USB device

Three layers, tested at three levels. Run them in this order; each one
localises a failure the next would only report.

## 1. Protocol, no hardware (seconds)

```bash
cd cport && make test                       # includes test_usb, 11 checks
make usbemu && cd .. && ./host/test_ofdm_modem.py     # 11 checks, real device-side C over a pipe
```

The emulator runs the actual `usb_modem.c`/`usb_proto.c`; a passing run
exercises both ends of the protocol as shipped. It cannot exercise the
USB peripheral driver, which is the only thing level 3 adds.

## 2. Firmware builds (needs the third-party submodules)

```bash
make -C cport deps      # once per checkout; skip if third_party/ is populated
make -C cport usbfw
```

TinyUSB and the two CMSIS trees are submodules pinned by commit under
`cport/third_party/`, not initialised by a plain clone -- see
`cport/usb/README.md`. The build stops with the `make deps` line if
they are missing, so an empty `third_party/` cannot look like a broken
port.

## 3. On the board

RAM bring-up (`make run-usb_bringup` after `make usbfw`), or flash it so
it survives reset (`make flash-usb && make reset`). Plug a **USB-C-to-A**
cable from the board's own connector, then:

```bash
lsusb -d 1209:0001 -v 2>/dev/null | grep -E 'iProduct|iSerial|bInterfaceSubClass|bEndpointAddress'
ls -l /dev/ofdm-modem*                       # udev rule installed?
./host/ofdm_modem.py --send "HELLO" --listen 1.5
./host/ofdm_modem.py --serial 2400...        # by unit, when several
```

Expect `iSerial` = the part's 24-hex UID, subclass 79, `ping ok`,
`resyncs 0`, and a `drained N stale bytes` line on every open after the
first. The beacon (`target-jtag`) shows `stage 6` = mounted and the
frame/byte counters; `isr_count` in the hundreds is healthy, in the
tens of millions is the wedge below.

## Traps -- every one cost real time

- **A C-to-C cable does not work on most dev boards** (no CC
  pull-downs, so no VBUS). Every register reads correct and nothing
  appears on the bus. C-to-A.
- **VDD33USB has two sources needing opposite settings.** This board
  supplies it externally: `USBREGEN` must stay OFF or `USB33RDY` never
  asserts. The BSP tries external first and records which worked.
- **`clear_halt(EP_IN)` wedges the device after one packet.** The device
  pushes status unprompted, so EP_IN is armed at open; TinyUSB drops its
  BUSY flag on clear-stall without disarming the hardware. The host
  driver DRAINS instead. Any other client must too. Details and the
  measurements: `cport/usb/README.md`.
- **The udev rule** gives non-root access and the stable name; without
  it libusb fails with `Errno 13`.
- **VID:PID 1209:0001 is the pid.codes TEST id** -- development only.

## Reading a failure

`no response to CMD_INFO` with a healthy beacon: read `txq_len` /
`write_avail` in the beacon. Queue empty and FIFO room means the
endpoint is wedged (trap 3); FIFO full means the host is not reading.
