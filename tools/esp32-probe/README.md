# ESP32 as an SWD/JTAG probe for the STM32

Turns a bare ESP32 devkit into an OpenOCD adapter, so the C port can be
flashed and debugged on real silicon -- which is what FEASIBILITY.md's
"Pending for exactness" needs (on-target DWT->CYCCNT counts; QEMU is
functional, not cycle-accurate).

    esp32_rbb/     probe firmware (Arduino sketch)
    rbb_bridge.py  TCP <-> serial bridge OpenOCD connects to

## Why this shape

OpenOCD's `remote_bitbang` driver speaks one ASCII character per pin
edge over a socket, so any microcontroller that can toggle GPIOs can be
the adapter, and OpenOCD keeps its real target support -- including the
`stm32h7x` flash driver, which is the part a hand-rolled SWD flasher
would have to reimplement (H7 flash is 256-bit words with ECC, in two
banks).

Two things measured on this machine rather than assumed:

- **Ubuntu's OpenOCD 0.12 has `remote_bitbang`, but JTAG only.**
  `transport select swd` answers *"only one transport option; autoselect
  'jtag'"*. SWD support (`O o c d e f g`) is in OpenOCD **master**.
- The `cmsis-dap` driver in 0.12 has only HID and usb_bulk backends, so
  a WiFi CMSIS-DAP firmware (wireless-esp8266-dap) cannot be driven by
  it. That is why this uses `remote_bitbang` and not CMSIS-DAP.

The firmware implements BOTH halves of the protocol, so it works over
JTAG with the packaged OpenOCD today and over SWD once master is built.

Serial, not WiFi: every read request (`R`, `c`) is a round trip that
OpenOCD blocks on, so link latency dominates and bandwidth is nearly
irrelevant.

## Wiring

ESP32 GPIOs are 3.3 V and STM32 debug pins are 3.3 V, so they connect
directly. **Common ground is required**; power the STM32 from its own
supply, not from the ESP32.

| ESP32 | signal | STM32H7 pin | needed for |
|---|---|---|---|
| 18 | TCK / SWCLK | PA14 | both |
| 19 | TMS / SWDIO | PA13 | both |
| 21 | TDI | PA15 | JTAG only |
| 22 | TDO | PB3 | JTAG only |
| 23 | nTRST | PB4 | JTAG only, optional |
| 25 | nSRST | NRST | optional, both |
| GND | GND | GND | **required** |

### Which transport can I use?

Look at the board for a debug header:

- **4 pins marked SWD / SWDIO / SWCLK / GND** (or a 2x5 1.27 mm Cortex
  Debug header): SWD only in practice -> needs OpenOCD from master.
- **2x10 0.1" header marked JTAG**, or a Nucleo/Discovery whose morpho
  headers expose **PA15, PB3 and PB4**: JTAG works with the packaged
  OpenOCD, no rebuild.
- Nucleo/Discovery boards: PA13/PA14 go to the on-board ST-LINK. Remove
  the ST-LINK jumpers (usually labelled `ST-LINK`/`NRST`+`SWD`) before
  driving those lines from outside, or the two probes will fight.

If PA15/PB3 are not brought out, use SWD -- it is two wires and every
STM32 board exposes it.

## Building OpenOCD from master (for SWD)

Only `remote_bitbang` is needed, so none of the USB adapter libraries
are required:

    sudo apt install -y autoconf automake libtool     # texinfo only for docs
    git clone --depth 1 https://github.com/openocd-org/openocd
    cd openocd && git submodule update --init --depth 1
    ./bootstrap
    ./configure --prefix=$HOME/.local --enable-remote_bitbang \
                --disable-doxygen-html --disable-werror
    make -j$(nproc) && make install

## Running

    # 1. flash the probe firmware
    arduino --upload --board esp32:esp32:esp32 --port /dev/ttyUSB0 \
            esp32_rbb/esp32_rbb.ino

    # 2. bridge the serial port to a socket
    ./rbb_bridge.py --port /dev/ttyUSB0 --baud 921600 --listen 3335 &

    # 3. attach
    openocd -f stm32h7-rbb.cfg

Expect slow-but-workable throughput: remote_bitbang is one character per
edge, so at 921600 baud the ceiling is around 90 k edges/s. Bulk flash
writes stream through a RAM-resident loader and batch well; the reads
are what cost.

## Nothing works?

`Error connecting DP: cannot read IDR` is the normal first failure. In
order of likelihood: no common ground; the target is not powered; SWDIO
and SWCLK swapped; an on-board ST-LINK still driving the same lines; or
`adapter speed` too high -- drop it to 100 kHz while proving the wiring.
