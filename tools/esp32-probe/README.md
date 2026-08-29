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
| 18 | TCK / SWCLK | PA14 (LQFP100 pin 76) | both |
| 19 | TMS / SWDIO | PA13 (LQFP100 pin 72) | both |
| 21 | TDI | PA15 (LQFP100 pin 77) | JTAG only |
| 22 | TDO | PB3 (LQFP100 pin 89) | JTAG only |
| 23 | nTRST | PB4 (LQFP100 pin 90) | JTAG only, **optional** -- see below |
| 25 | nSRST | NRST | optional, both |
| GND | GND | GND | **required** |

**nTRST can be left unwired.** Both configs use `reset_config srst_only`,
so OpenOCD never drives TRST, and PB4/NJTRST has an internal pull-up that
holds the TAP out of reset on its own. It is in the table because the
probe firmware can drive it, not because the link needs it. Pin numbers
above are LQFP100 (the `V` in H743**V**I); they are the standard STM32
LQFP100 debug pins, confirmed against Table 9 of DS12110 Rev 11.

For a second board on the same probe, the three push-pull control lines
have a duplicate pin each -- 26 (TCK), 27 (TMS), 13 (nTRST) -- see
[Two boards on one probe](#two-boards-on-one-probe).

### Two boards on one probe

JTAG daisy-chains, so one ESP32 can debug both boards of the audio-link
stand at once -- `stm32h7-rbb-dual.cfg`. TCK/TMS/nTRST get a driver pin
per board and nSRST is shared; only TDI/TDO chain, which is the one wire
that makes it a chain:

    ESP32 21 (TDI) --> board1 PA15
                       board1 PB3  --> board2 PA15    <-- the chain wire
                                       board2 PB3 --> ESP32 22 (TDO)

    ESP32 18 (TCK)   --> board1 PA14     ESP32 26 --> board2 PA14
    ESP32 19 (TMS)   --> board1 PA13     ESP32 27 --> board2 PA13
    ESP32 23 (nTRST) --> board1 PB4      ESP32 13 --> board2 PB4
    ESP32 25 (nSRST) --> board1 NRST  +  board2 NRST      (shared)

The three push-pull control lines are **duplicated**: each board gets its
own driver pin instead of a Y-splice. The firmware addresses lines by
mask and every duplicate sits in GPIO bank 0, so a line and its copy are
switched by the *same* `w1ts`/`w1tc` store -- zero skew between the two
boards' TCK. This is not fixing a timing problem (at ~20 kHz of TCK a
half-period is 25 us, against nanoseconds of everything else); it is one
driver per board and no splices on the breadboard. Build the probe with
`-DDUAL_PROBE=0` to get the single-board behaviour back, which
`test_pinmask.cpp` asserts is byte-for-byte the old one -- those pins are
then never driven at all.

`nSRST` is deliberately *not* duplicated. It is open-drain and wired-OR
by nature, so multi-drop is the normal arrangement there, and duplicating
it would not buy per-board reset anyway: `remote_bitbang` drives both
resets from one pair of commands (`r`..`u`).

Both boards must be powered and wired before `init`: OpenOCD examines
every TAP in the chain, so one missing board fails all of it. Keep using
`stm32h7-rbb.cfg` for single-board work.

The config declares the chain by sourcing the stock `stm32h7x.cfg` twice
under `CHIPNAME` `stmA` and `stmB`. That is safe -- the file takes
`CHIPNAME` from the environment, and its helper procs either take the
target as an argument or recover the chip name from `target current`
rather than a global. Each board contributes the same TAP pair in the
same order (cpu irlen 4, then bs irlen 5), so two boards chained either
way round give the same repeating 4,5,4,5 pattern; the declaration is
right without having to settle OpenOCD's TDI-vs-TDO ordering convention.
What the convention *does* decide is which label lands on which physical
board, so settle that by measurement:

    make ids        # stmA  uid/serial 3B0028000A51...

`ab_ids` prints each chip's 96-bit UID formatted exactly as
`ofdm_usb_serial()` does, so the label maps to a board you can then
address by `host/ofdm_modem.py --serial`.

Two consequences of the shared lines, both wanted here:

- **nSRST resets both boards at once** -- a common start of time for the
  audio link. To reset one alone, use SYSRESETREQ on that target:
  `targets stmA.cpu0; mww 0xE000ED0C 0x05FA0004`.

While OpenOCD talks to one board the other's two TAPs sit in BYPASS,
adding 2 bits to each DR scan. DAP DR scans are 35 bits, so ~6%. IR scans
do double (9 -> 18 bits) but are rare next to the DR traffic that
dominates flashing.

SWD cannot do this: it has no chaining, so two SWD targets need a mux or
a second probe.

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

    # 1. flash the probe firmware. The legacy `arduino` IDE cannot do it
    #    -- esp32 core 3.x needs arduino-cli (or IDE 2.x); the old one
    #    answers "Error: esp32: Unknown package".
    arduino-cli compile --fqbn esp32:esp32:esp32 esp32_rbb
    arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/ttyUSB0 esp32_rbb

    # host test of the GPIO layer, no hardware needed (both build arms)
    make -C ../../cport test-probe

    # 2. bridge the serial port to a socket
    ./rbb_bridge.py --port /dev/ttyUSB0 --baud 921600 --listen 3335 &

    # 3. attach
    openocd -f stm32h7-rbb.cfg

### Measured

Bench-tested with the probe flashed and NO target wired:

    2000 pipelined reads in 47 ms -> 42236 edges/s

That is 92 % of the theoretical ceiling (a read is one character out and
one back = 20 bits at 921600 baud -> 46080/s), so the serial link, not
the ESP32, is the limit -- which is why the firmware does no bit-banging
delays and `adapter speed` is absent from the config (OpenOCD reports
"This adapter doesn't support configurable speed" for remote_bitbang).

### What a correct un-wired run looks like

Before connecting anything to the STM32, `openocd -f stm32h7-rbb.cfg`
should reach the target and fail only there:

    Info : remote_bitbang driver initialized
    Error: JTAG scan chain interrogation failed: all ones

"All ones" is TDO floating on its pull-up, i.e. everything from OpenOCD
through the bridge, the serial link and the GPIOs is working and only
the wires are missing. If you do NOT get this far -- if it cannot
connect to localhost:3335 -- the problem is the bridge or the probe, not
the wiring.

## If the ESP32 reboots, so does the target

GPIO25 is wired to the STM32's NRST as an open-drain reset line, and
during the ESP32's own boot that pin is briefly low. So anything that
resets the ESP32 -- a USB re-enumeration, a brownout when a wire goes
in next to it -- **resets the STM32 too**, and a RAM-resident image is
gone. It presented as: the remote_bitbang socket dropping mid-session,
the CH340 coming back as a new USB device, `/dev/ttyUSB0` becoming
`/dev/ttyUSB1`, and the target's beacon reading somebody else's data.
Restart the bridge on the new node, then reload the image. (Or leave
NRST unwired if the target must survive the probe.)

## Nothing works?

`Error connecting DP: cannot read IDR` is the normal first failure. In
order of likelihood: no common ground; the target is not powered; SWDIO
and SWCLK swapped; an on-board ST-LINK still driving the same lines; or
`adapter speed` too high -- drop it to 100 kHz while proving the wiring.
