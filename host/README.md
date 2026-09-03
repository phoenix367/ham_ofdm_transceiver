# Host-side tools for the OFDM modem

Everything a computer runs to talk to the modem over USB: the driver that
opens it, and the applications built on that driver -- a KISS TNC, an IP
tunnel, and a push-to-talk voice app. The device-side firmware is in
`cport/`; this directory is its other end.

## The modem is a USB device, opened by identity

The modem enumerates as its own device rather than as a USB-serial
bridge. That is the point: a bridge appears as one more `/dev/ttyACM*`
among however many adapters are attached, numbered by enumeration order,
so the host has to guess which port is the modem -- and guesses wrong
after a reboot or a re-plug. This device is opened by identity instead.

    VID:PID   1209:0001        (pid.codes TEST id -- development only)
    class     0xFF vendor, subclass 0x4F 'O', protocol 0x01
    endpoints 0x01 bulk OUT, 0x81 bulk IN, 64 bytes
    serial    24 hex chars, the STM32 96-bit unique ID

Two modems on one machine are therefore always distinguishable, and
`99-ofdm-modem.rules` gives each a stable `/dev/ofdm-modem-<serial>`.

On open the driver **drains** EP_IN -- reads until the device goes quiet
and discards the backlog -- and reports it:

    drained   185 stale bytes left by a previous session

That is deliberate and must stay. The device pushes status unprompted,
so its IN endpoint is usually armed with a frame when a host opens; the
obvious alternative, `clear_halt(EP_IN)`, wedges TinyUSB's endpoint after
exactly one packet (`cport/usb/README.md`, "What the stall was"). Any
other client for this device must drain, not reset. `ofdm_modem.py`'s
serial-string read is also retried and clears pyusb's poisoned langid
cache -- a control transfer issued while the device pushes bulk
transiently fails, and pyusb caches the failure (`test_modem.py` pins
both without hardware).

## Layout

    ofdm_modem.py        the driver (pyusb) and a small CLI
    kiss_bridge.py       a KISS TNC over the modem's ARQ
    ip_tun.py            IP over the link, no AX.25 in the path
    ax25_ip.sh           bring up / tear down / inspect the AX.25 IP path
    webvoice/            push-to-talk voice (its own README)
    measure_ax25.sh      throughput through the AX.25 path
    measure_sendfile.sh  the modem's own file-transfer throughput
    99-ofdm-modem.rules  udev: stable names, non-root access

    test_ofdm_modem.py   end-to-end against the device-side C (11 checks)
    test_modem.py        the host driver in isolation (7)
    test_kiss.py         KISS codec and mapping (41)
    test_ip_tun.py       ip_tun policy, no root/board/TUN (32)

Device side, in `cport/`: `src/usb_proto.[ch]` (the wire protocol),
`src/usb_modem.[ch]` (binds it to a station), `usb/usb_descriptors.c` (USB descriptors and the serial builder), `bench/usb_modem_emu.c` (the device
minus the USB peripheral).

## The applications

**KISS TNC** (`kiss_bridge.py`) presents a standard KISS interface so
existing packet-radio software attaches to this modem. It stays a HOST
program on purpose: the board's protocol already carries rung, SNR,
capabilities, temperature and broadcast pacing, none of which KISS can
express, so putting KISS in the firmware would duplicate the transport
and hide the adaptation. Frames are admitted by AIR TIME, not length (256
bytes is 2.6 s at rung 12 and 279 s at rung 0), from a table GENERATED
from the link model and asserted never optimistic. It imports only pyusb,
not NumPy/SciPy, so it starts wherever pyusb does.

**IP tunnel** (`ip_tun.py`, `ax25_ip.sh`) carries IP over the link.
`ip_tun.py` is the direct path with no AX.25 framing; `ax25_ip.sh` sets
up the AX.25/KISS route for interoperating with standard tooling. Both
are host-side and testable without a board (`test_ip_tun.py`).

**Voice** (`webvoice/`) is the push-to-talk app: LSCodec-25Hz speech at
250 bit/s over broadcast, decoded as it arrives. It has its own README
and its own external dependency (the codec submodule and checkpoints).

## Trying it without hardware

`usb_modem_emu` runs the real device-side C over a pipe, speaking the
same wire protocol, so the whole transport is exercised off-target:

    make -C cport usbemu
    ./host/test_ofdm_modem.py                      # 11 checks
    ./host/ofdm_modem.py --emulate cport/build/usb_modem_emu --send HI

`test_modem.py`, `test_kiss.py` and `test_ip_tun.py` need no emulator at
all -- they exercise the host code directly. A passing run of all four
covers both ends of the protocol as shipped, and the KISS and IP policy
in full. None of them exercises the USB peripheral driver, which cannot
be tested off-target.

## With hardware

    pip install pyusb
    sudo cp host/99-ofdm-modem.rules /etc/udev/rules.d/
    sudo udevadm control --reload && sudo udevadm trigger
    ./host/ofdm_modem.py                           # finds it by VID/PID
    ./host/ofdm_modem.py --serial 2400...3436      # or by unit

## The other end

Everything here is transport-agnostic: on the device, bytes arrive
through `usb_modem_rx()` and leave through `usb_modem_poll()`, and the
USB peripheral's whole job is to move them between two bulk endpoints.
That peripheral driver lives in `cport/usb/` -- `usb_radio_main.c` and
friends on TinyUSB (a pinned submodule under `cport/third_party/`),
bound to the STM32H7's OTG_FS core in `bsp_stm32h7.c`. It runs on the
boards; the two-station stand in this session drove the whole protocol
over it. The descriptors are also self-checked for structural
consistency off-target, so a host walks the configuration exactly the
way that check does.

**What is still not shipped: Windows binding.** Linux and macOS attach
no driver to a vendor-class interface, so libusb claims it unaided.
Windows needs either an `.inf` pointing at WinUSB or an MS OS 2.0
descriptor set in the firmware. The latter is a long blob that cannot
be validated without a Windows host, so it is not shipped half-checked.
