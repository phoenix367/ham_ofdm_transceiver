# OFDM modem as a USB device

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

## Layout

    ofdm_modem.py        host driver (pyusb), and the CLI below
    test_ofdm_modem.py   end-to-end test against the device-side C
    99-ofdm-modem.rules  udev: stable names, non-root access

Device side, in `cport/`:

    src/usb_proto.[ch]   the wire protocol -- framing and codecs
    src/usb_modem.[ch]   binds the protocol to a station
    src/usb_desc.h       USB descriptors and the serial builder
    bench/usb_modem_emu.c  the device, minus the USB peripheral

## Trying it without hardware

`usb_modem_emu` runs the real device-side C over a pipe, speaking the
same wire protocol:

    make -C cport usbemu
    ./host/test_ofdm_modem.py                      # 11 checks
    ./host/ofdm_modem.py --emulate cport/build/usb_modem_emu --send HI

A passing run exercises both ends of the protocol as shipped. It does
not exercise the USB peripheral driver, which cannot be tested
off-target.

## With hardware

    pip install pyusb
    sudo cp host/99-ofdm-modem.rules /etc/udev/rules.d/
    sudo udevadm control --reload && sudo udevadm trigger
    ./host/ofdm_modem.py                           # finds it by VID/PID
    ./host/ofdm_modem.py --serial 2400...3436      # or by unit

## What is NOT here

**The USB peripheral driver.** Everything above is transport-agnostic:
bytes arrive through `usb_modem_rx()` and leave through
`usb_modem_poll()`, and the peripheral's whole job is to move those
bytes between two bulk endpoints. Binding that to the STM32H7's OTG_FS
core means either ST's USB device library or TinyUSB, neither of which
is vendored here -- the C port is deliberately dependency-free, and
adding a USB stack to it is a decision about the project, not a detail
of this protocol.

It is also the one part that cannot be verified without flashing the
board and plugging in a cable, and untested USB enumeration code is
worth very little. The descriptors in `usb_desc.h` are self-checked for
structural consistency (a host walks the configuration exactly the way
that check does) but have not been enumerated by a real host controller.

**Windows binding.** Linux and macOS attach no driver to a vendor-class
interface, so libusb claims it unaided. Windows needs either an `.inf`
pointing at WinUSB or an MS OS 2.0 descriptor set in the firmware. The
latter is a long blob that cannot be validated without a Windows host,
so it is not shipped half-checked.
