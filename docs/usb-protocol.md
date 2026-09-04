# OMHP over USB: Transport Binding

Specification, version 1

**Status of this document.** This is the normative description of how
the OFDM Modem Host Protocol ([modem-protocol.md](modem-protocol.md),
"OMHP") is carried over USB: the device's identity on the bus, its
endpoints, how a host opens and drives them, and the second binding of
the same byte stream to stdio. The message layer -- framing, commands,
events, structures, registries -- is specified in OMHP and is **not**
restated here. The reference implementations are `cport/src/usb_desc.h`
(identity), `cport/usb/usb_descriptors.c` (descriptors as TinyUSB emits
them), `cport/usb/usb_radio_main.c` (endpoint service), and the two
hosts `host/ofdm_modem.py` and `demoapp/usb_host.c`. The descriptor
values in §3 were read back from two attached boards with `lsusb -v`,
not copied from the source.

## Table of contents

1. Introduction (informative)
2. Conventions (normative)
3. Device identity and descriptors (normative)
4. Host operating-system binding (normative)
5. Data transfer model (normative)
6. Opening procedure (normative)
7. Timeouts and liveness (normative)
8. Transport-level faults (normative)
9. The stdio binding (normative)
10. Assignment considerations (normative)
11. Timing constants (informative)
12. Security considerations (informative)
13. References

Appendix A. Descriptor octets
Appendix B. The endpoint stall, as measured (informative)

---

## 1. Introduction

### 1.1 Scope

OMHP is a self-delimiting byte stream in each direction (OMHP §4). This
document specifies the two ways that stream is carried:

- over **USB full speed**, as a vendor-class interface with one bulk IN
  and one bulk OUT endpoint (§3–§8), which is how a board is driven;
- over **stdio**, as the same octets on a process's stdin/stdout (§9),
  which is how the device-side reference code runs without hardware.

Both bindings deliver an ordered, lossless octet stream. Neither adds
framing, addressing, or integrity of its own; OMHP's sync bytes
delimit, USB's CRC and retry protect, and there is one host per board.

### 1.2 Layering

```
    OMHP frames          A5 5A type len payload      -- modem-protocol.md
    -----------------------------------------------
    octet stream         this document
    -----------------------------------------------
    USB bulk IN/OUT  |  stdin/stdout                 -- §3-§8  |  §9
```

### 1.3 Applicability statement

This binding applies to any host that opens the modem's interface and
to any firmware image that enumerates as it (`usbfw`, `usbflash`,
`radiofw`). It assumes a full-speed USB 2.0 device with a 64-byte bulk
packet size. It is a **development** binding in one respect stated in
§10: the product ID is pid.codes' shared test ID, and a shipped unit
needs its own.

## 2. Conventions

The key words "MUST", "MUST NOT", "REQUIRED", "SHALL", "SHALL NOT",
"SHOULD", "SHOULD NOT", "RECOMMENDED", "NOT RECOMMENDED", "MAY", and
"OPTIONAL" in this document are to be interpreted as described in BCP 14
[RFC 2119] [RFC 8174] when, and only when, they appear in all capitals,
as shown here. Following RFC 2119 §6 they are used only where a
behaviour is needed for the two ends to interoperate or where departing
from it has been **measured** to cause harm; the effect of each
departure is in §12.1.

Notation follows OMHP §2: `u8`/`u16le` widths, little-endian
multi-octet fields (USB descriptors are little-endian by the USB
specification, so this is the bus's own convention), bit 0 = LSB.
Elements are written `<element>`; syntax uses `?`, `*`, `+` and `|` as
in OMHP §2.2. "The device" is the modem; "the host" is the program
that opens it; "the stack" is the device's USB device stack (TinyUSB in
the reference firmware).

## 3. Device identity and descriptors

### 3.1 Identity

A host identifies the modem by the triple below and by nothing else --
not by a serial-port name, not by enumeration order.

| field | value | meaning |
|---|---|---|
| `idVendor` | `0x1209` | pid.codes, the shared open-hardware vendor ID |
| `idProduct` | `0x0001` | pid.codes **test** PID -- development only (§10) |
| `bInterfaceClass` | `0xFF` | vendor-specific: no operating system binds a driver |
| `bInterfaceSubClass` | `0x4F` (`'O'`) | this device family |
| `bInterfaceProtocol` | `0x01` | OMHP framing **version 1** (OMHP §14.1) |
| `iSerialNumber` | string 3 | 24 uppercase hex digits: the STM32 96-bit unique ID |

The subclass/protocol pair exists so that a future incompatible framing
can be told apart **before** the host opens anything: a host MUST check
`bInterfaceProtocol` and MUST NOT open an interface whose value it does
not implement. (The reference firmware writes the interface descriptor
out by hand because TinyUSB's `TUD_VENDOR_DESCRIPTOR` macro hardcodes
both fields to zero; a declared identity that is not transmitted is
worse than none, and `lsusb` showing the macro's zeros is how that was
caught.)

### 3.2 Serial string

```
<serial> := <hexpair>{12}
<hexpair> := two uppercase hexadecimal digits of one UID octet,
             in the octet order the UID occupies at 0x1FF1E800
```

Example: `240041000551333438363436`. The device's `RSP_INFO` carries
the same twelve octets in `uid[12]` (OMHP §8.1), so a host MAY confirm
after opening that the interface it claimed belongs to the board it
addressed. Two boards on one host are always distinguishable by this
string; a host MUST NOT distinguish them by bus/port position, which
changes with every re-plug.

### 3.3 Descriptors

Device descriptor (18 octets; full octets in Appendix A):

| field | value |
|---|---|
| `bcdUSB` | `0x0200` |
| `bDeviceClass/SubClass/Protocol` | `0xFF / 0x00 / 0x00` -- vendor at device level too, so nothing binds before the configuration is read |
| `bMaxPacketSize0` | 64 |
| `bcdDevice` | `0x0100` |
| `iManufacturer / iProduct / iSerialNumber` | 1 / 2 / 3 |
| `bNumConfigurations` | 1 |

Configuration 1 (32 octets total):

| descriptor | field | value |
|---|---|---|
| configuration | `bNumInterfaces` | 1 |
| | `bmAttributes` | `0x80` -- bus powered, no remote wakeup |
| | `bMaxPower` | 50 = **100 mA** |
| interface 0, alt 0 | `bNumEndpoints` | 2 |
| | class / subclass / protocol | `0xFF / 0x4F / 0x01` |
| | `iInterface` | 4 |
| endpoint | `bEndpointAddress` | `0x01` OUT, bulk, `wMaxPacketSize` 64, interval 0 |
| endpoint | `bEndpointAddress` | `0x81` IN, bulk, `wMaxPacketSize` 64, interval 0 |

Strings (language `0x0409`, en-US):

| index | value |
|---|---|
| 1 | `ofdm-transceiver-proto` |
| 2 | `OFDM Modem` |
| 3 | serial (§3.2) |
| 4 | `OFDM modem data` |

There is no Microsoft OS descriptor set (§4.3).

## 4. Host operating-system binding

### 4.1 Linux

The kernel binds no driver to a vendor-class interface; libusb claims it
directly. Access as a non-root user REQUIRES the udev rule
`host/99-ofdm-modem.rules`, which matches
`1209:0001` **and** `ATTR{manufacturer}=="ofdm-transceiver-proto"` (the
test PID is shared with other prototypes; the manufacturer string
narrows it), grants `uaccess` / `plugdev`, and creates

```
/dev/ofdm-modem-<serial>     one per unit, stable across reboots
/dev/ofdm-modem              whichever enumerated first
```

The symlinks are for humans and shell scripts; the reference hosts open
by VID:PID and serial, not by path.

### 4.2 macOS

Same as Linux without the udev step: nothing binds, libusb claims.

### 4.3 Windows

**Not supported by this revision.** Windows binds no driver to a
vendor-class interface either, but libusb cannot claim it until either
an `.inf` names WinUSB for `1209:0001` or the device carries an MS OS
2.0 descriptor set. The firmware ships neither: the descriptor set is a
long blob that cannot be validated without a Windows host, and an
unverified one is worse than none.

## 5. Data transfer model

### 5.1 Octet stream over bulk endpoints

Each direction is one octet stream. The host writes OMHP frames to
`EP 0x01 OUT`; the device writes OMHP frames to `EP 0x81 IN`. USB
guarantees order and integrity (CRC-16 with retry); this binding adds
no framing, and OMHP's parser (OMHP §4.2) is what turns the stream back
into frames. Consequently:

- A frame MAY span any number of USB packets, and one packet MAY carry
  parts of two frames. A host MUST NOT assume packet boundaries align
  with frame boundaries in either direction.
- The device services its OUT endpoint in reads of up to 512 octets
  and its IN endpoint in writes bounded by the stack's 512-octet
  transmit FIFO; a frame larger than the FIFO (up to 3341 octets, OMHP
  §4.1) crosses in several writes. Neither figure is visible to a
  conforming host and neither MUST be relied on.

### 5.2 No zero-length packets

The device does **not** append a zero-length packet after a transfer
whose length is a multiple of 64. A host reading with a buffer larger
than the transfer therefore cannot rely on a short packet to end the
read; the read ends on its timeout, and the host MUST keep the octets
transferred before the timeout rather than treating the timeout as
"nothing arrived". Both reference hosts do (pyusb ≥ 1.0.2 returns the
transferred count on `LIBUSB_ERROR_TIMEOUT`; `usb_host.c` accepts
`rc == 0 || rc == LIBUSB_ERROR_TIMEOUT`). A host library that discards
partial data on timeout is not usable with this device without a
wrapper.

A host writing a frame whose length is a multiple of 64 SHOULD NOT
append a zero-length packet either; the device's stack accepts it
harmlessly, but the device's parser needs none.

### 5.3 Unsolicited traffic

The device transmits without being asked: `EVT_STATUS` every 0.5 s from
power-up, and `EVT_MESSAGE` / `EVT_BCAST` / `EVT_LOG` / `EVT_DIAG` as
they arise (OMHP §7). Two properties follow and bind the host:

- **EP IN is armed at essentially any instant**, with a frame waiting
  for an IN token. This is what makes §6.3 necessary.
- The device's staging ring is 4096 octets and is drained only as the
  host reads (OMHP §7). A host MUST read continuously -- a poll at
  least every few hundred milliseconds -- or frames are dropped whole
  at the device (`dropped` counter). Dropping rather than blocking is
  by design: the modem MUST NOT stall its air-side loop on an absent
  host.

### 5.4 Half-duplex service on the device

The device's USB service and its modem run in one loop. During its
worst blocking receive burst (2283 ms measured at EXTREME, the
end-of-frame commit) the device reads no OUT packets and writes no IN
packets; the bus-level effect is a NAK on every token for that long.
This is healthy behaviour and is what sizes the host's write timeout
(§7.1).

## 6. Opening procedure

A host MUST perform the following in order.

### 6.1 Locate

Enumerate devices matching `1209:0001`; read each candidate's
`bInterfaceProtocol` and serial string. Selecting by serial is REQUIRED
when more than one candidate matches; a host MUST NOT pick one of
several silently.

Reading the serial string MAY fail transiently: a control transfer
issued while the device is pushing bulk data can be refused, and pyusb
then caches the failed language-ID fetch so that every later
`get_string` on that handle fails instantly. A host MUST retry the read
for at least the device's worst blocking burst (§5.4; the reference
hosts use 10 × 300 ms) and, through pyusb, MUST clear the handle's
cached language IDs (`dev._langids`) between attempts -- a retry that
does not is a no-op. Measured: 3 opens in 12 raised `ValueError: no
langid`, which reads like a permission problem and is not.

### 6.2 Claim

Set configuration 1 and claim interface 0. Exactly one host holds the
interface; a second claimant receives `EBUSY` and MUST NOT retry in a
loop -- the reference hosts report "the modem is already claimed by
another program" and exit. There is no sharing and no multiplexing at
this layer.

### 6.3 Recover EP OUT; never reset EP IN

A host MAY issue `clear_halt` on **EP 0x01 OUT** before its first
write, to recover from a previous host that died mid-transfer (three
consecutive opens against an armed OUT endpoint were measured
harmless).

A host MUST NOT issue `clear_halt` on **EP 0x81 IN** at any time. The
endpoint is armed with an unsolicited frame (§5.3); the stack's
clear-stall path drops its software "busy" state without disarming the
hardware transfer, and the next write re-arms on top of the live one.
The endpoint then delivers exactly one more packet and wedges for the
life of the enumeration, with the device otherwise healthy. Appendix B
gives the measured sequence. The recovery for a stale armed transfer is
§6.4, which consumes it.

### 6.4 Drain

Read EP IN repeatedly, discarding everything, until no octets have
arrived for **150 ms**; bound the whole drain at 2 s. Only after the
drain MAY the host send its first command. The reference hosts report
the count (`drained N stale bytes left by a previous session`). The
count is informational: because the device pushes status unprompted, a
non-zero drain is the normal case and says nothing about the previous
session.

### 6.5 First exchange

The reference hosts send `CMD_INFO` and wait for `RSP_INFO` (OMHP
§6.1) with a 2 s timeout as the open's success test. This is prose, not
a requirement: the transport is open once §6.4 completes.

## 7. Timeouts and liveness

### 7.1 Write timeout

A host's bulk OUT timeout MUST exceed the device's worst blocking burst
(§5.4): the reference hosts use **5000 ms**. Both hosts sat under it
(1000 and 2000 ms) and the Python one died out of its 1 Hz heartbeat
with a traceback against a board that was merely decoding.

A write that times out having transferred **zero** octets MAY be
retried. A write that times out having transferred some octets MUST NOT
be retried from the start: the octets already delivered are inside the
device's parser, and repeating them desynchronises it until the next
resync. (pyusb cannot report a partial write count at all, which is why
the Python host does not retry and the C host retries only `done == 0`.)

### 7.2 Read timeout

A bulk IN read timing out is the **normal** case for an event stream
and MUST NOT be treated as an error. The reference hosts poll with
short timeouts (50 ms in the drain loops) and clamp a computed timeout
to at least 1 ms: libusb reads `timeout = 0` as "block forever", and a
deadline that has just expired computes to exactly zero.

### 7.3 Host liveness

The device counts a host as attached while a command has arrived
within 3000 ms (OMHP §3.6); this drives only the LED. A host MAY keep
it lit with `CMD_PING` at any period under 3 s.

## 8. Transport-level faults

| fault | detection | disposition |
|---|---|---|
| device disappears (unplug, reset, re-flash) | `LIBUSB_ERROR_NO_DEVICE` / `EPIPE` on any transfer | the handle is dead; the host MUST re-enumerate from §6.1. A `CMD_RESET` does **not** cause this (OMHP §6.5) |
| interface busy | `EBUSY` at claim | another host holds it; report and exit (§6.2) |
| EP OUT halted | `EPIPE` on write | `clear_halt(EP OUT)` once, then resume; a second `EPIPE` is a device fault |
| EP IN halted | `EPIPE` on read | do NOT clear the halt (§6.3); close and re-enumerate |
| serial string refused | `no langid` / `EPIPE` on the string request | retry per §6.1 |
| stream misaligned | OMHP parser `resyncs` > 0 | OMHP §4.2 recovers within one frame; a rising count with no partial writes (§7.1) indicates a fault below this layer |

No fault at this layer is reported to the device, and the device has
no way to report a transport fault to the host except by the `dropped`
counter it exposes in diagnostics.

## 9. The stdio binding

The device-side reference code runs as a process
(`cport/build/usb_modem_emu`, built by `make -C cport usbemu`) that
reads the OMHP stream on **stdin** and writes it on **stdout**.

```
<stdin>  := <frame>*        host -> device
<stdout> := <frame>*        device -> host
```

Rules: the octets are identical to the bus; the process pushes
`EVT_STATUS` on the same 0.5 s cadence; a host MUST poll stdout with a
timeout (`select`, then `read1`) rather than a blocking read, because a
device with nothing to say is the normal case; there is no drain step
and no halt handling, since a fresh process starts with an empty
stream. The reference host selects this binding with `--emulate CMD` or
`emulate=[argv]`, and `host/test_ofdm_modem.py` exercises both ends of
OMHP over it. The binding does not exercise §3–§8; the USB peripheral
driver is the one part of the system that cannot be tested off-target.

## 10. Assignment considerations

| value | owner | policy |
|---|---|---|
| `idVendor:idProduct` `1209:0001` | pid.codes | **test ID, development only.** A shipped unit MUST carry a PID allocated at https://pid.codes (or a vendor ID of its own); the udev rule and both hosts MUST then be narrowed to it. Two unrelated devices sharing the test ID collide on a user's machine, which is the exact failure identity exists to prevent. |
| `bInterfaceSubClass` `0x4F` | `usb_desc.h` | identifies this device family; unchanged across framing versions |
| `bInterfaceProtocol` | `usb_desc.h` | equals OMHP's `proto_ver` (OMHP §14.1). It is incremented **only** for a framing change an old host cannot parse; appending fields to structures does not change it. A host implements a set of values and refuses the rest (§3.1). |
| `bcdDevice` | `usb_desc.h` | hardware/firmware revision for display; hosts MUST NOT gate behaviour on it -- `RSP_INFO.fw_ver` is the field for that |
| string indices 1–4 | `usb_desc.h` | fixed; a new string takes the next index |
| endpoint addresses `0x01` / `0x81` | `usb_desc.h` | fixed for protocol 1 |

One header, `cport/src/usb_desc.h`, holds every value in this table so
that firmware, udev rule and hosts cannot drift apart; a change is made
there and mirrored in `host/ofdm_modem.py` (`VID`, `PID`, `EP_OUT`,
`EP_IN`) and `host/99-ofdm-modem.rules`.

## 11. Timing constants

| constant | value | side |
|---|---|---|
| bulk packet size | 64 octets | both |
| device OUT read chunk / IN FIFO | 512 / 512 octets | device |
| device staging ring | 4096 octets | device |
| unsolicited status period | 0.5 s | device |
| worst device blocking burst | 2283 ms (measured) | device |
| host write timeout | 5000 ms | host |
| drain quiet / bound | 150 ms / 2 s | host |
| serial-string retry | 10 × 300 ms | host |
| poll read timeout | 50 ms, ≥ 1 ms | host |
| host-attached window | 3000 ms | device |

## 12. Security considerations

The binding is a local bus between a host and a device it owns; it
provides no authentication or confidentiality and none is expected of
it. The udev rule grants the interface to the logged-in user
(`uaccess`) and to `plugdev`; on a shared machine that is the access
control, and an administrator who needs less SHOULD drop `uaccess`.
Because exactly one host may claim the interface, any local process
that can open the device can deny it to every other; that is inherent.

### 12.1 Consequences of not meeting a requirement

Per RFC 2119 §7; each was observed on the two-board stand.

| requirement | what happens without it |
|---|---|
| never `clear_halt(EP IN)` (§6.3) | the endpoint delivers one packet and wedges until re-enumeration; `RSP_INFO` never arrives; `isr_count` climbs into the tens of millions on a level-triggered FIFO-empty interrupt |
| drain before the first command (§6.4) | the parser starts inside a stale frame; recovers by resync, but the first status frames decode as garbage or not at all |
| read continuously (§5.3) | frames dropped whole at the device; `dropped` climbs; a file part or a broadcast group is missing with no error on the host |
| keep partial data on read timeout (§5.2) | a 64-multiple frame (a 128-octet `EVT_LOG`, a 3328-octet file part) is silently lost |
| write timeout > 2283 ms (§7.1) | the host raises on a healthy board mid-decode and its session dies |
| never repeat a partial write (§7.1) | the device parser desynchronises; the frame is lost and possibly one after it |
| retry the serial string and clear the langid cache (§6.1) | one open in twelve fails with `no langid`, read by the operator as a permission problem |
| select by serial with two boards (§6.1) | the wrong board is opened after a re-plug; transmit and receive swap sides silently |
| check `bInterfaceProtocol` (§3.1) | a host parses a future incompatible framing as garbage from the first octet |

## 13. References

Normative:

- [RFC 2119], [RFC 8174] -- BCP 14 requirement words.
- [USB 2.0] Universal Serial Bus Specification, revision 2.0, chapters
  5 (bulk transfers), 8 (protocol layer) and 9 (device framework).
- [OMHP] [modem-protocol.md](modem-protocol.md) -- the message layer
  this binding carries.
- Code: `cport/src/usb_desc.h`, `cport/usb/usb_descriptors.c`,
  `cport/usb/usb_radio_main.c`, `host/ofdm_modem.py`,
  `demoapp/usb_host.c`, `host/99-ofdm-modem.rules`.

Informative:

- [RFC 5444] -- the model for the syntax notation, fault table and
  assignment section.
- `cport/usb/README.md` -- the bring-up narrative, including the stall
  investigation summarised in Appendix B.
- [pid.codes] https://pid.codes -- vendor ID `0x1209` allocation policy.

---

## Appendix A. Descriptor octets

Device descriptor, as enumerated:

```
12 01  00 02  ff 00 00  40  09 12  01 00  00 01  01 02 03  01
│  │   │      │         │   │      │      │      │         └ bNumConfigurations
│  │   │      │         │   │      │      │      └ iManufacturer iProduct iSerial
│  │   │      │         │   │      │      └ bcdDevice 1.00
│  │   │      │         │   │      └ idProduct 0x0001
│  │   │      │         │   └ idVendor 0x1209
│  │   │      │         └ bMaxPacketSize0
│  │   │      └ class/subclass/protocol
│  │   └ bcdUSB 2.00
│  └ DEVICE
└ bLength
```

Configuration descriptor set (32 octets):

```
09 02  20 00  01  01  00  80  32          configuration: 32 total, 1 itf, bus-powered, 100 mA
09 04  00  00  02  ff 4f 01  04           interface 0 alt 0, 2 EPs, FF/4F/01, string 4
07 05  01  02  40 00  00                  EP 0x01 OUT bulk 64
07 05  81  02  40 00  00                  EP 0x81 IN  bulk 64
```

## Appendix B. The endpoint stall, as measured

Informative; the full account with the five disproven hypotheses is in
`cport/usb/README.md`, "What the stall was".

For most of a day the station image enumerated, answered nothing, and
went quiet after one packet. Instrumentation read, at the stall:
staging queue empty, 150 of 512 FIFO octets free, 399 octets written to
the FIFO -- of which 362 = 29 (`RSP_INFO`) + 9 × 37 (status frames)
were sitting **inside** the stack's FIFO unsent, and 399 − 362 = 37 =
exactly one status frame had reached the wire. The device was producing
correctly; the endpoint was wedged after its first transfer.

Mechanism, confirmed from the stack's source:

1. The device pushes status unprompted, so at open EP IN is already
   armed in hardware with a frame waiting for an IN token.
2. The host called `clear_halt(EP IN)` on open -- a common recovery
   idiom.
3. `usbd_edpt_clear_stall` clears its software BUSY flag
   unconditionally ("long-standing behavior", its own comment) while
   the dwc2 `dcd_edpt_clear_stall` only clears the STALL bit and resets
   the PID: nothing disarms the hardware.
4. The next flush sees "not busy" and re-arms the endpoint on top of
   the live transfer -- `DIEPTSIZ` rewritten with `EPENA` set, which the
   core does not define. One packet escapes; the completion no longer
   matches what the stack started; BUSY never clears.
5. Side signature: `isr_count` climbs to tens of millions, the
   TX-FIFO-empty interrupt being level-triggered.

Tested by prediction both ways: gating status until the host had spoken
made the *first* open work and every later one wedge, exactly as the
mechanism says. The device is deliberately not made robust to
`clear_halt(EP IN)` -- that is the stack's clear-stall path, its
business, not this project's -- so the requirement lives in §6.3, on
every host.
