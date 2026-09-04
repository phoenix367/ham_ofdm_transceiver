# OFDM Modem Host Protocol (OMHP)

Specification, version 1

**Status of this document.** This is the normative description of the
protocol a host program speaks to an OFDM modem board over USB. The
reference implementation is `cport/src/usb_proto.[ch]` (codec) and
`cport/src/usb_modem.c` (binding), with the broadcast command bound in
`cport/usb/usb_radio_main.c`; the reference host is `host/ofdm_modem.py`.
Where this document and the code disagree, the code is a bug in one of
them and the tests in `cport/tests/test_usb.c` and `host/test_modem.py`
decide which. The narrative companion is [usb-protocol.md](usb-protocol.md);
the air-side machinery the commands drive is in [link.md](link.md) and
[phy.md](phy.md).

## Table of contents

1. Introduction (informative)
2. Conventions and notation (normative)
3. Transport (normative)
4. Framing (normative)
5. Message type registry (normative)
6. Host-to-device commands (normative)
7. Device-to-host responses and events (normative)
8. Fixed-layout structures (normative)
9. Configuration registry (normative)
10. Broadcast (normative; §10.3 and §10.6 informative)
11. Application envelopes (informative -- host conventions)
12. The over-the-air capability record, as surfaced (informative)
13. Diagnostic stream (normative)
14. Versioning and compatibility (normative)
15. Assignment considerations (normative)
16. Timing constants (informative)
17. Security considerations (informative)
18. References

Appendix A. Byte-offset tables
Appendix B. Registries
Appendix C. Worked example (informative)

---

## 1. Introduction

### 1.1 Scope

The board *is* the station: PHY, FEC, link layer, ARQ, rate ladder,
carrier sense and broadcast all execute in firmware. What crosses USB is
**messages and control**, not audio -- roughly 27 bytes per air frame at
a few frames per second. The link is idle by USB standards; its job is
identity and framing, not throughput.

A host program therefore implements a thin terminal that (a) submits
messages and receives delivered ones, (b) paces itself against a status
stream the board pushes unprompted, (c) configures the station, (d) feeds
and drains broadcasts, and (e) keeps itself declared alive by traffic.

### 1.2 Layering

```
    application envelopes  (FILE:, voice tokens)      -- §11
    ---------------------------------------------
    OMHP commands / events                             -- §5-§10
    ---------------------------------------------
    OMHP framing  A5 5A type len payload              -- §4
    ---------------------------------------------
    USB bulk endpoints, vendor class                   -- §3
```

The over-the-air protocol (link control word, capability handshake,
broadcast SYNC descriptor) is a separate layer beneath the station and is
NOT described here except where its state is surfaced to the host (§10,
§12).

### 1.3 Applicability statement

This specification applies to any program that opens the modem's USB
interface, and to the framed stream the `usb_modem_emu` bench speaks on
stdio, which is the same stream without the bus. It assumes a reliable,
ordered, lossless byte stream in each direction (USB bulk provides this;
so does a pipe) and does not itself recover from loss -- a resynchronising
parser (§4.2) tolerates a *truncated* transfer, not a missing one. It is
a point-to-point host interface: there is one host per board and no
addressing in the frame. It is not a transport for audio samples; the
byte rate it carries is a few hundred bytes per second, and a host that
wants signal samples must obtain them elsewhere.

## 2. Conventions and notation

The key words "MUST", "MUST NOT", "REQUIRED", "SHALL", "SHALL NOT",
"SHOULD", "SHOULD NOT", "RECOMMENDED", "NOT RECOMMENDED", "MAY", and
"OPTIONAL" in this document are to be interpreted as described in BCP 14
[RFC 2119] [RFC 8174] when, and only when, they appear in all capitals,
as shown here.

Following RFC 2119 §6, these imperatives are used only where a behaviour
is required for the two ends to interoperate or where departing from it
has been measured to cause harm (lost frames, a wedged endpoint, a
truncated transmission). Everything else -- how the reference host
happens to do something -- is stated in plain prose and binds nobody.
§17 lists what goes wrong when a requirement is not met.

### 2.1 Terminology

- "The board" and "the device" are the modem; "the host" is the program
  across USB. "The peer" is the *other* station reached over the air.
- A **frame** is one unit of the byte stream (§4); its **payload** is
  the bytes after the 5-byte header. A **structure** is a fixed-layout
  payload (§8). An **element** is any field or composite of fields named
  in the syntax (§2.2).
- Byte offsets in tables are relative to the start of the **payload**,
  not the frame.

### 2.2 Data types and syntax notation

Field types: `u8`, `u16le`, `u32le`, `i8`, `i16le`, `i32le` are
unsigned/signed integers of the given width; all multi-byte fields are
**little-endian** (least significant octet first). `Qn` is fixed-point
with `n` fractional bits (`snr_q8` = dB × 256). Bit numbering within an
octet is **LSB = bit 0**, MSB = bit 7. (Note the two differences from
the IETF convention of RFC 5444: byte order and bit numbering are both
reversed, because the boards and the reference host are little-endian
and the code names bits by shift count.)

Elements are written `<element>`. An element is either a field of the
stated width or a composite written as a concatenation of elements.
The syntax uses regular-expression operators: `<a> <b>` concatenation,
`(...)` grouping, `?` zero or one, `*` zero or more, `+` one or more,
`|` alternation. Every element has a length that is either fixed or
determined by an earlier field, so any conforming frame can be parsed in
one forward pass with no lookahead (§4.4).

A value written `name` without brackets is a variable computed from
fields, not a field itself (e.g. `want := 5 + <len>`).

### 2.3 Bit diagrams

Where a byte carries flags, it is drawn with bit 7 on the left:

```
   7   6   5   4   3   2   1   0
 +---+---+---+---+---+---+---+---+
 | a | b | c |  rsv  |   value   |
 +---+---+---+---+---+---+---+---+
```

**Reserved bits and fields** (`rsv`, "reserved", "unassigned") MUST be
cleared (0) on transmission and MUST be ignored on reception. This is
what lets a later revision assign them without a version change (§14).

### 2.4 Normative and informative content

Sections are marked in the table of contents. Informative sections
describe how the reference implementations behave or what the protocol
is used for; they contain no requirements, and a conforming
implementation may differ from them. Measured figures quoted beside a
requirement are its evidence, not part of the requirement.

## 3. Transport

### 3.1 Device identity

The modem enumerates as its own USB device class, not as a serial bridge,
so a host opens it **by identity** rather than by port number.

| property | value |
|---|---|
| VID:PID | `1209:0001` (pid.codes test ID -- development only) |
| interface class / subclass / protocol | `0xFF` vendor / `0x4F` (`'O'`) / `0x01` |
| endpoints | `0x01` bulk OUT, `0x81` bulk IN, 64 bytes |
| serial string | 24 hex characters: the STM32 96-bit unique ID |

Two boards on one machine are always distinguishable by serial. The udev
rule `host/99-ofdm-modem.rules` provides a stable
`/dev/ofdm-modem-<serial>` and non-root access.

### 3.2 Claiming

A host MUST hold exactly one open handle per board. The reference host
refuses to open a board another process has claimed
("the modem is already claimed by another program").

### 3.3 Opening: drain, never reset

The device pushes `EVT_STATUS` frames at 2 Hz whether or not anyone is
listening (§7.3), so its IN endpoint is armed with a frame at essentially
any moment a host opens it.

On open a host MUST **drain** EP_IN -- read and discard until the device
has been quiet for at least 150 ms (bounded, 2 s in the reference host).
The reference host reports the number of discarded bytes. A host MUST
NOT issue
`clear_halt` on EP_IN as an opening step: it desynchronises the device
stack's software state from the hardware transfer and wedges the endpoint
after exactly one packet (measured; `cport/usb/README.md`, "What the
stall was"). A host MAY clear halts on both endpoints *before* the first
transfer to recover from a previous process killed mid-transfer, which
leaves an endpoint halted with writes silently accepted and never
delivered.

### 3.4 Serial string acquisition

A control transfer issued while the device is pushing bulk data can
transiently fail, and pyusb caches a failed language-ID fetch, after
which every later string read on that handle fails instantly. A host
MUST tolerate a transient failure of the serial-string read by
retrying it for at least the device's worst blocking receive burst
(§16; the reference host retries 10 × 300 ms). Implementation note: a
retry through pyusb is a no-op unless the handle's cached language IDs
(`dev._langids`) are cleared between attempts.

### 3.5 Host write timeout

The device may block for up to ~2.3 s inside a single receive commit
(§16) during which it is not reading its OUT endpoint. A host's write
timeout MUST exceed that; the reference hosts use 5000 ms. A host MUST
NOT retry a timed-out write that moved any bytes -- repeating a partial
frame desynchronises the device parser -- and MAY retry one that moved
none.

### 3.6 Host liveness

The device considers a host **attached** if any command frame has
arrived within `HOST_ALIVE_MS` = 3000 ms. Attachment drives only
indication (the LED's "host attached" state); it gates no protocol
behaviour. A host that wants the indication MAY send `CMD_PING` at any
period under 3 s; the reference hosts use 1 s. Merely holding the USB
handle open does not count: closing a program unmounts nothing.

## 4. Framing

### 4.1 Frame format

Both directions carry an identical, self-delimiting byte stream:

```
  0      1      2        3        4        5
  +------+------+--------+--------+--------+-----------------+
  | 0xA5 | 0x5A | type   | len_lo | len_hi | payload[len]    |
  +------+------+--------+--------+--------+-----------------+
  UP_SYNC0 UP_SYNC1  u8      u16le (len)      len bytes
```

- `UP_HDR_LEN` = 5.
- `len` MUST be ≤ `UP_MAX_PAYLOAD` = 3336, sized so one station message
  (`ST_MSG_MAX` = 3328 on the boards) plus its QoS byte crosses in one
  frame. A frame that would exceed `UP_MAX_FRAME` = 3341 MUST be refused
  by the encoder, never truncated.
- There is **no checksum**. USB CRCs and retries every packet; a second
  integrity layer would answer a question the bus already answered.

Two sync bytes rather than one make resynchronisation after a truncated
transfer cheap and, in practice, unambiguous.

### 4.2 Parser behaviour

A receiver MUST implement a streaming parser: bytes arrive in whatever
chunks the bus delivers and frames come out whole. The reference parser:

1. If the first buffered byte is not `0xA5`, or the second is not
   `0x5A`, drop **one** byte, count a `resync`, and re-examine.
2. With a complete header, if `len` > `UP_MAX_PAYLOAD`, treat the header
   as false: drop one byte, count a resync.
3. Deliver each complete frame; carry any remainder forward.

`resyncs` is a **fault counter**, not a statistic: it is 0 on a healthy
link, and a rising count means the stream lost alignment. A receiver MUST
tolerate frames coalesced into one read and frames split across reads.

### 4.3 Unknown types

A receiver MUST ignore a frame whose `type` it does not recognise, after
consuming it whole. This is what allows the type registry (§5) to grow.

### 4.4 Formal syntax

The whole protocol, in the notation of §2.2. Widths in octets follow
each field in brackets; a bare name with no width is a composite defined
below it.

```
<stream>      := <frame>*

<frame>       := <sync0>[1] <sync1>[1] <type>[1] <len>[2] <payload>[len]
                 <sync0> = 0xA5, <sync1> = 0x5A, 0 <= len <= 3336

<payload>     := <cmd-info> | <cmd-submit> | <cmd-config> | <cmd-ping>
               | <cmd-reset> | <cmd-bcast>
               | <rsp-info> | <evt-message> | <evt-status> | <evt-diag>
               | <rsp-pong> | <evt-log> | <evt-audio> | <evt-bcast>
                 (selected by <type>; an unknown <type> is <opaque>)

<opaque>      := <octet>*

<cmd-info>    :=                                        (len = 0)
<cmd-submit>  := <qos>[1] <data>[len-1]                 (len >= 2)
<cmd-config>  := ( <key>[1] <value>[4] <octet>* )?      (len = 0 query;
                                                         len >= 5 set)
<cmd-ping>    := <token>[4] <octet>*                    (len >= 4)
<cmd-reset>   :=                                        (len = 0)
<cmd-bcast>   := <ptype>[1] <rung>[1] <data>[len-2]     (2 < len <= 8194)

<rsp-info>    := <info-struct>[26]
<evt-message> := <qos>[1] <data>[len-1]
<evt-status>  := <status-struct>[44]
<evt-diag>    := <ev>[1] <a>[4] <b>[4] <c>[4] <d>[4] <t_ms>[4]
<rsp-pong>    := <token>[4]
<evt-log>     := <utf8-octet>*                          (len <= 256)
<evt-audio>   := <opaque>                               (reserved)
<evt-bcast>   := <bc-start> | <bc-data> | <bc-eos>
<bc-start>    := <bflags>[1]                            (bflags & 0x80)
<bc-data>     := <bflags>[1] <data>[len-1]              (bflags = 0)
<bc-eos>      := <bflags>[1] <frames_ok>[2] <frames_lost>[2] <snr_q8>[2]
                                                        (bflags & 0x40)
```

Structure layouts (`<info-struct>`, `<status-struct>`) are in §8; a
receiver takes the **prefix it knows** of each (§14.2), so the widths
above are what a current encoder emits, not what a decoder requires.
Where a command's syntax admits trailing octets (`<octet>*`), a device
MUST ignore them; this is the same extension room as reserved bits.

### 4.5 Malformed elements

An element is **malformed** if it cannot be parsed according to its
syntax, including when fewer octets are available than its width
requires. Disposition is by level:

| level | condition | disposition |
|---|---|---|
| stream | byte at frame start is not `0xA5 0x5A` | drop one byte, count `resyncs`, re-examine (§4.2) |
| frame header | `len` > 3336 | header is false: drop one byte, count `resyncs` |
| frame | `type` unknown | consume whole, ignore (§4.3) |
| command | payload shorter than its minimum (`<cmd-submit>` < 2, `<cmd-config>` 1..4, `<cmd-ping>` < 4, `<cmd-bcast>` < 3) | consume whole, ignore silently -- no error frame exists |
| command | `<cmd-bcast>` data > 8192 | ignore silently |
| structure | payload shorter than its minimum decodable length (§8) | decoder rejects it; the host treats the frame as absent |
| structure | payload longer than the decoder knows | decode the known prefix, ignore the rest (§14.2) |

There is no negative-acknowledgement frame: a malformed command
produces **no** response, and a host that needs to know whether a
command was taken uses the state it changes (`EVT_STATUS`, the config
query, `EVT_LOG`). The one exception is a refused `CMD_SUBMIT` (§6.2),
which is well-formed but declined, and is logged.

## 5. Message type registry

| type | direction | name | payload |
|---|---|---|---|
| `0x01` | H→D | `CMD_INFO` | none |
| `0x02` | H→D | `CMD_SUBMIT` | `qos:u8, data[]` |
| `0x03` | H→D | `CMD_CONFIG` | `key:u8, value:i32le` or empty (query) |
| `0x04` | H→D | `CMD_PING` | `token:u32le` |
| `0x05` | H→D | `CMD_RESET` | none |
| `0x06` | H→D | `CMD_BCAST` | `ptype:u8, rung:u8, data[]` |
| `0x81` | D→H | `RSP_INFO` | `up_info_t` (§8.1) |
| `0x82` | D→H | `EVT_MESSAGE` | `qos:u8, data[]` |
| `0x83` | D→H | `EVT_STATUS` | `up_status_t` (§8.2) |
| `0x84` | D→H | `EVT_DIAG` | `ev:u8, a,b,c,d:i32le, t_ms:u32le` |
| `0x85` | D→H | `RSP_PONG` | `token:u32le` |
| `0x86` | D→H | `EVT_LOG` | UTF-8 text, no terminator |
| `0x87` | D→H | `EVT_AUDIO` | `int16le` samples (reserved, §7.7) |
| `0x88` | D→H | `EVT_BCAST` | `flags:u8, ...` (§10.4) |

Types `0x07`–`0x80` and `0x89`–`0xFF` are unassigned. Host→device types
occupy the low half; device→host types set bit 7.

## 6. Host-to-device commands

Every command increments the device's `host_cmds` counter, which is the
liveness signal of §3.6.

### 6.1 `CMD_INFO` (0x01)

Payload: none. The device MUST answer with `RSP_INFO` (§8.1). The
answer is the only source of `msg_max`, from which a host MUST size its
message buffers (§8.1); the reference hosts send it at attach.

### 6.2 `CMD_SUBMIT` (0x02)

```
  0        1
  +--------+-----------------------------------+
  | qos:u8 | data[len-1]                        |
  +--------+-----------------------------------+
```

Queue one message for transmission over the ARQ link.

- `len` MUST be ≥ 2 (at least one data byte); shorter frames are
  ignored.
- `qos`: 0 = control, 1 = interactive, 2 = bulk. A value > 2 is treated
  as 2.
- `data` length MUST be ≤ `msg_max` from `RSP_INFO`.

The device queues the message on the station's QoS queue. On success
nothing is sent. If the queue or the message store is full the device
MUST emit `EVT_LOG` with the text `submit refused: queue or store full`.
A host queues on the board's behalf and therefore MUST watch for this
refusal and MUST pace bulk submissions against the queue depths in
`EVT_STATUS` (`q_ctl`, `q_inter`, `q_bulk`). Pacing is self-correcting
by construction: the board reports true depths, so a refused submit shows
up as room again.

A submitted message may also be refused by the station when its pooled
store is full even though queue positions remain (§12 of
`cport/src/station.h`); the same log text is used.

### 6.3 `CMD_CONFIG` (0x03)

Two forms.

**Query** -- payload empty (`len` = 0). The device MUST answer with one
`EVT_LOG` line of the form

```
config: rung_ceiling N  win_max N  burst_window N  burst_stream N  anchor N  diag_stream N  freq_trim_hz N
```

The settings live on the board and survive a host reattach, so a host
cache of them lies after one; the reference consoles query instead of
remembering. (`codecs`, key 9, is not included in the query line.)

**Set** -- payload `key:u8, value:i32le` (`len` ≥ 5):

```
  0        1
  +--------+----------------+
  | key:u8 | value:i32le    |
  +--------+----------------+
```

Keys are in §9. An unknown key MUST be ignored. Values are clamped as
specified per key. Setting `rung_ceiling` or `win_max` while the peer's
capability record is held MUST schedule a refreshed record to be pushed
to the peer (`caps_reply_due`), since both are declared over the air.
No acknowledgement is sent for a set.

### 6.4 `CMD_PING` (0x04)

Payload `token:u32le`. The device MUST answer with `RSP_PONG` carrying
the same four bytes. A frame shorter than 4 bytes is ignored (no pong).
This is both the keepalive (§3.6) and the round-trip probe.

### 6.5 `CMD_RESET` (0x05)

Payload: none. The device re-initialises the station's delivered-message
log and clears the host's delivery cursor **while keeping the USB link
up**. The host's handle stays valid -- the difference between a modem
that recovers and one that re-enumerates.

### 6.6 `CMD_BCAST` (0x06)

```
  0          1         2
  +----------+---------+------------------------------+
  | ptype:u8 | rung:u8 | data[len-2]                   |
  +----------+---------+------------------------------+
```

Start, or continue, a non-ARQ broadcast. Full semantics in §10.

## 7. Device-to-host responses and events

All device→host frames are staged in a 4096-byte ring (`UM_TXQ`) and
drained as the host reads. A frame that does not fit the remaining ring
space MUST be **dropped whole** and counted (`dropped`), never written
partially -- a partial frame would desynchronise the stream, which costs
far more than the frame. `EVT_DIAG` is shed earlier than everything else
(§13).

### 7.1 `RSP_INFO` (0x81)

Answer to `CMD_INFO`. Layout §8.1.

### 7.2 `EVT_MESSAGE` (0x82)

```
  0        1
  +--------+-----------------------------------+
  | qos:u8 | data[len-1]                        |
  +--------+-----------------------------------+
```

A message delivered by the ARQ link, complete. `qos` is always 2 on
delivery (the station reports delivered messages as bulk). `data` is at
most `msg_max` bytes. The device drains its delivered-message log once
every entry has been staged, so delivery cannot stall at the log's
capacity.

### 7.3 `EVT_STATUS` (0x83)

Pushed **every 0.5 s** whether or not a host is attached, and never on
request. Layout §8.2. This is the host's pacing signal (queue depths,
`bc_free`) and its view of link state.

### 7.4 `EVT_DIAG` (0x84)

Off by default. Layout §8.3, semantics §13.

### 7.5 `RSP_PONG` (0x85)

The four token bytes of the `CMD_PING` being answered.

### 7.6 `EVT_LOG` (0x86)

UTF-8 text, no terminator, at most 256 bytes. Carries human-readable
board decisions and refusals. Several are protocol-significant and are
listed where they arise (§6.2, §6.3, §10); a host that acts on one MUST
match the text exactly as given there. The reference consoles display
every line verbatim.

### 7.7 `EVT_AUDIO` (0x87)

**Reserved.** Registered for a receive-path sample tap (`int16le`
samples, decimated by configuration key 5) that no firmware emits; no
shipped device sends it and hosts MUST ignore it. The reference host
names it for display only.

### 7.8 `EVT_BCAST` (0x88)

A received broadcast, streamed. Layout and semantics §10.4.

## 8. Fixed-layout structures

All structures are little-endian and fixed-size. Each has a **minimum
decodable length** below which a decoder MUST reject the payload, and
fields beyond that minimum are **present-by-length** (§14).

### 8.1 `up_info_t` -- 26 bytes

| offset | field | type | meaning |
|---|---|---|---|
| 0 | `proto_ver` | u8 | this protocol's version; **1** |
| 1 | `n_modes` | u8 | link modes the PHY offers; 3 |
| 2 | `fw_ver` | u16le | `(major << 8) \| minor` |
| 4 | `uid[12]` | u8×12 | STM32 96-bit unique ID, as read |
| 16 | `caps` | u32le | `UP_CAP_*` bits (Appendix B) |
| 20 | `sample_rate` | u32le | 12000 |
| 24 | `msg_max` | u16le | largest `CMD_SUBMIT` data length |

Minimum decodable length: **24**. `msg_max` absent (older firmware)
⇒ a host MUST assume 256. A host's own message buffer MUST be at least
`msg_max`: the reference host once capped at the protocol's original
1024 while boards grew to 3328, and its parser then resynchronised past
every delivered file part as garbage.

### 8.2 `up_status_t` -- 44 bytes

| offset | field | type | meaning |
|---|---|---|---|
| 0 | `rung` | i32le | rung of the last frame actually **transmitted**; −1 = none yet. May be hours stale. |
| 4 | `snr_q8` | i32le | filtered SNR of the peer's frames, dB × 256; **−25344** (−99.0 dB) = no measurement within the last 60 s |
| 8 | `tx_frames` | u32le | counters since boot |
| 12 | `rx_frames` | u32le | |
| 16 | `timeouts` | u32le | |
| 20 | `retransmissions` | u32le | |
| 24 | `q_ctl` | u16le | queue depths (messages) |
| 26 | `q_inter` | u16le | |
| 28 | `q_bulk` | u16le | |
| 30 | `busy` | u8 | carrier sense (reserved; currently 0) |
| 31 | `pending` | u8 | 1 = a transmitted frame awaits acknowledgement |
| 32 | `peer_state` | u8 | 0 unknown, 1 legacy (never answered), 2 record held, 3 held and ours confirmed |
| 33 | `peer_caps` | u8 | peer's `CAP_*` flags (§12) |
| 34 | `peer_msg_max` | u16le | peer's largest message |
| 36 | `peer_win_max` | u8 | peer's streamed-window ceiling |
| 37 | `peer_max_rung1` | u8 | peer's rung ceiling **+ 1**; 0 = unspecified |
| 38 | `bc_free` | u16le | free bytes in the broadcast source buffer (§10.2) |
| 40 | `temp_q8` | i16le | die temperature, °C × 256; `UP_TEMP_NONE` = −32768 when no sensor |
| 42 | `rung_now` | i8 | rung the **next** frame would go out at; −1 = none yet; `UP_RUNG_ABSENT` = −128 when the field is absent |
| 43 | `peer_codecs` | u8 | `CODEC_*` bits the peer declared it can decode; 0 = never said |

Minimum decodable length: **32**. Decoding rules for shorter frames
(older firmware):

- `len` < 40: the peer block, `bc_free` and everything after it read as
  0 / `UP_TEMP_NONE` / `UP_RUNG_ABSENT`.
- `len` < 42: `temp_q8` = `UP_TEMP_NONE`. A decoder MUST NOT default
  the temperature to 0, which is a real temperature.
- `len` < 43: `rung_now` = `UP_RUNG_ABSENT`.
- `len` < 44: `peer_codecs` = 0.

`snr_q8` = −25344 is a sentinel (`LC_SNR_NONE`), emitted whenever no
SNR sample younger than 60 s exists -- at boot and whenever the peer has
been silent. A host MUST NOT plot or average it as a reading.

`rung` and `rung_now` are both carried on purpose: an overnight-idle
station reported `rung 12` (the last transmission) and then sent at rung
0 (what the decayed ladder would actually use). A host MUST NOT read
`rung` as the rung the next transmission will take; that is `rung_now`.

`peer_codecs` = 0 means the peer **never said** -- a peer predating the
field, or one whose host never configured `codecs` -- and MUST NOT be
read as "supports none".

Every buffer that holds a status payload MUST be sized from
`UP_STATUS_LEN`, never from a literal (§14.3).

### 8.3 `EVT_DIAG` payload -- 21 bytes

| offset | field | type |
|---|---|---|
| 0 | `ev` | u8 -- `ST_EV_*` (Appendix B) |
| 1 | `a` | i32le |
| 5 | `b` | i32le |
| 9 | `c` | i32le |
| 13 | `d` | i32le |
| 17 | `t_ms` | u32le -- device time, milliseconds |

The meaning of `a..d` depends on `ev` (Appendix B).

## 9. Configuration registry

`CMD_CONFIG` keys. Values are `i32le`.

| key | name | value | clamp / notes |
|---|---|---|---|
| 1 | `rung_ceiling` | fastest rung transmitted **or requested** | 0..12; declared to the peer; refreshes the caps record when set |
| 2 | `burst_window` | operator ceiling on the selective-repeat window | `burst_window` ≥ 2 is what engages burst ARQ at all |
| 3 | `burst_stream` | 0/1 | advertises `CAP_STREAM` to the peer |
| 4 | `freq_trim_mhz` | LO trim, **millihertz**, signed | applied through the AFC |
| 5 | `audio_tap` | 0 = off, else decimation factor | **reserved**: registered, accepted, ignored by every shipped firmware (§7.7) |
| 6 | `anchor` | 0/1 | AFC frequency reference |
| 7 | `diag_stream` | 0/1 | default **0**; enables `EVT_DIAG` |
| 8 | `win_max` | streamed-window ceiling accepted **and** sent | 1..16; declared to the peer; refreshes the caps record when set |
| 9 | `codecs` | `CODEC_*` bitmap the **host** can decode | low 8 bits; declared to the peer (§12); default 0 = never said |

Keys 0 and 10–255 are unassigned and MUST be ignored by the device.

`codecs` exists because the board has no codec of its own -- it moves
bytes -- so the only party that can answer "what voice can you play?" is
the program across USB. A host that decodes a voice codec SHOULD set
the bit before the peer's capability record is exchanged (in practice,
at attach); a bit set later reaches the peer only with the next
refreshed record, and until then the peer reads "never said" (§8.2).

## 10. Broadcast

A broadcast is **fire-and-forget on the air but a conversation with the
board**: the host feeds a source, the board keys it in groups of frames,
and the receiving board streams the payload up as it decodes. Nothing is
acknowledged and nothing is retransmitted; the receiver's own statistics
say what arrived.

### 10.1 `CMD_BCAST` semantics

```
   7    6    5    4    3   2   1   0
 +----+----+----+----+---+---+---+---+
 |MORE|CONT| RT |rsv |     ptype     |     <ptype> octet
 +----+----+----+----+---+---+---+---+

  MORE  -- further chunks will follow
  CONT  -- continuation of the broadcast in flight
  RT    -- real time: key what is queued, do not wait for a full group
  ptype -- payload type (Appendix B.4)
  rung:u8   0..12 pins the rung; any other value (conventionally 0xFF) = board chooses
  data[]    1..8192 bytes (BC_TX_CAP)
```

The four MORE/CONT combinations and what each means for the stream:

| MORE | CONT | meaning | source buffer |
|---|---|---|---|
| 0 | 0 | one-shot broadcast: `data` is the whole source | replaced |
| 1 | 0 | first chunk of a new stream | replaced |
| 1 | 1 | middle chunk | appended |
| 0 | 1 | last chunk: completes the stream, EOS follows its last frame | appended |

Every combination is valid. RT is orthogonal and is read from **every**
chunk -- the board follows the most recent one -- so a host MUST set it
identically on every chunk of one stream. Bit 4 is reserved (§2.3).

`len` < 2, or `data` empty or longer than 8192, MUST be rejected
silently.

**One-shot** (MORE = 0, CONT = 0): the classic broadcast. The whole
payload is the source; the board keys it in groups and closes the stream
with EOS on the last frame.

**Chunked** (MORE and/or CONT set): a source larger than the 8 kB buffer
streams from the host.

- The first chunk has CONT = 0 and MORE = 1: it **starts** a stream.
- Later chunks have CONT = 1; the last has MORE = 0, which is what
  completes the stream -- EOS then lands on its real last frame.
- A CONT chunk with no broadcast in flight MUST be ignored, so that a
  stray continuation cannot seed a new stream with garbage.
- A CONT chunk that does not fit even after the sent prefix is compacted
  is a **host pacing failure**: the board MUST drop the **entire**
  queued broadcast rather than corrupt its byte stream, and emit
  `EVT_LOG`: `broadcast: chunk overran the source buffer -- dropped (host pacing bug)`.
- A **non-continuation** command while a stream is unfinished on the
  air **supersedes** it: the board closes the old stream with a
  zero-length EOS frame, then starts the new one at sequence 0. A host
  MUST therefore never send a non-CONT chunk mid-stream -- this is how
  a stray chunk turns into a truncated transmission.

**Pacing.** A host MUST pace chunks against `bc_free` in `EVT_STATUS`
(§8.2), submitting only what fits. The reference voice host waits until
`bc_free` ≥ 2 × the chunk size.

**Group keying.** While chunks are still arriving the board keys only
**full** groups (4 frames × 24 bytes = 96 bytes of payload), so a
starved tail group never goes out without EOS. With RT set the board
instead keys whatever is queued, sizing the group to a power of two
matching the data -- the group descriptor carries log₂(group) and the
count MUST match the frames really keyed. RT trades one preamble per
fewer frames for latency; it is opt-in and off for every other
broadcast. RT also suppresses the mid-stream block/stats exchange
(§10.3): speech cannot afford the 8 s hold it costs.

**Rung selection.** A pinned rung (0..12) is honoured exactly, including
a slow one -- EXTREME (rung 0) is the only mode an idle receiver is
guaranteed to keep active, so a beacon for strangers belongs there. The
default is `station_tx_rung()`: the controller's rung passed through the
operator's ceiling **and the peer's declared maximum** (§12). If that
default is a **decayed memory** (the peer has been silent past the
ladder's staleness decay, one rung per 90 s), the broadcast is **held**
while control probes bring the ladder up, released at NORMAL, dropped
with a log line after 180 s. The board logs `broadcast: held -- link is
idle, probing to bring it up (release at NORMAL, give up after 180 s)`.
A host for which a hold is unacceptable (push-to-talk) SHOULD NOT start
a broadcast on a decayed link; §10.6 describes how the reference host
avoids it. A new `CMD_BCAST` clears a pending hold.

On every start the board emits `EVT_LOG` with what it chose: rung, group
geometry, and total air-time estimate (`broadcast: N B at rung R (MODE),
G frame(s) per group, T s each, ~S s total`), and `broadcast: last group
keying now -- N frame(s) sent` at the end.

### 10.2 `bc_free`

Free bytes in the board's 8192-byte broadcast source buffer, published
in every `EVT_STATUS`. The board compacts the sent prefix before
refusing a chunk, so `bc_free` reflects the space a chunk can actually
use.

### 10.3 Block statistics (air side, surfaced)

When the sender holds the peer's capability record with `CAP_BC_STATS`
and RT is **not** set, the stream is cut into ~45 s blocks. At each
block end the sender keys a dataless end-of-block marker, holds its
transmitter for a reply window (~9.4 s), and the receiver answers with
frames-ok / frames-lost / SNR / desired rung. The sender may re-rung the
next block. This is entirely board-to-board; the host sees only
`EVT_LOG` lines (`stats keyed: asks rung R` on the receiver, `broadcast
stats from peer: N ok, M lost, asks rung R [-- re-rung]` or `broadcast:
no stats reply -- stepping down to rung R` on the sender). An older
receiver walks past the marker and appends zero bytes.

### 10.4 `EVT_BCAST` -- streamed reception

```
   7     6    5    4    3   2   1   0
 +-----+----+----+----+---+---+---+---+
 |START|EOS |   rsv   |     ptype     |     <bflags> octet
 +-----+----+----+----+---+---+---+---+
```

| START | EOS | payload after `bflags` | ptype nibble |
|---|---|---|---|
| 1 | 0 | none | the stream's payload type |
| 0 | 0 | reassembled data bytes | 0 |
| 0 | 1 | the 6-byte stats record | the stream's payload type |
| 1 | 1 | -- | MUST NOT be emitted; a receiver MUST ignore the frame |

Bits 5 and 4 are reserved (§2.3).

**Start** (`flags & 0x80`): payload is the single flags byte; the stream
sink SHOULD be reset. Exactly **one** start event per stream, not per
group.

**Data** (`flags` = 0): bytes 1.. are payload bytes in decoded order.
Gaps are never repaired; a lost group is simply absent.

**End of stream** (`flags & 0x40`), 7 bytes:

| offset | field | type |
|---|---|---|
| 0 | `flags` | u8 = `0x40 \| ptype` |
| 1 | `frames_ok` | u16le |
| 3 | `frames_lost` | u16le |
| 5 | `snr_q8` | i16le |

`frames_lost` is authoritative: it counts sequence gaps **including head
loss** (a stream starts at sequence 0, so a first lock at sequence N
means N frames are gone -- a case the gap arithmetic alone cannot see).
Byte counts say what arrived; the stats say what the air cost.

A stream that dies without EOS is closed by the receiver after
2 × the broadcast receive hold time of silence (the hold is 12 s at
NORMAL, 35 s at ROBUST/EXTREME), and the next START is a new stream.
On the sending side a chunked stream whose host stops feeding is closed
on the air with a zero-length EOS frame 30 s after the source **runs
dry** (`broadcast: host stopped feeding -- closing the stream`); the
timeout counts from the source running dry, never from chunk gaps, since
at a slow rung a healthy host's next chunk is legitimately ~27 s out.

### 10.5 Payload types and codecs

The ptype is a **label, not a demultiplexer**: a receiver that does not
know a type MUST still store the bytes. Registry in Appendix B. For
`BC_PT_LSCODEC_25` -- the only codec type any sender emits today; the two
Codec2 values are reserved names with no implementation behind them --
the payload is a stream of 10-bit tokens packed four
to five bytes; senders MUST emit only multiples of four tokens per chunk
so that every group boundary is a whole number of tokens -- otherwise
pad bits shift every later group and the stream is noise after the
first.

### 10.6 Broadcast and the ARQ link

A broadcast refreshes nothing at the peer (the peer says nothing back),
so a station's rung memory decays through a long broadcast or a pause
exactly as through silence. Two consequences bind a host running a
real-time source. It SHOULD NOT start a broadcast while `EVT_STATUS`
shows `pending` set or a non-empty queue: the link is half duplex, and
a broadcast keyed while the peer is still answering the previous
exchange goes out into a deaf receiver. And it SHOULD NOT start one on
a link the board would hold (§10.1). The reference host satisfies both
by performing an ARQ exchange (a `CMD_SUBMIT` the peer answers) when
the peer has been silent longer than ~60 s, then waiting for `pending`
to clear and the queues to empty before keying.

## 11. Application envelopes

These ride inside `CMD_SUBMIT` / `EVT_MESSAGE` data and are conventions
between host programs; the board is agnostic to them.

### 11.1 `FILE:` envelope

```
  magic:u8  "FILE:"  basename  0x00  part  n_parts  data...
```

| magic | index width | payload |
|---|---|---|
| `0x01` | `part:u8, n_parts:u8` | raw |
| `0x02` | `part:u8, n_parts:u8` | whole-file DEFLATE |
| `0x03` | `part:u16le, n_parts:u16le` | raw |
| `0x04` | `part:u16le, n_parts:u16le` | whole-file DEFLATE |

Senders MUST emit the wide form (`0x03`/`0x04`); receivers MUST accept
all four. The byte-indexed form's 255-part cap refused a 68 kB file at
~230 B/part. A distinct magic for the compressed form (rather than a
flag) means a peer predating compression sees an unknown message rather
than misparsing a valid-looking one. Compression is applied to the
**whole file** before splitting; per-part compression throws the ratio
away (measured 2.82× on a 14 kB config file).

One part is one station message. Any part size up to the peer's
`msg_max` interoperates; throughput favours **window-aligned** parts,
split at `peer_win_max × 200` minus the envelope head, so each part is
an exact multiple of the 200-byte fragment and costs exactly one
acknowledgement. Measured: 2048-byte parts 87 B/s (3 acks/part), aligned
1600-byte parts 102 B/s. The first transfer to a stranger still splits
conservatively (window 8): the capability handshake is triggered *by*
that transfer, so its record arrives too late to size it.
`demoapp/test_board_console.py` pins these offsets byte-for-byte.

### 11.2 Voice

Voice tokens travel as broadcast payload `BC_PT_LSCODEC_25` (§10.5),
not in the ARQ envelope. The host-side pipeline is documented in
`host/webvoice/README.md` and `voice/README.md`.

## 12. The over-the-air capability record, as surfaced

The stations exchange an 11-byte capability record over the air
(`FLAG_CAPS` frames; `link.md`). It is not a USB structure, but its
contents are surfaced in `EVT_STATUS` (§8.2) and two of its fields are
sourced from `CMD_CONFIG` (§9), so a host needs to know its shape:

| byte | field | surfaced as |
|---|---|---|
| 0 | `ver` = 1 | -- |
| 1 | `flags` (`CAP_*`) | `peer_caps` |
| 2 | `msg_max` u16le | `peer_msg_max` |
| 4 | `win_max` | `peer_win_max` |
| 5 | `pool_slots` | -- |
| 6 | `fw_ver` u16le | -- |
| 8 | `max_frags` | -- |
| 9 | `max_rung + 1` (0 = unspecified) | `peer_max_rung1` |
| 10 | `codecs` (`CODEC_*`) | `peer_codecs` |

The record **grows** without a version change: minimum accepted length
is 10, absent fields read as 0 / unspecified, and an older peer takes the
prefix it knows. Bumping the version would make both directions refuse
each other, which a capability mechanism cannot afford.

`peer_state` semantics: the record goes out only when bulk traffic is
waiting; an unanswered probe is forgiven by the rate controller (silence
is a fact about an older peer, not the channel); after 2 unanswered
probes (`CAPS_TRIES`) the peer is **legacy** (state 1) and that verdict
holds for 300 s (`CAPS_RETRY_S`) before a re-probe. A record older than
900 s (`CAPS_STALE_S`) is re-asked.

`CAP_STREAM` is advertised from the `burst_stream` configuration, never
from whether the firmware happens to have a burst-receive hook.

## 13. Diagnostic stream

`EVT_DIAG` mirrors the station's `ST_EV_*` events (Appendix B). It is
**off by default** (`diag_stream` = 0) and, even when on, is **shed
before anything else**: an event is dropped, and `diag_suppressed`
counted, whenever the staging ring is more than half full. Rationale,
measured: a station with no radio attached times out continuously, each
event was a frame, and with nothing draining them they filled the 512-byte
endpoint buffer within milliseconds -- the device sent exactly 549 bytes
and went silent while every register read healthy, with command
*responses* stuck behind a debug firehose. A lost diagnostic costs
nothing; a lost reply costs the session.

`station_diag_format()` renders any event as a sentence; both consoles
use it.

## 14. Versioning and compatibility

### 14.1 Protocol version

`proto_ver` in `RSP_INFO` is **1** and has not changed since the first
revision. Appending fields to a structure does **not** bump it.

### 14.2 Grow-only structures

Structures grow by **appending** fields; a field, once assigned an
offset, never moves and never changes meaning. Compatibility is by
**length**, both ways:

- A decoder MUST accept any payload at or above the structure's minimum
  decodable length and MUST substitute the documented "absent" value
  for each field beyond what was received -- never 0 where 0 is a valid
  reading (temperature) and never "none" where the field means "never
  said" (codecs).
- An encoder MUST emit the full current length.
- An older receiver reads the prefix it knows and ignores the rest.

This is the same contract as the over-the-air capability record (§12).

### 14.3 Buffer sizing

Every buffer that holds a structure MUST be sized from the structure's
length constant (`UP_STATUS_LEN`, `UP_MAX_FRAME`), never from a literal.
Appending `temp_q8` once left a caller at `UP_HDR_LEN + 40`; `up_encode`
refused the too-large frame, and the board stopped pushing status
**entirely** while enumeration and every command still worked -- it
looks like a wedged firmware and is arithmetic.

### 14.4 Two languages

Every implementation's payload cap MUST equal `UP_MAX_PAYLOAD` (§4.1).
The reference sizes exist in C (`cport/src/usb_proto.h`) and Python
(`host/ofdm_modem.py`); a mismatch fails **silently in the direction
that matters**: a host whose cap is smaller than the device's treats
every larger frame as garbage and resynchronises past it.

### 14.5 Registries

Unknown message types (§4.3), configuration keys (§9), diagnostic
events and payload types MUST be ignored, not rejected, so each registry
can grow.

## 15. Assignment considerations

There is no external registrar; this section is the local analogue of
an IANA considerations section. Every registry below is owned by
`cport/src/usb_proto.h` (or the header named), and an assignment is
made by a change there that (a) is mirrored in `host/ofdm_modem.py`,
(b) is exercised by `cport/tests/test_usb.c` or `host/test_modem.py`,
and (c) adds the row to Appendix B of this document. Because every
registry's consumers ignore unknown values (§14.5), a new assignment
never breaks an older implementation on the other side.

| registry | owner | assigned | unassigned | experimental use |
|---|---|---|---|---|
| message types, host→device | `usb_proto.h` | `0x01`–`0x06` | `0x07`–`0x6F` | `0x70`–`0x7F` |
| message types, device→host | `usb_proto.h` | `0x81`–`0x88` | `0x89`–`0xEF` | `0xF0`–`0xFF` |
| configuration keys | `usb_proto.h` | 1–9 | 10–239, 0 | 240–255 |
| `UP_CAP_*` bits | `usb_proto.h` | 0–4 | 5–31 | -- |
| `CAP_*` bits (air record) | `station.h` | 0–5, 7 | 6 | -- |
| `CODEC_*` bits | `station.h` | 0 (1–2 reserved) | 3–7 | -- |
| broadcast payload types | `broadcast.h` | 0, 3, 15 (1–2 reserved) | 4–14 | -- |
| diagnostic events | `station.h` | 0–16 | 17–255 | -- |
| `EVT_BCAST` flag bits | `usb_radio_main.c` | 7, 6 | 5, 4 | -- |
| `CMD_BCAST` flag bits | `usb_radio_main.c` | 7, 6, 5 | 4 | -- |
| `FILE:` envelope magics | `demoapp/app.c` | `0x01`–`0x04` | `0x05`–`0xFF` | -- |

**Experimental use** ranges are for a host or firmware under development
and MUST NOT appear in a shipped image; a value from them carries no
meaning outside the pair of implementations that agreed on it. Bit 7 of
a message type is the direction bit and MUST be preserved: a host→device
type MUST have it clear and a device→host type MUST have it set.

A **reserved** value (Codec2 bits and payload types, key 5, `EVT_AUDIO`)
has a name but no implementation. It stays reserved until an
implementation exists on both ends; a document that assigns it MUST
also remove the reservation note here.

Structures (§8) are not registries: they grow by appending (§14.2), and
the appended field's "absent" value is assigned with it.

## 16. Timing constants

| constant | value | where |
|---|---|---|
| status push period | 0.5 s | device |
| host attached window (`HOST_ALIVE_MS`) | 3000 ms | device |
| reference keepalive period | 1 s | host |
| open drain quiet / cap | 150 ms / 2 s | host |
| serial-string retry | 10 × 300 ms | host |
| host write timeout | 5000 ms | host |
| device worst blocking receive (EXTREME end-of-frame commit) | 2283 ms measured | device |
| device→host staging ring | 4096 B | device |
| diag shed threshold | ring > 2048 B | device |
| broadcast source buffer | 8192 B | device |
| broadcast hold-and-probe give-up | 180 s | device |
| broadcast block length (stats exchange) | 45 s (20 s after trouble) | device |
| stats reply window | ~9.4 s | device |
| ladder staleness decay | 1 rung / 90 s of peer silence | device |
| caps: probes before legacy / legacy hold / record staleness | 2 / 300 s / 900 s | device |

## 17. Security considerations

The transport provides no authentication, integrity beyond USB's own
CRC, or confidentiality; it is a local bus to a device the host owns.
`CMD_BCAST` data, `CMD_SUBMIT` data and `EVT_MESSAGE` data are opaque to
the board. A host MUST bound what it accepts from `EVT_MESSAGE`,
`EVT_BCAST` and `EVT_LOG` by the declared lengths and MUST NOT treat
`EVT_LOG` text as instructions. Payload types (§10.5) are labels chosen
by the sender and MUST NOT be trusted to select a decoder without
validating the content.

### 17.1 Consequences of not meeting a requirement

RFC 2119 §7 asks that the effect of ignoring an imperative be spelled
out. Each below was observed, not predicted.

| requirement | what happens without it |
|---|---|
| drain, never `clear_halt`, on open (§3.3) | the device endpoint wedges after one packet; every later read times out with the board healthy |
| one handle per board (§3.2) | two readers race for every event and the loser drops it; a warm link reports "did not warm up" by scheduling luck |
| write timeout > 2283 ms; never retry a partial write (§3.5) | the host dies out of its heartbeat mid-decode; a repeated partial frame desynchronises the device parser until a resync |
| payload cap = `UP_MAX_PAYLOAD` (§14.4) | a delivered file part is acked by the board and never seen by the host -- no error anywhere |
| message buffer ≥ `msg_max` (§8.1) | the same silent loss, at the application layer |
| pace `CMD_SUBMIT` on queue depths (§6.2) | messages are refused; a host that ignores the `EVT_LOG` refusal believes they were sent |
| pace `CMD_BCAST` on `bc_free`; no non-CONT chunk mid-stream (§10.1) | the whole queued broadcast is dropped, or the stream on the air is closed and superseded -- a truncated transmission |
| four-token alignment for `LSCODEC_25` (§10.5) | one lost group turns the rest of the stream into noise instead of a one-second gap |
| absent-value defaults on short structures (§8.2, §14.2) | a board with no sensor reports 0 °C; a peer that never declared codecs reads as "supports none" |
| reserved codec bits stay clear (Appendix B.3) | the peer is promised a decoder that does not exist and sends audio nobody can play |
| `EVT_DIAG` shed under backpressure (§13) | command responses queue behind diagnostics and the session stalls (measured: 549 bytes, then silence) |
| no broadcast into `pending` (§10.6) | speech keyed into a deaf receiver; nothing reports it |

## 18. References

Normative references:

- [RFC 2119] Bradner, S., "Key words for use in RFCs to Indicate
  Requirement Levels", BCP 14, RFC 2119, March 1997.
- [RFC 8174] Leiba, B., "Ambiguity of Uppercase vs Lowercase in RFC 2119
  Key Words", BCP 14, RFC 8174, May 2017.

Informative references:

- [RFC 5444] Clausen, T., Dearlove, C., Dean, J., and C. Adjih,
  "Generalized Mobile Ad Hoc Network (MANET) Packet/Message Format",
  RFC 5444, February 2009 -- the model for §2.2 syntax notation, §4.5
  malformed-element rules, §15 registries and Appendix C.

Normative code:

- `cport/src/usb_proto.h`, `usb_proto.c` -- framing and structures
- `cport/src/usb_modem.c` -- command binding, staging, status, diag
- `cport/usb/usb_radio_main.c` -- `bc_cmd`, broadcast keying and walk
- `cport/src/station.h` -- capability record, `ST_EV_*`, `CAP_*`, `CODEC_*`
- `cport/src/broadcast.h` -- `BC_*`, payload types
- `host/ofdm_modem.py` -- reference host
- `demoapp/app.c` -- `FILE:` envelope

Tests: `cport/tests/test_usb.c`, `host/test_modem.py`,
`host/test_ofdm_modem.py`, `demoapp/test_board_console.py`.

Companion documents: [usb-protocol.md](usb-protocol.md) (narrative),
[link.md](link.md) (ladder, ARQ, handshake), [phy.md](phy.md)
(broadcast air format).

---

## Appendix A. Byte-offset tables

### A.1 Frame

| offset | size | field |
|---|---|---|
| 0 | 1 | `0xA5` |
| 1 | 1 | `0x5A` |
| 2 | 1 | `type` |
| 3 | 2 | `len` u16le |
| 5 | `len` | payload |

### A.2 `EVT_BCAST` end-of-stream record

| offset | size | field |
|---|---|---|
| 0 | 1 | `flags` = `0x40 \| ptype` |
| 1 | 2 | `frames_ok` u16le |
| 3 | 2 | `frames_lost` u16le |
| 5 | 2 | `snr_q8` i16le |

### A.3 Structure minimum lengths

| structure | full length | minimum accepted |
|---|---|---|
| `up_info_t` | 26 | 24 |
| `up_status_t` | 44 | 32 |
| `EVT_DIAG` | 21 | 21 |
| capability record (air) | 11 | 10 |

## Appendix B. Registries

### B.1 `UP_CAP_*` (`RSP_INFO.caps`)

| bit | name |
|---|---|
| 0 | `LDPC` |
| 1 | `EXT_FRAMES` -- 255-byte payload frames |
| 2 | `BURST` -- selective-repeat bursts |
| 3 | `AUDIO_TAP` |
| 4 | `BCAST` |

### B.2 `CAP_*` (capability record `flags`, `peer_caps`)

| bit | name | meaning |
|---|---|---|
| 0 | `STREAM` | can follow streamed burst windows |
| 1 | `EXT` | EXT_DATA frames |
| 2 | `LDPC` | |
| 3 | `BURST` | selective-repeat bursts at all |
| 4 | `BCAST` | receives broadcasts |
| 5 | `BC_STATS` | answers an end-of-block marker with a stats frame |
| 7 | `ACK` | this record answers yours (a leg, not a capability) |

### B.3 `CODEC_*` (`codecs` config, `peer_codecs`)

| bit | name | broadcast ptype | status |
|---|---|---|---|
| 0 | `LSCODEC_25` -- 250 bit/s | 3 | implemented (`host/webvoice`, `voice/`) |
| 1 | `CODEC2_700` | 1 | **reserved** -- bit and ptype assigned, no encoder or decoder exists |
| 2 | `CODEC2_450` | 2 | **reserved** -- as above |

A host MUST NOT set a reserved bit in `codecs` (§9): the peer would read
it as a promise to play a codec nobody has written. `LSCODEC_25` is the
only voice codec in the system today.

### B.4 Broadcast payload types (`ptype` low nibble)

| value | name |
|---|---|
| 0 | `TELEMETRY` |
| 1 | `CODEC2_700` -- reserved, not implemented |
| 2 | `CODEC2_450` -- reserved, not implemented |
| 3 | `LSCODEC_25` |
| 4–14 | unassigned |
| 15 | `OPAQUE` |

### B.5 `ST_EV_*` diagnostic events

| ev | name | a | b | c | d |
|---|---|---|---|---|---|
| 0 | `TX` | rung | typ | lc.flags | payload bytes |
| 1 | `RX` | lc.flags | lc.seq | lc.ack | snr_db × 10 |
| 2 | `TIMEOUT` | losses (after) | rung | 1 = forgiven first miss | |
| 3 | `RUNG` | old | new | losses | cap |
| 4 | `BURST_ENGAGE` | nfrags | frag_size | transfer id | |
| 5 | `BURST_FRAG` | frag idx | ack_req | window_left (before) | |
| 6 | `BURST_ACKTX` | transfer id | bitmap bytes | | |
| 7 | `BURST_ACKRX` | frags acked (total) | nfrags | | |
| 8 | `BURST_PROBE` | timeout inside a burst → 1-frame probe | | | |
| 9 | `BURST_DONE` | 0 tx complete / 1 rx delivered | | | |
| 10 | `BURST_STREAM` | blocks streamed | samples | resync period | |
| 11 | `BURST_SRX` | blocks decoded | blocks examined | | |
| 12 | `BURST_SOFF` | `ST_SOFF_*`: 1 BUILD, 2 NOACK, 3 TIMEOUT | | | |
| 13 | `RTO` | srtt ms | rttvar ms | budget ms | air-time term ms |
| 14 | `BURST_WIN` | ceiling | used | fragments | burst air time, s |
| 15 | `BURST_REFRAG` | frag_size | rung | nfrags | |
| 16 | `CAPS` | 0 sent / 1 sent as reply / 2 received / 3 peer assumed legacy | flags | msg_max | win_max |

Only `ST_SOFF_NOACK` is a statement about the peer and is remembered
across transfers; `TIMEOUT` and `BUILD` describe the channel and local
buffers and stay per-transfer.

## Appendix C. Worked example

Every byte below was captured from the device-side reference code
(`cport/build/usb_modem_emu`, the firmware's protocol layer on stdio)
driven by `host/ofdm_modem.py`'s encoder. Frames are shown as the octets
on the wire; `H→D` is host to device.

### C.1 Attach: `CMD_INFO`

```
H→D  a5 5a 01 00 00
     └sync┘ ty len=0

D→H  a5 5a 81 1a 00 | 01 03 04 01 | 24 00 41 00 05 51 33 34 38 36 34 36
     └sync┘ ty len=26  ver n  fw     uid[12]
                      | 07 00 00 00 | e0 2e 00 00 | 00 01
                        caps=0x07     rate=12000    msg_max=256
```

`proto_ver` 1, three modes, firmware 1.4 (`0x0104`), caps LDPC | EXT |
BURST, 12 kHz, and `msg_max` 256 -- this build's `ST_MSG_MAX`; a radio
image reports 3328 here.

### C.2 Keepalive: `CMD_PING` / `RSP_PONG`

```
H→D  a5 5a 04 04 00 | ef be ad de          token 0xDEADBEEF
D→H  a5 5a 85 04 00 | ef be ad de          echoed verbatim
```

### C.3 Submit an interactive message

```
H→D  a5 5a 02 0c 00 | 01 | 43 51 20 64 65 20 52 32 41 42 43
                      qos  "CQ de R2ABC"
```

No response. The effect is visible in the next status frame (C.5):
`q_inter` = 1.

### C.4 Configuration query and set

```
H→D  a5 5a 03 00 00                         empty = query

D→H  a5 5a 86 6b 00 | 63 6f 6e 66 69 67 3a 20 ...   (107 bytes of text)
     "config: rung_ceiling 12  win_max 8  burst_window 0  burst_stream 0
      anchor 0  diag_stream 0  freq_trim_hz 0"

H→D  a5 5a 03 05 00 | 09 | 01 00 00 00      key 9 (codecs) := 1 (LSCODEC_25)
```

The set produces no response; the bit reaches the peer in the next
capability record.

### C.5 An unsolicited `EVT_STATUS`, decoded

```
D→H  a5 5a 83 2c 00 |                                      len = 44
       00 00 00 00   rung            0
       00 9d ff ff   snr_q8          -25344  = -99.0 dB, no measurement (§8.2)
       00 00 00 00   tx_frames       0
       00 00 00 00   rx_frames       0
       00 00 00 00   timeouts        0
       00 00 00 00   retransmissions 0
       00 00         q_ctl           0
       01 00         q_inter         1       <- the message from C.3
       00 00         q_bulk          0
       00            busy            0
       00            pending         0
       00            peer_state      0       unknown
       00            peer_caps       0
       00 00         peer_msg_max    0
       00            peer_win_max    0
       00            peer_max_rung1  0       unspecified
       00 00         bc_free         0       no broadcast source in this build
       00 80         temp_q8         -32768  UP_TEMP_NONE: no sensor
       00            rung_now        0
       00            peer_codecs     0       never said
```

A 40-byte status from older firmware would end at `bc_free`; a decoder
then reports `temp_q8` as `UP_TEMP_NONE`, `rung_now` as
`UP_RUNG_ABSENT` and `peer_codecs` as 0 (§8.2) -- exactly the values
this full frame happens to carry, which is the point of choosing them.

### C.6 A chunked real-time broadcast (host side)

The voice host sends LSCodec tokens as they are encoded, one profile
step at a time -- here 24 tokens = 30 bytes, always a multiple of four
tokens (§10.5) -- real-time, board-chosen rung:

```
H→D  a5 5a 06 20 00 | a3 ff | <30 bytes>     MORE|RT|ptype 3, rung 0xFF: start
H→D  a5 5a 06 20 00 | e3 ff | <30 bytes>     MORE|CONT|RT: middle
     ...
H→D  a5 5a 06 20 00 | 63 ff | <30 bytes>     CONT|RT, MORE clear: last chunk
```

On the receiving board the same stream surfaces as:

```
D→H  a5 5a 88 01 00 | 83                     START, ptype 3
D→H  a5 5a 88 61 00 | 00 | <96 bytes>        DATA, one 4-frame group
     ...
D→H  a5 5a 88 07 00 | 43 | 1c 00 | 00 00 | 8c 16
                      EOS   ok=28   lost=0   snr_q8=5772 = +22.5 dB
```

The `CMD_BCAST` and `EVT_BCAST` octets in C.6 are constructed from the
syntax in §4.4 and §10 rather than captured: the emulator has no radio,
so it neither keys nor receives a broadcast. Their field values
(28 frames, 0 lost, +22.5 dB) are from a measured 16-second
clean-wire run.
