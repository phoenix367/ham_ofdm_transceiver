# USB transport protocol

How a host talks to a board (`cport/src/usb_proto.h` is normative; this
page is the narrative). The semantic layer above it — what the commands
and events *mean* — is [modem-protocol.md](modem-protocol.md).

## Device

- Vendor-class interface, VID:PID `1209:0001`, one bulk IN + one bulk
  OUT endpoint. The USB serial string is the STM32's 96-bit unique ID
  in hex — it is how a console addresses a specific board.
- Access needs the udev rule (`cport/usb/README.md`).
- **Opening rule: drain, never reset.** The device pushes status frames
  at 2 Hz whether anyone listens or not, so the IN endpoint is armed at
  any moment. A `clear_halt` then desynchronizes the stack's software
  state from the hardware transfer and wedges the endpoint (measured;
  report §12.6 has the sequence diagram). Both host transports instead
  read-and-discard until 150 ms of quiet.

## Framing

Byte stream in both directions, framed identically:

```
A5 5A <type:u8> <len:u16le> <payload[len]>
```

`len ≤ UP_MAX_PAYLOAD` (3336 — sized so one station message plus its
qos byte crosses in one frame). The parser resynchronizes on the sync
pair after garbage; its `resyncs` counter is 0 on a healthy link. All
multi-byte payload fields are little-endian, all layouts fixed-size.

## Frame types

Host → device:

| type | name | payload |
|---|---|---|
| 0x01 | `CMD_INFO` | — |
| 0x02 | `CMD_SUBMIT` | `qos:u8, data[]` |
| 0x03 | `CMD_CONFIG` | `key:u8, value:i32le` — or **empty = query**, answered with a one-line `EVT_LOG` |
| 0x04 | `CMD_PING` | `token:u32le` (echoed in `RSP_PONG`) |
| 0x05 | `CMD_RESET` | re-init the station, keep the link up |
| 0x06 | `CMD_BCAST` | `ptype:u8, rung:u8, data[]` — see below |

Device → host:

| type | name | payload |
|---|---|---|
| 0x81 | `RSP_INFO` | `up_info_t` |
| 0x82 | `EVT_MESSAGE` | `qos:u8, data[]` — a delivered message |
| 0x83 | `EVT_STATUS` | `up_status_t`, pushed every 0.5 s |
| 0x84 | `EVT_DIAG` | `ev:u8, a..d:i32le, t_ms:u32le` — off by default (`diag_stream`), shed before anything else under backpressure |
| 0x85 | `RSP_PONG` | echoed token |
| 0x86 | `EVT_LOG` | UTF-8 text, no terminator |
| 0x87 | `EVT_AUDIO` | reserved (a sample tap no firmware emits) |
| 0x88 | `EVT_BCAST` | received broadcast, streamed — see below |

## `CMD_BCAST` — chunked broadcast source

`ptype`'s low nibble is the payload type (0 telemetry, 1/2 Codec2
700/450, 3 LSCodec-25Hz, 15 opaque — a label, not a demux). Three high
bits shape the source:

| bit | meaning |
|---|---|
| 7 | more chunks follow |
| 6 | continuation of the broadcast in flight |
| 5 | real time: key what is queued, do not wait for a full group; no block/stats pauses |

Bits 7 and 6 make the source chunk-fed, so a file larger than the
board's 8 kB source buffer streams from the host.

Both clear = the classic one-shot broadcast. `rung` 0xFF asks the board
to choose (hold-and-probe applies); 0..12 pins it. The host paces
chunks against `bc_free` in the status frame. A stray continuation with
no broadcast in flight is ignored.

## `EVT_BCAST` — streamed reception

First byte is flags: bit 7 = stream start (low nibble = ptype), bit 6 =
end-of-stream, whose payload is the stats record
(`frames_ok:u16le, frames_lost:u16le, snr_q8:i16le`); otherwise the
payload is reassembled data bytes as they decode. One start event per
*stream* (not per group); consoles reset their sink on it.

## `up_info_t` (26 B)

`proto_ver:u8, n_modes:u8, fw_ver:u16, uid[12], caps:u32,
sample_rate:u32, msg_max:u16`. `msg_max` is the station's largest
SUBMIT (0 from older firmware ⇒ assume 256, and the host's own buffer
must actually match what it advertises). `caps` bits: LDPC(0),
EXT_FRAMES(1), BURST(2), AUDIO_TAP(3), BCAST(4).

## `up_status_t` (44 B)

`rung:i32, snr_q8:i32, tx_frames:u32, rx_frames:u32, timeouts:u32,
retransmissions:u32, q_ctl:u16, q_inter:u16, q_bulk:u16, busy:u8,
pending:u8`, then the peer's capability record as learned over the air
(`peer_state:u8` 0 unknown / 1 legacy / 2 held / 3 confirmed,
`peer_caps:u8, peer_msg_max:u16, peer_win_max:u8,
peer_max_rung1:u8` = ceiling+1, 0 unspecified), then `bc_free:u16` —
free bytes in the broadcast source buffer, the chunk-pacing signal,
then `temp_q8:i16` — the die temperature in Q8 °C from the part's own
sensor, or `UP_TEMP_NONE` (−32768) where there is none — then
`rung_now:i8`, the rung the **next** frame would go out at (−1 = none
yet, `UP_RUNG_ABSENT` = older firmware omitted it), then
`peer_codecs:u8` — the `CODEC_*` bitmap the peer declared it can decode
(0 = it never said, NOT "supports none"). `rung` at the head
of the frame is the last rung actually *transmitted* and can be hours
stale, which is why both are carried: an overnight-idle station
reported `rung 12` and then sent at rung 0. Older firmware
sends 32 or 40 B; decoders default the peer record and `bc_free` to
zero and the temperature to `UP_TEMP_NONE` — never to 0 °C, which is a
real temperature.

**Size the buffer from `UP_STATUS_LEN`, not from a literal.** The
payload grows as fields are appended: the temperature turned a
hardcoded `UP_HDR_LEN + 40` in the encoder's caller into a buffer two
bytes short, `up_encode` refused the frame, and the board stopped
pushing status entirely while enumeration and every command still
worked.

## Config keys

| key | name | value |
|---|---|---|
| 1 | `rung_ceiling` | fastest rung transmitted or requested (0..12) |
| 2 | `burst_window` | operator window ceiling |
| 3 | `burst_stream` | 0/1 |
| 4 | `freq_trim_mhz` | LO trim, millihertz, signed |
| 5 | `audio_tap` | 0 off, else decimation |
| 6 | `anchor` | 0/1, AFC reference |
| 7 | `diag_stream` | 0/1, default off |
| 8 | `win_max` | streamed-window ceiling accepted and sent |
| 9 | `codecs` | `CODEC_*` bitmap the attached host can decode (bit 0 LSCodec-25Hz, 1 Codec2 700, 2 Codec2 450) |

`rung_ceiling`, `win_max` and `codecs` are also declared to the peer in
the over-the-air capability record (11 B, accepted from 10 — it grows
without a version bump); changing the first two while the peer is known
pushes a refreshed record immediately. Key 5 is registered but no
shipped firmware acts on it.
