# The modem's protocol model

What a host application implements against a board, one layer above the
[USB transport](usb-protocol.md). The air-side machinery it drives is
documented in [link.md](link.md) (ladder, ARQ, capability handshake)
and [phy.md](phy.md) (broadcast); this page is the host's-eye view.

## The model

The board *is* the station: queues, ARQ, rate ladder, carrier sense and
all the DSP run in firmware. The host is a thin terminal that

1. **submits messages** (`CMD_SUBMIT` with a QoS class — 0 control,
   1 interactive, 2 bulk) and receives delivered ones (`EVT_MESSAGE`);
2. **paces itself against the status stream** (`EVT_STATUS`, 2 Hz):
   queue depths for file parts, `bc_free` for broadcast chunks. Pacing
   is self-correcting by construction — the board reports true depths,
   so a dropped submit simply shows up as room again;
3. **configures** the station (`CMD_CONFIG`; empty payload = query,
   answered as a log line — the settings live on the board, and a host
   cache would lie after a reattach);
4. **stays alive by traffic**: both consoles ping once a second. The
   board drops its host-attached indication (and LED state) after 3 s
   without a command, because closing a program unmounts nothing.

A refused submit ("queue or store full") comes back as `EVT_LOG` —
the host queues on the board's behalf and has to know when to stop.

## Message envelopes (application layer, inside SUBMIT/MESSAGE)

Files ride the `FILE:` envelope:
`magic(1) "FILE:" basename NUL part(2:le) n_parts(2:le) data...`,
magic 0x03 raw / 0x04 whole-file DEFLATE (the byte-indexed 0x01/0x02
form is still received). One part is one station message, and parts
should be **window-aligned** — split at `peer_win_max × 200` minus the
envelope head — so each costs exactly one acknowledgment.
`demoapp/test_board_console.py` pins these offsets byte-for-byte.

## Broadcast, host side

A broadcast is fire-and-forget on the air but a *conversation* with the
board: the host feeds chunks (`CMD_BCAST`, bits 7/6), the board logs
what it chose (`EVT_LOG`: rung, group geometry, total air-time
estimate — or "held ... probing" when it negotiates first), and the
receiving board streams the payload up as `EVT_BCAST` chunks ending in
a stats record. Delivery guarantees: none; the stats say what arrived
(head loss included — a stream starts at seq 0, so a first lock at
seq N means N frames are gone).

When both boards hold each other's capability record with the
block-stats bit, the *air* protocol adds receiver feedback and
mid-stream rate adaptation — that machinery is entirely
board-to-board; the host only sees its effects in the log lines
("stats from peer: … asks rung 12 — re-rung").

## The diagnostic stream

`EVT_DIAG` events mirror `station.c`'s `ST_EV_*` set (TX, RX, TIMEOUT,
RUNG, the burst family, RTO, CAPS — 17 events). They are **off by
default** and shed before anything else under backpressure: a station
with no radio attached once filled the endpoint with timeout events
and buried its own command responses (the measured wedge that made the
stream opt-in). `station_diag_format()` renders any event as a
sentence; both consoles use it (`debug on` enables stream and printing
together).

## Liveness and identity

- `CMD_INFO` at attach returns protocol version, firmware version, the
  96-bit UID, capability bits and `msg_max` — the host sizes its
  buffers from it rather than assuming.
- `CMD_PING`/`RSP_PONG` is the keepalive and round-trip probe.
- `CMD_RESET` re-initializes the station without dropping USB — the
  difference between a modem that recovers and one that re-enumerates.
