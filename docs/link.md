# Link layer

Two half-duplex stations form **two directed links**, each governed by its
*receiver* — only the receiver knows its own noise floor. There is no shared
"link mode" to negotiate; each side requests what it can hear.

## The rate ladder

13 operating points spanning **7.8 → 1059 bit/s** over **−17.9 → +4.7 dB**
(sensitivities measured with the recalibrated receiver; dominated rungs
removed). Requests and grants are ladder indices in the LC word.

| # | mode / mod / FEC | bit/s | sens dB | # | mode / mod / FEC | bit/s | sens dB |
|---|---|---|---|---|---|---|---|
| 0 | EXTREME BPSK ⅓ | 7.8 | −17.9 | 7 | NORMAL QPSK ½ | 353 | −5.3 |
| 1 | ROBUST BPSK ⅓ | 31 | −11.7 | 8 | NORMAL QPSK ⅔ | 471 | −3.8 |
| 2 | ROBUST BPSK ½ | 46 | −11.8 | 9 | NORMAL QPSK ¾ | 529 | −2.2 |
| 3 | ROBUST QPSK ⅓ | 62 | −11.3 | 10 | NORMAL QAM16 ½ | 706 | +0.7 |
| 4 | NORMAL BPSK ⅓ | 118 | −7.6 | 11 | NORMAL QAM16 ⅔ | 941 | +2.6 |
| 5 | NORMAL BPSK ½ | 176 | −7.3 | 12 | NORMAL QAM16 ¾ | 1059 | +4.7 |
| 6 | NORMAL QPSK ⅓ | 235 | −7.0 | | | | |

## The link-control word (20 bits, in `Data.reserved`)

```mermaid
flowchart LR
    S["seq<br/>2b"] --- A["ack<br/>2b"] --- R["req_rung<br/>4b"] --- N["snr report<br/>4b · 2 dB steps"] --- F["freq corr<br/>5b · ±120 Hz"] --- G["flags<br/>3b"]
```

Flags: bit0 = last fragment, bit1 = no-data (ack-only), bit2 = priority
stream. Every frame piggybacks the full word — data, ACKs, everything.

## Adaptation: three feedback paths, three latencies

```mermaid
flowchart TD
    subgraph RXside["receiver side (per inbound link)"]
        M["per-frame SNR"] --> FLT["fade-aware filter:<br/>2nd-lowest, age ≤60 s"]
        FLT --> REQ["rung request<br/>up: sens+2.5 dB held<br/>down: margin < 1 dB<br/>silence: decay + purge history"]
    end
    subgraph TXside["transmitter side (per outbound link)"]
        PR["peer request"] --> MIN
        CAP["peer's SNR report of MY signal<br/>(clamps instantly when peer goes deaf)"] --> MIN
        OFF["learned per-rung offsets<br/>loss +0.7 dB / success −0.15 dB"] --> CAP
        MIN["min(...)"] --> LOSS["loss fallback<br/>2 losses → −2 rungs<br/>4 losses → rung 0"] --> RUNG["TX rung"]
        STALE["request age > 90 s<br/>→ decay"] --> LOSS
    end
    REQ -. "in LC word" .-> PR
```

Bootstrap: both sides start at rung 0 (EXTREME) — it works whenever anything
works — and one decoded exchange is enough measurement to jump to the
SNR-indicated rung (fast start).

## ARQ and HARQ

Stop-and-wait with piggybacked ACKs (2-bit seq is sufficient). On CRC
failure the PHY exports the data-block LLRs; a retransmission is first tried
alone, then **chase-combined** (LLR sum) with the stored attempt — blind,
because the seq lives inside the undecodable payload, and safe, because the
CRC arbitrates. Worth ~3 dB on a same-rung retransmission in the
noise-limited regime.

```mermaid
sequenceDiagram
    participant A as station A
    participant B as station B
    A->>B: DATA seq=1 (fades mid-frame)
    Note over B: CRC fails → store LLRs
    B-->>A: (no ack → timeout, backoff)
    A->>B: DATA seq=1 retransmit
    Note over B: alone: fail →<br/>fresh+stored: CRC OK
    B->>A: ACK 1 (+ SNR report, rung request)
```

## QoS

Three classes as *policy*, not machinery: strict priority
control > interactive > bulk; per-class air-time caps size fragments
(latency budgeting — a 27-byte EXTREME frame is 43 s); control/ACK traffic
rides one rung below bulk (extra margin). The priority stream **preempts**
an in-progress bulk message at fragment boundaries (LC flag bit2 selects one
of two reassembly buffers) — no head-of-line blocking behind a long
transfer.

## Simplex channel access

```mermaid
stateDiagram-v2
    [*] --> Listening
    Listening --> Transmitting: traffic or ack owed,<br/>channel idle, backoff expired
    Transmitting --> AwaitReply: frame sent,<br/>expects reply
    Transmitting --> Listening: ack-only sent
    AwaitReply --> Listening: frame decoded
    AwaitReply --> AwaitReply: timeout but channel BUSY<br/>(peer may answer at a slower rung)
    AwaitReply --> Backoff: timeout, channel idle<br/>→ count loss
    Backoff --> Listening: random 1–6 s elapsed
```

Key rules: carrier sense before keying; the listen window is sized from the
*expected reply's* air time; **a timeout on a busy channel is not a loss**;
random backoff breaks the symmetry of two stations keying together; a
station replies only to frames that carried data — no ack-of-ack loops.

## Capability handshake

`station.c` (`FLAG_CAPS`). Before this, what the peer could do was
discovered by failing at it: streaming was learned from a window that
came back one-eighth acked (`ST_SOFF_NOACK`, then re-probed every
`PEER_STREAM_RETRY` transfers), the fragment size was sized from *our*
limits, and `sendfile` fired a bare `LINK` frame just to wake the ladder.
The record makes it explicit, and it rides on the third impossible
flag combination — `NO_DATA|LAST|PRIO` = 7 — the same trick that gave
the burst frames their types, so an older peer reads it as a no-data
frame and simply does not answer.

```
A -> B   CAPS             A has bulk to send and knows nothing about B
B -> A   CAPS | CAP_ACK   B stores A's record and answers with its own
A -> B   (any frame)      its ack of B's seq is the third leg: B now
                          knows A holds B's record
```

Ten bytes: version, capability bits (`stream ext ldpc burst bcast`),
`msg_max`, `win_max`, pool slots, firmware version, max fragments. It
goes out at the control rung, and only when there is bulk to carry — a
chat message gains nothing from the peer's window size. Three things
the record decides:

- **streaming is declared, not inferred**: a peer that says no is never
  streamed to (`peer_stream_retry = -1`, which no clean exchange can
  overturn); a peer that says yes keeps the `NOACK` fallback, because a
  fade can still forge that signature;
- **the window is capped by the peer's `win_max`** at engage;
- **the message size is the peer's**: the boards report their own
  `ST_MSG_MAX` in the USB INFO reply and the peer's in STATUS, and the
  consoles split files against the smaller;
- **two operator knobs travel in the record**: `config win_max` (the
  streamed-window ceiling this station accepts and sends) and
  `config rung_ceiling` (the fastest rung it transmits at or asks
  for). Both are enforced at every rung decision and at window engage,
  against our own setting and against whatever the peer declared — and
  changing either while the peer is known pushes a refreshed record
  immediately.

An unanswered probe is *forgiven* by the rate controller — silence
from an older firmware is a fact about the peer, not the channel — and
after `CAPS_TRIES` the peer is marked legacy and today's defaults
apply, re-asked after `CAPS_RETRY_S`. A record older than
`CAPS_STALE_S` is re-asked too. The declared streaming bit comes from
the operator's `burst_stream` knob, **not** from the PHY hooks: a
streaming receiver leaves `receive_burst` NULL by design.

Why it mattered on the boards: at `ST_MSG_MAX` 256 a console part was
two fragments, so a transfer paid one acknowledgment per 260 bytes —
~50 B/s at rung 12 against a raw ~1 kbit/s, and every ack one more
exposure to the acquisition-miss rate. The boards now hold 3328-byte
messages (the station struct lives in SRAM4 — the slowest RAM on the
part, exactly right for a frame-cadence structure the ISR never
touches), and the streamed window ceiling is 16: a window-aligned part
is one acknowledgment per ~3.2 kB, measured at 114 B/s on the wire
against 87 before — 87 % of the raw channel rate.

## AFC frequency netting

The receiver measures the peer's carrier offset on every frame and requests
a correction in the LC word (±120 Hz/frame, 8 Hz steps, ±12 Hz deadband).
The peer trims its shared reference at **half gain** — both sides act on
stale measurements, and full-gain corrections would swap offsets each turn.

```mermaid
sequenceDiagram
    participant A as A (+284 Hz off)
    participant B as B
    A->>B: CQ (EXTREME) — B measures +284 Hz
    B->>A: ACK, freq_corr −120 (field max)
    Note over A: trim −60 (half gain)
    A->>B: DATA — B measures +224
    B->>A: ACK, freq_corr −120
    Note over A,B: … 3 more exchanges …
    Note over A,B: netted: residual < 12 Hz,<br/>drift tracked thereafter
```

Guard rails (the loop only sees the *differential* offset, so the common
frequency would otherwise random-walk): `afc_max_trim_hz` (±150 Hz default)
hard-clamps each station's cumulative trim — un-correctable residue is left
to the modem's ±300 Hz tolerance; `afc_anchor=True` designates a station
that never trims, pinning the absolute frequency.
