# `ofdm_console` — the station console

One binary (`demoapp/build/ofdm_console`, built by `make -C demoapp`)
with two modes that share the application layer but differ in where the
modem runs:

```bash
./ofdm_console /tmp/ofdmchan/s1.sock S1     # socket mode: DSP + station IN THIS PROCESS,
                                            # audio to a channel driver (drivers.md)
./ofdm_console --list                       # enumerate attached boards
./ofdm_console --usb [serial] [name]        # USB mode: a terminal onto a real board's
                                            # own station (usb-protocol.md)
```

`board_console.py` is the Python equivalent of USB mode (same commands,
same envelopes; `--serial`, `--list`, `--msg-max` to override what INFO
reports). Serial may be omitted when exactly one board is attached; the
default console name is the serial's head (both boards of a wafer share
the tail).

## Commands

Delivered transfers (ARQ: tracked, retransmitted until acknowledged):

| command | what it does |
|---|---|
| `send <text>` | interactive message; jumps ahead of bulk traffic |
| `sendfile <path>` | file over burst ARQ, DEFLATEd whole first (magic 0x04/0x03; received as `rx_<name>`) |
| `bulk <n>` | queue an n-byte test pattern |

Non-ARQ transfers (nothing is ever repeated; losses stay lost):

| command | what it does |
|---|---|
| `bcast [-r <rung>] <text>` | text broadcast (`broadcast` in socket mode) |
| `bcastfile [-r <rung>] <path>` | file broadcast, raw bytes, received as `rx_broadcast.bin`; no size cap — the console reads the file sequentially as the board drains |

Configuration and diagnostics:

| command | what it does |
|---|---|
| `config` | ask the board for its current settings (the settings live there) |
| `config <key> <val>` | set one: `rung_ceiling burst_window burst_stream freq_trim_mhz audio_tap anchor diag_stream win_max` |
| `debug [on\|off]` | the board's diagnostic event stream + printing (one command does both) |
| `status` | rung (the next frame's; `last tx N` when it differs), SNR, die temperature, queues, peer capabilities, broadcastfile feed progress |
| `stats` | frame counters |

## Behaviors worth knowing

- **File parts are window-aligned.** A part is one station message,
  split at `peer_win_max × 200` minus the envelope head so every part is
  exactly one streamed window — one acknowledgment per part. The first
  bulk transfer to a stranger splits conservatively (window 8): the
  capability handshake is *triggered by* that transfer, so its record
  arrives too late to size it. Prime with any small bulk item when it
  matters.
- **Pacing is against the board, not a timer**: file parts against the
  bulk-queue depth in the status stream (`INFLIGHT` in flight),
  broadcastfile chunks against the `bc_free` field.
- **Broadcasts negotiate first** when no rung is given: an idle or
  stale link holds the payload while the board probes (the probe *is*
  the capability exchange), releasing at the negotiated rung — the
  board's log line states the rung, the group geometry and the total
  air-time estimate. An explicit `-r` bypasses the hold; `-r 0` is the
  beacon-to-strangers case (EXTREME is the only mode an idle station
  is guaranteed to be listening on).
- **`status` shows the peer's declared record** once the handshake has
  run: capabilities, message size, window, rung ceiling. `capabilities
  unknown` means no bulk exchange has happened yet.
- Socket mode adds channel-side knobs (`stream on|off`, `window <n>`,
  `compress on|off`, `tune <hz>`) — see `help` in the console.

## Scripted: the two-board benchmark

`host/board_bench.py` is two `board_console.py` consoles driven by a
script instead of a keyboard (the `Console` class, subclassed to record
instead of print, one thread per board owning its USB handle), plus the
webvoice server for speech. It runs the traffic an operator generates,
in both directions and in combination, and records what each cost:

| scenario | what is measured |
|---|---|
| `attach` | INFO, initial rung / SNR / peer state / die temperature |
| `warmup` | a bulk primer each way: seconds to delivery and to the capability handshake, rung and SNR after |
| `message` | interactive messages A→B then B→A, latency each |
| `chat` | messages queued on both boards at once: arrival times |
| `bulk` | bulk-queue test patterns each way: latency, B/s |
| `file` | a file each way: byte-exact, B/s, tx frames, timeouts, retransmissions |
| `file_bidir` | a file each way at the same time |
| `file_chat` | a file one way while the receiver chats back |
| `bcast` / `bcastfile` | text / file broadcast each way: frames ok / lost, bytes stored |
| `bcast_msg` | a reply queued at the receiver during a broadcast: when it arrives |
| `speech` | push-to-talk each way: warm-up, first audio out, bytes lost, frames ok / lost |
| `speech_file` | a file right after the speech |
| `stale` | idle past `RX_STALE_S`, then one message: rung decay and recovery |

Every scenario records ok/failed and its metrics; a failure does not
stop the run. Output: `results/board_bench.json` (metrics plus both
consoles' logs) and `results/board_bench.md` (one line per scenario);
`--quick` for small sizes, `--scenarios` for a subset, `--merge` to
re-run a subset into the existing record. Received files, logs and the
speech source live in `results/board_bench_work/` (ignored by git).

Two things the numbers mean: a message queued during a broadcast
arrives ~18 s after the broadcast ends (the receiver's transmitter is
held for `BC_RX_HOLD_S` past the last group), and a cold link (both
boards decayed to rung 0) takes ~55 s to warm because the primer and
its acknowledgement go out as EXTREME frames.

## Channel debug mode (boards only)

The board can impair its OWN transmit output so the peer sees a
channel instead of a cross-wire: `config chan_snr <dB>` adds white
Gaussian noise at that SNR relative to the clean signal's mean power
(999 = off), `config chan_fade <centi-Hz>` adds Rayleigh fading with
that Doppler spread (0 = off), and `config chan_delay <0.1 ms>` makes
it two equal taps with that delay (0 = one flat tap). CCIR 520 HF
presets: good 5/10, moderate 10/50, poor 20/100 (delay/fade). The
impairment is integer and saturating (clamped to int16, never wrapped)
and restarts at every key-up; fading's Hilbert delay is flushed at the
end of each transmission. `config` alone shows the settings; the
benchmark applies them to both boards with `--config`. `cport/src/chanimp.c`,
tested by `make test` (`test_chan`).

What it showed the first time it was switched on (2026-09-13, stand):
with A impaired and B clean, a 46 s EXTREME frame and BPSK groups
cross the CCIR "moderate" channel at 20 dB intact, while a QAM16 group
at rung 12 is not even acquired -- and a file transfer with BOTH boards
impaired never finishes, because the receiver's SNR estimate reads
15-19 dB for a set 10-20 dB, the ladder therefore sits on QAM16 rungs
that Rayleigh fades kill, and the acknowledgements die the same way
until consecutive losses walk it down to rung 0. Two calibrations the
mode exposes: the on-wire SNR estimate reads ~9 dB above the configured
AWGN level (10 dB set -> 19 dB read, 3 -> 15), and the rate ladder has
no notion of fading depth. Neither is the mode's business; both are now
measurable without a real channel.

## PTT for a radio (boards only)

The radio firmware keys **PB14** (active-high) for every transmission:
from key-up until the last sample has left the DAC. Wire a radio's PTT
driver to it and the radio transmits exactly when the board does. Two
settings add time around the frame for the radio, both 0 by default
(the cable stand has no radio, and its measured figures were taken
without them): `config ptt_lead <ms>` holds the DAC at mid-rail for
that long after keying, before the frame's first sample -- the radio's
TX attack time; the frame starts later, it is never cut -- and
`config ptt_tail <ms>` holds it after the last sample, before
releasing. Both clamp to 0..2000 and show in `config`.

Measured with a Baofeng on the pin (HackRF, 2026-11-15): the firmware
holds PB14 for exactly lead + frame + tail (the beacon's
`ptt_last_ticks`), and the radio stays on the air another 0.20-0.36 s
after release. A station replies 150 ms after decoding and nobody hears
while keyed, so a TAIL is only safe when the peer's lead covers it: tail
200 against a peer with lead 0 lost every acknowledgement and walked the
ladder down to EXTREME, while the same tail against a peer with lead 300
delivered every message first time. With radios at both ends, set
`ptt_lead` to ~400 ms on BOTH stations (it also has to cover the other
radio's release time) and leave `ptt_tail` at 0. The lead must also
cover the RECEIVING radio's squelch opening: the EXTREME receiver needs
nearly its whole preamble -- host replay of a clean frame (interf_rx,
2026-11-17) reads 16.2 dB whole, 7.4 dB with the first 0.4 s missing,
-5.7 dB at 0.6 s, and fails to decode at 0.8 s. That is how a HackRF
recording that started 0.6 s late read a clean Baofeng link as -7.5 dB.

The pin is released if the firmware stops: after 1 s with the DAC
starved while keyed (the beacon's `ptt_cuts`), and on any fault. The
beacon also carries `ptt_last_ticks` (the last keyed time, in 12 kHz
ticks) and `ptt_keyups`, which must equal `tx_frames` --
`bench/radio_beacon.py` says so when it does not. `cport/src/ptt.h`,
tested by `make test` (`test_ptt`).

### Receive input

The receiver samples **PA6** by default -- the board-to-board wire.
`config rx_input 1` switches it to **PA0**, where a radio's audio
arrives through an RC front end (`tools/baofeng/kicad/`: J2 pin 2 to
PA0, pin 1 to 3.3 V, pin 3 to GND), and `config rx_input 0` switches
back, so the wire can stay connected while a radio is tried. The switch
happens between two converter samples; the DC blocker and carrier sense
then settle (~0.1 s). It shows in `config`, and the beacon's
`rx_input` reports the input the converter is actually set to.

### Radio microphone output

Every transmitted sample goes out on **two** DAC pins: **PA4** (the
board-to-board wire) and **PA5** (DAC channel 2), for a radio's
microphone through the AIOC rev 1.2 mic network (PA5 -> 4.7 k -> node,
1.5 k node to GND, 4.7 uF node to MIC+; the 100 R || 100 nF attenuator
leg from the node to **PA3**). PA3 is an open-drain output held LOW
from boot -- mic level, ~24 mV peak at our frame peaks, the AIOC's own
default for these radios. `config mic_boost 1` releases it for line
level (~0.3 V peak, 22 dB hotter, which overdrives a Baofeng), and
`config mic_boost 0` pulls it low again. `config mic_gain <dB>`
(-40..0, default 0) scales PA5 alone, digitally, for finding the
level the radio's limiter leaves alone. `config chirp <ms>` keys one
linear 280-2700 Hz test sweep through the same path, at a modem
frame's RMS, for measuring a radio channel. Separate pins, so the
network never loads the wire. PA2, PA3 and PA5 were measured free on
both boards (each follows the internal pull both ways).

What these settings measured on a Baofeng -- linearity, the limiter,
the response, the noise floor, the recommended `mic_gain` -- is in
[radio.md](radio.md).

## Leaving

Both consoles say goodbye to the board (`CMD_DISCONNECT`) on `quit`,
on end of input and on Ctrl-C, so the board's "host attached" LED state
drops at once instead of 3 s after the last keepalive. A crash or a
pulled cable still relies on the timeout. The Python driver sends it
from `OfdmModem.close()`, so every Python host program that closes
cleanly (`board_console.py`, `board_bench.py`, the webvoice server, the
KISS bridge) gets it for free.

## Files produced

| file | source |
|---|---|
| `rx_<name>` | received `sendfile` (in the console's working directory) |
| `rx_broadcast.bin` | received `bcastfile`/`broadcastfile` (opaque broadcasts) |
