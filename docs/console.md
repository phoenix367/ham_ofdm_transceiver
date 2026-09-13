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
