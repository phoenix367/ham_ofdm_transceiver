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
| `status` | rung, SNR, queues, peer capabilities, broadcastfile feed progress |
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

## Files produced

| file | source |
|---|---|
| `rx_<name>` | received `sendfile` (in the console's working directory) |
| `rx_broadcast.bin` | received `bcastfile`/`broadcastfile` (opaque broadcasts) |
