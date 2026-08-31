---
name: test-broadcast
description: Exercise or diagnose the non-ARQ broadcast path - host harnesses first (make bcrepro, make bcfade), then bcast/bcastfile on the two boards - including hold-and-probe negotiation, block statistics and mid-stream re-rung. Use after changing bc_* in usb_radio_main.c, broadcast.c, or the block/stats machinery.
---

# Broadcast: harnesses, then the wire

Broadcast has no ARQ, so a lost group is lost whole and nothing in the
protocol will tell you twice. Reproduce off-board first; the stand is
for what only silicon shows.

## Host harnesses (seconds, not board cycles)

```bash
cd cport
make bcrepro && ./build/bc_repro          # build-and-walk twin of the
                                          # firmware path: expect 6/6
make bcfade                               # adaptive vs fixed rung over
                                          # a fading profile, both arms
./build/bc_fade -lo -5 -len 8192          # deep fade; -fixed for the
                                          # control arm, -r pins a rung
```

`bc_repro` decoding 6/6 means a group loss on the air is an
acquisition miss, not a code path. `bcfade` is the only place the
control laws can be exercised: measured +32 % delivered in a deep
(-5 dB) fade and +39 % in a mild one, trajectory 12->10->8->4->12.
It is also the harness where a hardcoded group size once chased three
ghost blocks per block marker and invented 58 % loss -- parse the
group geometry from the descriptor, as the firmware does.

## On the stand

Drive the consoles with `drive-boards`; broadcasts are long.

```
bcast [-r <rung>] <text>          # -r 0 = the beacon-to-strangers case
bcastfile [-r <rung>] <path>      # streamed from the host, no size cap
status                            # shows the feed progress and bc_free
```

With no `-r`, the board **negotiates first**: an idle link (or one
whose rung is a decayed memory, `req_age > 90 s`) holds the payload,
probes -- the probe *is* the capability exchange for a stranger --
and releases at the negotiated rung, logging rung, group geometry and
an air-time estimate. Held-then-released is the expected log, not a
fault. Give it 180 s before concluding the peer is deaf.

Reference clean runs, for comparison rather than nostalgia:

| run | result |
|---|---|
| 73362 B bcastfile, rung 12 | byte-identical, 22.3 min, 3089 frames, 0 lost, one start event |
| 8192 B with block stats | 3/3 stats windows, receiver asks rung 12, sender re-rungs 8->12 |
| 26-byte `bcast -r 0` to a board that never exchanged a frame | decoded at +16.2 dB (EXTREME is the only mode an idle station keeps active) |
| group loss rate at rung 4 | 38/39 groups over a campaign; a loss is whole-group |

## Failure signatures measured here

| symptom | cause |
|---|---|
| nothing on the air, `tx_frames 0` through the hold | the probe was swallowed by poll_tx's early-out gate (must pass `caps_kick`/`caps_reply_due`) |
| receiver asks rung 0 into a strong stream | ask derived from the chat ladder instead of the stream's own SNR |
| stream dies right after a stats window | the reply did not fit the window (below rung 4 the receiver must stay silent) or the station keyed a leftover chat frame into it |
| sender never sees a reply | its own `follow_rung` mask excluded the broadcast mode, or the receivers were not rearmed after its own transmission |
| received file longer than sent | a previous incomplete stream was never closed -- a zero-length EOS closes it |
| "host stopped feeding" mid-transfer | the feed timeout counted chunk gaps; at rung 8 a healthy host's next chunk is ~27 s out. It counts source starvation only |
| receiver stores far more than one chunk's worth | console pump missing its `break` after the last chunk |

Invariants (group size a power of two, EOB as a dataless SYNC, one
start event per stream, head loss counted from seq 0, stats bit in the
capability record) are in CLAUDE.md; the narrative with the numbers is
report Section 12.12.
