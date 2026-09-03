# Push-to-talk voice over the radio

A browser front end that drives two boards: pick a transmit and a receive
station, warm the link, hold TRANSMIT and speak. The far station's audio is
written to a `.wav`, decoded **as it arrives** so the listener hears the
first words seconds in, not after the release.

Speech is LSCodec-25Hz at 250 bit/s carried over BROADCAST
(`PKT_TYP_BCAST`, ptype `BC_PT_LSCODEC_25`, non-ARQ): nothing is
acknowledged and nothing is retransmitted, because for speech a late
repeat is worth less than a gap.

    /mnt/data/lscodec/adapter/venv/bin/python host/webvoice/server.py
    # then open http://localhost:8080

On this stand that needs no environment at all. See the dependency note
for a fresh checkout.

## Dependencies

The **OFDM host code** (`host/ofdm_modem.py`) is imported by relative
path, so the app moves with the repo. The **LSCodec code** is vendored as
a submodule (`voice/third_party/LSCodec-Inference`) and located, together
with the checkpoints, by `voice/_lscodec.py` -- the one resolver the
whole voice pipeline shares. There is no `LSCODEC_HOME`.

Three things live outside git; on this stand every default already
resolves, so only a clone needs to set them:

- `git submodule update --init --depth 1 voice/third_party/LSCodec-Inference`
  for the codec code;
- `LSCODEC_CKPT` -- the checkpoint directory (~580 MB, default
  `/mnt/data/lscodec/adapter/ckpt`);
- `WAVLM_CKPT` -- WavLM-Large.pt (1.2 GB), used to derive the speaker
  prompt.

The models run on the **GPU** when there is one (`VOICE_DEVICE=cpu`
forces the CPU path, also the fallback with no CUDA device). WavLM stays
on the CPU deliberately: it runs once per session for the prompt.

## Streaming decode and profiles

Decoding is chunked and starts as tokens land; the assembly at the end
reuses what was streamed and vocodes only the tail. A chunk decoded in
isolation drifts from a one-shot decode (the vocoder normalises over
time), so each step is fed **left context** -- already-received tokens,
free in latency -- and optionally **lookahead**, which costs latency.
The three profiles trade one against the other (`?profile=`):

| profile    | chunk | lookahead | first audio |
|------------|-------|-----------|-------------|
| `live`     | 1 s   | 0 s       | ~3.6 s      |
| `balanced` | 1 s   | 1 s       | ~4.8 s      |
| `quality`  | 3 s   | 2 s       | ~8.5 s      |

A step must finish inside its chunk's duration or the decoder falls
behind speech: the table is chosen for that on both devices. The page's
selector is generated from the server's own `PROFILES` so it cannot drift
from what was measured.

## Invariants worth keeping

Depth for all of these is in the report (`technical-report`, the
"voice at 250 bit/s" section) and CLAUDE.md; the load-bearing points:

**Ingest is decoupled from encode.** `/api/audio` only queues; a worker
does resample, encode and send at its own pace. The encode used to run
inside the HTTP request, so a client posting fast (an AudioWorklet firing
per 128-sample render quantum -- 125 req/s) starved the pipeline to 0.17x
real time. `feed()` is serialized under a lock regardless: without it,
concurrent blocks race `k`/`pending`/`_tokcarry`, and in `_send_chunk`
`bc_open` read-before-write lets two threads both send a
non-continuation command that the board reads as a rival broadcast.

**`finish()` waits for the encoder, not just the queue.** The worker
swaps the queue out before processing it, so "queue empty" does not mean
"encoder idle". Closing the stream while a `feed()` is in flight let a
late chunk go out after the close -- a non-continuation command that
truncated the transmission (measured: 13 s delivered as 3 s). `finish()`
waits on queue-empty AND worker-idle, and `_send_chunk` refuses a
non-final chunk once a final one has gone out.

**Transmit has priority over the USB drain loops.** `_rx_loop`/`_tx_loop`
hold the device lock across a 50 ms poll; Python locks are not fair, so a
sender competing with two spinners starved (92.5% of the host's time
waiting for the lock). The drain loops stand aside for a pending send
(`_send_want`/`_drain_lock`).

**One reader per USB device.** `warmup()`/`settle()` read the mailbox the
drain loops fill, never the device directly -- two threads polling the
same event stream race for every event, and the loser drops it (a warm
link then reported "did NOT warm up" by scheduling luck alone).

**A session generation counter guards resets.** A decode or encode step
takes up to seconds; a reset during it must make its result inert, or
one session's audio lands in the next. `reset_session()` bumps `gen`
under the airlock; long steps capture it and discard if it moved.

**Warm before a stale transmit.** The station's rung memory decays with
peer silence, and a broadcast entering with a decayed rung is HELD by the
firmware while it re-probes -- 20 s of speech into a hold, then one batch.
`/api/start` warms first when the peer has been silent past the decay,
and the button says PREPARING until the link is open. `settle()` also
waits for the warm-up's own ARQ ack to finish keying, since the link is
half duplex.

**Byte-aligned groups make a lost group survivable.** The packer pads to
a byte, so only multiples of 4 tokens (40 bits = 5 bytes) are emitted and
the remainder carries; otherwise pad bits shift every later group and the
stream is noise after the first. With alignment, a lost group is a
one-second GAP the decoder resumes past -- which is what makes
non-acknowledged voice viable. Each recording keeps its sent and received
byte streams (`tx_*.bin` / `rx_*.bin`) so a loss can be located exactly.

## What is NOT the cause of group loss

An earlier version of this file credited the warm-up `settle()` with
eliminating clean-wire group loss (5 losses in 23 without it, 0 in 16
with). That was wrong, and it took a third measurement to see it: the
0.76% loss was a **receiver** defect -- the streaming detector committing
an isolated partial-overlap spike two blocks before the real preamble,
whose failed attempts then consumed it. It is fixed in firmware
(`rx_stream.c`, the `WEAK_COMMIT_X` commit gate; `make bcsoak` is the
regression) and is unrelated to the warm-up. `settle()` is still correct
on its own terms -- the warm-up did return before the ack finished -- but
it is not what closed the loss. The full three-answer story is in the
report.

## Capture note

Microphone capture uses the deprecated `createScriptProcessor`
deliberately: an `AudioWorklet` replacement (audio thread, immune to
main-thread stalls) was written and shipped and captured **nothing** --
every session came back flat silence -- and the fault is not visible
without a browser to step through. The server SAYS SO when a source is
silent, and the page shows a live input meter, so a broken capture is no
longer invisible until a recording of nothing comes back.
