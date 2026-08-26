# C port of the fixed-point model

Executes [docs/c-port-plan.md](../docs/c-port-plan.md). Portable C99, no
malloc, no float on the signal path; every module is validated bit-exactly
(memcmp, no tolerances) against golden vectors generated from the Python
fixed-point model.

## Build & test

```bash
make            # regenerates vectors if needed, builds, runs all suites
make distclean  # also removes generated ROM/vector headers
```

`gen_vectors.py` (run via `../venv/bin/python`) dumps the ROM tables
(`src/rom_tables.h`) and golden vectors (`tests/test_vectors.h`) from the
Python model.

## Status

| Plan step | Module(s) | State |
|---|---|---|
| 1. primitives | `fxp.h`, `fft.c`, `dsp.c` | **done** — 11/11 bit-exact |
| 2. bit pipeline | `bits.c`, `conv.c`, `packets.c` | **done** — 16/16 bit-exact (LDPC pending) |
| 3. TX | `tx.c` | **done** — 5/5 full frames bit-exact (all modes, BPSK/QPSK/QAM16); **gate G1 passed** |
| 4. RX demod | `rx_demod.c` | **done** — 6/6 genie-synced decodes incl. noisy −5 dB/CFO/multipath; gated coarse search, tracker, all three modulations, per-mod quantizers |
| 5. RX detect | `rx_detect.c` + `rxd_receive()` | **done** — 6/6 full receptions; detection (start, cfo_word) integer-exact vs Python, all modes, clean + noisy |
| 5b. RX extras | `ldpc.c`, HARQ, calibrate, SNR estimator | **done** — LDPC ver=2 TX+RX, chase combining (calibrated LLR streams hash-exact), alpha fit + reliability ROM, SNR estimate double-exact |
| 5c. MCU streaming | `rx_stream.c` | **done** — ring-buffer state machine (SEARCH → ZC → HEADER → DATA), streaming Hilbert, per-block tone summaries + causal peak-commit; 6/6 corpus cases in arbitrary chunk sizes, clean cases land bit-identical to frame-at-once |
| 5d. measurement harness | `bench/replay.c`, `make bench`/`make armsize` | **done** — [FEASIBILITY.md](FEASIBILITY.md): ≤1 % of one x86 core worst case; G2/G3 **pass on M7-class**, M4 fits NORMAL(+ROBUST); flash ≈ 57 KB (preambles stored as periodic blocks 183 KB → 1.3 KB; single-table trig: NCO_SIN + twiddles are index-arithmetic views of NCO_COS) |
| 6. link layer | `link.c`, `station.c` | **done** — LC word golden-exact (incl. half-even rounding), controller parity vs a Python-recorded op trace, and a C-only two-station simplex session over the C fixed PHY: all messages bit-exact, ladder climbs from EXTREME bootstrap to QAM16 rungs in 46 virtual s |

**Protocol extension beyond the Python reference — burst ARQ**
(`station.c`, `burst_window >= 2`): bulk transfers switch from
stop-and-wait to selective-repeat — up to `burst_window` back-to-back
frames, then one bitmap acknowledgment (NACK-by-omission). Frame types
ride in the two flag combinations impossible in the legacy protocol
(`NO_DATA|PRIO` = burst data, `NO_DATA|LAST` = burst ack); fragments are
a uniform per-transfer size with a 3-byte index/total/frag-size
sub-header, the idle LC `seq` field carries the transfer id, and the
extension engages only at NORMAL rungs and above. Off by default
(`burst_window = 0`) — the legacy protocol and all parity tests are
untouched. Python backport: open thread.

**Protocol extension — extended (variable-size) frames**
(`PKT_TYP_EXT_DATA`, unused header `typ` codepoint 5): the article's
header `len` field counts packet *bits* (255 max → 27-byte payloads), so
at the top rungs the fixed preamble+header air time (~0.64 s NORMAL)
dwarfs the ~0.2 s of data it fronts. EXT frames reinterpret `len` as
payload *bytes* — up to 255-byte payloads (2076-bit packets) behind one
preamble. Conv-FEC only (LDPC K=256 is too small); rejected at TX build
and RX header stage for ver=2. Burst ARQ picks its fragment size from
the engage-time rung (`burst_frag_size_for_rung`: 200 B at rung ≥ 10,
100 B at ≥ 7, else 25) and carries big fragments in EXT frames —
measured: 250 B in 3 frames vs 20 for stop-and-wait; the demo's 5 KB
file drops from 211 to 45 transmitted frames. Costs that an MCU build
without EXT can drop back down: `CONV_MAX_STEPS` 272 → 2112 (Viterbi
dep ~50 KB + traceback ~133 KB), `MAX_LLRS` 1024 → 8192.

**Streamed bursts** (`tx_build_burst` / `rxd_receive_burst`): the other
half of the same amortization argument as EXT frames. EXT makes one frame
carry more; streaming makes N frames share one preamble and one header —
`[preamble][header][blk 0][ZC][blk 1]…`, with the preamble's own ZC block
re-emitted every `resync_every` blocks to refresh timing and residual CFO.
All blocks must share type, size, modulation and rate (that is what lets a
single header describe them) and the block count is not in the waveform —
the caller signals it. Measured on the float chain: **1.73× for 20 × 27-byte
NORMAL packets at 0.08 dB** (`experiments/stream_mode.py`); ~0.35 dB at
EXTREME, which is why the recommendation is NORMAL rungs only. Conv-FEC
only. C burst waveforms are bit-exact against the fixed model
(`TX_BURST_*` vectors), and `rxd_receive_burst` decodes every block with
its ZC resyncs locked.

`station.c` uses it for the burst window (`burst_stream = 1`): a whole
selective-repeat window goes out as one transmission instead of
`burst_window` separate frames. The packets are byte-identical to
per-frame burst fragments — bit 7 of the sub-header's index byte is the
only new thing on the wire — so **a receiver without streaming support
still decodes the first block** of every burst as an ordinary fragment,
which is what makes the fallback safe rather than fatal. A stream carries
only full-size fragments; the short tail always travels as its own frame,
because the receiver learns the message length from that fragment's own
length. Measured in `test_link.c`: a 250-byte transfer at 25-byte
fragments takes **4 transmissions streamed vs 13 after fallback**, both
bit-exact.

Three ways back to per-frame bursts, each ending the transfer's
streaming for good (`ST_EV_BURST_SOFF` says which): the PHY refuses to
build (`ST_SOFF_BUILD`), two windows come back with a bitmap acking at
most one fragment (`ST_SOFF_NOACK` — the signature of a peer decoding
only block 0), or two streamed windows time out (`ST_SOFF_TIMEOUT`).

Only NOACK is a statement about the **peer**, and it is the only one
remembered across transfers (`peer_stream_ok`): a receiver that cannot
follow a stream will not learn to between transfers, so re-discovering
it costs two wasted windows every time. TIMEOUT and BUILD describe the
channel and the local buffers, and deliberately do NOT stick — measured,
a fading channel raises `ST_SOFF_TIMEOUT` against a peer that streams
perfectly well, and making that sticky would disable streaming for the
session on a capable link. Nor is the peer verdict permanent: a deep
fade can forge the NOACK signature, so streaming is re-probed after
`PEER_STREAM_RETRY` (8) transfers. The
ack request rides on the burst's **first** block as well as its last, so
a peer that cannot follow the stream still answers and the sender learns
by bitmap rather than by waiting out a timeout.

Two receivers, two continuations: the frame-at-once path re-runs the
recording through `rxd_receive_burst` (and re-locks on each ZC), while
the streaming receiver steps to the next block with
`rxs_continue_burst()` and steps *over* the ZC without re-locking — a
documented divergence that is benign because bursts are NORMAL-only
(`BURST_MIN_RUNG`) and an open-loop NORMAL stream holds far longer than
any burst lasts. No per-block HARQ yet (the float chain has it) — open
thread. RAM: the resync search window is sized for ROBUST (41.6 kB); an
EXTREME burst resyncs open loop rather than spend 164 kB, and
`n_resync_out` reports the shortfall.

**Burst window sized to the transfer** (`station.c`, `btx.win`): the
window is chosen once at engage as `min(operator ceiling, buffer cap,
fragment count)`, then clipped by an air-time cap
(`BURST_WIN_MAX_AIR_S`, 30 s) so one transmission can never outlast a
plausible fade — everything in a streamed window is exposed before any
of it is acked. Sizing it to the transfer is what makes a short transfer
cost exactly one acknowledgment, which is the LTP/CFDP "deferred NAK"
endpoint reached with a constant rather than new wire format. Measured
on a 14 KB file: window 4 → 14 transmissions, 8 → 9, 16 → **6**.

Resizing the window *during* a transfer was implemented and then
reverted on the evidence. Halving it on each streamed-window timeout
(instead of striking out of streaming) collapsed it 16→8→4→2→1 with no
path back — it only grew on a fully-acked window, which never happens in
a fade — and produced **196 transmissions with 72 timeouts against 134
and 9** for the strike-out path. When a fade is what breaks a burst, the
right answer is to stop streaming, not to stream less. On a clean
channel the two were identical (6 transmissions each), so the dynamic
half bought nothing anywhere.

**Adaptive reply timer** (`station.c`, RFC 6298 shape + Karn's rule): the
ack budget used to be a fixed `turnaround + timeout_margin` guess on top
of a computed air time, with the peer's reply rung *guessed* as "my
request minus 2". The budget now splits along what is knowable: the
reply's **air time stays computed exactly** (`estimate_air_time` — it
swings 40x across the ladder, so smoothing it would be nonsense), while
the **overhead** — peer turnaround, decode time, carrier-sense wait,
scheduling, and the error in that rung guess — is measured and smoothed
(srtt/rttvar, K=4). Karn's rule keeps ambiguous exchanges out of the
estimator, and a timeout doubles the timer until a clean exchange clears
it. Measured in `test_link.c`: a 2.30 s bootstrap guess converges to
0.40 s against a peer that really takes 0.40 s.

**External oscillator fine-tune endpoint** (`station_set_freq_trim` /
`station_freq_trim` / `station_freq_trim_total`): registers the LO
actuator (VCTCXO DAC, PLL word, CAT clarifier) that the AFC netting
drives from peer LC-word requests, clamped to a total-trim budget
(default ±150 Hz); `anchor=1` makes the station the frequency reference
(never auto-trims — exactly one side should anchor). `station_freq_trim`
is the operator's manual nudge through the same budget accounting; it
works on anchors too. The demo app wires a logging stub (its virtual
channel has no trimmable LO) plus a `tune <hz>` console command.

**Diagnostic endpoints** (`station_set_diag` + `ctl_diag`): every
internal state switch — TX/RX with rung and flags, timeouts with the
loss streak, rung changes annotated with the controller inputs that
caused them (losses, cap, peer request/report, request age), burst
engage/frag/ack/probe/complete — streams through an optional callback
(`station_diag_name` labels events); `ctl_diag` snapshots the whole
tx-rung decision. Zero cost when unset. These diagnostics found and
fixed a real bug: burst ack windows were systematically timing out
(the reply budget assumed a 1-byte reply at the requested rung, but the
bitmap ack comes at the peer's control rung), and each spurious timeout
poisoned rung offsets and walked `consecutive_losses` toward the >=4
hard rung-0 — the "fallback to rung 0 right after a file transfer".
Fix: a burst window's first ack miss is forgiven (probe still goes
out; only a repeated miss counts as a loss) and the reply budget is
sized for the actual bitmap at a conservative rung. The other visible
"slow start" cause — one-rung-per-90-s stale-request decay on an idle
channel — is intended controller behavior and is now legible in the
diagnostics (`req_age_s`).

**The port is complete**: the entire fixed-point TX and RX feature family
(both FEC families, BPSK/QPSK/16-QAM, all three modes, gated two-stage
frequency search, HARQ, calibrated LLRs, integer SNR estimator), the MCU
streaming architecture, the measurement harness, and the full link layer
(rate ladder, LC word, adaptation controller, QoS station with ARQ and
simplex access) — **79/79 tests**. The station takes a PHY-callback
interface, so the same `station.c` drives the C fixed PHY today and a
soundcard/DMA loop on a target tomorrow.

**Streaming architecture notes** (`rx_stream.c`): the tone stage is
necessarily causal — block exponents and the median floor are windowed
(a streaming receiver cannot know the whole capture's minimum exponent),
and detection commits on a local metric peak (3-block decline) instead of
a global argmax. On the corpus this lands on the identical tone anchor
for clean frames (bit-identical results downstream); under noise the
anchor can shift a few Hz, ZC re-anchors timing exactly, and the demod
tracker absorbs the residual. Memory: ONE shared raw int16 ring
(147456 samples, 288 KB) serves all instances — every receiver hears
the same audio, so concurrent instances write identical values, and
the analytic signal is reconstructed on extraction (Hilbert-on-read,
bit-identical to the former write-time FIR). The ring is sized from
the measured ZC re-anchor lookback (`rxs_ring_hwm`: 8510/32318/124478
samples incl. FIR history) — the tone stage is incremental, so the
ring never holds the whole preamble. Per-mode residue is just block
summaries + symbol scratch (8/35/69 KB). Full three-mode RX ≈ 400 KB
+ 53 KB decoders. Viterbi traceback is packed to 1 bit/state/step
(8× vs byte-per-state, bit-identical).
Per-block work: one FFT(B) + 2×33 mask dots + one windowed metric —
bounded and far inside a 10.7 ms block period. HARQ combining is
exposed on the frame-at-once API; a station integration would carry
stored LLRs across streaming events the same way.

Frame vectors compare by FNV-1a 64 hash over every int16 sample (an
EXTREME frame is 260k samples — embedding it whole would bloat the test
header) plus the first 64 samples verbatim for debuggability. LDPC (ver=2)
frames land with `ldpc.c`.

RX genie vectors are self-hosting: the clean frames are rebuilt by the
bit-exact C TX inside the test, so only the Python detection genie
(start, cfo_word) and the expected bits are dumped; the one noisy case
(−5 dB, +41 Hz CFO, article multipath) embeds its samples.

Width findings and porting traps: [WIDTHS.md](WIDTHS.md).
