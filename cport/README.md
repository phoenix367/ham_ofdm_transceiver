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
