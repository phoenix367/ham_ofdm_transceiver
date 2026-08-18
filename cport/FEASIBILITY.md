# DSP/MCU feasibility — measured (plan §6, gates G2/G3)

Harness: `make bench` (host timing of the streaming receiver + burst-stage
micro-benches), `make armsize` (Cortex-M7 code/flash from
`arm-none-eabi-gcc -mcpu=cortex-m7 -O2`). Host = one core of an
i5-8300H @ 2.3 GHz; hardware perf counters are unavailable in this
environment, so Cortex-M projections scale the **analytic MAC budgets**
(exact, from the dimensions) against DSP-extension throughput — the host
numbers confirm the hotspot ranking and functional correctness of every
scenario (all three bench sessions decode).

## Measured host load (streaming receiver, ms of CPU per second of audio)

| Mode | idle search | framed session | frame length |
|---|---|---|---|
| NORMAL | 2.6 | 2.9 | 2.9 s |
| ROBUST | 2.7 | 5.8 | 11.0 s |
| EXTREME | 2.8 | 9.2 | 43.5 s |

≤ 1 % of one 2.3 GHz core in the worst case; idle search ≈ 0.3 %.

## Burst stages (host)

| Stage | Time | Note |
|---|---|---|
| Hilbert | 24.4 Msamp/s | 0.5 ms per audio-second |
| Acquisition burst (EXTREME tone+ZC) | 295 ms | frame-at-once worst case; streaming amortizes per block |
| First symbol, gated coarse/fine | 0.9 ms | vs **4.9 ms** full 275-hyp grid (the gated search's 5.5× in vivo) |
| Tracked symbol (5 hyps) | 0.10 ms | per 685 ms EXTREME symbol |
| Viterbi, 255 bits | 0.16 ms | per frame |
| LDPC, non-converging 60 iters | 0.88 ms | worst case; converges in a few iters on good frames |
| TX EXTREME frame | 9.5 ms | for 20.9 s of audio |

## Cortex-M projection (analytic MACs vs DSP throughput)

Assumed sustained integer-MAC throughput (SMLAD-class, ×2 overhead
margin): Cortex-M4 @ 168 MHz ≈ 80 MMAC/s; Cortex-M7 @ 480 MHz ≈
400 MMAC/s. C674x/SHARC-class DSPs: ≥ 10× the M7 — omitted (trivial).

| Load | MMAC/s | M4 @168 | M7 @480 |
|---|---|---|---|
| Continuous (Hilbert + tone detect + tracked demod) | ≤ 2.5 | 3 % | <1 % |
| EXTREME first symbol, gated (typ.) / full grid (worst) | 2.4 / 13 | 3 / 16 % | <1 / 3 % |
| EXTREME ZC acquisition (amortized over 5.8 s preamble) | ~40 | **50 %** | 10 % |
| NORMAL/ROBUST ZC acquisition | ≤ 4 | 5 % | 1 % |
| Decoders (per frame) | ≪ 1 | — | — |

## Memory

Flash (measured, arm-none-eabi `-O2`; const tables count into .text):

| Object | Flash | Of which tables |
|---|---|---|
| tx.o | 3.5 KB | 1.3 KB preamble blocks (see below) |
| fft.o | 9.4 KB | **8 KB NCO_COS — the only trig table** (see below) |
| dsp.o | 0.9 KB | — |
| rx_detect.o | 10.5 KB | ZC kernels + masks |
| rx_stream.o + rx_demod.o | 16.9 KB | search-word tables |
| ldpc.o | 7.2 KB | 8 KB graph |
| conv.o + bits.o + packets.o | 2.1 KB | — |
| link.o + station.o | 7.2 KB | ladder ROM |
| **Total** | **≈ 57 KB** | all modes included |

Trig is one table: NCO_SIN and all six FFT twiddle arrays were exact
views of NCO_COS (sin[i] = cos[(i+3·N/4) mod N]; twiddle_n[k] = cos /
−sin at stride N/n), so they are read through index arithmetic instead
of stored — one add+mask beside the derotator's multiplies, one negate
per FFT butterfly; bench numbers unchanged. `gen_vectors.py` asserts
the identities at generation time and the golden-vector tests pin the
outputs bit-exact. fft.c owns the single definition (`extern` for other
objects — a static-in-header table would be duplicated per object). A
further quarter-wave fold (8 → 2 KB) is possible but adds mirror/sign
logic to the two hottest loops; not taken.

The preamble ROMs (formerly 183 KB, the dominant flash cost) are stored
as their unique periodic blocks: each mode's preamble is exactly a
32-sample tone-A block tiled 2T×4 times, a 64-sample tone-B block tiled
T×2 times, and one 128-sample ZC tile (whose tail doubles as the cyclic
prefix) repeated `sym_tile` times — 224 samples per mode, re-expanded by
`tx.c` with modulo indexing at zero RAM cost. `gen_vectors.py` asserts
block-periodicity and the TX golden-hash tests prove the reconstruction
bit-exact. This supersedes the earlier "synthesize EXTREME at init"
idea (better ratio, no synthesis code, no init time).

RAM (shared-raw-ring streaming architecture, as implemented):

| Component | Size |
|---|---|
| Shared raw int16 ring (147456 samples, all instances) | 288 KB |
| Per-mode block summaries + symbol scratch | 8 / 35 / 69 KB |
| Decoder state (Viterbi + LDPC messages) | 53 KB |
| **NORMAL only** | **≈ 100 KB** (ring may shrink to its 8.5 k lookback: ≈ 30 KB) |
| **NORMAL + ROBUST** | **≈ 140 KB** (ring sized to ROBUST: ≈ 120 KB) |
| **All three modes** | **≈ 453 KB** |

(`RXS_RAW_RING_LEN` is overridable at build time; a single-mode build
sizes it to that mode's measured lookback plus margin.)

Note: enabling the extended-frame protocol (`PKT_TYP_EXT_DATA`, 255-byte
payloads) grows the Viterbi buffers to ~42 KB (2112-step trellis:
25 KB int32 depuncture + 17 KB packed traceback) and the LLR scratch 8×
(~190 KB/instance as int64); a build without EXT frames keeps the
figures above (`CONV_MAX_STEPS` back to 272, `MAX_LLRS` to 1024).

Memory-shape notes (why the budgets are what they are):
- **Viterbi traceback is 1 bit per state per step** — the predecessor
  is `(s>>1) + (winner_bit << 5)`, so storing the byte-sized state was
  8× waste; packing is bit-identical (suite-verified). The depuncture
  buffer holds quantized LLRs (≤ ±254 with HARQ) in int32, not int64.
- **The ring only covers the ZC re-anchor lookback, not the preamble**:
  the tone stage is already incremental (512-sample blocks fold into
  ~540 B summaries; raw tone samples are never re-read). Measured
  deepest lookback (`rxs_ring_hwm`, whole corpus incl. noisy):
  8510 / 32318 / 124478 samples per mode (incl. 62 FIR history).
- **One shared raw int16 ring serves every instance** (implemented):
  all receivers listen to the same audio, so concurrent instances write
  identical values (idempotent); the analytic signal is reconstructed
  on extraction (Hilbert-on-read, bit-identical to write-time FIR --
  suite-verified). Ring = 147456 samples (288 KB, non-power-of-2:
  2^17 would leave 5 % margin over EXTREME's 124478). Bonus: streaming
  load DROPPED (idle 1.4-1.8 vs 2.6-2.8 ms/audio-s) because the FIR
  now runs only on consumed regions, not every sample per instance.
  Constraint: instances fed different audio must not be live
  concurrently (sequential reuse is fine); test_stream fails a case if
  `rxs_ring_hwm` ever exceeds capacity.
- Further headroom if ever needed: detection scratch (ZC segment) and
  decode scratch (Viterbi) are never live simultaneously — a union
  saves the smaller; LLR buffers fit int32; quarter-wave NCO saves
  6 KB flash. None taken — budgets already fit the target parts.

(The host reference's multi-MB `.bss` is oversized frame-at-once scratch,
not part of the streaming budget.)

## Target fit: STM32H723xG (chosen target)

Cortex-M7 @ 550 MHz, 1 MB flash, 564 KB RAM (128 KB DTCM + 64 KB ITCM
+ 320 KB AXI + 32 KB D2 + 16 KB D3 + 4 KB backup ≈ 496 KB usable for
data). CPU: the @480 MHz projections scale by 480/550 — EXTREME ZC
acquisition ≈ 9 % amortized, everything else < 3 %. Flash: the whole
57 KB stack fits in ITCM alone.

| Configuration | RAM | Fits? |
|---|---|---|
| NORMAL only | ≈ 125 KB | yes |
| NORMAL + EXT frames (int64 LLR scratch, as coded) | ≈ 357 KB | yes |
| NORMAL + ROBUST | ≈ 345 KB | yes |
| NORMAL + ROBUST + EXT (needs the int32-LLR trim) | ≈ 482 KB | tight |
| All three modes, shared raw ring (**implemented**) | ≈ 453 KB | **yes** |

The shared-ring architecture is now the only one: one raw 2 B/sample
ring (288 KB) serves all instances, Hilbert recomputed on extraction
(bit-identical, suite-verified), per-mode summaries/scratch (~112 KB)
+ decoders (~53 KB). The full three-mode station fits the H723 with
~43 KB to spare. Linker placement: Viterbi/FFT/symbol scratch in DTCM,
the ring in AXI SRAM.

## Verdicts

- **G2 (EXTREME ZC burst fits the cycle budget)**: **PASS on M7**
  (~10 % amortized). On M4 it is ~50 % during the 5.8 s acquisition —
  workable but tight; the planned FFT overlap-save correlation (~10×)
  brings it to ~5 % if EXTREME-on-M4 is wanted.
- **G3 (≤ 60 % load, RAM within target)**: **PASS on M7/H7-class**
  (≥ 400 KB RAM: full three-mode build ≈ 645 KB needs a 1 MB part such
  as STM32H743, or the raw-int16-ring variant at roughly half). **PASS
  on M4-class for NORMAL+ROBUST** (≈ 345 KB with LDPC — a 192 KB part
  fits NORMAL-only ≈ 125 KB, or NORMAL+ROBUST without LDPC ≈ 300 KB is
  still over: ROBUST needs a ≥ 384 KB part).
- The gated two-stage frequency search is confirmed in vivo: 0.9 ms vs
  4.9 ms per EXTREME first symbol on identical input.

## What the harness caught (why it exists)

The bench's noise-prefixed sessions exposed an acquisition bug the
zero-prefixed golden corpus never triggered: the causal tone stage
committed prematurely on partial tone overlap (the windowed floor cannot
reproduce the model's acausal whole-capture median). Fixed with a
causal floor (1 % of the window's maximum block power) plus a commit
rule that waits for the above-threshold region to end or the argmax to
stay stable for a full window span. All 58 golden tests and all bench
scenarios pass with the fix.

## Pending for exactness

On-target cycle counts (QEMU/board with DWT->CYCCNT) to replace the
MAC-scaling assumptions; host perf counters were unavailable here.
