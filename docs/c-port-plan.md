# Plan: porting the fixed-point model to pure C (DSP/MCU feasibility)

> **Status**: in execution — see [`cport/README.md`](../cport/README.md).
> Steps 1 (primitives), 2 (bit pipeline; LDPC pending), 3 (TX; gate G1
> passed) and 4 (RX demod: gated coarse search, tracker, channel
> estimation, BPSK/QPSK/16-QAM LLRs, per-modulation quantizers, conv
> decode), step 5's detection (`rx_detect.c`) and the RX extras
> (`ldpc.c`, HARQ chase combining, calibrated-LLR mode, integer SNR
> estimator) are all done, and the **MCU streaming architecture**
> (`rx_stream.c`: ring buffer, streaming Hilbert, per-block tone
> summaries with causal peak-commit, symbol-by-symbol demod) is
> validated — 58/58 tests; clean frames land bit-identical to the
> frame-at-once reference through the streaming path, and the corpus
> decodes in arbitrary chunk sizes. The measurement harness
> (`make bench`, `make armsize`) closed **gates G2 and G3** — see
> [`cport/FEASIBILITY.md`](../cport/FEASIBILITY.md): ≤1 % of one x86
> core worst case, M7-class passes everything with margin, M4 fits
> NORMAL(+ROBUST), and the harness's noise-prefixed sessions caught a
> real causal-acquisition bug the golden corpus missed. The link layer
> (`link.c`/`station.c`) completes the plan: LC word golden-exact,
> controller parity against a Python-recorded trace, and a C-only
> two-station session over the C fixed PHY climbing from the EXTREME
> bootstrap to the QAM16 rungs — **62/62 tests, every plan step done**.
> Remaining beyond the plan: on-target cycle counts.

Goal: a `libofdm` in portable C99 (no malloc, no float on the signal path),
bit-exact against the Python fixed-point model, plus a measurement harness
that answers "does it fit target X" with cycle and byte numbers instead of
guesses.

## 0. What the numbers already say (from the Python model)

Per-mode dimensions (12 kHz, int16 audio):

| | NORMAL | ROBUST | EXTREME |
|---|---|---|---|
| symbol | 544 smp / 45 ms | 2080 / 173 ms | 8224 / 685 ms |
| preamble | 4 384 smp / 0.37 s | 17 440 / 1.45 s | 69 664 / 5.81 s |
| 27-byte frame | 2.9 s | 11.0 s | 43.5 s |
| detect FFT | 256 | 512 | 512 |
| freq-search grid (1st symbol) | 7 | 23 | 275 |
| ZC kernel × groups × CFO hyps | 128c × 4 × 1 | 256c × 8 × 7 | 512c × 16 × 7 |

Back-of-envelope compute at real time (16×16→32 MACs):

- **Hilbert FIR**: 63 taps (type III → ~32 nonzero) × 12 kHz ≈ **0.4 MMAC/s**, continuous.
- **Tone detection**: one FFT(256/512) per block + mask sums ≈ **≤1 MMAC/s**, continuous while searching.
- **ZC timing search** (burst, once per detection): NORMAL ≈ 2 MMAC;
  EXTREME worst case ≈ 512c × 16 groups × ~10³ positions × 7 hyps ≈
  **~230 MMAC burst** (amortized over the 5.8 s preamble ≈ 40 MMAC/s).
  This is the #1 hotspot; see optimizations below.
- **Demod**: EXTREME first symbol, worst case (full 275-hyp grid) ≈
  9 MMAC in 0.69 s ≈ **13 MMAC/s** — but the gated two-stage search
  (implemented and A/B-verified lossless in the Python model) takes a
  quarter-length coarse pass + top-3 fine windows whenever the coarse
  contrast clears 2.25×, ≈ 414k derotated samples ≈ **5.5× less**; the
  full grid remains only as the low-margin fallback, so 13 MMAC/s is the
  provisioning number and ~2.4 MMAC/s the typical one. Tracked symbols
  (±2 grid) ≈ 1 MMAC/s; FFT(128) per symbol is noise. NORMAL/ROBUST ≪ this.
- **Decoders** (per frame, i.e. per 3–43 s): Viterbi 64-state ≈ 0.3 M ops;
  LDPC min-sum 60 it × 3072 edges ≈ 1 M ops. Trivial at these frame rates.

Memory (the model is frame-at-once; C must restream — see phase 4):

- Detection lookback ring (preamble + margin, I/Q int16): NORMAL ≈ 25 KB,
  ROBUST ≈ 80 KB, **EXTREME ≈ 300 KB**.
- One-symbol buffer for the first-symbol grid: EXTREME 8224 × 4 B ≈ 33 KB.
- Const tables: 4096-entry sine ROM (8 KB), twiddles, Q15 ZC kernels
  (≤4 KB), Hilbert taps, LDPC matrix indices ≈ **~25 KB flash**.
- State: Viterbi metrics+traceback ≈ 6 KB, LDPC messages ≈ 4 KB, HARQ LLR
  store ≤ 1.5 KB, channel-est/accumulators ≈ 2 KB.

**Preliminary verdict to be confirmed by the harness**: NORMAL+ROBUST fit a
Cortex-M4 @ ≥168 MHz with 128 KB RAM; the full three-mode receiver wants an
M7/H7-class MCU (≥400 KB RAM) or any DSP (C6000/SHARC: trivial). TX is
negligible everywhere (one IFFT(128) + FIR per symbol). The decisive
uncertainty is the EXTREME ZC search burst — hence gate G2 below.

## 1. Repository layout

```
cport/
├── src/            libofdm: one .c/.h pair per Python module
│   ├── fxp.h            sat/rounding-shift/cmul macros (header-only)
│   ├── fft.c            radix-2 DIT, per-stage >>1, BFP wrapper
│   ├── dsp.c            Hilbert FIR, NCO (32-bit phase word), CORDIC
│   ├── bits.c           CRC-8/16, scrambler, interleaver, puncturing
│   ├── conv.c           encoder + int Viterbi (6/8-bit LLRs)
│   ├── ldpc.c           accumulator encoder + int min-sum
│   ├── packets.c        Header/Beacon/Data pack/unpack, LC word
│   ├── tx.c             FixedTransmitter equivalent
│   ├── rx_detect.c      streaming tone + ZC detection state machine
│   ├── rx_demod.c       symbol demod, channel est, LLRs, SNR estimator
│   ├── rx.c             frame state machine (header→data→HARQ)
│   └── link.c/station.c control plane (straight C translation, no DSP)
├── tests/          golden-vector runners (see §3)
├── vectors/        generated by the Python model
└── bench/          cycle/memory measurement harness (§6)
```

C99, `-Wall -Wextra -Werror`, no heap, no VLA on the signal path; a single
`ofdm_config.h` selects modes compiled in (an M4 build can exclude EXTREME
and drop the big ring buffer).

## 2. Width audit (before writing C)

Python ints don't overflow; C ints do. One-time instrumentation pass on the
Python model: wrap the arithmetic hot points (FFT stages, ZC metric
numerators, energy sums, SNR-estimator moments, Viterbi metrics, BFP
exponent alignment) with max-|value| logging and run the full regression
corpus (`fixed_point.py`, all modes, ±SNR extremes, HARQ, the two demo
WAVs). Output: a table `op-site → required bits` checked into
`cport/WIDTHS.md`. Known from the model already: 10-bit SNR-estimator rows
keep every moment in int64; ZC metrics need the two ≫15 compensations; BFP
energies compare only after 2·Δexp alignment.

## 3. Bit-exactness strategy (the core discipline)

The Python fixed model is already the golden reference — reuse it:

1. Add a `--dump-taps` mode to the Python model that writes, for a given
   int16 WAV: analytic I/Q, detection decision (start, CFO word), per-symbol
   accumulated bins, LLR stream pre/post quantizer, decoded bits, SNR
   moments. Each as flat binary + JSON manifest into `cport/vectors/`.
2. Every C module gets a runner that consumes the stage input and must
   reproduce the stage output **bit-exactly** (memcmp, no tolerances —
   integer DSP has no excuse).
3. Corpus: the three per-mode clean WAVs, the sensitivity-edge WAVs
   (`make_wav.py` at −7/−12/−18 dB), both system-demo WAVs (fading, CFO,
   AFC trims, HARQ events, QAM16 frames), plus the deterministic HARQ
   complementary-erasure pair.
4. CI target: `make -C cport test` runs the whole matrix on the host.

## 4. The one real redesign: streaming RX

The Python receiver takes the whole capture as an array. A target build
must consume 12 kHz int16 in ISR/DMA blocks. Planned structure:

- **Block pipeline**: 128-sample hop. Hilbert FIR → I/Q pushed into the
  detection ring (sized per compiled-in modes, §0).
- **Detection state machine**: SEARCH (per-block spectrogram column +
  running tone-contrast over the mask grid) → TONE_HIT (refine CFO: block
  FFT peak + lag-N) → ZC_SEARCH (matched filter over the bounded window in
  the ring) → DEMOD.
- **Demod without frame buffering**: tile accumulation is a 128-bin complex
  accumulator per CFO hypothesis — 5 tracked hypotheses × 128 × 8 B ≈ 5 KB,
  regardless of tile factor. Only the *first* symbol (full grid) is
  two-passed from a buffered symbol (≤33 KB). Everything downstream
  (FFT(128), pilots, LLRs) runs once per symbol.
- **Decode task**: Viterbi/LDPC + CRC + HARQ store at frame end; SNR
  estimator moments accumulate during demod (they are per-symbol sums).
- Latency/RT budget rule: every per-block step must fit the 10.7 ms block
  period on the weakest target with the ZC burst amortized (bounded
  positions per block, resumable search state).

ZC-search optimizations (ordered, apply until G2 passes):
(a) early-abort on the normalized-metric threshold; (b) 2× decimated coarse
pass + fine pass around the peak; (c) FFT overlap-save correlation
(kernel 512 → FFT(1024): ~10× fewer MACs); (d) per-block amortization.

## 5. Porting order (each step lands with its golden test green)

1. `fxp.h` + `fft.c` + `dsp.c` (primitives; vectors from unit taps).
2. `bits.c` + `conv.c` + `ldpc.c` + `packets.c` (pure bit pipeline —
   integer-exact in Python today, so vectors are trivial).
3. `tx.c` — validates against the Python fixed TX waveform (bit-exact
   int16, not correlation).
4. `rx_demod.c` on genie-synced vectors (skip detection first).
5. `rx_detect.c` streaming state machine; full `rx.c`; HARQ; SNR estimator.
6. `link.c`/`station.c` + a `select()`-style event loop harness; replay the
   simplex-session scenario against the Python station for protocol parity.

Estimated effort: primitives+bits ~small; TX small; RX ≈ half the total
work (the streaming redesign); link layer mechanical.

## 6. Measurement harness (the actual goal)

- **Host profile**: `bench/replay.c` feeds the demo WAVs at ∞ speed;
  callgrind/perf per function → MAC counts to sanity-check §0.
- **Cycle-accurate**: build for Cortex-M with arm-none-eabi + run under
  QEMU (or Renode) with cycle counters; then one real board.
  Suggested reference targets:
  - Cortex-M4F @ 168 MHz (STM32F407-class, 192 KB RAM) — floor target,
    NORMAL+ROBUST build;
  - Cortex-M7 @ 480 MHz (STM32H743, 1 MB RAM) — full three-mode build;
  - optionally a C674x/SHARC eval to represent "real DSP" (expected ≫
    headroom, CMSIS-DSP → TI DSPLIB mapping is mechanical).
- **Report** (`cport/FEASIBILITY.md`): per target × mode: peak and average
  MCU load per pipeline stage, RAM/flash from the linker map, worst-case
  block-period overrun, margin vs. the 12 kHz deadline.
- Optimization backlog if needed: CMSIS-DSP kernels (`arm_fir_q15`,
  `arm_cfft_q15`, `arm_cmplx_mult_cmplx_q15` cover FIR/FFT/derotation),
  SMLAD dual-MAC intrinsics, the ZC FFT-correlation, and Q15 saturating
  arithmetic via SSAT — the model's conventions (Q15, per-stage FFT
  scaling, 32-bit phase words) were chosen to map 1:1 onto these.

## 7. Feasibility gates

- **G1** (after step 3): TX + bit pipeline bit-exact on host. Risk: none —
  abort criteria n/a.
- **G2** (after step 5, host profile): EXTREME ZC search fits the M7 cycle
  budget with optimizations (a–c). If not: EXTREME detection degrades
  gracefully (longer acquisition, decimated-only search) or EXTREME becomes
  a "big-target-only" feature — the mode ladder already tolerates absent
  modes.
- **G3** (after step 6): measured load ≤ 60 % on M7 for the full build,
  ≤ 60 % on M4 for the NORMAL+ROBUST build, RAM within target — else record
  the real numbers and the specific hotspot in `FEASIBILITY.md` and decide
  with data.

## Out of scope (explicitly)

Audio I/O drivers and CAT control (platform-specific), the float model, the
channel/RF simulators (host-only test equipment), and any RTL — the C port
is a *firmware* feasibility answer; the RTL path keeps verifying against
the Python model directly.
