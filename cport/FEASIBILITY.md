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

RAM (streaming architecture, as implemented). These are MEASURED on a
linked Cortex-M7 image (`arm-none-eabi-gcc -Os`, `--gc-sections`) that
references only the streaming APIs, not projected:

| Image | Flash | RAM (.bss) |
|---|---|---|
| Transmit only (`txs_open`/`txs_pull`) | 20 KB | **28 KB** |
| **Full station, all three modes** (streaming RX + TX + link layer) | **50 KB** | **1.04 MB** |

Where the station's RAM goes:

| Component | Size |
|---|---|
| Shared raw int16 ring (147456 samples, all instances) | 288 KB |
| Per-mode tone block summaries | 204 KB |
| Scratch arena (detect / demod / decode, unioned) | 153 KB |
| LLR buffers, 3 instances (int32) | 192 KB |
| Everything else (FEC scratch, station/link state, TX) | ~227 KB |

A caution about single-mode builds. **Rung 0 of the ladder is EXTREME**
(`LADDER_MODE[0] == 2`), and rung 0 is where both stations bootstrap,
where `ctl_tx_rung()` drops after four consecutive losses, and where
staleness decay lands. Rungs 1-3 are ROBUST. So a build that omits
EXTREME cannot acquire a link at all, and cannot recover one after a
fade -- which is exactly when EXTREME's 64x tiling earns its cost.
Per-mode sizing is therefore a CAPABILITY choice, not a memory knob, and
would need the bootstrap rung changed first. `RXS_RAW_RING_LEN` is still
overridable for a deployment that has made that decision.

How the figures got here (they were ~10 MB of `.bss` before):
- the transmitter generates its waveform on demand from one 128-sample
  IFFT tile instead of buffering a frame (4.3 MB -> 10 kB);
- detection pulls samples through `zc_src_t` instead of materialising
  the search window, and both lag correlations run off a 128-sample
  delay line, so detection memory follows the PREAMBLE rather than
  anything it searches (2.4 MB -> 220 kB);
- the call-scoped scratch of the detect, demod and decode phases shares
  one arena, sized by the largest phase rather than their sum
  (646 kB -> 153 kB). Note demod is the largest, not detect: `eval_hyp`'s
  derotation scratch is live *while* the symbol samples still are;
- LLRs are int32 (measured peak 1211062) and analytic samples are int32
  (measured peak 43077), halving both.

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
- **Taken since**: the scratch union, int32 LLRs and int32 analytic
  samples above. Still available: a quarter-wave NCO table (~6 KB
  flash), and dropping `PKT_TYP_EXT_DATA` support, which returns
  `MAX_LLRS` to 1024 and `CONV_MAX_STEPS` to 272 -- a FEATURE trade
  rather than a mode trade, worth roughly 170 KB across the LLR buffers
  and the arena.
- **Narrowing a type is where the bugs live.** Three defects this way:
  a `memcpy` with a hardcoded `sizeof(int64_t)` into a now-int32 array
  (loud), a `>>= sh` with `sh` up to 34 that is undefined on int32
  (UBSan only), and a 2x buffer overrun in `zc_arr_fetch` that the whole
  suite passed straight through (found by grep, confirmed by ASan). Use
  `sizeof(*ptr)`, and run the sanitizers after any width change.

(The host reference still carries multi-MB frame-at-once scratch. It is
not part of the streaming budget and `--gc-sections` drops it -- but only
if the build enables section GC, which is why `demoapp/Makefile` now
does. Without it the linker keeps whole objects and the buffers stay.)

## Target fit: STM32H723xG (chosen target)

Cortex-M7 @ 550 MHz, 1 MB flash, 564 KB RAM (128 KB DTCM + 64 KB ITCM
+ 320 KB AXI + 32 KB D2 + 16 KB D3 + 4 KB backup ≈ 496 KB usable for
data). CPU: the @480 MHz projections scale by 480/550 — EXTREME ZC
acquisition ≈ 9 % amortized, everything else < 3 %. Flash: the whole
57 KB stack fits in ITCM alone.

| Configuration (MEASURED, linked image) | RAM | Fits ~496 KB? |
|---|---|---|
| Transmit only | 28 KB | yes |
| All three modes, no EXT frames (`-DMAX_LLRS=1024`) | 834 KB | **no** |
| All three modes + EXT frames (as built) | 1.04 MB | **no** |

**This verdict is a correction.** The table previously read "all three
modes ≈ 453 KB, fits with ~43 KB to spare". That was an estimate, and
the linked image does not meet it. Two places the estimate was optimistic:

- it costed per-mode block summaries (8/35/69 KB, ~112 KB total), but
  `g_blk[RXS_MAX_INST][BLK_CAP]` gives EVERY instance the EXTREME block
  count -- 204 KB as built. Sizing each instance to its own mode's
  window (15 / 30 / 120 blocks) would recover ~116 KB and costs no
  capability, since an instance only ever scans its own mode. This is
  the clearest remaining win;
- it assumed a build without EXT frames. That is a real option and is
  now measurable (`-DMAX_LLRS=1024`), worth 236 KB.

Even with both, the floor is the 288 KB raw ring plus ~88 KB of
right-sized summaries plus the 153 KB arena: about 529 KB, still over
the H723's usable data RAM. Closing the rest means shrinking the arena
(its detect phase is EXTREME's ZC geometry) or accepting a larger part.
The ring itself is not negotiable while EXTREME is supported: its
measured lookback is 124478 samples.

What has NOT changed: flash is comfortable at 50 KB against 1 MB, and
the CPU projections stand.

## Verdicts

- **G2 (EXTREME ZC burst fits the cycle budget)**: **PASS on M7**
  (~10 % amortized). On M4 it is ~50 % during the 5.8 s acquisition —
  workable but tight; the planned FFT overlap-save correlation (~10×)
  brings it to ~5 % if EXTREME-on-M4 is wanted.
- **G3 (≤ 60 % load, RAM within target)**: load **PASS**; RAM
  **QUALIFIED** — see "Target fit" above. The measured three-mode image
  is 834 KB without EXT frames and 1.04 MB with, so it needs a ~1 MB
  part (STM32H743 class) rather than the H723xG named as the target.
  Single-mode figures are NOT a way out: rung 0 is EXTREME, so a build
  without it cannot bootstrap or recover a link at all. The path back to
  an H723 is right-sizing the per-instance block summaries (~116 KB) and
  the arena, not dropping modes.
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
