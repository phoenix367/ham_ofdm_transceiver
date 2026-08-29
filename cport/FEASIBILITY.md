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
linked Cortex-M7 image, not projected, and they are REPRODUCIBLE:

    make armmeas                                    # as built
    make armmeas ARMMEAS_DEFS=-DMAX_LLRS=1024       # no EXT frames
    make armmeas ARMMEAS_SRC='$(TXSRC)' \
         ARMMEAS_DEFS='-DARMMEAS_TX_ONLY -DOFDM_ARENA_BYTES=27000'

`bench/armmeas.c` is the reference main. It exists because these figures
depend entirely on which entry points an image REFERENCES -- every static
buffer here is a worst case and `--gc-sections` drops what is unreachable
-- so a number measured from an ad-hoc main is not a number anyone can
check. (The earlier 50 KB / 929 KB pair came from a main that is no
longer known; the flash figure differs from it mostly because that one
linked newlib's float formatter, which a station does not.)

| Image | Flash | RAM (.bss) |
|---|---|---|
| Transmit only (`txs_open`/`txs_pull`) | 24 KB | **27 KB** |
| **Full station, all three modes, no EXT frames** | **61 KB** | **690 KB** |
| Full station, all three modes + EXT frames | 61 KB | 921 KB |

Where the station's RAM goes:

| Component | Size |
|---|---|
| Shared raw int16 ring (147456 samples, all instances) | 288 KB |
| Scratch arena (RX detect/demod/decode + TX generator, unioned) | 128.5 KB |
| Tone summaries, each instance sized to its own mode | 93.5 KB |
| LLR buffers, 3 instances (int32) | 24 KB (192 KB with EXT frames) |
| Everything else (station/link state, FEC and LDPC scratch) | ~156 KB |

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
  (measured peak 43077), halving both;
- the TRANSMITTER shares that same arena (26992 B). A station is half
  duplex, so the streaming generator's state -- the one buffer here that
  is live across calls -- can never overlap a receive phase in time. The
  assumption is checked rather than trusted: receive entry points stamp
  an owner tag and `txs_pull` refuses to continue a generator whose state
  a receive phase walked over, reporting it through `txs_faulted()`
  (`test_tx` asserts both halves). It also un-penalises a transmit-only
  image, which no longer carries the receiver's arena size: 27 KB with
  `-DOFDM_ARENA_BYTES=27000`, and every region asserts its own fit at
  compile time so a too-small override fails the build by name.

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
| Transmit only | 27 KB | yes |
| All three modes, no EXT frames (`-DMAX_LLRS=1024`) | 690 KB | **no** |
| All three modes + EXT frames (as built) | 921 KB | **no** |

**This verdict is a correction.** The table previously read "all three
modes ≈ 453 KB, fits with ~43 KB to spare". That was an estimate, and
the linked image does not meet it. Two places the estimate was optimistic:

- it costed per-mode block summaries (8/35/69 KB, ~112 KB total), but
  `g_blk[RXS_MAX_INST][BLK_CAP]` gave EVERY instance the EXTREME block
  count -- 204 KB. **Since fixed**: each instance is sized to its own
  mode's window (caps 16 / 32 / 128 for 15 / 30 / 120 blocks), which
  recovered 113 KB at no cost in capability. It required keying the
  instance pool by mode rather than round-robin, so the slice and the
  window agree;
- it assumed a build without EXT frames. That is a real option and is
  now measurable (`-DMAX_LLRS=1024`), worth 236 KB.

Even with both taken, the floor is the 288 KB raw ring plus the 128.5 KB
arena plus 93.5 KB of summaries: 510 KB before anything else, against
~496 KB usable. The ring is not negotiable while EXTREME is supported
(measured lookback 124478 samples). Folding the transmitter into the
arena (above) is worth 26 KB and does not change that verdict: the three
floor terms are all receive-side, and the largest of them is set by how
far back an EXTREME acquisition must look, not by anything a buffer
policy can reach.

The arena is close to spent too, and the reason is worth recording: its
three phases are now within 6 KB of each other (detect 125956, demod
131584, decode 131072). A union costs the MAXIMUM, so shrinking any one
buys nothing until all three come down together. Streaming eval_hyp's
derotation per tile, for instance, would cut demod to 65792 and move the
arena by 512 bytes, because decode immediately binds. The slide window
inside detect is likewise at its floor: the scan's live span IS
preamble_len+1 = 8193 samples and it holds 10240 (that plus one slide
block); a smaller block trades RAM for 8x the memmove traffic.

So an H743-class part, unless a deployment drops EXT frames AND the
ring shrinks with a lower bootstrap mode.

What has NOT changed: flash is comfortable at 62 KB against 1 MB, and
the CPU projections stand.

## Target fit: STM32H743VI -- and it can run entirely out of RAM

Checked against DS12110 Rev 11 (LQFP100, `I` = 2 MB flash). The RAM is
1060 KB but it is SEVEN separate blocks, which is what decides the
answer -- the totals were never the hard part:

| Block | Size | Base | Notes |
|---|---|---|---|
| ITCM | 64 KB | 0x0000 0000 | 0 wait state, 64-bit fetch (DS §3.3.2) |
| DTCM | 128 KB | 0x2000 0000 | 0 wait state, 2x 64 KB on 2x32-bit ports |
| AXI-SRAM | 512 KB | 0x2400 0000 | D1; the only block > 288 KB |
| SRAM1+2+3 | 288 KB | 0x3000 0000 | D2, contiguous |
| SRAM4 | 64 KB | 0x3800 0000 | D3 |
| Backup | 4 KB | 0x3880 0000 | retained in Standby/VBAT |

Data-capable total (all but ITCM): **1019904 B**. Measured `.bss`:

| Configuration | `.bss` | Fits? |
|---|---|---|
| cport defaults, no EXT frames | 706256 | yes, 306 KB spare |
| cport defaults + EXT frames | 942800 | yes, 75 KB spare |
| demoapp settings, no EXT frames | 892396 | yes, 125 KB spare |
| demoapp settings + EXT frames | 1128940 | **no**, over by 109 KB |

"cport defaults" throughout means the `#ifndef` values in the headers,
with no `-D` on the command line -- what `make armmeas` builds. The ones
that matter: `ST_MAX_MSGS` 8, `ST_MSG_MAX` 256, `ST_ASM_MAX` 4096,
`ST_DELIVERED_MAX` 16, `BURST_STREAM_MAX` 8, `BC_MAX_GROUP` 8,
`MAX_LLRS` 8192, `RXS_RAW_RING_LEN` 147456, `OFDM_ARENA_BYTES` 131584.

"demoapp settings" adds the three overrides in `demoapp/Makefile`, whose
measured costs are additive:

| override | delta .bss |
|---|---|
| `ST_MSG_MAX` 256 -> 4096 | **+161280** |
| `BURST_STREAM_MAX` 8 -> 16 | +16672 |
| `ST_ASM_MAX` 4096 -> 8192 | +8192 |

`ST_MSG_MAX` dominates because it is multiplied 42 times inside
`station_t`: `qdata[3][8][ST_MSG_MAX]`, `delivered[16][ST_MSG_MAX]`, and
the two current messages. 42 x 3840 = 161280 exactly. So the knob that
decides whether the largest configuration fits is not a PHY parameter --
it is how big a message the link layer queues, times how many it keeps
in flight and remembers for duplicate suppression. Size it to the
application, or pool the queues instead of giving all 42 slots the worst
case.

**Code in RAM: yes.** The image is 62009 B against ITCM's 65536 -- 94.6 %
full, 3527 B spare. Only ~32 KB of that is instructions; the other
~29 KB is const tables (NCO_COS, the LDPC graph, preamble ROM blocks),
which can stay in flash and leave ITCM half empty. ITCM/DTCM are
volatile, so the boot path is flash -> MDMA copy -> jump (DS §3.3.2
names MDMA for exactly this; §3.4 also allows BOOT_ADDx to point
directly at any RAM address).

A placement that works, demoapp settings without EXT frames:

| Block | Used | Holds |
|---|---|---|
| ITCM | 62009 / 65536 | all code |
| DTCM | 130996 / 131072 | `g_blk`, FEC and LDPC scratch |
| AXI-SRAM | 466488 / 524288 | `ofdm_arena_store`, `g_st`, LLRs |
| D2 | 294912 / 294912 | `g_raw` |
| SRAM4 | 0 / 65536 | free -- stack, heap, codec DMA |

Two tight spots worth knowing before committing to a board:

- **`ofdm_arena_store` is 131584 B: 128 KB plus 512.** It therefore fits
  neither DTCM nor a single 128 KB D2 bank, and the hottest scratch in
  the receiver is pushed out of zero-wait-state memory into AXI. The 512
  is `CP_LEN` -- demod sizes the arena at 4*(CP_LEN + 64*FFT_BINS)*4.
  Streaming eval_hyp's derotation per tile would cut demod to 65792 and
  land the arena on 131072 exactly (decode binds), which is DTCM to the
  byte.
- **`g_raw` is 294912 B, which is all 288 KB of D2 with nothing left.**
  D2 is where the codec's DMA descriptors would naturally live. Either
  trim the ring (see `rx_stream.h` -- 288 KB is a detector timeout, not a
  frame length) or put it in AXI and give D2 to the peripherals.

One caveat about sourcing: this datasheet's memory map (Table 7) covers
the STM32H742xI/G only and defers the H743 map to RM0433. The H743 base
addresses above -- in particular SRAM1/2/3 being CONTIGUOUS across
0x3000 0000-0x3004 8000, which is what makes a single 288 KB `g_raw`
placeable there -- come from RM0433, not from DS12110. They are
consistent with the H742 bases in Table 7. Confirm against RM0433 before
writing a linker script.

## Verdicts

- **G2 (EXTREME ZC burst fits the cycle budget)**: **PASS on M7**
  (~10 % amortized). On M4 it is ~50 % during the 5.8 s acquisition —
  workable but tight; the planned FFT overlap-save correlation (~10×)
  brings it to ~5 % if EXTREME-on-M4 is wanted.
- **G3 (≤ 60 % load, RAM within target)**: load **PASS**; RAM
  **PASS on STM32H743VI**, fail on the H723xG originally named. The
  measured three-mode image is 690 KB without EXT frames and 921 KB
  with, against 996 KB of data-capable RAM on the H743VI -- which also
  takes the whole 62 KB image into its 64 KB ITCM, so the firmware can
  run entirely out of RAM. See "Target fit: STM32H743VI".
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
