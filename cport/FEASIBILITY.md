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

## Cortex-M7: MEASURED, on an STM32H743

The projection below used to assume "SMLAD-class, x2 overhead margin:
M7 @480 MHz = 400 MMAC/s". That assumption was the weakest line in this
document, and it was **3-13x optimistic**. `bench/armbench.c` now
measures it with the DWT cycle counter on the real part, loaded into RAM
over `tools/esp32-probe` (the target's flash is never written).

Cycle counts are clock-independent; the ms column assumes 400 MHz
(`RCC` on the part under test reads SWS=PLL1, DIVM1=5, DIVN1=160,
DIVP1=/2, i.e. **HSE x 16**).

| primitive | cycles | per unit | @400 MHz |
|---|---|---|---|
| `fft_bfp` 128-point | 91376 | 714 /bin | 0.228 ms |
| `ifft_fixed` 128-point | 84310 | 659 /bin | 0.211 ms |
| `hilbert_analytic` | 3970631 / 4096 | **969 /sample** | 9.93 ms/4096 |
| `nco_derotate` | 159763 / 4096 | **39 /sample** | 0.40 ms/4096 |
| `cordic_atan2` | 365 | 365 /call | 0.9 us |
| `conv_encode` 255 bits r1/3 | 19529 | 77 /bit | 0.049 ms |
| **`conv_decode` (Viterbi) 255 bits r1/3** | 2465972 | 9670 /bit | **6.17 ms** |
| `ldpc_encode` 128 bits | 7598 | 59 /bit | 0.019 ms |
| **`ldpc_decode_int` 128 bits, 60 iters** | 21885598 | — | **54.7 ms** |

### Where the ZC window lives does NOT matter

A correlation shaped like the real ZC scan (512-tap kernel, 4096
offsets, 8.4 M complex MACs), run over byte-identical data in each of
the three SRAMs the window could occupy:

| window in | D-cache on | D-cache off | MPU off | best cyc/MAC |
|---|---|---|---|---|
| DTCM | 3.27 | 3.27 | -- | **3.27** |
| AXI-SRAM | 7.52 | 7.52 | **3.27** | **3.27** |
| D2 SRAM1-3 | 3.27 | 8.52 | 3.27 | **3.27** |

**All three are identical -- 3.27 cycles per complex MAC, 147 MMAC/s at
480 MHz.** The scan advances one sample per offset and re-reads 512, so
reuse is ~99.8 % and the 16 kB D-cache absorbs it from any backing SRAM.
DTCM buys nothing here; the D2 row shows the cache doing the work
(2.6x when it is turned off).

**The AXI column is a trap, not a property of the part.** The firmware
resident on the test board leaves an MPU region over AXI-SRAM
(0x24000000, 512 kB) with TEX=0 C=1 B=0 **S=1** -- Normal, write-through,
*shareable*. The Cortex-M7 has no cache coherency unit, so a shareable
Normal region is effectively uncached, which is exactly what "the
D-cache wins 1.00x on AXI" measured. Disable that MPU and AXI matches
the others. STM32H7 projects mark AXI-SRAM shareable/non-cacheable
routinely, to sidestep DMA coherency -- so **an MPU misconfiguration
costs 2.3x on the acquisition hot path**, and is worth checking before
blaming the algorithm.

Re-scaling the projection with measured throughput (147 MMAC/s @480):

| Load | MMAC/s | old assumption (400 MMAC/s) | MEASURED |
|---|---|---|---|
| Continuous (Hilbert + tone detect + tracked demod) | ≤ 2.5 | <1 % | **1.7 %** |
| EXTREME first symbol, gated / full grid | 2.4 / 13 | <1 / 3 % | **1.6 / 8.8 %** |
| EXTREME ZC acquisition (amortized over 5.8 s) | ~40 | 10 % | **27 %** |
| NORMAL/ROBUST ZC acquisition | ≤ 4 | 1 % | **2.7 %** |

So the assumption was ~2.7x optimistic, and the load still fits: EXTREME
acquisition costs about a quarter of a 480 MHz M7, not a tenth.

**A proposal this killed.** An arena split -- detect's 125956-byte region
into DTCM, demod/decode left in AXI -- was designed and costed at
+126 kB before being measured. It buys nothing: the correlation runs at
DTCM speed from any SRAM once the MPU is right. It is not implemented,
and should not be.

Also checked and found NOT to matter: `.rodata` in ITCM, where data
reads could contend with instruction fetch (1.00x).

### The whole frame, on target

No longer analytic. `bench_frame()` pipes the streaming TRANSMITTER
straight into the streaming RECEIVER on the part -- `txs_pull` generates
a chunk untimed, `rxs_push` consumes it timed -- so nothing ever holds a
frame (at EXTREME that would be ~1 MB of int16). `g_raw` is placed in D2
by the linker script, the rest of `.bss` in AXI, code in ITCM.

An EXTREME BPSK 1/3 frame, 521984 samples = 43.5 s of audio, decoded
CRC-clean (`ev.type == 1`), `rxs_ring_miss` zero, and
**`rxs_ring_hwm` 124478 -- matching the host to the sample**:

| phase | samples | audio | Mcycles | @400 MHz | @480 MHz |
|---|---|---|---|---|---|
| tone search (block summaries) | 70144 | 5.85 s | 94.6 | 4.0 % | 3.4 % |
| **acquisition (commit + ZC lock)** | 60928 | 5.08 s | **4050.4** | **199 %** | **166 %** |
| demod: header + data symbols | 390912 | 32.58 s | 809.0 | 6.2 % | 5.2 % |
| **whole frame** | 521984 | 43.50 s | **4954.0** | **28.5 %** | **23.7 %** |

The first measurement of this said the acquisition burst does NOT run in
real time -- 8.44 s of CPU inside a 5.08 s window at 480 MHz, falling
3.36 s behind and catching up out of the ring, whose depth then had to
cover the reach-back AND the lag: 164810 needed against 147456. That
has since been fixed, by taking both of the reach-backs away (see
`rx_stream.h`): the lag-N residual is now accumulated per block instead
of re-reading the tone field, and the ZC scan is anchored at the tone
field's END instead of searching forward from `cs_abs`. Same frame, same
board, after:

| phase | before Mcyc | after Mcyc | change | @480 after |
|---|---|---|---|---|
| tone search | 94.6 | 103.6 | 1.10x | 3.7 % |
| **acquisition** | **4050.4** | **1018.0** | **0.25x** | 41.8 % |
| demod | 809.0 | 856.8 | 1.06x | 5.5 % |
| **whole frame** | **4954.0** | **1978.5** | **0.40x** | **9.5 %** |

The ZC scan searches 8225 offsets instead of 62497 at EXTREME (7.6x),
which is where most of the 4x comes from -- the rest of the acquisition
bucket is fixed cost the anchoring does not touch. Tone search pays 10 % more for the per-block lag
correlation -- 9 Mcycles to save 3032.

And the real-time problem is gone:

    ring capacity       81920 samples  (6.83 s)
    measured lookback   67134 samples  (5.59 s)   <- host AND target agree
    acquisition @480     2.12 s CPU for 5.08 s of audio -- it KEEPS UP,
                         with 2.4x headroom, so there is no lag to add
    needed = 67134  <  81920            fits, 14786 samples spare

### Does the narrower search cost sensitivity? No -- measured

Narrowing a search from 62497 candidate offsets to 8225 is exactly the
kind of change that trades sensitivity for cost without saying so, and
the C suite could not answer it (its only noisy case is NORMAL at -5 dB
against golden samples). `make zcab` now does: it builds BOTH arms from
one source (`-DZC_ANCHOR_LEGACY` restores the old anchor and window) and
runs them over byte-identical EXTREME waveforms -- same seed, same
Gaussian noise, same CFO -- so the comparison is paired.

Wide sweep, 60 trials/point, 8 points: **legacy 291/480 decoded,
anchored 291/480**. Identical.

At the knee, where the curve is steepest and any loss would show, 250
trials/point:

| SNR | legacy | anchored | delta |
|---|---|---|---|
| -11.5 dB | 223 (89 %) | 222 (89 %) | -1 |
| -12.0 dB | 195 (78 %) | 193 (77 %) | -2 |
| -12.5 dB | 138 (55 %) | 133 (53 %) | -5 |
| -13.0 dB | 58 (23 %) | 61 (24 %) | +3 |
| **total** | **614/1000** | **609/1000** | **-5** |

0.5 percentage points, with mixed-sign deltas. The curve falls ~43
points per dB through there, so that bounds any sensitivity difference
at **~0.01 dB** -- far below the ~0.5 dB effects this project treats as
real (the gated coarse search, the stream resync).

**Noise-only false alarms: 0 in 250 runs, both arms.** The original
detection thresholds were tuned against 0/20, so this is an order of
magnitude more evidence than the bar they were set at.

(The SNR here is referenced to the full-band RMS of the transmitted
waveform, which reads ~9 dB pessimistic against the article's
convention. It does not matter: what is being compared is two arms on
identical input, not an absolute sensitivity.)

The margin itself remains a tuned constant. `ZC_ANCHOR_MARGIN_BLK` = 8
blocks passes all 8 `test_stream` cases; 4 fails, and 16 fails too --
at 16 the anchor clamps to zero for NORMAL and the window re-admits
data symbols, which is a false-lock risk. So the value is not arbitrary
and should not be changed without re-running `make zcab`.

The lag-N change was verified separately, against the path it replaces,
on a real EXTREME preamble across +-120 Hz of coarse word: worst
disagreement 0.0018 Hz against a 93.75 Hz bin.

Caveats, so this is not over-read:
- The frame above is noiseless. A real channel spends longer in
  acquisition, retries, and re-locks.
- One receiver instance. `demoapp` runs three concurrently, one per
  mode, and the idle tone search (3.4 %) is paid by each.
- The MMAC/s primitive figures above are still what the per-stage
  analytic projections are scaled against.
- A pattern with NO reuse (streaming a buffer once) is a different
  regime and is genuinely bandwidth-bound; nothing here measures it.
- Cortex-M4 numbers remain projections; nothing was run on an M4.

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
| **Full station, all three modes, no EXT frames** | **61 KB** | **561 KB** |
| Full station, all three modes + EXT frames | 61 KB | 793 KB |

Where the station's RAM goes:

| Component | Size |
|---|---|
| Shared raw int16 ring (81920 samples, all instances) | 160 KB |
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
| cport defaults, no EXT frames | 574952 | yes, 434 KB spare |
| cport defaults + EXT frames | 811496 | yes, 203 KB spare |
| demoapp settings, no EXT frames | 645896 | yes, 365 KB spare |
| demoapp settings + EXT frames | 882440 | yes, 134 KB spare |

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

`ST_MSG_MAX` used to dominate because it was multiplied 42 times inside
`station_t`: `qdata[3][8][ST_MSG_MAX]`, `delivered[16][ST_MSG_MAX]` and
the two current messages -- 42 x 3840 = 161280 B for one 256 -> 4096
change. That sized RAM by the number of positions that COULD hold a
message rather than by how many can exist at once.

**Since pooled** (`ST_POOL_SLOTS`, station.h): the 42 positions now hold
a slot index, and the payloads share one store. Measured peak across
everything: **3** slots in the C suites, **6** on a 14162-byte file
transfer (sender side), **1** on the receiving side. The default is 12.

| Configuration | before | after | saved |
|---|---|---|---|
| cport defaults, no EXT | 706252 | 698804 | 7 KB |
| cport defaults + EXT | 942800 | 935348 | 7 KB |
| demoapp settings, no EXT | 892396 | 769748 | 120 KB |
| demoapp settings + EXT | 1128940 | 1006292 | **120 KB** |

The last row is the one that did not fit an H743VI and now does, with
13612 B to spare -- tight enough that it should be read as "possible",
not "comfortable".

Two properties worth keeping: a full store refuses `station_submit`
exactly as a full queue always did (and `station_pool_free` lets a
caller check before submitting a file in parts, so a transfer is refused
whole rather than half), and `pool_free` walks the free list to catch a
slot released twice -- which would hand one payload to two owners and
present as a corrupted message rather than a crash. `test_link` asserts
both counters are zero.

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

### Confirmed on silicon

DS12110's memory map (Table 7) covers the STM32H742xI/G only and defers
the H743 to RM0433, so the table above started as datasheet + reference
manual. It has since been READ OFF A LIVE PART over JTAG
(`tools/esp32-probe`), which is what the linker script actually needs:

| probe | result |
|---|---|
| `DBGMCU_IDC` @0x5C001000 | `0x20036450` -- DEV_ID 0x450, REV_ID 0x2003 (rev V) |
| flash size @0x1FF1E880 | `0x0800` = **2048 KB** (so H743/H753, not the 128 KB H750) |
| DTCM 0x2000 0000 | reads to 0x2001 FFFC -- **128 KB** |
| AXI-SRAM 0x2400 0000 | reads to 0x2407 FFFC; **0x2408 0000 faults** -- **512 KB**, so H743 and not the 384 KB H742 |
| D2 0x3000 0000 | reads at +0, +128K, +256K and +288K-4 -- SRAM1/2/3 are **one contiguous 288 KB block** |
| D3 0x3800 0000 | reads to 0x3800 FFFC -- **64 KB** |
| backup 0x3880 0000 | present |

Total data-capable: 128 + 512 + 288 + 64 + 4 = **996 KB**, matching the
1019904 B the placement above assumes, exactly.

The D2 contiguity mattered most: it is what lets a single 294912-byte
`g_raw` be placed there, and it was the one figure with no datasheet
backing. It is now measured.

One trap for anyone repeating this: **D2 and D3 SRAM are clock-gated and
default to OFF.** The part under test had `RCC_AHB2ENR = 0`, so every
read of 0x3000 0000 fails until SRAM1/2/3EN (bits 29-31) are set --
which reads as "the memory is not there" rather than "the clock is off".
Enable, probe, restore.

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

## Analog loopback: a frame through DAC -> wire -> ADC, on one board

First step towards two boards joined by an audio channel. A station is
half duplex and the transmitter shares the receiver's arena, so the
stand (`bench/analog_loop.c`, `make analogfw`) does not do both at once:
it generates a NORMAL QPSK 1/2 frame into a buffer, a 12 kHz timer plays
it to DAC1_OUT1 (PA4) while recording ADC1 channel 3 (PA6) sample for
sample, then the streaming receiver runs over the recording.
`bench/analog_loop_dump.py` pulls both buffers over JTAG and fits the
path. No DMA -- the timer ISR writes one DAC sample, starts one ADC
conversion (19.2 us, measured), stores it -- so there are no trigger
tables to get from a reference manual this project does not have.

Measured, PA4 shorted to PA6 with a jumper:

| | |
|---|---|
| TIM6 input clock | 199,999,720 Hz (ratio to DWT 0.500) |
| sample rate | 11,999.743 Hz, and ~12 kHz absolute to ~10 % by wall clock |
| ADC bring-up | LDO ready, calibrated, enabled; conversion 19.2 us |
| ISR worst case | 25.6 us of an 83 us period |
| **path gain** | **0.9992 (-0.01 dB)**, delay 0 samples, correlation 1.0000 |
| **loop SNR** | **61.9 dB** -- 12-bit DAC quantisation, as predicted |
| **decode** | **frame decoded, payload bit-exact, CFO 0.00 Hz** |

Unwired for comparison: correlation 0.47 at zero delay, -32 dB -- not
noise but crosstalk, the floating PA6 hearing 2.4 % of its neighbour
PA4, which showed the DAC producing the frame before any wire existed.

### The bug it found was not analog

With the path at 62 dB the first wired run still failed the header CRC,
locked 65 samples late at -94.07 Hz on a loop with ZERO actual carrier
offset (DAC and ADC share one clock). Reproduced on the host with the
clean digital waveform: a NORMAL frame whose tone field starts EXACTLY
on a 256-sample detection block (lead 0 or 512) failed in every
modulation; lead 700 decoded. Same on every receiver back to before the
acquisition changes, so not a regression -- a latent gap.

-94.07 Hz is the signature CLAUDE.md gives for a one-bin coarse miss
(46.875 Hz) whose lag-N residual wraps at its +-46.9 Hz boundary. The
frame-at-once detector resolves it with a second, lag-N/2 correlation.
**The streaming commit never had the second lag**: `tone_commit` used
lag-N alone. It now accumulates both lags per block and unwraps exactly
as `rx_residual_word_src` does.

Cost: 16 more bytes per block summary and one more CORDIC per commit.
Return: the block-aligned cases decode, the analog recording decodes,
and the EXTREME sweep on the same eight points went from 291/480 to
**324/480 decoded -- +6.9 %**, almost exactly the "~8 % of all
acquisitions" the one-bin miss was documented to cost the fixed detector.
The streaming receiver had been paying that all along. `test_stream` now
carries both block-aligned leads as self-consistency cases.

## Pending for exactness

On-target cycle counts (DWT->CYCCNT) to replace the MAC-scaling
assumptions; host perf counters were unavailable here, and QEMU is
functional but not cycle-accurate.

**Now unblocked**: `tools/esp32-probe` brings up JTAG against the real
part through an ESP32 bit-banger and the packaged OpenOCD. The core
halts, and OpenOCD reports 8 hardware breakpoints and 4 watchpoints --
enough to instrument the acquisition burst directly. The link runs at
42236 edges/s, which is slow for bulk flashing but irrelevant for
reading a cycle counter.
