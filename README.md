# OFDM Transceiver Prototype

Reproduction of the amateur-radio OFDM PHY layer described in the Habr article
[«Разработка цифрового радиолюбительского протокола на базе OFDM. PHY-уровень»](https://habr.com/ru/articles/1070804/)
by bashkirtsevich: an audio-band OFDM modem for SSB transceivers with FEC, packet
framing and a two-stage synchronization preamble, plus the article's air-channel
simulator and its BER/PER results.

**C port for DSP/MCU: [cport/](cport/README.md)** — portable C99, no
malloc, no float on the signal path, validated bit-exactly against the
Python model (golden-vector suites); MCU streaming receiver (one
shared raw sample ring, Hilbert-on-read), burst ARQ + extended frames,
streamed burst windows, diagnostic and oscillator fine-tune endpoints
([cport/FEASIBILITY.md](cport/FEASIBILITY.md)).

**Running on real hardware: two STM32H743 boards** — flash-resident
firmware (`cport/usb/usb_radio_main.c`) that is a complete station: a
USB modem device (vendor class, addressed by the chip's unique serial)
with the radio behind it — DAC out, ADC in at 12 kHz over a cross-wired
audio link between the boards. Full ARQ exchanges, SNR-adaptive rate
ladder, streamed bursts (8 blocks behind one preamble), and an 8 kB
file transfer measured byte-exact in ~4 minutes including bootstrap.
Programmed and debugged through an ESP32 JTAG probe daisy-chained
through both boards ([tools/esp32-probe/](tools/esp32-probe/README.md));
receiver robustness is host-regression-gated (`make -C cport robust`:
carrier-sense scenario suite + an input-abuse decode matrix), and the
integer SNR estimator carries a measured per-(mode, modulation)
calibration map so the ladder reads one truth whatever frame carried
the measurement.

**Real radio: [demoapp/sdr_driver.py](demoapp/README.md)** — the same
device interface over SSB on a HackRF One (or any SoapySDR device), with a
hardware-free loopback mode that runs the whole SSB/resampling/int8 path
so it can be tested and regression-checked without a radio.

**Interactive demo: [demoapp/](demoapp/README.md)** — console messenger
over a virtual HF channel (two station devices + config device), running
the C fixed-point stack end to end in real time, with an audible mode
(stereo output, one station's receiver per ear) and multi-part file
transfer over the burst protocol. The same console speaks USB
(`ofdm_console --usb`, `--list` to enumerate boards), becoming a
terminal onto a real board's own station -- including `bcast`, the
non-ARQ broadcast mode the firmware transmits and receives on its own
(nothing is acknowledged, and a beacon sent at EXTREME reaches a board
that has never been heard from). Stations open a bulk transfer with a
three-leg capability handshake, so streaming, window and message size
are declared by the peer rather than discovered by failing at them.

**Technical report:
[technical-report/OFDM_Transceiver_Technical_Report.pdf](technical-report/OFDM_Transceiver_Technical_Report.pdf)**
(100+ pages, committed; from the article reproduction through the C
port, on-target measurement on silicon, the modem as a USB device, the
two-board audio stand, streamed-burst debugging, receiver hardening,
the SNR calibration, broadcast on the boards and the capability
handshake — plus the measured transmit spectrum, drive-level
sweep and off-air decode from a HackRF One + tinySA Ultra) — rebuild with `make` inside
[technical-report/](technical-report/README.md).

**Full documentation with architecture diagrams: [docs/](docs/README.md)** —
[architecture](docs/architecture.md) · [PHY](docs/phy.md) ·
[drivers](docs/drivers.md) · [console](docs/console.md) ·
[USB protocol](docs/usb-protocol.md) · [modem protocol](docs/modem-protocol.md) ·
[link layer](docs/link.md) · [RF](docs/rf.md) ·
[fixed-point](docs/fixed-point.md) · [performance](docs/performance.md) ·
[experiments](docs/experiments.md)

## Setup

```bash
python3 -m venv venv
./venv/bin/pip install -r requirements.txt
```

## Layout

- `ofdm_phy/` — the PHY implementation
  - `ofdm.py` — OFDM modem: 12 kHz / 128-bin FFT, 300–2400 Hz band (23 subcarriers,
    93.75 Hz spacing), 7 Zadoff-Chu pilots, 25% cyclic prefix, 4× symbol tiling
    (~6 dB coherent gain); Newman tone + Zadoff-Chu preambles, CFO estimation,
    ZF/Wiener channel estimation, MMSE equalization, LLR output
  - `coding.py` — rate-1/3 convolutional code (K=7, polys 133/171/165 octal),
    puncturing to 1/2, 2/3, 3/4, soft-decision Viterbi (vectorized)
  - `packets.py` — Header / Beacon (Base38 callsign + QTH locator) / Data packets
  - `crc.py`, `interleaver.py`, `scrambler.py`, `papr.py` (clip-and-filter),
    `channel.py` (AWGN + multipath + CFO drift + BSC/BEC), `transceiver.py`
    (full TX/RX chains)
- `experiments/` — reproduction scripts (each report figure has a
  dedicated entry point that writes a JSON record of every parameter
  and per-point count next to its PNG)
- `results/` — generated tables, curves, figures and the JSON
  reproducibility records
- `cport/` — the C99 port (src, golden-vector tests, bench harness,
  `gen_vectors.py`); `make -C cport` regenerates vectors, builds and
  runs all suites
- `demoapp/` — interactive demo: virtual channel driver (Python) + two
  console station apps (C)
- `technical-report/` — LaTeX sources + the built PDF
- `docs/` — architecture documentation with diagrams

## Reproducing the article's results

```bash
# every worked example from the article (CRC values, Base38/QTH encodings,
# convolutional-code outputs at all 4 rates, interleaver/scrambler sequences,
# ZC ACF properties) — 44 bit-exact checks
./venv/bin/python experiments/verify_article.py

# Tables 1-2: Shannon-Hartley limits and protocol efficiency
./venv/bin/python experiments/shannon_limit.py

# end-to-end sanity: TX -> channel -> RX at various SNR/CFO
./venv/bin/python experiments/smoke_e2e.py

# Figures 23-24: BER/PER vs SNR over the article channel model
# (multipath [1,0,0.4,0,0,0.2], BSC 0.001, BEC 0.02, random CFO/timing)
./venv/bin/python experiments/ber_per_simulation.py

# on-air test simulation with SNR-adaptive modes: the channel degrades
# -5 -> -10 -> -17.5 dB, the TX picks the mode via select_mode(), and the RX
# decodes the stream blind, auto-detecting each frame's mode from its
# preamble root (article-style decoder log)
./venv/bin/python experiments/ota_demo.py

# ZC autocorrelation, TX spectrogram, constellations before/after equalization
./venv/bin/python experiments/figures.py

# adaptive link modes: PER vs SNR for NORMAL/ROBUST/EXTREME
./venv/bin/python experiments/adaptive_modes.py

# record a frame to WAV (12 kHz/16-bit, transceiver-ready) + a channel-impaired
# copy, then verify both by decoding the WAVs back
./venv/bin/python experiments/make_wav.py --mode EXTREME --text "CQ CQ de R9FEU"

# validate the fixed-point (RTL reference) model against the float model
./venv/bin/python experiments/fixed_point.py

# validate the RF layer: SSB mod/demod transparency, CFO emerging from
# per-station LO ppm errors, decode through fading RF chain
./venv/bin/python experiments/rf_channel.py

# record a complete link session to WAV over the full RF chain (negotiation
# at EXTREME -> speed selection -> transfer, with Rayleigh fading, noise and
# per-station LO errors), then blind-decode the session transcript
./venv/bin/python experiments/demo_wav.py

# whole-system test: two stations on one simplex frequency, QoS traffic both
# ways, mid-transfer fade, bit-exact delivery check
./venv/bin/python experiments/simplex_session.py
```

## AFC / frequency netting

The receiver measures the peer's carrier offset on every decoded frame
(`stats.cfo_hz`), so the link closes the loop: a 5-bit field in the repacked
LC word (`seq(2)|ack(2)|rung(4)|snr(4)|freq(5)|flags(3)`) carries "shift
your carrier by −X Hz" requests (±120 Hz per frame, 8 Hz steps, ±12 Hz
deadband), and the peer trims its shared reference (`StationRF.trim_hz` —
one reference per rig, so TX and RX move together, which is exactly why
asking the peer beats retuning locally). Corrections are applied at **half
gain**: both sides act on stale measurements, and full-gain corrections
would swap offsets each turn instead of converging.

**Guard rails** — the loop only observes the *differential* offset, so the
pair's common frequency would random-walk over a long session (out of the
channel slot, bystanders' passbands, or the rig's trim range). Two bounds:
`afc_max_trim_hz` (default ±150 Hz) hard-clamps each station's cumulative
trim — requests beyond the budget are partially honored or refused, the
peer's own headroom carries the rest, and any residual is left to the
modem's ±300 Hz tolerance; and `afc_anchor=True` designates a station (e.g.
the CQ caller) that never trims, pinning the absolute frequency entirely.

`experiments/afc_netting.py`: two worn-TCXO rigs on 14.2 MHz start **+284 Hz
apart** (near the modem's ±375 Hz limit) with opposing thermal drifts —
netted to within the deadband in ~5 correction exchanges (~104 s, EXTREME
bootstrap frames dominate), steady-state |CFO| ≤ 11 Hz under continuing
drift, 400/400 bytes delivered. Guard scenarios: default budgets net fully
(final |CFO| 3 Hz, trims clamped at −150/+144); deliberately tight ±40 Hz
budgets stop at their limits leaving a 222 Hz residual **by design** — the
link still delivers, riding the modem's CFO tolerance; an anchored A forces
B to do all the correcting (+292 Hz) and the pair's absolute frequency never
moves. Beyond robustness margin, a netted link could shrink the EXTREME
per-symbol frequency-search range (compute) and eases mask-grid detection at
the extremes.

## Front-end LLR calibration (the review's biggest single win)

`experiments/llr_calibration.py` measured the demodulator's LLR quality
against ground truth and found a SHAPE miscalibration, not a temperature
one: bits with |L| < 2 err 11% empirically vs the 39% their value implies —
the per-carrier EsN0 weighting over-spreads confidence and the ±20 clip
understates the top. A single temperature can't fix it (the moment fit is
dragged by the clipped mass); a **monotone reliability map** (each |L| bin →
log-odds of its empirical error rate) can, and it also caps LLRs at the
data-justified maximum, defusing confidently-wrong bits from BSC flips and
fades. Measured effect at the sensitivity edge (NORMAL BPSK ⅓, 40 trials):

| SNR | raw | recalibrated |
|---|---|---|
| −8 dB | 33/40 | 38/40 |
| −9 dB | 6/40 | **29/40** |

≈ **1.5–2 dB of sensitivity** — for the existing convolutional chain
(Viterbi is scale-invariant but not shape-invariant), and it finally lets
LDPC sum-product work (9/40 → 24/40 at −9). ROBUST/EXTREME are unaffected;
16-QAM regresses under the BPSK-trained map, so the map is gated to MU ≤ 2.
Default OFF in `Transceiver` (article-faithful reproduction preserved);
the link-layer station enables it (`llr_recal="auto"`). Per-frame temperature
fitting from the decoded header (96 known bits) rides alongside and keeps
HARQ combining weights consistent. The full ladder was re-swept with
recalibration enabled (`experiments/ladder_sweep.py`, 48 packets/point,
results/ladder_recal.json) and `link.py` carries the measured values: the
mid-ladder BPSK/QPSK rungs gained 0.7-1.4 dB (QPSK ⅓: −5.6 → −7.0; QPSK ½:
−4.3 → −5.3; the formerly interpolated ROBUST ½ and QPSK ⅓ rungs measured at
−11.8/−11.3), the accumulation-limited EXTREME/ROBUST ⅓ rungs moved <0.2 dB,
and the 16-QAM rungs (map gated off) re-measured 0.2-0.6 dB tighter than the
original coarse sweep. Note: ROBUST BPSK ⅓ (rung 1) is now dominated by
ROBUST BPSK ½ (faster and equally sensitive) — kept for ladder-index
stability, never selected.

**Reproducible calibration studies (300 frames/point, JSON records).**
The report's calibration story is backed by a suite of reproduction
scripts, each writing `results/<name>.json` with every parameter and
per-point decode count (non-default `--trials` get suffixed filenames so
smoke runs can't clobber the reference):

- `experiments/llr_shape.py` — the measured miscalibration shape
  (216k LLR samples; the empirical-reliability curve peaks at ≈7.5
  log-odds, independently validating the deployed map's 7.4 cap, and
  *decreases* beyond |L|≈9 into the clipped mass);
- `experiments/viterbi_recal.py` — raw vs temperature vs map on the
  Viterbi chain: temperature is bit-identical to raw at every point
  (asserted — Viterbi is scale-invariant), the map shifts the waterfall
  ≈1.5 dB (63→216/300 at −9 dB);
- `experiments/ldpc_recal.py` — min-sum vs exact sum-product, raw vs
  mapped: raw-LLR sum-product collapses ≈1.5–2 dB *below* min-sum
  (0/300 at −9 dB vs 112/300), the map fully rescues it, and calibrated
  min-sum ≡ calibrated sum-product;
- `experiments/fec_comparison.py` — both families calibrated: LDPC
  holds +0.3–0.4 dB over conv (193 vs 151/300 at −9.5 dB), so the
  front end — not the code — was the binding constraint;
- `experiments/extreme_recal.py` — the same study at the EXTREME edge:
  the NORMAL-trained map transfers only *below* the operating point
  (+0.5 dB at −20.5 dB, zero by −18 dB), and the code families reach
  parity (the deep edge is acquisition-limited);
- `experiments/qam16_recal.py` — the 16-QAM regression is
  **rate-dependent**: the map *helps* +0.4 dB at rate ⅓ but regresses
  −0.5 dB at the ladder rate ⅔ (so the MU ≤ 2 gate is right, for a
  rate reason), and calibrated min-sum *beats* calibrated sum-product
  here — min-sum is robust to residual miscalibration in a way the
  exact rule cannot be.

## HARQ, QoS preemption, LDPC and 16-QAM (review follow-ups)

- **Chase-combining HARQ** (`Transceiver.demod_frame(prev_data_llrs=...)`):
  a data-stage failure carries its LLRs in the `DemodError`; the station
  stores them and the next decode tries fresh-alone then fresh+stored, CRC-
  gated (the seq number is inside the payload, so combining is blind and
  safe). Measured: second-attempt success 15/20 vs 10/20 without, at
  −8.5 dB. Helps in the noise-limited regime; fade losses are mostly
  sync-level (no LLRs to combine).
- **QoS preemption**: control/interactive fragments preempt an in-progress
  bulk message at fragment boundaries (LC flag bit2 = stream id, two
  reassembly buffers) — an urgent message injected mid-bulk arrives ~23 s
  before the bulk completes instead of after it.
- **LDPC** (`ofdm_phy/ldpc.py`, `build_frame(fec="ldpc")`, header ver=2):
  rate-1/3 IRA, N=768/K=256, girth≥6, linear-time accumulator encoding,
  normalized min-sum (deliberately: min-sum is scale-invariant, and the
  modem's estimated/clipped LLRs cost exact sum-product several dB). AWGN:
  ~0.7 dB better than the K=7 conv code at BLER 10%. End-to-end on raw LLRs the gain
  compresses to ≈ parity — and the 300-frame calibrated re-measurement
  (`experiments/fec_comparison.py`) shows why: with the shape map on
  both chains the LDPC advantage materializes at +0.3–0.4 dB, and
  calibrated min-sum gives up nothing to exact sum-product. The
  remaining distance to a standards-grade code (~2 dB) is now
  attributable to the matrix itself (short block, hand-rolled IRA),
  with the decoder and front end exonerated by measurement. Default
  FEC remains convolutional.
- **16-QAM** (`QAM16Mapper`, Gray, max-log LLRs): ladder rungs 10-12 —
  706 bit/s @ +0.1 dB, 941 @ +2.4, 1059 @ +4.2 (PER≤10%) — filling the
  article's own "можно применить QAM" gap above 0 dB. Covered by the
  fixed-point model and the C port as well (integer max-log demapper,
  8-bit data-LLR quantizer).
- **Learned rung offsets** (`LinkController.note_outcome`): every
  ack/timeout nudges the effective sensitivity of the rung used
  (+0.7 dB on loss, −0.15 dB on success), correcting the static table
  per deployment; applied in the peer-report rung cap.

## RF layer (`ofdm_phy/rf.py`)

Models the transceiver chain the protocol rides on: audio → SSB (USB)
modulator with the TX station's LO → RF channel (Rayleigh fading via
`rayleigh_fading`, multipath, AWGN, BSC/BEC) → product detector with the RX
station's LO → audio. Each `StationRF` has ONE reference oscillator (shared
by TX and RX, as in a real rig) with a ppm error and thermal drift, so the
audio-band CFO between stations **emerges physically**:
`CFO = f_rf·(ppm_tx−ppm_rx)·1e-6 + Δdrift(t)` — validated to 0.1 Hz at
+99/−107/+284 Hz offsets (`experiments/rf_channel.py`). The AWGN calibration
keeps the audio-band SNR convention, so all measured sensitivities carry
over (9/10 decodes at −5 dB through the full RF chain). One derivation
gotcha lives in a comment: the product detector's `Re()` halves uncorrelated
noise but not the coherent signal, so the RF noise variance factor is 2, not
(RF_FS/2)/AUDIO_BW.

`experiments/demo_wav.py` records the system demo through this chain: A at
+6 ppm and B at −8 ppm on 7.1 MHz (A→B CFO ≈ +99.4 Hz, handled invisibly by
the protocol), and the WAV is the audio of a passive MONITOR receiver with
its own +2 ppm LO — A's frames appear at ≈+28 Hz and B's at ≈−71 Hz in the
blind-decoded transcript, with the thermal drift visible across the session.

## Fixed-point model for RTL (`ofdm_phy/fixed/`)

An integer-only twin of the modem, structured the way an FPGA/ASIC datapath
would be — the golden reference an RTL implementation verifies against:

- **Primitives**: Q15 arithmetic with round-half-up and saturation; radix-2
  DIT FFT with a Q15 twiddle ROM, per-stage scaling and a block-floating-point
  wrapper (59 dB SQNR); 63-tap FIR Hilbert transformer (61 dB); NCO with a
  4096-entry sine ROM (−67 dBc); CORDIC atan2/magnitude; integer soft-decision
  Viterbi (6-bit LLRs, add/subtract branch metrics).
- **RTL-oriented choices**: CFO is carried end-to-end as a 32-bit
  phase-increment word (never float Hz); channel estimation is division-free
  (ZC pilots have unit magnitude → estimate = rotation; pilot interpolation is
  a Q15 weight ROM); ZC correlation magnitude uses α-max+β-min/2; LLRs are
  matched-filter Re/Im(Y·conj(H)) with per-frame exponent alignment; the
  residual-frequency search is a slew-limited tracker (full grid on the first
  symbol, ±2 grid steps after).
- **Validation** (`experiments/fixed_point.py`): fixed TX correlates 0.99998
  with the float waveform; the full fixed/float TX×RX cross matrix decodes;
  NORMAL-mode PER is within ~0.5 dB of the float receiver; ROBUST and EXTREME
  modes decode at −11/−17 dB. The bit pipeline (CRC, FEC, interleaver,
  scrambler) is shared with the float package — it was already integer-exact.

The fixed RX covers the **full frame family**: LDPC frames (header ver=2)
via an integer min-sum kernel (α=0.75 as `x−(x>>2)`); 16-QAM via an integer
max-log demapper (per-symbol amplitude reference from the mean
matched-filter magnitude, one divider per symbol); **HARQ chase combining**
(the DemodError carries the integer LLRs, `receive(prev_data_llrs=...)`
combines, CRC-gated — validated with a deterministic complementary-erasure
test); and an optional **calibrated-LLR mode** (`calibrate=True`): integer
header-based temperature fit (one divider) + a 32-entry reliability ROM
measured on the fixed chain itself. Measured surprise: the default 6-bit
peak-normalized quantization already acts as a compressive recalibration —
the fixed RX **matches or beats the recalibrated float chain at −9/−10 dB**
(22 vs 20 and 11 vs 5 of 30) — so calibrate mode's value is the stable
cross-frame LLR scale that makes HARQ combining legitimate, not extra PER.
Also notable: the tiled chain survives ≤65% contiguous audio erasure (the
4× tiles keep partial symbols alive).

The fixed TX covers the full frame family as well: `build_frame(fec="ldpc")`
emits ver=2 frames (the LDPC accumulator encoding is already pure integer,
so only the header/codec switch was needed), and its Q15 constellations
cover BPSK/QPSK/16-QAM (Gray per axis, levels {±1,±3}/√10).

The fixed RX also produces an **integer data-aided SNR estimate**
(accumulator moments over the known header + decoded data bits, per-symbol
gain weighting so QSB fading isn't counted as noise, one log2 LUT, no
dividers; ≤1.5 dB error over −17…+8 dB) — which is what lets the whole
link layer run on the integer pipeline: `FixedPHY` plugs the fixed TX/RX
into `LinkStation(phy=...)`, and `experiments/demo_wav.py --phy fixed`
records the complete adaptive-QSO system demo (negotiation, speed
selection, AFC netting, HARQ) with every station and the monitor decoding
on fixed point end to end.

Known deltas vs the float model: tone-detection FFT is 256-point for NORMAL
(so the coarse CFO grid is fine enough for an unambiguous lag-N residual),
and the demodulator uses the frequency-search tracker for all tile factors
(the float model's polynomial phase fit is not RTL-friendly).

The integer SNR estimate carries a measured per-(mode, modulation)
**output map** (`FixedReceiver.SNR_MAP`, emitted into the C tables by
`gen_vectors.py`): the raw LLR-moment estimate rails at a per-combo
ceiling at high SNR, and the tiling-gain subtraction spreads those
ceilings up to 12 dB apart — EXTREME frames read +0.5 dB on a wire
NORMAL frames read at +12 dB, whipsawing the rate ladder. The map is
built from (integer, float) estimate pairs on identical waveforms, so
the integer twin reports what the float reference reads (±0.3 dB across
the measured grid).

## Adaptive link modes (beyond the article)

`ofdm_phy/modes.py` extends the waveform with SNR-adaptive modes that scale
the coherent-accumulation tile factor (and the preambles with it), trading
bitrate for sensitivity:

| mode | tiles | user rate (BPSK ⅓) | sensitivity (PER≤10%, recalibrated RX) |
|---|---|---|---|
| NORMAL | 4× | 118 bit/s | −7.6 dB (article design: ≈ −7 raw) |
| ROBUST | 16× | 31 bit/s | −11.7 dB |
| EXTREME | 64× | 7.8 bit/s | −17.9 dB |

Shannon capacity of the 2100 Hz band at −20 dB is ~30 bit/s; EXTREME runs at
22 bit/s channel rate (78% of capacity), so −20.7 dB is the theoretical wall;
the measured floor is −18 dB (PER 56% at −20, 19% at −19, 10% at −18).

What makes the low-SNR modes work (each necessary, found empirically):

- **Per-symbol residual-CFO hypothesis search** in the tiled demodulator
  (`_demodulate_freq_search`): long symbols (0.17–0.69 s) cannot tolerate
  even a few Hz of residual CFO across the accumulation window, and per-tile
  phase tracking is pure noise at −19 dB; the search spends the whole
  symbol's energy on the frequency decision instead.
- **Finer tone-detection FFT** (512-point blocks): 4× more tone energy per
  bin, and a 23.4 Hz coarse-CFO grid whose residual fits a plain lag-N
  estimate.
- **Group-coherent ZC correlation** (2–4 symbols per kernel, magnitudes
  summed across groups) with a small fractional-CFO hypothesis grid.
- **Per-mode ZC preamble roots** (17/19/21): the receiver cannot read the
  tile factor from the header (it needs the tile factor to demodulate the
  header), so the preamble itself identifies the mode —
  `Transceiver.demod_frame_auto` tries the modes in turn and a wrong-root
  matched filter simply does not lock.

`select_mode(snr_db)` implements the adaptation policy (mode + modulation +
coding rate for an expected SNR).

### Streamed bursts (`build_stream` / `demod_stream`)

Every frame pays for its own preamble and header — 0.637 s at NORMAL, 2.49 s
at ROBUST, 9.92 s at EXTREME — which is 24% of a full 27-byte frame and 74%
of a short one at the top rung. `Transceiver.build_stream` pays it once for a
whole burst instead:

```
[preamble][header][blk 0][ZC][blk 1][blk 2][blk 3][ZC][blk 4]…
```

All blocks share the packet type, size, modulation and code rate, so one
header describes them all; a ZC symbol every 4 blocks refreshes timing and
residual CFO. Measured against the same packets sent as independent frames
(`experiments/stream_mode.py`, article channel):

| | air time | goodput | sensitivity |
|---|---|---|---|
| 20 × 27 B, per-frame | 28.16 s | 153 bit/s | reference |
| 20 × 27 B, streamed | 16.23 s | 266 bit/s | 0.08 dB worse |

(300 bursts = 6000 blocks per SNR point, −5.5…−4.0 dB.) The dB is the price
of carrying one preamble-derived CFO estimate across the burst rather than
re-estimating per frame, and it grows with burst length — a 5-block EXTREME
burst measured ≈0.35 dB on a much smaller sample (200 blocks/point), where
1.21× is a poor trade. Three properties that
took measurement rather than reasoning to establish:

- **The ZC resync is not holding the stream together.** With resync disabled
  entirely a 24 s stream decodes at the same PER — the per-symbol frequency
  search and the pilot channel estimate already do all the tracking. The ZC
  earns its place on sample-clock offset (the 32-sample CP slips in ~133 s at
  20 ppm) and as a mid-burst re-entry point for a receiver that missed the
  opening preamble.
- **Failures do not cascade**: block offsets are deterministic, so a failed
  CRC costs one block, and its raw LLRs are kept in `BlockStats.llrs` for
  chase combining (measured 1/12 → 11/12 blocks on one retransmission).
- **Rate is frozen for the burst.** No per-block header means no mid-burst
  rate change, which is the real cost on a fading channel and the reason to
  keep bursts bounded.

The block count is not carried in the waveform — the link layer signals it,
or `demod_stream(n_blocks=None)` decodes until the samples run out.

### Link adaptation & QoS (`ofdm_phy/link.py`)

Two-station protocol layer (`experiments/link_adaptation.py` simulates it
over the real PHY): a 13-rung speed ladder (mode × modulation × FEC rate incl. the
16-QAM rungs, dominated rungs removed, sensitivities from the measured
recalibrated sweeps), a 20-bit link-control word packed into
`Data.reserved` (`seq(2)|ack(2)|req_rung(4)|snr(4)|freq(5)|flags(3)`)
with stop-and-wait ARQ,
and a per-direction, receiver-driven controller:

- modes are self-labeling at the PHY (preamble roots), so switching cannot
  deadlock — worst case both sides fall to rung 0 (EXTREME) and re-converge;
- upshift needs 2.5 dB margin on a fade-aware (low-percentile, age-windowed)
  SNR statistic; downshift at <1 dB margin (hysteresis band prevents
  ping-pong); transmit side falls back on consecutive losses (2 → −2 rungs,
  4 → rung 0) and decays stale requests;
- three feedback paths with different latencies: the peer's rung request,
  the peer's SNR report of *your* signal (caps the rung immediately when the
  peer goes deaf), and inbound silence (decays your own request — stale
  measurements are purged so they can't resurrect a high rung after a fade);
- QoS is margin/latency policy on top: control/interactive traffic rides one
  rung below bulk and caps payloads by air time (`max_payload_bytes`).

`ofdm_phy/station.py` packages the full station on top of the controller:
QoS priority queues, message segmentation/reassembly (LC flags: last-fragment
/ no-data), stop-and-wait ARQ, and **simplex channel access** — carrier
sense, listen windows sized from the expected reply's air time (a timeout on
a *busy* channel is not a loss: the peer may be replying at a slower rung),
and random backoff after timeouts. `experiments/simplex_session.py` is the
whole-system test: two stations on one shared frequency, half-duplex radios
deaf while transmitting, bidirectional QoS traffic, and a −14 dB fade in the
middle of the bulk transfer — all messages deliver bit-exact in QoS priority
order, the fade costs 4 timeouts and a fallback-and-recover cycle, and the
session terminates without deadlock (11–22 bit/s effective on the shared
channel depending on fade severity).

The controller-level simulation (asymmetric channel, −16 dB start, sudden −9.5 dB fade)
shows bootstrap at EXTREME, a fast climb to QPSK rungs, loss-driven fallback
within two frames of the fade, and recovery — 1290/1500 bytes delivered at
24 bit/s effective goodput including ACK overhead and turnarounds
(`results/link_adaptation.png`). Intrinsic limit worth knowing: once a
direction falls to EXTREME, the feedback loop period is bounded by its
25–45 s frame air times.

### Link budget: 10 W on 80 m with EXTREME mode

What range does the −17.5 dB operating point buy on the 80 m band (3.5 MHz)
with a 10 W transmitter?

**Sensitivity in radio terms.** Our SNR is referenced to the full 6 kHz audio
band, so EXTREME mode at −17.5 dB needs S/N₀ = −17.5 + 10·log₁₀(6000) ≈
**+20.3 dB-Hz**, i.e. ≈ **−13.7 dB in the ham-standard 2.5 kHz bandwidth** —
about 7 dB less sensitive than FT8 (−21 dB), ~20 dB better than SSB voice.

**Required signal at the receiver.** 80 m is externally noise-limited
(atmospheric + man-made noise, ITU-R P.372), so with N₀ = −174 dBm/Hz + Fa
and 0 dBi antennas (a low 80 m dipole) the budget from +40 dBm (10 W) is:

| RX environment | Fa | required RX power | max path loss |
|---|---|---|---|
| quiet rural, winter night | ~50 dB | −103.7 dBm | ~144 dB |
| typical rural night | ~58 dB | −95.7 dBm | ~136 dB |
| residential (QRM) | ~65 dB | −88.7 dBm | ~129 dB |
| summer night, storm static | ~72 dB | −81.7 dBm | ~122 dB |

**Against 3.5 MHz path losses:** NVIS 0–400 km ≈ 105–112 dB (closes with
25–35 dB margin anywhere); single F-hop 500–2000 km at night ≈ 115–125 dB
(closes comfortably rural, marginal residential); two hops 3000–5000 km ≈
125–135 dB (needs a quiet rural RX and good conditions). Daytime D-layer
absorption kills 3.5 MHz skywave → ground wave + weak NVIS only.

**Bottom line:**

- night, typical stations: **500–2000 km reliably** (NVIS + one hop);
- night, quiet rural RX, winter/gray line: **3000–5000 km realistic**,
  occasionally intercontinental (the same nights 10 W FT8 works DX, minus
  our 7 dB);
- day: **~100–300 km**, best near sunrise/sunset.

**At 100 W** (+10 dB, roughly the cost of one extra ionospheric hop): two-hop
DX (3000–5000 km) becomes routine at any rural station and possible into
residential noise; multi-hop 6000–10000+ km closes on quiet rural winter
nights and gray-line paths; even summer storm static leaves the whole
single-hop range workable. Daytime improves least (~200–500 km NVIS) since
D-layer absorption scales with the path. Equivalence: 100 W EXTREME ≈ 10 W
FT8 + 3 dB — and 10 W FT8 works worldwide on 80 m winter nights.

### Link budget: the same on 40 m (7 MHz)

The modem numbers are unchanged (S/N₀ = +20.3 dB-Hz, −13.7 dB in 2.5 kHz);
what changes is the band. Noise at 7 MHz runs ~8–10 dB below 3.5 MHz
(ITU-R P.372), fattening every budget:

| RX environment | Fa | required RX power | max path loss @ 10 W | @ 100 W |
|---|---|---|---|---|
| quiet rural, winter night | ~42 dB | −111.7 dBm | ~152 dB | ~162 dB |
| typical rural night | ~50 dB | −103.7 dBm | ~144 dB | ~154 dB |
| residential (QRM) | ~58 dB | −95.7 dBm | ~136 dB | ~146 dB |
| summer night, storm static | ~64 dB | −89.7 dBm | ~130 dB | ~140 dB |

Free-space loss is +6 dB vs 3.5 MHz, but D-layer absorption falls as ~1/f²,
so **daytime skywave works**: day one-hop/NVIS 300–1500 km ≈ 115–125 dB;
night single hop 1000–4000 km ≈ 122–128 dB; two hops 5000–8000 km ≈
132–140 dB; multi-hop/long path 10000+ km ≈ 140–150 dB.

Bottom line at 10 W: **300–1500 km all day**, night single-hop with 15–20 dB
margin, two-hop routine at rural sites, and **worldwide multi-hop closing at
quiet rural sites on winter nights** (equivalence: 10 W EXTREME on 40 m ≈
2 W FT8, which works DX there nightly). At 100 W worldwide multi-hop becomes
routine at typical rural stations and reachable even into residential noise.
40 m-specific caveats: a ~100–500 km night skip zone opens when foF2 drops
below 7 MHz (short night paths are better served by 80 m), and evening
broadcast/contest congestion acts as localized noise well above the P.372
averages.

Caveats (independent of TX power): the −17.5 dB point was measured on the
article's channel model, so plan real links around −14…−15 dB (~3 dB fading
margin); and EXTREME's 64× accumulation integrates coherently over 0.69 s
per symbol, which assumes ≲0.1 Hz Doppler spread — fine on quiet
mid-latitude night paths, but on disturbed/auroral paths (≥1 Hz spread) the
coherent gain collapses at any power and ROBUST mode (0.17 s symbols,
−11.5 dB, 31 bit/s) is the better choice — one more reason for adaptive mode
selection.

## Known deviations from the article

The article omits some implementation details; where guessing was required the
choices are documented in code:

- The Zadoff-Chu matched-filter detector selects the best *normalized*
  correlation metric over all positions (the raw-peak argmax locks onto tiled
  data symbols), and the composite detector restricts the ZC search to the
  window right after the tone preamble.
- The tone-preamble metric is an in-mask/out-of-mask power-contrast ratio
  (the plain energy fraction with a fixed 0.05 threshold is SNR-dependent and
  fails below ~0 dB).
- Preamble bins are scaled (×2 for ZC, ×√5.75 for tones) so the preamble has
  the same per-sample power as data symbols; without this the preamble region
  sits ~7 dB below the frame-average SNR and the claimed −9 dB sensitivity is
  unreachable.
- The Wiener channel estimator uses an exponential frequency-correlation model
  (correlation length 8 bins).
- The composite detector locks the ZC matched filter to the zero-CFO hypothesis
  (the tone stage already leaves a residual well below half a bin). Scanning
  m=±1 lets the ZC time-frequency ambiguity — a frequency-shifted replica
  correlates almost as well at a shifted time — win at low SNR, injecting a
  one-subcarrier (93.75 Hz) CFO error plus a ~30-sample timing error; this
  alone cost ~1 dB of sensitivity.
- The tone-preamble residual CFO comes from a full-length FFT peak over the
  first tone block (unambiguous over ±1.5 bins), refined by the lag-N phase
  estimate; a lag-N estimate alone is ambiguous modulo one subcarrier spacing.

An alternative preamble (`STFOFDMModem`, suggested by a Habr commenter) places
the tones every 8 bins (combs [8,16,24] / [4,12,20]), making the tone block
periodic with 16 samples so the residual CFO comes from a simple 802.11-style
delay-and-correlate estimate (lag-16 → lag-128), unambiguous over ±375 Hz.
`experiments/stf_vs_newman.py` shows it matches the article preamble's
detection rate, timing and CFO accuracy over the full ±300 Hz range, and the
full BER/PER sweep (`--modem stf`, results in `results/*_stf.*`) gives the
same −7.2 dB sensitivity, with all eight per-config sensitivities within
0.14 dB of the Newman preamble (`experiments/compare_stf_newman.py` overlays
the curves — every point agrees within 2σ binomial noise). The
article-faithful `FullOFDMModem` remains the default.
- LLR sign convention: positive = logical 1, consistent with the BPSK mapping
  table (the article's prose states the opposite of its own mapper).

Measured sensitivity (PER ≤ 10%, BPSK 1/3, article channel, 120 packets/point):
≈ −7.2 dB, vs ≈ −7.5 dB reported in the article. A genie-synchronized receiver
(true timing/CFO supplied) measures ≈ −7.6 dB, so the remaining ~0.3 dB is
residual synchronization loss at low SNR (occasional missed detections and
one-symbol timing slips), not the decode chain.
