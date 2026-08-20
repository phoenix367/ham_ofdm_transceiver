# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A Python reproduction of the amateur-radio OFDM PHY layer from
https://habr.com/ru/articles/1070804/ (audio-band OFDM over SSB): modem,
FEC, packet framing, air-channel simulator, and the article's BER/PER results.
Pure NumPy/SciPy, no build step, no packaging.

## Commands

Everything runs through the local venv (`./venv/bin/python`); install deps with
`./venv/bin/pip install -r requirements.txt`.

- `./venv/bin/python experiments/verify_article.py` — the test suite: 44 bit-exact
  checks against worked examples from the article (CRCs, Base38/QTH, conv-code
  outputs, scrambler/interleaver sequences). Run after any change to `ofdm_phy/`.
- `./venv/bin/python experiments/smoke_e2e.py` — end-to-end TX→channel→RX sanity
  (6 cases incl. CFO and the −6 dB article channel).
- `./venv/bin/python experiments/ber_per_simulation.py [--trials N]` — the main
  result (Figures 23-24), multiprocess sweep; ~2 min at 120 trials on 8 cores.
  Use `--trials 10` as a quick regression check of the whole chain.
- `./venv/bin/python experiments/ota_demo.py` — multi-packet stream decode demo.
- `./venv/bin/python experiments/fec_comparison.py`,
  `experiments/viterbi_recal.py`, `experiments/ldpc_recal.py`,
  `experiments/llr_shape.py`, `experiments/extreme_recal.py`,
  `experiments/qam16_recal.py`
  (all take `[--trials N]`) — reproduction
  entry points for the report's calibration figures (300
  trials/point reference;
  JSON record beside each PNG; non-default trial counts get suffixed
  filenames so smoke runs can't clobber the reference).
  `experiments/figures.py` has per-figure flags for the other report
  figures (`--llr-shape-only`, `--viterbi-recal-only`, etc.).

There is no linter configured. Scripts in `experiments/` insert the repo root
into `sys.path` themselves.

Full architecture documentation with mermaid diagrams lives in `docs/`
(index: `docs/README.md`) — keep it in sync when changing layer behavior.

## Architecture

Signal path (`ofdm_phy/`): `packets` (Header/Beacon/Data + CRC) → `coding`
(rate-1/3 conv. code K=7, punctured to 1/2·2/3·3/4, soft Viterbi) →
`interleaver` → `scrambler` → `ofdm` (mapper, pilots, CP, 4× tiling, preambles,
channel est., MMSE, LLRs) → `papr` clip-and-filter. `transceiver.Transceiver`
glues the whole chain (`build_frame` / `demod_frame`); `channel.simulate_channel`
is the impairment model. Class hierarchy in `ofdm.py`: `OFDMModem` →
`TiledOFDMModem` (4× repeat + phase-drift polyfit) → `FullOFDMModem` (Newman
tone preamble + two-stage detection).

Cross-module invariants that are easy to break:

- **LLR sign convention: positive = logical 1** (matches BPSK map bit1→+1).
  The Viterbi `EXPECTED_SYMS` (+1 ⇔ bit 1), `descramble` (sign flip), and hard
  decisions (`llr > 0`) all assume it. The article's prose claims the opposite
  of its own code; do not "fix" this back.
- TX applies interleave **then** scramble; RX must descramble **then**
  deinterleave (see `_encode_block`/`_decode_block`).
- FEC-coded blocks are zero-padded to whole OFDM symbols **before**
  interleaving; RX crops to `codec.calc_cc_elements(bits_count)` after
  deinterleaving.
- The header is always BPSK + rate 1/3 (6 tiled symbols = 93 coded bits); the
  header's `mod`/`spd`/`len` fields drive the data-block demod.
- Preamble bins are deliberately scaled (×2 ZC, ×√5.75 tones) to equalize
  per-sample power with data symbols — detection sensitivity depends on it.
- The composite detector calls `detect_zc_preamble(..., max_cfo=0)` on
  purpose: widening the ZC frequency scan re-introduces the ZC time-frequency
  ambiguity and costs ~1 dB of sensitivity at low SNR.
- `STFOFDMModem` is an equivalent-performance preamble variant (tones every
  8 bins, delay-and-correlate CFO); tone bins in any comb variant must share a
  common divisor d so the tone block is periodic with `fft_bins/d` samples —
  the residual-CFO lag must be a multiple of that period or the estimate
  decoheres. `FullOFDMModem` (article-faithful) stays the default.
- `ofdm_phy/fixed/` is the integer-only RTL reference model
  (`experiments/fixed_point.py` validates it against the float model — run it
  after touching either side). Its invariants: CFO is a 32-bit
  phase-increment word everywhere (`hz_to_phase_word` only at boundaries);
  the FFT carries 1/N scaling (per-stage >>1) and BFP exponents must be
  tracked through every energy/LLR comparison (scale ∝ 2^(2·exp)); the ZC
  metric's Q-scaling accounts for the two /2^15 factors from the Q15 kernel;
  the Hilbert FIR taps are negated vs scipy's remez convention. NORMAL-mode
  fixed detection uses a 256-point FFT (not 128) by design. The integer SNR
  estimator (`last_stats.snr_db`, feeds link adaptation via `FixedPHY`)
  gain-weights symbol rows by mean |LLR| — equal-weight pooling counts QSB
  fading as noise (measured 5–15 dB pessimistic on EXTREME frames) and
  stalls the rate ladder; its −7.2 dB calibration constant is measured, not
  derived. The frequency search (fixed: first symbol via `COARSE_GATE_Q4`;
  float: every symbol via `TiledOFDMModem.COARSE_GATE`) is two-stage behind
  a measured contrast gate; don't remove the full-grid fallback — the
  quarter-length coarse argmax alone costs ~0.5 dB at the EXTREME edge
  (A/B: `experiments/coarse_search_ab.py`, parity-asserted).
- `link.py` (controller/ladder/LC word) and `station.py` (full station: QoS
  queues, ARQ, simplex channel access) are the link layer;
  `experiments/simplex_session.py` is their system test. Invariants: seq
  numbers 0..7 are ALL legitimate — "nothing received yet" is
  `last_rx_seq is None`, never a magic value; a reply timeout only counts as
  a loss when the channel is idle (a busy channel means the peer may be
  answering at a slower rung); stations reply only to frames that carried
  data (no ack-of-ack loops).
- `modes.py` defines SNR-adaptive presets (NORMAL 4×/ROBUST 16×/EXTREME 64×
  tiling → ≈ −7/−13.5/−19 dB sensitivity). Constraints: `detect_fft_len` must
  divide the tone-field lengths (`newman_tile·fft_bins`); each mode's
  `zc_root` must stay distinct (the preamble root IS the mode signal for
  `demod_frame_auto`); modes with `sym_tile > 4` use the per-symbol
  frequency-search demodulator, whose `freq_range` must cover worst-case
  preamble CFO error (±25 Hz for EXTREME). Detection thresholds were tuned
  against noise-only false alarms (0/20) — don't lower them casually.
- QoS air-time payload caps must be judged on the **whole frame**, not on
  the payload's own transmission time: at EXTREME the fixed preamble+header
  is 16.8 s, above every QoS budget, so the rate-based cap alone fragmented
  a 22-byte message into six 5-byte frames (~4x the air time). `payload_cap`
  (`station.c`) / `_payload_cap` (`station.py`) return the full 27 bytes
  whenever `estimate_air_time(rung, 1) >= budget`; keep both twins in sync.
- Energy-based carrier sense (`demoapp/app.c`) cannot distinguish a long
  frame from a raised noise floor by level alone — the floor tracker drops
  instantly but climbs 0.05%/block, so a sharp SNR drop used to freeze
  carrier sense at BUSY for ~82 s (and a timeout on a busy channel is
  deliberately not a loss, so the ladder did not adapt either). The bound
  is `CS_REBASE_S` (60 s > the 38 s longest frame): sustained energy past
  it is re-baselined as the new floor.
- `demoapp/sdr_driver.py` presents the same socket devices as `driver.py`
  but over SSB on a real SDR; its `--selftest` and `--loopback` cover all
  the DSP without hardware. Two levels that are easy to get wrong: the TX
  drive (`--tx-ref`, the audio peak mapped to the DAC full scale — the C
  transmitter reaches int16 full scale, so anything lower clips against
  the int8 rail and nothing decodes) and the time scale (the SDR path is
  ~20x the virtual channel's cost, so ~2x is the ceiling, not 25x).
- `Header.PACKET_SIZE`-style sizes count CRC bits; `Header.len` is the size of
  the *data packet bits including its CRC* (e.g. 90 for a Beacon). C-port
  exception: `PKT_TYP_EXT_DATA` (typ=5, `cport/` only) reinterprets `len` as
  payload *bytes* (≤255-byte payloads, conv-FEC only) — use
  `PKT_BITS_FROM_HDR(typ, len)` whenever turning a decoded header into a bit
  count; burst-ARQ fragment size rides these frames and scales with rung.

Known intentional deviations from the article (normalized ZC detection metric,
tone-contrast Newman metric, preamble gain, Wiener estimator model) are listed
in README.md "Known deviations" — keep that list updated.
