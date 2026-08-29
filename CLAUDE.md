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
- `./venv/bin/python experiments/stream_mode.py [--trials N]` — streamed
  bursts vs per-frame preambles (delivery, goodput, fitted dB cost).
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
- Streaming (`build_stream`/`demod_stream`, `stream_layout`) amortizes the
  per-frame preamble+header over a burst. Invariants: every block must have
  the same packet type, size, mod and rate (one header describes all — the
  builder asserts it); the block COUNT is not in the waveform (the link layer
  signals it, or `n_blocks=None` decodes to the end of the samples); the ZC
  resync calls `detect_zc_preamble(..., max_cfo=0)` for the same reason the
  composite detector does, and a lock outside the ±`win` plausibility window
  must be discarded or a spurious correlation walks the stream off its grid.
  The resync is NOT what keeps the stream coherent (open-loop 24 s streams
  decode identically) — it is there for sample-clock drift and mid-burst
  re-entry. Costs 0.08 dB at NORMAL for 1.73x (measured, 6000 blocks/point)
  and ~0.35 dB at EXTREME for 1.21x (200 blocks/point); use it at NORMAL
  rungs, not at EXTREME. Three twins to keep in sync: float
  (`transceiver.py`), fixed (`fixed/tx.py` `build_stream`, `fixed/rx.py`
  `receive_stream`) and C (`tx_build_burst`/`rxd_receive_burst`, golden
  `TX_BURST_*` vectors). Conv-FEC only. The clip threshold is a
  WHOLE-waveform RMS, so a burst is not the concatenation of separately
  built frames — a 1-block burst with no resync is bit-identical to a
  frame, and both test suites assert it. In C the resync window buffer is
  sized for ROBUST on purpose (EXTREME would cost 164 kB of the 453 kB
  budget); an EXTREME burst resyncs open loop and `n_resync_out` shows it.
- Streamed burst windows (`station.c`, `burst_stream = 1`) put a whole
  selective-repeat window in one transmission. Invariants: the packets
  stay byte-identical to per-frame fragments (bit 7 of the sub-header
  index byte is the only marker), so a peer without streaming still
  decodes block 0 — that property IS the fallback; a stream carries only
  full-size fragments, because the receiver derives the message length
  from the last fragment's own length (`brx.last_len`); the ack request
  rides on the FIRST block as well as the last, so a peer that cannot
  follow the stream still replies and the sender falls back by bitmap
  (`ST_SOFF_NOACK`) instead of by timeout. Every timeout of a streamed
  window is a strike even the forgiven first one — forgiveness is about
  not poisoning the rate controller, not about the evidence. Two
  receivers continue a burst differently: `rxd_receive_burst` re-runs the
  recording and re-locks each ZC; `rxs_continue_burst` (streaming, what
  `demoapp/app.c` uses) steps over the ZC without re-locking. The
  streaming continuation MUST know where to stop — while stepping through
  blocks the receiver is not running the preamble detector, so chasing
  blocks that were never sent makes it deaf for one block-time each (a
  measured ~12 s hole that ate the peer's next burst and produced an
  endless retransmit loop). The stop signal is the ack-request bit on the
  burst's last block, "set AND not the first block of this stream"
  (the first carries it too, for peers that cannot stream), plus a
  consecutive-failure bound for when that last block is itself lost.
- `ST_SOFF_NOACK` is the ONLY streaming failure that is a peer property,
  and the only one remembered across transfers (`peer_stream_ok`,
  re-probed after `PEER_STREAM_RETRY`). `ST_SOFF_TIMEOUT`/`ST_SOFF_BUILD`
  describe the channel and local buffers and must stay per-transfer — a
  fading channel raises TIMEOUT against a peer that streams fine, so
  making it sticky would kill streaming on a capable link.
- The burst window (`btx.win`) is fixed at engage: min(ceiling, buffer
  cap, fragment count), clipped by `BURST_WIN_MAX_AIR_S`. Do NOT add
  dynamic resizing — it was implemented and reverted: halving the window
  per streamed-window timeout gave 196 tx / 72 timeouts on a fading
  channel vs 134 / 9 for striking out of streaming, because the window
  collapses to 1 and only grows on a fully-acked window (which never
  happens in a fade). A fade means stop streaming, not stream less.
- The reply timer (`station.c` `rto_*`) splits the budget: the reply's air
  time is computed exactly by `estimate_air_time` (it swings 40x across
  the ladder), and only the OVERHEAD is smoothed RFC-6298-style. Karn's
  rule gates the sampling (`rto_ambiguous` set on timeout) and a timeout
  doubles `rto_backoff` until a clean exchange resets it. Do not smooth
  the air-time term, and do not sample an exchange that followed a
  retransmission.
- File transfers are DEFLATEd whole before splitting into parts
  (`demoapp/app.c`, magic 0x02 vs 0x01) — per-part compression would
  throw the ratio away. Compression lives in the app, not `cport/`, so
  the C port stays dependency-free; an MCU build swaps zlib for
  miniz/heatshrink. Measured 2.82x on a 14 KB config file, which is
  ~1.4x fewer transmissions end to end (acks and the ladder bootstrap do
  not compress).
- The fixed detector's residual CFO uses TWO lags (`fixed/rx.py`
  `_detect_newman`, `cport/src/rx_detect.c` `rx_residual_word`) — keep
  them in step. The lag-N phase wraps beyond +-fs/2N = +-46.9 Hz, which
  is exactly ONE coarse bin, and the coarse mask-shift search is a 4%
  near-tie between adjacent bins; a one-bin miss then makes the residual
  wrap and land 93.75 Hz out, which cyclically shifts the ZC correlation
  ~34 samples past the 32-sample CP and kills the frame. Cost ~8% of ALL
  acquisitions (ARQ hid it by retransmitting). The lag-N/2 phase is
  unambiguous over +-fs/N and picks the right lag-N cycle. Do not remove
  it, and do not "fix" this in `_detect_zc` — the ZC stage was always
  answering correctly for the frequency it was given.
- Station message storage is POOLED (`ST_POOL_SLOTS`, `cport/`): the
  3x8 queue positions, the 16 delivered-log entries and the two
  current messages hold a slot index, and payloads share one store.
  Sizing by addressable positions instead cost 42x`ST_MSG_MAX`
  (161 kB at the demo app's 4096). Consequences to respect: never
  zero `delivered_n` by hand -- that strands slots, use
  `station_delivered_reset`; free a slot exactly once and set its
  handle to -1 (`msg_release` does both, and `pool_free` walks the
  free list to catch the rest); a caller submitting a message in
  several parts must check `station_pool_free` up front, since a full
  store refuses `station_submit` the way a full queue always has. The
  Python twin (`station.py`) does NOT mirror this -- it is an
  allocation change with no behavioural effect, and the C suites
  assert the counters stay zero.
- One scratch arena serves the whole C modem (`cport/src/arena.h`).
  Two separate facts make it safe and BOTH must hold: the receiver's
  detect/demod/decode scratch is call-scoped (nothing survives the
  return from `rxs_push`), and the link is HALF DUPLEX, which is the
  only reason the streaming transmitter's generator state -- live
  across `txs_pull` calls -- may share it. Anything live across calls
  that is NOT transmit state must stay out (`g_raw`, `g_blk`,
  `g_h64`/`g_d64`). Every region asserts its own fit at compile time,
  so `OFDM_ARENA_BYTES` is a safe knob: a transmit-only build sets it
  to 27000 and a too-small value fails the build naming the region.
  The half-duplex half is checked at runtime, not trusted: receive
  entry points call `arena_claim(ARENA_RX)` and `txs_pull` aborts a
  generator whose state a receive phase overwrote (`txs_faulted()`).
  If you add a receive entry point that reaches arena-using code
  without going through `rxs_push`/`rxd_receive*`/`rx_detect*`, stamp
  it too. Measure the result with `make armmeas`, never by hand --
  these figures depend on which entry points an image references, and
  the previously documented pair came from a main nobody kept.
- Mixing float and fixed receivers: the fixed chain's detector returns
  positions in ANALYTIC index space, offset by the 31-sample Hilbert
  group delay. It cancels inside `FixedReceiver.receive()` but any
  caller that detects in one space and slices/advances in the other
  accumulates 31 samples per step.
- `demoapp` at `--time-scale 25` outruns an 8-core host: protocol time is
  sample-derived, so the two stations' clocks drift APART (the mostly-
  transmitting side stays current, the decoding side lags) and the
  transmitter times out before the receiver has finished the burst.
  Signature: every timeout on one station, zero on the other, each
  followed at once by the reply. Measured 22 timeouts at 25x, 0 at 8x on
  the same transfer. Drop the time scale before debugging a timeout.
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
