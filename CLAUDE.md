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
  derived. On top of that raw estimate sits a per-(mode, modulation)
  OUTPUT MAP (`FixedReceiver.SNR_MAP` in `fixed/rx.py`, emitted into
  `rom_modes.h` by gen_vectors.py, applied by `rxd_snr_map` in both C
  receive paths): the raw estimate rails at a per-combo ceiling (LLRs
  saturate, then the tile_db subtraction spreads the ceilings up to
  12 dB apart), so EXTREME frames read +0.5 dB on a wire NORMAL frames
  read at +12 dB, and every EXTREME decode cratered the peer's filtered
  SNR and whipsawed the ladder (measured: rung 10 -> 0 -> 8 with zero
  losses). The map is measured (est_fixed, est_float) pairs over
  simulate_channel's own nominal convention, so the integer twin now
  reports what the FLOAT estimator -- the reference the ladder was
  system-tested against -- reads on the same waveform; post-map the
  twins agree to ~0.3 dB across the grid and `fixed_point.py` asserts
  that parity at −7/0/+15 (the old "estimate ≈ nominal" check described
  the pre-map contract and is gone). Recalibrate by re-running the
  sweep if the estimator or the channel model changes. The frequency search (fixed: first symbol via `COARSE_GATE_Q4`;
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
- The CAPABILITY HANDSHAKE (`station.c` `FLAG_CAPS` = flags 7, the third
  impossible combination) declares what a peer can do instead of
  discovering it by failing. Invariants: the record goes out only when
  BULK is waiting and nothing is in flight (`caps_probe_wanted`); an
  unanswered probe is FORGIVEN by the rate controller (`caps_inflight`
  in the timeout path) -- silence is a fact about an older peer, not the
  channel -- and after `CAPS_TRIES` the peer is `legacy` and the old
  defaults apply; a declared "no stream" sets `peer_stream_retry = -1`,
  which `rto_sample`'s optimistic reset and the engage re-probe both
  respect (do not reset it to 0 anywhere). `CAP_STREAM` is advertised
  from the `burst_stream` KNOB, never from `phy.receive_burst`: the
  streaming receivers (demoapp, firmware) leave that hook NULL by
  design and the first version masked on it -- smoke test went 5 -> 29
  frames, every window per-frame against a peer that streams fine.
  Unit tests that drive one station by hand set `caps_disabled` (they
  expect the first frame to be theirs, not a probe). The Python
  `station.py` does NOT mirror this (it never mirrored burst ARQ
  either); `test_link.c` `test_caps` is the twin. The record also
  carries two OPERATOR knobs -- `my_win_max` (byte 4, `config win_max`,
  UP_CFG key 8) and `my_max_rung` (byte 9 as ceiling+1 so 0 stays
  "unspecified"; `config rung_ceiling`, key 1, which was documented
  since the first protocol rev and implemented NOWHERE until this) --
  and both are enforced on BOTH ends: the window at engage is
  min(burst_window, BURST_STREAM_MAX, my_win_max, peer.win_max), and
  every tx-rung decision goes through `st_tx_rung()` = controller
  clamped by my_max_rung and the peer's declared ceiling (clamping
  inside ctl would poison its offset learning -- keep the clamp in
  station.c). Requests go through `st_rx_request()` for the same
  reason. A config change while the peer is known sets
  `caps_reply_due` so the refreshed record is pushed rather than
  waiting for the next transfer.
- The radio boards hold 3328-byte messages (`-DST_MSG_MAX=3328`,
  `-DBURST_STREAM_MAX=16` in radiofw). Placement: `g_st` (~58 kB)
  lives in SRAM4 -- the D3 domain, the SLOWEST RAM for the M7 (two bus
  bridges), which is fine because the station runs at frame cadence
  from the main loop and never the ISR; the 16-block stream TX buffer
  (`g_stream_blocks`, hoisted to file scope in station.c so the linker
  can place it by name) joins the rings in D2, which is now full to
  ~2 kB. SRAM4 has NO RCC gate for CPU access; it needs its own zero
  loop in startup_flash.c (`_ssram4bss`). `UP_MAX_PAYLOAD` is 3336 so
  one message crosses USB in one frame; the modem's delivered-message
  buffers are static. The consoles take the board's `msg_max` from
  INFO and the peer's from STATUS and split files against the smaller
  -- and the console-side buffer cap (`USB_MSG_CAP`) must move with
  `ST_MSG_MAX`, because the INFO value is only as useful as the buffer
  behind it (a measurement run silently split at the old 2048).
- File parts must be WINDOW-ALIGNED: one part is one station message,
  and the throughput lever is acknowledgments per byte. The consoles
  split at `peer_win_max * 200 - head` so the MESSAGE (envelope
  included) is an exact multiple of the 200-byte fragment -- the
  streamer deliberately refuses short tails in windows
  (`last_len != fs` in burst_send_stream), so a misaligned part pays a
  whole extra ack cycle for its tail. Measured on the 68 kB wire
  transfer at rung 12: 2048-byte parts 87 B/s (3 acks/part), aligned
  1600-byte parts 102 B/s (1 ack/part), 2 kB parts under window 16
  98 B/s (the tail penalty). The FIRST bulk transfer to a stranger
  still splits conservatively (window 8): the caps handshake is
  triggered BY that transfer, so its record arrives too late to size
  it -- prime with any small bulk item when it matters.
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
  (`demoapp/app.c`, magic 0x04 vs 0x03; the part index and count are
  16-bit LE — the byte-indexed 0x01/0x02 envelope is still RECEIVED
  but its 255-part cap refused a 68 kB PNG over USB at ~230 B/part,
  and both consoles + `test_board_console.py` moved together because
  nothing at run time reports an envelope mismatch) — per-part
  compression would throw the ratio away. Compression lives in the app, not `cport/`, so
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
- The raw ring (`RXS_RAW_RING_LEN`) is sized by how far back the
  receiver reaches when the tone detector COMMITS, which is ~2 tone
  fields after the peak. Two things used to anchor at `cs_abs` and set
  that depth; neither does now, and both must stay that way: the lag-N
  residual is accumulated per block (`lag_sum_t`, its own ring because
  `g_blk` only holds ONE tone window and commit needs two), and the ZC
  scan is anchored at the tone field's END
  (`ZC_ANCHOR_MARGIN_BLK`). Together: 124478 -> 67134 samples at
  EXTREME, ring 288 -> 160 kB, and the acquisition burst went from 166 %
  of a 480 MHz M7 to 42 %, i.e. from not keeping up to keeping up.
  The narrower search costs NO sensitivity: `make zcab` A/Bs both arms
  (`-DZC_ANCHOR_LEGACY` builds the old one) over byte-identical EXTREME
  waveforms -- 614 vs 609 decodes per 1000 at the knee, bounding the
  difference at ~0.01 dB, and 0 false alarms in 250 noise-only runs
  each. But `ZC_ANCHOR_MARGIN_BLK` = 8 is TUNED: 4 fails and 16 fails
  (at 16 the anchor clamps to 0 for NORMAL and data symbols re-enter
  the window). Re-run `make zcab` if you change it.
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
- The C STREAMING receiver's tone commit (`rx_stream.c` `tone_commit`)
  carries the same two-lag residual as the frame-at-once detector:
  lag-N for the fine estimate, lag-N/2 to pick its cycle, unwrapped
  exactly as `rx_residual_word_src`. Both lags are accumulated per
  block in `lag_sum_t` (its own ring, two tone windows deep) and summed
  at commit with the coarse word subtracted as an angle. It did NOT have
  the second lag until the analog loopback stand found the gap: a frame
  whose tone field starts exactly on a detection block (lead 0 or 512)
  missed the coarse bin by one, the lag-N residual wrapped, and every
  modulation failed at -94.07 Hz -- on a loop with zero actual CFO.
  Keep both lags; `test_stream` holds the two block-aligned leads as
  self-consistency cases, and the EXTREME sweep gained 6.9 % decodes
  from it. The Python fixed `_detect_newman` has the same construction.
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
- Flash-resident images MUST enable the L1 caches (`startup_flash.c`
  `cache_init`). Without it code runs from flash at 2 wait states with
  no I-cache and every SRAM access uncached -- 5-10x slower than the
  same code out of ITCM, which is why the RAM benches never showed it.
  It did not matter while the flash images only ran USB and a link
  layer; it is fatal once the streaming RECEIVER must keep up with a
  12 kHz converter. Measured with caches off: the receiving board
  dropped 1522686 samples (64% of everything the ADC produced) and
  decoded nothing; the transmitter underran its DAC 375 times. Two
  consequences: a debugger reading over the AHB-AP does NOT see dirty
  lines, so anything read over JTAG (the beacon) must be cleaned
  explicitly (`beacon_flush`); and `vectors.c` must clean vector-table
  writes for RAM images that inherit DC=1.
- In `stm32h743_flash.ld`, `.d2_bss`/`.dtcm_bss` MUST be placed BEFORE
  `.bss`. The linker assigns an input section to the first output
  section that matches, and `.bss`'s `*(.bss*)` matches `.bss.g_raw`
  too -- with `.bss` first the D2 lines matched nothing, the 160 kB
  sample ring sat in AXI, and adding the receiver overflowed AXI by
  100 kB while D2 stayed empty. `stm32h743_usb.ld` always had the right
  order, which is why only the flash images were affected.
- Any IRQ the firmware enables in the NVIC must have an entry in
  `startup_flash.c`'s `g_vectors`. Designated initialisers leave every
  other slot NULL, so an enabled interrupt with no entry vectors to
  address 0 -- TIM6_DAC (IRQ 54) had no entry when the radio build
  first enabled it.
- `station_phy_t::build` may return a frame's LENGTH without rendering
  a sample. The station never reads or writes that buffer -- it only
  passes it through to `build`/`build_burst` and hands the returned
  count to the caller. `usb_radio_main.c` relies on this: an EXTREME
  frame is ~456000 samples (912 kB), impossible to buffer on the part,
  so `build` opens a streaming generator and the main loop pulls from
  it into a small DAC FIFO. Pre-fill that FIFO before starting the
  carrier -- arming it empty underran once per transmission (166
  underruns over 3 frames).
- The capture FIFO between the converter ISR and `rxs_push` is sized by
  the receiver's WORST BLOCKING BURST, not its average. Measured at
  EXTREME on the part: 2283 ms in a single `rxs_push` (the end-of-frame
  commit) against a 19.5 s frame -- ~12% average duty, comfortably real
  time, but it arrives all at once. 16384 samples (1.37 s) dropped
  33036 samples mid-frame and decoded nothing; 65536 (5.5 s) decodes
  with `cap_overruns` 0. Check it with the beacon's `cap_overruns` and
  `push_ms_max`, never by assuming.
- Opening a receiver per mode is a CPU budget, not a free choice: each
  runs its own detector over every sample and EXTREME is much the most
  expensive. demoapp can afford all three; the part cannot, so
  `usb_radio_main.c` opens all three and MUTES the ones it does not
  need (`rxs_set_active`). A muted instance still consumes every sample
  -- it must, the raw ring is shared and indexed by each instance's own
  abs_n, so an instance that stopped counting would write the others'
  history at the wrong offsets -- and skips only the per-block
  detection, which is where the cost is. Unmuting rearms the search
  rather than resuming a state machine that was mid-frame when it went
  quiet.
- `phy.build` reporting a length it never renders means NOTHING else
  would notice if the generator then produced a different number of
  samples -- the station would believe it transmitted a frame the air
  never carried. `usb_radio_main.c` checks `g_tx_pulled` against
  `g_tx_total` at key-down (`tx_short`). Worth the check: the FIFO
  capacity arithmetic was written twice and the second copy passed
  `TXF_N - (w - r) - (w & (TXF_N-1))`, which with r = 0 is `TXF_N - 2w`
  -- it double-counts the write index against the free space and reaches
  zero once the FIFO is half full. `txs_pull` then returned 0, the
  caller read that as "frame finished", and the board transmitted a
  TRUNCATED frame. It hid at EXTREME (the first pull fills the whole
  FIFO) and for frames shorter than the FIFO, so the link bootstrapped,
  climbed to a rung whose frames are longer than the FIFO but pulled in
  pieces, and then stopped being decodable. One `tx_fill()` now.
- Carrier sense must be measured on LIVE samples, not on whatever the
  decoder has caught up to. demoapp accumulates over the samples it
  pushes into the receiver, which is the same thing there because it
  pushes in real time; on the part a single `rxs_push` can block for
  2.4 s, so the pushed stream runs seconds behind the air and energy
  detection reports the channel as it WAS. `usb_radio_main.c`
  accumulates in the 12 kHz tick and publishes the mean as one 32-bit
  store.
- `burst_window >= 2` is what engages burst ARQ at all (`station.c`);
  `burst_stream` alone does nothing. Both are ON in the radio firmware
  now that the three streamed-burst causes are fixed. The 224-s-frame
  hazard that once argued for OFF (`frag_size` fixed at engage, rung
  collapsing mid-transfer) was a symptom: the collapse was driven by
  those failures poisoning the controller.
- Streamed bursts DECODE across two boards (`burst_advance` driving
  `rxs_continue_burst`; `phy.receive_burst` stays NULL because it is
  only reachable from the frame-at-once `phy.receive` a streaming
  receiver never calls). The failure that looked like a channel problem
  had THREE causes, none of them the burst machinery, and none of them
  the clocks (`cport/bench/burst_repro.c`, `make burstrepro`, decodes
  8/8 under DAC/ADC requantization + DC + 56 ppm):
  1. `-DMAX_LLRS=1024` (copied from the small-frame bench builds)
     overflowed `g_d64` for any `frag_size >= ~100` -- silently, there
     was no guard. Host repro: 0/3 blocks at 1024, 3/3 at the default.
     `rx_stream.c` now REFUSES a header whose block exceeds
     `MAX_SYMS`/`MAX_LLRS` (type -2), and the radio build uses 4416
     (largest EXT frame), with `g_d64` placed in DTCM by the linker.
  2. Carrier sense was DEAD: the edit that was to call `note_busy_isr`
     from the tick had wrong indentation in its pattern and
     `str.replace` silently did nothing -- `g_cs_mean` stayed 0 and
     `channel_busy()` said idle forever. Caught by the beacon's key-up
     recorder (`cs=0` at every key-up where a quiet wire reads ~2e4).
     Assert every patch replacement, and record cs/floor at key-up.
  3. Block 0 of a stream carries the ack request BY DESIGN (for peers
     that cannot stream), so the station answers it immediately -- and
     with CS dead, B keyed its bitmap ack ~2 s into A's 9.75 s stream,
     dropped its own capture at key-up, and the walk decoded -30 dB
     noise. The fix is structural, not just CS: a receiver whose walk
     expects more blocks (`g_burst_left`) HOLDS its transmitter, with a
     15 s deadline so a peer dying mid-stream cannot mute us forever.
  With all three fixed: one 8-block stream acks 8 of 11 frags, the
  1200-byte file crosses byte-exact, `burst_blocks` 8 / `misses` 0, and
  the burst defaults (`burst_window`, `burst_stream`) are ON.
- BROADCAST on the boards is a THIRD walk, not `src/broadcast.c`. That
  file's `bc_receive` scans a whole recording, which a board never has;
  the firmware (`usb/usb_radio_main.c`) builds one group per keying
  (`bc_open_group`) and walks the received group with `bc_advance` over
  the STREAMING receiver, stepping blocks with `rxs_continue_burst`
  exactly as `burst_advance` does. Keep the invariants:
  * BCAST frames (`PKT_TYP_BCAST`, typ 6) are Data-shaped but carry NO
    link-control word, so `bc_advance` runs BEFORE `burst_advance` and
    `station_on_decoded` -- one reaching the ARQ reassembler would be
    read as a fragment.
  * A broadcast walk holds this station's transmitter AND pins its mode
    active (`g_bc_left`, in the poll_tx gate and in `follow_rung`), the
    same as a burst walk -- and therefore needs the same escape hatch,
    `BC_WALK_DEADLINE_MS`. Without it a peer that stops mid-group leaves
    the board permanently mute; the walk only advances on events, and a
    peer that stopped sends none.
  * One group is ONE keying and the yield between groups IS the
    carrier-sense gate; a receiver that heard broadcast traffic within
    `BC_RX_HOLD_S` holds its own transmitter, because the gaps between
    groups are not idle channel.
  * Group size must be a power of two -- the SYNC descriptor carries
    log2(group) -- and is capped by air time (`bc_group_frames`): four
    26-byte frames are 9.2 s at rung 4 but over a minute at EXTREME,
    which would break the carrier-sense time constants above.
  * The rung DECIDES WHETHER THE PEER IS LISTENING AT ALL, because
    active modes follow the negotiated rung (`follow_rung`) and decay
    to EXTREME-only after `RX_STALE_S` of silence. So the default
    (0xFF) is `ctl_tx_rung()` -- the rung we would send the peer a
    frame at, which IS the peer's own request and therefore the one
    rung it is certainly receiving on. Do NOT floor it: an earlier
    default floored `stats.last_rung` at `BURST_MIN_RUNG`, which
    turned "this link runs at EXTREME" into "broadcast at NORMAL", and
    a peer whose ladder had decayed heard the carrier at 1.1e8 and
    decoded nothing -- twice, with every counter healthy on both
    boards. An explicit `bcast -r N` is honoured as given, including a
    slow one: EXTREME is the only mode an idle station is guaranteed
    to keep active, so a beacon for strangers belongs there. bc_cmd
    logs the rung it chose (UP_EVT_LOG) precisely because "nothing
    arrived" cannot distinguish the two cases from the host.
  * BROADCASTFILE streams the file from the host in CHUNKS: bit 7 of
    UP_CMD_BCAST's ptype byte = more follows, bit 6 = continuation of
    the broadcast in flight (both clear = today's one-shot bcast, so
    old consoles keep working). The board's source buffer is 8 kB in
    DTCM, the host paces against `bc_free` in the status frame, and
    two rules keep the wire format honest: the EOS flag is only
    emitted once the host has said the stream is complete
    (`g_bc_complete`), and while chunks are still arriving only FULL
    groups are keyed -- a starved tail group would otherwise go out
    without EOS and the receiver's walk would never see the end. Raw
    bytes, ptype OPAQUE, same convention as socket-mode broadcastfile:
    every receiver stores rx_broadcast.bin.
  * A lost group is lost WHOLE: the loss is a missed preamble
    acquisition, and every frame behind it goes with it. Measured at
    rung 4 (4-frame groups, 9.2 s each): 24/24 groups over a
    12-broadcast run, 38/39 across the campaign; and a `-r 0` beacon
    reaches a board that has never exchanged a frame (EXTREME-only
    listening) byte-exact at +16.2 dB. ARQ hides this failure rate by
    retransmitting and broadcast cannot, which is exactly what
    `frames_ok`/`frames_lost` in the EOS event are for. The host twin
    of the SAME build-and-walk path is `make bcrepro` (in
    `make robust`) -- it decodes 6/6, so a group loss is an on-air
    acquisition miss, not a code path.
- The board LED is on **PA1**, and the pin was found by measurement,
  not by reading a schematic for a different board: these are
  STM32H743VI in **LQFP100** (`SYSCFG_PKGR` @0x58000524 reads PKG=0 on
  both), where port H is bonded out as PH0/PH1 only -- both taken by
  the 25 MHz crystal -- so PH2..PH15 do not exist and writing their
  MODER/ODR bits drives nothing. PA0, the obvious first guess, floats
  (input probe: follows an internal pull both ways); PA1 reads HIGH
  against an internal pull-down, which is the LED's own pull to VDD --
  it is ACTIVE-LOW (`LED_ACTIVE_LOW 1`; the solid state came out dark
  on the stand with active-high, and that is the one thing the beacon
  cannot see, so ask the eye). The
  finder is `make run-led` (`bench/led_test.c`, RAM-resident so flash
  is untouched; `LED_PIN=` / `LED_DEFS=-DLED_PORT_BASE=` to try
  another pin), and it blinks a four-phase pattern rather than a plain
  blink because a plain blink cannot be told from a power LED.
  `src/led.h` is the shared one-pin driver. Firmware states, highest
  priority first: 10 Hz transmitting, 2 Hz receiving (a walk is live,
  carrier sense reads busy, or a frame decoded within 500 ms), solid
  when a host program has attached (a COMMAND from the host, not the
  cable being plugged in), dark otherwise. `led_tick()` reads the
  carrier-sense verdict the transmit gate already computed
  (`g_cs_busy_seen`) -- do NOT add a second `cs_busy()` caller: that
  function carries the floor tracker, whose climb is defined per 40 ms
  window and per CALL SITE (see the 40x-too-fast climb above).
- Every carrier-sense TIME CONSTANT must exceed the longest frame the
  station can emit, and on the boards that used to be 224 s: frag_size
  is fixed at engage, so a burst engaged at rung 10 (203-byte frags)
  that collapsed to rung 0 sent 224-s EXTREME frames. Those violate the
  60-s rebase AND the floor climb (which crosses the 9x busy threshold
  after ~176 s of continuous signal) -- the peer then keys over the
  frame, and the link death-spirals with both boards healthy. Fixes,
  all three: CS_REBASE_S 300 on the boards; poll_tx DISENGAGES a burst
  whose next fragment would exceed BURST_FRAG_MAX_AIR_S (45 s) at the
  CURRENT rung (diag ST_EV_BURST_REFRAG; cur_bulk falls to the legacy
  air-capped path and burst re-engages when the ladder recovers); and
  burst_send_stream refuses a stream over BURST_WIN_MAX_AIR_S at the
  current rung.
- Carrier sense needs a WARM-UP GATE on both the producer and the
  consumer. Gating only the ISR feed leaves g_cs_mean at 0 through
  warm-up, and channel_busy() -- called every 1 ms from boot -- reads
  that zero and snaps the floor to its 25.0 clamp; the real quiet wire
  then reads busy forever. Measured to the millisecond, twice: both
  boards' first-ever key-ups at ms=300031/300029 = CS_REBASE_S exactly.
  channel_busy() returns idle and leaves the floor alone until
  g_ms >= 1000.
- The receive front end is DC-BLIND BY CONSTRUCTION: the firmware ISR
  runs every ADC sample through `src/dcblock.h` (one-pole blocker,
  7.5 Hz corner, integer-only) BEFORE the capture FIFO and carrier
  sense, so no input operating point -- parked peer DAC, AC-coupling
  bias network, ground offset -- can reach the busy logic. Verified:
  -0.02 dB at 300 Hz, a +12000 DC step settles in 117 ms, and the
  decode matrix passes 8/8 at DC +-18000.
- Carrier sense lives in `src/csense.c` (shared, host-testable), NOT in
  the firmware: `make cstest` replays every measured field failure as a
  scenario regression (boot latch, parked DC bare vs through dcblock,
  45-s busy hold, frame/gap cycling, DC-step transient). `make robust`
  is the receiver-reliability gate: cstest + the input-abuse decode
  matrix (DC/steps/clipping/hum/impulses/stuck-converter through
  burst_repro). The DEMODULATOR passes the whole matrix even bare --
  every field failure of the stress campaign lived in carrier sense,
  which is why CS gets the scenario suite.
- The IDLE DAC must be PARKED AT MID-RAIL explicitly (usb_radio_main
  tx-end path). The tick only writes the DAC while transmitting, so
  without the park the pin HOLDS the frame's last sample -- anywhere in
  +-0.9 V -- until the next transmission. Decoding never notices (bin 0
  unused, DC-blind) but the peer's carrier sense reads mean SQUARE: a
  park 0.83 V off mid-rail puts a constant 2.7e8 on the peer's cs, its
  floor glues to it, and the level changes with every frame's final
  sample. This manufactured every "flaky wire" symptom on the stand --
  wandering DC levels, heal-and-relapse, power-cycle "fixes" (fresh
  dac_init parks mid-rail until the first frame ends). The diagnostic
  signature is cs nearly CONSTANT (+-0.4%) across seconds while frames
  still DECODE fine; the raw capture ring (g_cap over JTAG) settles it
  in one read (stdev ~27 at a fixed mean = a parked/floating level,
  stdev ~11000 = signal). Suspect the park/bias before the wire.
- The CS floor's climb rate is defined PER 40 ms WINDOW (demoapp's
  cadence), not per call: `channel_busy()` runs at 1 kHz on the boards
  vs ~25 Hz in demoapp, and multiplying per call raised the floor 40x
  too fast. Climb only when 40 ms have elapsed.
- Which modes are active follows the NEGOTIATED rung (`follow_rung`),
  and two extras are kept alongside `ladder_mode(ctl.my_req)` for
  reasons that are not optional. EXTREME always: it is rung 0 (the
  bootstrap), it is where the ladder decays to after RX_STALE_S of
  silence, and it is what a peer that cannot hear us falls back to.
  And the mode we LAST DECODED on: my_req is a request, not an
  observation, so between raising it and the peer acting on it the peer
  is still transmitting at the old mode -- dropping that mode
  immediately would make us deaf for exactly the exchange that would
  have confirmed the change, and the link would oscillate. Measured on
  the wire: both boards idle on EXTREME alone (one detector), and after
  the ladder climbed to my_req 9/7 they listen on NORMAL+EXTREME, with
  cap_overruns 0 throughout.
- A RAM bench inherits the machine state of the firmware it displaces,
  and the M7 vector table lives in CACHED SRAM. `vectors_set`/
  `vectors_install` (`cport/target/vectors.c`) MUST clean by MVA
  (`vect_flush`): `DSB` orders accesses, it does not clean, and an
  exception vector fetch reads MEMORY, not the D-cache, so a new entry
  sits in a dirty line while the hardware still fetches the old one.
  Measured on the two-board stand (`make -C cport link-run`): the
  receiver took its first TIM6 interrupt into the STRAY handler, which
  masked IRQ 54 -- ISER1 0, ISPR1 pending forever, isr_count 0, run
  loop spinning. Reading the table over JTAG showed the CORRECT
  handler, because the line had been evicted by then; that is what
  makes this look impossible. The transmitter role never saw it --
  `build_frame()` runs between install and first interrupt and evicts
  the line in time, so the SAME binary on the SAME board failed only in
  one role. Same root cause is why `vectors_irq_enable` clears ICPR
  before ISER: ISER3 bit 5 (OTG_FS) is still set at image entry, so a
  stale pending interrupt would be taken first and get its source
  masked. `SCB_CCR` read 0x00070200 (DC set) on the stalled board.
- Comparing the two boards' TIM6 clocks CANNOT measure their
  sample-rate offset and will confidently report 0 ppm: each board
  measures TIM6 against its own DWT and both come off the SAME PLL, so
  the ratio is exact on each board and identical between them however
  far apart the crystals are. Measure it on the wire instead, from the
  spacing of consecutively decoded frames. With 2 frames in the
  54000-sample window that is a BOUND (+-56 ppm), not a figure.
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
