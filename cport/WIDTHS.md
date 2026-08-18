# Width audit notes (grows with each ported module)

Established by the ports so far (all verified bit-exact by golden vectors):

| Site | Width | Note |
|---|---|---|
| FFT datapath | int64 accum, values stay int16-range × N headroom | per-stage >>1 keeps width flat; twiddles Q15 from ROM |
| BFP exponent | int, ±small | counts headroom **left**-shifts: true = stored >> 2·exp (energy) |
| Hilbert accumulator | ≤ 63 × 2^15 × 2^15 ≈ 2^36 | int64 fine; int40 in RTL |
| NCO phase | uint32 accumulator | negative words fold via modulo-2^32 cast |
| CORDIC x/y | input ×1.647 growth | int64 for ≤2^30 inputs; angle fits int33 signed |
| Viterbi path metrics | init −2^30, grows ≤ steps×3×31 | int32 safe ≤ 261 steps (int64 used) |
| isqrt | full int64 | non-restoring |
| TX clip energy sum | ≤ 5.3e5 samples × 2^30 ≈ 2^49 | int64; int50 in RTL |
| TX LPF accumulator | ≤ 33 × 2^15 × 2^15 ≈ 2^35 | int64; int36 in RTL |
| Tone metric `c0·c1` | c ≈ sig·255·1024/rest ≤ ~2^30 each | product int64, tight — cap analysis before narrowing |
| ZC correlator accum | klen × int16 × Q15 ≤ 2^40 | int64; int41 in RTL |
| ZC energy prefix sum | window × 2^31 ≤ 2^48 | int64 |

Traps found while porting (cost real debugging time — do not repeat):

- **R34 encoded length ≠ calc_cc_elements**: the Python encoder emits
  mask-sum-over-elements bits (142 for 100 info bits) while
  `calc_cc_elements` rounds up to whole puncture periods (144). RX crops
  to the latter; the decoder must accept an explicit `soft_len` and leave
  unfilled depuncture slots at zero, exactly like the Python truncation.
- Platform assumption: arithmetic right shift of negative signed ints
  (`fxp_selftest()` guards it at startup).
- **Two float remnants in the model's detection thresholds**:
  `int(np.mean(cc))` (ZC floor) is mirrored with the identical IEEE double
  division in C — an integer `sum/len` could differ by 1 on adversarial
  values; `int(np.median(...))` (tone floor) reduces exactly to
  `(a+b)/2` floor arithmetic for nonnegative int64 sums, so it stays
  integer. An RTL implementation should switch the model to integer mean
  first and revalidate the thresholds.
- All ROMs (twiddles, NCO sine, Hilbert taps, CORDIC atan, preambles,
  pilots, TX LPF taps) are **dumped from the Python model** by
  `gen_vectors.py` — never recomputed in C — so numpy's round-half-even
  (and scipy's remez/firwin) can never diverge from a C reimplementation.

Target-cost notes accumulated for `FEASIBILITY.md`:

- Preamble ROMs cost flash: 8.8 KB (NORMAL) / 34.9 KB (ROBUST) /
  139.3 KB (EXTREME) as stored int16 tables. An EXTREME-capable MCU build
  should synthesize the preamble at init (tones + ZC from small tables)
  instead of storing it.
- The host TX keeps the whole frame in an int64 scratch (4.2 MB worst
  case) for the frame-wide clip threshold; an MCU TX streams symbols and
  needs a two-pass or running-RMS clip instead.
