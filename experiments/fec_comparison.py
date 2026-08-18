"""FEC families head-to-head with a calibrated front end -- the
reproduction entry point for the technical report's Figure
"fec_comparison" (Section "Coding and Modulation Extensions").

What it measures: convolutional+Viterbi vs LDPC (exact sum-product and
normalized min-sum), all decoding shape-mapped LLRs from the production
demodulator, NORMAL BPSK rate 1/3, 27-byte packets, article channel
(random offset, +-60 Hz CFO, AWGN), SNR -10.5..-6.5 dB in 0.5 dB steps.
Channel draws are seeded per SNR point, so all three variants see
identical payloads/offsets/CFO/noise.

Reference result (300 frames/point, seeds as committed):
  - LDPC holds ~0.3-0.4 dB over conv (at -9.5 dB: 193 vs 151 /300)
  - calibrated min-sum ~= calibrated sum-product (identical above -9 dB;
    SPA keeps ~0.1 dB only at the -10.5 dB edge: 54 vs 40 /300)

Run:  ./venv/bin/python experiments/fec_comparison.py [--trials N]
Outputs: results/fec_comparison.png + results/fec_comparison.json
(the JSON records every parameter and per-point decode count).
~9 min at 300 trials on 8 cores (parallel over variant x SNR cells).
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from figures import fig_fec_comparison  # noqa: E402


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trials", type=int, default=300,
                    help="frames per (variant, SNR) point (default 300)")
    args = ap.parse_args()
    fig_fec_comparison(n_frames=args.trials)
