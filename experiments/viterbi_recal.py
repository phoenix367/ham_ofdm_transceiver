"""Viterbi decoder vs LLR calibration -- the reproduction entry point
for the technical report's Figure "viterbi_recal" (Section "Front-End
LLR Calibration").

What it measures: each received frame is demodulated ONCE and the same
raw LLR vector is decoded three ways -- raw, temperature-scaled
(0.67*L), and monotone shape-mapped. NORMAL BPSK, conv K=7 rate 1/3,
27-byte packets, article channel, SNR -10.5..-6.5 dB.

Key expected behaviour: the temperature-only counts are BIT-IDENTICAL
to raw at every point (Viterbi is scale-invariant -- the script asserts
this); the monotone shape map shifts the decode waterfall by ~1.5 dB.

Run:  ./venv/bin/python experiments/viterbi_recal.py [--trials N]
Outputs: results/viterbi_recal.png + results/viterbi_recal.json (all
parameters and per-point decode counts). Non-default trial counts get
suffixed filenames so smoke runs cannot clobber the reference.
~5 min at 300 trials on 8 cores (parallel over SNR points).
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from figures import fig_viterbi_recal  # noqa: E402


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trials", type=int, default=300,
                    help="frames per SNR point (default 300)")
    args = ap.parse_args()
    fig_viterbi_recal(n_frames=args.trials)
