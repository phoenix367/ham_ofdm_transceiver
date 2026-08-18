"""Front-end LLR miscalibration shape -- the reproduction entry point
for the technical report's Figure "llr_shape" (Section "Front-End LLR
Calibration", incl. its Measurement Procedure subsection).

What it measures: raw data-block LLRs from the production receive chain
paired with the known transmitted coded bits (self-labeling via the
deterministic TX pipeline), NORMAL BPSK 1/3 at -8 dB; binned by |L|
into empirical error rates and empirical log-odds, plus the
single-temperature moment fit alpha = 2*E[l]/Var[l].

Key expected behaviour: weak LLRs (|L|<2) err far less than their value
implies (underconfident); the strong tail errs ~100x more
(overconfident); the moment-fit alpha < 1 while the weak bins want
alpha > 1 -- the contradiction only the monotone shape map resolves.

Run:  ./venv/bin/python experiments/llr_shape.py [--trials N]
Outputs: results/llr_shape.png + results/llr_shape.json (bin table,
alpha, all parameters). Non-default trial counts get suffixed filenames
so smoke runs cannot clobber the reference.
~2 min at 300 trials on 8 cores (parallel 30-frame chunks).
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from figures import fig_llr_shape  # noqa: E402


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trials", type=int, default=300,
                    help="frames to collect (default 300)")
    args = ap.parse_args()
    fig_llr_shape(n_frames=args.trials)
