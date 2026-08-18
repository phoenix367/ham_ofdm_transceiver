"""The LLR-calibration study repeated at the EXTREME edge -- the
reproduction entry point for the technical report's Figure
"extreme_recal".

What it measures: at EXTREME (64x tiling, BPSK rate 1/3, the -17.9 dB
rung-0 regime), the same three-way Viterbi study (raw / temperature /
shape map), the four-way LDPC kernel study (min-sum & exact SPA, raw &
mapped), and the calibrated head-to-head -- SNR -20.5..-16.5 dB.
The shape map is the NORMAL-BPSK-trained deployed map, so this also
tests whether it transfers across modes, and it directly tests the
report's "accumulation-limited rungs are unaffected" claim.

Run:  ./venv/bin/python experiments/extreme_recal.py [--trials N]
Outputs: results/extreme_recal.png + results/extreme_recal.json.
Non-default trial counts get suffixed filenames. EXTREME frames are
~44 s of audio each: ~25 min at 300 trials on 8 cores.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from figures import fig_extreme_recal  # noqa: E402


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trials", type=int, default=300,
                    help="frames per SNR point (default 300)")
    args = ap.parse_args()
    fig_extreme_recal(n_frames=args.trials)
