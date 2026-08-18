"""The LLR-calibration study repeated for 16-QAM -- the reproduction
entry point for the technical report's Figure "qam16_recal".

What it measures: at NORMAL mode with 16-QAM (rate 1/3 for code parity
with the LDPC code), the three-way Viterbi study (raw / temperature /
BPSK-trained shape map), the four-way LDPC kernel study, and the
head-to-head -- SNR -2.5..+1.5 dB. The deployed map is gated to
mu <= 2 bits/symbol BECAUSE it regresses on 16-QAM; this study
measures that regression at 300 frames/point (the 4-bit Gray
constellation has two LLR shape classes -- axis-sign and
inner/outer -- and a map trained on BPSK's single class mis-corrects
them).

Run:  ./venv/bin/python experiments/qam16_recal.py [--trials N]
Outputs: results/qam16_recal.png + results/qam16_recal.json.
Non-default trial counts get suffixed filenames.
~6 min at 300 trials on 8 cores.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from figures import fig_qam16_recal  # noqa: E402


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trials", type=int, default=300,
                    help="frames per SNR point (default 300)")
    args = ap.parse_args()
    fig_qam16_recal(n_frames=args.trials)
