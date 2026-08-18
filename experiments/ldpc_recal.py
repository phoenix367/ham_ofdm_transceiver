"""LDPC decoder kernels vs LLR calibration -- the reproduction entry
point for the technical report's Figure "ldpc_recal" (Section "Coding
and Modulation Extensions").

What it measures: each received frame is demodulated ONCE and the same
raw LLR vector is decoded four ways -- normalized min-sum (the shipped
kernel) and exact sum-product (reconstructed tanh rule), each on raw
and on shape-mapped LLRs. NORMAL BPSK, LDPC (IRA rate 1/3), 27-byte
packets, article channel, SNR -10.5..-6.5 dB.

Key expected behaviour: sum-product on raw LLRs collapses ~1.5 dB below
min-sum (it consumes the front end's overconfident absolute values);
the shape map fully rescues it; min-sum gains ~1 dB from the map but is
never gated by it.

Run:  ./venv/bin/python experiments/ldpc_recal.py [--trials N]
Outputs: results/ldpc_recal.png + results/ldpc_recal.json (all
parameters and per-point decode counts). Non-default trial counts get
suffixed filenames so smoke runs cannot clobber the reference.
~12 min at 300 trials on 8 cores (parallel over SNR points).
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from figures import fig_ldpc_recal  # noqa: E402


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trials", type=int, default=300,
                    help="frames per SNR point (default 300)")
    args = ap.parse_args()
    fig_ldpc_recal(n_frames=args.trials)
