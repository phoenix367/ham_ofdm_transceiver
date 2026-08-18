"""Overlay the BER/PER curves of the Newman-preamble baseline
(results/ber_per.json) and the STF-preamble variant (results/ber_per_stf.json)
and flag any point that differs beyond binomial noise.

Run after both sweeps:
  python experiments/ber_per_simulation.py --trials 120
  python experiments/ber_per_simulation.py --trials 120 --modem stf
  python experiments/compare_stf_newman.py
"""

import json
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"
N_TRIALS = 120  # packets per point in both sweeps


def main():
    nm = json.load(open(RESULTS / "ber_per.json"))
    st = json.load(open(RESULTS / "ber_per_stf.json"))
    labels = list(nm)

    for metric, fname in (("per", "per_stf_vs_newman.png"),
                          ("ber", "ber_stf_vs_newman.png")):
        fig, axes = plt.subplots(2, 4, figsize=(15, 7), sharey=True)
        for ax, label in zip(axes.flat, labels):
            snr = nm[label]["snr"]
            a = np.maximum(nm[label][metric], 1e-4)
            b = np.maximum(st[label][metric], 1e-4)
            ax.semilogy(snr, a, "-o", ms=4, color="tab:blue", label="Newman")
            ax.semilogy(snr, b, "--s", ms=4, color="tab:red", label="STF")
            if metric == "per":
                ax.axhline(0.1, color="gray", ls=":", lw=1)
            ax.set_title(label, fontsize=10)
            ax.grid(True, which="both", alpha=0.3)
            ax.set_ylim(5e-5, 1.5)
        axes[0][0].legend(fontsize=9)
        for ax in axes[1]:
            ax.set_xlabel("SNR, dB")
        for ax in axes[:, 0]:
            ax.set_ylabel(metric.upper())
        fig.suptitle(f"{metric.upper()} vs SNR: Newman baseline vs STF preamble "
                     f"({N_TRIALS} packets/point)")
        fig.tight_layout()
        fig.savefig(RESULTS / fname, dpi=130)
        print(f"saved {RESULTS / fname}")

    # statistical check: PER difference vs 2-sigma binomial noise
    print(f"\nPER deltas (STF - Newman) exceeding 2 sigma for n={N_TRIALS}:")
    flagged = 0
    max_d = (0.0, "", 0)
    for label in labels:
        for i, snr in enumerate(nm[label]["snr"]):
            pa, pb = nm[label]["per"][i], st[label]["per"][i]
            d = pb - pa
            p_pool = (pa + pb) / 2
            sigma = np.sqrt(max(2 * p_pool * (1 - p_pool) / N_TRIALS, 1e-12))
            if abs(d) > max_d[0]:
                max_d = (abs(d), label, snr)
            if abs(d) > 2 * sigma and sigma > 0:
                print(f"  {label} @ {snr:+d} dB: {pa:.3f} -> {pb:.3f} "
                      f"(delta {d:+.3f}, {abs(d) / sigma:.1f} sigma)")
                flagged += 1
    if not flagged:
        print("  none - every point is within 2 sigma of the baseline")
    print(f"largest |delta PER|: {max_d[0]:.3f} ({max_d[1]} @ {max_d[2]:+d} dB)")


if __name__ == "__main__":
    main()
