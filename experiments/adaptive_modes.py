"""PER vs SNR for the adaptive link modes (BPSK, rate 1/3, article channel).

  NORMAL   4x tiles   118 bit/s user rate
  ROBUST  16x tiles    31 bit/s
  EXTREME 64x tiles   7.8 bit/s

Run:  python experiments/adaptive_modes.py [--trials N]
Outputs: results/adaptive_modes.json, results/per_adaptive_modes.png
"""

import argparse
import json
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ofdm_phy import (Transceiver, Data, ModType, CCSpeed, LinkMode, make_modem,
                      simulate_channel)

SNR_GRID = {
    LinkMode.NORMAL: list(range(-9, -3)),
    LinkMode.ROBUST: list(range(-16, -10)),
    LinkMode.EXTREME: list(range(-22, -16)),
}

_TRX = {}


def _get_trx(mode_v):
    if mode_v not in _TRX:
        _TRX[mode_v] = Transceiver(make_modem(LinkMode(mode_v)))
    return _TRX[mode_v]


def run_chunk(args):
    mode_v, snr_db, seed, trials = args
    trx = _get_trx(mode_v)
    fs = trx.modem.sample_rate
    rng = np.random.default_rng(seed)

    pkt_errors = 0
    for _ in range(trials):
        pkt = Data(reserved=123, payload=rng.bytes(27))
        sig = trx.build_frame(pkt, mod=ModType.BPSK, spd=CCSpeed.R13)
        rx = simulate_channel(sig, int(rng.integers(200, 1500)),
                              float(rng.uniform(-100, 100)), fs, snr_db=snr_db, rng=rng)
        try:
            dec, _ = trx.demod_frame(rx, check_crc=False)
            if dec != pkt:
                pkt_errors += 1
        except Exception:
            pkt_errors += 1
    return pkt_errors, trials


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=48)
    ap.add_argument("--chunk", type=int, default=6)
    args = ap.parse_args()

    results_dir = ROOT / "results"
    results_dir.mkdir(exist_ok=True)

    jobs = []
    for mode in LinkMode:
        for snr in SNR_GRID[mode]:
            for c in range(-(-args.trials // args.chunk)):
                trials = min(args.chunk, args.trials - c * args.chunk)
                seed = hash((mode.value, snr, c)) & 0x7FFFFFFF
                jobs.append((mode.value, snr, seed, trials))

    print(f"{len(jobs)} chunks, {args.trials} packets/point")
    t0 = time.time()
    acc = {}
    with ProcessPoolExecutor() as pool:
        for job, (pe, tr) in zip(jobs, pool.map(run_chunk, jobs, chunksize=1)):
            a = acc.setdefault((job[0], job[1]), [0, 0])
            a[0] += pe
            a[1] += tr
    print(f"finished in {time.time() - t0:.1f} s")

    curves = {}
    for mode in LinkMode:
        snrs = SNR_GRID[mode]
        pers = [acc[(mode.value, s)][0] / acc[(mode.value, s)][1] for s in snrs]
        trx = Transceiver(make_modem(mode))
        rate = trx.data_bit_rate(ModType.BPSK, CCSpeed.R13)
        curves[mode.name] = {"snr": snrs, "per": pers, "user_rate": rate}
        print(f"\n{mode.name} ({rate:.1f} bit/s user rate)")
        print("  SNR:", " ".join(f"{s:7d}" for s in snrs))
        print("  PER:", " ".join(f"{p:7.3f}" for p in pers))
        sens = None
        for i in range(len(snrs) - 1):
            if pers[i] >= 0.1 >= pers[i + 1]:
                sens = snrs[i] + (pers[i] - 0.1) / (pers[i] - pers[i + 1] + 1e-12)
        if sens is not None:
            print(f"  sensitivity (PER=10%): {sens:.1f} dB")
            curves[mode.name]["sensitivity"] = sens

    with open(results_dir / "adaptive_modes.json", "w") as fh:
        json.dump(curves, fh, indent=2)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 6))
    for mode, color in zip(LinkMode, ("tab:blue", "tab:green", "tab:purple")):
        c = curves[mode.name]
        ax.semilogy(c["snr"], np.maximum(c["per"], 1e-3), "-o", ms=4, color=color,
                    label=f"{mode.name} ({c['user_rate']:.1f} bit/s)")
        if "sensitivity" in c:
            ax.axvline(c["sensitivity"], color=color, ls=":", lw=1)
    ax.axhline(0.1, color="gray", ls=":", lw=1)
    ax.set_xlabel("SNR, dB")
    ax.set_ylabel("PER")
    ax.set_title("Adaptive link modes: PER vs SNR (BPSK 1/3, article channel)")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(results_dir / "per_adaptive_modes.png", dpi=130)
    print(f"\nsaved {results_dir / 'per_adaptive_modes.png'}")


if __name__ == "__main__":
    main()
