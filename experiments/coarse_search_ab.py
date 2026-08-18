"""Noise-sensitivity A/B of the gated two-stage frequency search: PER vs
SNR for the exhaustive grid and the gated coarse/fine search, on both the
float and the fixed-point receiver, same seeds/channels per arm.

The gate is designed to be lossless at the sensitivity edge (low-contrast
frames fall back to the full grid), so the gated curves must overlay the
full-grid curves within binomial noise everywhere. A third arm -- the
quarter-length coarse pass WITHOUT the gate (two-stage always trusted) --
is the ablation: its edge loss is what the gate exists to prevent.

Run:  python experiments/coarse_search_ab.py [--trials N]
Outputs: results/coarse_search_ab.json + results/coarse_search_ab.png
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

from ofdm_phy import Transceiver, Data, simulate_channel, make_modem
from ofdm_phy.modes import LinkMode
from ofdm_phy.fixed import FixedTransmitter, FixedReceiver

FS = 12000
GRIDS = {LinkMode.EXTREME: [-20, -19, -18, -17, -16, -14],
         LinkMode.ROBUST: [-14, -13, -12, -11, -10, -8]}
# variants: full = exhaustive grid; gated = two-stage with full-grid
# fallback below the contrast gate; coarse = two-stage always (no gate)
VARIANTS = ("full", "gated", "coarse")
ARMS = [(m, v) for m in ("float", "fixed") for v in VARIANTS]

_CTX = {}


def run_chunk(args):
    mode_name, snr, model, variant, seed, trials = args
    mode = LinkMode[mode_name]
    key = (mode, model, variant)
    if key not in _CTX:
        if model == "fixed":
            rx = FixedReceiver(mode)
            rx.coarse_search = variant != "full"
            if variant == "coarse":
                rx.COARSE_GATE_Q4 = 0  # ablation: always trust the coarse pass
            _CTX[key] = (FixedTransmitter(mode), rx)
        else:
            trx = Transceiver(make_modem(mode))
            trx.llr_recal = "auto"
            trx.modem.coarse_freq_search = variant != "full"
            if variant == "coarse":
                trx.modem.COARSE_GATE = 0.0
            _CTX[key] = (trx, trx)
    tx, rx = _CTX[key]

    rng = np.random.default_rng(seed)
    errors = 0
    for _ in range(trials):
        pkt = Data(reserved=123, payload=rng.bytes(6))
        if model == "fixed":
            sig = tx.build_frame(pkt).astype(np.float64)
        else:
            sig = tx.build_frame(pkt)
        rxs = simulate_channel(sig, int(rng.integers(200, 1500)),
                               float(rng.uniform(-80, 80)), FS,
                               snr_db=snr, rng=rng)
        try:
            if model == "fixed":
                r16 = np.clip(rxs / np.max(np.abs(rxs)) * 0.9 * 32767,
                              -32768, 32767).astype(np.int16)
                dec, *_ = rx.receive(r16)
            else:
                dec, _ = rx.demod_frame(rxs, check_crc=False)
            if dec != pkt:
                errors += 1
        except Exception:
            errors += 1
    return errors, trials


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=36)
    ap.add_argument("--chunk", type=int, default=3)
    args = ap.parse_args()

    jobs = []
    for mode, snrs in GRIDS.items():
        for snr in snrs:
            for model, variant in ARMS:
                for c in range(-(-args.trials // args.chunk)):
                    seed = hash((mode.name, snr, c)) & 0x7FFFFFFF  # same per arm
                    jobs.append((mode.name, snr, model, variant, seed,
                                 min(args.chunk, args.trials - c * args.chunk)))

    print(f"{len(jobs)} chunks, {args.trials} packets/point")
    t0 = time.time()
    acc = {}
    with ProcessPoolExecutor() as pool:
        for job, (err, tr) in zip(jobs, pool.map(run_chunk, jobs, chunksize=1)):
            a = acc.setdefault((job[0], job[1], job[2], job[3]), [0, 0])
            a[0] += err
            a[1] += tr
    print(f"finished in {time.time() - t0:.1f} s\n")

    out = {}
    fail = False
    for mode, snrs in GRIDS.items():
        for model, variant in ARMS:
            pers = [acc[(mode.name, s, model, variant)][0] /
                    acc[(mode.name, s, model, variant)][1] for s in snrs]
            arm = f"{model}-{variant}"
            out[f"{mode.name}/{arm}"] = {"snr": snrs, "per": pers}
            print(f"{mode.name:8} {arm:13} " +
                  "  ".join(f"{s:+d}:{p:.2f}" for s, p in zip(snrs, pers)))
        # parity check (gated vs full only -- the ungated coarse arm is the
        # ablation and is EXPECTED to lose at the edge)
        for model in ("float", "fixed"):
            for s in snrs:
                ef, tf = acc[(mode.name, s, model, "full")]
                eg, tg = acc[(mode.name, s, model, "gated")]
                p = max(ef, eg) / tf
                sigma = max(np.sqrt(tf * p * (1 - p)), 1.0)
                if abs(ef - eg) > 3 * sigma:
                    print(f"  PARITY FAIL {mode.name} {model} {s} dB: "
                          f"{ef} vs {eg} errors")
                    fail = True

    with open(ROOT / "results" / "coarse_search_ab.json", "w") as fh:
        json.dump(out, fh, indent=2)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), sharey=True)
    styles = {("float", "full"): ("tab:blue", "-", "o", "float, full grid"),
              ("float", "gated"): ("tab:blue", "--", "x", "float, gated coarse/fine"),
              ("float", "coarse"): ("tab:blue", ":", "^", "float, coarse only (no gate)"),
              ("fixed", "full"): ("tab:orange", "-", "o", "fixed, full grid"),
              ("fixed", "gated"): ("tab:orange", "--", "x", "fixed, gated coarse/fine"),
              ("fixed", "coarse"): ("tab:orange", ":", "^", "fixed, coarse only (no gate)")}
    for ax, (mode, snrs) in zip(axes, GRIDS.items()):
        for (model, variant), (color, ls, mk, label) in styles.items():
            pers = out[f"{mode.name}/{model}-{variant}"]["per"]
            ax.plot(snrs, pers, ls, marker=mk, color=color, label=label)
        ax.axhline(0.1, color="gray", lw=0.8, ls=":")
        ax.set_title(f"{mode.name} ({make_modem(mode).sym_tile}x tiles)")
        ax.set_xlabel("SNR, dB (6 kHz band)")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("PER")
    axes[0].legend(fontsize=8)
    fig.suptitle("Gated two-stage frequency search vs exhaustive grid "
                 f"({args.trials} packets/point, same channels per arm)")
    fig.tight_layout()
    fig.savefig(ROOT / "results" / "coarse_search_ab.png", dpi=130)
    print("\nsaved results/coarse_search_ab.json + coarse_search_ab.png")
    sys.exit(1 if fail else 0)


if __name__ == "__main__":
    main()
