"""What does a bigger broadcast group cost under fading?

A broadcast repeats nothing, so group size is a straight trade: one
preamble amortised over more frames, against a missed acquisition
costing the whole group instead of part of it. On a clean channel that
second term never fires, and a 14162-byte file arrives complete at
groups of 4, 8 and 16 alike -- which says nothing about the case the
trade-off was written for.

This sweeps group size against SNR with Rayleigh fading enabled
(simulate_channel's fading_doppler_hz, which broadcast_demo.py leaves at
zero), reporting both halves of the trade:

  frames   fraction of frames decoded -- what the amortisation buys
  payload  fraction of payload bytes recovered -- what a lost group costs

The two diverge exactly where group size matters: losing a group's
preamble takes every frame behind it in that group, so payload recovery
falls off faster than the frame count as the group grows.

Run:  ./venv/bin/python experiments/broadcast_fading.py [--trials N]
Outputs: results/broadcast_fading.png + .json
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ofdm_phy import LinkMode, ModType, CCSpeed, simulate_channel  # noqa: E402
from ofdm_phy.broadcast import BroadcastTx, BroadcastRx, PT_TELEMETRY  # noqa

RESULTS = ROOT / "results"
RESULTS.mkdir(exist_ok=True)

FS = 12000
MOD, SPD = ModType.QPSK, CCSpeed.R12
GROUPS = [4, 8, 16]
SNRS = [-4.0, -2.0, 0.0, 2.0, 5.0, 9.0]
FADE_HZ = 0.5          # HF QSB: ~2 s fade period
PAYLOAD_N = 600


def run_point(group, snr_db, fade_hz, trials, payload):
    tx = BroadcastTx(LinkMode.NORMAL, MOD, SPD, group=group)
    rx = BroadcastRx(LinkMode.NORMAL, group=group)
    audio = np.concatenate(tx.build(payload, ptype=PT_TELEMETRY)).astype(float)
    deliv, frac = [], []
    for k in range(trials):
        # group is deliberately NOT in the seed: all three group sizes
        # then face the same fading realisation at each SNR and trial,
        # which is what makes the comparison paired rather than three
        # independent samples of a noisy quantity
        rng = np.random.default_rng([9161, int(snr_db * 10) + 1000, k])
        got, st = rx.receive(simulate_channel(
            audio, 300, 3.0, FS, snr_db=snr_db,
            fading_doppler_hz=fade_hz, rng=rng))
        deliv.append(st.delivery if st.expected else 0.0)
        frac.append(len(got) / len(payload))
    return float(np.mean(deliv)), float(np.mean(frac))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trials", type=int, default=30)
    ap.add_argument("--replot", action="store_true",
                    help="re-render from the saved JSON without measuring")
    ap.add_argument("--fade-hz", type=float, default=FADE_HZ)
    args = ap.parse_args()

    stem = ("broadcast_fading" if args.trials == 30
            else f"broadcast_fading_n{args.trials}")
    if args.replot:
        src = RESULTS / f"broadcast_fading_n{args.trials}.json"
        if not src.exists():
            src = RESULTS / "broadcast_fading.json"
        rec = json.load(open(src))["delivered"]
    else:
        payload = bytes((i * 37 + 11) & 0xFF for i in range(PAYLOAD_N))
        rec = {}
        print(f"  QPSK 1/2, NORMAL, Rayleigh {args.fade_hz} Hz, "
              f"{args.trials} trials/point")
        for group in GROUPS:
            rec[str(group)] = {"frames": [], "payload": []}
            for snr in SNRS:
                d, f = run_point(group, snr, args.fade_hz, args.trials,
                                 payload)
                rec[str(group)]["frames"].append(d)
                rec[str(group)]["payload"].append(f)
                print(f"    group {group:2d}  {snr:+5.1f} dB   "
                      f"frames {100*d:5.1f}%   payload {100*f:5.1f}%",
                      flush=True)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.3))
    for group, style, col in zip(GROUPS, ("o-", "s--", "^:"),
                                 ("tab:blue", "tab:orange", "tab:red")):
        ax1.plot(SNRS, [100 * v for v in rec[str(group)]["payload"]], style,
                 color=col, label=f"group {group}")
        gap = [100 * (a - b) for a, b in zip(rec[str(group)]["frames"],
                                             rec[str(group)]["payload"])]
        ax2.plot(SNRS, gap, style, color=col, label=f"group {group}")
    ax1.set_ylabel("payload bytes recovered, %")
    ax1.set_title("What actually arrives", fontsize=10)
    ax2.set_ylabel("frames decoded $-$ payload recovered, points")
    ax2.set_title("Loss concentration: the cost of a missed group",
                  fontsize=10)
    for ax in (ax1, ax2):
        ax.set_xlabel("SNR, dB")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    means = {g: 100 * sum(x - y for x, y in zip(rec[str(g)]["frames"],
                                                rec[str(g)]["payload"]))
             / len(SNRS) for g in GROUPS}
    ax2.annotate("mean gap: "
                 + ", ".join(f"g{g} {means[g]:.1f}" for g in GROUPS),
                 xy=(0.03, 0.94), xycoords="axes fraction", fontsize=8,
                 color="0.3", va="top")
    fig.suptitle(f"Broadcast group size under Rayleigh fading "
                 f"({args.fade_hz} Hz). A larger group decodes MORE frames "
                 f"but recovers no more payload:\nlosing one acquisition "
                 f"now costs 8 or 16 frames instead of 4, and nothing is "
                 f"retransmitted.", fontsize=9)
    fig.tight_layout()
    fig.savefig(RESULTS / f"{stem}.png", dpi=130)
    with open(RESULTS / f"{stem}.json", "w") as fh:
        json.dump({"experiment": "broadcast group size under fading",
                   "mode": "NORMAL", "mod": "QPSK", "rate": "R12",
                   "fading_doppler_hz": args.fade_hz,
                   "payload_bytes": PAYLOAD_N, "groups": GROUPS,
                   "snr_db": SNRS, "trials": args.trials,
                   "delivered": rec}, fh, indent=2)
    print(f"  saved {RESULTS / stem}.png and {stem}.json")


if __name__ == "__main__":
    main()
