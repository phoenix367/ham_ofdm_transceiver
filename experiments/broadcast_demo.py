"""Broadcast (non-ARQ) mode: what gets through when nothing is repeated.

Speech and telemetry cannot wait for retransmission, so the interesting
questions are not "did it all arrive" but: what fraction arrives, does a
receiver that tunes in late catch up, and what does the sender learn
from the one report it gets back.

    ./venv/bin/python experiments/broadcast_demo.py [--trials N]

Outputs results/broadcast_demo.png + .json. Non-default trial counts get
suffixed filenames so smoke runs cannot clobber the reference.
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

from ofdm_phy import LinkMode, ModType, CCSpeed, simulate_channel  # noqa
from ofdm_phy.broadcast import (BroadcastTx, BroadcastRx,  # noqa
                                PT_TELEMETRY, PT_CODEC2_700)

RESULTS = ROOT / "results"
RESULTS.mkdir(exist_ok=True)
FS = 12000

# rung 12 (QAM16 3/4, 1059 bit/s user) is the only place Codec2 700
# leaves room for framing; rung 7 (QPSK 1/2, 353 bit/s) is telemetry
CASES = [
    ("speech @ rung 12", ModType.QAM16, CCSpeed.R34, PT_CODEC2_700,
     np.arange(2.0, 11.1, 1.5)),
    ("telemetry @ rung 7", ModType.QPSK, CCSpeed.R12, PT_TELEMETRY,
     np.arange(-6.0, 3.1, 1.5)),
]


def run_point(mod, spd, ptype, snr_db, payload, trials, group, seed):
    """Returns (delivery fraction, bytes recovered / bytes sent)."""
    tx = BroadcastTx(LinkMode.NORMAL, mod, spd, group=group)
    rx = BroadcastRx(LinkMode.NORMAL, group=group)
    audio = np.concatenate(tx.build(payload, ptype=ptype)).astype(float)
    deliv, frac = [], []
    for k in range(trials):
        rng = np.random.default_rng([seed, k])
        got, st = rx.receive(simulate_channel(audio, 300, 3.0, FS,
                                              snr_db=snr_db, rng=rng))
        deliv.append(st.delivery if st.expected else 0.0)
        frac.append(len(got) / len(payload))
    return float(np.mean(deliv)), float(np.mean(frac))


def late_join(group=4, n_bytes=600):
    """A receiver that starts listening part-way through must still
    acquire: every group re-sends the preamble and a SYNC descriptor, so
    the joiner picks up at the next group boundary rather than waiting
    for the broadcast to end. It cannot recover what it did not hear --
    there is no retransmission -- so the measure is how much of the
    REMAINDER it catches."""
    payload = bytes((i * 37 + 11) & 0xFF for i in range(n_bytes))
    tx = BroadcastTx(LinkMode.NORMAL, ModType.QAM16, CCSpeed.R34,
                     group=group)
    rx = BroadcastRx(LinkMode.NORMAL, group=group)
    groups = tx.build(payload, ptype=PT_CODEC2_700)
    audio = np.concatenate(groups).astype(float)
    cut = len(groups[0]) // 2          # half-way through the FIRST group
    remaining = sum(len(g) for g in groups[1:])
    rng = np.random.default_rng(7)
    late = simulate_channel(audio[cut:], 300, 3.0, FS, snr_db=12.0, rng=rng)
    got, st = rx.receive(late)
    return st, len(got), len(payload), len(groups)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trials", type=int, default=20)
    ap.add_argument("--group", type=int, default=4)
    ap.add_argument("--bytes", type=int, default=200)
    args = ap.parse_args()

    payload = bytes((i * 37 + 11) & 0xFF for i in range(args.bytes))
    record = {"experiment": "broadcast (non-ARQ) delivery",
              "payload_bytes": args.bytes, "group": args.group,
              "trials": args.trials, "cases": []}

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    for ax, (name, mod, spd, ptype, snrs) in zip(axes, CASES):
        tx = BroadcastTx(LinkMode.NORMAL, mod, spd, group=args.group)
        air = tx.air_time(args.bytes)
        print(f"  {name}: {args.bytes} B in {air:.1f} s "
              f"({8 * args.bytes / air:.0f} bit/s delivered if clean)")
        d, f = [], []
        for snr in snrs:
            dd, ff = run_point(mod, spd, ptype, float(snr), payload,
                               args.trials, args.group, seed=41)
            d.append(dd)
            f.append(ff)
            print(f"    {snr:+5.1f} dB: {100 * dd:5.1f}% of frames, "
                  f"{100 * ff:5.1f}% of bytes", flush=True)
        ax.plot(snrs, d, "o-", color="tab:blue", label="frames delivered")
        ax.plot(snrs, f, "s--", color="tab:green", label="payload recovered")
        ax.set_xlabel("SNR, dB (6 kHz convention)")
        ax.set_ylabel("fraction")
        ax.set_title(f"{name}\n{args.bytes} B in {air:.1f} s", fontsize=10)
        ax.set_ylim(-0.03, 1.03)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        record["cases"].append({
            "name": name, "modulation": mod.name, "rate": spd.name,
            "payload_type": ptype, "air_seconds": air,
            "snr_db": [float(x) for x in snrs],
            "frames_delivered": d, "payload_recovered": f})

    st, got, total, ngroups = late_join(args.group)
    print(f"\n  late join (tuned in half-way through group 1 of {ngroups}): "
          f"{st.report()}")
    print(f"    recovered {got} of {total} B -- the first group was already "
          f"gone when it started listening, and nothing is retransmitted")
    record["late_join"] = {"groups_in_broadcast": ngroups,
                           "groups_acquired": st.groups,
                           "frames_ok": st.frames_ok,
                           "frames_lost": st.frames_lost,
                           "bytes": got, "of": total, "ptype": st.ptype}

    fig.suptitle("Broadcast mode: non-ARQ delivery, nothing retransmitted",
                 fontsize=11)
    fig.tight_layout()
    stem = "broadcast_demo" if args.trials == 20 \
        else f"broadcast_demo_n{args.trials}"
    fig.savefig(RESULTS / f"{stem}.png", dpi=130)
    with open(RESULTS / f"{stem}.json", "w") as fh:
        json.dump(record, fh, indent=2)
    print(f"  saved {RESULTS / stem}.png and {stem}.json")


if __name__ == "__main__":
    main()
