"""What the coarse-CFO unwrap is worth -- the reproduction entry point
for the technical report's Figure "cfo_unwrap" (Section "Coarse-CFO
Ambiguity and the Two-Lag Unwrap").

The integer detector estimates its residual CFO from the tone field's
lag-N phase, which wraps beyond +-fs/2N = +-46.9 Hz -- exactly one
coarse bin. When the coarse mask-shift search slips to a neighbouring
bin (the metrics are a ~4% near-tie), the residual has to supply more
than the wrap boundary, lands 93.75 Hz out, and the Zadoff-Chu
correlation shifts ~34 samples: one more than the 32-sample cyclic
prefix, so the frame dies of inter-symbol interference. The fix takes
the lag-N/2 phase as well, unambiguous over +-fs/N, to choose the right
lag-N cycle.

This sweeps CFO across one bin with the unwrap on and off, measuring
both the cause (locks whose timing error exceeds half the CP) and the
effect (frames delivered). The failures cluster at MULTIPLES OF THE BIN
SPACING, where a one-bin slip needs a residual of exactly +-46.9 Hz --
the phase sits at +-pi, maximally ambiguous. That includes CFO ~ 0,
which is where the AFC netting loop drives a netted link, so the
worst-affected operating point is the one the system steers toward.

Run:  ./venv/bin/python experiments/cfo_unwrap.py [--trials N]
Outputs: results/cfo_unwrap.png + results/cfo_unwrap.json.
~6 min at 40 trials.
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

import ofdm_phy.fixed.rx as frx  # noqa: E402
from ofdm_phy.fixed.tx import FixedTransmitter  # noqa: E402
from ofdm_phy.fixed.rx import FixedReceiver  # noqa: E402
from ofdm_phy import (Data, ModType, CCSpeed, LinkMode,  # noqa: E402
                      simulate_channel)

RESULTS = ROOT / "results"
RESULTS.mkdir(exist_ok=True)
FS = 12000
BIN_HZ = FS / 256.0          # fixed NORMAL detection FFT is 256 bins
CFOS = [0.0, 3.0, 10.0, 20.0, 23.4, 30.0, 40.0, 46.9]
SNR_DB = -3.5


def run(trials):
    pkt = Data(reserved=123, payload=b"THROUGHPUT MEASUREMENT PKT")
    tx = FixedTransmitter(LinkMode.NORMAL)
    sig = tx.build_frame(pkt, mod=ModType.QPSK, spd=CCSpeed.R12)
    pre = sum(len(c) for c in tx._m.gen_preamble())
    out = {"off": {"ok": [], "bad_lock": []}, "on": {"ok": [], "bad_lock": []}}

    print(f"  QPSK 1/2 at {SNR_DB:+.1f} dB, {trials} frames/point; "
          f"coarse bin = {BIN_HZ:.1f} Hz")
    for cfo in CFOS:
        for tag, flag in (("off", False), ("on", True)):
            frx.RESIDUAL_UNWRAP = flag
            rx = FixedReceiver(LinkMode.NORMAL, calibrate=True)
            ok = bad = 0
            for k in range(trials):
                r = np.asarray(
                    simulate_channel(sig.astype(float), 900, cfo, FS,
                                     snr_db=SNR_DB,
                                     rng=np.random.default_rng([7, k])),
                    dtype=np.int64)
                # the cause: how far off is the lock?
                i_a, q_a = rx.hilbert.analytic(r)
                det = rx._detect(i_a, q_a)
                if det is not None:
                    err = (det[0] - rx.hilbert.delay) - (900 + pre)
                    bad += abs(err) > 16
                # the effect: did the frame survive?
                try:
                    d, h, st, cf = rx.receive(r)
                    ok += (d == pkt)
                except Exception:
                    pass
            out[tag]["ok"].append(ok / trials)
            out[tag]["bad_lock"].append(bad / trials)
        print(f"    {cfo:5.1f} Hz: delivered "
              f"{100*out['off']['ok'][-1]:5.1f}% -> "
              f"{100*out['on']['ok'][-1]:5.1f}%   bad locks "
              f"{100*out['off']['bad_lock'][-1]:5.1f}% -> "
              f"{100*out['on']['bad_lock'][-1]:5.1f}%", flush=True)
    frx.RESIDUAL_UNWRAP = True
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trials", type=int, default=40)
    ap.add_argument("--replot", action="store_true",
                    help="re-render from the saved JSON without measuring")
    args = ap.parse_args()
    if args.replot:
        with open(RESULTS / "cfo_unwrap.json") as fh:
            rec = json.load(fh)
        res = {k: {"ok": rec["delivered"][k], "bad_lock": rec["bad_locks"][k]}
               for k in ("off", "on")}
    else:
        res = run(args.trials)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))
    ax1.plot(CFOS, [100 * v for v in res["off"]["bad_lock"]], "o-",
             color="tab:red", label="lag-$N$ only (before)")
    ax1.plot(CFOS, [100 * v for v in res["on"]["bad_lock"]], "s-",
             color="tab:green", label="two-lag unwrap")
    ax1.set_ylabel("locks off by more than half a CP, %")
    ax1.set_title("Cause: timing outliers", fontsize=10)
    ax2.plot(CFOS, [100 * v for v in res["off"]["ok"]], "o-",
             color="tab:red", label="lag-$N$ only (before)")
    ax2.plot(CFOS, [100 * v for v in res["on"]["ok"]], "s-",
             color="tab:green", label="two-lag unwrap")
    ax2.set_ylabel("frames delivered, %")
    ax2.set_title("Effect: frames delivered", fontsize=10)
    for ax in (ax1, ax2):
        ax.set_xlabel("carrier frequency offset, Hz")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        for b in (0.0, BIN_HZ):
            ax.axvline(b, color="k", ls=":", lw=0.8, alpha=0.6)
    ax1.annotate("coarse bin\ncentres", xy=(BIN_HZ, ax1.get_ylim()[1] * 0.75),
                 xytext=(-58, 0), textcoords="offset points", fontsize=8,
                 color="0.35")
    # The coincidence IS the bug: the coarse bin spacing (fs/B) and the
    # lag-N wrap boundary (fs/2N) are the same 46.875 Hz, so a one-bin
    # slip needs a residual sitting exactly on the wrap.
    fig.suptitle("Coarse-CFO unwrap: failures cluster at bin centres, where "
                 f"a one-bin slip needs exactly $\\pm${BIN_HZ:.1f} Hz — and "
                 f"the lag-$N$ phase wraps at $\\pm${BIN_HZ:.1f} Hz",
                 fontsize=10)
    fig.tight_layout()
    stem = "cfo_unwrap" if args.trials == 40 else f"cfo_unwrap_n{args.trials}"
    fig.savefig(RESULTS / f"{stem}.png", dpi=130)
    with open(RESULTS / f"{stem}.json", "w") as fh:
        json.dump({"experiment": "coarse-CFO unwrap A/B",
                   "modulation": "QPSK", "rate": "R12", "mode": "NORMAL",
                   "snr_db": SNR_DB, "trials": args.trials,
                   "coarse_bin_hz": BIN_HZ, "cfo_hz": CFOS,
                   "delivered": {k: res[k]["ok"] for k in res},
                   "bad_locks": {k: res[k]["bad_lock"] for k in res}},
                  fh, indent=2)
    print(f"  saved {RESULTS / stem}.png and {stem}.json")


if __name__ == "__main__":
    main()
