"""Re-measure every ladder rung's PER-10% sensitivity with the front-end
LLR recalibration enabled (llr_recal="auto", as the link-layer station runs).

Per-rung SNR grids: NORMAL BPSK/QPSK rungs extend 4 dB below the old
sensitivity (recalibration moves them); ROBUST/EXTREME and 16-QAM rungs
get a +-2 dB bracket (validation showed little/no shift there).

Run:  python experiments/ladder_sweep.py [--trials N]
Outputs: results/ladder_recal.json + old-vs-new table
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
from ofdm_phy.link import LADDER
from ofdm_phy.modes import LinkMode

_TRX = {}


def _grid(idx, rung):
    s = rung.sens_db
    if rung.mode is LinkMode.NORMAL and rung.mod.value <= 1:  # BPSK/QPSK
        return [round(s + d) for d in range(-4, 2)]
    return [round(s + d) for d in range(-2, 3)]


def run_chunk(args):
    rung_idx, snr_db, seed, trials = args
    rung = LADDER[rung_idx]
    if rung.mode not in _TRX:
        t = Transceiver(make_modem(rung.mode))
        t.llr_recal = "auto"
        _TRX[rung.mode] = t
    trx = _TRX[rung.mode]

    rng = np.random.default_rng(seed)
    errors = 0
    for _ in range(trials):
        pkt = Data(reserved=123, payload=rng.bytes(27))
        sig = trx.build_frame(pkt, mod=rung.mod, spd=rung.spd)
        rx = simulate_channel(sig, int(rng.integers(200, 1500)),
                              float(rng.uniform(-100, 100)), 12000,
                              snr_db=snr_db, rng=rng)
        try:
            dec, _ = trx.demod_frame(rx, check_crc=False)
            if dec != pkt:
                errors += 1
        except Exception:
            errors += 1
    return errors, trials


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=48)
    ap.add_argument("--chunk", type=int, default=6)
    args = ap.parse_args()

    jobs = []
    for idx, rung in enumerate(LADDER):
        for snr in _grid(idx, rung):
            for c in range(-(-args.trials // args.chunk)):
                trials = min(args.chunk, args.trials - c * args.chunk)
                seed = hash((idx, snr, c)) & 0x7FFFFFFF
                jobs.append((idx, snr, seed, trials))

    print(f"{len(LADDER)} rungs, {args.trials} packets/point, {len(jobs)} chunks")
    t0 = time.time()
    acc = {}
    with ProcessPoolExecutor() as pool:
        for job, (err, tr) in zip(jobs, pool.map(run_chunk, jobs, chunksize=1)):
            a = acc.setdefault((job[0], job[1]), [0, 0])
            a[0] += err
            a[1] += tr
    print(f"finished in {time.time() - t0:.1f} s\n")

    out = {}
    print(f"{'rung':>4} {'config':28} {'old sens':>9} {'new sens':>9} {'delta':>7}")
    for idx, rung in enumerate(LADDER):
        snrs = _grid(idx, rung)
        pers = [acc[(idx, s)][0] / acc[(idx, s)][1] for s in snrs]
        sens = None
        for i in range(len(snrs) - 1):
            if pers[i] >= 0.1 >= pers[i + 1]:
                sens = snrs[i] + (pers[i] - 0.1) / (pers[i] - pers[i + 1] + 1e-12)
        if sens is None and pers[0] < 0.1:
            sens = float(snrs[0])  # sensitivity below the scanned bracket
        name = f"{rung.mode.name} {rung.mod.name} " \
               f"{['1/3', '1/2', '2/3', '3/4'][rung.spd.value]}"
        out[idx] = {"config": name, "snr": snrs, "per": pers, "sens": sens,
                    "old_sens": rung.sens_db}
        d = f"{sens - rung.sens_db:+.1f}" if sens is not None else "  n/a"
        sens_s = f"{sens:+.1f}" if sens is not None else "  n/a"
        mark = " (<= bracket floor)" if sens is not None and sens == snrs[0] else ""
        print(f"{idx:>4} {name:28} {rung.sens_db:>+8.1f} {sens_s:>9} {d:>7}{mark}")
        print(f"       PER: " + "  ".join(f"{s:+d}:{p:.2f}" for s, p in zip(snrs, pers)))

    with open(ROOT / "results" / "ladder_recal.json", "w") as fh:
        json.dump(out, fh, indent=2)
    print("\nsaved results/ladder_recal.json")


if __name__ == "__main__":
    main()
