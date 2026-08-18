"""Measure the fixed-point chain's 16-QAM sensitivity (the three QAM16
ladder rungs, NORMAL mode) and compare it A/B against the float receiver
on the same channel realizations.

Methodology mirrors ladder_sweep.py: article channel, 27-byte payloads,
48 packets/point, PER-10% sensitivity by linear interpolation. The frame
is built by the fixed TX (Q15 Gray constellation); the identical int16
samples go to both receivers, so the comparison isolates the RX chains.
The float RX runs llr_recal="auto" exactly as the ladder sweep did (the
reliability map is gated to MU<=2, so for 16-QAM this is the per-frame
temperature fit only).

Run:  python experiments/fixed_qam16_sweep.py [--trials N]
Outputs: results/fixed_qam16.json + comparison table
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
from ofdm_phy.packets import ModType
from ofdm_phy.fixed import FixedTransmitter, FixedReceiver

FS = 12000
QAM16_RUNGS = [i for i, r in enumerate(LADDER) if r.mod is ModType.QAM16]

_CTX = {}


def _grid(rung):
    s = round(rung.sens_db)
    return [s + d for d in range(-2, 4)]


def to_int16(x):
    return np.clip(x / np.max(np.abs(x)) * 0.9 * 32767, -32768, 32767).astype(np.int16)


def run_chunk(args):
    rung_idx, snr_db, seed, trials = args
    if not _CTX:
        _CTX["ftx"] = FixedTransmitter(LinkMode.NORMAL)
        _CTX["frx"] = FixedReceiver(LinkMode.NORMAL)
        flt = Transceiver(make_modem(LinkMode.NORMAL))
        flt.llr_recal = "auto"
        _CTX["flt"] = flt
    rung = LADDER[rung_idx]

    rng = np.random.default_rng(seed)
    err_fix = err_flt = 0
    for _ in range(trials):
        pkt = Data(reserved=123, payload=rng.bytes(27))
        sig = _CTX["ftx"].build_frame(pkt, mod=rung.mod, spd=rung.spd)
        rx = simulate_channel(sig.astype(np.float64), int(rng.integers(200, 1500)),
                              float(rng.uniform(-100, 100)), FS,
                              snr_db=snr_db, rng=rng)
        r16 = to_int16(rx)
        try:
            dec, *_ = _CTX["frx"].receive(r16)
            err_fix += dec != pkt
        except Exception:
            err_fix += 1
        try:
            dec2, _ = _CTX["flt"].demod_frame(r16.astype(np.float64) / 32768.0,
                                              check_crc=False)
            err_flt += dec2 != pkt
        except Exception:
            err_flt += 1
    return err_fix, err_flt, trials


def _sens(snrs, pers):
    sens = None
    for i in range(len(snrs) - 1):
        if pers[i] >= 0.1 >= pers[i + 1]:
            sens = snrs[i] + (pers[i] - 0.1) / (pers[i] - pers[i + 1] + 1e-12)
    if sens is None and pers[0] < 0.1:
        sens = float(snrs[0])
    return sens


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=48)
    ap.add_argument("--chunk", type=int, default=4)
    args = ap.parse_args()

    jobs = []
    for idx in QAM16_RUNGS:
        for snr in _grid(LADDER[idx]):
            for c in range(-(-args.trials // args.chunk)):
                trials = min(args.chunk, args.trials - c * args.chunk)
                seed = hash((idx, snr, c)) & 0x7FFFFFFF
                jobs.append((idx, snr, seed, trials))

    print(f"{len(QAM16_RUNGS)} QAM16 rungs, {args.trials} packets/point, "
          f"{len(jobs)} chunks")
    t0 = time.time()
    acc = {}
    with ProcessPoolExecutor() as pool:
        for job, (ef, el, tr) in zip(jobs, pool.map(run_chunk, jobs, chunksize=1)):
            a = acc.setdefault((job[0], job[1]), [0, 0, 0])
            a[0] += ef
            a[1] += el
            a[2] += tr
    print(f"finished in {time.time() - t0:.1f} s\n")

    out = {}
    print(f"{'rung':>4} {'config':22} {'ladder':>7} {'fixed':>7} {'float':>7}")
    for idx in QAM16_RUNGS:
        rung = LADDER[idx]
        snrs = _grid(rung)
        pers_fix = [acc[(idx, s)][0] / acc[(idx, s)][2] for s in snrs]
        pers_flt = [acc[(idx, s)][1] / acc[(idx, s)][2] for s in snrs]
        s_fix = _sens(snrs, pers_fix)
        s_flt = _sens(snrs, pers_flt)
        name = f"NORMAL QAM16 {['1/3', '1/2', '2/3', '3/4'][rung.spd.value]}"
        out[idx] = {"config": name, "snr": snrs,
                    "per_fixed": pers_fix, "per_float": pers_flt,
                    "sens_fixed": s_fix, "sens_float": s_flt,
                    "ladder_sens": rung.sens_db}
        fmt = lambda v: f"{v:+.1f}" if v is not None else "  n/a"
        print(f"{idx:>4} {name:22} {rung.sens_db:>+7.1f} {fmt(s_fix):>7} {fmt(s_flt):>7}")
        print("       fixed PER: " + "  ".join(
            f"{s:+d}:{p:.2f}" for s, p in zip(snrs, pers_fix)))
        print("       float PER: " + "  ".join(
            f"{s:+d}:{p:.2f}" for s, p in zip(snrs, pers_flt)))

    with open(ROOT / "results" / "fixed_qam16.json", "w") as fh:
        json.dump(out, fh, indent=2)
    print("\nsaved results/fixed_qam16.json")


if __name__ == "__main__":
    main()
