"""Reproduce Figures 23-24: BER and PER vs SNR for every modulation and coding
rate, over the article's channel model (AWGN + multipath [1, 0, 0.4, 0, 0, 0.2]
+ quadratic CFO drift + BSC bit flips + BEC erasures), with random time offset
and carrier frequency offset per packet.

Run:  python experiments/ber_per_simulation.py [--trials N] [--modem newman|stf]
Outputs: results/ber_per[_stf].json, results/ber_vs_snr[_stf].png,
         results/per_vs_snr[_stf].png
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

from ofdm_phy import (Transceiver, Data, ModType, CCSpeed, simulate_channel,
                      FullOFDMModem, STFOFDMModem)

MODEMS = {"newman": FullOFDMModem, "stf": STFOFDMModem}

PAYLOAD_LEN = 27  # bytes, as in the article's on-air test
SNR_GRID = {
    ModType.BPSK: list(range(-12, 0)),
    ModType.QPSK: list(range(-9, 3)),
}
CONFIGS = [(mod, spd) for mod in ModType for spd in CCSpeed]

_TRX = {}


def _get_trx(modem_name):
    if modem_name not in _TRX:
        _TRX[modem_name] = Transceiver(MODEMS[modem_name]())
    return _TRX[modem_name]


def run_chunk(args):
    """Run a chunk of trials for one (mod, spd, snr) point. Returns
    (packet_errors, bit_errors, total_bits, trials)."""
    mod_v, spd_v, snr_db, seed, trials, modem_name = args
    mod, spd = ModType(mod_v), CCSpeed(spd_v)

    trx = _get_trx(modem_name)
    fs = trx.modem.sample_rate
    rng = np.random.default_rng(seed)

    pkt_errors = 0
    bit_errors = 0
    total_bits = 0

    for _ in range(trials):
        payload = rng.bytes(PAYLOAD_LEN)
        packet = Data(reserved=123, payload=payload)
        tx_bits = packet.encode()
        n_bits = len(tx_bits)

        signal = trx.build_frame(packet, mod=mod, spd=spd)
        rx = simulate_channel(
            signal,
            time_shift=int(rng.integers(200, 1500)),
            freq_shift_hz=float(rng.uniform(-100, 100)),
            sample_rate=fs,
            snr_db=snr_db,
            rng=rng,
        )

        total_bits += n_bits
        try:
            decoded, _ = trx.demod_frame(rx, check_crc=False)
            rx_bits = decoded.encode()
            if len(rx_bits) == n_bits:
                errs = int(np.sum(rx_bits != tx_bits))
            else:
                errs = n_bits // 2
        except Exception:
            decoded, errs = None, n_bits // 2

        bit_errors += errs
        if decoded != packet:
            pkt_errors += 1

    return pkt_errors, bit_errors, total_bits, trials


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=120, help="packets per SNR point")
    ap.add_argument("--chunk", type=int, default=15)
    ap.add_argument("--modem", choices=list(MODEMS), default="newman")
    args = ap.parse_args()

    results_dir = ROOT / "results"
    results_dir.mkdir(exist_ok=True)

    jobs = []
    for mod, spd in CONFIGS:
        for snr in SNR_GRID[mod]:
            n_chunks = -(-args.trials // args.chunk)
            for c in range(n_chunks):
                trials = min(args.chunk, args.trials - c * args.chunk)
                seed = hash((mod.value, spd.value, snr, c)) & 0x7FFFFFFF
                jobs.append((mod.value, spd.value, snr, seed, trials, args.modem))

    print(f"{len(CONFIGS)} configs, {args.trials} packets/point, {len(jobs)} chunks, "
          f"modem={args.modem}")
    t0 = time.time()

    acc = {}
    with ProcessPoolExecutor() as pool:
        for job, res in zip(jobs, pool.map(run_chunk, jobs, chunksize=2)):
            mod_v, spd_v, snr = job[0], job[1], job[2]
            key = (mod_v, spd_v, snr)
            pe, be, tb, tr = res
            a = acc.setdefault(key, [0, 0, 0, 0])
            a[0] += pe
            a[1] += be
            a[2] += tb
            a[3] += tr

    print(f"simulation finished in {time.time() - t0:.1f} s")

    curves = {}
    for (mod, spd) in CONFIGS:
        label = f"{mod.name} {['1/3', '1/2', '2/3', '3/4'][spd.value]}"
        snrs, bers, pers = [], [], []
        for snr in SNR_GRID[mod]:
            pe, be, tb, tr = acc[(mod.value, spd.value, snr)]
            snrs.append(snr)
            bers.append(be / tb)
            pers.append(pe / tr)
        curves[label] = {"snr": snrs, "ber": bers, "per": pers}
        print(f"\n{label}")
        print("  SNR:", " ".join(f"{s:7d}" for s in snrs))
        print("  BER:", " ".join(f"{b:7.4f}" for b in bers))
        print("  PER:", " ".join(f"{p:7.3f}" for p in pers))

    suffix = "" if args.modem == "newman" else f"_{args.modem}"
    with open(results_dir / f"ber_per{suffix}.json", "w") as fh:
        json.dump(curves, fh, indent=2)

    # sensitivity: PER = 10% crossing for BPSK 1/3
    c = curves["BPSK 1/3"]
    snr_arr, per_arr = np.array(c["snr"], float), np.array(c["per"], float)
    sens = None
    for i in range(len(snr_arr) - 1):
        if per_arr[i] >= 0.1 >= per_arr[i + 1]:
            sens = snr_arr[i] + (per_arr[i] - 0.1) / (per_arr[i] - per_arr[i + 1] + 1e-12)
            break
    if sens is not None:
        print(f"\nSensitivity (BPSK 1/3, PER=10%): SNR ~ {sens:.1f} dB (article: ~ -7.5 dB)")

    plot(curves, results_dir, sens, suffix)


def plot(curves, results_dir, sens, suffix=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    label_sfx = " - STF preamble" if suffix else ""
    for metric, fname, title in (
            ("ber", f"ber_vs_snr{suffix}.png", f"BER vs SNR (Figure 23){label_sfx}"),
            ("per", f"per_vs_snr{suffix}.png", f"PER vs SNR (Figure 24){label_sfx}")):
        fig, ax = plt.subplots(figsize=(9, 6))
        for label, c in curves.items():
            style = "-o" if label.startswith("BPSK") else "--s"
            vals = np.maximum(c[metric], 1e-4)
            ax.semilogy(c["snr"], vals, style, ms=4, label=label)
        if metric == "per":
            ax.axhline(0.1, color="gray", ls=":", lw=1)
            ax.text(ax.get_xlim()[0], 0.11, " PER = 10%", color="gray", fontsize=9)
            if sens is not None:
                ax.axvline(sens, color="red", ls=":", lw=1)
                ax.text(sens, 2e-4, f" {sens:.1f} dB", color="red", fontsize=9)
        ax.set_xlabel("SNR, dB")
        ax.set_ylabel(metric.upper())
        ax.set_title(title)
        ax.grid(True, which="both", alpha=0.3)
        ax.legend(ncol=2, fontsize=9)
        fig.tight_layout()
        fig.savefig(results_dir / fname, dpi=130)
        print(f"saved {results_dir / fname}")


if __name__ == "__main__":
    main()
