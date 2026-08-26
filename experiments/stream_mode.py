"""Streamed bursts vs per-frame preambles -- the reproduction entry point
for the streaming mode added to `Transceiver.build_stream` /
`demod_stream`.

What it measures: the same N packets delivered two ways over the same
channel, at NORMAL. The BASELINE sends N independent frames, each paying
its own tone field, ZC and header (0.637 s of fixed cost per frame). The
STREAM sends one preamble and one header, then the N data blocks back to
back with a ZC resync every `--resync` blocks. Both arms are scored per
block (CRC), so the comparison is packets delivered, not frames sent.

Key expected behaviour: the stream is ~1.7x faster for 20 x 27-byte
blocks and costs ~0.2 dB of sensitivity. The cost is real but small --
it comes from carrying ONE preamble-derived CFO estimate across the
whole burst instead of re-estimating per frame; the per-symbol frequency
search and the pilot channel estimate do the rest of the tracking, which
is why an open-loop stream (--resync 0) still decodes.

Run:  ./venv/bin/python experiments/stream_mode.py [--trials N]
Outputs: results/stream_mode.png + results/stream_mode.json (all
parameters, per-point block counts, air times and the fitted dB
penalty). Non-default trial counts get suffixed filenames so smoke runs
cannot clobber the reference.
~5 min at 300 trials on 8 cores (parallel over SNR points).
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

from ofdm_phy import (  # noqa: E402
    Transceiver, Data, ModType, CCSpeed, LinkMode, simulate_channel,
)
from ofdm_phy.modes import make_modem  # noqa: E402

RESULTS = ROOT / "results"
RESULTS.mkdir(exist_ok=True)

FS = 12000
SNRS = np.arange(-5.5, -3.9, 0.5)
MODE = LinkMode.NORMAL
MOD = ModType.QPSK
SPD = CCSpeed.R12
BLOCKS = 20
PAYLOAD = 27


def _cell(args):
    """One SNR point: `trials` bursts each way, scored per block."""
    si, snr_db, trials, blocks, resync = args
    trx = Transceiver(make_modem(MODE))
    packets = [Data(reserved=123, payload=bytes([65 + (k % 26)]) * PAYLOAD)
               for k in range(blocks)]
    want = [bytes(p.payload) for p in packets]

    ok_stream = ok_frames = 0
    t_stream = t_frames = 0.0
    rng = np.random.default_rng([37, si])

    for _ in range(trials):
        tx = trx.build_stream(packets, mod=MOD, spd=SPD, resync_every=resync)
        t_stream += len(tx) / FS
        rx = simulate_channel(tx, 300, 3.0, FS, snr_db=snr_db, rng=rng)
        try:
            got, _ = trx.demod_stream(rx, n_blocks=blocks, resync_every=resync)
        except Exception:
            got = [None] * blocks
        ok_stream += sum(1 for g, w in zip(got, want)
                         if g is not None and bytes(g.payload) == w)

        for pkt, w in zip(packets, want):
            frame = trx.build_frame(pkt, mod=MOD, spd=SPD)
            t_frames += len(frame) / FS
            rxf = simulate_channel(frame, 300, 3.0, FS, snr_db=snr_db, rng=rng)
            try:
                got1, _ = trx.demod_frame(rxf)
                ok_frames += bytes(got1.payload) == w
            except Exception:
                pass

    return si, ok_stream, ok_frames, t_stream / trials, t_frames / trials


def _penalty_db(snrs, stream_psr, frame_psr):
    """How far the stream curve sits to the right of the per-frame curve,
    in dB, averaged over the points where both are informative."""
    gaps = []
    for s, p in zip(snrs, frame_psr):
        if not 0.05 < p < 0.98:
            continue
        # SNR at which the stream reaches the same PSR
        order = np.argsort(stream_psr)
        equiv = np.interp(p, np.asarray(stream_psr)[order],
                          np.asarray(snrs)[order])
        gaps.append(equiv - s)
    return float(np.mean(gaps)) if gaps else float("nan")


def run(trials=300, blocks=BLOCKS, resync=4):
    import multiprocessing as mp

    trx = Transceiver(make_modem(MODE))
    pkt_bits = len(Data(reserved=123, payload=b"x" * PAYLOAD).encode())
    layout = trx.stream_layout(pkt_bits, MOD, SPD, blocks, resync_every=resync)

    print(f"  {MODE.name} {MOD.name} {SPD.name}, {blocks} x {PAYLOAD} B, "
          f"ZC resync every {resync}, {trials} trials/point")
    ok_s = np.zeros(len(SNRS))
    ok_f = np.zeros(len(SNRS))
    air_s = np.zeros(len(SNRS))
    air_f = np.zeros(len(SNRS))
    cells = [(si, float(s), trials, blocks, resync) for si, s in enumerate(SNRS)]
    with mp.Pool(max(1, min(len(cells), mp.cpu_count() - 1))) as pool:
        for si, os_, of_, ts, tf in pool.imap_unordered(_cell, cells):
            ok_s[si], ok_f[si], air_s[si], air_f[si] = os_, of_, ts, tf
            n = trials * blocks
            print(f"  {SNRS[si]:+.1f} dB: stream {os_:5d}/{n} "
                  f"frames {of_:5d}/{n}  ({ts:.2f} s vs {tf:.2f} s)",
                  flush=True)

    n = trials * blocks
    psr_s, psr_f = ok_s / n, ok_f / n
    speedup = float(np.mean(air_f / air_s))
    penalty = _penalty_db(SNRS, psr_s, psr_f)
    print(f"\n  air time {air_s[0]:.2f} s vs {air_f[0]:.2f} s -> "
          f"{speedup:.2f}x")
    print(f"  sensitivity cost {penalty:+.2f} dB")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.4))
    ax1.plot(SNRS, psr_f, "o-", color="tab:blue",
             label="per-frame preambles")
    ax1.plot(SNRS, psr_s, "s-", color="tab:green",
             label=f"streamed (ZC every {resync})")
    ax1.set_xlabel("SNR, dB (6 kHz convention)")
    ax1.set_ylabel("P(block delivered)")
    ax1.set_title(f"Delivery, {blocks} x {PAYLOAD} B at "
                  f"{MODE.name} {MOD.name} {SPD.name}")
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=9)
    ax1.annotate(f"{penalty:+.2f} dB", xy=(0.5, 0.12),
                 xycoords="axes fraction", fontsize=10, ha="center",
                 bbox=dict(boxstyle="round,pad=0.3", fc="#ffe9a8", lw=0))

    good_f = psr_f * n / trials * PAYLOAD * 8 / air_f
    good_s = psr_s * n / trials * PAYLOAD * 8 / air_s
    ax2.plot(SNRS, good_f, "o-", color="tab:blue", label="per-frame preambles")
    ax2.plot(SNRS, good_s, "s-", color="tab:green", label="streamed")
    ax2.set_xlabel("SNR, dB (6 kHz convention)")
    ax2.set_ylabel("delivered goodput, bit/s")
    ax2.set_title(f"Goodput ({speedup:.2f}x the air time saved)")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=9)

    fig.suptitle("Streamed bursts vs per-frame preambles "
                 f"({trials} bursts/point, article channel)", fontsize=11)
    fig.tight_layout()

    stem = "stream_mode" if trials == 300 else f"stream_mode_n{trials}"
    fig.savefig(RESULTS / f"{stem}.png", dpi=130)
    record = {
        "experiment": "streamed bursts vs per-frame preambles",
        "mode": MODE.name, "modulation": MOD.name, "fec": "conv K=7 rate 1/2",
        "payload_bytes": PAYLOAD, "blocks_per_burst": blocks,
        "resync_every_blocks": resync,
        "n_bursts_per_point": trials,
        "snr_grid_db": [float(x) for x in SNRS],
        "snr_convention": "noise in the 6 kHz Nyquist band",
        "channel": "simulate_channel: offset 300, CFO +3 Hz plus the "
                   "model's quadratic drift, AWGN, article multipath",
        "seed_scheme": "np.random.default_rng([37, snr_index]); the same "
                       "generator feeds both arms in turn",
        "layout_samples": {k: int(v) for k, v in layout.items()
                           if k != "seconds"},
        "burst_seconds": {"streamed": float(air_s[0]),
                          "per_frame": float(air_f[0])},
        "speedup": speedup,
        "sensitivity_cost_db": penalty,
        "blocks_delivered": {"streamed": [int(x) for x in ok_s],
                             "per_frame": [int(x) for x in ok_f]},
        "blocks_per_point": int(n),
    }
    with open(RESULTS / f"{stem}.json", "w") as f:
        json.dump(record, f, indent=2)
    print("saved", RESULTS / f"{stem}.png", "and", f"{stem}.json")
    return record


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trials", type=int, default=300,
                    help="bursts per SNR point (default 300)")
    ap.add_argument("--blocks", type=int, default=BLOCKS,
                    help="packets per burst (default 20)")
    ap.add_argument("--resync", type=int, default=4,
                    help="ZC resync period in blocks, 0 = open loop")
    args = ap.parse_args()
    run(trials=args.trials, blocks=args.blocks, resync=args.resync)
