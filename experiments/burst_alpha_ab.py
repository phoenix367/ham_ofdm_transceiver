"""Is the once-per-group LLR temperature worth refitting per block? No.

Kept as the record of a negative result, and as the harness that proved
it. The demo application once delivered a 400-byte broadcast complete at
4 frames per group and as "5 frames, 12 lost" at 16, which looked like a
group-size limit in the DSP. The LLR temperature was the leading
suspect: the fixed/C receiver fits alpha ONCE from the group's header
(fixed/rx.py receive_stream) and applies it to every block, and it is
not a harmless scale there -- _calibrated_llrs uses alpha to INDEX a
nonlinear reliability ROM that saturates at 31, so an alpha suiting the
header but not a later block moves that block's LLRs onto the wrong part
of the curve. (In the float chain alpha IS a harmless scale: llr_recal
defaults to None and a uniform positive scale cannot change a Viterbi
survivor path. So this experiment only makes sense on the fixed chain.)

Four arms, identical walks apart from where alpha comes from and whether
the ZC is re-locked:

  group     alpha fitted once from the header               (what ships)
  oracle    alpha refitted per block from that block's OWN true bits --
            not implementable, it needs the answer; it is the UPPER
            BOUND on what any per-block refit could ever buy
  causal    alpha refitted from the last block that decoded
  openloop  header alpha, and the ZC stepped over WITHOUT re-locking,
            which is what the C rxs_continue_burst actually does

RESULT (12 trials/point): all four arms tie. 100% of blocks delivered at
every SNR from 0 to +12 dB at both group sizes, and 99.0/99.0/99.0/98.4%
at -2 dB. The oracle arm buying nothing is the whole point: no
implementable refit can beat it, so per-block refitting is dead. The
open-loop arm costs 0.6 points at -2 dB and nothing above it.

Since the DSP delivers a 16-block group perfectly 11 dB below where the
application was failing, the cause was never here. It was two bugs in
demoapp/app.c (SYNC descriptor parsed after the block counter was armed;
EOS not ending the walk) -- see the BC_GROUP_CAP comment there.

Run:  ./venv/bin/python experiments/burst_alpha_ab.py [--trials N]
Outputs: results/burst_alpha_ab.png + .json
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

from ofdm_phy import (Data, ModType, CCSpeed, LinkMode,  # noqa: E402
                      simulate_channel)
from ofdm_phy.packets import Header, PACKET_CLASSES  # noqa: E402
from ofdm_phy.transceiver import (CODECS, MAPPERS,  # noqa: E402
                                  HEADER_CODEC, HEADER_MAPPER)
from ofdm_phy.fixed.tx import FixedTransmitter  # noqa: E402
from ofdm_phy.fixed.rx import FixedReceiver  # noqa: E402

RESULTS = ROOT / "results"
RESULTS.mkdir(exist_ok=True)

FS = 12000
MODE = LinkMode.NORMAL
MOD, SPD = ModType.QPSK, CCSpeed.R23      # ladder rung 8, as the demo used
RESYNC = 4
GROUPS = [4, 16]
SNRS = [-2.0, 0.0, 2.0, 5.0, 8.0, 12.0]

# arm -> (alpha policy, re-lock the ZC at a resync?).  The last arm is the
# demo application's actual receiver: rxs_continue_burst steps over the ZC
# symbol without re-locking, which is the ONE thing a 16-block group does
# three times and a 4-block group never does at all.
ARMS = {
    "group":    ("group",  True),
    "oracle":   ("oracle", True),
    "causal":   ("causal", True),
    "openloop": ("group",  False),
}


def fit_alpha_ref(rx, blk64, ref):
    """_fit_alpha_q12 generalized to any block with a known reference.

    Same integer arithmetic; the stock method hardcodes the header's."""
    peak = int(np.max(np.abs(blk64))) if len(blk64) else 0
    fit_shift = max(0, peak.bit_length() - 20)
    hf = blk64[:len(ref)] >> fit_shift
    lx = hf * ref
    n = len(lx)
    m = int(np.sum(lx))
    ssq = int(np.sum(lx * lx))
    den = n * ssq - m * m
    if den <= 0 or m <= 0:
        return None, fit_shift
    return max((2 * m * n << 12) // den, 1), fit_shift


def walk(rx, samples, n_blocks, resync_every, policy, truth, relock=True):
    """The receive_stream walk with a pluggable alpha policy.

    Mirrors fixed/rx.py receive_stream exactly apart from the alpha
    source, so a difference between arms is attributable to alpha alone.
    Returns a per-block ok/fail list.
    """
    i_arr, q_arr = rx.hilbert.analytic(np.asarray(samples, dtype=np.int64))
    det = rx._detect(i_arr, q_arr)
    if det is None:
        return None
    start, cfo_word = det
    rx._last_hyp = None
    pad = np.zeros(rx.symbol_len, dtype=np.int64)
    i_arr = np.concatenate([i_arr, pad])
    q_arr = np.concatenate([q_arr, pad])

    n_hdr = -(-HEADER_CODEC.calc_cc_elements(Header.PACKET_SIZE) //
              (rx._m.data_carriers_len * HEADER_MAPPER.MU))
    try:
        h64, scale_h = rx._demod_block(i_arr, q_arr, start, cfo_word,
                                       n_hdr, HEADER_MAPPER)
        hdr_bits = rx._decode_block(rx._quantize6(h64), HEADER_CODEC,
                                    Header.PACKET_SIZE)
        header = Header.decode(hdr_bits, check_crc=True)
    except Exception:
        return None

    codec = CODECS[header.spd]
    mapper = MAPPERS[header.mod]
    n_data = -(-codec.calc_cc_elements(header.len) //
               (rx._m.data_carriers_len * mapper.MU))
    block_len = n_data * rx.symbol_len
    cap_d = rx._m.data_carriers_len * mapper.MU

    hdr_alpha, hdr_shift = rx._fit_alpha_q12(h64, hdr_bits)
    # the alpha in force, and the domain its fit_shift refers to
    cur = (hdr_alpha, scale_h + hdr_shift)

    pos = start + n_hdr * rx.symbol_len
    out, resyncs = [], []
    for k in range(n_blocks):
        if resync_every and k and k % resync_every == 0:
            if relock:
                pos, cfo_word = rx._stream_resync(i_arr, q_arr, pos, cfo_word,
                                                  k, resyncs)
            else:
                # what rxs_continue_burst does: step over the ZC symbol
                # without re-locking timing or updating the CFO word
                pos += rx.symbol_len
        if pos + block_len > len(i_arr):
            out.append(False)
            continue
        d64, scale_d = rx._demod_block(i_arr, q_arr, pos, cfo_word,
                                       n_data, mapper)

        if policy == "oracle":
            ref = rx._known_ref(codec.encode(truth[k].astype(np.uint8)), cap_d)
            a, fs_ = fit_alpha_ref(rx, d64, ref)
            use = (a, scale_d + fs_) if a else cur
        else:
            use = cur

        alpha, dom = use
        llrs = (rx._calibrated_llrs(d64, scale_d, alpha, dom) if alpha
                else rx._quantize_data(d64, mapper.MU))
        ok = False
        try:
            b = rx._decode_block(llrs, codec, header.len)
            PACKET_CLASSES[header.typ].decode(b, check_crc=True)
            ok = True
            if policy == "causal":   # learn from what just decoded
                ref = rx._known_ref(codec.encode(b.astype(np.uint8)), cap_d)
                a, fs_ = fit_alpha_ref(rx, d64, ref)
                if a:
                    cur = (a, scale_d + fs_)
        except Exception:
            pass
        out.append(ok)
        pos += block_len
    return out


def run(trials):
    rec = {}
    for group in GROUPS:
        pkts = [Data(reserved=k, payload=bytes([(k * 37 + j) & 0xFF
                                                for j in range(26)]))
                for k in range(group)]
        tx = FixedTransmitter(MODE)
        sig = tx.build_stream(pkts, mod=MOD, spd=SPD, resync_every=RESYNC)
        truth = [p.encode() for p in pkts]
        print(f"  group {group}: {len(sig)/FS:.1f} s per transmission")
        for snr in SNRS:
            acc = {p: [0, 0] for p in ARMS}
            per_idx = {p: np.zeros(group) for p in acc}
            for t in range(trials):
                r = np.asarray(
                    simulate_channel(sig.astype(float), 900, 0.0, FS,
                                     snr_db=snr,
                                     rng=np.random.default_rng([11, t])),
                    dtype=np.int64)
                for pol in acc:
                    rx = FixedReceiver(MODE, calibrate=True)
                    res = walk(rx, r, group, RESYNC, ARMS[pol][0], truth,
                               relock=ARMS[pol][1])
                    if res is None:
                        acc[pol][1] += group
                        continue
                    acc[pol][0] += sum(res)
                    acc[pol][1] += len(res)
                    per_idx[pol] += np.asarray(res, dtype=float)
            rec[f"g{group}_s{snr}"] = {
                p: acc[p][0] / max(acc[p][1], 1) for p in acc}
            rec[f"g{group}_s{snr}_idx"] = {
                p: (per_idx[p] / trials).tolist() for p in acc}
            print(f"    {snr:+5.1f} dB  " + "  ".join(
                f"{p} {100*rec[f'g{group}_s{snr}'][p]:5.1f}%"
                for p in ARMS), flush=True)
    return rec


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trials", type=int, default=20)
    args = ap.parse_args()
    rec = run(args.trials)

    fig, axes = plt.subplots(1, len(GROUPS), figsize=(11, 4.2), sharey=True)
    for ax, group in zip(axes, GROUPS):
        for pol, style in (("group", "o-"), ("oracle", "s--"),
                           ("causal", "^:"), ("openloop", "v-")):
            ax.plot(SNRS, [100 * rec[f"g{group}_s{s}"][pol] for s in SNRS],
                    style, label=pol)
        ax.set_title(f"{group} blocks per group", fontsize=10)
        ax.set_xlabel("SNR, dB")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        ax.axvline(-3.8, color="k", ls=":", lw=0.8, alpha=0.6)
    axes[0].set_ylabel("blocks delivered, %")
    fig.suptitle("Per-group vs per-block LLR temperature in a streamed burst "
                 "(NORMAL QPSK 2/3; oracle = upper bound)", fontsize=10)
    fig.tight_layout()
    stem = ("burst_alpha_ab" if args.trials == 20
            else f"burst_alpha_ab_n{args.trials}")
    fig.savefig(RESULTS / f"{stem}.png", dpi=130)
    with open(RESULTS / f"{stem}.json", "w") as fh:
        json.dump({"experiment": "per-block LLR alpha refit A/B",
                   "mode": "NORMAL", "mod": "QPSK", "rate": "R23",
                   "resync_every": RESYNC, "groups": GROUPS,
                   "snr_db": SNRS, "trials": args.trials,
                   "delivered": rec}, fh, indent=2)
    print(f"  saved {RESULTS / stem}.png and {stem}.json")


if __name__ == "__main__":
    main()
