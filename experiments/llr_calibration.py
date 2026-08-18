"""Front-end LLR calibration: measure the miscalibration, validate the
header-based temperature fit, and re-measure the LDPC end-to-end gap with
calibrated sum-product decoding.

Run:  python experiments/llr_calibration.py
"""

import sys
from pathlib import Path

import numpy as np
from scipy.signal import hilbert

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ofdm_phy import Transceiver, Data, ModType, CCSpeed, LinkMode, make_modem, simulate_channel
from ofdm_phy.ldpc import LDPCCodec
from ofdm_phy.ofdm import freq_shift
from ofdm_phy.transceiver import CODECS, MAPPERS, HEADER_CODEC, HEADER_MAPPER


def collect(trx, snr, n_frames, rng):
    """Per-frame (alpha_header_fit, alpha_true_fit) + pooled (L, x) samples."""
    m = trx.modem
    alphas_hdr, alphas_true, pool = [], [], []
    for _ in range(n_frames):
        pkt = Data(reserved=123, payload=rng.bytes(27))
        sig = trx.build_frame(pkt, mod=ModType.BPSK, spd=CCSpeed.R13)
        rx = simulate_channel(sig, int(rng.integers(300, 1200)), float(rng.uniform(-60, 60)),
                              12000, snr_db=snr, rng=rng)
        try:
            d, st = trx.demod_frame(rx, check_crc=False)
        except Exception:
            continue
        alphas_hdr.append(st.llr_alpha)

        # ground truth: raw data-block LLRs vs the known transmitted stream
        analytic = hilbert(rx).astype(np.complex64)
        det = m.detect_preamble(analytic)
        corr = freq_shift(12000, analytic, det[1])
        corr = np.concatenate([corr, np.zeros(m.symbol_len, dtype=corr.dtype)])
        m.set_mapper(HEADER_MAPPER)
        pos = det[0] + 6 * m.symbol_len
        m.set_mapper(MAPPERS[ModType.BPSK])
        coded = CODECS[CCSpeed.R13].calc_cc_elements(len(pkt.encode()))
        n_syms = -(-coded // m.data_carriers_len)
        llrs, _, _ = trx._demod_symbols(corr, pos, n_syms)
        ref = 2.0 * trx._encode_block(pkt.encode(), CODECS[CCSpeed.R13],
                                      MAPPERS[ModType.BPSK]).ravel().astype(float) - 1.0
        lx = llrs[:len(ref)] * ref
        v = np.var(lx)
        alphas_true.append(float(np.clip(2 * np.mean(lx) / v, 0.05, 4.0)))
        pool.append(lx)
    return alphas_hdr, alphas_true, (np.concatenate(pool) if pool else np.array([]))


def reliability(lx, alpha, edges=(0, 2, 5, 10, 15, 20.01)):
    print(f"    {'|L| bin':>10} {'N':>6} {'empirical P(err)':>17} {'ideal P(err)':>13}")
    s = lx * alpha
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = (np.abs(s) >= lo) & (np.abs(s) < hi)
        if sel.sum() < 50:
            continue
        emp = float(np.mean(s[sel] < 0))
        mid = float(np.mean(np.abs(s[sel])))
        ideal = 1.0 / (1.0 + np.exp(mid))
        print(f"    {f'{lo:.0f}-{hi:.0f}':>10} {int(sel.sum()):>6} {emp:>17.3f} {ideal:>13.3f}")


def main():
    trx = Transceiver(make_modem(LinkMode.NORMAL))
    rng = np.random.default_rng(21)

    print("=== temperature vs SNR (NORMAL BPSK 1/3, 25 frames each) ===")
    print("alpha < 1 means the raw LLRs are overconfident")
    pools = {}
    for snr in (-5, -7, -9):
        ah, at, pool = collect(trx, snr, 25, rng)
        pools[snr] = pool
        print(f"  {snr:+d} dB: header-fit alpha {np.mean(ah):.2f}+-{np.std(ah):.2f}   "
              f"true-fit alpha {np.mean(at):.2f}+-{np.std(at):.2f}   ({len(ah)} frames)")

    print("\n=== reliability at -8 dB: raw vs calibrated ===")
    _, at8, pool8 = collect(trx, -8, 25, rng)
    a8 = float(np.mean(at8))
    print("  raw LLRs (alpha=1):")
    reliability(pool8, 1.0)
    print(f"  calibrated (alpha={a8:.2f}):")
    reliability(pool8, a8)

    print("\n=== LDPC end-to-end with calibration (40 trials) ===")
    for snr in (-8, -9):
        row = {}
        for label, fec, spa in (("conv", "cc", False),
                                ("ldpc min-sum", "ldpc", False),
                                ("ldpc SPA+cal", "ldpc", True)):
            LDPCCodec.USE_SPA = spa
            ok = 0
            r = np.random.default_rng(6)
            for _ in range(40):
                pkt = Data(reserved=123, payload=r.bytes(27))
                sig = trx.build_frame(pkt, mod=ModType.BPSK, spd=CCSpeed.R13, fec=fec)
                rx = simulate_channel(sig, int(r.integers(300, 1200)), float(r.uniform(-60, 60)),
                                      12000, snr_db=snr, rng=r)
                try:
                    d, st = trx.demod_frame(rx)
                    ok += d == pkt
                except Exception:
                    pass
            row[label] = ok
        LDPCCodec.USE_SPA = True
        print(f"  {snr:+d} dB: " + "   ".join(f"{k} {v}/40" for k, v in row.items()))


if __name__ == "__main__":
    main()
