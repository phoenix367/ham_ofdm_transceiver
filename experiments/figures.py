"""Reproduce the article's illustrative figures:
  - ZC-sequence periodic/aperiodic autocorrelation (Figure 19)
  - TX signal oscillogram + spectrum (Figures 20/26)
  - BPSK/QPSK constellations before/after equalization at -9 and -3 dB SNR
    (Figures 21/22), plus 16-QAM at +1 and +5 dB (its operating range)

Run:  python experiments/figures.py
Outputs PNG files into results/.
"""

import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ofdm_phy import (
    Transceiver, Data, ModType, CCSpeed, simulate_channel, FullOFDMModem,
)
from ofdm_phy.ofdm import freq_shift
from scipy.signal import hilbert

RESULTS = ROOT / "results"
RESULTS.mkdir(exist_ok=True)


def fig_zc_acf():
    zc = FullOFDMModem._gen_zc_seq(17, 23)
    lags = np.arange(-22, 23)
    periodic = [np.abs(np.sum(zc * np.conj(np.roll(zc, k)))) for k in lags]
    padded = np.concatenate([zc, np.zeros(23)])
    aperiodic = np.correlate(padded, zc, mode="full")
    ap_lags = np.arange(-len(padded) + 1, 23)

    fig, axes = plt.subplots(1, 2, figsize=(10, 3.5))
    axes[0].stem(lags, periodic)
    axes[0].set_title("Periodic ACF of ZC(root=17, N=23)")
    axes[1].plot(ap_lags, np.abs(aperiodic))
    axes[1].set_title("Aperiodic ACF")
    for ax in axes:
        ax.set_xlabel("lag")
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(RESULTS / "zc_acf.png", dpi=130)
    print("saved", RESULTS / "zc_acf.png")


def fig_tx_signal():
    trx = Transceiver()
    sig = trx.build_frame(Data(reserved=123, payload=b"    Though this be madness,"))
    fs = trx.modem.sample_rate
    t = np.arange(len(sig)) / fs

    fig, axes = plt.subplots(2, 1, figsize=(10, 6))
    axes[0].plot(t, sig, lw=0.4)
    axes[0].set_title("TX frame oscillogram (tones + ZC preamble + OFDM symbols)")
    axes[0].set_xlabel("time, s")

    from scipy.signal import spectrogram
    f, tt, sxx = spectrogram(sig, fs=fs, nperseg=256, noverlap=192)
    axes[1].pcolormesh(tt, f, 10 * np.log10(sxx + 1e-12), shading="auto")
    axes[1].set_ylim(0, 3000)
    axes[1].set_title("Spectrogram")
    axes[1].set_xlabel("time, s")
    axes[1].set_ylabel("Hz")
    fig.tight_layout()
    fig.savefig(RESULTS / "tx_signal.png", dpi=130)
    print("saved", RESULTS / "tx_signal.png")


def collect_constellation(mod, snr_db, seed=11):
    trx = Transceiver()
    m = trx.modem
    rng = np.random.default_rng(seed)

    pkt = Data(reserved=123, payload=rng.bytes(27))
    sig = trx.build_frame(pkt, mod=mod, spd=CCSpeed.R13)
    rx = simulate_channel(sig, 700, 20.0, m.sample_rate, snr_db=snr_db, rng=rng)

    analytic = hilbert(rx).astype(np.complex64)
    det = m.detect_preamble(analytic)
    if det is None:
        return None
    start, cfo = det
    corrected = freq_shift(m.sample_rate, analytic, cfo)

    from ofdm_phy.transceiver import MAPPERS, CODECS, HEADER_MAPPER
    n_hdr = 6
    pos = start + n_hdr * m.symbol_len
    coded_len = CODECS[CCSpeed.R13].calc_cc_elements(len(pkt.encode()))
    mapper = MAPPERS[mod]
    n_syms = -(-coded_len // (m.data_carriers_len * mapper.MU))

    m.set_mapper(mapper)
    before, after = [], []
    for i in range(n_syms):
        sym = corrected[pos + i * m.symbol_len: pos + (i + 1) * m.symbol_len]
        *_, b, a = m.demodulate_symbol_cp_soft(sym, return_symbols=True)
        before.append(b)
        after.append(a)
    return np.concatenate(before), np.concatenate(after)


def fig_constellations():
    # per-modulation SNR pairs: 16-QAM operates at +0.7..+4.7 dB (rungs 10-12),
    # so its "noisy"/"clean" pair sits ~10 dB above the BPSK/QPSK one
    for mod, fname, snrs in ((ModType.BPSK, "constellation_bpsk.png", (-9, -3)),
                             (ModType.QPSK, "constellation_qpsk.png", (-9, -3)),
                             (ModType.QAM16, "constellation_qam16.png", (1, 5))):
        fig, axes = plt.subplots(2, 2, figsize=(8, 8))
        for col, snr in enumerate(snrs):
            res = collect_constellation(mod, snr)
            if res is None:
                continue
            before, after = res
            for row, (pts, name) in enumerate(((before, "before eq (ZF only)"),
                                               (after, "after MMSE eq"))):
                ax = axes[row][col]
                ax.scatter(pts.real, pts.imag, s=6, alpha=0.5)
                ax.set_title(f"{mod.name} {name}, SNR {snr} dB", fontsize=10)
                ax.axhline(0, color="gray", lw=0.5)
                ax.axvline(0, color="gray", lw=0.5)
                lim = 1.8 if mod is ModType.QAM16 else 3
                ax.set_xlim(-lim, lim)
                ax.set_ylim(-lim, lim)
                ax.set_aspect("equal")
        fig.tight_layout()
        fig.savefig(RESULTS / fname, dpi=130)
        print("saved", RESULTS / fname)


def collect_rung_constellation(rung, snr_db, seed=11):
    """Equalized data-carrier symbols through the real per-symbol receive
    chain (tiling accumulation, per-symbol frequency search, MMSE) for one
    ladder rung at one SNR."""
    from ofdm_phy.modes import make_modem
    trx = Transceiver(make_modem(rung.mode))
    m = trx.modem
    rng = np.random.default_rng(seed)

    pkt = Data(reserved=123, payload=rng.bytes(27))
    sig = trx.build_frame(pkt, mod=rung.mod, spd=rung.spd)
    rx = simulate_channel(sig, 700, 20.0, m.sample_rate, snr_db=snr_db,
                          rng=rng)

    analytic = hilbert(rx).astype(np.complex64)
    det = m.detect_preamble(analytic)
    if det is None:
        return None
    start, cfo = det
    corrected = freq_shift(m.sample_rate, analytic, cfo)

    from ofdm_phy.transceiver import MAPPERS, CODECS
    pos = start + 6 * m.symbol_len  # header is always 6 tiled symbols
    coded_len = CODECS[rung.spd].calc_cc_elements(len(pkt.encode()))
    mapper = MAPPERS[rung.mod]
    n_syms = -(-coded_len // (m.data_carriers_len * mapper.MU))

    # ground truth: replicate the TX bit pipeline (FEC + pad + interleave
    # + scramble) and map each symbol's bits to its transmitted
    # constellation point
    rows = trx._encode_block(pkt.encode(), CODECS[rung.spd], mapper)
    tx_syms = [mapper.map(row.reshape(m.data_carriers_len, mapper.MU))
               for row in rows]

    m.set_mapper(mapper)
    pts, truth = [], []
    for i in range(n_syms):
        sym = corrected[pos + i * m.symbol_len: pos + (i + 1) * m.symbol_len]
        *_, after = m.demodulate_symbol_cp_soft(sym, return_symbols=True)
        pts.append(after)
        truth.append(tx_syms[i])
    return np.concatenate(pts), np.concatenate(truth)


def fig_constellation_ladder():
    """Constellations across the operating range: at each SNR, the rung the
    rate controller would pick (highest rung with sens + 1 dB margin), so
    each panel shows the constellation actually on the air at that SNR."""
    from ofdm_phy.link import LADDER

    def rung_for(snr):
        best = 0
        for i, r in enumerate(LADDER):
            if snr >= r.sens_db + 1.0:
                best = i
        return best

    IDEAL = {
        ModType.BPSK: np.array([-1, 1], dtype=complex),
        ModType.QPSK: (np.array([-1 - 1j, -1 + 1j, 1 - 1j, 1 + 1j])
                       / np.sqrt(2)),
        ModType.QAM16: np.array([complex(i, q) for i in (-3, -1, 1, 3)
                                 for q in (-3, -1, 1, 3)]) / np.sqrt(10),
    }
    SPD_NAME = {CCSpeed.R13: "1/3", CCSpeed.R12: "1/2",
                CCSpeed.R23: "2/3", CCSpeed.R34: "3/4"}

    snrs = [-17.5, -10.0, -6.5, -3.0, 0.0, 4.0, 8.0, 20.0]
    fig, axes = plt.subplots(2, 4, figsize=(13, 7))
    for k, snr in enumerate(snrs):
        ax = axes[k // 4][k % 4]
        idx = rung_for(snr)
        rung = LADDER[idx]
        res = None
        for seed in (11, 12, 13):
            res = collect_rung_constellation(rung, snr, seed=seed)
            if res is not None:
                break
        if res is None:
            ax.set_title(f"{snr:+.1f} dB: no detect")
            continue
        pts, tx_pts = res
        ideal = IDEAL[rung.mod]
        # color each received point by the TRANSMITTED symbol (ground
        # truth), so raw pre-FEC errors show up as points of the "wrong"
        # color inside a neighbouring decision region
        truth = np.argmin(np.abs(tx_pts[:, None] - ideal[None, :]), axis=1)
        nearest = np.argmin(np.abs(pts[:, None] - ideal[None, :]), axis=1)
        ser = float(np.mean(nearest != truth))
        if len(ideal) > 4:
            # 16 saturated hues, permuted (7 is coprime to 16) so
            # neighbouring cells land on distant colors
            hsv = plt.get_cmap("hsv")
            colors = hsv((truth * 7 % 16) / 16.0)
        else:
            colors = plt.get_cmap("tab10")(truth)
        ax.scatter(pts.real, pts.imag, s=7, alpha=0.65, lw=0, c=colors)
        ax.text(0.03, 0.03, f"raw SER {ser:.0%}", transform=ax.transAxes,
                fontsize=8, color="dimgray")
        ax.scatter(ideal.real, ideal.imag, s=34, marker="+", color="black",
                   zorder=3)
        if rung.mod is ModType.QAM16:
            # decision-boundary subgrid between the 16 cells
            for b in (-2 / np.sqrt(10), 2 / np.sqrt(10)):
                ax.axvline(b, color="gray", lw=0.5, ls=":")
                ax.axhline(b, color="gray", lw=0.5, ls=":")
        ax.set_title(f"{snr:+.1f} dB $\\rightarrow$ rung {idx}\n"
                     f"{rung.mode.name} {rung.mod.name} "
                     f"{SPD_NAME[rung.spd]}", fontsize=10)
        ax.axhline(0, color="gray", lw=0.5)
        ax.axvline(0, color="gray", lw=0.5)
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_aspect("equal")
        ax.tick_params(labelsize=8)
    fig.suptitle("Equalized constellations across the operating range "
                 "(rung chosen by the rate controller)", fontsize=12)
    fig.tight_layout()
    fig.savefig(RESULTS / "constellation_ladder.png", dpi=130)
    print("saved", RESULTS / "constellation_ladder.png")


def _llr_shape_cell(args):
    """One chunk of the LLR ground-truth collection (parallel worker)."""
    chunk, n_frames, snr = args
    sys.path.insert(0, str(ROOT / "experiments"))
    import llr_calibration as lc
    trx = Transceiver()
    rng = np.random.default_rng([7, chunk])
    _, _, pool = lc.collect(trx, snr, n_frames, rng)
    return pool


def fig_llr_shape(n_frames=300):
    """The LLR shape miscalibration that motivates the monotone map
    (Section 6 of the report): empirical reliability vs the value the
    LLR claims, at the -8 dB operating point of the LDPC comparison.
    Parallel collection; every bin value is annotated on the plot."""
    import multiprocessing as mp

    snr = -8
    chunk_size = 30
    n_chunks = -(-n_frames // chunk_size)
    cells = [(ci, min(chunk_size, n_frames - ci * chunk_size), snr)
             for ci in range(n_chunks)]
    print(f"collecting LLR ground truth at {snr} dB "
          f"({n_frames} frames, {n_chunks} parallel chunks)...")
    with mp.Pool(max(1, mp.cpu_count() - 1)) as pool:
        pools = pool.map(_llr_shape_cell, cells)
    lx = np.concatenate([p for p in pools if len(p)])

    edges = np.array([0, 0.5, 1, 1.5, 2, 3, 4, 6, 8, 11, 14, 17, 20.01])
    mids, emp = [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = (np.abs(lx) >= lo) & (np.abs(lx) < hi)
        n = int(sel.sum())
        if n < 80:
            continue
        p_err = float(np.mean(lx[sel] < 0))
        p_err = min(max(p_err, 0.5 / n), 1 - 0.5 / n)  # keep log-odds finite
        mids.append(float(np.mean(np.abs(lx[sel]))))
        emp.append(p_err)
    mids, emp = np.array(mids), np.array(emp)
    l_emp = np.log((1 - emp) / emp)
    alpha = float(2 * np.mean(lx) / np.var(lx))  # single-temperature fit

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.6))

    grid = np.linspace(0.01, 20, 300)
    ax1.semilogy(grid, 1 / (1 + np.exp(grid)), "k-", lw=1,
                 label=r"ideal $P(\mathrm{err}) = 1/(1+e^{|L|})$")
    ax1.semilogy(mids, emp, "o-", color="tab:red", ms=5,
                 label="measured")
    for i, (x, y) in enumerate(zip(mids, emp)):
        ax1.annotate(f"{y:.1%}" if y >= 0.01 else f"{y:.2%}",
                     xy=(x, y),
                     xytext=(4, 5) if i % 2 == 0 else (4, -12),
                     textcoords="offset points", fontsize=7,
                     color="tab:red")
    ax1.annotate(f"|L|<2: err {np.mean(lx[np.abs(lx) < 2] < 0):.0%}\n"
                 f"(value implies "
                 f"{1 / (1 + np.exp(np.mean(np.abs(lx[np.abs(lx) < 2])))):.0%})",
                 xy=(mids[1], emp[1]), xytext=(5, 0.25), fontsize=9,
                 arrowprops=dict(arrowstyle="->", lw=0.8))
    ax1.annotate("strong-LLR tail: errors ~100x the claim\n"
                 "(reliability DROPS as claimed\n"
                 "confidence rises)",
                 xy=(mids[-1], emp[-1]), xytext=(10.5, 2e-6), fontsize=9,
                 arrowprops=dict(arrowstyle="->", lw=0.8))
    ax1.set_xlabel("|L| (nominal LLR magnitude)")
    ax1.set_ylabel("P(bit error)")
    ax1.set_title("What the LLR claims vs how often it is wrong")
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=9)

    ax2.plot([0, 20], [0, 20], "k--", lw=1, label="perfectly calibrated")
    ax2.plot([0, 20], [0, 20 * alpha], ":", color="tab:blue", lw=1.5,
             label=f"single temperature $\\alpha$={alpha:.2f}")
    ax2.plot(mids, l_emp, "o-", color="tab:red", ms=5,
             label="measured shape (input to the monotone map)")
    for i, (x, y) in enumerate(zip(mids, l_emp)):
        ax2.annotate(f"{x:.1f}$\\rightarrow${y:.1f}",
                     xy=(x, y),
                     xytext=(-10, 8) if i % 2 == 0 else (4, -13),
                     textcoords="offset points", fontsize=7,
                     color="tab:red")
    ax2.annotate("weak LLRs underconfident:\ncurve ABOVE identity\n"
                 r"(wants $\alpha>1$)",
                 xy=(mids[2], l_emp[2]), xytext=(0.4, 9.2), fontsize=9,
                 arrowprops=dict(arrowstyle="->", lw=0.8))
    ax2.annotate("strong LLRs overconfident:\ncurve falls BELOW identity\n"
                 r"(drags the fit to $\alpha<1$)",
                 xy=(mids[-1], l_emp[-1]), xytext=(7.5, 14.0), fontsize=9,
                 arrowprops=dict(arrowstyle="->", lw=0.8))
    ax2.set_xlabel("|L| (nominal)")
    ax2.set_ylabel(r"empirical LLR  $\ln\frac{1-P(\mathrm{err})}{P(\mathrm{err})}$")
    ax2.set_title("The shape no single temperature can fix")
    ax2.set_xlim(0, 20)
    ax2.set_ylim(0, 20)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=9, loc="lower right")

    fig.suptitle(f"Front-end LLR miscalibration, NORMAL BPSK 1/3 @ {snr} dB "
                 f"({n_frames} frames, raw data-block LLRs vs known bits)",
                 fontsize=11)
    fig.tight_layout()
    stem = "llr_shape" if n_frames == 300 else f"llr_shape_n{n_frames}"
    fig.savefig(RESULTS / f"{stem}.png", dpi=130)
    import json
    record = {
        "experiment": "front-end LLR miscalibration shape",
        "mode": "NORMAL", "modulation": "BPSK", "rate": "1/3",
        "snr_db": snr,
        "snr_convention": "noise in the 6 kHz Nyquist band",
        "n_frames": n_frames, "n_llr_samples": int(len(lx)),
        "seed_scheme": "np.random.default_rng([7, chunk]) per 30-frame "
                       "chunk; channel: offset 300-1200, CFO +-60 Hz, AWGN",
        "bin_edges": [float(x) for x in edges],
        "min_samples_per_bin": 80,
        "bins": [{"mean_abs_L": float(m), "p_err": float(e),
                  "empirical_llr": float(l)}
                 for m, e, l in zip(mids, emp, l_emp)],
        "single_temperature_alpha": alpha,
    }
    with open(RESULTS / f"{stem}.json", "w") as f:
        json.dump(record, f, indent=2)
    print("saved", RESULTS / f"{stem}.png", "and", f"{stem}.json")


def _viterbi_recal_cell(args):
    """One SNR point of the Viterbi calibration sweep: each frame is
    demodulated once and the same raw LLRs are decoded three ways
    (raw / temperature-scaled / monotone shape map)."""
    si, snr, n_frames = args
    from ofdm_phy.transceiver import (CODECS, MAPPERS, default_llr_recal)
    trx = Transceiver()
    m = trx.modem
    codec = CODECS[CCSpeed.R13]
    rng = np.random.default_rng([21, si])
    ok = [0, 0, 0]
    for _ in range(n_frames):
        pkt = Data(reserved=123, payload=rng.bytes(27))
        truth = pkt.encode().astype(np.uint8)
        sig = trx.build_frame(pkt, mod=ModType.BPSK, spd=CCSpeed.R13)
        rx = simulate_channel(sig, int(rng.integers(300, 1200)),
                              float(rng.uniform(-60, 60)), 12000,
                              snr_db=float(snr), rng=rng)
        analytic = hilbert(rx).astype(np.complex64)
        det = m.detect_preamble(analytic)
        if det is None:
            continue
        corr = freq_shift(12000, analytic, det[1])
        corr = np.concatenate(
            [corr, np.zeros(m.symbol_len, dtype=corr.dtype)])
        pos = det[0] + 6 * m.symbol_len
        m.set_mapper(MAPPERS[ModType.BPSK])
        n_syms = -(-codec.calc_cc_elements(len(truth))
                   // m.data_carriers_len)
        try:
            llrs, _, _ = trx._demod_symbols(corr, pos, n_syms)
        except Exception:
            continue
        for vi, arr in enumerate((llrs, 0.67 * llrs,
                                  default_llr_recal(llrs))):
            bits = trx._decode_block(arr, codec, len(truth))
            if np.array_equal(bits.astype(np.uint8), truth):
                ok[vi] += 1
    return si, ok


def fig_viterbi_recal(n_frames=300):
    """Calibration and the Viterbi decoder: decode-vs-SNR for raw LLRs,
    temperature-only scaling (provably identical decisions -- Viterbi is
    scale-invariant), and the monotone shape map (the measured 1.5-2 dB);
    plus the deployed map itself. Parallel over SNR points."""
    import multiprocessing as mp
    from ofdm_phy.transceiver import _RECAL_MIDS, _RECAL_OUTS

    snrs = np.arange(-10.5, -6.4, 0.5)
    variants = ["raw LLRs", "temperature only (0.67*L)",
                "monotone shape map"]
    ok = np.zeros((3, len(snrs)))
    cells = [(si, float(snr), n_frames) for si, snr in enumerate(snrs)]
    with mp.Pool(max(1, mp.cpu_count() - 1)) as pool:
        for si, counts in pool.imap_unordered(_viterbi_recal_cell, cells):
            ok[:, si] = counts
            print(f"  {snrs[si]:+.1f} dB: "
                  + " ".join(str(c) for c in counts)
                  + f" /{n_frames}", flush=True)
    assert np.array_equal(ok[0], ok[1]), \
        "temperature-only must be bit-identical to raw (scale invariance)"

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.4))
    ax1.plot(snrs, ok[0] / n_frames, "o-", color="tab:red",
             label="raw LLRs")
    ax1.plot(snrs, ok[1] / n_frames, "x--", color="tab:blue", ms=8,
             label=r"temperature only ($0.67 \cdot L$)")
    ax1.plot(snrs, ok[2] / n_frames, "s-", color="tab:green",
             label="monotone shape map")
    ax1.set_xlabel("SNR, dB (6 kHz convention)")
    ax1.set_ylabel("P(frame decodes)")
    ax1.set_title("Viterbi chain, NORMAL BPSK 1/3\n"
                  "(temperature curve overlays raw exactly: "
                  "scale-invariance)")
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=9, loc="upper left")

    grid = np.linspace(0, 20, 200)
    ax2.plot([0, 20], [0, 20], "k--", lw=1, label="identity (no correction)")
    ax2.plot(grid, 0.67 * grid, ":", color="tab:blue", lw=1.5,
             label=r"temperature $0.67 \cdot L$ (no effect on Viterbi)")
    ax2.plot(grid, np.interp(grid, _RECAL_MIDS, _RECAL_OUTS), "-",
             color="tab:green", lw=2,
             label="deployed monotone map (changes path sums)")
    ax2.plot(_RECAL_MIDS, _RECAL_OUTS, "s", color="tab:green", ms=4)
    ax2.set_xlabel("|L| in (raw)")
    ax2.set_ylabel("|L| out")
    ax2.set_title("The correction applied before decoding")
    ax2.set_xlim(0, 20)
    ax2.set_ylim(0, 20)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=9, loc="upper left")

    fig.suptitle("What calibration does for the Viterbi decoder "
                 f"({n_frames} frames/point, article channel)",
                 fontsize=11)
    fig.tight_layout()
    stem = ("viterbi_recal" if n_frames == 300
            else f"viterbi_recal_n{n_frames}")
    fig.savefig(RESULTS / f"{stem}.png", dpi=130)
    import json
    record = {
        "experiment": "Viterbi decoder vs LLR calibration",
        "mode": "NORMAL", "modulation": "BPSK", "fec": "conv K=7 rate 1/3",
        "payload_bytes": 27, "n_frames_per_point": n_frames,
        "snr_grid_db": [float(x) for x in snrs],
        "snr_convention": "noise in the 6 kHz Nyquist band",
        "channel": "simulate_channel: random offset 300-1200, "
                   "CFO uniform +-60 Hz, AWGN",
        "seed_scheme": "np.random.default_rng([21, snr_index]); one demod "
                       "per frame shared by all three decode variants",
        "variants": variants,
        "decodes": {variants[vi]: [int(x) for x in ok[vi]]
                    for vi in range(3)},
        "note": "raw and temperature-only are asserted bit-identical "
                "(Viterbi scale invariance)",
    }
    np.save(RESULTS / f"{stem}_counts.npy", ok)
    with open(RESULTS / f"{stem}.json", "w") as f:
        json.dump(record, f, indent=2)
    print("saved", RESULTS / f"{stem}.png", "and", f"{stem}.json")


def _ldpc_recal_cell(args):
    """One SNR point of the LDPC calibration sweep: each frame is
    demodulated once and the same raw LLRs are decoded four ways
    (min-sum / exact sum-product, each on raw / shape-mapped inputs)."""
    si, snr, n_frames = args
    from ofdm_phy.transceiver import MAPPERS, default_llr_recal
    import ofdm_phy.ldpc as L
    trx = Transceiver()
    m = trx.modem
    rng = np.random.default_rng([33, si])
    ok = [0, 0, 0, 0]
    for _ in range(n_frames):
        pkt = Data(reserved=123, payload=rng.bytes(27))
        truth = pkt.encode().astype(np.uint8)
        sig = trx.build_frame(pkt, mod=ModType.BPSK, spd=CCSpeed.R13,
                              fec="ldpc")
        rx = simulate_channel(sig, int(rng.integers(300, 1200)),
                              float(rng.uniform(-60, 60)), 12000,
                              snr_db=float(snr), rng=rng)
        analytic = hilbert(rx).astype(np.complex64)
        det = m.detect_preamble(analytic)
        if det is None:
            continue
        corr = freq_shift(12000, analytic, det[1])
        corr = np.concatenate(
            [corr, np.zeros(m.symbol_len, dtype=corr.dtype)])
        pos = det[0] + 6 * m.symbol_len
        m.set_mapper(MAPPERS[ModType.BPSK])
        n_syms = -(-L.LDPCCodec.calc_cc_elements(len(truth))
                   // m.data_carriers_len)
        try:
            llrs, _, _ = trx._demod_symbols(corr, pos, n_syms)
        except Exception:
            continue
        mapped = default_llr_recal(llrs)
        cases = [(L._sum_product, llrs), (L._sum_product, mapped),
                 (_fec_spa_kernel, llrs), (_fec_spa_kernel, mapped)]
        for vi, (kernel, arr) in enumerate(cases):
            bits = _fec_ldpc_decode(kernel, arr, len(truth),
                                    m.data_carriers_len)
            if np.array_equal(bits.astype(np.uint8), truth):
                ok[vi] += 1
    return si, ok


def fig_ldpc_recal(n_frames=300):
    """The LDPC analogue of fig_viterbi_recal: the same raw LLRs decoded
    by normalized min-sum (the shipped, scale-invariant kernel) and by
    exact sum-product (reconstructed tanh rule, consumes absolute LLRs),
    each on raw and on shape-mapped inputs. Parallel over SNR points."""
    import multiprocessing as mp

    snrs = np.arange(-10.5, -6.4, 0.5)
    variants = ["min-sum, raw", "min-sum, shape map",
                "sum-product, raw", "sum-product, shape map"]
    ok = np.zeros((4, len(snrs)))
    cells = [(si, float(snr), n_frames) for si, snr in enumerate(snrs)]
    with mp.Pool(max(1, mp.cpu_count() - 1)) as pool:
        for si, counts in pool.imap_unordered(_ldpc_recal_cell, cells):
            ok[:, si] = counts
            print(f"  {snrs[si]:+.1f} dB: "
                  + " ".join(str(c) for c in counts)
                  + f" /{n_frames}", flush=True)

    fig, (ax1, ax2) = plt.subplots(
        1, 2, figsize=(11, 4.4), gridspec_kw={"width_ratios": [1.7, 1]})
    styles = [("tab:green", "s-"), ("tab:green", "^--"),
              ("tab:red", "o-"), ("tab:blue", "x-")]
    for vi, label in enumerate(variants):
        c, st = styles[vi]
        ax1.plot(snrs, ok[vi] / n_frames, st, color=c, label=label, ms=6)
    ax1.set_xlabel("SNR, dB (6 kHz convention)")
    ax1.set_ylabel("P(frame decodes)")
    ax1.set_title("LDPC (IRA rate 1/3), NORMAL BPSK, same raw LLRs\n"
                  "min-sum tolerates the shape; sum-product needs the map")
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=9, loc="upper left")

    i9 = int(np.argmin(np.abs(snrs - (-9.0))))
    names = [v.replace(", ", ",\n") for v in variants]
    colors = [c for c, _ in styles]
    ax2.bar(range(4), ok[:, i9] / n_frames, color=colors, alpha=0.75)
    ax2.set_xticks(range(4))
    ax2.set_xticklabels(names, fontsize=8)
    for vi in range(4):
        ax2.text(vi, ok[vi, i9] / n_frames + 0.02,
                 f"{int(ok[vi, i9])}/{n_frames}", ha="center", fontsize=9)
    ax2.set_ylim(0, 1.05)
    ax2.set_ylabel("P(frame decodes)")
    ax2.set_title(f"At {snrs[i9]:+.1f} dB")
    ax2.grid(alpha=0.3, axis="y")

    fig.suptitle("What calibration does for the LDPC decoders "
                 f"({n_frames} frames/point, article channel)", fontsize=11)
    fig.tight_layout()
    stem = ("ldpc_recal" if n_frames == 300 else f"ldpc_recal_n{n_frames}")
    fig.savefig(RESULTS / f"{stem}.png", dpi=130)
    import json
    record = {
        "experiment": "LDPC decoder kernels vs LLR calibration",
        "mode": "NORMAL", "modulation": "BPSK", "fec": "ldpc (IRA 1/3)",
        "payload_bytes": 27, "n_frames_per_point": n_frames,
        "snr_grid_db": [float(x) for x in snrs],
        "snr_convention": "noise in the 6 kHz Nyquist band",
        "channel": "simulate_channel: random offset 300-1200, "
                   "CFO uniform +-60 Hz, AWGN",
        "seed_scheme": "np.random.default_rng([33, snr_index]); one demod "
                       "per frame shared by all four decode variants",
        "variants": variants,
        "decodes": {variants[vi]: [int(x) for x in ok[vi]]
                    for vi in range(4)},
    }
    np.save(RESULTS / f"{stem}_counts.npy", ok)
    with open(RESULTS / f"{stem}.json", "w") as f:
        json.dump(record, f, indent=2)
    print("saved", RESULTS / f"{stem}.png", "and", f"{stem}.json")


def _fec_spa_kernel(llr_std, max_iter=60):
    """Exact sum-product (tanh rule) over the ldpc graph -- reconstructed
    for the calibration measurements; the shipped code is min-sum only."""
    import ofdm_phy.ldpc as L
    v2c = np.where(L._EV_VALID, llr_std[L._EV_SAFE], np.inf)
    bits = (llr_std < 0).astype(np.uint8)
    for _ in range(max_iter):
        t = np.tanh(np.clip(v2c, -30, 30) / 2.0)
        t = np.where(np.abs(t) < 1e-12, 1e-12, t)
        row = np.prod(t, axis=1, keepdims=True)
        excl = np.clip(row / t, -0.999999999, 0.999999999)
        c2v = np.where(L._EV_VALID, 2.0 * np.arctanh(excl), 0.0)
        totals = llr_std + np.bincount(
            L._EV_SAFE[L._EV_VALID].ravel(),
            weights=c2v[L._EV_VALID].ravel(), minlength=L.N)
        bits = (totals < 0).astype(np.uint8)
        syn = np.bitwise_xor.reduce(
            np.where(L._EV_VALID, bits[L._EV_SAFE], 0), axis=1)
        if not syn.any():
            return bits
        v2c = np.where(L._EV_VALID, totals[L._EV_SAFE] - c2v, np.inf)
    return bits


def _fec_ldpc_decode(kernel, block_llrs, bits_count, ncar):
    import ofdm_phy.ldpc as L
    from ofdm_phy.scrambler import descramble
    from ofdm_phy.interleaver import deinterleave
    cropped = deinterleave(descramble(block_llrs), ncar)
    cropped = cropped[:L.LDPCCodec.calc_cc_elements(bits_count)]
    llr = np.zeros(L.N)
    pos = L.LDPCCodec._transmit_positions(bits_count)
    llr[pos] = np.asarray(cropped, dtype=np.float64)[:len(pos)]
    llr[bits_count:L.K_MAX] = -L.LDPCCodec.PIN_LLR
    return kernel(-llr)[:bits_count]


def _fec_cell(args):
    """One (variant, snr) cell of the head-to-head sweep: n_frames frames,
    channel draws seeded by the snr index only, so every variant sees the
    same payloads/offsets/CFO/noise sequence."""
    vi, si, snr, n_frames = args
    from ofdm_phy.transceiver import CODECS, MAPPERS, default_llr_recal
    import ofdm_phy.ldpc as L
    trx = Transceiver()
    m = trx.modem
    rng = np.random.default_rng([55, si])
    fec = "cc" if vi == 0 else "ldpc"
    ok = 0
    for _ in range(n_frames):
        pkt = Data(reserved=123, payload=rng.bytes(27))
        truth = pkt.encode().astype(np.uint8)
        sig = trx.build_frame(pkt, mod=ModType.BPSK, spd=CCSpeed.R13,
                              fec=fec)
        rx = simulate_channel(sig, int(rng.integers(300, 1200)),
                              float(rng.uniform(-60, 60)), 12000,
                              snr_db=float(snr), rng=rng)
        analytic = hilbert(rx).astype(np.complex64)
        det = m.detect_preamble(analytic)
        if det is None:
            continue
        corr = freq_shift(12000, analytic, det[1])
        corr = np.concatenate(
            [corr, np.zeros(m.symbol_len, dtype=corr.dtype)])
        pos = det[0] + 6 * m.symbol_len
        m.set_mapper(MAPPERS[ModType.BPSK])
        celems = (CODECS[CCSpeed.R13].calc_cc_elements(len(truth))
                  if vi == 0
                  else L.LDPCCodec.calc_cc_elements(len(truth)))
        n_syms = -(-celems // m.data_carriers_len)
        try:
            llrs, _, _ = trx._demod_symbols(corr, pos, n_syms)
        except Exception:
            continue
        mapped = default_llr_recal(llrs)
        if vi == 0:
            bits = trx._decode_block(mapped, CODECS[CCSpeed.R13],
                                     len(truth))
        elif vi == 1:
            bits = _fec_ldpc_decode(_fec_spa_kernel, mapped, len(truth),
                                    m.data_carriers_len)
        else:
            bits = _fec_ldpc_decode(L._sum_product, mapped, len(truth),
                                    m.data_carriers_len)
        if np.array_equal(bits.astype(np.uint8), truth):
            ok += 1
    return vi, si, ok


def fig_fec_comparison(n_frames=300):
    """Head-to-head of the two FEC families with the front end fully
    calibrated (shape map on both): what remains is the code itself.
    Parallel over (variant, snr) cells; per-snr seeding gives every
    variant identical channel draws."""
    import multiprocessing as mp

    snrs = np.arange(-10.5, -6.4, 0.5)
    variants = ["conv + Viterbi", "LDPC + sum-product", "LDPC + min-sum"]
    cells = [(vi, si, float(snr), n_frames)
             for vi in range(3) for si, snr in enumerate(snrs)]
    ok = np.zeros((3, len(snrs)))
    with mp.Pool(max(1, mp.cpu_count() - 1)) as pool:
        for vi, si, cnt in pool.imap_unordered(_fec_cell, cells):
            ok[vi, si] = cnt
            print(f"  {variants[vi]:20s} {snrs[si]:+.1f} dB: "
                  f"{cnt}/{n_frames}", flush=True)

    fig, ax = plt.subplots(figsize=(7.5, 4.6))
    styles = [("tab:green", "s-"), ("tab:blue", "x-"), ("tab:orange", "^--")]
    for vi, name in enumerate(variants):
        c, st = styles[vi]
        ax.plot(snrs, ok[vi] / n_frames, st, color=c, ms=6,
                label=name + " (shape-mapped LLRs)")
    ax.set_xlabel("SNR, dB (6 kHz convention)")
    ax.set_ylabel("P(frame decodes)")
    ax.set_title("FEC families head-to-head with a calibrated front end\n"
                 "(NORMAL BPSK rate 1/3, 27-byte packets, "
                 f"article channel, {n_frames} frames/point)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9, loc="upper left")
    fig.tight_layout()
    # non-reference trial counts get suffixed names so a quick smoke run
    # cannot clobber the committed 300-frame reference artifacts
    stem = ("fec_comparison" if n_frames == 300
            else f"fec_comparison_n{n_frames}")
    fig.savefig(RESULTS / f"{stem}.png", dpi=130)
    # full reproducibility record next to the figure
    import json
    record = {
        "experiment": "FEC head-to-head with calibrated front end",
        "mode": "NORMAL", "modulation": "BPSK", "rate": "1/3",
        "payload_bytes": 27, "n_frames_per_point": n_frames,
        "snr_grid_db": [float(x) for x in snrs],
        "snr_convention": "noise in the 6 kHz Nyquist band",
        "channel": "simulate_channel: random offset 300-1200, "
                   "CFO uniform +-60 Hz, AWGN",
        "seed_scheme": "np.random.default_rng([55, snr_index]) per cell; "
                       "identical draws for all variants at each SNR",
        "llr_processing": "production demod, default_llr_recal shape map",
        "variants": variants,
        "decodes": {variants[vi]: [int(x) for x in ok[vi]]
                    for vi in range(3)},
    }
    np.save(RESULTS / f"{stem}_counts.npy", ok)
    with open(RESULTS / f"{stem}.json", "w") as f:
        json.dump(record, f, indent=2)
    print("saved", RESULTS / f"{stem}.png", "and", f"{stem}.json")


def _extreme_cell(args):
    """One (family, snr) cell of the EXTREME-edge calibration study.
    family 0: conv frames decoded raw / 0.67*L / shape map (Viterbi).
    family 1: LDPC frames decoded min-sum & exact SPA, raw & mapped.
    Channel draws are seeded by the snr index only, so both families
    (and all decode variants) see the same draw sequence."""
    family, si, snr, n_frames = args
    from ofdm_phy.transceiver import (CODECS, MAPPERS, default_llr_recal)
    from ofdm_phy.modes import make_modem
    from ofdm_phy import LinkMode
    import ofdm_phy.ldpc as L
    trx = Transceiver(make_modem(LinkMode.EXTREME))
    m = trx.modem
    codec = CODECS[CCSpeed.R13]
    rng = np.random.default_rng([91, si])
    nv = 3 if family == 0 else 4
    ok = [0] * nv
    for _ in range(n_frames):
        pkt = Data(reserved=123, payload=rng.bytes(27))
        truth = pkt.encode().astype(np.uint8)
        sig = trx.build_frame(pkt, mod=ModType.BPSK, spd=CCSpeed.R13,
                              fec="cc" if family == 0 else "ldpc")
        rx = simulate_channel(sig, int(rng.integers(300, 1200)),
                              float(rng.uniform(-60, 60)), 12000,
                              snr_db=float(snr), rng=rng)
        analytic = hilbert(rx).astype(np.complex64)
        det = m.detect_preamble(analytic)
        if det is None:
            continue
        corr = freq_shift(12000, analytic, det[1])
        corr = np.concatenate(
            [corr, np.zeros(m.symbol_len, dtype=corr.dtype)])
        pos = det[0] + 6 * m.symbol_len
        m.set_mapper(MAPPERS[ModType.BPSK])
        celems = (L.LDPCCodec.calc_cc_elements(len(truth)) if family == 1
                  else codec.calc_cc_elements(len(truth)))
        n_syms = -(-celems // m.data_carriers_len)
        try:
            llrs, _, _ = trx._demod_symbols(corr, pos, n_syms)
        except Exception:
            continue
        mapped = default_llr_recal(llrs)
        if family == 0:
            cases = [(None, llrs), (None, 0.67 * llrs), (None, mapped)]
            for vi, (_, arr) in enumerate(cases):
                bits = trx._decode_block(arr, codec, len(truth))
                if np.array_equal(bits.astype(np.uint8), truth):
                    ok[vi] += 1
        else:
            cases = [(L._sum_product, llrs), (L._sum_product, mapped),
                     (_fec_spa_kernel, llrs), (_fec_spa_kernel, mapped)]
            for vi, (kernel, arr) in enumerate(cases):
                bits = _fec_ldpc_decode(kernel, arr, len(truth),
                                        m.data_carriers_len)
                if np.array_equal(bits.astype(np.uint8), truth):
                    ok[vi] += 1
    return family, si, ok


def fig_extreme_recal(n_frames=300):
    """The calibration study repeated at the EXTREME edge (64x tiling,
    rung 0 territory): does the NORMAL-trained shape map transfer, and
    which code family wins down here? Tests the report's
    'accumulation-limited rungs are unaffected' claim directly."""
    import multiprocessing as mp

    snrs = np.arange(-20.5, -16.4, 0.5)
    conv_names = ["raw LLRs", "temperature only (0.67*L)",
                  "monotone shape map"]
    ldpc_names = ["min-sum, raw", "min-sum, shape map",
                  "sum-product, raw", "sum-product, shape map"]
    okc = np.zeros((3, len(snrs)))
    okl = np.zeros((4, len(snrs)))
    cells = ([(0, si, float(snr), n_frames)
              for si, snr in enumerate(snrs)]
             + [(1, si, float(snr), n_frames)
                for si, snr in enumerate(snrs)])
    with mp.Pool(max(1, mp.cpu_count() - 1)) as pool:
        for family, si, counts in pool.imap_unordered(_extreme_cell,
                                                      cells):
            if family == 0:
                okc[:, si] = counts
            else:
                okl[:, si] = counts
            print(f"  {'conv' if family == 0 else 'ldpc'} "
                  f"{snrs[si]:+.1f} dB: "
                  + " ".join(str(c) for c in counts)
                  + f" /{n_frames}", flush=True)
    assert np.array_equal(okc[0], okc[1]), \
        "temperature-only must be bit-identical to raw (scale invariance)"

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4.4))

    ax1.plot(snrs, okc[0] / n_frames, "o-", color="tab:red",
             label="raw LLRs")
    ax1.plot(snrs, okc[1] / n_frames, "x--", color="tab:blue", ms=8,
             label=r"temperature only ($0.67 \cdot L$)")
    ax1.plot(snrs, okc[2] / n_frames, "s-", color="tab:green",
             label="monotone shape map")
    ax1.set_title("Viterbi chain (conv 1/3)")

    styles = [("tab:green", "s-"), ("tab:green", "^--"),
              ("tab:red", "o-"), ("tab:blue", "x-")]
    for vi, label in enumerate(ldpc_names):
        c, st = styles[vi]
        ax2.plot(snrs, okl[vi] / n_frames, st, color=c, label=label, ms=6)
    ax2.set_title("LDPC kernels (IRA 1/3)")

    ax3.plot(snrs, okc[2] / n_frames, "s-", color="tab:green",
             label="conv + Viterbi, shape map")
    ax3.plot(snrs, okl[3] / n_frames, "x-", color="tab:blue",
             label="LDPC + sum-product, shape map")
    ax3.plot(snrs, okl[1] / n_frames, "^--", color="tab:orange",
             label="LDPC + min-sum, shape map")
    ax3.set_title("Head-to-head, calibrated")

    for ax in (ax1, ax2, ax3):
        ax.set_xlabel("SNR, dB (6 kHz convention)")
        ax.set_ylabel("P(frame decodes)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="upper left")
        ax.set_ylim(-0.03, 1.05)

    fig.suptitle("The calibration study at the EXTREME edge "
                 f"(64$\\times$ tiling, BPSK 1/3, {n_frames} "
                 "frames/point, article channel)", fontsize=11)
    fig.tight_layout()
    stem = ("extreme_recal" if n_frames == 300
            else f"extreme_recal_n{n_frames}")
    fig.savefig(RESULTS / f"{stem}.png", dpi=130)
    import json
    record = {
        "experiment": "calibration study at the EXTREME edge",
        "mode": "EXTREME", "modulation": "BPSK", "rate": "1/3",
        "payload_bytes": 27, "n_frames_per_point": n_frames,
        "snr_grid_db": [float(x) for x in snrs],
        "snr_convention": "noise in the 6 kHz Nyquist band",
        "channel": "simulate_channel: random offset 300-1200, "
                   "CFO uniform +-60 Hz, AWGN",
        "seed_scheme": "np.random.default_rng([91, snr_index]); "
                       "shared draws across families and variants",
        "conv_variants": conv_names,
        "conv_r13_decodes": {conv_names[vi]: [int(x) for x in okc[vi]]
                             for vi in range(3)},
        "ldpc_variants": ldpc_names,
        "ldpc_decodes": {ldpc_names[vi]: [int(x) for x in okl[vi]]
                         for vi in range(4)},
        "conv_r23_snr_grid_db": [float(x) for x in snrs23],
        "conv_r23_decodes": {conv_names[vi]: [int(x) for x in okc23[vi]]
                             for vi in range(3)},
        "note": "shape map is the NORMAL-BPSK-trained default_llr_recal; "
                "raw==temperature asserted (Viterbi scale invariance)",
    }
    np.save(RESULTS / f"{stem}_counts_conv.npy", okc)
    np.save(RESULTS / f"{stem}_counts_ldpc.npy", okl)
    with open(RESULTS / f"{stem}.json", "w") as f:
        json.dump(record, f, indent=2)
    print("saved", RESULTS / f"{stem}.png", "and", f"{stem}.json")


def _qam16_cell(args):
    """One (family, snr) cell of the 16-QAM calibration study.
    family 0: conv rate-1/3 frames, raw / 0.67*L / shape map (Viterbi).
    family 1: LDPC frames (rate 1/3), min-sum & exact SPA, raw & mapped.
    family 2: conv rate-2/3 frames (the ladder rung 11 rate), 3-way."""
    family, si, snr, n_frames = args
    from ofdm_phy.transceiver import (CODECS, MAPPERS, default_llr_recal)
    import ofdm_phy.ldpc as L
    trx = Transceiver()
    m = trx.modem
    spd = CCSpeed.R23 if family == 2 else CCSpeed.R13
    codec = CODECS[spd]
    mapper = MAPPERS[ModType.QAM16]
    rng = np.random.default_rng([77, si])
    nv = 4 if family == 1 else 3
    ok = [0] * nv
    for _ in range(n_frames):
        pkt = Data(reserved=123, payload=rng.bytes(27))
        truth = pkt.encode().astype(np.uint8)
        sig = trx.build_frame(pkt, mod=ModType.QAM16, spd=spd,
                              fec="ldpc" if family == 1 else "cc")
        rx = simulate_channel(sig, int(rng.integers(300, 1200)),
                              float(rng.uniform(-60, 60)), 12000,
                              snr_db=float(snr), rng=rng)
        analytic = hilbert(rx).astype(np.complex64)
        det = m.detect_preamble(analytic)
        if det is None:
            continue
        corr = freq_shift(12000, analytic, det[1])
        corr = np.concatenate(
            [corr, np.zeros(m.symbol_len, dtype=corr.dtype)])
        pos = det[0] + 6 * m.symbol_len
        m.set_mapper(mapper)
        celems = (L.LDPCCodec.calc_cc_elements(len(truth)) if family == 1
                  else codec.calc_cc_elements(len(truth)))
        n_syms = -(-celems // (m.data_carriers_len * mapper.MU))
        try:
            llrs, _, _ = trx._demod_symbols(corr, pos, n_syms)
        except Exception:
            continue
        mapped = default_llr_recal(llrs)
        # NB: TX interleaves with data_carriers_len regardless of MU,
        # so the deinterleaver must use 16, not 16*MU
        ncar = m.data_carriers_len
        if family != 1:
            for vi, arr in enumerate((llrs, 0.67 * llrs, mapped)):
                bits = trx._decode_block(arr, codec, len(truth))
                if np.array_equal(bits.astype(np.uint8), truth):
                    ok[vi] += 1
        else:
            cases = [(L._sum_product, llrs), (L._sum_product, mapped),
                     (_fec_spa_kernel, llrs), (_fec_spa_kernel, mapped)]
            for vi, (kernel, arr) in enumerate(cases):
                bits = _fec_ldpc_decode(kernel, arr, len(truth), ncar)
                if np.array_equal(bits.astype(np.uint8), truth):
                    ok[vi] += 1
    return family, si, ok


def fig_qam16_recal(n_frames=300):
    """The calibration study repeated for 16-QAM (NORMAL mode, rate 1/3
    for code parity): does the BPSK-trained shape map really regress on
    the 4-bit constellation, and how do the code families compare?"""
    import multiprocessing as mp

    snrs = np.arange(-5.0, -0.9, 0.5)
    snrs23 = np.arange(0.0, 4.1, 0.5)  # rung-11 rate 2/3 waterfall
    conv_names = ["raw LLRs", "temperature only (0.67*L)",
                  "monotone shape map"]
    ldpc_names = ["min-sum, raw", "min-sum, shape map",
                  "sum-product, raw", "sum-product, shape map"]
    okc = np.zeros((3, len(snrs)))
    okl = np.zeros((4, len(snrs)))
    okc23 = np.zeros((3, len(snrs23)))
    cells = ([(0, si, float(snr), n_frames)
              for si, snr in enumerate(snrs)]
             + [(1, si, float(snr), n_frames)
                for si, snr in enumerate(snrs)]
             + [(2, si, float(snr), n_frames)
                for si, snr in enumerate(snrs23)])
    fam_name = {0: "conv13", 1: "ldpc13", 2: "conv23"}
    with mp.Pool(max(1, mp.cpu_count() - 1)) as pool:
        for family, si, counts in pool.imap_unordered(_qam16_cell, cells):
            grid = snrs23 if family == 2 else snrs
            (okc if family == 0 else
             okl if family == 1 else okc23)[:, si] = counts
            print(f"  {fam_name[family]} {grid[si]:+.1f} dB: "
                  + " ".join(str(c) for c in counts)
                  + f" /{n_frames}", flush=True)
    assert np.array_equal(okc[0], okc[1]), \
        "temperature-only must be bit-identical to raw (scale invariance)"
    assert np.array_equal(okc23[0], okc23[1])

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4.4))

    ax1.plot(snrs, okc[0] / n_frames, "o-", color="tab:red",
             label="raw LLRs")
    ax1.plot(snrs, okc[1] / n_frames, "x--", color="tab:blue", ms=8,
             label=r"temperature only ($0.67 \cdot L$)")
    ax1.plot(snrs, okc[2] / n_frames, "s-", color="tab:green",
             label="monotone shape map (BPSK-trained)")
    ax1.set_title("Viterbi chain (conv 1/3)")

    styles = [("tab:green", "s-"), ("tab:green", "^--"),
              ("tab:red", "o-"), ("tab:blue", "x-")]
    for vi, label in enumerate(ldpc_names):
        c, st = styles[vi]
        ax2.plot(snrs, okl[vi] / n_frames, st, color=c, label=label, ms=6)
    ax2.set_title("LDPC kernels (IRA 1/3)")

    ax3.plot(snrs23, okc23[0] / n_frames, "o-", color="tab:red",
             label="raw LLRs")
    ax3.plot(snrs23, okc23[2] / n_frames, "s-", color="tab:green",
             label="monotone shape map")
    ax3.set_title("Viterbi at the ladder rate (conv 2/3)\n"
                  "-- the regime behind the $\\mu \\leq 2$ gate")

    for ax in (ax1, ax2, ax3):
        ax.set_xlabel("SNR, dB (6 kHz convention)")
        ax.set_ylabel("P(frame decodes)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="upper left")
        ax.set_ylim(-0.03, 1.05)

    fig.suptitle("The calibration study for 16-QAM (NORMAL mode, "
                 f"rate 1/3 for code parity, {n_frames} frames/point, "
                 "article channel)", fontsize=11)
    fig.tight_layout()
    stem = ("qam16_recal" if n_frames == 300
            else f"qam16_recal_n{n_frames}")
    fig.savefig(RESULTS / f"{stem}.png", dpi=130)
    import json
    record = {
        "experiment": "calibration study for 16-QAM",
        "mode": "NORMAL", "modulation": "QAM16",
        "rate": "1/3 (code parity; ladder QAM16 rungs use 1/2..3/4)",
        "payload_bytes": 27, "n_frames_per_point": n_frames,
        "snr_grid_db": [float(x) for x in snrs],
        "snr_convention": "noise in the 6 kHz Nyquist band",
        "channel": "simulate_channel: random offset 300-1200, "
                   "CFO uniform +-60 Hz, AWGN",
        "seed_scheme": "np.random.default_rng([77, snr_index]); "
                       "shared draws across families and variants",
        "conv_variants": conv_names,
        "conv_r13_decodes": {conv_names[vi]: [int(x) for x in okc[vi]]
                             for vi in range(3)},
        "ldpc_variants": ldpc_names,
        "ldpc_decodes": {ldpc_names[vi]: [int(x) for x in okl[vi]]
                         for vi in range(4)},
        "conv_r23_snr_grid_db": [float(x) for x in snrs23],
        "conv_r23_decodes": {conv_names[vi]: [int(x) for x in okc23[vi]]
                             for vi in range(3)},
        "note": "shape map is the NORMAL-BPSK-trained default_llr_recal "
                "(deployment gates it to mu<=2; applied here to measure "
                "the regression); raw==temperature asserted",
    }
    np.save(RESULTS / f"{stem}_counts_conv.npy", okc)
    np.save(RESULTS / f"{stem}_counts_ldpc.npy", okl)
    with open(RESULTS / f"{stem}.json", "w") as f:
        json.dump(record, f, indent=2)
    print("saved", RESULTS / f"{stem}.png", "and", f"{stem}.json")


if __name__ == "__main__":
    if "--qam16-recal-only" in sys.argv:
        fig_qam16_recal()
        sys.exit(0)
    if "--extreme-recal-only" in sys.argv:
        fig_extreme_recal()
        sys.exit(0)
    if "--fec-comparison-only" in sys.argv:
        fig_fec_comparison()
        sys.exit(0)
    if "--ldpc-recal-only" in sys.argv:
        fig_ldpc_recal()
        sys.exit(0)
    if "--viterbi-recal-only" in sys.argv:
        fig_viterbi_recal()
        sys.exit(0)
    if "--llr-shape-only" in sys.argv:
        fig_llr_shape()
        sys.exit(0)
    if "--ladder-only" in sys.argv:
        fig_constellation_ladder()
        sys.exit(0)
    fig_zc_acf()
    fig_tx_signal()
    fig_constellations()
    fig_constellation_ladder()
    fig_llr_shape()
    fig_viterbi_recal()
    fig_ldpc_recal()
    fig_fec_comparison()
    fig_extreme_recal()
    fig_qam16_recal()
