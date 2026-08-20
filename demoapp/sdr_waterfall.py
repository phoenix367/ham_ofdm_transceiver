#!/usr/bin/env python3
"""Waterfall plots of frames actually received over the air.

Takes the recording made by sdr_am_test.py --receive, locates every
frame in it with the real detector, and plots the spectrogram: the whole
capture, then each frame close up. Unlike the simulated pictures
elsewhere in the project this is measured RF -- what the demodulator was
handed, not what a channel model produced.

    python3 sdr_waterfall.py --wav ../results/am_rx_offair.wav \\
        --save ../results/sdr_waterfall

The frequency axis stops at 3 kHz because that is where the modem lives
(300-2400 Hz, 23 subcarriers at 93.75 Hz). The analysis FFT is 128 bins
at 12 kHz, i.e. exactly the modem's own subcarrier grid, so each row of
the waterfall is one OFDM bin.
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.io import wavfile
from scipy.signal import spectrogram

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ofdm_phy import Transceiver, LinkMode          # noqa: E402
from ofdm_phy.modes import make_modem               # noqa: E402

FS = 12000
NFFT = 128          # 93.75 Hz bins -- the modem's own subcarrier spacing


def find_frames(audio):
    """Every frame in the recording, with its true start sample."""
    found = []
    for mode in (LinkMode.NORMAL, LinkMode.ROBUST, LinkMode.EXTREME):
        trx = Transceiver(make_modem(mode))
        pos = 0
        while pos < len(audio) - 2 * FS:
            try:
                pkt, st = trx.demod_frame(audio[pos:pos + int(12 * FS)])
            except Exception:
                pos += int(0.25 * FS)
                continue
            # demod_frame reports where the HEADER starts; the frame
            # itself begins one preamble earlier (checked against the
            # burst onset: 10.838 s computed vs 10.823 s measured)
            start = pos + st.start_sample - preamble_samples(mode.name)
            if not any(abs(start - f["start"]) < FS for f in found):
                found.append({"start": start, "mode": mode.name,
                              "payload": bytes(pkt.payload),
                              "snr": st.snr_db, "hdr": st.header})
            pos += int(0.25 * FS)
    return sorted(found, key=lambda f: f["start"])


def preamble_samples(mode_name):
    from ofdm_phy.modes import MODE_SPECS, LinkMode as LM
    sp = MODE_SPECS[LM[mode_name]]
    return 3 * sp.newman_tile * 128 + (32 + sp.sym_tile * 128)


def frame_layout(mode_name, hdr):
    """Preamble, header and data lengths in seconds, computed exactly
    rather than guessed from the envelope: the preamble is three tone
    fields of newman_tile x fft_bins plus one ZC symbol, the header is
    always 6 tiled symbols, and the data block follows from the header's
    own modulation, code rate and length fields."""
    from ofdm_phy.modes import MODE_SPECS, LinkMode as LM
    from ofdm_phy.transceiver import CODECS, MAPPERS
    sp = MODE_SPECS[LM[mode_name]]
    sym = 32 + sp.sym_tile * 128
    pre = 3 * sp.newman_tile * 128 + sym
    mapper = MAPPERS[hdr.mod]
    coded = CODECS[hdr.spd].calc_cc_elements(hdr.len)
    n_data = -(-coded // (16 * mapper.MU))
    # the ZC symbol is shown separately from the tone fields: it closes
    # the preamble but occupies all 23 carriers, so on a waterfall it
    # looks like data and reads as if the preamble ran into the header
    tones = 3 * sp.newman_tile * 128
    return (tones / FS, sym / FS, 6 * sym / FS, n_data * sym / FS)


def waterfall(ax, audio, t0=0.0, nfft=NFFT, overlap=0.75, vmax_ref=None,
              span_db=32.0):
    f, t, S = spectrogram(audio, fs=FS, nperseg=nfft,
                          noverlap=int(nfft * overlap), window="hann",
                          mode="magnitude")
    S = 20 * np.log10(S + 1e-9)
    vmax = vmax_ref if vmax_ref is not None else np.percentile(S, 99.5)
    im = ax.pcolormesh(t + t0, f, S, shading="auto",
                       vmin=vmax - span_db, vmax=vmax, cmap="turbo")
    ax.set_ylim(0, 3000)
    ax.set_ylabel("Hz")
    return im, vmax


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--wav", default="../results/am_rx_offair.wav")
    ap.add_argument("--save", default="../results/sdr_waterfall")
    args = ap.parse_args()

    fs, x = wavfile.read(args.wav)
    audio = x.astype(float)
    if fs != FS:
        raise SystemExit(f"expected {FS} Hz, got {fs}")
    print(f"  {args.wav}: {len(audio)/FS:.1f} s")

    frames = find_frames(audio)
    for fr in frames:
        print(f"    {fr['start']/FS:6.2f} s  {fr['mode']:7s} "
              f"snr {fr['snr']:+5.1f} dB  {fr['payload']!r}")
    if not frames:
        raise SystemExit("no frames found")

    # one playback of the test file is enough; show the span that holds
    # the frames plus a little air either side
    lo = max(0, frames[0]["start"] - int(2.0 * FS))
    hi = min(len(audio), frames[-1]["start"] + int(10.0 * FS))
    seg = audio[lo:hi]

    n = len(frames)
    fig = plt.figure(figsize=(12, 3.0 + 3.0))
    gs = fig.add_gridspec(2, n, height_ratios=[1.05, 1.0], hspace=0.62,
                          wspace=0.25)

    ax0 = fig.add_subplot(gs[0, :])
    im, vmax = waterfall(ax0, seg, t0=lo / FS)
    ax0.set_title("Received off air: the whole capture", fontsize=10)
    ax0.set_xlabel("time, s")
    for fr in frames:
        ax0.annotate(f"{fr['mode']}\n{fr['payload'].decode(errors='replace')}",
                     xy=(fr["start"] / FS, 2500), xytext=(0, 6),
                     textcoords="offset points", fontsize=7, ha="left",
                     color="white",
                     bbox=dict(boxstyle="round,pad=0.15", fc="black",
                               alpha=0.45, lw=0))
        ax0.axvline(fr["start"] / FS, color="white", lw=0.7, alpha=0.7)
    fig.colorbar(im, ax=ax0, pad=0.01).set_label("dB", fontsize=8)

    band_handles = []
    for i, fr in enumerate(frames):
        # the frame map gets its own strip above the spectrogram, with a
        # gap, rather than being laid over the data
        sub = gs[1, i].subgridspec(2, 1, height_ratios=[1, 7], hspace=0.12)
        axb = fig.add_subplot(sub[0])
        ax = fig.add_subplot(sub[1])
        ton_s, zc_s, hdr_s, dat_s = frame_layout(fr["mode"], fr["hdr"])
        flen = ton_s + zc_s + hdr_s + dat_s
        t_start = fr["start"] / FS
        margin = 0.15 * flen
        a = max(0, fr["start"] - int(margin * FS))
        b = min(len(audio), fr["start"] + int((flen + margin) * FS))
        waterfall(ax, audio[a:b], t0=a / FS)
        bands = ((0.0, ton_s, "#dce8ff", "tones"),
                 (ton_s, zc_s, "#8ab4ff", "ZC"),
                 (ton_s + zc_s, hdr_s, "#ffd24d", "header"),
                 (ton_s + zc_s + hdr_s, dat_s, "#7fe07f", "data"))
        for off, width, col, lab in bands:
            h = axb.axvspan(t_start + off, t_start + off + width, color=col)
            if i == 0:
                band_handles.append((h, lab))
            ax.axvline(t_start + off, color="white", lw=0.6, alpha=0.35)
        axb.set_xlim(a / FS, b / FS)
        axb.set_ylim(0, 1)
        axb.set_xticks([])
        axb.set_yticks([])
        for sp_ in axb.spines.values():
            sp_.set_visible(False)
        axb.set_title(f"{fr['mode']}  "
                      f"{fr['payload'].decode(errors='replace')}"
                      f"  ({fr['snr']:+.1f} dB, {flen:.2f} s)", fontsize=9)
        ax.set_xlabel("time, s")
        if i:
            ax.set_ylabel("")

    fig.legend([h for h, _ in band_handles], [l for _, l in band_handles],
               loc="lower center", ncol=4, fontsize=8, frameon=False,
               bbox_to_anchor=(0.5, -0.055), handlelength=1.6,
               columnspacing=1.4,
               title="frame structure, computed from the decoded header",
               title_fontsize=8)
    fig.suptitle("OFDM frames received over a 7 MHz RF path "
                 "(12 kHz audio, 128-bin FFT = the 93.75 Hz subcarrier "
                 "grid)", fontsize=11)
    fig.savefig(f"{args.save}.png", dpi=130, bbox_inches="tight")
    print(f"  wrote {args.save}.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
