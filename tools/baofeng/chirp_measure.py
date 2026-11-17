#!/usr/bin/env python3
"""Measure a radio channel's LINEARITY with the board's test chirp.

    ./venv/bin/python tools/baofeng/chirp_measure.py -o chirp.npy          # record + analyse
    ./venv/bin/python tools/baofeng/chirp_measure.py --analyze chirp.npy   # analyse a recording

Board -> radio mic -> FM -> HackRF. The HackRF starts receiving FIRST
(it needs ~1 s to come up), then the board keys one `config chirp`
sweep per `mic_gain` level (a linear 280 -> 2700 Hz sweep at the RMS of
a modem frame, through exactly the path frames take; see
docs/radio.md). Each sweep is one carrier; the analysis demodulates
the FM, locates each sweep against the firmware's own phase law, and
reports per frequency and level:

  * the fundamental's deviation, and its GAIN relative to the quietest
    sweep with the input steps removed -- 0 dB means linear, a negative
    number is compression (the radio's deviation limiter);
  * the response SHAPE at the quietest level against a 750 us
    pre-emphasis;
  * the 2nd and 3rd harmonics in dBc (synchronous detection at 2x / 3x
    the known instantaneous phase);
  * SNDR in 250-3000 Hz: the fundamental against everything that is not
    the fundamental or those two harmonics;
  * the keyed-but-silent floor from each sweep's PTT lead, and how much
    of it sits in a harmonic comb (`--comb`, 244.14 Hz on the stand).

Run from the repository root. The board's other settings (rx_input,
ptt_lead, mic_gain) are restored to `--restore-gain` / lead 0 at the
end, because a reflash or a sweep must not leave the radio somewhere
nobody chose.
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy.signal import butter, fftconvolve, sosfiltfilt

REPO = Path(__file__).resolve().parents[2]
FS = 50e3                                  # ptt_carrier_check --iq rate
F1, F2 = 280.0, 2700.0                     # usb_radio_main.c CHIRP_F1/F2

ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
ap.add_argument("-o", "--out", help="record to this .npy (50 kS/s IQ), then analyse")
ap.add_argument("--analyze", help="only analyse an existing recording")
ap.add_argument("--serial", default="320047000851333438363436",
                help="the board wired to the radio (default: board1)")
ap.add_argument("--levels", default="0,-3,-6,-9,-12,-15,-18",
                help="mic_gain per sweep, dB, comma-separated")
ap.add_argument("--ms", type=int, default=4000, help="sweep length")
ap.add_argument("--lead", type=int, default=500,
                help="ptt_lead during the run: the radio's attack time, and "
                     "the silent stretch the floor is measured on")
ap.add_argument("--restore-gain", type=int, default=-9,
                help="mic_gain to leave the board at")
ap.add_argument("--comb", type=float, default=244.140625,
                help="fundamental of the pickup comb to split out of the floor")
a = ap.parse_args()
LEVELS = [int(v) for v in a.levels.split(",")]
if not (a.out or a.analyze):
    ap.error("give -o FILE (record) or --analyze FILE")


def record(path):
    sys.path.insert(0, str(REPO / "host"))
    import ofdm_modem as om
    gap = a.ms / 1000 + a.lead / 1000 + 1.6     # sweep + lead + release tail + margin
    listen = 3.5 + gap * len(LEVELS) + 2
    rx = subprocess.Popen([sys.executable, "-u", str(REPO / "tools/baofeng/ptt_carrier_check.py"),
                           "--listen", f"{listen:.0f}", "--iq", path],
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=REPO)

    def pump(m, secs):
        end = time.time() + secs
        while time.time() < end:
            for n, p in m.events(timeout=0.2):
                if n == "log":
                    print("  board:", p, flush=True)

    with om.OfdmModem(serial=a.serial) as m:
        m.config("rx_input", 1)
        m.config("ptt_lead", a.lead)
        pump(m, 3.0)                              # the HackRF is receiving by now
        for g in LEVELS:
            m.config("mic_gain", g)
            time.sleep(0.1)
            m.config("chirp", a.ms)
            pump(m, gap)
        m.config("mic_gain", a.restore_gain)
        m.config("ptt_lead", 0)
        pump(m, 0.5)
    out = rx.communicate()[0]
    for line in out.splitlines():
        if any(k in line for k in ("IQ:", "carrier at")):
            print("  " + line.strip())


def analyse(path):
    iq = np.load(path).astype(np.complex128)
    n = a.ms * 12
    T, DUR = (n - 1) / 12000.0, n / 12000.0
    fi = np.angle(iq[1:] * np.conj(iq[:-1])) * FS / (2 * np.pi)
    p = np.abs(iq) ** 2
    blk = 250
    pb = p[: len(p) // blk * blk].reshape(-1, blk).mean(1)
    idx = np.flatnonzero(pb > 30 * np.percentile(pb, 10))   # the carrier can fill most of it
    segs = [s for s in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
            if len(s) * blk / FS > 0.75 * DUR]
    if len(segs) != len(LEVELS):
        sys.exit(f"{len(segs)} sweeps found for {len(LEVELS)} levels")
    tt = np.arange(int(DUR * FS)) / FS
    phi = F1 * tt + (F2 - F1) * tt ** 2 / (2 * T)            # cycles, the firmware's law
    fins = F1 + (F2 - F1) * tt / T
    ref = np.sin(2 * np.pi * phi)
    bp = butter(4, [250, 3000], btype="band", fs=FS, output="sos")
    W = int(0.02 * FS)
    box = np.ones(W) / W
    res, leads = {}, []
    for g, s in zip(LEVELS, segs):
        a0, a1 = s[0] * blk, (s[-1] + 1) * blk
        x0 = fi[a0:a1] - np.median(fi[a0:a1])
        xb0 = sosfiltfilt(bp, x0)
        k = int(np.argmax(fftconvolve(xb0, ref[::-1], mode="valid")))
        x, xb = x0[k:k + len(tt)], xb0[k:k + len(tt)]
        z = {h: np.convolve(x * np.exp(-2j * np.pi * h * phi), box, "same") for h in (1, 2, 3)}
        recon = sum(np.real(2 * z[h] * np.exp(2j * np.pi * h * phi)) for h in (1, 2, 3))
        resid = sosfiltfilt(bp, x - recon)
        res[g] = dict(onset=k / FS, a={h: 2 * np.abs(z[h]) for h in (1, 2, 3)},
                      r=np.sqrt(np.convolve(resid ** 2, box, "same")),
                      peak=np.percentile(np.abs(xb), 99.9))
        lo_, hi_ = int(0.1 * FS), int(min(0.9 * k, k - 0.05 * FS))  # inside the silent lead
        if hi_ - lo_ > int(0.1 * FS):
            leads.append(x0[lo_:hi_] - np.median(x0[lo_:hi_]))

    def at(arr, f):
        i = int(np.argmin(np.abs(fins - f)))
        w = int(0.04 * FS)
        return np.median(arr[max(i - w, 0): i + w])

    GRID = [300, 400, 500, 700, 1000, 1400, 1800, 2200, 2500, 2650]
    lo = LEVELS[-1]
    head = "  f Hz " + "".join(f"{g:>8d}" for g in LEVELS)
    print("sweep onset after carrier start, s:",
          " ".join(f"{res[g]['onset']:.3f}" for g in LEVELS))
    print("\nFUNDAMENTAL deviation, Hz peak, per mic_gain:\n" + head)
    for f in GRID:
        print(f"  {f:5d}" + "".join(f"{at(res[g]['a'][1], f):8.0f}" for g in LEVELS))
    print(f"\nGAIN vs the {lo} dB sweep, dB (0 = linear, input steps removed):\n" + head)
    for f in GRID:
        r0 = at(res[lo]['a'][1], f) / 10 ** (lo / 20)
        print(f"  {f:5d}" + "".join(
            f"{20 * np.log10(at(res[g]['a'][1], f) / 10 ** (g / 20) / r0):+8.2f}" for g in LEVELS))
    print(f"\nRESPONSE SHAPE at {lo} dB, dB re 1 kHz (vs a 750 us pre-emphasis):")
    r1k = at(res[lo]['a'][1], 1000)
    for f in GRID:
        pe = 10 * np.log10((1 + (2 * np.pi * f * 750e-6) ** 2) / (1 + (2 * np.pi * 1000 * 750e-6) ** 2))
        print(f"  {f:5d} Hz {20 * np.log10(at(res[lo]['a'][1], f) / r1k):+6.1f} dB   (pre-emphasis {pe:+5.1f})")
    for h, name in ((2, "H2"), (3, "H3")):
        print(f"\n{name}, dBc (fundamentals whose harmonic is still in band):\n" + head)
        for f in [f for f in GRID if f <= 1400]:
            print(f"  {f:5d}" + "".join(
                f"{20 * np.log10(at(res[g]['a'][h], f) / at(res[g]['a'][1], f)):8.1f}" for g in LEVELS))
    print("\nSNDR in 250-3000 Hz, dB (fundamental vs all but f, 2f, 3f):\n" + head)
    for f in GRID:
        print(f"  {f:5d}" + "".join(
            f"{20 * np.log10(at(res[g]['a'][1], f) / np.sqrt(2) / at(res[g]['r'], f)):8.1f}" for g in LEVELS))
    print("\n99.9 % peak deviation in band, Hz:", " ".join(f"{g}:{res[g]['peak']:.0f}" for g in LEVELS))
    if leads:
        nf = min(len(l) for l in leads)
        lb = [sosfiltfilt(bp, l[:nf]) for l in leads]
        flo = np.concatenate(lb).std()
        P = np.mean([np.abs(np.fft.rfft(l * np.hanning(nf))) ** 2 for l in lb], axis=0)
        f = np.fft.rfftfreq(nf, 1 / FS)
        band = (f >= 250) & (f <= 3000)
        comb = np.zeros_like(band)
        for k in range(1, int(3000 / a.comb) + 1):
            comb |= np.abs(f - k * a.comb) < 6
        share = P[band & comb].sum() / P[band].sum()
        print(f"\nkeyed-silent floor (PTT leads), 250-3000 Hz: {flo:.1f} Hz rms; "
              f"{100 * share:.0f} % in {a.comb:.2f} Hz harmonics -> {flo * np.sqrt(1 - share):.1f} Hz rms without them")


if a.out:
    record(a.out)
analyse(a.out or a.analyze)
