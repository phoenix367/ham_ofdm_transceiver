#!/usr/bin/env python3
"""Decode a board's transmission from a HackRF recording, off the air.

    ./venv/bin/python tools/baofeng/reply_decode.py reply.npy

The recording is `ptt_carrier_check.py --iq` (50 kS/s, channel at 0 Hz).
This FM-demodulates the longest carrier, reports its deviation in the
modem band (rms, 99.9 % peak) and spectrum shape, resamples the audio
to 12 kHz and runs it through the C STREAMING receiver -- the boards'
own -- in all three modes (`cport/build/interf_rx`, `make -C cport
interfrx`), twice: flat, and through a radio-style 750 us de-emphasis.

The recording must hold the WHOLE frame. The EXTREME receiver needs
nearly its whole preamble: a clean frame reads 16.2 dB whole, 7.4 dB
with its first 0.4 s missing, -5.7 dB at 0.6 s, and does not decode at
0.8 s -- and a HackRF that is switched from TX to RX takes ~1 s to
start, so record a board's reply with `config ptt_lead 1500` on it (a
recording that started late read a clean Baofeng link as -7.5 dB).
"""
import subprocess
import sys
from pathlib import Path

import numpy as np
from scipy.signal import butter, lfilter, resample_poly, sosfiltfilt

REPO = Path(__file__).resolve().parents[2]
FS = 50e3
MODES = ["NORMAL", "ROBUST", "EXTREME"]


def decode(path, out_prefix=None, quiet=False):
    """Returns {'dev_rms', 'dev_peak', 'decodes': {variant: [(mode, snr), ...]}}."""
    say = (lambda *_: None) if quiet else print
    iq = np.load(path).astype(np.complex128)
    p = np.abs(iq) ** 2
    blk = 250
    pb = p[: len(p) // blk * blk].reshape(-1, blk).mean(1)
    floor = np.percentile(pb, 10)            # the carrier can fill most of the window
    idx = np.flatnonzero(pb > 30 * floor)
    segs = [s for s in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1) if len(s) > 20]
    if not segs:
        raise SystemExit(f"{path}: no carrier")
    s = max(segs, key=len)
    a, b = s[0] * blk, (s[-1] + 1) * blk
    say(f"carrier {a / FS:.2f}..{b / FS:.2f} s ({(b - a) / FS:.2f} s), "
        f"{10 * np.log10(pb[s].mean() / floor):+.1f} dB")
    a0 = max(a - 25000, 0)
    pre = a - a0
    y = iq[a0: b + 25000]
    fi = np.angle(y[1:] * np.conj(y[:-1])) * FS / (2 * np.pi)
    core = slice(pre + 2500, pre + (b - a) - 2500)
    aud = fi - np.median(fi[core])
    band = sosfiltfilt(butter(4, [300, 2700], btype="band", fs=FS, output="sos"), aud)[core]
    out = dict(dev_rms=float(np.std(band)), dev_peak=float(np.percentile(np.abs(band), 99.9)),
               decodes={})
    say(f"deviation 300-2700 Hz: rms {out['dev_rms']:.0f} Hz, 99.9% peak {out['dev_peak']:.0f} Hz")
    a12 = resample_poly(sosfiltfilt(butter(6, 4500, fs=FS, output="sos"), aud), 6, 25)
    c12 = a12[int(0.75 * 12000): -int(0.75 * 12000)]
    f = np.fft.rfftfreq(4096, 1 / 12e3)
    P = (np.abs(np.fft.rfft(c12[: len(c12) // 4096 * 4096].reshape(-1, 4096) * np.hanning(4096),
                            axis=1)) ** 2).mean(0)
    lv = [10 * np.log10(P[(f >= lo) & (f < lo + 300)].mean()) for lo in range(300, 3001, 300)]
    say("band level, dB re 300-600 Hz:",
        " ".join(f"{lo}:{v - lv[0]:+.1f}" for lo, v in zip(range(300, 3001, 300), lv)))
    al = np.exp(-1 / (12e3 * 750e-6))
    prefix = out_prefix or str(Path(path).with_suffix(""))
    for name, x in (("flat", a12), ("de-emph", lfilter([1 - al], [1, -al], a12))):
        x = x - x.mean()
        x = x / np.abs(x).max() * 16000
        fn = f"{prefix}_{name}.raw"
        x.astype(np.int16).tofile(fn)
        r = subprocess.run([str(REPO / "cport/build/interf_rx"), "-m", "0", "-m", "1", "-m", "2",
                            str(Path(fn).resolve())], capture_output=True, text=True)
        ok = [l.split() for l in r.stdout.splitlines() if l.startswith("EV") and l.split()[2] == "1"]
        out["decodes"][name] = [(MODES[int(t[1])], float(t[4])) for t in ok]
        say(f"{name:8s}: " + (", ".join(f"{m} snr {s:.2f} dB" for m, s in out["decodes"][name])
                              or "NO DECODE"))
    return out


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    if not (REPO / "cport/build/interf_rx").exists():
        sys.exit("build the receiver first: make -C cport interfrx")
    for path in sys.argv[1:]:
        if len(sys.argv) > 2:
            print(f"== {path}")
        decode(path)
