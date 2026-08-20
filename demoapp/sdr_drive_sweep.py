#!/usr/bin/env python3
"""Drive-level sweep: how the transmitter's spectrum degrades with gain.

For each TX VGA setting it keys the radio with continuous OFDM and
measures, on a tinySA, the fundamental, the spectral shoulders either
side of the occupied band, and the 2nd/3rd harmonics. That answers the
practical question a HackRF raises: how much output can you take before
the signal stops being clean and a low-pass filter stops being optional.

    python3 sdr_drive_sweep.py --save results/sdr_drive_sweep

Safety: the analyser is wired straight to the antenna port, so the sweep
stops climbing if the measured fundamental approaches --max-dbm (the
tinySA Ultra's input is rated +10 dBm). The radio is unkeyed on every
exit path -- a killed script must never leave a transmitter on.
"""

import argparse
import signal
import sys
import threading
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

from sdr_driver import AUDIO_FS, SSBModulator, TX_PEAK  # noqa: E402
from tinysa import TinySA, _unkey  # noqa: E402

RATE = 2_400_000


def build_iq(seconds, if_offset):
    """A few seconds of continuous OFDM, modulated once and looped."""
    from ofdm_phy import Transceiver, Data, ModType, CCSpeed, LinkMode
    from ofdm_phy.modes import make_modem
    trx = Transceiver(make_modem(LinkMode.EXTREME))
    frame = trx.build_frame(Data(reserved=123, payload=b"DRIVE SWEEP"),
                            mod=ModType.QPSK, spd=CCSpeed.R12)
    audio = np.clip(frame / np.max(np.abs(frame)) * 32767, -32768, 32767)
    audio = audio[:int(seconds * AUDIO_FS)]
    mod = SSBModulator(RATE, if_offset)
    step = 1200
    iq = np.concatenate([mod(audio[a:a + step])
                         for a in range(0, len(audio), step)])
    return (iq * (TX_PEAK / 32768.0) / 127.0).astype(np.complex64)


class Keyer:
    """Streams the buffer on a loop until stopped, so a sweep sees a
    continuous signal rather than a burst."""

    def __init__(self, dev, S, iq):
        self.dev, self.S, self.iq = dev, S, iq
        self.stream = dev.setupStream(S.SOAPY_SDR_TX, S.SOAPY_SDR_CF32)
        self.run = False
        self.thread = None

    def _pump(self):
        while self.run:
            off = 0
            while self.run and off < len(self.iq):
                n = min(16384, len(self.iq) - off)
                sr = self.dev.writeStream(self.stream, [self.iq[off:off + n]],
                                          n, timeoutUs=int(2e6))
                off += sr.ret if sr.ret > 0 else n

    def start(self):
        self.run = True
        self.dev.activateStream(self.stream)
        self.thread = threading.Thread(target=self._pump, daemon=True)
        self.thread.start()

    def stop(self):
        self.run = False
        if self.thread:
            self.thread.join(timeout=3)
        try:
            self.dev.deactivateStream(self.stream)
        except Exception:
            pass

    def close(self):
        self.stop()
        try:
            self.dev.closeStream(self.stream)
        except Exception:
            pass


def band(f, d, lo, hi):
    n = min(len(f), len(d))
    f, d = f[:n], d[:n]
    m = (f >= lo) & (f <= hi)
    return float(np.max(d[m])) if m.any() else float("nan")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--freq", type=float, default=7.0e6)
    ap.add_argument("--if-offset", type=float, default=50e3)
    ap.add_argument("--gains", type=float, nargs="*",
                    default=[0, 8, 16, 24, 32, 40, 47])
    ap.add_argument("--amp", type=float, default=0.0)
    ap.add_argument("--max-dbm", type=float, default=-8.0,
                    help="stop climbing if the fundamental reaches this")
    ap.add_argument("--points", type=int, default=290)
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--save", default=None)
    args = ap.parse_args()

    import SoapySDR
    print("building the test waveform...")
    iq = build_iq(args.seconds, args.if_offset)
    print(f"  {len(iq)} samples ({len(iq)/RATE:.1f} s), peak "
          f"{np.max(np.abs(iq)):.3f} of full scale")

    sa = TinySA(args.port)
    print(f"  {sa.version()}")
    dev = SoapySDR.Device(dict(driver="hackrf"))
    dev.setSampleRate(SoapySDR.SOAPY_SDR_TX, 0, float(RATE))
    dev.setFrequency(SoapySDR.SOAPY_SDR_TX, 0,
                     float(args.freq - args.if_offset))
    dev.setGain(SoapySDR.SOAPY_SDR_TX, 0, "AMP", float(args.amp))
    keyer = Keyer(dev, SoapySDR, iq)

    def bail(*_):
        keyer.close(); _unkey(); sys.exit(1)
    signal.signal(signal.SIGINT, bail)
    signal.signal(signal.SIGTERM, bail)

    f0 = args.freq
    rows = []
    try:
        for vga in args.gains:
            dev.setGain(SoapySDR.SOAPY_SDR_TX, 0, "VGA", float(vga))
            keyer.start()
            time.sleep(1.0)
            sa.sweep(f0 - 12e3, f0 + 15e3, args.points, 3)
            fn, dn = sa.scan(f0 - 12e3, f0 + 15e3, args.points)
            sa.sweep(13.9e6, 21.1e6, args.points, 30)
            fw, dw = sa.scan(13.9e6, 21.1e6, args.points)
            keyer.stop()

            fund = band(fn, dn, f0 - 0.5e3, f0 + 3.0e3)
            # shoulders: 5-8 kHz outside the 300..2400 Hz occupied band
            sh_lo = band(fn, dn, f0 - 8e3, f0 - 5e3)
            sh_hi = band(fn, dn, f0 + 7.4e3, f0 + 10.4e3)
            h2 = band(fw, dw, 2 * f0 - 0.15e6, 2 * f0 + 0.15e6)
            h3 = band(fw, dw, 3 * f0 - 0.15e6, 3 * f0 + 0.15e6)
            rows.append((vga, fund, sh_lo - fund, sh_hi - fund,
                         h2 - fund, h3 - fund))
            print(f"  VGA {vga:4.0f} dB : fundamental {fund:+6.1f} dBm | "
                  f"shoulders {sh_lo-fund:+6.1f} / {sh_hi-fund:+6.1f} dBc | "
                  f"h2 {h2-fund:+6.1f} dBc | h3 {h3-fund:+6.1f} dBc")
            if fund >= args.max_dbm:
                print(f"  stopping: fundamental reached {fund:+.1f} dBm "
                      f"(limit {args.max_dbm:+.1f})")
                break
    finally:
        keyer.close()
        _unkey()
        sa.close()

    if not rows:
        return 1
    a = np.array(rows)
    if args.save:
        np.savetxt(f"{args.save}.csv", a, delimiter=",",
                   header="vga_db,fundamental_dbm,shoulder_lo_dbc,"
                          "shoulder_hi_dbc,h2_dbc,h3_dbc", comments="")
        print(f"  wrote {args.save}.csv")
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))
            ax1.plot(a[:, 0], a[:, 1], "o-", color="tab:blue")
            ax1.set_xlabel("TX VGA gain, dB")
            ax1.set_ylabel("fundamental, dBm")
            ax1.set_title("Output power")
            ax1.grid(alpha=0.3)
            for col, lab, st in ((2, "shoulder (low side)", "s-"),
                                 (3, "shoulder (high side)", "^-"),
                                 (4, "2nd harmonic", "o--"),
                                 (5, "3rd harmonic", "x--")):
                ax2.plot(a[:, 0], a[:, col], st, label=lab)
            ax2.axhline(-43, color="k", ls=":", lw=1,
                        label="-43 dBc (typical spurious limit)")
            ax2.set_xlabel("TX VGA gain, dB")
            ax2.set_ylabel("relative to fundamental, dBc")
            ax2.set_title("Spectral purity vs drive")
            ax2.grid(alpha=0.3)
            ax2.legend(fontsize=8)
            fig.suptitle("HackRF One drive-level sweep, continuous OFDM at "
                         f"{f0/1e6:.3f} MHz", fontsize=11)
            fig.tight_layout()
            fig.savefig(f"{args.save}.png", dpi=130)
            print(f"  wrote {args.save}.png")
        except ImportError:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
