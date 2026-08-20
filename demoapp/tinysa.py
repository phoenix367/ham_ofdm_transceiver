#!/usr/bin/env python3
"""Capture spectra from a tinySA / tinySA Ultra over USB.

Used to measure what the SDR transmitter actually puts on the wire: run
a transmission (demoapp/sdr_bringup.py --tx ...) and sweep at the same
time, then compare the trace against the waveform's design.

  python3 tinysa.py --sweep 6.99e6 7.01e6 --points 450 --save trace.csv
  python3 tinysa.py --measure-tx        # transmit and sweep together

The device speaks a small text protocol over a USB CDC port: commands
terminated with CR, output ending at the "ch>" prompt.
"""

import argparse
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

DEFAULT_PORT = "/dev/ttyACM0"


class TinySA:
    def __init__(self, port=DEFAULT_PORT, timeout=2.0):
        import serial
        self.s = serial.Serial(port, 115200, timeout=timeout)
        time.sleep(0.2)
        self._drain()

    def _drain(self):
        self.s.reset_input_buffer()

    def cmd(self, text, settle=0.05, limit=30.0):
        """Send a command, return everything up to the next prompt."""
        self._drain()
        self.s.write((text + "\r").encode())
        time.sleep(settle)
        out, t0 = b"", time.time()
        while time.time() - t0 < limit:
            chunk = self.s.read(65536)
            if chunk:
                out += chunk
                if out.rstrip().endswith(b"ch>"):
                    break
            elif out:
                break
        text_out = out.decode(errors="replace")
        # strip the echoed command and the trailing prompt
        lines = [ln for ln in text_out.replace("\r", "").split("\n")]
        if lines and lines[0].strip() == text.strip():
            lines = lines[1:]
        return "\n".join(ln for ln in lines if ln.strip() != "ch>").strip()

    def version(self):
        return self.cmd("version").splitlines()[0]

    def sweep(self, start_hz, stop_hz, points=450, rbw_khz=None):
        """Configure the sweep; returns the frequency axis actually used."""
        self.cmd(f"sweep {int(start_hz)} {int(stop_hz)} {int(points)}")
        if rbw_khz is not None:
            self.cmd(f"rbw {int(rbw_khz)}")
        return np.linspace(float(start_hz), float(stop_hz), int(points))

    def scan(self, start_hz, stop_hz, points=290, limit=60.0):
        """Run ONE sweep synchronously and return (freqs, dBm).

        `data 0` returns whatever sweep last completed, which can predate
        the command -- fine for a free-running display, useless when the
        point is to compare a trace taken with the transmitter off against
        one taken with it on. `scan` blocks until the sweep it starts is
        finished, so the trace is always the one we asked for.
        """
        raw = self.cmd(f"scan {int(start_hz)} {int(stop_hz)} "
                       f"{int(points)} 3", settle=0.1, limit=limit)
        fs, db = [], []
        for ln in raw.splitlines():
            parts = ln.split()
            if len(parts) >= 2:
                try:
                    fs.append(float(parts[0])); db.append(float(parts[1]))
                except ValueError:
                    pass
        return np.asarray(fs), np.asarray(db)

    def trace(self, points=450, limit=30.0):
        """Last completed trace in dBm (free-running; see scan())."""
        raw = self.cmd("data 0", limit=limit)
        vals = []
        for ln in raw.splitlines():
            ln = ln.strip()
            if not ln or ln.startswith("data"):
                continue
            try:
                vals.append(float(ln.split()[-1]))
            except ValueError:
                pass
        return np.asarray(vals, dtype=float)

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


def summarise(freqs, dbm, label=""):
    """Occupied bandwidth and shoulders -- what matters for a transmitter."""
    peak_i = int(np.argmax(dbm))
    peak_f, peak_db = freqs[peak_i], dbm[peak_i]
    floor = float(np.median(dbm))
    print(f"  {label}peak {peak_db:+.1f} dBm at {peak_f/1e6:.5f} MHz, "
          f"median floor {floor:+.1f} dBm "
          f"(dynamic range {peak_db - floor:.1f} dB)")
    # -20 dB occupied bandwidth around the peak
    for down in (3.0, 20.0, 30.0):
        above = np.where(dbm >= peak_db - down)[0]
        if len(above) > 1:
            bw = freqs[above[-1]] - freqs[above[0]]
            print(f"  {label}-{down:.0f} dB bandwidth "
                  f"{bw/1e3:8.2f} kHz  "
                  f"({freqs[above[0]]/1e6:.5f} .. "
                  f"{freqs[above[-1]]/1e6:.5f} MHz)")
    return peak_f, peak_db, floor


def cmd_sweep(args):
    sa = TinySA(args.port)
    print(f"  {sa.version()}")
    freqs = sa.sweep(args.sweep[0], args.sweep[1], args.points, args.rbw)
    time.sleep(args.settle)
    dbm = sa.trace(args.points)
    sa.close()
    if len(dbm) != len(freqs):
        print(f"  note: {len(dbm)} points returned for {len(freqs)} "
              f"requested; using the device's count")
        freqs = np.linspace(args.sweep[0], args.sweep[1], len(dbm))
    summarise(freqs, dbm)
    if args.save:
        np.savetxt(args.save, np.column_stack([freqs, dbm]),
                   delimiter=",", header="freq_hz,dbm", comments="")
        print(f"  wrote {args.save}")
    return freqs, dbm


def _unkey():
    """Force the radio out of transmit. A transmitter left keyed by a
    killed process is both a bad measurement and a bad idea; opening a
    receive stream is the cheapest way to guarantee it is off."""
    try:
        import SoapySDR
        from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CF32
        dev = SoapySDR.Device(dict(driver="hackrf"))
        st = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
        dev.activateStream(st)
        buf = np.empty(65536, dtype=np.complex64)
        for _ in range(2):
            dev.readStream(st, [buf], len(buf), timeoutUs=int(1e6))
        dev.deactivateStream(st)
        dev.closeStream(st)
    except Exception as exc:
        print(f"  note: could not verify the radio is unkeyed ({exc})")


def cmd_measure_tx(args):
    """Sweep the analyser while the SDR transmits, and once with the
    transmitter off, so the signal can be separated from the ambient."""
    here = Path(__file__).resolve().parent
    py = str(here.parents[0] / "venv/bin/python")

    sa = TinySA(args.port)
    print(f"  {sa.version()}")
    freqs = sa.sweep(args.sweep[0], args.sweep[1], args.points, args.rbw)

    _unkey()                      # make sure nothing is still keyed
    time.sleep(0.5)
    print("  baseline sweep (transmitter off)...")
    freqs, base = sa.scan(args.sweep[0], args.sweep[1], args.points)

    print(f"  starting transmitter: {args.freq/1e6:.4f} MHz carrier, "
          f"VGA {args.vga:g}, amp {args.amp:g}")
    tx = subprocess.Popen(
        [py, "-u", str(here / "sdr_bringup.py"), "--tx",
         "--i-have-a-dummy-load", "--tx-mode", "extreme",
         "--repeat", str(args.repeat), "--rate", "2.4e6",
         "--freq", repr(args.freq - args.if_offset),
         "--if-offset", repr(args.if_offset),
         "--vga", repr(args.vga), "--amp", repr(args.amp)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(args.warmup)

    sweeps = []
    for i in range(args.sweeps):
        _, t = sa.scan(args.sweep[0], args.sweep[1], args.points)
        if len(t):
            sweeps.append(t)
            print(f"    sweep {i+1}/{args.sweeps}: peak "
                  f"{np.max(t):+.1f} dBm")
    tx.send_signal(signal.SIGINT)      # lets sdr_bringup unkey cleanly
    try:
        tx.wait(timeout=10)
    except Exception:
        tx.kill()
    _unkey()
    sa.close()

    if not sweeps:
        print("  !! no sweeps captured")
        return None
    n = min(len(s) for s in sweeps + [base])
    on = np.max(np.stack([s[:n] for s in sweeps]), axis=0)  # max-hold
    base = base[:n]
    freqs = freqs[:n] if len(freqs) >= n else np.linspace(
        args.sweep[0], args.sweep[1], n)

    print("\n  transmitter ON (max-hold of "
          f"{len(sweeps)} sweeps):")
    peak_f, peak_db, _ = summarise(freqs, on)
    print("  transmitter OFF (ambient):")
    summarise(freqs, base, label="")
    print(f"  signal rises {np.max(on) - np.max(base):+.1f} dB above the "
          f"ambient peak")

    stem = args.save or "tx_spectrum"
    np.savetxt(f"{stem}.csv",
               np.column_stack([freqs, on, base]), delimiter=",",
               header="freq_hz,dbm_tx_on,dbm_tx_off", comments="")
    print(f"  wrote {stem}.csv")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 4.6))
        ax.plot(freqs / 1e6, base, lw=0.8, color="tab:gray",
                label="transmitter off (ambient)")
        ax.plot(freqs / 1e6, on, lw=1.2, color="tab:red",
                label="transmitter on (max-hold)")
        ax.axvspan((args.freq + 300) / 1e6, (args.freq + 2400) / 1e6,
                   color="tab:blue", alpha=0.12,
                   label="designed occupied band\n(carrier+300..2400 Hz)")
        ax.set_xlabel("frequency, MHz")
        ax.set_ylabel("level, dBm")
        ax.set_title("SDR transmitter measured on a tinySA Ultra\n"
                     "(continuous EXTREME OFDM, 2.4 Msps)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(f"{stem}.png", dpi=130)
        print(f"  wrote {stem}.png")
    except ImportError:
        pass
    return freqs, on, base


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--sweep", nargs=2, type=float,
                    default=[6.995e6, 7.010e6],
                    metavar=("START_HZ", "STOP_HZ"))
    ap.add_argument("--points", type=int, default=450)
    ap.add_argument("--rbw", type=int, default=None,
                    help="resolution bandwidth in kHz (device default if "
                         "omitted)")
    ap.add_argument("--settle", type=float, default=1.5,
                    help="seconds to wait for a fresh sweep")
    ap.add_argument("--save", default=None)
    ap.add_argument("--measure-tx", action="store_true")
    ap.add_argument("--freq", type=float, default=7.0e6,
                    help="suppressed-carrier frequency to transmit on")
    ap.add_argument("--if-offset", type=float, default=50e3)
    ap.add_argument("--vga", type=float, default=20.0)
    ap.add_argument("--amp", type=float, default=0.0)
    ap.add_argument("--repeat", type=int, default=4)
    ap.add_argument("--sweeps", type=int, default=4)
    ap.add_argument("--warmup", type=float, default=3.0)
    args = ap.parse_args()

    if args.measure_tx:
        cmd_measure_tx(args)
    else:
        cmd_sweep(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
