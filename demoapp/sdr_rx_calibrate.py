#!/usr/bin/env python3
"""Receiver calibration against a traceable source.

The tinySA can act as a signal generator, which turns the measurement
around: instead of using it to look at our transmitter, feed a known CW
level into the SDR's input and measure what the receive chain makes of
it. That yields three things the rest of the project can only assume:

  * frequency accuracy -- the recovered audio tone offset is the
    combined LO error, directly comparable to the +-375 Hz CFO budget
  * level calibration -- audio amplitude against input dBm, i.e. how
    many dBm one unit of the receiver's int16 audio is worth
  * an ABSOLUTE sensitivity: the measured noise floor referred back to
    the antenna port, plus each mode's required SNR, gives the minimum
    discernible signal in dBm rather than in relative dB

    python3 sdr_rx_calibrate.py --save results/sdr_rx_calibration

Wiring: the tinySA's low port to the SDR's antenna port. The SDR only
receives here, and the generator tops out around -6 dBm, far below the
HackRF's damage level.
"""

import argparse
import signal
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

from sdr_driver import AUDIO_FS, SSBDemodulator, TX_PEAK  # noqa: E402
from tinysa import TinySA, _unkey  # noqa: E402

RATE = 2_400_000


class CalSource:
    """The tinySA's CAL output: a reference tone at a documented level,
    which is what the free-running signal generator failed to be (its
    output level had no measurable effect at the receiver, and it sweeps
    unless pinned). Frequencies available: 1, 2, 3, 4, 10, 15, 30 MHz."""

    def __init__(self, sa, mhz):
        self.sa, self.mhz = sa, int(mhz)

    def on(self):
        self.sa.cmd(f"caloutput {self.mhz}")
        time.sleep(1.2)

    def off(self):
        self.sa.cmd("caloutput off")
        time.sleep(1.2)


def capture_audio(dev, S, stream, dem, seconds):
    """Receive, demodulate, and return audio at the driver's int16 scale."""
    want = int(seconds * RATE)
    mtu = dev.getStreamMTU(stream)
    buf = np.empty(mtu, dtype=np.complex64)
    got, chunks = 0, []
    t0 = time.time()
    while got < want and time.time() - t0 < seconds * 4 + 3:
        sr = dev.readStream(stream, [buf], mtu, timeoutUs=int(1e6))
        if sr.ret > 0:
            chunks.append(buf[:sr.ret].copy())
            got += sr.ret
    if not chunks:
        return np.zeros(0)
    iq = np.concatenate(chunks)
    step = (RATE // AUDIO_FS) * 120
    audio = np.concatenate([dem(iq[a:a + step])
                            for a in range(0, len(iq) - step, step)])
    return audio * (32768.0 / TX_PEAK) * 127.0


def analyse(audio, want_hz):
    """Tone frequency/amplitude and the noise power over the 6 kHz band."""
    n = (len(audio) // 2) * 2
    if n < 4096:
        return None
    a = audio[:n]
    w = np.hanning(n)
    P = np.abs(np.fft.rfft(a * w)) ** 2
    f = np.fft.rfftfreq(n, 1 / AUDIO_FS)
    band = (f > 200) & (f < 3000)
    k = int(np.argmax(np.where(band, P, 0)))
    # parabolic interpolation for a sub-bin frequency estimate
    if 0 < k < len(P) - 1:
        d = 0.5 * (P[k - 1] - P[k + 1]) / (P[k - 1] - 2 * P[k] + P[k + 1] + 1e-30)
    else:
        d = 0.0
    tone_hz = (k + d) * AUDIO_FS / n
    # tone amplitude from the coherent bins, noise from everything else
    lo, hi = max(k - 3, 0), min(k + 4, len(P))
    tone_pow = P[lo:hi].sum()
    mask = np.ones(len(P), bool)
    mask[lo:hi] = False
    noise_psd = np.median(P[mask])              # per-bin noise power
    noise_pow = noise_psd * np.count_nonzero(f <= 6000)   # over 6 kHz
    amp = np.sqrt(2 * tone_pow) / (w.sum() / 2)  # int16-scale amplitude
    snr_db = 10 * np.log10(tone_pow / max(noise_pow, 1e-30))
    return tone_hz, amp, snr_db, noise_pow


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cal-mhz", type=float, default=10.0,
                    choices=(1.0, 2.0, 3.0, 4.0, 10.0, 15.0, 30.0),
                    help="tinySA CAL output frequency")
    ap.add_argument("--cal-dbm", type=float, default=-25.0,
                    help="CAL output level at the fundamental")
    ap.add_argument("--pad-db", type=float, default=40.0,
                    help="external attenuation between CAL out and the SDR")
    ap.add_argument("--audio-hz", type=float, default=1500.0)
    ap.add_argument("--if-offset", type=float, default=50e3)
    ap.add_argument("--lna", type=float, default=40.0)
    ap.add_argument("--vga", type=float, default=30.0)
    ap.add_argument("--vga-steps", type=float, nargs="*",
                    default=[30, 26, 22, 18, 14],
                    help="receiver gains used to check linearity")
    ap.add_argument("--seconds", type=float, default=0.5)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--save", default=None)
    args = ap.parse_args()

    import SoapySDR
    from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CF32
    from ofdm_phy.link import LADDER

    p_in = args.cal_dbm - args.pad_db          # level at the SDR input
    carrier = args.cal_mhz * 1e6 - args.audio_hz
    sa = TinySA(args.port)
    print(f"  {sa.version()}")
    cal = CalSource(sa, args.cal_mhz)

    dev = SoapySDR.Device(dict(driver="hackrf"))
    dev.setSampleRate(SOAPY_SDR_RX, 0, float(RATE))
    dev.setFrequency(SOAPY_SDR_RX, 0, float(carrier - args.if_offset))
    dev.setGain(SOAPY_SDR_RX, 0, "LNA", args.lna)
    dev.setGain(SOAPY_SDR_RX, 0, "AMP", 0.0)
    stream = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)

    def bail(*_):
        cal.off(); _unkey(); sys.exit(1)
    signal.signal(signal.SIGINT, bail)
    signal.signal(signal.SIGTERM, bail)

    print(f"  CAL {args.cal_mhz:.0f} MHz at {args.cal_dbm:+.0f} dBm through "
          f"{args.pad_db:.0f} dB of pad = {p_in:+.1f} dBm at the SDR")
    print(f"  receiver tuned so it lands at {args.audio_hz:.0f} Hz audio, "
          f"LNA {args.lna:g}")
    rows = []
    try:
        dev.activateStream(stream)
        dem = SSBDemodulator(RATE, args.if_offset)
        capture_audio(dev, SoapySDR, stream, dem, 0.3)

        for vga in args.vga_steps:
            dev.setGain(SOAPY_SDR_RX, 0, "VGA", float(vga))
            cal.on()
            dem = SSBDemodulator(RATE, args.if_offset)
            on = analyse(capture_audio(dev, SoapySDR, stream, dem,
                                       args.seconds), args.audio_hz)
            cal.off()
            dem = SSBDemodulator(RATE, args.if_offset)
            off = analyse(capture_audio(dev, SoapySDR, stream, dem,
                                        args.seconds), args.audio_hz)
            if on is None or off is None:
                continue
            tone_hz, amp, snr, npow = on
            rows.append((vga, tone_hz, amp, snr, npow, off[3]))
            print(f"  VGA {vga:4.0f} dB: tone {tone_hz:7.1f} Hz "
                  f"({tone_hz - args.audio_hz:+6.1f} Hz), amp {amp:8.1f}, "
                  f"tone/noise {snr:+6.1f} dB")
    finally:
        try:
            dev.deactivateStream(stream); dev.closeStream(stream)
        except Exception:
            pass
        cal.off()
        sa.close()

    if len(rows) < 2:
        return 1
    a = np.array(rows)
    vga, tone, amp, snr, npow_on, npow_off = (a[:, i] for i in range(6))

    ferr = float(np.mean(tone - args.audio_hz))
    ppm = ferr / (args.cal_mhz * 1e6) * 1e6
    print(f"\n  frequency accuracy")
    print(f"    tone offset {ferr:+.1f} Hz at {args.cal_mhz:.0f} MHz "
          f"= {ppm:+.2f} ppm (spread {np.std(tone):.1f} Hz)")
    print(f"    at 7 MHz that is {ppm * 7:+.0f} Hz, against the modem's "
          f"+-375 Hz acquisition range")

    print(f"\n  linearity (receiver gain stepped, input fixed)")
    adb = 20 * np.log10(np.maximum(amp, 1e-9))
    k, b = np.polyfit(vga, adb, 1)
    print(f"    audio dB vs VGA dB: slope {k:.3f} (ideal 1.000), "
          f"residual {np.std(adb - (k*vga+b)):.2f} dB")

    # noise referred to the input: the tone is a known p_in, so the
    # in-band noise power maps to an equivalent input level
    nf_dbm = p_in - float(np.mean(snr))
    print(f"\n  noise floor referred to the antenna port")
    print(f"    tone/noise in the 6 kHz band {np.mean(snr):+.1f} dB at "
          f"{p_in:+.1f} dBm in")
    print(f"    -> receiver noise {nf_dbm:+.1f} dBm in 6 kHz "
          f"(thermal would be -136 dBm, so NF ~ {nf_dbm + 136:.0f} dB)")
    print(f"\n  absolute sensitivity = noise floor + the mode's required SNR")
    for idx in (0, 4, 12):
        r = LADDER[idx]
        print(f"    rung {idx:2d}  {r.mode.name:7s} {r.mod.name:5s} "
              f"{r.user_rate:6.1f} bit/s : {nf_dbm + r.sens_db:+7.1f} dBm")
    print("    (that is the HackRF's own noise; on HF a real antenna "
          "delivers far more, so the link is externally limited)")

    if args.save:
        np.savetxt(f"{args.save}.csv", a, delimiter=",",
                   header="vga_db,tone_hz,audio_amp,snr_db,noise_on,"
                          "noise_off", comments="")
        print(f"\n  wrote {args.save}.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
