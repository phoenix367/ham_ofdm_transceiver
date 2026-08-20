#!/usr/bin/env python3
"""End-to-end receive test over a real RF link, using an AM modulator as
the transmitter.

    laptop sound card -> AM modulator (carrier from an external generator)
                      -> attenuator -> HackRF -> this receiver

    python3 sdr_am_test.py --make-wav results/am_test.wav
    python3 sdr_am_test.py --receive --carrier 7.0e6 --seconds 60

Why this needs its own detector: AM is double-sideband *with* a carrier,
so the SSB product detector the driver uses would fold the lower
sideband on top of the upper one and wreck the signal. The fix is
synchronous detection -- recover the carrier (it is right there, unlike
in SSB), derotate everything by it so the carrier sits at exactly 0 Hz
and 0 phase, and then the real part is the modulating audio. Without the
lock, any frequency error between the generator and the SDR multiplies
the audio by a slow cosine, which splits every subcarrier in two.

Levels: the OFDM waveform has ~8 dB of peak-to-average, so set the
modulation depth on the 1 kHz alignment tone the WAV starts with -- it
is generated at the same *peak* amplitude as the frames, so a depth that
does not clip on the tone will not clip on the data either.
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
from scipy.io import wavfile
from scipy.signal import firwin, lfilter, resample_poly

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

from sdr_driver import (AUDIO_FS, BoxcarDecimator, FIRDecimator,  # noqa
                        NCO, _lowpass, _split_ratio)

RATE = 2_400_000
WAV_FS = 48000            # play natively; 12 kHz would be resampled by the OS
PEAK = 20000              # leaves headroom in the file


def make_wav(path):
    """Alignment tone plus a series of frames with distinct payloads."""
    from ofdm_phy import Transceiver, Data, ModType, CCSpeed, LinkMode
    from ofdm_phy.modes import make_modem

    plan = [("NORMAL", LinkMode.NORMAL, ModType.QPSK, CCSpeed.R12, b"RF TEST ONE"),
            ("NORMAL", LinkMode.NORMAL, ModType.BPSK, CCSpeed.R13, b"RF TEST TWO"),
            ("ROBUST", LinkMode.ROBUST, ModType.BPSK, CCSpeed.R13, b"RF TEST THREE")]
    parts = [np.zeros(int(1.5 * AUDIO_FS))]
    t = np.arange(int(3.0 * AUDIO_FS)) / AUDIO_FS
    parts.append(np.sin(2 * np.pi * 1000.0 * t))     # alignment tone, peak 1
    parts.append(np.zeros(int(1.5 * AUDIO_FS)))
    print("  contents:")
    print(f"    0.0 s  1.5 s silence")
    print(f"    1.5 s  3.0 s 1 kHz alignment tone (same peak as the frames)")
    for name, mode, mod, spd, payload in plan:
        trx = Transceiver(make_modem(mode))
        f = trx.build_frame(Data(reserved=123, payload=payload),
                            mod=mod, spd=spd)
        f = f / np.max(np.abs(f))                     # normalise to peak 1
        at = sum(len(p) for p in parts) / AUDIO_FS
        print(f"    {at:4.1f} s  {name} {mod.name} {spd.name}, "
              f"{len(f)/AUDIO_FS:.1f} s, payload {payload!r}")
        parts.append(f)
        parts.append(np.zeros(int(2.0 * AUDIO_FS)))
    audio = np.concatenate(parts)
    up = resample_poly(audio, WAV_FS // AUDIO_FS, 1)
    up = np.clip(up * PEAK, -32768, 32767).astype(np.int16)
    wavfile.write(path, WAV_FS, up)
    print(f"  wrote {path}: {len(up)/WAV_FS:.1f} s at {WAV_FS} Hz, "
          f"peak {np.max(np.abs(up))}")


class AMDetector:
    """Synchronous AM detection: decimate to 12 kHz complex, recover the
    carrier with a narrow low-pass, derotate by it, take the real part."""

    def __init__(self, if_offset, carrier_bw=120.0):
        r1, r2, r3 = _split_ratio(RATE // AUDIO_FS)
        self.nco = NCO(RATE, -if_offset)
        self.d1 = BoxcarDecimator(r3)
        f1 = RATE // r3
        self.d2 = FIRDecimator(r2, _lowpass(r2, f1))
        f2 = f1 // r2
        self.d3 = FIRDecimator(r1, _lowpass(r1, f2, extra_taps=32))
        # narrow low-pass isolating the carrier from the 300-2400 Hz audio
        self.carrier_taps = firwin(301, carrier_bw, fs=AUDIO_FS,
                                   window=("kaiser", 8.0))
        self.zi = None

    def __call__(self, iq):
        z = self.d3(self.d2(self.d1(self.nco(np.asarray(iq)))))
        if self.zi is None:
            self.zi = np.zeros(len(self.carrier_taps) - 1, dtype=np.complex128)
        c, self.zi = lfilter(self.carrier_taps, 1.0, z, zi=self.zi)
        mag = np.abs(c)
        ref = np.where(mag > 1e-12, c / np.maximum(mag, 1e-12), 1.0)
        y = z * np.conj(ref)          # carrier now at 0 Hz, 0 phase
        return np.real(y), mag


def receive(args):
    import SoapySDR
    from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CF32
    from ofdm_phy import Transceiver, LinkMode
    from ofdm_phy.modes import make_modem

    dev = SoapySDR.Device(dict(driver="hackrf"))
    dev.setSampleRate(SOAPY_SDR_RX, 0, float(RATE))
    dev.setFrequency(SOAPY_SDR_RX, 0, float(args.carrier - args.if_offset))
    for k, v in (("LNA", args.lna), ("VGA", args.vga), ("AMP", 0.0)):
        dev.setGain(SOAPY_SDR_RX, 0, k, float(v))
    st = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
    dev.activateStream(st)
    mtu = dev.getStreamMTU(st)
    buf = np.empty(mtu, dtype=np.complex64)

    det = AMDetector(args.if_offset)
    print(f"  listening on {args.carrier/1e6:.4f} MHz for "
          f"{args.seconds:.0f} s (LO {(args.carrier-args.if_offset)/1e6:.4f} "
          f"MHz, LNA {args.lna:g} VGA {args.vga:g})")
    audio, carrier, t0, last = [], [], time.time(), 0.0
    while time.time() - t0 < args.seconds:
        sr = dev.readStream(st, [buf], mtu, timeoutUs=int(1e6))
        if sr.ret <= 0:
            continue
        a, mag = det(buf[:sr.ret])
        audio.append(a)
        carrier.append(mag)
        now = time.time() - t0
        if now - last >= 5.0:
            last = now
            cm = float(np.mean(mag)) if len(mag) else 0.0
            am = float(np.sqrt(np.mean(a ** 2))) if len(a) else 0.0
            print(f"    t={now:5.1f}s  carrier {20*np.log10(cm+1e-12):+6.1f} "
                  f"dBFS, audio rms {20*np.log10(am+1e-12):+6.1f} dBFS")
    dev.deactivateStream(st)
    dev.closeStream(st)

    a = np.concatenate(audio)
    c = np.concatenate(carrier)
    print(f"  captured {len(a)/AUDIO_FS:.1f} s of audio; carrier "
          f"{20*np.log10(np.mean(c)+1e-12):+.1f} dBFS")
    if np.mean(c) < 1e-5:
        print("  !! no carrier found -- check the generator, the modulator "
              "and --carrier")
    # scale to the int16 domain the decoders expect
    a = a - np.mean(a)
    peak = np.max(np.abs(a)) or 1.0
    a16 = np.clip(a / peak * 12000, -32768, 32767)
    if args.save:
        wavfile.write(args.save, AUDIO_FS, a16.astype(np.int16))
        print(f"  wrote {args.save}")

    print("  decoding...")
    found = 0
    for mode in (LinkMode.NORMAL, LinkMode.ROBUST, LinkMode.EXTREME):
        trx = Transceiver(make_modem(mode))
        pos, guard = 0, 0
        while pos < len(a16) - 2000 and guard < 40:
            guard += 1
            try:
                pkt, stats = trx.demod_frame(a16[pos:])
            except Exception:
                break
            print(f"    {mode.name}: DECODED payload={pkt.payload!r} "
                  f"snr={stats.snr_db:+.1f} dB cfo={stats.cfo_hz:+.1f} Hz")
            found += 1
            pos += int(1.0 * AUDIO_FS)
    print(f"  {found} frame(s) decoded")
    return 0 if found else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--make-wav", metavar="PATH")
    ap.add_argument("--receive", action="store_true")
    ap.add_argument("--carrier", type=float, default=7.0e6)
    ap.add_argument("--if-offset", type=float, default=50e3)
    ap.add_argument("--lna", type=float, default=40.0)
    ap.add_argument("--vga", type=float, default=30.0)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--save", default=None,
                    help="write the recovered audio as a 12 kHz WAV")
    args = ap.parse_args()
    if args.make_wav:
        make_wav(args.make_wav)
        return 0
    if args.receive:
        return receive(args)
    ap.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
