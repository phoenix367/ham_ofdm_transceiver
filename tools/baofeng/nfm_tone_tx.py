#!/usr/bin/env python3
"""Transmit an NFM test tone (optionally with DCS) from a HackRF One.

Bench stand: HackRF One -> attenuator -> Baofeng, channel 464.425 MHz with
DCS D073I (= D506N). Verified 2026-11-07: plain NFM stays squelched, this
opens it:

    python3 tools/baofeng/nfm_tone_tx.py --dcs D073I --dev 2000 --seconds 30
    python3 tools/baofeng/nfm_tone_tx.py --selftest --dcs D073I   # no hardware

Keep the Baofeng's PTT off while it is wired to the HackRF unless the pad
is >= 40 dB: 5 W is +37 dBm into the HackRF's port.

Offset tuning: the LO sits OFFSET below the target and the FM carrier is
synthesised at +OFFSET, so the HackRF's LO/DC leakage lands 200 kHz away
from the channel instead of on top of the carrier.

DCS: 23-bit Golay(23,12) word (9-bit octal code + '100' + 11 parity bits,
generator 0xC75), sent LSB first, repeated continuously at 134.4 bit/s as
NRZ FSK low-passed at 300 Hz. Normal code: bit 1 = +deviation. Inverted
('I') codes complement all 23 bits. The transmission ends with the
turn-off code (134.4 Hz for 200 ms) so the receiver closes its squelch
without a noise burst.

  --selftest   demodulate the generated IQ offline and check the tone
               deviation and the DCS word; touches no hardware
  --audio NPY  send 12 kHz audio instead of the tone -- an OFDM frame from
               modem_frame.py -- at --audio-dev peak (default 1500 Hz),
               after --lead s of carrier. Verified 2026-11-17: an EXTREME
               NO_DATA frame, HackRF -> Baofeng speaker -> RC front end ->
               board1 PA0 (config rx_input 1), decoded on the board.
"""
import argparse
import signal
import time

import numpy as np
from scipy.signal import butter, resample_poly, sosfilt, sosfilt_zi

DCS_BPS = 134.4
AUD_FS = 50_000            # modulating signal rate; RF rate is a multiple


def dcs_word(code_oct, inverted):
    w = int(code_oct, 8) + 0x800
    r = w
    for _ in range(12):
        r <<= 1
        if r & 0x1000:
            r ^= 0x08EA
    w |= (r & 0x0FFE) << 11
    return w ^ 0x7FFFFF if inverted else w


def parse_dcs(s):
    """'D073I', '073I', '073N', '073' -> (word, label)."""
    s = s.upper().lstrip("D")
    inv = s.endswith("I")
    code = s.rstrip("IN")
    assert len(code) == 3 and all(c in "01234567" for c in code), s
    return dcs_word(code, inv), f"D{code}{'I' if inv else 'N'}"


def load_audio(a):
    """--audio: the whole modulating sequence at AUD_FS, in Hz of
    deviation -- lead, the file (peak = --audio-dev) repeated with gaps,
    tail. Plain FM, no pre-emphasis: the receiving radio's de-emphasis
    tilts it, and an OFDM receiver equalises a smooth tilt per carrier."""
    x = np.load(a.audio).astype(np.float64).ravel()
    g = np.gcd(int(a.audio_rate), AUD_FS)
    x = resample_poly(x, AUD_FS // g, int(a.audio_rate) // g)
    x *= a.audio_dev / np.max(np.abs(x))
    z = lambda s: np.zeros(int(round(s * AUD_FS)))
    parts = [z(a.lead)]
    for i in range(a.repeat):
        parts += [x] + ([z(a.gap)] if i < a.repeat - 1 else [])
    parts.append(z(a.tail))
    return np.concatenate(parts)


def generate(a, word, aud=None):
    """Yield complex64 chunks at a.rate: the whole transmission. `aud`
    (from load_audio) replaces the tone."""
    up = int(round(a.rate / AUD_FS))
    assert up * AUD_FS == a.rate, "rate must be a multiple of 50 kHz"
    sos = butter(4, 300, fs=AUD_FS, output="sos")
    zi = sosfilt_zi(sos) * 0.0
    pol = -1.0 if a.dcs_flip else 1.0
    # --seconds 0 = until stopped (range() is lazy, so a huge bound costs
    # nothing; ~6 years of samples)
    n_total = int(a.seconds * AUD_FS) if a.seconds > 0 else 10 ** 13
    if aud is not None:
        n_total = len(aud)
    n_off = int(0.2 * AUD_FS) if word is not None else 0
    chunk = AUD_FS // 10                       # 100 ms
    ph = 0.0
    for s0 in range(0, n_total + n_off, chunk):
        n = np.arange(s0, min(s0 + chunk, n_total + n_off))
        t = n / AUD_FS
        body = n < n_total
        if aud is None:
            f = np.where(body, a.dev * np.sin(2 * np.pi * a.tone * t), 0.0)
        else:
            f = np.where(body, aud[np.minimum(n, n_total - 1)], 0.0)
        if word is not None:
            bit = (np.floor(t * DCS_BPS).astype(np.int64)) % 23
            nrz = np.where((word >> bit) & 1, 1.0, -1.0) * pol
            turnoff = np.sign(np.sin(2 * np.pi * DCS_BPS * t))
            d = np.where(body, nrz, turnoff)
            d, zi = sosfilt(sos, d, zi=zi)
            f = f + a.dcs_dev * d
        f_rf = np.repeat(f + a.offset, up)
        phase = ph + 2 * np.pi * np.cumsum(f_rf) / a.rate
        ph = float(phase[-1] % (2 * np.pi))
        yield (a.level * np.exp(1j * phase)).astype(np.complex64)


def selftest(a, word, label, aud=None):
    if aud is not None:
        # demodulate the WHOLE transmission and compare it with the
        # modulating sequence it was built from
        x = np.concatenate(list(generate(a, word, aud)))
        fi = np.angle(x[1:] * np.conj(x[:-1])) * a.rate / (2 * np.pi) - a.offset
        up = int(round(a.rate / AUD_FS))
        fa = fi[: len(fi) // up * up].reshape(-1, up).mean(axis=1)[: len(aud) - 1]
        ref = aud[: len(fa)]
        if word is not None:   # take the DCS sub-audio out of the comparison
            fa = fa - sosfilt(butter(4, 300, fs=AUD_FS, output="sos"), fa)
            ref = ref - sosfilt(butter(4, 300, fs=AUD_FS, output="sos"), ref)
        err = fa - ref
        print(f"audio: {len(aud) / AUD_FS:.2f} s on air, peak deviation "
              f"{np.max(np.abs(fa)):.0f} Hz (set {a.audio_dev:.0f}); demodulated "
              f"vs modulating: error {20 * np.log10(np.std(err) / np.std(ref)):.1f} dB "
              f"re the signal")
        return
    # 3 s of signal whatever was asked -- including 0 (continuous), which
    # would otherwise concatenate an endless generator
    a.seconds = 3.0 if a.seconds <= 0 else min(a.seconds, 3.0)
    x = np.concatenate(list(generate(a, word)))
    fi = np.angle(x[1:] * np.conj(x[:-1])) * a.rate / (2 * np.pi) - a.offset
    up = int(round(a.rate / AUD_FS))
    fa = fi[: len(fi) // up * up].reshape(-1, up).mean(axis=1)
    body = fa[: int(a.seconds * AUD_FS)]
    tt = np.arange(len(body)) / AUD_FS
    amp = 2 * abs(np.mean(body * np.exp(-2j * np.pi * a.tone * tt)))
    print(f"tone: {a.tone:.0f} Hz at {amp:.0f} Hz deviation (set {a.dev:.0f}); "
          f"total peak deviation {np.max(np.abs(body[AUD_FS//2:])):.0f} Hz")
    if word is None:
        return
    sub = sosfilt(butter(4, 300, fs=AUD_FS, output="sos"),
                  sosfilt(butter(4, 600, fs=AUD_FS, output="sos"), body))
    sub = sub[AUD_FS // 2:]                      # skip filter start-up
    spb = AUD_FS / DCS_BPS
    best = None
    for ph in np.linspace(0, spb, 32, endpoint=False):
        idx = (ph + spb * np.arange(int((len(sub) - ph) / spb))).astype(int)
        v = sub[idx]
        m = np.mean(np.abs(v))
        if best is None or m > best[0]:
            best = (m, v)
    bits = (best[1] > 0).astype(int)
    rot = lambda w, k: ((w >> k) | (w << (23 - k))) & 0x7FFFFF
    found = None
    for s in range(23):
        w = sum(int(b) << i for i, b in enumerate(bits[s:s + 23]))
        if any(rot(word, k) == w for k in range(23)):
            found = s
            break
    print(f"DCS: sliced {len(bits)} bits, peak sub-audio deviation "
          f"{np.percentile(np.abs(sub), 99.9):.0f} Hz (set {a.dcs_dev:.0f}); "
          f"{label} word {'FOUND' if found is not None else 'NOT found'}"
          f" in the demodulated stream")
    if found is not None:
        ok = all(sum(int(b) << i for i, b in enumerate(bits[s:s + 23])) in
                 {rot(word, k) for k in range(23)}
                 for s in range(found, len(bits) - 23))
        print(f"     every 23-bit window from there decodes to it: {ok}")


ap = argparse.ArgumentParser()
ap.add_argument("--freq", type=float, default=464.425e6)
ap.add_argument("--ppm", type=float, default=0.0,
                help="HackRF clock error to pre-correct (+ = runs fast)")
ap.add_argument("--tone", type=float, default=1000.0)
ap.add_argument("--dev", type=float, default=2500.0, help="tone peak deviation, Hz")
ap.add_argument("--dcs", default=None, help="e.g. D073I, 023N")
ap.add_argument("--dcs-dev", type=float, default=600.0)
ap.add_argument("--dcs-flip", action="store_true",
                help="opposite FSK polarity (if the receiver won't open)")
ap.add_argument("--seconds", type=float, default=30.0,
                help="0 = continuous until SIGINT/SIGTERM")
ap.add_argument("--rate", type=float, default=2e6)
ap.add_argument("--offset", type=float, default=200e3)
ap.add_argument("--vga", type=float, default=20.0, help="TX VGA 0-47 dB")
ap.add_argument("--amp", type=float, default=0.0, help="TX amp 0 or 14 dB")
ap.add_argument("--level", type=float, default=0.7, help="IQ amplitude, <1")
ap.add_argument("--audio", default=None, metavar="NPY",
                help="transmit this audio (float .npy, e.g. modem_frame.py) "
                     "instead of the tone; --seconds is then ignored")
ap.add_argument("--audio-rate", type=float, default=12000.0)
ap.add_argument("--audio-dev", type=float, default=1500.0,
                help="peak deviation for --audio, Hz. Below the tone's 2500: "
                     "an OFDM frame's crest is ~7.6 dB, and the radio's "
                     "de-emphasis lifts its low carriers ~9 dB re 1 kHz -- "
                     "the 2500 Hz tone already read -3.7 dBFS at the board")
ap.add_argument("--lead", type=float, default=1.0,
                help="--audio: s of plain carrier first (squelch opens)")
ap.add_argument("--tail", type=float, default=0.5)
ap.add_argument("--repeat", type=int, default=1)
ap.add_argument("--gap", type=float, default=2.0, help="s between repeats")
ap.add_argument("--selftest", action="store_true")
a = ap.parse_args()

word, label = parse_dcs(a.dcs) if a.dcs else (None, "none")
aud = load_audio(a) if a.audio else None
if a.selftest:
    selftest(a, word, label, aud)
    raise SystemExit(0)

import SoapySDR
from SoapySDR import SOAPY_SDR_TX, SOAPY_SDR_CF32, SOAPY_SDR_END_BURST

sdr = SoapySDR.Device(dict(driver="hackrf"))
sdr.setSampleRate(SOAPY_SDR_TX, 0, a.rate)
sdr.setFrequency(SOAPY_SDR_TX, 0, (a.freq - a.offset) / (1 + a.ppm * 1e-6))
sdr.setGain(SOAPY_SDR_TX, 0, "AMP", a.amp)
sdr.setGain(SOAPY_SDR_TX, 0, "VGA", a.vga)
what = (f"audio {a.audio} x{a.repeat} peak dev +-{a.audio_dev:.0f} Hz "
        f"({len(aud) / AUD_FS:.1f} s on air)" if aud is not None
        else f"tone {a.tone:.0f} Hz dev +-{a.dev:.0f} Hz")
print(f"carrier {a.freq/1e6:.6f} MHz (LO {sdr.getFrequency(SOAPY_SDR_TX, 0)/1e6:.6f},"
      f" ppm corr {a.ppm:+.2f}), {what}, "
      f"DCS {label}{' FLIPPED' if a.dcs_flip else ''}"
      f"{f' dev +-{a.dcs_dev:.0f} Hz' if word is not None else ''}, "
      f"AMP {sdr.getGain(SOAPY_SDR_TX, 0, 'AMP'):.0f} dB, "
      f"VGA {sdr.getGain(SOAPY_SDR_TX, 0, 'VGA'):.0f} dB", flush=True)

def _stop(*_):
    raise KeyboardInterrupt
# SIGTERM/SIGHUP would otherwise kill us with the HackRF still keyed
# (measured in demoapp/sdr_bringup.py: -37.7 dBm until an RX stream was
# opened). Turn them into the same clean exit as Ctrl-C.
for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
    signal.signal(sig, _stop)

st = sdr.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32)
sdr.activateStream(st)
sent, errors = 0, 0
t0 = time.monotonic()
try:
    for buf in generate(a, word, aud):
        off = 0
        while off < len(buf):
            r = sdr.writeStream(st, [buf[off:]], len(buf) - off,
                                timeoutUs=1000000)
            if r.ret <= 0:
                errors += 1
                if errors > 20:
                    raise RuntimeError(f"writeStream failed: {r.ret}")
                continue
            off += r.ret
        sent += len(buf)
    sdr.writeStream(st, [np.zeros(1000, np.complex64)], 1000,
                    flags=SOAPY_SDR_END_BURST, timeoutUs=1000000)
    # writeStream returns once samples are buffered; let the radio drain
    time.sleep(max(0.0, sent / a.rate - (time.monotonic() - t0)) + 0.3)
except KeyboardInterrupt:
    print("interrupted")
finally:
    sdr.deactivateStream(st)
    sdr.closeStream(st)
print(f"done: {sent/a.rate:.1f} s sent in {time.monotonic()-t0:.1f} s, "
      f"write errors {errors}")
