#!/usr/bin/env python3
"""SDR channel driver: the same device interface as driver.py, but the
audio crosses a real radio (HackRF One, or anything else SoapySDR drives)
instead of a simulated channel.

The console apps speak 12 kHz int16 audio over a Unix socket, so nothing
in cport/ or ofdm_console changes -- only what sits behind the socket:

    ofdm_console  <--audio-->  driver.py      (simulated channel)
    ofdm_console  <--audio-->  sdr_driver.py  (SSB over a real SDR)

Signal path (USB, exactly the model in ofdm_phy/rf.py):

  TX   12 kHz real audio -> analytic signal (63-tap Hilbert, the same
       design the receiver uses) -> interpolate to the SDR rate -> optional
       NCO offset -> int8 I/Q
  RX   int8 I/Q -> optional NCO offset -> decimate to 12 kHz -> real part
       (that IS an SSB product detector) -> int16 audio

Tune the radio to the *suppressed carrier* frequency: the 300-2400 Hz
audio band then occupies carrier+300..carrier+2400 Hz on air, and the LO
error between two stations appears as exactly the audio-band CFO the modem
already tracks (+-375 Hz design range, with the AFC netting loop on top).

Modes
  --loopback     two station devices cross-connected through the full
                 SSB/resampling chain (drop-in for driver.py, and the way
                 this file is tested without hardware)
  --device ...   one station device on a real radio via SoapySDR
  --selftest     verify the signal path: chunk-continuity plus a real OFDM
                 frame pushed through TX->RX and decoded

Rates: pick an SDR rate that is an integer multiple of 12 kHz. HackRF One's
minimum is 2 Msps, so 2.4 Msps (x200) is the natural choice.
"""

import argparse
import asyncio
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.signal import firwin, lfilter, remez

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

AUDIO_FS = 12000
TICK = 120                 # 10 ms of audio per pump tick
AUDIO_BW_HZ = 3500.0       # keep 0..3.5 kHz; the modem occupies 300..2400
HILBERT_TAPS = 63          # same design as the receiver's front end
TX_PEAK = 100.0            # int8 full scale is 127; leave OFDM headroom
TX_REF_PEAK = 32768.0      # audio peak mapped to TX_PEAK -- the drive
                           # level, exactly like setting mic gain on a rig.
                           # Measured: the C transmitter's frames run at
                           # rms ~14000 and reach int16 full scale, so
                           # anything lower here clips the waveform against
                           # the int8 rail (at 4000 it was an 18 dB
                           # overdrive and nothing decoded).


# --------------------------------------------------------------------------
# stateful DSP primitives (exact across chunk boundaries)
# --------------------------------------------------------------------------

class FIR:
    """FIR filter carrying its state, so a stream can be filtered in
    arbitrary chunks with no edge transients."""

    def __init__(self, taps):
        self.taps = np.asarray(taps, dtype=np.float64)
        self.zi = None

    def __call__(self, x):
        x = np.asarray(x)
        if self.zi is None or (np.iscomplexobj(x) != np.iscomplexobj(self.zi)):
            dt = np.complex128 if np.iscomplexobj(x) else np.float64
            self.zi = np.zeros(len(self.taps) - 1, dtype=dt)
        y, self.zi = lfilter(self.taps, 1.0, x, zi=self.zi)
        return y


class Delay:
    """Integer sample delay with state."""

    def __init__(self, n, dtype=np.float64):
        self.buf = np.zeros(n, dtype=dtype)

    def __call__(self, x):
        if len(self.buf) == 0:
            return x
        both = np.concatenate([self.buf, x])
        self.buf = both[len(x):] if len(x) < len(both) else both[-len(self.buf):]
        return both[:len(x)]


class Analytic:
    """Real signal -> analytic (positive-frequency) signal, streaming.

    The 63-tap type-III Hilbert transformer is the same design the fixed
    point receiver uses (ofdm_phy/fixed/dsp.py), so the driver's idea of
    'the analytic signal' matches the demodulator's exactly.
    """

    def __init__(self):
        taps = remez(HILBERT_TAPS, [0.02, 0.48], [1.0], type="hilbert", fs=1.0)
        # scipy's convention gives -sin for a cos input; negate so that
        # I + jQ carries positive frequencies
        self.q = FIR(-taps)
        self.i = Delay((HILBERT_TAPS - 1) // 2)

    def __call__(self, x):
        return self.i(np.asarray(x, dtype=np.float64)) + 1j * self.q(x)


class NCO:
    """Complex exponential with a phase accumulator that survives chunk
    boundaries (a restart every chunk would smear the constellation)."""

    def __init__(self, fs, freq_hz=0.0):
        self.step = 2.0 * np.pi * freq_hz / fs
        self.phase = 0.0

    def __call__(self, x):
        if self.step == 0.0:
            return x
        n = len(x)
        ph = self.phase + self.step * np.arange(n)
        self.phase = float((self.phase + self.step * n) % (2.0 * np.pi))
        return x * np.exp(1j * ph)


class FIRDecimator:
    """Anti-alias FIR then keep every Dth sample, carrying both the filter
    state and the decimation phase across chunks."""

    def __init__(self, factor, taps):
        self.factor = int(factor)
        self.fir = FIR(taps)
        self.phase = 0

    def __call__(self, x):
        y = self.fir(x)
        idx = np.arange(self.phase, len(y), self.factor)
        self.phase = int(self.phase + self.factor * len(idx) - len(y))
        return y[idx]


class BoxcarDecimator:
    """Cheap first-stage decimator: an averaging window of exactly D taps
    has spectral nulls at every multiple of fs/D -- i.e. centred on each
    band that would alias onto baseband. Costs no multiplies, which is what
    makes 2.4 Msps affordable in Python."""

    def __init__(self, factor):
        self.factor = int(factor)
        self.rem = None

    def __call__(self, x):
        if self.rem is not None and len(self.rem):
            x = np.concatenate([self.rem, x])
        n = (len(x) // self.factor) * self.factor
        self.rem = x[n:]
        if n == 0:
            return x[:0]
        return x[:n].reshape(-1, self.factor).mean(axis=1)


class FIRInterpolator:
    """Zero-stuff by L then FIR (state carried). Used only at the low rates
    -- the final stretch to the SDR rate is done by ZOHInterpolator."""

    def __init__(self, factor, taps):
        self.factor = int(factor)
        self.fir = FIR(np.asarray(taps) * factor)  # zero-stuffing gain

    def __call__(self, x):
        if self.factor == 1:
            return self.fir(x)
        up = np.zeros(len(x) * self.factor, dtype=x.dtype)
        up[::self.factor] = x
        return self.fir(up)


class ZOHInterpolator:
    """Zero-order hold by L: the images it leaves sit at multiples of the
    input rate, exactly where the hold's sinc has nulls. Free (np.repeat),
    which is what keeps the 2.4 Msps stage cheap."""

    def __init__(self, factor):
        self.factor = int(factor)

    def __call__(self, x):
        return np.repeat(x, self.factor) if self.factor > 1 else x


def _split_ratio(ratio):
    """Factor an integer resampling ratio into (fir1, fir2, cheap): the two
    FIR stages stay small and run at low rates, the remainder is handled by
    the multiply-free boxcar/hold stage at the SDR rate."""
    if ratio < 1 or int(ratio) != ratio:
        raise ValueError("SDR rate must be an integer multiple of 12 kHz")
    ratio = int(ratio)
    r1 = next((d for d in (5, 4, 3, 2) if ratio % d == 0), 1)
    rest = ratio // r1
    r2 = next((d for d in (5, 4, 3, 2) if rest % d == 0), 1)
    return r1, r2, rest // r2


def _lowpass(factor, fs_out, extra_taps=0):
    """Anti-alias/anti-image lowpass for one resampling stage. The passband
    only has to reach 3.5 kHz while the stopband may start near the input
    Nyquist, so these are short filters."""
    n = max(31, 8 * factor + 1 + extra_taps)
    n += 1 - (n % 2)
    return firwin(n, AUDIO_BW_HZ, fs=fs_out, window=("kaiser", 8.0))


# --------------------------------------------------------------------------
# SSB conversion
# --------------------------------------------------------------------------

class SSBModulator:
    """12 kHz real audio -> complex baseband at sdr_fs (upper sideband)."""

    def __init__(self, sdr_fs, if_offset_hz=0.0, lower=False):
        self.analytic = Analytic()
        r1, r2, r3 = _split_ratio(sdr_fs // AUDIO_FS)
        f1 = AUDIO_FS * r1
        f2 = f1 * r2
        self.up1 = FIRInterpolator(r1, _lowpass(r1, f1, extra_taps=32))
        self.up2 = FIRInterpolator(r2, _lowpass(r2, f2))
        self.up3 = ZOHInterpolator(r3)
        self.nco = NCO(sdr_fs, if_offset_hz)
        self.lower = lower
        self.stages = (r1, r2, r3)

    def __call__(self, audio):
        z = self.analytic(np.asarray(audio, dtype=np.float64))
        if self.lower:
            z = np.conj(z)
        return self.nco(self.up3(self.up2(self.up1(z))))


class SSBDemodulator:
    """Complex baseband at sdr_fs -> 12 kHz real audio (product detector)."""

    def __init__(self, sdr_fs, if_offset_hz=0.0):
        r1, r2, r3 = _split_ratio(sdr_fs // AUDIO_FS)
        # mirror of the modulator: the cheap stage runs first, at full rate
        self.nco = NCO(sdr_fs, -if_offset_hz)
        self.down1 = BoxcarDecimator(r3)
        f1 = sdr_fs // r3
        self.down2 = FIRDecimator(r2, _lowpass(r2, f1))
        f2 = f1 // r2
        self.down3 = FIRDecimator(r1, _lowpass(r1, f2, extra_taps=32))

    def __call__(self, iq):
        z = self.down3(self.down2(self.down1(self.nco(np.asarray(iq)))))
        # Re() of the complex baseband is the SSB product detector: it
        # halves uncorrelated noise but not the coherent signal, which is
        # the calibration ofdm_phy/rf.py documents.
        return np.real(z)


# --------------------------------------------------------------------------
# back ends
# --------------------------------------------------------------------------

class SoapyBackend:
    """One real radio via SoapySDR (HackRF One and friends).

    Half-duplex: the stream is switched between RX and TX around each
    transmission, which matches the protocol's simplex channel access (the
    station's 0.3 s turnaround covers the switch).

    NOTE: the DSP either side of this class is covered by --selftest and
    --loopback; the device calls themselves need hardware to exercise.
    """

    def __init__(self, args_str, freq_hz, sdr_fs, rx_gain, tx_gain,
                 antenna=None, elements=None):
        try:
            import SoapySDR
            from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_TX, SOAPY_SDR_CF32
        except ImportError:
            raise SystemExit(
                "SoapySDR python bindings not found. On Debian/Ubuntu:\n"
                "  sudo apt install python3-soapysdr soapysdr-module-hackrf\n"
                "(the loopback mode needs no bindings and no hardware)")
        self.S = SoapySDR
        # CF32 (not CS8): SoapySDR converts to the device's native int8
        # itself, and the buffers we hand it are complex float
        self.RX, self.TX, self.FMT = SOAPY_SDR_RX, SOAPY_SDR_TX, SOAPY_SDR_CF32
        self.dev = SoapySDR.Device(dict(kv.split("=", 1)
                                        for kv in args_str.split(",") if kv))
        for d in (self.RX, self.TX):
            self.dev.setSampleRate(d, 0, float(sdr_fs))
            self.dev.setFrequency(d, 0, float(freq_hz))
            if antenna:
                self.dev.setAntenna(d, 0, antenna)
        # HackRF (and most SoapySDR devices) expose named gain stages.
        # Measured on a HackRF One: the *aggregate* setGain barely moves
        # the level at all (32 -> 62 dB changed the I/Q rms by 3 %), while
        # the individual stages span rms 0.010 -> 0.94. Always drive the
        # stages when the device has them.
        self.elements = elements or {}
        for d, gain in ((self.RX, rx_gain), (self.TX, tx_gain)):
            names = list(self.dev.listGains(d, 0))
            named = {k: v for k, v in self.elements.items()
                     if k in names and v is not None}
            if named:
                for k, v in named.items():
                    self.dev.setGain(d, 0, k, float(v))
            else:
                self.dev.setGain(d, 0, float(gain))
        self.rx = self.dev.setupStream(self.RX, self.FMT)
        self.tx = self.dev.setupStream(self.TX, self.FMT)
        self.sdr_fs = float(sdr_fs)
        self.queued = 0
        self.tx_started = 0.0
        self.mode = None
        self._set_mode(self.RX)

    def _set_mode(self, mode):
        if self.mode is mode:
            return
        if self.mode is self.RX:
            self.dev.deactivateStream(self.rx)
        elif self.mode is self.TX:
            self.dev.deactivateStream(self.tx)
        self.dev.activateStream(self.rx if mode is self.RX else self.tx)
        if mode is self.TX:
            self.tx_started = time.monotonic()
            self.queued = 0
        self.mode = mode

    def read(self, n):
        """n complex samples at the SDR rate; zeros while transmitting."""
        if self.mode is not self.RX:
            return np.zeros(n, dtype=np.complex64)
        buf = np.empty(n, dtype=np.complex64)
        got = self.dev.readStream(self.rx, [buf], n, timeoutUs=1000000)
        n_ok = got.ret if got.ret > 0 else 0
        return buf[:n_ok]

    def write(self, iq):
        self._set_mode(self.TX)
        buf = iq.astype(np.complex64)
        self.dev.writeStream(self.tx, [buf], len(buf), timeoutUs=1000000)
        self.queued += len(buf)

    def end_tx(self):
        """Switch back to receive -- but only once the radio has actually
        played out what was queued. writeStream returns as soon as the
        samples are buffered, so deactivating immediately truncates the
        tail of the frame (measured during bring-up: 0.5 s of a 1.18 s
        frame still unsent when writeStream returned)."""
        if self.mode is self.TX and self.queued:
            drain = self.queued / float(self.sdr_fs) - (
                time.monotonic() - self.tx_started)
            if drain > 0:
                time.sleep(min(drain, 2.0))
        self.queued = 0
        self._set_mode(self.RX)

    def close(self):
        for s in (self.rx, self.tx):
            try:
                self.dev.deactivateStream(s)
                self.dev.closeStream(s)
            except Exception:
                pass


class LoopbackChannel:
    """Cross-connects two stations through the full SSB/resampling chain at
    the SDR sample rate: station A's int8 I/Q becomes station B's input and
    vice versa, with half-duplex muting.

    AWGN is added in the audio band so that snr_db keeps exactly the meaning
    it has in driver.py and in every measured result in the repo (noise over
    the 6 kHz Nyquist band); the RF path here contributes the int8
    quantization, and a real device contributes its own noise.
    """

    def __init__(self, sdr_fs, rng):
        self.sdr_fs = sdr_fs
        self.rng = rng
        self.air = [np.zeros(0, dtype=np.complex64) for _ in range(2)]

    def submit(self, idx, iq):
        self.air[idx] = np.concatenate([self.air[idx], iq])

    def take(self, idx, n):
        """Samples heard by station idx: whatever the *other* one sent."""
        src = self.air[1 - idx]
        out = np.zeros(n, dtype=np.complex64)
        k = min(n, len(src))
        out[:k] = src[:k]
        self.air[1 - idx] = src[k:]
        return out


# --------------------------------------------------------------------------
# driver
# --------------------------------------------------------------------------

class SDRDriver:
    def __init__(self, ddir, sdr_fs, backend, freq_hz, if_offset,
                 time_scale=1.0, snr_db=None, tx_ref=TX_REF_PEAK,
                 rx_gain=1.0):
        self.dir = Path(ddir)
        self.sdr_fs = sdr_fs
        self.backend = backend
        self.scale = time_scale
        self.n_dev = 2 if isinstance(backend, LoopbackChannel) else 1
        self.cfg = {"mode": "loopback" if self.n_dev == 2 else "sdr",
                    "sdr_rate": sdr_fs, "freq_hz": freq_hz,
                    "if_offset_hz": if_offset,
                    "snr_db": snr_db if snr_db is not None else 99.0,
                    "time_scale": time_scale}
        self.mod = [SSBModulator(sdr_fs, if_offset) for _ in range(self.n_dev)]
        self.dem = [SSBDemodulator(sdr_fs, if_offset)
                    for _ in range(self.n_dev)]
        self.writers = [None] * self.n_dev
        self.txq = [np.zeros(0, dtype=np.int16) for _ in range(self.n_dev)]
        self.sig_rms = [3000.0] * self.n_dev
        self.rng = np.random.default_rng(20260820)
        self.tx_ref = float(tx_ref)
        self.rx_gain = float(rx_gain)
        self.loopback = isinstance(backend, LoopbackChannel)

    # ---- audio <-> radio ----

    def _tx_tick(self, i, n_audio):
        """Take up to n_audio queued samples, SSB-modulate, hand to the
        radio. Returns True if this station transmitted."""
        if len(self.txq[i]) == 0:
            return False
        take = min(n_audio, len(self.txq[i]))
        audio = np.zeros(n_audio, dtype=np.float64)
        audio[:take] = self.txq[i][:take]
        self.txq[i] = self.txq[i][take:]
        # noise sizing follows driver.py: track the RMS of what is
        # actually being transmitted
        p = float(np.mean(audio ** 2))
        if p > 1e4:
            self.sig_rms[i] = 0.98 * self.sig_rms[i] + 0.02 * np.sqrt(p)
        # fixed drive level -- a per-chunk AGC would amplitude-modulate the
        # waveform at the tick rate and destroy the constellation
        iq = self.mod[i](audio) * (TX_PEAK / self.tx_ref)
        iq = (np.round(np.clip(iq.real, -127.0, 127.0))
              + 1j * np.round(np.clip(iq.imag, -127.0, 127.0)))
        iq = iq.astype(np.complex64) / 127.0      # normalised full scale
        if self.loopback:
            self.backend.submit(i, iq)
        else:
            self.backend.write(iq)
        return take > 0

    def _rx_tick(self, i, n_audio, muted):
        n_iq = n_audio * (self.sdr_fs // AUDIO_FS)
        if isinstance(self.backend, LoopbackChannel):
            iq = self.backend.take(i, n_iq)
        else:
            iq = self.backend.read(n_iq)
            if len(iq) < n_iq:
                iq = np.concatenate(
                    [iq, np.zeros(n_iq - len(iq), dtype=np.complex64)])
        if muted:
            iq = np.zeros_like(iq)
        # undo the TX drive scaling so the app sees ordinary audio levels
        # (on a real radio this is what --rx-gain sets)
        audio = self.dem[i](iq * 127.0) * (self.tx_ref / TX_PEAK)
        audio = audio * self.rx_gain
        if len(audio) < n_audio:   # decimator warm-up
            audio = np.concatenate([audio, np.zeros(n_audio - len(audio))])
        audio = audio[:n_audio]
        if self.loopback:
            snr = float(self.cfg["snr_db"])
            if snr < 60.0:
                noise_rms = max(self.sig_rms[1 - i]
                                * 10.0 ** (-snr / 20.0), 20.0)
                audio = audio + self.rng.standard_normal(len(audio)) * noise_rms
        return np.clip(audio, -32768, 32767).astype(np.int16)

    def tick(self):
        txing = [self._tx_tick(i, TICK) for i in range(self.n_dev)]
        if not isinstance(self.backend, LoopbackChannel):
            if not txing[0]:
                self.backend.end_tx()
        return [self._rx_tick(i, TICK, txing[i]) for i in range(self.n_dev)]

    # ---- devices ----

    async def station_conn(self, i, reader, writer):
        print(f"[sdr] s{i + 1}: app connected")
        self.writers[i] = writer
        try:
            while True:
                data = await reader.read(65536)
                if not data:
                    break
                chunk = np.frombuffer(data[:len(data) & ~1], dtype="<i2")
                self.txq[i] = np.concatenate([self.txq[i], chunk])
        except (ConnectionResetError, BrokenPipeError):
            pass
        self.writers[i] = None
        self.txq[i] = np.zeros(0, dtype=np.int16)
        print(f"[sdr] s{i + 1}: app disconnected")

    async def ctl_conn(self, reader, writer):
        try:
            while True:
                line = await reader.readline()
                if not line:
                    break
                try:
                    req = json.loads(line)
                except json.JSONDecodeError:
                    writer.write(b'{"error": "bad json"}\n')
                    continue
                if not req.get("get"):
                    for k in ("snr_db",):
                        if k in req:
                            self.cfg[k] = float(req[k])
                    print(f"[sdr] config: {self.cfg}")
                writer.write((json.dumps(self.cfg) + "\n").encode())
                await writer.drain()
        except (ConnectionResetError, BrokenPipeError):
            pass

    async def pump(self):
        tick_s = TICK / AUDIO_FS / self.scale
        next_t = time.monotonic()
        while True:
            deliver = self.tick()
            for i in range(self.n_dev):
                if self.writers[i] is not None:
                    try:
                        self.writers[i].write(
                            deliver[i].astype("<i2").tobytes())
                    except (ConnectionResetError, BrokenPipeError):
                        self.writers[i] = None
            next_t += tick_s
            dt = next_t - time.monotonic()
            if dt > 0:
                await asyncio.sleep(dt)
            else:
                next_t = time.monotonic()

    async def run(self):
        self.dir.mkdir(parents=True, exist_ok=True)
        self.servers = []   # keep references or asyncio closes the listeners
        for i in range(self.n_dev):
            path = self.dir / f"s{i + 1}.sock"
            path.unlink(missing_ok=True)
            self.servers.append(await asyncio.start_unix_server(
                lambda r, w, i=i: self.station_conn(i, r, w), path=str(path)))
        ctl = self.dir / "ctl.sock"
        ctl.unlink(missing_ok=True)
        self.servers.append(
            await asyncio.start_unix_server(self.ctl_conn, path=str(ctl)))
        devs = ", ".join(f"s{i + 1}.sock" for i in range(self.n_dev))
        print(f"[sdr] devices ready in {self.dir} ({devs}, ctl.sock); "
              f"{self.cfg['mode']} at {self.sdr_fs / 1e6:.3f} Msps "
              f"(x{self.sdr_fs // AUDIO_FS} of 12 kHz)")
        await self.pump()


# --------------------------------------------------------------------------
# self-test: the signal path must be transparent to the modem
# --------------------------------------------------------------------------

def selftest(sdr_fs):
    from ofdm_phy import Transceiver, Data, ModType, CCSpeed

    ok = True

    def check(name, cond):
        nonlocal ok
        print(f"[{'PASS' if cond else 'FAIL'}] {name}")
        ok = ok and bool(cond)

    ratio = sdr_fs // AUDIO_FS
    print(f"SDR rate {sdr_fs / 1e6:.3f} Msps = {ratio}x 12 kHz, "
          f"stages {SSBModulator(sdr_fs).stages}")

    # 1. chunked processing must equal one-shot processing
    rng = np.random.default_rng(7)
    audio = (rng.standard_normal(6000) * 2000).astype(np.float64)
    whole = SSBModulator(sdr_fs)(audio)
    chunked = SSBModulator(sdr_fs)
    parts = [chunked(audio[a:a + TICK]) for a in range(0, len(audio), TICK)]
    step = np.concatenate(parts)
    check("modulator: chunked == one-shot",
          np.allclose(whole, step[:len(whole)], atol=1e-9))

    iq = whole
    whole_rx = SSBDemodulator(sdr_fs)(iq)
    chunked_rx = SSBDemodulator(sdr_fs)
    n_iq = TICK * ratio
    parts = [chunked_rx(iq[a:a + n_iq]) for a in range(0, len(iq), n_iq)]
    step_rx = np.concatenate(parts)
    check("demodulator: chunked == one-shot",
          np.allclose(whole_rx, step_rx[:len(whole_rx)], atol=1e-9))

    # 2. round trip must preserve the audio band (300-2400 Hz).
    #    Measured in the spectrum: the chain has a large group delay, so a
    #    sample-by-sample comparison would measure the delay, not the
    #    fidelity. Note there is no real-valued RF stage here (the radio
    #    carries complex I/Q), so the factor of 1/2 that ofdm_phy/rf.py's
    #    product detector applies does not appear -- this path is unity
    #    gain by construction.
    n = 18000                      # an integer number of 1500 Hz cycles
    amp = 8000.0
    t = np.arange(n + 6000) / AUDIO_FS
    tone = amp * np.sin(2 * np.pi * 1500.0 * t)
    rt = SSBDemodulator(sdr_fs)(SSBModulator(sdr_fs)(tone))
    seg = rt[4000:4000 + n]        # past the filter transients
    P = np.abs(np.fft.rfft(seg)) ** 2
    k = int(round(1500.0 * n / AUDIO_FS))
    gain = 2.0 * np.sqrt(P[k]) / n / amp
    band = slice(1, int(round(6000.0 * n / AUDIO_FS)))
    spur = P[band].sum() - P[k - 2:k + 3].sum()
    sfdr_db = 10.0 * np.log10(spur / P[k] + 1e-18)
    check(f"round trip: 1500 Hz gain {gain:.3f} (expect ~1.0)",
          0.9 < gain < 1.1)
    check(f"round trip: in-band spurious {sfdr_db:.1f} dB (< -40 dB)",
          sfdr_db < -40.0)

    # 3. the real thing: an OFDM frame through TX -> RX must still decode
    trx = Transceiver()
    pkt = Data(reserved=123, payload=b"SDR PATH CHECK 73")
    frame = trx.build_frame(pkt, mod=ModType.QPSK, spd=CCSpeed.R12)
    lead = np.zeros(700)
    air = np.concatenate([lead, frame, lead])
    mod, dem = SSBModulator(sdr_fs), SSBDemodulator(sdr_fs)
    rx = np.concatenate([dem(mod(air[a:a + TICK]))
                         for a in range(0, len(air), TICK)])
    # int8 quantization, as the radio would apply it
    peak = np.max(np.abs(rx)) or 1.0
    q = np.round(rx / peak * TX_PEAK) / TX_PEAK * peak
    try:
        got, stats = trx.demod_frame(q)
        check(f"OFDM frame survives the SSB path "
              f"(SNR est {stats.snr_db:+.1f} dB)", got == pkt)
    except Exception as exc:
        check(f"OFDM frame survives the SSB path ({exc})", False)

    print("\nself-test:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(
        description="SDR channel driver (HackRF One via SoapySDR)")
    ap.add_argument("--dir", default="/tmp/ofdmsdr")
    ap.add_argument("--rate", type=float, default=2.4e6,
                    help="SDR sample rate; must be an integer multiple of "
                         "12 kHz (HackRF minimum is 2 Msps)")
    ap.add_argument("--freq", type=float, default=7.05e6,
                    help="suppressed-carrier frequency in Hz")
    ap.add_argument("--if-offset", type=float, default=None,
                    help="offset tuning, Hz: the carrier sits this far "
                         "above the radio's LO, so the LO/DC leakage lands "
                         "outside the 3.5 kHz decimation passband and is "
                         "filtered out. Defaults to 50 kHz on real "
                         "hardware (measured on a HackRF: with the LO on "
                         "the carrier the recovered audio is 98 % DC "
                         "leakage), 0 in loopback, which has none.")
    ap.add_argument("--device", default=None,
                    help="SoapySDR device args, e.g. driver=hackrf")
    ap.add_argument("--antenna", default=None)
    ap.add_argument("--rx-gain", type=float, default=32.0,
                    help="aggregate gain, used only if the device has no "
                         "named gain stages")
    ap.add_argument("--tx-gain", type=float, default=20.0)
    ap.add_argument("--lna", type=float, default=40.0,
                    help="HackRF RX LNA gain, 0-40 in 8 dB steps")
    ap.add_argument("--vga", type=float, default=40.0,
                    help="HackRF VGA gain (RX 0-62/2 dB, TX 0-47/1 dB); "
                         "40 leaves headroom on a quiet HF band, raise it "
                         "until sdr_bringup.py --rx reports clipping")
    ap.add_argument("--amp", type=float, default=0.0,
                    help="HackRF 14 dB amplifier: 0 or 14")
    ap.add_argument("--loopback", action="store_true",
                    help="two station devices cross-connected through the "
                         "full SSB chain (no hardware)")
    ap.add_argument("--snr-db", type=float, default=99.0,
                    help="loopback only: AWGN in the audio band, same "
                         "convention as driver.py")
    ap.add_argument("--time-scale", type=float, default=1.0,
                    help="loopback only: run the world N times faster")
    ap.add_argument("--tx-ref", type=float, default=TX_REF_PEAK,
                    help="audio peak mapped to the DAC's full scale (TX "
                         "drive level, like a rig's mic gain)")
    ap.add_argument("--rx-audio-gain", type=float, default=1.0,
                    help="extra gain on the recovered audio")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    sdr_fs = int(round(args.rate))
    if sdr_fs % AUDIO_FS:
        raise SystemExit(f"--rate must be a multiple of {AUDIO_FS} Hz "
                         f"(2.4e6 works: 200x)")
    if args.selftest:
        raise SystemExit(selftest(sdr_fs))

    if_offset = args.if_offset
    if args.loopback:
        if if_offset is None:
            if_offset = 0.0
        backend = LoopbackChannel(sdr_fs, np.random.default_rng(1))
        scale = args.time_scale
    else:
        if if_offset is None:
            if_offset = 50e3       # keep the LO leakage out of the audio
        if not args.device:
            raise SystemExit("need --device (e.g. --device driver=hackrf) "
                             "or --loopback")
        # tune the radio below the carrier by if_offset; the driver's NCO
        # puts the signal back, and the DC spike ends up out of band
        backend = SoapyBackend(args.device, args.freq - if_offset, sdr_fs,
                               args.rx_gain, args.tx_gain, args.antenna,
                               elements={"LNA": args.lna, "VGA": args.vga,
                                         "AMP": args.amp})
        scale = 1.0
        print(f"[sdr] radio open: {args.device}, carrier "
              f"{args.freq / 1e6:.4f} MHz (LO "
              f"{(args.freq - if_offset) / 1e6:.4f} MHz, offset tuned "
              f"{if_offset / 1e3:.0f} kHz), gains LNA {args.lna:g} / "
              f"VGA {args.vga:g} / AMP {args.amp:g}")
    drv = SDRDriver(args.dir, sdr_fs, backend, args.freq, if_offset,
                    time_scale=scale, snr_db=args.snr_db,
                    tx_ref=args.tx_ref, rx_gain=args.rx_audio_gain)
    try:
        asyncio.run(drv.run())
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(backend, "close"):
            backend.close()


if __name__ == "__main__":
    main()
