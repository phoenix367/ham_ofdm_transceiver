#!/usr/bin/env python3
"""Virtual channel driver: exposes two station "devices" and a control
device as Unix sockets, and moves 12 kHz int16 audio between the stations
through a simulated HF channel in (scaled) real time.

Devices (created under --dir, default /tmp/ofdmchan):
  s1.sock, s2.sock  full-duplex audio streams (raw int16 LE both ways),
                    one per station -- connect one app instance to each
  ctl.sock          channel configuration: newline-delimited JSON, e.g.
                    {"snr_db": 0, "fading_hz": 0.2, "delay_ms": 15}
                    {"get": true} returns the current config

Channel model per direction (independent fading realizations):
  propagation delay (delay_ms) -> Rayleigh fading (Doppler-band-limited
  complex gain, fading_hz; 0 disables) -> AWGN at snr_db relative to the
  running RMS of active signal -> half-duplex mute (a transmitting station
  hears only its own noise floor).

Pacing: TICK samples every TICK/fs/time_scale wall seconds. time_scale > 1
accelerates the world uniformly; apps that clock the protocol from received
samples (as ofdm_console does) are unaffected by the scale.
"""

import argparse
import asyncio
import json
import time
from pathlib import Path

import numpy as np

FS = 12000
TICK = 120  # 10 ms of audio per tick


class Fader:
    """Streamed Rayleigh gain: one-pole-filtered complex Gaussian,
    normalized to unit mean power."""

    def __init__(self, rng):
        self.rng = rng
        self.state = complex(1.0, 0.0)

    def gains(self, n, doppler_hz):
        if doppler_hz <= 0.0:
            return np.ones(n)
        # one-pole IIR at the Doppler corner, per-sample
        a = float(np.exp(-2.0 * np.pi * doppler_hz / FS))
        g = np.empty(n, dtype=np.complex128)
        s = self.state
        # generate at tick rate then hold (Doppler << tick rate)
        w = (self.rng.standard_normal() + 1j * self.rng.standard_normal())
        s = a ** n * s + np.sqrt(max(1.0 - a ** (2 * n), 1e-12)) * w
        self.state = s
        g[:] = s
        # stationary |s|^2 averages 2 (unit variance per component)
        return np.abs(g) / np.sqrt(2.0)


class Station:
    def __init__(self, name):
        self.name = name
        self.writer = None
        self.txq = np.zeros(0, dtype=np.int16)  # queued TX audio
        self.delay = np.zeros(0, dtype=np.float64)  # propagation line
        self.sig_rms = 3000.0  # EWMA of active-signal RMS (noise sizing)


class Driver:
    def __init__(self, ddir, time_scale, audio=False, volume=0.3):
        self.dir = Path(ddir)
        self.scale = time_scale
        self.cfg = {"snr_db": 20.0, "fading_hz": 0.0, "delay_ms": 5.0,
                    "time_scale": time_scale}
        self.rng = np.random.default_rng(1234)
        self.st = [Station("s1"), Station("s2")]
        self.faders = [Fader(np.random.default_rng(77)),
                       Fader(np.random.default_rng(99))]
        self.volume = volume
        self.audio = None
        self.audio_up = 1
        if audio:
            self._open_audio()

    def _open_audio(self):
        """Monitor speaker: left = station 1's receiver, right = station
        2's. The blocking audio writes become the pacing clock (the sound
        card is the most honest real-time source there is)."""
        try:
            import sounddevice as sd
        except ImportError:
            raise SystemExit("--audio needs the sounddevice module: "
                             "pip install sounddevice")
        for rate, up in ((FS, 1), (4 * FS, 4)):
            try:
                s = sd.OutputStream(samplerate=rate, channels=2,
                                    dtype="int16", blocksize=TICK * up)
                s.start()
                self.audio = s
                self.audio_up = up
                print(f"[driver] audio monitor on at {rate} Hz "
                      f"(L = s1 RX, R = s2 RX, volume {self.volume})")
                return
            except Exception as exc:
                err = exc
        raise SystemExit(f"--audio: no usable output device ({err})")

    def _play(self, deliver):
        v = self.volume
        st = np.stack([deliver[0].astype(np.float64) * v,
                       deliver[1].astype(np.float64) * v], axis=1)
        if self.audio_up > 1:  # linear interpolation to 4x rate
            n = st.shape[0]
            xi = np.arange(n * self.audio_up) / self.audio_up
            st = np.stack([np.interp(xi, np.arange(n), st[:, 0]),
                           np.interp(xi, np.arange(n), st[:, 1])], axis=1)
        self.audio.write(np.clip(st, -32768, 32767).astype(np.int16))

    # --- per-tick channel ---------------------------------------------------

    def _channel(self, tx, fader):
        """One direction, one tick: fading + AWGN sized from signal RMS."""
        out = tx.astype(np.float64)
        out *= fader.gains(len(out), float(self.cfg["fading_hz"]))
        return out

    def tick(self):
        cfg_delay = int(float(self.cfg["delay_ms"]) * FS / 1000.0)
        snr = float(self.cfg["snr_db"])
        outs = []
        txing = []
        for i, s in enumerate(self.st):
            if len(s.txq) >= 1:
                take = min(TICK, len(s.txq))
                tx = np.zeros(TICK, dtype=np.int16)
                tx[:take] = s.txq[:take]
                s.txq = s.txq[take:]
                txing.append(take > 0)
            else:
                tx = np.zeros(TICK, dtype=np.int16)
                txing.append(False)
            # propagation delay line: keep a cfg_delay backlog, pop TICK
            # from the front once primed (constant delay thereafter)
            s.delay = np.concatenate([s.delay, self._channel(tx, self.faders[i])])
            outs.append(None)
            if len(s.delay) >= cfg_delay + TICK:
                outs[i] = s.delay[:TICK]
                s.delay = s.delay[TICK:]
                if len(s.delay) > cfg_delay + 4 * TICK:  # delay was reduced
                    s.delay = s.delay[len(s.delay) - cfg_delay - TICK:]
            else:
                outs[i] = np.zeros(TICK)
            # track active-signal RMS for noise sizing
            p = float(np.mean(outs[i] ** 2))
            if p > 1e4:
                s.sig_rms = 0.98 * s.sig_rms + 0.02 * float(np.sqrt(p))

        deliver = []
        for i, s in enumerate(self.st):
            peer = self.st[1 - i]
            sig = np.zeros(TICK) if txing[i] else outs[1 - i]  # half-duplex
            noise_rms = peer.sig_rms * (10.0 ** (-snr / 20.0))
            noise_rms = max(noise_rms, 20.0)
            rx = sig + noise_rms * self.rng.standard_normal(TICK)
            deliver.append(np.clip(rx, -32768, 32767).astype(np.int16))
        return deliver

    # --- socket plumbing ----------------------------------------------------

    async def station_conn(self, idx, reader, writer):
        s = self.st[idx]
        s.writer = writer
        print(f"[driver] {s.name}: app connected")
        try:
            while True:
                data = await reader.read(65536)
                if not data:
                    break
                chunk = np.frombuffer(data[:len(data) & ~1], dtype="<i2")
                s.txq = np.concatenate([s.txq, chunk])
        except (ConnectionResetError, BrokenPipeError):
            pass
        s.writer = None
        s.txq = np.zeros(0, dtype=np.int16)
        print(f"[driver] {s.name}: app disconnected")

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
                    for k in ("snr_db", "fading_hz", "delay_ms"):
                        if k in req:
                            self.cfg[k] = float(req[k])
                    print(f"[driver] config: {self.cfg}")
                writer.write((json.dumps(self.cfg) + "\n").encode())
                await writer.drain()
        except (ConnectionResetError, BrokenPipeError):
            pass

    async def pump(self):
        tick_s = TICK / FS / self.scale
        next_t = time.monotonic()
        loop = asyncio.get_running_loop()
        while True:
            deliver = self.tick()
            for i, s in enumerate(self.st):
                if s.writer is not None:
                    try:
                        s.writer.write(deliver[i].astype("<i2").tobytes())
                    except (ConnectionResetError, BrokenPipeError):
                        s.writer = None
            if self.audio is not None:
                # the blocking write paces the world at the card's clock
                await loop.run_in_executor(None, self._play, deliver)
                continue
            next_t += tick_s
            dt = next_t - time.monotonic()
            if dt > 0:
                await asyncio.sleep(dt)
            else:
                next_t = time.monotonic()  # fell behind; don't burst

    async def run(self):
        self.dir.mkdir(parents=True, exist_ok=True)
        self.servers = []  # keep references: unreferenced asyncio Servers
        for i in (0, 1):   # get garbage-collected and stop listening
            path = self.dir / f"s{i + 1}.sock"
            path.unlink(missing_ok=True)
            self.servers.append(await asyncio.start_unix_server(
                lambda r, w, i=i: self.station_conn(i, r, w), path=str(path)))
        ctl = self.dir / "ctl.sock"
        ctl.unlink(missing_ok=True)
        self.servers.append(
            await asyncio.start_unix_server(self.ctl_conn, path=str(ctl)))
        print(f"[driver] devices ready in {self.dir} "
              f"(s1.sock, s2.sock, ctl.sock), time_scale={self.scale}")
        await self.pump()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="/tmp/ofdmchan")
    ap.add_argument("--time-scale", type=float, default=1.0,
                    help="run the world N times faster than real time")
    ap.add_argument("--audio", action="store_true",
                    help="play the receivers on the speakers "
                         "(L = station 1 RX, R = station 2 RX); forces "
                         "real time -- the sound card paces the channel")
    ap.add_argument("--volume", type=float, default=0.3)
    args = ap.parse_args()
    if args.audio and args.time_scale != 1.0:
        print("[driver] --audio forces real time; ignoring --time-scale")
        args.time_scale = 1.0
    asyncio.run(Driver(args.dir, args.time_scale, audio=args.audio,
                       volume=args.volume).run())


if __name__ == "__main__":
    main()
