#!/usr/bin/env python3
"""Push-to-talk voice over the OFDM radio, driven from a browser.

    LSCODEC_HOME=/mnt/data/lscodec/adapter \
        /mnt/data/lscodec/adapter/venv/bin/python host/webvoice/server.py
    then open http://localhost:8080

Pick a transmit board and a receive board, warm the link, hold TRANSMIT
and speak. The far station's audio is written to a .wav file.

Transport is BROADCAST (ptype 15, non-ARQ): nothing is acknowledged and
nothing is retransmitted. For speech a late repeat is worse than a gap,
and measured on this stand ARQ costs 2.0-5.2 s per message against
0.9 s for a broadcast.

The link is warmed before every transfer on purpose. The rate ladder
decays after RX_STALE_S of silence and an idle station falls back to
listening on EXTREME only, so a cold broadcast would go out at a rung
the peer may not be receiving on -- the failure looks like a dead radio
and is a negotiation problem.

Enrolment is OUT OF BAND here: the receiver's speaker prompt is derived
from the first 2 s of the talker's own audio, held server-side. On a
real link it would be a one-off ~4 kB transfer, cached per correspondent.
"""
import http.server, io, json, os, socketserver, sys, threading, time, wave
import urllib.parse
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
# The OFDM host tools ship beside this file; the CODEC does not. LSCodec
# is a large third-party tree with its own checkpoints and its own venv,
# so its location is a knob rather than a path baked in here -- point
# LSCODEC_HOME at the directory holding LSCodec-Inference/ and ckpt/.
sys.path.insert(0, os.path.dirname(HERE))
ROOT = os.environ.get("LSCODEC_HOME", "/mnt/data/lscodec/adapter")
sys.path.insert(0, os.path.join(ROOT, "LSCodec-Inference"))

import scipy.signal as _ss
if not hasattr(_ss, "kaiser"):
    from scipy.signal.windows import kaiser as _k
    _ss.kaiser = _k
import torch, torch.nn as nn, yaml, soundfile as sf
_o = torch.load
torch.load = lambda *a, **k: _o(*a, **{**k, "weights_only": False})
from ofdm_modem import OfdmModem, encode
from lscodec.utils import load_model, load_vocoder
from lscodec.ssl_models.wavlm_extractor import Extractor

SR, H = 16000, 640
CHUNK, LEFT, RIGHT = 1.0, 1.28, 0.32      # 1.32 s codec latency
PER_MSG = 1.0                              # speech per broadcast
# Streaming DECODE. The far end used to vocode nothing until the stream
# ended, so the listener heard the first word only after the last one was
# spoken -- on a 200 s transmission, ~202 s of latency for word one, which
# throws away the 1.32 s the codec was tuned to. Decode as the tokens land
# instead: DEC_CH tokens per step, with DEC_LEFT tokens of already-decoded
# history as context. The left context is FREE in latency terms (those
# tokens are already in hand); lookahead would not be, and is not used.
DEC_CH, DEC_LEFT = 25, 50                  # 1.0 s step, 2.0 s of history
BC_RT = 0x20        # firmware: key what is queued, do not wait for
                    # a full 96-byte group (that wait WAS the latency)
OUTDIR = os.path.expanduser("~/voice_rx")
os.makedirs(OUTDIR, exist_ok=True)

print("loading codec (about 20 s) ...", flush=True)
PD = os.path.join(ROOT, "ckpt", "lscodec_25hz")
_ec = yaml.load(open(f"{PD}/encoder_config.yml"), Loader=yaml.Loader)
_ec["pretrain_codebook"] = f"{PD}/codebook.npy"
_vc = yaml.load(open(f"{PD}/vocoder_config.yml"), Loader=yaml.Loader)
_vc["vq_codebook"] = f"{PD}/codebook.npy"
ENC = load_model(_ec, f"{PD}/lscodec_encoder.pt").eval()
VOC = load_vocoder(_vc, f"{PD}/lscodec_vocoder.pt").eval()
WLM = Extractor(checkpoint=os.environ.get(
                    "WAVLM_CKPT",
                    os.path.expanduser("~/Downloads/WavLM-Large.pt")),
                device="cpu")
_cb = torch.tensor(np.load(_vc["vq_codebook"], allow_pickle=True))
if _cb.ndim == 2:
    _cb = _cb.unsqueeze(0)
NG = _cb.shape[0]
CBM = nn.ModuleList([nn.Embedding.from_pretrained(_cb[i], freeze=True)
                     for i in range(NG)])
OUT_SR = _vc["sampling_rate"]

# Warm the graphs. The models are loaded above, but torch pays lazy
# allocation and kernel selection on the FIRST call: measured 826 ms for
# the first encode against 47 ms for every one after. That lands squarely
# on the first transmission, where it is most visible.
_t = time.time()
with torch.no_grad():
    _w = np.zeros(int(1.6 * SR), dtype=np.float32)
    _, _, _i = ENC.encode(torch.from_numpy(_w).view(1, 1, -1))
    _p = WLM.extract(_w[:2 * SR] if len(_w) >= 2 * SR else _w).float().unsqueeze(0)
    _e = _i.repeat_interleave(2, dim=0) if _vc.get("repeat_input_tokens") else _i
    _v = torch.cat([CBM[g](_e[:, g]) for g in range(NG)], dim=-1).unsqueeze(0)
    VOC.inference(_v, _p)
print("codec ready (warmed in %.1f s)" % (time.time() - _t), flush=True)


DIM = 1024


def parse_prompt(raw, fmt="auto"):
    """float16 LE (T x 1024), or int8 with per-channel lo/scale in front.

    Auto-detection is by plausibility, not by size alone: WavLM layer-6
    features sit near mean 0.16 / std 3.6, and a misread format lands
    nowhere near that."""
    sz = len(raw); cands = []
    if fmt in ("auto", "f16") and sz and sz % (DIM * 2) == 0:
        cands.append(("f16", np.frombuffer(raw, dtype="<f2")
                      .astype(np.float32).reshape(-1, DIM)))
    if fmt in ("auto", "int8") and sz > DIM * 8 and (sz - DIM * 8) % DIM == 0:
        lo = np.frombuffer(raw, dtype="<f4", count=DIM).reshape(1, DIM)
        sc = np.frombuffer(raw, dtype="<f4", count=DIM, offset=DIM * 4).reshape(1, DIM)
        q = np.frombuffer(raw, dtype=np.uint8, offset=DIM * 8).reshape(-1, DIM)
        cands.append(("int8", q.astype(np.float32) * sc + lo))
    if not cands:
        raise ValueError("%d bytes is not a whole number of %d-dim frames "
                         "in either format" % (sz, DIM))
    cands.sort(key=lambda c: abs(float(c[1].std()) - 3.6)
               + abs(float(c[1].mean()) - 0.16))
    return cands[0]


def pack10(idx):
    b = "".join(format(int(v), "010b") for v in idx.reshape(-1).tolist())
    b += "0" * ((8 - len(b) % 8) % 8)
    return bytes(int(b[i:i + 8], 2) for i in range(0, len(b), 8))


def unpack10(raw, n):
    b = "".join(format(v, "08b") for v in raw)
    return [int(b[i * 10:(i + 1) * 10], 2) for i in range(n)]


class Link:
    def __init__(self):
        self.lock = threading.Lock()
        self.tx = self.rx = None
        self.tx_serial = self.rx_serial = None
        self.rung = None
        self.tx_status = self.rx_status = None
        self.rx_times = []
        self.tx_stream = bytearray()
        self.airlock = threading.RLock()
        self.feed_races = 0
        self.log = []
        self.prompt_up = None       # uploaded prompt, survives reset_session
        self.prompt_up_name = None
        self.reset_session()
        self.stop = threading.Event()
        self.thread = None
        self.tx_thread = None

    def say(self, msg):
        self.log.append("%s  %s" % (time.strftime("%H:%M:%S"), msg))
        del self.log[:-400]
        print(msg, flush=True)

    def reset_session(self):
        self.buf = np.zeros(0, dtype=np.float32)
        self._nat = np.zeros(0, dtype=np.float32)
        self._tail_guard = 0
        self._consumed = 0        # native samples already turned into buf
        self._dec_pos = 0         # tokens already vocoded and emitted
        self._pcm_out = []        # progressive decoded audio
        self.t_first_out = None   # when the listener could first hear audio
        self._up = self._down = 1
        self._pad_n = self._pad_o = 0
        # Transmit PRIORITY over the two USB drain loops. self.lock is
        # contended by _rx_loop and _tx_loop, which each acquire it and
        # block up to 50 ms in events(), then re-acquire immediately.
        # Python locks are not fair, so a sender competing with two such
        # spinners starves: measured 73.7 s of a 79.6 s feed() budget
        # spent inside _send_chunk for 60 s of speech -- 92.5% of the
        # host's time, and the reason the radio sat idle a third of the
        # wall clock while end-to-end lag grew. The drain loops now stand
        # aside whenever a send is waiting; they lose nothing by it,
        # because the device buffers events and the next poll collects
        # them. */
        self._send_want = 0
        self.k = 0
        self.pending = []
        self.prompt = None
        self.rx_bytes = bytearray()
        self.rx_times = []
        self.tx_stream = bytearray()
        self._dec_pos = 0
        self._pcm_out = []
        self.t_first_out = None
        self.tx_bytes = 0
        self.groups = 0
        self.eos = False
        self.eos_ok = self.eos_lost = 0
        self.t0 = None
        self.t_first_audio = None
        self.t_first_enc = None
        self.t_first_tx = None
        self.t_first_rx = None
        self.enc_ms = 0
        self.last_file = None
        self.src_file = None
        self.talking = False
        self.draining = False
        self.realtime = True
        self.per_msg = PER_MSG
        self.bc_open = False
        self._tokcarry = None
        self.tail_marker = 0
        self._bcfree = 8192

    # ---- boards ------------------------------------------------------
    def open(self, tx_serial, rx_serial):
        self.close()
        self.tx = OfdmModem(serial=tx_serial)
        self.rx = OfdmModem(serial=rx_serial)
        self.tx_serial, self.rx_serial = tx_serial, rx_serial
        self.stop.clear()
        self.thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.thread.start()
        self.dec_thread = threading.Thread(target=self._dec_loop, daemon=True)
        self.dec_thread.start()
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.tx_thread.start()
        self.say("opened TX %s / RX %s" % (tx_serial[:6], rx_serial[:6]))

    def close(self):
        self.stop.set()
        for t in (self.thread, self.tx_thread):
            if t:
                t.join(timeout=2)
        for m in (self.tx, self.rx):
            try:
                m and m.close()
            except Exception:
                pass
        self.tx = self.rx = self.thread = self.tx_thread = None

    def _rx_loop(self):
        while not self.stop.is_set():
            try:
                with self._drain_lock():
                    evs = list(self.rx.events(timeout=0.05))
            except Exception:
                time.sleep(0.2); continue
            for name, p in evs:
                if name == "log":
                    self.say("board(rx): %s" % str(p).strip())
                elif name == "status" and isinstance(p, dict):
                    self.rx_status = p
                elif name == "0x88" and p:
                    if p[0] & 0x80:
                        self.groups += 1
                    elif p[0] & 0x40:
                        # END OF STREAM, carrying the radio's own verdict:
                        # frames_ok, frames_lost. Authoritative -- byte
                        # counts say what arrived, not what the air cost.
                        if len(p) >= 5:
                            self.eos_ok = int.from_bytes(p[1:3], "little")
                            self.eos_lost = int.from_bytes(p[3:5], "little")
                        self.eos = True
                    else:
                        if self.t_first_rx is None and len(p) > 1:
                            self.t_first_rx = time.time()
                        self.rx_bytes += p[1:]
                        self.rx_times.append((round(time.time(), 3), len(p) - 1))

    # ---- link warm-up ------------------------------------------------
    def warmup(self, budget=75.0, settle=True):
        # Warm-up is ARQ: it makes the FAR station transmit an
        # acknowledgement. The link is half duplex, so while the far end
        # keys that ack it is deaf, and any broadcast group passing at
        # that moment is missed whole -- the radio then reports 0 frames
        # lost, because the group never reached its receiver to be
        # counted. Measured: a warm-up clicked during the drain cost
        # 11.8 % of the bytes with a clean channel.
        if self.talking or self.draining:
            self.say("warm-up refused: a transmission is still on the air")
            return {"ok": False, "busy": True, "rung": self.rung,
                    "reason": "a transmission is still on the air -- "
                              "warming now would deafen the far station"}
        """An idle station decays to EXTREME-only listening and its
        remembered rung decays with it. Exchange real frames until the
        peer answers, so the broadcast goes out at a rung it is on."""
        seen = []
        deadline = time.time() + budget
        for n in range(8):
            tag = bytes([0xE0 + n])
            with self.lock:
                self.tx.submit(tag + b"warm", qos=1)
            t = time.time()
            while time.time() < min(deadline, t + 20):
                with self.lock:
                    evs = list(self.rx.events(timeout=0.1))
                    for _n, _p in self.tx.events(timeout=0.02):
                        if _n == "status" and isinstance(_p, dict):
                            self.rung = _p.get("rung_now", self.rung)
                for name, p in evs:
                    if name == "message" and p["data"][:1] == tag:
                        # the rung comes from a STATUS event, which may not
                        # have arrived yet -- ask for one rather than
                        # reporting a stale 0 that reads as a dead ladder
                        el = time.time() - t
                        rdl = time.time() + 4.0
                        while time.time() < rdl:
                            with self.lock:
                                for _n, _p in self.tx.events(timeout=0.2):
                                    if _n == "status" and isinstance(_p, dict):
                                        r = _p.get("rung_now", _p.get("rung"))
                                        if r is not None and r >= 0:
                                            self.rung = r
                            if self.rung:
                                break
                        if settle:
                            self.settle()
                        self.say("link warm in %.1f s, rung %s" % (el, self.rung))
                        return {"ok": True, "rung": self.rung,
                                "seconds": round(el, 2)}
                    if name == "status" and isinstance(p, dict):
                        self.rung = p.get("rung_now", self.rung)
            if time.time() > deadline:
                break
        self.say("link did NOT warm up")
        return {"ok": False, "rung": self.rung}

    def settle(self, budget=20.0, need=2):
        """Wait for the ARQ warm-up exchange to actually FINISH.

        warmup() returns the moment the far station DECODES the message --
        but the acknowledgement it owes is still to be keyed, and the link
        is half duplex, so a broadcast started now goes out while the far
        station is transmitting and therefore deaf. It misses the first
        group whole, and because a stream always starts at seq 0 the
        receiver's head-loss arithmetic reports it exactly: "2 lost".

        MEASURED, 16 transmissions on a clean cross-wire: 4 lost the first
        group and nothing else. Every failure was the same size -- the
        recording came back 0.96 s short, which is the first chunk (24
        tokens at 25 Hz) to the sample -- and the receiving board's own
        cap_overruns and failed-block counters stayed at zero throughout,
        i.e. it never heard the group rather than failing to decode it.

        The signal is the TRANSMIT board's own status: pending goes clear
        when the ack has ARRIVED, which is to say the far station has
        stopped keying. The queues have to be empty too, or a warm-up
        frame still waiting its turn goes out mid-broadcast and costs a
        group the same way."""
        end = time.time() + budget
        clean = 0
        while time.time() < end:
            with self.lock:
                for n, p in self.tx.events(timeout=0.2):
                    if n == "status" and isinstance(p, dict):
                        self.tx_status = p
                    elif n == "log":
                        self.say("board(tx): %s" % str(p).strip())
            s = self.tx_status
            if s and not s.get("pending") and not any(s.get("queues") or [1]):
                clean += 1
                if clean >= need:   # status arrives every 0.5 s
                    return True
            else:
                clean = 0
        self.say("link did not go quiet -- the first group may be lost")
        return False

    # ---- transmit ----------------------------------------------------
    def feed(self, pcm, in_sr):
        """Accept a block of microphone audio and broadcast what is ready.

        SERIALIZED. The HTTP server is threaded and the browser posts
        fire-and-forget, so without this every audio block runs its own
        thread through the encoder and the broadcast sender at once. What
        that costs is not subtle: `self.k`, `self.pending` and
        `self._tokcarry` are read-modify-written (duplicated or dropped
        tokens, and the 4-bit alignment bug back again), and in
        _send_chunk `self.bc_open` is read before the USB write and
        assigned after it -- so two threads both see it clear and both
        send a NON-continuation command. bc_cmd() treats the second as a
        new broadcast: it closes the one on the air, overwrites the
        unsent bytes and resets g_bc_seq to 0. Both threads can also
        clear the bc_free pacing wait together and overrun the board's
        source buffer, which drops the whole queued broadcast.

        Resampling is done on the WHOLE accumulated native-rate buffer, not
        per block. resample_poly is a fixed FIR, so its output prefix is
        stable as the buffer grows; a naive per-block carry scrambles the
        phase at every seam and produces garbled audio."""
        if not self.airlock.acquire(blocking=False):
            self.feed_races += 1
            self.airlock.acquire()
        try:
            self._feed(pcm, in_sr)
        finally:
            self.airlock.release()

    def _feed(self, pcm, in_sr):
        if self.t0 is None:
            self.t0 = time.time()
            self.t_first_audio = self.t0
        if in_sr != SR:
            # INCREMENTAL, by overlap-save. Resampling the whole accumulated
            # buffer on every post is O(n) per post and therefore O(n^2) in
            # the length of the transmission: measured 11.9 ms at a 10 s
            # buffer, 231.7 ms at 200 s, against posts arriving every
            # 0.1-0.26 s. Past ~100 s the encoder drops below real time and
            # STARVES the radio -- on a 200 s stream the board idled 31% of
            # the wall clock, keyed only 0.84x real-time worth of air, and
            # end-to-end lag grew +0.207 s per second of speech to +41 s.
            # (Serializing feed() did not cause this, but it did stop
            # threads from hiding part of it.)
            #
            # Only the NEW tail is resampled, with PAD_N native samples of
            # real history so the FIR sees no implicit zero-padding, and
            # the last PAD_O output samples held back because the segment
            # END carries the same transient. Verified bit-identical to the
            # whole-buffer result (max abs diff 0.0 over 112000 samples),
            # which is the property the chunker relies on: the prefix of
            # self.buf must never change once emitted.
            if self._down == 1 and self._up == 1:
                # first post of the session fixes the ratio and the guards.
                # resample_poly's default FIR is 2*10*max(up,down)+1 taps at
                # the UP rate, so 4x its half-length is a generous guard.
                from math import gcd
                g = gcd(SR, in_sr)
                self._up, self._down = SR // g, in_sr // g
                self._pad_o = 10 * max(self._up, self._down) * 4
                self._pad_n = self._pad_o * self._down // self._up
            self._nat = np.concatenate([self._nat, pcm])
            end = (len(self._nat) // self._down) * self._down
            if end > self._consumed:
                lo = max(0, self._consumed - self._pad_n)
                y = _ss.resample_poly(self._nat[lo:end], SR,
                                      in_sr).astype(np.float32)
                drop = (self._consumed - lo) // self._down * self._up
                keep = max(0, len(y) - drop - self._pad_o)
                if keep > 0:
                    self.buf = np.concatenate([self.buf, y[drop:drop + keep]])
                    self._consumed += keep * self._down // self._up
            # drop native history that can no longer affect any output
            if self._consumed > 4 * self._pad_n:
                cut = self._consumed - 2 * self._pad_n
                self._nat = self._nat[cut:]
                self._consumed -= cut
            self._tail_guard = int(0.05 * SR)   # keep chunks out of the edge
        else:
            self.buf = np.concatenate([self.buf, pcm])
            self._tail_guard = 0
        CH, L, R = int(CHUNK * SR), int(LEFT * SR), int(RIGHT * SR)
        while (self.k + 1) * CH + R + self._tail_guard <= len(self.buf):
            lo = max(0, self.k * CH - L)
            hi = (self.k + 1) * CH + R
            seg = self.buf[lo:hi]
            _t_enc = time.time()
            with torch.no_grad():
                _, _, t = ENC.encode(torch.from_numpy(seg).view(1, 1, -1))
            if self.t_first_enc is None:
                self.t_first_enc = time.time()
                self.enc_ms = (self.t_first_enc - _t_enc) * 1000
            skip = (self.k * CH - lo) // H
            self.pending.append(t[skip:skip + min(CH // H, t.shape[0] - skip)])
            self.k += 1
            if self.prompt is None and len(self.buf) >= 2 * SR:
                self.prompt = WLM.extract(self.buf[:2 * SR]).numpy()
            if len(self.pending) >= max(1, int(round(self.per_msg / CHUNK))):
                idx = torch.cat(self.pending, 0); self.pending = []
                # pace against the board, as bcastfile does; a chunk that
                # does not fit is dropped by the firmware, not queued
                need = (idx.shape[0] * 10 + 7) // 8
                t_wait = time.time()
                while self.bc_free() < 2 * need and time.time() - t_wait < 8:
                    time.sleep(0.05)
                self._send(idx)

    def _send(self, idx):
        """Append to ONE broadcast stream, emitting whole BYTES only.

        Two separate things are being got right here.

        The stream: bc_cmd() treats a non-continuation command as
        superseding whatever is still unsent, so a fresh broadcast per
        group overwrites the tail of the one before it. Continuation
        chunks (bit 6) append instead, as app.c's bcastfile does.

        The alignment: pack10 zero-pads to a byte boundary, so 50 tokens
        is 500 bits in 63 bytes -- 4 pad bits. The receiver has no framing
        and unpacks the concatenated bytes as one stream, so those pad
        bits shift every later group by 4 bits, then 8, then 12. The first
        group decodes perfectly and everything after it is noise. 4 tokens
        = 40 bits = 5 bytes exactly, so only multiples of 4 go out and the
        remainder waits for the next group.
        """
        if self._tokcarry is not None and self._tokcarry.shape[0]:
            idx = torch.cat([self._tokcarry, idx], 0)
        keep = (idx.shape[0] // 4) * 4
        self._tokcarry = idx[keep:]
        if keep == 0:
            return
        self._send_chunk(pack10(idx[:keep]), final=False, ntok=keep)

    def _send_chunk(self, raw, final, ntok=None):
        if self.t0 is not None and self.t_first_tx is None:
            self.t_first_tx = time.time()
        """One chunk of the stream. The MORE bit goes clear on the LAST
        chunk of real data -- that is what completes a broadcast.

        app.c does exactly this: `more = off + n < len`, and its only
        1-byte command is the ABORT path for a truncated file. Using that
        as a normal close injects a byte of payload into the stream, which
        the receiver then decodes as token data."""
        MORE, CONT = 0x80, 0x40
        self._send_want += 1
        try:
            self._acquire_tx()
        finally:
            self._send_want -= 1
        try:
            # bc_open is read, the command is written, and bc_open is
            # updated as ONE step. Reading it before the write and
            # assigning after -- the shape this had -- lets a second
            # sender see it stale and start a rival broadcast.
            ptype = (15 | (BC_RT if self.realtime else 0)
                     | (0 if final else MORE)
                     | (CONT if self.bc_open else 0))
            self.tx.t.write(encode(0x06, bytes([ptype, 0xFF]) + raw))
            self.bc_open = not final
            self.tx_stream += raw
        finally:
            self.lock.release()
        self.tx_bytes += len(raw)
        self.say("%s%s %s%d B"
                 % ("stream +" if ptype & CONT else "broadcast start",
                    " (last)" if final else "",
                    ("%d tokens / " % ntok) if ntok else "", len(raw)))

    def _tx_loop(self):
        """Drain the TRANSMIT board continuously.

        Two things depend on this. bc_free has to be fresh or pacing is
        guesswork -- and a chunk the board cannot fit is DROPPED, taking
        the whole queued broadcast with it (bc_cmd: "chunk overran the
        source buffer"). And that drop is only ever reported in an
        EVT_LOG, which was being discarded here, so the loss showed up as
        missing audio with no explanation anywhere."""
        while not self.stop.is_set():
            try:
                with self._drain_lock():
                    evs = list(self.tx.events(timeout=0.05))
            except Exception:
                time.sleep(0.2); continue
            for name, p in evs:
                if name == "status" and isinstance(p, dict):
                    self.tx_status = p
                    self._bcfree = p.get("bc_free", self._bcfree)
                    r = p.get("rung_now", p.get("rung"))
                    if r is not None and r >= 0:
                        self.rung = r
                elif name == "log":
                    self.say("board(tx): %s" % str(p).strip())

    # ---- streaming decode -------------------------------------------
    def _vocode(self, lo, hi, pr):
        """Vocode tokens [lo,hi) with [lo-DEC_LEFT,lo) as context, and
        return only the audio belonging to [lo,hi)."""
        ctx = max(0, lo - DEC_LEFT)
        data = bytes(self.rx_bytes)
        n = (len(data) * 8) // 10
        hi = min(hi, n)
        if hi <= lo:
            return None
        idx = torch.tensor(unpack10(data, hi)[ctx:hi],
                           dtype=torch.long).unsqueeze(1)
        with torch.no_grad():
            i = (idx.repeat_interleave(2, dim=0)
                 if _vc.get("repeat_input_tokens") else idx)
            v = torch.cat([CBM[g](i[:, g]) for g in range(NG)],
                          dim=-1).unsqueeze(0)
            y = VOC.inference(v, torch.from_numpy(pr).float()
                              .unsqueeze(0))[-1].view(-1).numpy()
        skip = int(round((lo - ctx) / 25.0 * OUT_SR))
        return y[skip:]

    def _prompt(self):
        """The speaker prompt, or None if it is not available yet."""
        if self.prompt_up is not None:
            return self.prompt_up
        if self.prompt is not None:
            return self.prompt
        if len(self.buf) >= 2 * SR:
            return WLM.extract(self.buf[:2 * SR]).numpy()
        return None

    def _dec_loop(self):
        """Vocode arriving tokens as they land, a chunk at a time.

        Its own thread on purpose: vocoding one chunk costs far more than
        a USB poll, and doing it inside _rx_loop would stall event
        draining -- the same starvation that _drain_lock exists to
        prevent, in the other direction."""
        while not self.stop.is_set():
            time.sleep(0.05)
            avail = (len(self.rx_bytes) * 8) // 10
            if avail - self._dec_pos < DEC_CH:
                continue
            pr = self._prompt()
            if pr is None:
                continue
            hi = self._dec_pos + ((avail - self._dec_pos) // DEC_CH) * DEC_CH
            try:
                y = self._vocode(self._dec_pos, hi, pr)
            except Exception as e:
                # back off rather than spin: a step that fails every
                # 50 ms would rotate the log clean before anyone reads it
                self.say("decode step failed: %r" % (e,))
                time.sleep(1)
                continue
            if y is None or not len(y):
                continue
            self._pcm_out.append(y)
            self._dec_pos = hi
            if self.t_first_out is None:
                self.t_first_out = time.time()
                if self.t0:
                    self.say("first audio decodable %.1f s after the talker "
                             "started" % (self.t_first_out - self.t0))

    def _acquire_tx(self):
        """Take the USB lock for a transmit, ahead of the drain loops."""
        self.lock.acquire()

    def _drain_lock(self):
        """Drain loops defer to any waiting transmit."""
        while self._send_want:
            time.sleep(0.002)
        return self.lock

    def bc_free(self):
        """Free bytes in the board's 8 kB broadcast source buffer."""
        return self._bcfree

    def finish(self):
        with self.airlock:
            return self._finish()

    def _finish(self):
        self.talking = False
        # flush: the tail of the speech never gets its lookahead, so pad
        # with silence and encode the remaining chunks
        self._tail_guard = 0
        CH, R = int(CHUNK * SR), int(RIGHT * SR)
        if len(self.buf) > self.k * CH + int(0.3 * SR):
            pad = (self.k + 1 + (len(self.buf) - self.k * CH) // CH) * CH + R
            self.buf = np.concatenate(
                [self.buf, np.zeros(max(0, pad - len(self.buf)), np.float32)])
            self.feed(np.zeros(0, np.float32), SR)
        tail = list(self.pending); self.pending = []
        if self._tokcarry is not None and self._tokcarry.shape[0]:
            tail.insert(0, self._tokcarry)
        self._tokcarry = None
        if tail:
            idx = torch.cat(tail, 0)
            # Nothing follows the last group, but the receiver derives its
            # token count from the byte count, so pad to a multiple of 4
            # by repeating the final token rather than leaving pad bits to
            # be read as a spurious one.
            r = idx.shape[0] % 4
            if r:
                idx = torch.cat([idx] + [idx[-1:]] * (4 - r), 0)
            self._send_chunk(pack10(idx), final=True, ntok=idx.shape[0])
        elif self.bc_open:
            # Nothing left to say but the stream is open. There is no
            # zero-length command (bc_cmd rejects len <= 0), so the only
            # way to close is app.c's abort marker -- one byte that DOES
            # reach the receiver. Account for it so the loss figure stays
            # honest, and drop it again before decoding.
            self._send_chunk(b"\x00", final=True)
            self.tail_marker = 1
        if self.tx_bytes == 0:
            self.say("nothing to send -- transmission shorter than one chunk")
            return {"ok": False,
                    "reason": "no speech captured (keep transmitting for at "
                              "least ~2 s)"}
        return self.wait_and_write()

    def wait_and_write(self, quiet=25.0, budget=180.0):
        self.draining = True
        try:
            return self._wait_and_write(quiet, budget)
        finally:
            self.draining = False

    def _wait_and_write(self, quiet, budget):
        """Wait for the far station's END-OF-STREAM, not for silence.

        The board keys only FULL groups while chunks are still arriving, so
        the end of a transmission leaves a backlog: several groups, each
        seconds of air with gaps between them. A quiet-period timeout fires
        in one of those gaps and truncates the recording -- which is what
        cost a 19 s transmission its last two seconds."""
        last = len(self.rx_bytes); t = time.time(); end = time.time() + budget
        while time.time() < end:
            time.sleep(0.5)
            if self.eos:
                self.say("far station signalled end of stream")
                time.sleep(0.5)                  # let a trailing chunk land
                break
            if len(self.rx_bytes) != last:
                last = len(self.rx_bytes); t = time.time()
            elif time.time() - t > quiet and last > 0:
                self.say("no end-of-stream after %.0f s quiet -- decoding "
                         "what arrived" % quiet)
                break
        if not self.rx_bytes:
            self.say("nothing received")
            return {"ok": False, "reason": "no audio decoded at the far station"}
        data = bytes(self.rx_bytes)
        if self.tail_marker and len(data) >= self.tail_marker:
            data = data[:-self.tail_marker]     # the abort marker is not audio
        n = (len(data) * 8) // 10
        pr = self._prompt()
        if pr is None:
            pr = WLM.extract(self.buf[:2 * SR]).numpy()
        # Everything up to _dec_pos was already vocoded AS IT ARRIVED and
        # the listener has had it for a while; only the tail is left. The
        # abort marker can put the true end slightly behind the streaming
        # decoder, so clamp rather than assume.
        head = np.concatenate(self._pcm_out) if self._pcm_out \
            else np.zeros(0, np.float32)
        done = min(self._dec_pos, n)
        head = head[:int(round(done / 25.0 * OUT_SR))]
        tail = self._vocode(done, n, pr) if n > done else None
        y = np.concatenate([head, tail]) if tail is not None and len(tail) \
            else head
        if not len(y):        # nothing streamed (very short transmission)
            idx = torch.tensor(unpack10(data, n),
                               dtype=torch.long).unsqueeze(1)
            with torch.no_grad():
                i = (idx.repeat_interleave(2, dim=0)
                     if _vc.get("repeat_input_tokens") else idx)
                v = torch.cat([CBM[g](i[:, g]) for g in range(NG)],
                              dim=-1).unsqueeze(0)
                y = VOC.inference(v, torch.from_numpy(pr).float()
                                  .unsqueeze(0))[-1].view(-1).numpy()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(OUTDIR, "rx_%s.wav" % stamp)
        sf.write(path, y, OUT_SR, "PCM_16")
        self.last_file = path
        # what actually went INTO the encoder, after resampling to 16 kHz --
        # the honest reference for judging the far end against
        if len(self.buf):
            spath = os.path.join(OUTDIR, "tx_%s.wav" % stamp)
            sf.write(spath, self.buf, SR, "PCM_16")
            self.src_file = spath
        # the two byte streams, so a lost group can be located EXACTLY:
        # what we handed the board against what the far end delivered
        try:
            open(os.path.join(OUTDIR, "tx_%s.bin" % stamp), "wb").write(
                bytes(self.tx_stream))
            open(os.path.join(OUTDIR, "rx_%s.bin" % stamp), "wb").write(
                bytes(self.rx_bytes))
        except Exception:
            pass
        loss = 0.0 if not self.tx_bytes else 100.0 * (1 - len(self.rx_bytes) / self.tx_bytes)
        self.say("wrote %s (%.1f s, %.1f%% of bytes lost, %d frames ok / "
                 "%d lost on the air)"
                 % (os.path.basename(path), len(y) / OUT_SR, loss,
                    self.eos_ok, self.eos_lost))
        def dt(a):
            return round(a - self.t0, 2) if (a and self.t0) else None
        return {"ok": True, "file": path, "source": self.src_file,
                "t_first_enc": dt(self.t_first_enc), "t_first_tx": dt(self.t_first_tx),
                "t_first_rx": dt(self.t_first_rx),
                "t_first_out": dt(self.t_first_out), "enc_ms": round(self.enc_ms),
                "frames_ok": self.eos_ok, "frames_lost": self.eos_lost,
                "seconds": round(len(y) / OUT_SR, 2),
                "tx_bytes": self.tx_bytes, "rx_bytes": len(self.rx_bytes),
                "loss_pct": round(loss, 1), "groups": self.groups}


LINK = Link()


class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"   # Range needs 1.1; every response below
                                    # sets an accurate Content-Length
    def _json(self, obj, code=200):
        b = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers(); self.wfile.write(b)

    def log_message(self, *a):
        pass

    def _send_wav(self, path, name, download):
        """Serve a wav, honouring Range.

        An <audio> element asks for `Range: bytes=0-` and expects 206. Answer
        200 without Accept-Ranges and the browser cannot seek and commonly
        stops a second or two in -- which looks like a corrupt recording and
        is a missing header."""
        b = open(path, "rb").read()
        total = len(b)
        rng = self.headers.get("Range", "")
        start, end = 0, total - 1
        partial = False
        if rng.startswith("bytes="):
            try:
                a, _, z = rng[6:].split(",")[0].strip().partition("-")
                if a:
                    start = int(a)
                    end = int(z) if z else total - 1
                else:                       # suffix form: bytes=-N
                    start = max(0, total - int(z))
                if start >= total:
                    self.send_response(416)
                    self.send_header("Content-Range", "bytes */%d" % total)
                    self.send_header("Content-Length", "0")
                    self.end_headers(); return
                end = min(end, total - 1)
                partial = True
            except ValueError:
                start, end, partial = 0, total - 1, False
        body = b[start:end + 1]
        self.send_response(206 if partial else 200)
        self.send_header("Content-Type", "audio/wav")
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Content-Length", str(len(body)))
        if partial:
            self.send_header("Content-Range",
                             "bytes %d-%d/%d" % (start, end, total))
        self.send_header("Cache-Control", "no-store")
        if download:
            self.send_header("Content-Disposition",
                             'attachment; filename="%s"' % name)
        self.end_headers(); self.wfile.write(body)

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        if u.path in ("/", "/index.html"):
            b = open(os.path.join(HERE, "index.html"), "rb").read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers(); self.wfile.write(b); return
        if u.path == "/api/stations":
            import usb.core
            out = []
            for d in usb.core.find(find_all=True, idVendor=0x1209, idProduct=0x0001):
                try:
                    from ofdm_modem import _read_serial
                    out.append({"serial": _read_serial(d)})
                except Exception:
                    pass
            return self._json({"stations": out})
        if u.path.startswith("/api/recording/"):
            # serve ONLY by basename out of OUTDIR -- no paths, no traversal
            name = os.path.basename(urllib.parse.unquote(u.path.rsplit("/", 1)[1]))
            path = os.path.join(OUTDIR, name)
            if not (name.endswith(".wav") and os.path.isfile(path)):
                return self.send_error(404)
            return self._send_wav(path, name,
                                  "dl" in urllib.parse.parse_qs(u.query))
        if u.path == "/api/status":
            return self._json({
                "tx": LINK.tx_serial, "rx": LINK.rx_serial, "rung": LINK.rung,
                "talking": LINK.talking, "tx_bytes": LINK.tx_bytes,
                "rx_bytes": len(LINK.rx_bytes), "groups": LINK.groups,
                "file": LINK.last_file, "source": LINK.src_file,
                "prompt": LINK.prompt_up_name, "log": LINK.log[-12:],
                "tx_status": LINK.tx_status, "rx_status": LINK.rx_status,
                "full_log": LINK.log, "rx_times": LINK.rx_times,
                "feed_races": LINK.feed_races})
        self.send_error(404)

    def do_POST(self):
        u = urllib.parse.urlparse(self.path)
        n = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(n) if n else b""
        q = urllib.parse.parse_qs(u.query)
        try:
            if u.path == "/api/select":
                d = json.loads(body or b"{}")
                LINK.open(d["tx"], d["rx"]); LINK.reset_session()
                return self._json({"ok": True})
            if u.path == "/api/warmup":
                # ?settle=0 reproduces the pre-fix behaviour, for A/B
                return self._json(LINK.warmup(
                    settle=q.get("settle", ["1"])[0] != "0"))
            if u.path == "/api/prompt":
                if not body:
                    LINK.prompt_up = LINK.prompt_up_name = None
                    LINK.say("prompt cleared -- back to the talker's own voice")
                    return self._json({"ok": True, "cleared": True})
                fmt, P = parse_prompt(body)
                LINK.prompt_up = P
                LINK.prompt_up_name = q.get("name", ["prompt.bin"])[0]
                LINK.say("prompt loaded: %s, %s, %d frames"
                         % (LINK.prompt_up_name, fmt, P.shape[0]))
                return self._json({"ok": True, "format": fmt,
                                   "frames": int(P.shape[0]),
                                   "seconds": round(P.shape[0] / 50.0, 2)})
            if u.path == "/api/start":
                if q.get("settle", ["1"])[0] != "0":
                    LINK.settle(budget=3.0, need=1)
                LINK.reset_session(); LINK.talking = True
                LINK.realtime = q.get("rt", ["1"])[0] != "0"
                LINK.per_msg = float(q.get("per", [str(PER_MSG)])[0])
                LINK.say("transmit pressed")
                return self._json({"ok": True})
            if u.path == "/api/audio":
                sr = int(q.get("sr", ["48000"])[0])
                pcm = np.frombuffer(body, dtype="<f4").astype(np.float32)
                LINK.feed(pcm, sr)
                return self._json({"ok": True, "tx_bytes": LINK.tx_bytes})
            if u.path == "/api/stop":
                LINK.say("transmit released")
                return self._json(LINK.finish())
        except Exception as e:
            import traceback; traceback.print_exc()
            return self._json({"ok": False, "error": str(e)}, 500)
        self.send_error(404)


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    print("http://localhost:%d   (recordings land in %s)" % (port, OUTDIR), flush=True)
    Server(("127.0.0.1", port), Handler).serve_forever()
