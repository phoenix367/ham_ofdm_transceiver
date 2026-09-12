#!/usr/bin/env python3
"""Two-board integration benchmark: the stand, end to end, timed.

    ./venv/bin/python host/board_bench.py                 # everything
    ./venv/bin/python host/board_bench.py --quick         # small sizes
    ./venv/bin/python host/board_bench.py --scenarios file,bcast
    ./venv/bin/python host/board_bench.py --voice-start   # speech too

Drives both boards of the stand (PA4->PA6 cross-wired STM32H743s running
the radio firmware) through the traffic an operator generates, in both
directions and in combination, and records what each cost: seconds,
bytes per second, frames, timeouts, retransmissions, losses, and whether
the bytes came back exact. The stations run ON the boards; this program
is two `board_console.py` consoles driven by a script instead of a
keyboard, plus the webvoice server for speech.

Scenarios (run in this order, `--scenarios` picks a subset):

  attach       both boards answer INFO; initial rung / SNR / peer state
  warmup       a bulk primer each way: time to delivery, time to the
               capability handshake, rung and SNR after it
  message      interactive messages A->B then B->A, latency each
  chat         messages queued on BOTH boards at once (bidirectional)
  bulk         bulk-queue test patterns each way, latency and B/s
  file         a file A->B then B->A: byte-exact, B/s, tx/timeouts/retx
  file_bidir   a file each way AT THE SAME TIME
  file_chat    a file one way while the receiver chats back (both ways)
  bcast        text broadcast each way: frames ok / lost
  bcastfile    a broadcast file each way: bytes stored, frames lost
  bcast_msg    a message queued at the receiver DURING a broadcast: its
               latency (the broadcast holds the peer's transmitter)
  speech       push-to-talk speech each way over the webvoice server:
               warm-up, first audio, bytes lost, frames ok / lost
  speech_file  a file right after the speech (the link is warm from it)
  stale        idle past RX_STALE_S, then one message: recovery latency

Needs: both boards attached (serials auto-detected, or --a/--b), no
other console holding them, and for the speech scenarios the webvoice
server (`--voice-url`, or `--voice-start` to spawn it with
`--voice-python`) plus a speech source (`--voice-src`, default: the
longest tx_*.wav in ~/voice_rx, copied once into the work dir and cut
to `--voice-secs`). Speech releases the boards to the server and
re-attaches afterwards; an external server keeps them, so speech then
runs last.

Writes results/board_bench.json and results/board_bench.md (suffixed
_quick for --quick). Every scenario records ok/failed and its metrics;
a failure does not stop the run.
"""
import argparse
import json
import os
import queue
import random
import subprocess
import sys
import threading
import time
import urllib.request
import wave
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "demoapp"))

from ofdm_modem import OfdmModem, encode, USBTimeoutError, VID, PID  # noqa: E402
import board_console as bc  # noqa: E402

VOICE_PYTHON = "/mnt/data/lscodec/adapter/venv/bin/python"
VOICE_RX_DIR = os.path.expanduser("~/voice_rx")
RX_STALE_S = 45.0          # link.c: idle listening decays to EXTREME-only


def now():
    return time.monotonic()


def ts():
    return time.strftime("%H:%M:%S")


# ---------------------------------------------------------------------------
# a console that records instead of printing

class Station(bc.Console):
    def __init__(self, modem, name, msg_max, verbose):
        super().__init__(modem, name, msg_max, True)
        self.verbose = verbose
        self.log = []          # (t, text)
        self.msgs = []         # (t, qos, bytes)
        self.files = []        # (t, path, total, on_air)
        self.bcasts = []       # (t_start, t_end, ptype, frames_ok, lost, bytes)
        self.diags = []        # (t, dict)
        self._bc = None

    # -- output -> log
    def say(self, text):
        self.log.append((now(), text))
        if self.verbose:
            print(f"{ts()} [{self.name}] {text}", flush=True)

    plain = say

    # -- receive hooks
    def on_message(self, qos, data):
        n_files = self.rx_files
        super().on_message(qos, data)
        if self.rx_files == n_files and not (
                len(data) > 6 and data[0] in bc.FILE_MAGICS and data[1:6] == bc.FILE_TAG):
            self.msgs.append((now(), qos, bytes(data)))

    def on_file_part(self, msg):
        n_files = self.rx_files
        path_before = self.rx.path
        super().on_file_part(msg)
        if self.rx_files > n_files:
            # the parent closed rx after writing; recover what it logged
            text = self.log[-1][1]
            path = text.split("file complete: ")[1].split(" (")[0]
            total = int(text.split("(")[1].split(" bytes")[0])
            on_air = int(text.split(", ")[1].split(" on air")[0])
            self.files.append((now(), path, total, on_air))
        del path_before

    def on_bcast(self, pl):
        if not pl:
            return
        if pl[0] & 0x80:
            self._bc = [now(), pl[0] & 0x0F, bytearray()]
            self.say(f"<< broadcast starting (ptype {pl[0] & 0x0F})")
        elif pl[0] & 0x40:
            fo = int.from_bytes(pl[1:3], "little") if len(pl) >= 5 else 0
            lo = int.from_bytes(pl[3:5], "little") if len(pl) >= 5 else 0
            if self._bc is None:
                self._bc = [now(), 0, bytearray()]
            t0, ptype, buf = self._bc
            self.bcasts.append((t0, now(), ptype, fo, lo, bytes(buf)))
            self._bc = None
            self.say(f"<< broadcast ended: {fo} frame(s), {lo} lost, {len(buf)} bytes")
        else:
            if self._bc is None:
                self._bc = [now(), 0, bytearray()]
            self._bc[2] += pl[1:]

    def on_diag(self, d):
        self.diags.append((now(), dict(d)))
        super().on_diag(d)

    def counters(self):
        st = self.status or {}
        out = {k: st.get(k) for k in ("tx", "rx", "timeouts", "retransmissions",
                                       "rung", "rung_now", "snr_db", "peer_state",
                                       "peer_win_max", "temp_c")}
        if st.get("temp_c") is not None:
            out["temp_c"] = round(st["temp_c"], 1)
        return out


class Pump(threading.Thread):
    """One thread owns one board: heartbeat, events, and the commands the
    scenarios hand it (writes from a second thread would race the reads)."""

    def __init__(self, station):
        super().__init__(daemon=True)
        self.s = station
        self.q = queue.Queue()
        self.stop = threading.Event()
        self.fault = None

    def call(self, fn, wait=True):
        done = threading.Event()
        box = {}

        def job():
            try:
                box["r"] = fn()
            except Exception as e:      # noqa: BLE001
                box["e"] = e
            done.set()
        self.q.put(job)
        if wait:
            done.wait(10.0)
            if "e" in box:
                raise box["e"]
            return box.get("r")

    def run(self):
        m = self.s.m
        last_ping, misses = 0.0, 0
        try:
            while not self.stop.is_set():
                t = now()
                if t - last_ping >= 1.0:
                    try:
                        m.t.write(encode(0x04, int(t).to_bytes(4, "little")))
                        misses = 0
                    except USBTimeoutError:
                        misses += 1
                        if misses >= 5:
                            self.fault = "board stopped accepting writes"
                            return
                    last_ping = t
                for kind, payload in m.events(timeout=0.1, poke=False):
                    if kind == "message":
                        self.s.on_message(payload["qos"], payload["data"])
                    elif kind == "status" and payload:
                        self.s.on_status(payload)
                    elif kind == "log":
                        self.s.on_log(payload)
                    elif kind == "diag" and payload:
                        self.s.on_diag(payload)
                    elif kind == "0x88":
                        self.s.on_bcast(payload)
                while True:
                    try:
                        job = self.q.get_nowait()
                    except queue.Empty:
                        break
                    job()
        except Exception as e:          # noqa: BLE001
            self.fault = repr(e)


# ---------------------------------------------------------------------------
# the stand

class Stand:
    def __init__(self, serial_a, serial_b, verbose):
        self.serials = {"A": serial_a, "B": serial_b}
        self.verbose = verbose
        self.st = {}
        self.pump = {}
        self.archive = {"A": [], "B": []}   # console logs across re-attaches

    def attach(self, tries=8):
        for name in ("A", "B"):
            # a board just released by another program may be inside its
            # worst blocking decode (2.3 s measured) or mid-key: retry
            # INFO rather than declare it dead on the first 2 s
            info, m = None, None
            for i in range(tries):
                try:
                    m = OfdmModem(serial=self.serials[name])
                    info = m.info(timeout=3.0)
                except Exception as e:      # noqa: BLE001
                    info = None
                    if i == tries - 1:
                        raise RuntimeError(f"board {name}: {e}") from None
                if info:
                    break
                try:
                    m and m.close()
                except Exception:       # noqa: BLE001
                    pass
                time.sleep(2.0)
            if not info:
                raise RuntimeError(f"board {name} did not answer INFO in {tries} tries")
            s = Station(m, name, info.get("msg_max", bc.BOARD_MSG_MAX), self.verbose)
            s.info = info
            self.st[name] = s
            p = Pump(s)
            p.start()
            self.pump[name] = p
        # first status from both
        self.wait(lambda: all(s.status for s in self.st.values()), 5.0)

    def detach(self):
        for p in self.pump.values():
            p.stop.set()
        for p in self.pump.values():
            p.join(2.0)
        for n, s in self.st.items():
            self.archive[n] += s.log
            try:
                s.rx.close()
                s.m.close()
            except Exception:       # noqa: BLE001
                pass
        self.st, self.pump = {}, {}
        time.sleep(1.0)

    def other(self, name):
        return "B" if name == "A" else "A"

    def cmd(self, name, fn, *args):
        """run a Console command on the board's own thread"""
        return self.pump[name].call(lambda: fn(self.st[name], *args))

    def wait(self, pred, timeout, poll=0.2):
        t0 = now()
        while now() - t0 < timeout:
            if pred():
                return now() - t0
            for p in self.pump.values():
                if p.fault:
                    raise RuntimeError(f"pump {p.s.name}: {p.fault}")
            time.sleep(poll)
        return None

    def quiesce(self, timeout=120.0, need=3):
        """both boards idle: nothing pending, queues empty, `need` statuses"""
        clean = 0
        t0 = now()
        while now() - t0 < timeout:
            ok = all(s.status and not s.status["pending"] and not any(s.status["queues"])
                     and not s.pending for s in self.st.values())
            clean = clean + 1 if ok else 0
            if clean >= need:
                time.sleep(1.0)
                return True
            time.sleep(0.5)
        return False

    def counters(self):
        return {n: s.counters() for n, s in self.st.items()}


def delta(before, after):
    out = {}
    for n in before:
        out[n] = {k: (after[n][k] - before[n][k]) if isinstance(before[n].get(k), int)
                  and isinstance(after[n].get(k), int) and k in ("tx", "rx", "timeouts", "retransmissions")
                  else after[n][k] for k in after[n]}
    return out


# ---------------------------------------------------------------------------
# scenarios: each returns a dict of metrics and raises on failure

def sc_attach(b, a):
    out = {}
    for n, s in b.st.items():
        out[n] = dict(serial=s.info["serial"], fw=s.info.get("fw"), msg_max=s.msg_max,
                      **s.counters())
    return out


def send_and_wait(b, src, text, timeout):
    dst = b.other(src)
    tag = text.encode()
    t0 = now()
    b.cmd(src, Station.cmd_send, text)
    dt = b.wait(lambda: any(d == tag for _, _, d in b.st[dst].msgs), timeout)
    if dt is None:
        raise RuntimeError(f"message {text!r} {src}->{dst} not delivered in {timeout:.0f} s")
    return now() - t0


def sc_warmup(b, a):
    out = {}
    for src in ("A", "B"):
        dst = b.other(src)
        before = b.counters()
        st0 = dict(b.st[src].status or {})
        t0 = now()
        b.cmd(src, Station.cmd_bulk, 8)
        pattern = bytes(range(8))
        dt = b.wait(lambda: any(d == pattern for _, _, d in b.st[dst].msgs), a.t_msg)
        if dt is None:
            raise RuntimeError(f"primer {src}->{dst} not delivered")
        hs = b.wait(lambda: (b.st[src].status or {}).get("peer_state", 0) >= 3, 90.0)
        b.quiesce()
        out[f"{src}->{dst}"] = dict(
            rung_before=st0.get("rung_now"), peer_state_before=st0.get("peer_state"),
            deliver_s=round(dt, 1), handshake_s=None if hs is None else round(now() - t0, 1),
            rung_after=b.st[src].status.get("rung_now"), snr_db=round(b.st[src].status["snr_db"], 1),
            peer_win=b.st[src].status.get("peer_win_max"), counters=delta(before, b.counters()))
        b.st[dst].msgs.clear()
    return out


def sc_message(b, a):
    out = {}
    for src in ("A", "B"):
        dst = b.other(src)
        lat = []
        before = b.counters()
        for i in range(a.n_msgs):
            lat.append(round(send_and_wait(b, src, f"bench {src}{dst} msg {i} {random.random():.6f}", a.t_msg), 1))
            b.quiesce(60.0, 2)
        out[f"{src}->{dst}"] = dict(latency_s=lat, mean_s=round(sum(lat) / len(lat), 1),
                                    counters=delta(before, b.counters()))
    return out


def sc_chat(b, a):
    before = b.counters()
    texts = {n: [f"chat {n} {i} {random.random():.6f}" for i in range(a.n_msgs)] for n in ("A", "B")}
    t0 = now()
    for i in range(a.n_msgs):
        for n in ("A", "B"):
            b.cmd(n, Station.cmd_send, texts[n][i])
    want = {n: {t.encode() for t in texts[n]} for n in ("A", "B")}

    def all_in():
        return all(want[n] <= {d for _, _, d in b.st[b.other(n)].msgs} for n in ("A", "B"))
    dt = b.wait(all_in, a.t_msg * a.n_msgs)
    if dt is None:
        missing = {n: len(want[n] - {d for _, _, d in b.st[b.other(n)].msgs}) for n in ("A", "B")}
        raise RuntimeError(f"bidirectional chat incomplete, missing {missing}")
    lat = {}
    for n in ("A", "B"):
        arr = {d: t for t, _, d in b.st[b.other(n)].msgs}
        lat[f"{n}->{b.other(n)}"] = [round(arr[t.encode()] - t0, 1) for t in texts[n]]
    b.quiesce()
    return dict(all_delivered_s=round(dt, 1), arrival_s=lat, counters=delta(before, b.counters()))


def sc_bulk(b, a):
    out = {}
    for src in ("A", "B"):
        dst = b.other(src)
        before = b.counters()
        n = a.bulk_bytes
        pattern = bytes((i & 0xFF) for i in range(n))
        lat = []
        for _ in range(a.n_bulk):
            t0 = now()
            k0 = len(b.st[dst].msgs)
            b.cmd(src, Station.cmd_bulk, n)
            dt = b.wait(lambda: any(d == pattern for _, _, d in b.st[dst].msgs[k0:]), a.t_msg)
            if dt is None:
                raise RuntimeError(f"bulk {n} B {src}->{dst} not delivered")
            lat.append(round(now() - t0, 1))
            b.quiesce(60.0, 2)
        out[f"{src}->{dst}"] = dict(bytes=n, latency_s=lat,
                                    bytes_per_s=round(n / (sum(lat) / len(lat)), 1),
                                    counters=delta(before, b.counters()))
    return out


def make_file(path, nbytes, seed):
    rng = random.Random(seed)
    data = bytes(rng.getrandbits(8) for _ in range(nbytes))
    with open(path, "wb") as f:
        f.write(data)
    return data


def file_timeout(a, nbytes):
    return a.t_file_base + nbytes / a.t_file_rate


def sendfile_and_wait(b, src, path, data, timeout):
    dst = b.other(src)
    k0 = len(b.st[dst].files)
    t0 = now()
    b.cmd(src, Station.cmd_sendfile, path)
    dt = b.wait(lambda: len(b.st[dst].files) > k0, timeout)
    if dt is None:
        raise RuntimeError(f"file {os.path.basename(path)} {src}->{dst} not complete in {timeout:.0f} s "
                           f"({b.st[src].sent_parts}/{len(b.st[src].pending)} parts handed)")
    _, rx_path, total, on_air = b.st[dst].files[-1]
    got = open(rx_path, "rb").read()
    return dict(seconds=round(now() - t0, 1), bytes=len(data), on_air=on_air,
                exact=got == data, bytes_per_s=round(len(data) / (now() - t0), 1))


def sc_file(b, a):
    out = {}
    for src in ("A", "B"):
        dst = b.other(src)
        path = os.path.join(a.workdir, f"f{src}{dst}.bin")
        data = make_file(path, a.file_bytes, zlib.crc32(f"f{src}{a.file_bytes}".encode()))
        before = b.counters()
        r = sendfile_and_wait(b, src, path, data, file_timeout(a, a.file_bytes))
        b.quiesce()
        r["counters"] = delta(before, b.counters())
        r["rung"] = b.st[src].status.get("rung_now")
        out[f"{src}->{dst}"] = r
        if not r["exact"]:
            raise RuntimeError(f"file {src}->{dst} not byte-exact")
    return out


def sc_file_bidir(b, a):
    n = max(1000, a.file_bytes // 2)
    paths, datas = {}, {}
    for src in ("A", "B"):
        paths[src] = os.path.join(a.workdir, f"x{src}{b.other(src)}.bin")
        datas[src] = make_file(paths[src], n, zlib.crc32(f"bidir{src}".encode()))
    before = b.counters()
    k0 = {n_: len(b.st[n_].files) for n_ in ("A", "B")}
    t0 = now()
    for src in ("A", "B"):
        b.cmd(src, Station.cmd_sendfile, paths[src])
    dt = b.wait(lambda: all(len(b.st[n_].files) > k0[n_] for n_ in ("A", "B")), 2 * file_timeout(a, n))
    if dt is None:
        raise RuntimeError("bidirectional files did not both complete: " + ", ".join(
            f"{n_} got {len(b.st[n_].files) - k0[n_]}" for n_ in ("A", "B")))
    out = dict(bytes_each=n, both_done_s=round(dt, 1), aggregate_bytes_per_s=round(2 * n / dt, 1))
    for src in ("A", "B"):
        dst = b.other(src)
        t_done, rx_path, total, on_air = b.st[dst].files[-1]
        out[f"{src}->{dst}"] = dict(seconds=round(t_done - t0, 1),
                                    exact=open(rx_path, "rb").read() == datas[src])
    b.quiesce()
    out["counters"] = delta(before, b.counters())
    if not all(out[f"{s}->{b.other(s)}"]["exact"] for s in ("A", "B")):
        raise RuntimeError("bidirectional file not byte-exact")
    return out


def sc_file_chat(b, a):
    out = {}
    for src in ("A", "B"):
        dst = b.other(src)
        n = max(1000, a.file_bytes // 2)
        path = os.path.join(a.workdir, f"c{src}{dst}.bin")
        data = make_file(path, n, zlib.crc32(f"chat{src}".encode()))
        before = b.counters()
        k0 = len(b.st[dst].files)
        t0 = now()
        b.cmd(src, Station.cmd_sendfile, path)
        texts, sent_at = [], []
        deadline = t0 + file_timeout(a, n)
        while now() < deadline and len(b.st[dst].files) == k0:
            if len(texts) < a.n_msgs and now() - t0 >= 3.0 + len(texts) * a.chat_gap:
                t = f"during-file {dst}->{src} {len(texts)} {random.random():.6f}"
                b.cmd(dst, Station.cmd_send, t)
                texts.append(t.encode())
                sent_at.append(now())
            time.sleep(0.3)
        if len(b.st[dst].files) == k0:
            raise RuntimeError(f"file {src}->{dst} with chat not complete")
        t_file = now() - t0
        rx_path = b.st[dst].files[-1][1]
        exact = open(rx_path, "rb").read() == data
        dt = b.wait(lambda: all(any(d == t for _, _, d in b.st[src].msgs) for t in texts), a.t_msg)
        lat = []
        for t, t_s in zip(texts, sent_at):
            arr = [tt for tt, _, d in b.st[src].msgs if d == t]
            lat.append(round(arr[0] - t_s, 1) if arr else None)
        b.quiesce()
        out[f"file {src}->{dst}, chat {dst}->{src}"] = dict(
            file_s=round(t_file, 1), bytes=n, exact=exact, bytes_per_s=round(n / t_file, 1),
            chat_sent=len(texts), chat_latency_s=lat, chat_all_delivered=dt is not None,
            counters=delta(before, b.counters()))
        if not exact or dt is None:
            raise RuntimeError(f"file+chat {src}->{dst}: exact {exact}, chat delivered {dt is not None}")
    return out


def bcast_and_wait(b, src, fn, arg, timeout):
    dst = b.other(src)
    k0 = len(b.st[dst].bcasts)
    t0 = now()
    b.cmd(src, fn, arg)
    dt = b.wait(lambda: len(b.st[dst].bcasts) > k0, timeout)
    if dt is None:
        raise RuntimeError(f"broadcast {src}->{dst} did not end in {timeout:.0f} s "
                           f"({len(b.st[dst].bcasts) - k0} ended)")
    t_start, t_end, ptype, fo, lo, data = b.st[dst].bcasts[-1]
    return dict(seconds=round(t_end - t0, 1), first_group_s=round(t_start - t0, 1),
                frames_ok=fo, frames_lost=lo, ptype=ptype), data


def sc_bcast(b, a):
    out = {}
    for src in ("A", "B"):
        dst = b.other(src)
        text = f"broadcast from {src} {random.random():.6f} " + "x" * 60
        before = b.counters()
        r, data = bcast_and_wait(b, src, Station.cmd_bcast, text, a.t_bcast)
        r["exact"] = data.decode("utf-8", "replace") == text
        r["rung_logged"] = [t for _, t in b.st[src].log if "rung" in t and "broadcast" in t][-1:]
        b.quiesce()
        r["counters"] = delta(before, b.counters())
        out[f"{src}->{dst}"] = r
        if not r["exact"]:
            raise RuntimeError(f"broadcast text {src}->{dst} garbled")
    return out


def sc_bcastfile(b, a):
    out = {}
    for src in ("A", "B"):
        dst = b.other(src)
        path = os.path.join(a.workdir, f"bc{src}{dst}.bin")
        data = make_file(path, a.bcast_bytes, zlib.crc32(f"bc{src}".encode()))
        before = b.counters()
        r, got = bcast_and_wait(b, src, Station.cmd_bcastfile, path, file_timeout(a, a.bcast_bytes))
        r.update(bytes_sent=len(data), bytes_stored=len(got), exact=got == data,
                 bytes_per_s=round(len(got) / r["seconds"], 1))
        b.quiesce()
        r["counters"] = delta(before, b.counters())
        out[f"{src}->{dst}"] = r
    return out


def sc_bcast_msg(b, a):
    out = {}
    for src in ("A", "B"):
        dst = b.other(src)
        path = os.path.join(a.workdir, f"bm{src}{dst}.bin")
        data = make_file(path, a.bcast_bytes, zlib.crc32(f"bm{src}".encode()))
        before = b.counters()
        k0 = len(b.st[dst].bcasts)
        t0 = now()
        b.cmd(src, Station.cmd_bcastfile, path)
        # queue the reply once the first group is on the air
        b.wait(lambda: b.st[dst]._bc is not None, 60.0)
        text = f"during-broadcast {dst}->{src} {random.random():.6f}"
        t_msg = now()
        b.cmd(dst, Station.cmd_send, text)
        dt = b.wait(lambda: len(b.st[dst].bcasts) > k0, file_timeout(a, a.bcast_bytes))
        if dt is None:
            raise RuntimeError(f"broadcast {src}->{dst} with a queued reply did not end")
        _, t_end, _, fo, lo, got = b.st[dst].bcasts[-1]
        dm = b.wait(lambda: any(d == text.encode() for _, _, d in b.st[src].msgs), a.t_msg)
        b.quiesce()
        out[f"bcast {src}->{dst}, msg {dst}->{src}"] = dict(
            bcast_s=round(t_end - t0, 1), frames_ok=fo, frames_lost=lo, exact=got == data,
            msg_queued_at_s=round(t_msg - t0, 1),
            msg_latency_s=None if dm is None else round(now() - t_msg, 1),
            msg_after_bcast_end_s=None if dm is None else round(now() - t_end, 1),
            counters=delta(before, b.counters()))
        if dm is None:
            raise RuntimeError("message queued during the broadcast never arrived")
    return out


def sc_stale(b, a):
    out = {}
    idle = max(a.stale_secs, RX_STALE_S + 15)
    r0 = {n: b.st[n].status.get("rung_now") for n in ("A", "B")}
    time.sleep(idle)
    r1 = {n: b.st[n].status.get("rung_now") for n in ("A", "B")}
    before = b.counters()
    lat = send_and_wait(b, "A", f"after-idle {random.random():.6f}", a.t_msg)
    b.quiesce()
    out.update(idle_s=idle, rung_before_idle=r0, rung_after_idle=r1, message_latency_s=round(lat, 1),
               rung_after_message={n: b.st[n].status.get("rung_now") for n in ("A", "B")},
               counters=delta(before, b.counters()))
    return out


# ---------------------------------------------------------------------------
# speech through the webvoice server

class Voice:
    def __init__(self, url):
        self.url = url.rstrip("/")

    def _req(self, path, body=None, timeout=300.0):
        req = urllib.request.Request(self.url + path, data=body,
                                     method="POST" if body is not None else "GET")
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode())

    def up(self):
        try:
            return bool(self._req("/api/status", timeout=3.0))
        except Exception:       # noqa: BLE001
            return False

    def select(self, tx, rx):
        return self._req("/api/select", json.dumps({"tx": tx, "rx": rx}).encode())

    def status(self):
        return self._req("/api/status")

    def talk(self, pcm, sr, profile="live"):
        """warm-up, start, feed in real time, stop (which drains and
        writes the far end's recording); returns the server's verdicts."""
        warm = self._req("/api/warmup", b"")
        t0 = now()
        self._req(f"/api/start?profile={profile}&rt=1", b"")
        block = sr // 2
        for i in range(0, len(pcm), block):
            self._req(f"/api/audio?sr={sr}", pcm[i:i + block].astype("<f4").tobytes(), timeout=30.0)
            ahead = (i + block) / sr - (now() - t0)
            if ahead > 0:
                time.sleep(ahead)
        res = self._req("/api/stop", b"", timeout=400.0)
        res["warmup"] = warm
        res["talk_s"] = round(now() - t0, 1)
        return res


def load_wav(path, secs):
    import numpy as np
    with wave.open(path, "rb") as w:
        sr, n, sw, ch = w.getframerate(), w.getnframes(), w.getsampwidth(), w.getnchannels()
        raw = w.readframes(n)
    assert sw == 2, "16-bit PCM expected"
    x = np.frombuffer(raw, dtype="<i2").astype("float32") / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch).mean(axis=1)
    if secs:
        x = x[:int(secs * sr)]
    return x, sr


def sc_speech(b, a):
    v = Voice(a.voice_url)
    pcm, sr = load_wav(a.voice_src, a.voice_secs)
    out = dict(source=os.path.basename(a.voice_src), seconds=round(len(pcm) / sr, 1))
    b.detach()          # the server needs the boards
    try:
        if a._server is None and a.voice_start:
            print(f"{ts()}   starting the webvoice server ...", flush=True)
            a._server = start_voice_server(a)
        if not v.up():
            raise RuntimeError(f"webvoice server not reachable at {a.voice_url}")
        for src in ("A", "B"):
            dst = b.other(src)
            v.select(b.serials[src], b.serials[dst])
            time.sleep(2.0)
            r = v.talk(pcm, sr)
            st = v.status()
            out[f"{src}->{dst}"] = dict(
                ok=r.get("ok"), warm_s=(r.get("warmup") or {}).get("seconds"),
                rung=st.get("rung"), talk_s=r.get("talk_s"),
                t_first_tx=r.get("t_first_tx"), t_first_rx=r.get("t_first_rx"),
                t_first_out=r.get("t_first_out"), rx_seconds=r.get("seconds"),
                tx_bytes=r.get("tx_bytes"), rx_bytes=r.get("rx_bytes"),
                loss_pct=r.get("loss_pct"), frames_ok=r.get("frames_ok"),
                frames_lost=r.get("frames_lost"), groups=r.get("groups"),
                file=r.get("file"), reason=r.get("reason"))
            time.sleep(3.0)
    finally:
        # The server has no "let go" call: a select only swaps one pair
        # of handles for another. A server this run spawned is stopped
        # (and respawned for the next speech scenario); an external one
        # keeps the boards, so the run ends after the speech scenarios
        # (main orders them last in that case).
        if a._server is not None:
            a._server.terminate()
            a._server.wait(10)
            a._server = None
            time.sleep(2.0)
            try:
                b.attach()
            except Exception as e:      # noqa: BLE001
                # the speech verdicts stand; main re-attaches before the
                # next scenario and reports if that fails too
                out["reattach_error"] = f"{type(e).__name__}: {e}"
        else:
            b.final_detach = True
    bad = [k for k in ("A->B", "B->A") if not out.get(k, {}).get("ok")]
    if bad:
        raise RuntimeError(f"speech failed: {bad}")
    return out


def sc_speech_file(b, a):
    path = os.path.join(a.workdir, "sfAB.bin")
    data = make_file(path, a.file_bytes, 4242)
    before = b.counters()
    r = sendfile_and_wait(b, "A", path, data, file_timeout(a, a.file_bytes))
    b.quiesce()
    r["counters"] = delta(before, b.counters())
    r["rung"] = b.st["A"].status.get("rung_now")
    if not r["exact"]:
        raise RuntimeError("file after speech not byte-exact")
    return {"A->B": r}


SCENARIOS = [
    ("attach", sc_attach), ("warmup", sc_warmup), ("message", sc_message),
    ("chat", sc_chat), ("bulk", sc_bulk), ("file", sc_file),
    ("file_bidir", sc_file_bidir), ("file_chat", sc_file_chat),
    ("bcast", sc_bcast), ("bcastfile", sc_bcastfile), ("bcast_msg", sc_bcast_msg),
    ("speech", sc_speech), ("speech_file", sc_speech_file), ("stale", sc_stale),
]


# ---------------------------------------------------------------------------

def list_serials():
    import usb.core
    from ofdm_modem import _read_serial
    out = []
    for d in usb.core.find(find_all=True, idVendor=VID, idProduct=PID):
        try:
            out.append(_read_serial(d))
        except Exception:       # noqa: BLE001
            pass
    return sorted(out)


def start_voice_server(a):
    log = open(os.path.join(a.workdir, "webvoice.log"), "ab")
    port = a.voice_url.rsplit(":", 1)[1].split("/")[0]
    p = subprocess.Popen([a.voice_python, os.path.join(HERE, "webvoice", "server.py"), port],
                         stdout=log, stderr=subprocess.STDOUT, cwd=ROOT)
    v = Voice(a.voice_url)
    t0 = now()
    while now() - t0 < 300 and p.poll() is None:
        if v.up():
            return p
        time.sleep(2.0)
    raise RuntimeError("webvoice server did not come up (see webvoice.log)")


def summary_line(name, rec):
    r = rec.get("result") or {}
    bits = []
    if name == "warmup":
        bits = [f"{k}: {v['deliver_s']} s, handshake {v['handshake_s']} s, rung {v['rung_after']}, "
                f"SNR {v['snr_db']} dB" for k, v in r.items()]
    elif name in ("message", "bulk"):
        bits = [f"{k}: {v['latency_s']} s" for k, v in r.items()]
    elif name == "chat":
        bits = [f"all {r.get('all_delivered_s')} s, arrivals {r.get('arrival_s')}"]
    elif name in ("file",):
        bits = [f"{k}: {v['bytes']} B in {v['seconds']} s ({v['bytes_per_s']} B/s), exact {v['exact']}, "
                f"tx {v['counters'][k[0]]['tx']} to {v['counters'][k[0]]['timeouts']} "
                f"retx {v['counters'][k[0]]['retransmissions']}" for k, v in r.items()]
    elif name == "file_bidir":
        bits = [f"{r.get('bytes_each')} B each way, both in {r.get('both_done_s')} s "
                f"({r.get('aggregate_bytes_per_s')} B/s aggregate)"]
    elif name == "file_chat":
        bits = [f"{k}: file {v['file_s']} s ({v['bytes_per_s']} B/s), chat latency {v['chat_latency_s']}"
                for k, v in r.items()]
    elif name == "bcast":
        bits = [f"{k}: {v['seconds']} s, {v['frames_ok']} ok / {v['frames_lost']} lost, exact {v['exact']}"
                for k, v in r.items()]
    elif name == "bcastfile":
        bits = [f"{k}: {v['bytes_stored']}/{v['bytes_sent']} B in {v['seconds']} s, "
                f"{v['frames_ok']} ok / {v['frames_lost']} lost" for k, v in r.items()]
    elif name == "bcast_msg":
        bits = [f"{k}: bcast {v['bcast_s']} s ({v['frames_lost']} lost), reply queued at {v['msg_queued_at_s']} s "
                f"arrived {v['msg_after_bcast_end_s']} s after the end" for k, v in r.items()]
    elif name == "speech":
        bits = [f"{k}: warm {v.get('warm_s')} s, first audio out {v.get('t_first_out')} s, "
                f"{v.get('rx_seconds')} s received, {v.get('loss_pct')} % lost, "
                f"{v.get('frames_ok')} ok / {v.get('frames_lost')} lost"
                for k, v in r.items() if isinstance(v, dict)]
    elif name == "speech_file":
        bits = [f"{k}: {v['seconds']} s ({v['bytes_per_s']} B/s)" for k, v in r.items()]
    elif name == "stale":
        bits = [f"idle {r.get('idle_s')} s, rung {r.get('rung_before_idle')} -> {r.get('rung_after_idle')}, "
                f"message {r.get('message_latency_s')} s, rung after {r.get('rung_after_message')}"]
    elif name == "attach":
        bits = [f"{k}: rung {v['rung_now']} SNR {bc.snr_str(v['snr_db'])} peer_state {v['peer_state']} "
                f"die {bc.temp_str(v['temp_c'])}" for k, v in r.items()]
    return "; ".join(bits)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--a", help="serial of board A (default: first attached)")
    ap.add_argument("--b", help="serial of board B (default: second attached)")
    ap.add_argument("--scenarios", default=None, help="comma list (default: all but stale)")
    ap.add_argument("--quick", action="store_true", help="small sizes and counts")
    ap.add_argument("--file-bytes", type=int, default=6000)
    ap.add_argument("--bcast-bytes", type=int, default=4096)
    ap.add_argument("--bulk-bytes", type=int, default=200)
    ap.add_argument("--n-msgs", type=int, default=3)
    ap.add_argument("--n-bulk", type=int, default=2)
    ap.add_argument("--chat-gap", type=float, default=8.0, help="seconds between chat messages during a file")
    ap.add_argument("--stale-secs", type=float, default=120.0)
    ap.add_argument("--t-msg", type=float, default=180.0, help="message delivery timeout")
    ap.add_argument("--t-bcast", type=float, default=240.0)
    ap.add_argument("--t-file-base", type=float, default=120.0)
    ap.add_argument("--t-file-rate", type=float, default=15.0, help="B/s assumed for file timeouts")
    ap.add_argument("--voice-url", default="http://127.0.0.1:8080")
    ap.add_argument("--voice-start", action="store_true", help="spawn the webvoice server")
    ap.add_argument("--voice-python", default=VOICE_PYTHON)
    ap.add_argument("--voice-src", default=None, help="speech WAV (default: newest ~/voice_rx/tx_*.wav)")
    ap.add_argument("--voice-secs", type=float, default=40.0, help="seconds of the speech source to send")
    ap.add_argument("--workdir", default=os.path.join(ROOT, "results", "board_bench_work"))
    ap.add_argument("--out", default=None)
    ap.add_argument("--merge", action="store_true",
                    help="update only the scenarios run into an existing results file")
    ap.add_argument("-v", "--verbose", action="store_true", help="echo both consoles")
    a = ap.parse_args()

    if a.quick:
        a.file_bytes = min(a.file_bytes, 2000)
        a.bcast_bytes = min(a.bcast_bytes, 1024)
        a.n_msgs, a.n_bulk = 2, 1
        a.voice_secs = min(a.voice_secs, 12.0)
    names = [n for n, _ in SCENARIOS if n != "stale"]
    if a.scenarios:
        names = [n.strip() for n in a.scenarios.split(",")]
        unknown = [n for n in names if n not in dict(SCENARIOS)]
        if unknown:
            ap.error(f"unknown scenario(s) {unknown}; known: {[n for n, _ in SCENARIOS]}")
    a.workdir = os.path.abspath(a.workdir)
    os.makedirs(a.workdir, exist_ok=True)
    if any(n.startswith("speech") for n in names) and not a.voice_src:
        # a stable source: copied once into the work dir. The server
        # writes a tx_*.wav of every transmission, so "the newest" would
        # be this benchmark's own previous (possibly truncated) run --
        # take the LONGEST recording the operator left there instead,
        # cut to --voice-secs.
        stable = os.path.join(a.workdir, "speech_src.wav")
        if not os.path.exists(stable):
            cands = sorted((f for f in os.listdir(VOICE_RX_DIR) if f.startswith("tx_") and f.endswith(".wav")),
                           key=lambda f: os.path.getsize(os.path.join(VOICE_RX_DIR, f))) \
                if os.path.isdir(VOICE_RX_DIR) else []
            if not cands:
                ap.error("no speech source: pass --voice-src")
            import shutil
            shutil.copy(os.path.join(VOICE_RX_DIR, cands[-1]), stable)
        a.voice_src = stable
    if a.out:
        a.out = os.path.abspath(a.out)
    if a.voice_src:
        a.voice_src = os.path.abspath(a.voice_src)
    os.chdir(a.workdir)            # rx_<file> lands here
    serials = list_serials()
    if not a.a or not a.b:
        if len(serials) < 2:
            print(f"need two boards, found {serials}", file=sys.stderr)
            return 1
        a.a = a.a or serials[0]
        a.b = a.b or [s for s in serials if s != a.a][0]

    a._server = None
    if any(n.startswith("speech") for n in names):
        if a.voice_start:
            if Voice(a.voice_url).up():
                print(f"a webvoice server is already up at {a.voice_url}; it holds the boards, "
                      "stop it or drop --voice-start", file=sys.stderr)
                return 1
        else:
            # an external server keeps the boards after speech: run
            # speech last, and nothing that needs the boards after it
            tail = [n for n in names if n.startswith("speech")]
            names = [n for n in names if not n.startswith("speech")] + tail
            if "speech_file" in tail:
                print("speech_file needs --voice-start (an external server keeps the boards); skipped",
                      file=sys.stderr)
                names.remove("speech_file")

    stand = Stand(a.a, a.b, a.verbose)
    stand.final_detach = False
    print(f"{ts()} attaching A={a.a[:6]} B={a.b[:6]}", flush=True)
    stand.attach()
    results, t_all = {}, now()
    try:
        for name in names:
            fn = dict(SCENARIOS)[name]
            if stand.final_detach:
                results[name] = dict(ok=False, seconds=0, result=None,
                                     error="skipped: the external voice server holds the boards",
                                     summary="skipped")
                continue
            print(f"{ts()} === {name}", flush=True)
            rec = dict(ok=False, seconds=None, result=None, error=None)
            t0 = now()
            try:
                if not stand.st:
                    stand.attach()
                stand.quiesce(60.0, 2)
                rec["result"] = fn(stand, a)
                rec["ok"] = True
            except Exception as e:      # noqa: BLE001
                rec["error"] = f"{type(e).__name__}: {e}"
                if a.verbose:
                    import traceback
                    traceback.print_exc()
                # the partial result, if the scenario built one, is lost;
                # the consoles' logs are kept below
                try:
                    if stand.st:
                        stand.quiesce(60.0, 2)
                except Exception:       # noqa: BLE001
                    pass
            rec["seconds"] = round(now() - t0, 1)
            rec["summary"] = summary_line(name, rec) if rec["ok"] else rec["error"]
            results[name] = rec
            print(f"{ts()}   {'ok ' if rec['ok'] else 'FAIL'} {rec['seconds']:7.1f} s  {rec['summary']}", flush=True)
    finally:
        if stand.st:
            stand.detach()
        logs = {n: [(round(t - t_all, 1), x) for t, x in stand.archive[n]] for n in ("A", "B")}
        if a._server is not None:
            a._server.terminate()

    out = dict(date=time.strftime("%Y-%m-%d %H:%M"), boards={"A": a.a, "B": a.b}, quick=a.quick,
               args={k: v for k, v in vars(a).items() if not k.startswith("_")},
               total_seconds=round(now() - t_all, 1), scenarios=results, console_logs=logs)
    suffix = "_quick" if a.quick else ""
    path = a.out or os.path.join(ROOT, "results", f"board_bench{suffix}.json")
    if a.merge and os.path.exists(path):
        old = json.load(open(path))
        for rec in results.values():
            rec["date"] = out["date"]
        old["scenarios"].update(results)
        old["console_logs"] = logs
        old["merged"] = out["date"]
        out, results = old, old["scenarios"]
    # write-then-rename: a failed write (disk full, measured) must not
    # destroy the previous record
    with open(path + ".tmp", "w") as f:
        json.dump(out, f, indent=1, default=str)
    os.replace(path + ".tmp", path)
    md = [f"# Board integration benchmark ({out['date']})", "",
          f"A = {a.a}, B = {a.b}; {out['total_seconds']:.0f} s total"
          + (" (quick)" if a.quick else ""), "",
          "| scenario | result | seconds | summary |", "|---|---|---|---|"]
    for name, rec in results.items():
        md.append(f"| {name} | {'ok' if rec['ok'] else 'FAIL'} | {rec['seconds']} | {rec['summary']} |")
    with open(path[:-5] + ".md.tmp", "w") as f:
        f.write("\n".join(md) + "\n")
    os.replace(path[:-5] + ".md.tmp", path[:-5] + ".md")
    n_ok = sum(r["ok"] for r in results.values())
    print(f"\n{n_ok}/{len(results)} scenarios ok in {out['total_seconds']:.0f} s -> {path} and .md")
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
