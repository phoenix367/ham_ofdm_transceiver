#!/usr/bin/env python3
"""Interactive station console against a REAL board, chosen by serial.

    ./board_console.py --list
    ./board_console.py --serial 240041000551333438363436 [--name B]

The same console as `ofdm_console` (app.c), but the station runs on the
STM32 instead of in this process. That is the whole difference, and it
decides the shape of this program:

  * app.c is handed a device carrying 12 kHz int16 audio and runs the
    entire stack locally -- streaming receiver, station, transmitter.
  * a board already runs that stack in its own firmware and speaks a
    MESSAGE-level protocol over USB (submit / message / status). There
    is no way to hand it audio: UP_EVT_AUDIO is a receive-side debug
    tap and there is no inbound audio command at all.

So this is not app.c with a different device -- it is a terminal onto a
station that lives somewhere else. What it does share with app.c, byte
for byte, is the application envelope, so a file sent from here is
received by an app.c station (and by another board) and vice versa:

    magic(1) "FILE:" basename NUL part n_parts data...
    magic 0x01/0x02: part(1) n_parts(1); 0x03/0x04 (what we send):
    part(2) n_parts(2) little-endian -- 255 parts capped a transfer at
    ~58 kB over USB. Even magics are DEFLATEd whole.

with magic 0x01 raw and 0x02 for a whole-file DEFLATE stream, exactly
as documented in app.c. Compression is applied once to the whole file
and kept only if it actually shrank.

Two board-specific limits this has to respect, because the firmware is
built with cport's MCU-modest defaults rather than demoapp's:

  * ST_MSG_MAX is 256 on the board against demoapp's 4096, so a part
    carries ~230 bytes, not 3000. Part size is NOT on the wire -- each
    part is self-delimiting -- so the two ends need not agree, and a
    board can send to an app.c peer that uses 3000-byte parts.
  * ST_POOL_SLOTS is 12 and each queue is ST_MAX_MSGS = 8, so a file
    cannot simply be dumped into the queue the way app.c dumps it.
    Parts are paced against the q_bulk depth the board reports.

The message limit is not discoverable: up_info_t does not carry it. It
is therefore a constant here, matching the cport default, with
--msg-max to override if the firmware was built differently.
"""
import argparse
import os
import select
import struct
import sys
import time
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "host"))
from ofdm_modem import OfdmModem, VID, PID          # noqa: E402

FILE_MAGIC = 0x01
FILE_MAGIC_Z = 0x02
FILE_MAGIC_W = 0x03
FILE_MAGIC_WZ = 0x04
FILE_MAGICS = (FILE_MAGIC, FILE_MAGIC_Z, FILE_MAGIC_W, FILE_MAGIC_WZ)
FILE_TAG = b"FILE:"
FILE_MAX_SRC = 1 << 19          # same cap app.c uses

QOS_CONTROL, QOS_INTERACTIVE, QOS_BULK = 0, 1, 2

# station.h's diagnostic events rendered as human-readable lines -- the
# Python twin of station_diag_format() in cport/src/station.c; keep the
# wording in step with it.
FLAGS = {0: "data", 1: "last-frag", 2: "no-data", 3: "burst-ack",
         4: "stream", 5: "stream+last", 6: "burst-data", 7: "burst+last"}
TYPS = {0: "beacon", 4: "data", 5: "ext-data", 6: "bcast"}
SOFF = {1: "the PHY refused to build", 2: "peer did not follow (sticky)",
        3: "windows kept timing out"}
DIAG_FMT = [
    lambda a, b, c, d: f"tx: rung {a}, {TYPS.get(b, '?')} frame, flags "
                       f"{FLAGS.get(c, c)}, {d} B payload",
    lambda a, b, c, d: f"rx: flags {FLAGS.get(a, a)}, seq {b}, ack {c}, "
                       f"snr {d / 10.0:+.1f} dB",
    lambda a, b, c, d: f"TIMEOUT at rung {b} -> {a} consecutive loss(es)"
                       + (" (first burst-ack miss, forgiven)" if c else ""),
    lambda a, b, c, d: f"rung {a} -> {b} (losses {c}, cap {d})",
    lambda a, b, c, d: f"burst engage: {a} frag(s) x {b} B, transfer {c}",
    lambda a, b, c, d: f"burst frag {a}"
                       + (" +ack-request" if b else "") + f", window left {c}",
    lambda a, b, c, d: f"bitmap ack sent (transfer {a}, {b} B)",
    lambda a, b, c, d: f"bitmap ack: {a}/{b} frag(s) delivered",
    lambda a, b, c, d: f"burst timeout -> 1-frame probe (transfer {a})",
    lambda a, b, c, d: "burst transfer "
                       + ("received whole" if a else "fully acked"),
    lambda a, b, c, d: f"STREAMED {a} block(s) behind one preamble, {b} "
                       f"samples ({b / 12000.0:.1f} s air), resync {c}",
    lambda a, b, c, d: f"stream rx: {a} of {b} block(s) decoded",
    lambda a, b, c, d: f"streaming OFF: {SOFF.get(a, a)}"
                       + (", remembered for this peer" if b else ""),
    lambda a, b, c, d: f"reply timer: srtt {a} ms, var {b} ms -> budget "
                       f"{c} ms (air term {d} ms)",
    lambda a, b, c, d: f"burst window {b} of {a} (ceiling), {c} frag(s), "
                       f"{d} s air",
    lambda a, b, c, d: f"frag {a} B exceeds the air cap at rung {b} "
                       f"({c} frags) -> disengage, legacy path",
    lambda a, b, c, d: (f"caps: no answer after {b} tries -- peer assumed "
                        "legacy, defaults apply" if a == 3 else
                        f"caps {('sent', 'sent (reply)', 'received')[a]}: "
                        f"{CAPS_NAMES(b)} msg {c} B, window {d}"),
]


def CAPS_NAMES(f):
    return "".join(n + " " for bit, n in ((1, "stream"), (2, "ext"),
                                          (4, "ldpc"), (8, "burst"),
                                          (16, "bcast")) if f & bit)
QOS_NAME = {0: "ctl", 1: "inter", 2: "bulk"}

BOARD_MSG_MAX = 256             # cport ST_MSG_MAX default
BOARD_QUEUE = 8                 # cport ST_MAX_MSGS
INFLIGHT = 4                    # parts we allow in the board's bulk queue


def ts():
    return time.strftime("%H:%M:%S")


def list_boards():
    import usb.core
    import usb.util
    found = list(usb.core.find(find_all=True, idVendor=VID, idProduct=PID))
    if not found:
        print(f"no OFDM modem on the bus ({VID:04x}:{PID:04x})")
        return 1
    for d in found:
        try:
            ser = usb.util.get_string(d, d.iSerialNumber)
        except Exception as e:                       # noqa: BLE001
            ser = f"<unreadable: {e}>"
        print(f"  bus {d.bus:03d} dev {d.address:03d}  serial {ser}")
    print(f"\n{len(found)} board(s). Pass --serial <serial> to choose one.")
    return 0


class RxFile:
    """Reassembles the parts of one incoming file, inflating on the fly."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.f = None
        self.path = None
        self.next_part = 0
        self.n_parts = 0
        self.zobj = None
        self.total = 0
        self.on_air = 0

    def close(self):
        if self.f:
            self.f.close()
        self.reset()


class Console:
    def __init__(self, modem, name, msg_max, compress):
        self.m = modem
        self.name = name
        self.msg_max = msg_max
        self.own_msg_max = msg_max
        self.compress = compress
        self.rx = RxFile()
        self.status = None
        self.pending = []          # parts of the file being sent
        self.pending_name = None
        self.sent_parts = 0
        self.refused = 0
        self.rx_msgs = 0
        self.rx_files = 0

    # --- output ---------------------------------------------------
    def say(self, text):
        print(f"\n{ts()} [{self.name}] {text}\n> ", end="", flush=True)

    def plain(self, text):
        print(f"{ts()} [{self.name}] {text}", flush=True)

    # --- receive --------------------------------------------------
    def on_message(self, qos, data):
        self.rx_msgs += 1
        if len(data) > 6 and data[0] in FILE_MAGICS \
                and data[1:6] == FILE_TAG:
            self.on_file_part(data)
            return
        text = data.decode("utf-8", "replace")
        self.say(f"<< [{QOS_NAME.get(qos, qos)}] {text}")

    def on_file_part(self, msg):
        nul = msg.find(b"\0", 6)
        wide = msg[0] in (FILE_MAGIC_W, FILE_MAGIC_WZ)
        meta_n = 4 if wide else 2
        if nul < 0 or len(msg) < nul + 1 + meta_n:
            self.say("<< malformed file envelope")
            return
        name = os.path.basename(msg[6:nul].decode("utf-8", "replace")) \
            or "unnamed"
        if wide:
            part = int.from_bytes(msg[nul + 1:nul + 3], "little")
            n_parts = int.from_bytes(msg[nul + 3:nul + 5], "little")
        else:
            part, n_parts = msg[nul + 1], msg[nul + 2]
        data = msg[nul + 1 + meta_n:]
        zipped = msg[0] in (FILE_MAGIC_Z, FILE_MAGIC_WZ)

        if part == 0:
            self.rx.close()
            self.rx.path = "rx_" + name
            self.rx.f = open(self.rx.path, "wb")
            self.rx.n_parts = n_parts
            self.rx.next_part = 0
            self.rx.zobj = zlib.decompressobj() if zipped else None
            self.say(f"<< file '{name}' incoming, {n_parts} part(s)"
                     f"{' deflated' if zipped else ''}")
        if not self.rx.f or part != self.rx.next_part \
                or n_parts != self.rx.n_parts:
            self.say(f"<< file '{name}': part {part + 1}/{n_parts} out of "
                     "order, transfer dropped")
            self.rx.close()
            return

        self.rx.on_air += len(data)
        try:
            out = self.rx.zobj.decompress(data) if self.rx.zobj else data
        except zlib.error as e:                      # noqa: BLE001
            self.say(f"<< file '{name}': corrupt compressed stream ({e})")
            self.rx.close()
            return
        self.rx.f.write(out)
        self.rx.total += len(out)
        self.rx.next_part += 1
        if self.rx.next_part >= self.rx.n_parts:
            if self.rx.zobj:
                tail = self.rx.zobj.flush()
                self.rx.f.write(tail)
                self.rx.total += len(tail)
            path, total, on_air = self.rx.path, self.rx.total, self.rx.on_air
            self.rx.close()
            self.rx_files += 1
            ratio = f", {total / on_air:.2f}x" if on_air and total else ""
            self.say(f"<< file complete: {path} ({total} bytes,"
                     f" {on_air} on air{ratio})")

    def on_diag(self, d):
        ev = d["ev"]
        if ev < len(DIAG_FMT):
            self.say(". " + DIAG_FMT[ev](d["a"], d["b"], d["c"], d["d"]))
        else:
            self.say(f"diag ev{ev} a={d['a']} b={d['b']} c={d['c']} "
                     f"d={d['d']}")

    def on_log(self, text):
        if "refused" in text:
            self.refused += 1
            # We paced against the queue depth and the board still said
            # no. Put the part back and let the next status retry it.
            if self.pending is not None and self.sent_parts > 0:
                self.sent_parts -= 1
        self.say(f"board: {text}")

    def on_status(self, st):
        self.status = st
        self.pump_bcfile()
        # once the handshake has run, the PEER's message limit applies
        # to what we split files into
        own = self.own_msg_max
        peer = st.get("peer_msg_max", 0) if st.get("peer_state", 0) >= 2 \
            else 0
        self.msg_max = min(own, peer) if peer else own
        self.pump_file(st)

    # --- transmit -------------------------------------------------
    def pump_file(self, st):
        """Top the board's bulk queue up to INFLIGHT parts.

        Self-correcting by construction: the board reports the true
        depth every status, so a submit that was dropped for any reason
        simply shows up as room again on the next one.
        """
        if not self.pending:
            return
        room = INFLIGHT - st["queues"][QOS_BULK]
        while room > 0 and self.sent_parts < len(self.pending):
            self.m.submit(self.pending[self.sent_parts], QOS_BULK)
            self.sent_parts += 1
            room -= 1
        if self.sent_parts >= len(self.pending):
            n = len(self.pending)
            name = self.pending_name
            self.pending = []
            self.pending_name = None
            self.sent_parts = 0
            self.say(f"file '{name}': all {n} part(s) handed to the board")

    def cmd_send(self, text):
        data = text.encode("utf-8")
        if len(data) > self.msg_max:
            self.plain(f"send: {len(data)} bytes exceeds the board's"
                       f" {self.msg_max}-byte message limit")
            return
        self.m.submit(data, QOS_INTERACTIVE)
        self.plain(f">> queued {len(data)} bytes (interactive)")

    def cmd_sendfile(self, path):
        if self.pending:
            self.plain("sendfile: a transfer is already in progress")
            return
        try:
            with open(path, "rb") as f:
                src = f.read(FILE_MAX_SRC + 1)
        except OSError as e:                         # noqa: BLE001
            self.plain(f"sendfile: cannot open {path}: {e}")
            return
        if len(src) > FILE_MAX_SRC:
            self.plain(f"sendfile: larger than the {FILE_MAX_SRC}-byte cap")
            return
        base = os.path.basename(path) or "unnamed"

        body, magic = src, FILE_MAGIC_W
        if self.compress and src:
            z = zlib.compress(src, 9)
            if len(z) < len(src):
                body, magic = z, FILE_MAGIC_WZ

        head = 1 + len(FILE_TAG) + len(base.encode("utf-8")) + 1 + 4
        part_data = self.msg_max - head
        # align each part to the peer's streamed window (one ack per
        # part): a part is one station message, fragmented at 200 B
        win = (self.status or {}).get("peer_win_max", 0) \
            if (self.status or {}).get("peer_state", 0) >= 2 else 0
        target = (win or 8) * 200 - head
        if 0 < target < part_data:
            part_data = target
        if part_data <= 0:
            self.plain(f"sendfile: name too long for a {self.msg_max}-byte"
                       " message")
            return
        n_parts = max(1, (len(body) + part_data - 1) // part_data)
        if n_parts > 65535:
            self.plain(f"sendfile: {len(body)} bytes on air needs {n_parts}"
                       " parts, but the envelope allows 65535")
            return

        prefix = bytes([magic]) + FILE_TAG + base.encode("utf-8") + b"\0"
        self.pending = [
            prefix + p.to_bytes(2, "little") + n_parts.to_bytes(2, "little")
            + body[p * part_data:(p + 1) * part_data]
            for p in range(n_parts)
        ]
        self.pending_name = base
        self.sent_parts = 0
        ratio = f" -> {len(body)} on air, {len(src) / len(body):.2f}x" \
            if magic == FILE_MAGIC_WZ else ""
        self.plain(f">> file '{base}' ({len(src)} bytes{ratio}),"
                   f" {n_parts} part(s) of <= {part_data} B, pacing against"
                   " the board's queue")
        if self.status:
            self.pump_file(self.status)

    def cmd_bulk(self, n):
        if n > self.msg_max:
            self.plain(f"bulk: capped at the board's {self.msg_max} bytes")
            return
        self.m.submit(bytes((i & 0xFF) for i in range(n)), QOS_BULK)
        self.plain(f">> queued {n}-byte test pattern (bulk)")

    def cmd_bcast(self, text):
        # `-r N` pins the rung; without it (0xFF) the board uses the rung
        # it would send the peer a frame at -- the one rung the peer is
        # certainly listening on -- and logs which that was.
        rung = 0xFF
        if text.startswith("-r "):
            part = text[3:].split(None, 1)
            try:
                v = int(part[0])
            except (ValueError, IndexError):
                v = -1
            if 0 <= v <= 12:
                rung = v
                text = part[1] if len(part) > 1 else ""
        data = text.encode("utf-8")
        if not data:
            self.plain("bcast: nothing to send")
            return
        if len(data) > 1022:
            self.plain("bcast: over the 1022-byte broadcast cap")
            return
        # UP_CMD_BCAST = 0x06: ptype (0 = telemetry/text), then the rung
        self.m.t.write(__import__("ofdm_modem").encode(
            0x06, bytes([0, rung]) + data))
        at = "the link's own rung" if rung == 0xFF else f"rung {rung}"
        self.plain(f">> broadcast queued, {len(data)} bytes at {at} "
                   f"(non-ARQ)")

    def on_bcast(self, pl):
        if not pl:
            return
        if pl[0] & 0x80:
            self._bc_buf = bytearray()
            self._bc_ptype = pl[0] & 0x0F
            self.say(f"<< broadcast starting (ptype {self._bc_ptype}"
                     f"{', storing' if self._bc_ptype == 15 else ''})")
        elif pl[0] & 0x40:
            fo = int.from_bytes(pl[1:3], "little") if len(pl) >= 5 else 0
            lo = int.from_bytes(pl[3:5], "little") if len(pl) >= 5 else 0
            buf = getattr(self, "_bc_buf", b"")
            if getattr(self, "_bc_ptype", 0) == 15:   # opaque: a file
                with open("rx_broadcast.bin", "wb") as f:
                    f.write(buf)
                self.say(f"<< broadcast ended: {fo} frame(s), {lo} lost -- "
                         f"stored {len(buf)} bytes as rx_broadcast.bin "
                         "(gaps are NOT repaired)")
            else:
                text = buf.decode("utf-8", "replace")
                self.say(f"<< broadcast ended: {fo} frame(s), {lo} lost -- "
                         f'"{text}" (gaps are NOT repaired)')
        else:
            if not hasattr(self, "_bc_buf"):
                self._bc_buf = bytearray()
            self._bc_buf += pl[1:]

    def cmd_bcastfile(self, rest):
        rung = 0xFF
        if rest.startswith("-r "):
            part = rest[3:].split(None, 1)
            try:
                v = int(part[0])
            except (ValueError, IndexError):
                v = -1
            if 0 <= v <= 12:
                rung = v
                rest = part[1] if len(part) > 1 else ""
        try:
            with open(rest, "rb") as f:
                data = f.read(262145)
        except OSError as e:                          # noqa: BLE001
            self.plain(f"bcastfile: cannot open {rest}: {e}")
            return
        if not data or len(data) > 262144:
            self.plain("bcastfile: empty, or over the 256 kB cap")
            return
        self._ubcf = data
        self._ubcf_off = 0
        self._ubcf_rung = rung
        self.plain(f">> broadcasting {rest} ({len(data)} bytes, raw, "
                   "no delivery guarantee)")
        self.pump_bcfile()

    def pump_bcfile(self):
        # chunk toward the board against the bc_free it reports; bit 7
        # of the ptype byte = more follows, bit 6 = continuation
        CHUNK = 1024
        off = getattr(self, "_ubcf_off", -1)
        if off < 0:
            return
        free = (self.status or {}).get("bc_free", 0)
        enc = __import__("ofdm_modem").encode
        while off < len(self._ubcf) and free >= 2 * CHUNK:
            n = min(CHUNK, len(self._ubcf) - off)
            more = off + n < len(self._ubcf)
            head = 0x0F | (0x80 if more else 0) | (0x40 if off else 0)
            self.m.t.write(enc(0x06, bytes([head, self._ubcf_rung])
                               + self._ubcf[off:off + n]))
            off += n
            free -= n
            if not more:
                self.plain(f"broadcastfile: all {len(self._ubcf)} bytes "
                           "handed to the board")
                off = -1
                break
        self._ubcf_off = off

    def cmd_status(self):
        st = self.status
        if not st:
            self.plain("no status yet -- the board pushes one every 0.5 s")
            return
        q = st["queues"]
        self.plain(f"rung {st['rung']}  SNR {st['snr_db']:+.1f} dB  "
                   f"{'BUSY' if st['busy'] else 'idle'}"
                   f"{'  pending-ack' if st['pending'] else ''}")
        self.plain(f"queues: ctl {q[0]}  inter {q[1]}  bulk {q[2]}"
                   f"  (board holds {BOARD_QUEUE} per queue)")
        ps = st.get("peer_state", 0)
        if ps >= 2:
            mr = st.get("peer_max_rung1", 0)
            self.plain(f"peer: {CAPS_NAMES(st['peer_caps'])}messages up to "
                       f"{st['peer_msg_max']} B, window {st['peer_win_max']},"
                       f" rung ceiling {mr - 1 if mr else 'unspecified'}"
                       + (" (handshake complete)" if ps == 3
                          else " (awaiting our confirmation)"))
        else:
            self.plain("peer: " + ("did not answer the capability probe -- "
                                   "legacy defaults" if ps == 1 else
                                   "capabilities unknown (asked on the "
                                   "first bulk transfer)"))

    def cmd_stats(self):
        st = self.status or {}
        self.plain(f"tx {st.get('tx', '?')}  rx {st.get('rx', '?')}  "
                   f"timeouts {st.get('timeouts', '?')}  "
                   f"retx {st.get('retransmissions', '?')}")
        self.plain(f"host: {self.rx_msgs} message(s), {self.rx_files} file(s)"
                   f" received, {self.refused} submit refusal(s)")
        if self.pending:
            self.plain(f"sending '{self.pending_name}':"
                       f" {self.sent_parts}/{len(self.pending)} parts handed"
                       " over")

    def command(self, line):
        line = line.strip()
        if not line:
            return True
        cmd, _, rest = line.partition(" ")
        rest = rest.strip()
        if cmd in ("quit", "exit"):
            return False
        elif cmd == "send" and rest:
            self.cmd_send(rest)
        elif cmd == "sendfile" and rest:
            self.cmd_sendfile(rest)
        elif cmd == "bulk" and rest.isdigit():
            self.cmd_bulk(int(rest))
        elif cmd == "bcast" and rest:
            self.cmd_bcast(rest)
        elif cmd == "bcastfile" and rest:
            self.cmd_bcastfile(rest)
        elif cmd == "config" and not rest:
            # no arguments: ask the board (the settings live there)
            self.m.t.write(__import__("ofdm_modem").encode(0x03))
        elif cmd == "config" and rest:
            k, _, v = rest.partition(" ")
            try:
                self.m.config(k, int(v))
                self.plain(f">> config {k} = {int(v)}")
            except (KeyError, ValueError) as e:            # noqa: BLE001
                self.plain(f"config: {e}  (keys: rung_ceiling, burst_window,"
                           " burst_stream, freq_trim_mhz, audio_tap, anchor,"
                           " diag_stream, win_max)")
        elif cmd == "debug":
            # the events come from the BOARD, whose diag stream is off
            # by default; this is `config diag_stream` with a memorable
            # name, the same command app.c has
            on = {"on": 1, "off": 0}.get(rest, None)
            if on is None:
                on = 0 if getattr(self, "_debug", 0) else 1
            self._debug = on
            self.m.config("diag_stream", on)
            self.plain(f"diag {'ON' if on else 'OFF'}: the board's event "
                       f"stream is {'on' if on else 'off'}")
        elif cmd == "status":
            self.cmd_status()
        elif cmd == "stats":
            self.cmd_stats()
        elif cmd == "help":
            self.plain("send <text> | sendfile <path> | bulk <n> | "
                       "bcast [-r <rung>] <text> | "
                       "bcastfile [-r <rung>] <path> | "
                       "config [<key> <val>] | debug [on|off] | status | "
                       "stats | quit")
        else:
            self.plain(f"unknown command '{line}' -- try help")
        return True


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true",
                    help="list attached boards with their serials and exit")
    ap.add_argument("--serial",
                    help="board to attach to (see --list). Optional when "
                         "exactly one board is present.")
    ap.add_argument("--name", default=None,
                    help="label for this console (default: last 4 of serial)")
    ap.add_argument("--msg-max", type=int, default=BOARD_MSG_MAX,
                    help=f"board's ST_MSG_MAX (default {BOARD_MSG_MAX}, the "
                         "cport default; not discoverable over the wire)")
    ap.add_argument("--no-compress", action="store_true",
                    help="send files without DEFLATE")
    args = ap.parse_args()

    if args.list:
        return list_boards()

    try:
        modem = OfdmModem(serial=args.serial)
    except Exception as e:                           # noqa: BLE001
        print(f"error: {e}", file=sys.stderr)
        return 1

    with modem:
        info = modem.info()
        if not info:
            print("error: board did not answer CMD_INFO", file=sys.stderr)
            return 1
        serial = info["serial"]
        # first 4, not last 4: the UID tail is the wafer/lot ID and is
        # identical for chips from one wafer (both boards end ...3436)
        name = args.name or serial[:4]
        # the board's limit comes from INFO; --msg-max only overrides it
        msg_max = args.msg_max if args.msg_max != BOARD_MSG_MAX \
            else (info or {}).get("msg_max", BOARD_MSG_MAX)
        con = Console(modem, name, msg_max, not args.no_compress)
        stale = getattr(modem.t, "stale", 0)
        if stale:
            con.plain(f"drained {stale} stale bytes from a previous session")
        con.plain(f"attached to board {serial}")
        con.plain(f"  proto v{info['proto_ver']}  fw {info['fw']}  "
                  f"{info['n_modes']} modes @{info['sample_rate']} Hz")
        con.plain("the station runs ON THE BOARD -- this is a terminal onto "
                  "it. 'help' for commands.")
        print("> ", end="", flush=True)

        last_ping = 0.0
        try:
            while True:
                # one ping a second keeps the board's "host attached"
                # state (and its LED) alive -- closing this program does
                # not unmount the device, so the board cannot tell
                now = time.monotonic()
                if now - last_ping >= 1.0:
                    modem.t.write(__import__("ofdm_modem").encode(
                        0x04, int(now).to_bytes(4, "little")))
                    last_ping = now
                # events() yields the SHORT names from TYPE_NAME
                # ("status", not "EVT_STATUS"), and poke=False because a
                # ping per loop returns a pong per loop -- measured at
                # 60329 of them in 8 s, which buries everything else.
                for kind, payload in modem.events(timeout=0.2, poke=False):
                    if kind == "message":
                        con.on_message(payload["qos"], payload["data"])
                    elif kind == "status" and payload:
                        con.on_status(payload)
                    elif kind == "log":
                        con.on_log(payload)
                    elif kind == "diag" and payload:
                        con.on_diag(payload)
                    elif kind == "0x88":
                        con.on_bcast(payload)
                if select.select([sys.stdin], [], [], 0)[0]:
                    line = sys.stdin.readline()
                    if not line:
                        break
                    if not con.command(line):
                        break
                    print("> ", end="", flush=True)
        except KeyboardInterrupt:
            print()
        con.rx.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
