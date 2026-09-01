#!/usr/bin/env python3
"""KISS TNC bridge: existing packet-radio software onto this modem.

    ./kiss_bridge.py --tcp 8001              # Dire Wolf-style TCP KISS
    ./kiss_bridge.py --pty                   # prints a /dev/pts/N for kissattach
    ./kiss_bridge.py --serial <uid> --tcp 8001
    ./kiss_bridge.py --emulate ../cport/build/usb_modem_emu   # no hardware

Why this is a host program and not firmware
-------------------------------------------
KISS is a host-to-TNC protocol, and this board already speaks a richer
one (usb-protocol.md): rung, SNR, peer capabilities, die temperature,
broadcast pacing, a diagnostic stream.  Putting KISS in the firmware
would duplicate that, add a fourth wire format to keep in step across
the C, Python and board twins, and -- worst -- present the board as a
dumb TNC, discarding the adaptation that is the point of the project.
So the translation lives here, costs the firmware nothing, and can be
tested without hardware.

What it maps
------------
A KISS data frame is one AX.25 frame.  Two modes, both preserving frame
boundaries exactly:

  message   (default)  frame <-> station message (SUBMIT / EVT_MESSAGE).
                       Point-to-point, ARQ'd, the rate ladder adapts.
  broadcast            frame <-> one non-ARQ broadcast (UP_CMD_BCAST,
                       ptype OPAQUE).  Connectionless, like AX.25 UI:
                       every listener gets it, nothing is repeated.

Deliberately NOT supported: AX.25 connected mode over the station's own
ARQ.  Two retransmission engines with independent timers, one adapting
the rung underneath the other, is a layering accident, not a feature.
Run UI frames (APRS and friends) and let this link do the reliability.

The paclen rule
---------------
Air time is the constraint, not bytes.  A 256-byte frame costs 2.6 s at
rung 12, 5.1 s at rung 8, 18.4 s at rung 4 and 278.6 s at rung 0 -- past
every carrier-sense constant the station has.  The bridge therefore
refuses a frame whose air time at the CURRENT rung exceeds --max-air
(45 s, the station's own fragment ceiling) and says so, rather than
keying the radio for four minutes.  paclen 200 is the sweet spot: a
full AX.25 frame is then ~216 bytes, inside the 255-byte single-frame
payload cap, and costs 3.0 s at rung 10.
"""
import argparse
import os
import select
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ofdm_modem import (OfdmModem, encode, USBTimeoutError)  # noqa: E402

# Air time per rung, WITHOUT importing the model: a bridge is a thin host
# tool and must run in a venv that has pyusb and nothing else -- importing
# ofdm_phy.station pulls in NumPy and SciPy, which is how the first
# version of this file failed to start at all.
#
# AIR0 is the fixed preamble+header cost and SLOPE the seconds per payload
# byte, both generated from ofdm_phy.station.estimate_air_time, which
# stays the canonical implementation. The exact function is a step (whole
# OFDM symbols); this linear form tracks it within +0.10/-0.35 s, so
# MARGIN keeps the estimate on the safe side -- it must never come out
# OPTIMISTIC, or a frame would be admitted that the channel cannot carry.
# test_kiss.py asserts both properties against the model across every
# rung and every length.
AIR0 = [15.4000, 3.8800, 3.5333, 3.1867, 1.0000, 0.9093, 0.8187,
        0.7733, 0.7280, 0.7280, 0.7280, 0.6827, 0.6827]
SLOPE = [1.028000, 0.260000, 0.173333, 0.130000, 0.068000, 0.045333,
         0.034000, 0.022667, 0.017000, 0.015141, 0.011333, 0.008523,
         0.007571]
MARGIN = 0.35


def estimate_air_time(rung: int, payload_len: int) -> float:
    """Seconds of air for one frame at this rung -- never optimistic."""
    r = 0 if rung < 0 else min(rung, len(AIR0) - 1)
    return AIR0[r] + SLOPE[r] * payload_len + MARGIN

FEND, FESC, TFEND, TFESC = 0xC0, 0xDB, 0xDC, 0xDD

# KISS command codes (low nibble of the type byte)
CMD_DATA, CMD_TXDELAY, CMD_P, CMD_SLOT = 0, 1, 2, 3
CMD_TXTAIL, CMD_DUPLEX, CMD_SETHW, CMD_RETURN = 4, 5, 6, 0xFF

UP_CMD_BCAST = 0x06
BC_PT_OPAQUE = 0x0F
BC_START, BC_EOS = 0x80, 0x40


def kiss_encode(payload: bytes, cmd: int = CMD_DATA, port: int = 0) -> bytes:
    """One KISS frame. FEND-delimited, FEND/FESC byte-stuffed."""
    out = bytearray([FEND, ((port & 15) << 4) | (cmd & 15)])
    for b in payload:
        if b == FEND:
            out += bytes([FESC, TFEND])
        elif b == FESC:
            out += bytes([FESC, TFESC])
        else:
            out.append(b)
    out.append(FEND)
    return bytes(out)


class KissDecoder:
    """Byte stream in, (port, cmd, payload) out.

    Tolerates the usual real-world sloppiness: repeated FENDs, leading
    garbage before the first FEND, and empty frames (a bare FEND FEND is
    how several stacks keep the line alive).
    """

    def __init__(self):
        self.buf = bytearray()
        self.in_frame = False
        self.escape = False

    def push(self, data: bytes):
        out = []
        for b in data:
            if b == FEND:
                if self.in_frame and self.buf:
                    out.append((self.buf[0] >> 4, self.buf[0] & 15,
                                bytes(self.buf[1:])))
                self.buf.clear()
                self.in_frame = True
                self.escape = False
                continue
            if not self.in_frame:
                continue                      # garbage before the first FEND
            if self.escape:
                self.buf.append(FEND if b == TFEND
                                else FESC if b == TFESC else b)
                self.escape = False
            elif b == FESC:
                self.escape = True
            else:
                self.buf.append(b)
        return out


class Bridge:
    def __init__(self, modem, args):
        self.m = modem
        self.args = args
        self.clients = {}                  # fd -> (sock_or_fd, KissDecoder)
        self.status = {}
        self.msg_max = 256
        self.bc_buf = None                 # broadcast reassembly, rx side
        self.sent = self.rcvd = self.refused = 0

    # -- host side ------------------------------------------------------
    def add_client(self, obj):
        self.clients[self._fd(obj)] = (obj, KissDecoder())

    @staticmethod
    def _fd(obj):
        return obj.fileno() if hasattr(obj, "fileno") else obj

    def _write_client(self, obj, data):
        try:
            if hasattr(obj, "sendall"):
                obj.sendall(data)
            else:
                os.write(obj, data)
        except OSError:
            return False
        return True

    def to_hosts(self, frame: bytes):
        """A frame off the air, up to every attached client."""
        wire = kiss_encode(frame)
        for fd, (obj, _) in list(self.clients.items()):
            if not self._write_client(obj, wire):
                self.drop_client(fd)
        self.rcvd += 1
        self.log(f"rx {len(frame)} B -> host")

    def drop_client(self, fd):
        obj, _ = self.clients.pop(fd, (None, None))
        if obj is not None and hasattr(obj, "close"):
            try:
                obj.close()
            except OSError:
                pass

    # -- air side -------------------------------------------------------
    def rung(self):
        """The rung the next frame would use; -1 when not yet known."""
        r = self.status.get("rung_now")
        if r is None:                       # older firmware: last transmitted
            r = self.status.get("rung", -1)
        return r

    def air_time(self, n):
        r = self.rung()
        return estimate_air_time(r if r >= 0 else 0, n)

    def from_host(self, port, cmd, payload):
        if cmd != CMD_DATA:
            # TXDELAY/P/SlotTime/TXtail/FullDuplex/SetHardware: a smart
            # TNC owns its own timing, and this one's carrier sense and
            # turnaround are measured, not configured from the host.
            names = {CMD_TXDELAY: "TXDELAY", CMD_P: "P", CMD_SLOT: "SlotTime",
                     CMD_TXTAIL: "TXtail", CMD_DUPLEX: "FullDuplex",
                     CMD_SETHW: "SetHardware", CMD_RETURN: "Return"}
            self.log(f"ignoring KISS {names.get(cmd, cmd)} "
                     f"({'exit' if cmd == CMD_RETURN else 'we own our timing'})")
            return
        if port != 0:
            self.log(f"frame for port {port} dropped -- one port only")
            return
        if not payload:
            return
        if len(payload) > self.msg_max:
            self.refused += 1
            self.log(f"frame of {len(payload)} B refused: over the board's "
                     f"{self.msg_max}-byte limit")
            return
        air = self.air_time(len(payload))
        if air > self.args.max_air:
            self.refused += 1
            self.log(f"frame of {len(payload)} B refused: {air:.0f}s of air "
                     f"at rung {self.rung()} (limit {self.args.max_air:.0f}s)"
                     f" -- lower paclen or wait for the ladder")
            return
        try:
            if self.args.mode == "broadcast":
                # one frame = one non-ARQ broadcast; bits 7/6 clear = a
                # complete one-shot, rung 0xFF = let the board negotiate
                self.m.t.write(encode(UP_CMD_BCAST,
                                      bytes([BC_PT_OPAQUE, 0xFF]) + payload))
            else:
                self.m.submit(payload, qos=self.args.qos)
        except USBTimeoutError:
            self.refused += 1
            self.log("frame dropped: the board did not accept the write "
                     "(busy decoding); the host will retry if it cares")
            return
        self.sent += 1
        self.log(f"tx {len(payload)} B, {air:.1f}s at rung {self.rung()}")

    def on_bcast(self, payload: bytes):
        """Reassemble a received broadcast into exactly one frame."""
        if not payload:
            return
        flags = payload[0]
        if flags & BC_START:
            self.bc_buf = bytearray()
        elif flags & BC_EOS:
            if self.bc_buf is not None:
                self.to_hosts(bytes(self.bc_buf))
            self.bc_buf = None
        elif self.bc_buf is not None:
            self.bc_buf += payload[1:]

    # -- plumbing -------------------------------------------------------
    def log(self, msg):
        if self.args.verbose:
            print(f"{time.strftime('%H:%M:%S')} [kiss] {msg}", flush=True)

    def pump_modem(self):
        for kind, payload in self.m.events(timeout=0.05, poke=False):
            if kind == "status" and payload:
                self.status = payload
                mm = payload.get("peer_msg_max") or 0
                if mm:
                    self.msg_max = min(self.msg_max, mm)
            elif kind == "message" and self.args.mode == "message":
                self.to_hosts(payload["data"])
            elif kind == "0x88" and self.args.mode == "broadcast":
                self.on_bcast(payload)
            elif kind == "log":
                self.log(f"board: {payload}")

    def run(self, listener):
        info = self.m.info()
        if info:
            self.msg_max = info.get("msg_max") or 256
            print(f"[kiss] board fw {info['fw']}, messages up to "
                  f"{self.msg_max} B, mode {self.args.mode}", flush=True)
        last_ping = 0.0
        while True:
            now = time.monotonic()
            if now - last_ping >= 1.0:
                # keeps the board's host-attached indication (and LED)
                # alive; a dropped ping is not fatal, the board can stall
                # USB for as long as its worst decode
                try:
                    self.m.t.write(encode(0x04, int(now).to_bytes(4, "little")))
                except USBTimeoutError:
                    pass
                last_ping = now
            fds = list(self.clients)
            if listener is not None:
                fds.append(listener.fileno())
            r, _, _ = select.select(fds, [], [], 0.1) if fds else ([], [], [])
            for fd in r:
                if listener is not None and fd == listener.fileno():
                    sock, addr = listener.accept()
                    self.add_client(sock)
                    self.log(f"client {addr[0]}:{addr[1]} attached")
                    continue
                obj, dec = self.clients[fd]
                try:
                    data = (obj.recv(4096) if hasattr(obj, "recv")
                            else os.read(fd, 4096))
                except OSError:
                    data = b""
                if not data and hasattr(obj, "recv"):
                    self.log("client detached")
                    self.drop_client(fd)
                    continue
                for port, cmd, payload in dec.push(data):
                    self.from_host(port, cmd, payload)
            self.pump_modem()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--serial", help="board UID (omit if only one attached)")
    ap.add_argument("--emulate", help="run the emulator binary instead")
    ap.add_argument("--tcp", type=int, default=8001,
                    help="listen for KISS-over-TCP clients (default 8001)")
    ap.add_argument("--pty", action="store_true",
                    help="present a pty for kissattach instead of TCP")
    ap.add_argument("--mode", choices=("message", "broadcast"),
                    default="message")
    ap.add_argument("--qos", type=int, default=2, choices=(0, 1, 2),
                    help="0 control, 1 interactive, 2 bulk (message mode)")
    ap.add_argument("--max-air", type=float, default=45.0,
                    help="refuse frames longer than this many seconds of air")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    try:
        modem = (OfdmModem(emulate=args.emulate.split()) if args.emulate
                 else OfdmModem(serial=args.serial))
    except RuntimeError as e:
        # no board, no pyusb, no udev rule: all install-shaped problems
        # with something useful to say. A traceback says none of it.
        print(f"[kiss] {e}", file=sys.stderr)
        return 1
    bridge = Bridge(modem, args)

    listener = None
    if args.pty:
        master, slave = os.openpty()
        os.set_blocking(master, False)
        print(f"[kiss] KISS pty ready: {os.ttyname(slave)}\n"
              f"       e.g. sudo kissattach {os.ttyname(slave)} radio",
              flush=True)
        bridge.add_client(master)
    else:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", args.tcp))
        listener.listen(4)
        print(f"[kiss] KISS/TCP on 127.0.0.1:{args.tcp}", flush=True)

    try:
        bridge.run(listener)
    except KeyboardInterrupt:
        print(f"\n[kiss] {bridge.sent} frame(s) sent, {bridge.rcvd} received, "
              f"{bridge.refused} refused", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
