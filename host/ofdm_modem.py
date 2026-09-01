#!/usr/bin/env python3
"""Host driver for the OFDM modem USB device.

Why this exists rather than a serial port: a USB-UART bridge enumerates
as one more /dev/ttyACM* or /dev/ttyUSB* among however many adapters are
plugged in, numbered by enumeration order, so the host has to guess which
one is the modem -- and guesses wrong after a reboot or a re-plug. This
opens the device by IDENTITY instead: vendor and product ID, and a serial
string derived from the STM32's 96-bit unique ID, so two modems on one
machine are always distinguishable.

    from ofdm_modem import OfdmModem
    with OfdmModem() as m:                    # or OfdmModem(serial="24...")
        print(m.info())
        m.submit(b"HELLO", qos=2)
        for ev in m.events(timeout=5):
            print(ev)

Two transports:
  usb    -- pyusb + libusb, the real device
  pipe   -- a subprocess speaking the same wire protocol, which is how
            this is tested without hardware (cport/bench/usb_modem_emu.c)

The framing is implemented once, here, and matches cport/src/usb_proto.c
byte for byte; the emulator runs the actual device-side C, so a passing
round trip exercises both ends of the real protocol.
"""

import argparse
import errno
import select
import struct
import subprocess
import sys
import threading
import time
from collections import deque

VID = 0x1209          # pid.codes
PID = 0x0001          # TEST pid -- see cport/src/usb_desc.h
EP_OUT, EP_IN = 0x01, 0x81

SYNC = b"\xA5\x5A"
HDR = 5
# UP_MAX_PAYLOAD in cport/src/usb_proto.h -- sized so one whole station
# message (ST_MSG_MAX 3328) plus its qos byte crosses in a single frame.
# It MUST track that header: this constant was left at the protocol's
# original 1024 when the message size grew, and the parser drops any
# frame declaring more as garbage. The device stayed healthy and the
# Python console simply never saw a file part (3185-byte EVT_MESSAGE),
# while encode() refused to send one -- measured, both ends of a
# transfer the C console completed fine.
MAX_PAYLOAD = 3336

CMD_INFO, CMD_SUBMIT, CMD_CONFIG, CMD_PING, CMD_RESET = 1, 2, 3, 4, 5
RSP_INFO, EVT_MESSAGE, EVT_STATUS, EVT_DIAG, RSP_PONG, EVT_LOG, EVT_AUDIO = (
    0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87)

CFG = {"rung_ceiling": 1, "burst_window": 2, "burst_stream": 3,
       "freq_trim_mhz": 4, "audio_tap": 5, "anchor": 6,
       "diag_stream": 7, "win_max": 8}

TYPE_NAME = {RSP_INFO: "info", EVT_MESSAGE: "message", EVT_STATUS: "status",
             EVT_DIAG: "diag", RSP_PONG: "pong", EVT_LOG: "log",
             EVT_AUDIO: "audio"}


def encode(type_, payload=b""):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload {len(payload)} > {MAX_PAYLOAD}")
    return SYNC + bytes([type_]) + struct.pack("<H", len(payload)) + payload


class Parser:
    """Byte stream in, whole frames out -- the twin of up_parser_t.

    Bulk transfers preserve boundaries only up to the endpoint size and
    a host read may coalesce several, so frames must be self-delimiting
    rather than assumed to arrive one per read.
    """

    def __init__(self):
        self.buf = bytearray()
        self.resyncs = 0

    def push(self, data):
        self.buf += data
        out = []
        while True:
            if len(self.buf) >= 1 and self.buf[0] != SYNC[0]:
                del self.buf[0]; self.resyncs += 1; continue
            if len(self.buf) >= 2 and self.buf[1] != SYNC[1]:
                del self.buf[0]; self.resyncs += 1; continue
            if len(self.buf) < HDR:
                return out
            n = struct.unpack_from("<H", self.buf, 3)[0]
            if n > MAX_PAYLOAD:
                del self.buf[0]; self.resyncs += 1; continue
            if len(self.buf) < HDR + n:
                return out
            out.append((self.buf[2], bytes(self.buf[HDR:HDR + n])))
            del self.buf[:HDR + n]


def decode_info(p):
    if len(p) < 24:
        return None
    ver, nmodes, fw = p[0], p[1], struct.unpack_from("<H", p, 2)[0]
    uid = p[4:16]
    caps, rate = struct.unpack_from("<II", p, 16)
    # msg_max: the station's ST_MSG_MAX; an older firmware omits it
    msg_max = struct.unpack_from("<H", p, 24)[0] if len(p) >= 26 else 256
    return {"proto_ver": ver, "n_modes": nmodes,
            "fw": f"{fw >> 8}.{fw & 0xFF}",
            "serial": uid.hex().upper(), "caps": caps, "sample_rate": rate,
            "msg_max": msg_max or 256}


def decode_status(p):
    if len(p) < 32:
        return None
    rung, snr_q8, tx, rx, to, rt = struct.unpack_from("<iiIIII", p, 0)
    qc, qi, qb = struct.unpack_from("<HHH", p, 24)
    d = {"rung": rung, "snr_db": snr_q8 / 256.0, "tx": tx, "rx": rx,
         "timeouts": to, "retransmissions": rt,
         "queues": (qc, qi, qb), "busy": bool(p[30]),
         "pending": bool(p[31]),
         # the peer's declared capabilities (capability handshake):
         # peer_state 0 unknown, 1 legacy, 2 held, 3 held + confirmed
         "peer_state": 0, "peer_caps": 0, "peer_msg_max": 0,
         "peer_win_max": 0,
         # free bytes in the board's broadcast source buffer -- the
         # chunk-pacing signal a broadcastfile feeds against. It was
         # never decoded here, so board_console.py's pacing loop read
         # a default of 0 and the command fed nothing, forever.
         "bc_free": 0,
         # die temperature in C, or None when the board has no reading
         # (older firmware, emulator, a part whose calibration words
         # did not check out). None, not a number: 0 is a plausible
         # temperature and must not stand in for "no sensor".
         "temp_c": None, "rung_now": None}
    if len(p) >= 40:
        d["peer_state"], d["peer_caps"] = p[32], p[33]
        d["peer_msg_max"] = struct.unpack_from("<H", p, 34)[0]
        d["peer_win_max"] = p[36]
        d["peer_max_rung1"] = p[37]
        d["bc_free"] = struct.unpack_from("<H", p, 38)[0]
    if len(p) >= 42:
        q8 = struct.unpack_from("<h", p, 40)[0]
        d["temp_c"] = None if q8 == -32768 else q8 / 256.0
    if len(p) >= 43:
        # the rung the NEXT frame would use, as opposed to "rung" above
        # which is the last one transmitted. -1 = none yet; None here
        # means the firmware predates the field.
        d["rung_now"] = struct.unpack_from("<b", p, 42)[0]
    return d


class _PipeTransport:
    """The emulator, or any process speaking the protocol on stdio."""

    def __init__(self, argv):
        self.p = subprocess.Popen(argv, stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE)
        self.desc = f"pipe:{' '.join(argv)}"

    def write(self, data):
        self.p.stdin.write(data)
        self.p.stdin.flush()

    def read(self, timeout):
        # select first: read1() on a pipe blocks forever when the device
        # has nothing to say, and a device with nothing to say is the
        # normal case for an event stream
        r, _, _ = select.select([self.p.stdout], [], [], timeout)
        return self.p.stdout.read1(4096) if r else b""

    def close(self):
        try:
            self.p.stdin.close()
            self.p.wait(timeout=2)
        except Exception:
            self.p.kill()


# The device can stop servicing USB for as long as its worst blocking
# receive burst -- 2283 ms measured on the part, the end-of-frame commit
# that sizes the capture FIFO. A write timeout below that turns a healthy
# board that happens to be mid-decode into a hard error: at 1000 ms it
# killed the console out of its once-a-second heartbeat, with a traceback,
# mid-transfer. pyusb cannot report how much of a timed-out transfer
# reached the device, so there is no safe retry at this layer -- a repeat
# of a partial frame would desync the device's parser. The fix is a
# timeout that clears the stall.
WRITE_TIMEOUT_MS = 5000

try:                       # pyusb is optional -- the emulator needs none
    from usb.core import USBTimeoutError
except ImportError:        # pragma: no cover - exercised without pyusb
    class USBTimeoutError(Exception):
        pass


class _UsbTransport:
    def __init__(self, serial=None):
        # imported late: only this path needs it, so the emulator and the
        # framing helpers work without pyusb. A missing pyusb is an
        # install problem, not a bug -- say so instead of unwinding a
        # ModuleNotFoundError out of a tool the user just started.
        try:
            import usb.core, usb.util
        except ImportError:
            raise RuntimeError(
                "pyusb is not installed in this interpreter. It is in "
                "requirements.txt:\n"
                "    ./venv/bin/pip install -r requirements.txt\n"
                "or run against the hardware-free emulator instead "
                "(--emulate).") from None
        matches = list(usb.core.find(find_all=True, idVendor=VID,
                                     idProduct=PID))
        if not matches:
            raise RuntimeError(
                f"no OFDM modem on the bus ({VID:04x}:{PID:04x}). "
                "Check the cable, and that the udev rule in "
                "host/99-ofdm-modem.rules is installed.")
        if serial:
            matches = [d for d in matches
                       if usb.util.get_string(d, d.iSerialNumber) == serial]
            if not matches:
                raise RuntimeError(f"no modem with serial {serial}")
        if len(matches) > 1:
            have = [usb.util.get_string(d, d.iSerialNumber) for d in matches]
            raise RuntimeError(
                "several modems attached; choose one with --serial "
                "(or serial= from Python). "
                f"Found: {have}")
        self.dev = matches[0]
        self.dev.set_configuration()
        usb.util.claim_interface(self.dev, 0)
        # Recover the OUT endpoint from a previous session that died
        # mid-transfer. Empirically harmless here: three consecutive
        # opens against an armed OUT endpoint all worked.
        try:
            self.dev.clear_halt(EP_OUT)
        except Exception:
            pass              # not halted, or the device disallows it
        # Deliberately NOT clear_halt(EP_IN). The device pushes status
        # unprompted, so EP_IN is usually ARMED with a frame when we
        # open. TinyUSB's usbd_edpt_clear_stall drops its software BUSY
        # flag unconditionally (usbd.c, "long-standing behavior") while
        # the dwc2 dcd_edpt_clear_stall never disarms the hardware --
        # so the next flush re-arms on top of a live transfer and the
        # endpoint wedges after exactly one packet. Measured twice:
        # first open works only if the device stayed quiet, every later
        # open fails, isr_count climbs into the tens of millions on a
        # level-triggered TX-FIFO-empty interrupt. Consuming the armed
        # transfer through the normal path is the safe recovery.
        self.serial = usb.util.get_string(self.dev, self.dev.iSerialNumber)
        self.desc = f"usb:{VID:04x}:{PID:04x}/{self.serial}"
        self.stale = self._drain()

    def _drain(self, quiet_s=0.15, max_s=2.0):
        """Read EP_IN until it goes quiet; returns bytes discarded.

        The device may have been pushing status frames to nobody since
        the last session closed, and whatever is armed comes out here
        rather than colliding with our first command. quiet_s must be
        shorter than the device's status period (0.5 s) or this never
        returns while the device is healthy.
        """
        import usb.core
        t0, n = time.time(), 0
        while time.time() - t0 < max_s:
            try:
                data = bytes(self.dev.read(EP_IN, 4096,
                                           timeout=int(quiet_s * 1000)))
            except usb.core.USBTimeoutError:
                break
            if not data:
                break
            n += len(data)
        return n

    def write(self, data):
        self.dev.write(EP_OUT, data, timeout=WRITE_TIMEOUT_MS)

    def read(self, timeout):
        import usb.core
        try:
            # Clamp to >=1 ms: libusb reads timeout=0 as "block forever",
            # and a deadline that has just expired computes to exactly
            # zero. The previous bug hid this one -- the exception
            # escaped before any caller could ask for a zero wait.
            ms = int(timeout * 1000)
            return bytes(self.dev.read(EP_IN, 4096,
                                       timeout=ms if ms > 0 else 1))
        except usb.core.USBTimeoutError:
            # A read timing out is the NORMAL case for an event stream:
            # the device has nothing to say. Caught by TYPE, not by
            # matching the message -- libusb renders it "Operation timed
            # out", which does not contain the substring "timeout", so a
            # text match silently let it escape and killed the session.
            return b""
        except usb.core.USBError as e:
            if e.errno == errno.ETIMEDOUT:
                return b""
            raise

    def close(self):
        import usb.util
        usb.util.release_interface(self.dev, 0)
        usb.util.dispose_resources(self.dev)


class OfdmModem:
    def __init__(self, serial=None, emulate=None):
        self.t = _PipeTransport(emulate) if emulate else _UsbTransport(serial)
        self.parser = Parser()
        self.q = deque()
        self.lock = threading.Lock()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    def close(self):
        self.t.close()

    # --- plumbing ---
    def _pump(self, timeout):
        data = self.t.read(timeout)
        if data:
            for f in self.parser.push(data):
                self.q.append(f)

    def _await(self, want, timeout):
        end = time.time() + timeout
        while time.time() < end:
            for i, (t, p) in enumerate(self.q):
                if t == want:
                    del self.q[i]
                    return p
            self._pump(min(0.25, max(0.0, end - time.time())))
        return None

    # --- API ---
    def info(self, timeout=2.0):
        self.t.write(encode(CMD_INFO))
        p = self._await(RSP_INFO, timeout)
        return decode_info(p) if p else None

    def ping(self, token=0xDEADBEEF, timeout=2.0):
        self.t.write(encode(CMD_PING, struct.pack("<I", token)))
        p = self._await(RSP_PONG, timeout)
        return p is not None and struct.unpack("<I", p)[0] == token

    def submit(self, data, qos=2):
        self.t.write(encode(CMD_SUBMIT, bytes([qos]) + data))

    def config(self, key, value):
        self.t.write(encode(CMD_CONFIG,
                            bytes([CFG[key]]) + struct.pack("<i", value)))

    def reset(self):
        self.t.write(encode(CMD_RESET))

    def events(self, timeout=1.0, poke=True):
        """Yield (name, payload) for whatever has arrived.

        `poke` sends a ping first. The real device pushes to its IN
        endpoint on its own schedule, but a pipe-backed emulator only
        runs when the host writes, so without this the emulator never
        gets an opportunity to stage anything.
        """
        if poke:
            self.t.write(encode(CMD_PING, struct.pack("<I", 0)))
        self._pump(timeout)
        while self.q:
            t, p = self.q.popleft()
            name = TYPE_NAME.get(t, f"0x{t:02x}")
            if t == EVT_STATUS:
                yield name, decode_status(p)
            elif t == EVT_MESSAGE:
                yield name, {"qos": p[0], "data": p[1:]}
            elif t == EVT_DIAG:
                ev = p[0]
                a, b, c, d, ms = struct.unpack_from("<iiiiI", p, 1)
                yield name, {"ev": ev, "a": a, "b": b, "c": c, "d": d,
                             "t_ms": ms}
            elif t == EVT_LOG:
                yield name, p.decode("utf-8", "replace")
            else:
                yield name, p


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--emulate", metavar="CMD",
                    help="run CMD as the device instead of opening USB")
    ap.add_argument("--serial", help="pick one modem by serial number")
    ap.add_argument("--send", help="submit a message, then listen")
    ap.add_argument("--listen", type=float, default=1.0)
    args = ap.parse_args()

    emu = args.emulate.split() if args.emulate else None
    with OfdmModem(serial=args.serial, emulate=emu) as m:
        print(f"transport: {m.t.desc}")
        if getattr(m.t, "stale", 0):
            print(f"drained   {m.t.stale} stale bytes left by a previous session")
        info = m.info()
        if not info:
            print("no response to CMD_INFO", file=sys.stderr)
            return 1
        print(f"modem     proto v{info['proto_ver']} fw {info['fw']} "
              f"{info['n_modes']} modes @{info['sample_rate']} Hz")
        print(f"serial    {info['serial']}")
        print(f"ping      {'ok' if m.ping() else 'NO REPLY'}")
        if args.send:
            m.submit(args.send.encode())
            print(f"submitted {len(args.send)} bytes")
        for name, payload in m.events(timeout=args.listen):
            print(f"  {name:8s} {payload}")
        print(f"resyncs   {m.parser.resyncs} (0 on a healthy link)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
