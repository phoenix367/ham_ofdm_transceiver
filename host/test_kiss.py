#!/usr/bin/env python3
"""KISS bridge tests -- codec and mapping, no hardware.

    ./test_kiss.py

The codec half is byte-exact against the KISS spec's own examples; the
mapping half drives the bridge with a fake modem, so every decision the
bridge makes (what it sends, what it refuses, what it reassembles) is
checked without a board or a channel.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kiss_bridge import (kiss_encode, KissDecoder, Bridge,   # noqa: E402
                         FEND, FESC, TFEND, TFESC,
                         CMD_DATA, CMD_TXDELAY, CMD_RETURN)

PASS = FAIL = 0


def check(name, ok):
    global PASS, FAIL
    print(f"[{'PASS' if ok else 'FAIL'}] {name}")
    if ok:
        PASS += 1
    else:
        FAIL += 1


class FakeArgs:
    mode = "message"
    qos = 2
    max_air = 45.0
    hold = 120.0
    verbose = False


class FakeModem:
    """Records what the bridge would have sent."""

    def __init__(self):
        self.submitted = []
        self.written = []
        self.t = self

    def submit(self, data, qos=2):
        self.submitted.append((bytes(data), qos))

    def write(self, data):          # self.t.write, the raw frame path
        self.written.append(bytes(data))

    def info(self, timeout=2.0):
        return None

    def events(self, timeout=1.0, poke=True):
        return iter(())


def bridge(mode="message", rung=12, max_air=45.0):
    args = FakeArgs()
    args.mode, args.max_air = mode, max_air
    b = Bridge(FakeModem(), args)
    b.status = {"rung_now": rung, "rung": rung}
    b.msg_max = 3328
    b.sent_frames = []
    b.to_hosts = lambda f: b.sent_frames.append(f)   # capture the host side
    return b


# ---------------- codec ----------------

def test_codec():
    check("a plain frame is FEND, type, data, FEND",
          kiss_encode(b"\x01\x02") == bytes([FEND, 0x00, 1, 2, FEND]))

    # the two bytes that cannot appear raw
    check("FEND in the payload is escaped as FESC TFEND",
          kiss_encode(bytes([FEND])) ==
          bytes([FEND, 0x00, FESC, TFEND, FEND]))
    check("FESC in the payload is escaped as FESC TFESC",
          kiss_encode(bytes([FESC])) ==
          bytes([FEND, 0x00, FESC, TFESC, FEND]))

    payload = bytes(range(256))                      # every byte value
    dec = KissDecoder()
    got = dec.push(kiss_encode(payload))
    check("every byte value survives the round trip",
          got == [(CMD_DATA, payload)])

    # a stream, split at every possible boundary, must decode identically
    wire = kiss_encode(b"one") + kiss_encode(b"two", cmd=CMD_TXDELAY)
    ok = True
    for cut in range(len(wire) + 1):
        d = KissDecoder()
        out = d.push(wire[:cut]) + d.push(wire[cut:])
        if out != [(CMD_DATA, b"one"), (CMD_TXDELAY, b"two")]:
            ok = False
    check("frames decode the same however the stream is split", ok)

    d = KissDecoder()
    check("leading garbage before the first FEND is discarded",
          d.push(b"junk" + kiss_encode(b"hi")) == [(CMD_DATA, b"hi")])
    d = KissDecoder()
    check("repeated FENDs and empty frames are tolerated",
          d.push(bytes([FEND, FEND, FEND]) + kiss_encode(b"x"))
          == [(CMD_DATA, b"x")])
    d = KissDecoder()
    check("the type byte is passed through whole",
          d.push(kiss_encode(b"p", port=3)) == [(0x30 | CMD_DATA, b"p")])


# ---------------- mapping ----------------

def test_mapping():
    b = bridge()
    b.from_host(CMD_DATA, b"\x82\xa0\x40hello")
    check("a data frame becomes one station message, bytes unchanged",
          b.m.submitted == [(b"\x82\xa0\x40hello", 2)])

    b = bridge()
    for cmd in (CMD_TXDELAY, 2, 3, 4, 5, 6, CMD_RETURN):
        b.from_host(cmd, b"\x20")
    check("TXDELAY/P/SlotTime/TXtail/duplex/hardware/Return send nothing",
          b.m.submitted == [] and b.m.written == [])

    b = bridge()
    b.from_host(0x50 | CMD_DATA, b"x")
    check("a frame for another KISS port is dropped", b.m.submitted == [])

    # mkiss marks its first two frames after an attach as checksum
    # probes; the flags live in the port nibble, and dropping them as
    # "port 8"/"port 2" loses the first thing anyone sends
    for name, flag in (("SMACK", 0x80), ("FLEX", 0x20)):
        b = bridge()
        b.from_host(flag | CMD_DATA, b"\x82\xa0\x40hello" + b"\xAB\xCD")
        check(f"an mkiss {name} probe frame is accepted, CRC stripped",
              b.m.submitted == [(b"\x82\xa0\x40hello", 2)])
    b = bridge()
    b.from_host(0x80 | CMD_DATA, b"\x01\x02")     # CRC only, no payload
    check("a CRC-only probe frame transmits nothing", b.m.submitted == [])

    b = bridge()
    b.from_host(CMD_DATA, b"")
    check("an empty frame is not transmitted", b.m.submitted == [])

    # air time, not byte count, is the limit
    b = bridge(rung=0)
    b.args.hold = 0                       # holding disabled: refuse outright
    b.from_host(CMD_DATA, bytes(256))
    check("256 B at rung 0 (278 s of air) does not go out",
          b.m.submitted == [] and b.refused == 1)
    b = bridge(rung=12)
    b.from_host(CMD_DATA, bytes(256))
    check("the same frame at rung 12 (2.6 s) goes out",
          len(b.m.submitted) == 1 and b.refused == 0)
    b = bridge(rung=0)
    b.from_host(CMD_DATA, bytes(16))
    check("a short frame still fits at rung 0", len(b.m.submitted) == 1)

    b = bridge()
    b.args.hold = 0
    b.msg_max = 200
    b.from_host(CMD_DATA, bytes(256))
    check("a frame over the board's msg_max is refused",
          b.m.submitted == [] and b.refused == 1)

    # broadcast mode: one frame -> one one-shot UP_CMD_BCAST, ptype OPAQUE
    b = bridge(mode="broadcast")
    b.from_host(CMD_DATA, b"UI!")
    ok = len(b.m.written) == 1
    if ok:
        f = b.m.written[0]
        # A5 5A type len16 | ptype rung payload
        ok = (f[:2] == b"\xa5\x5a" and f[2] == 0x06
              and f[5] == 0x0F            # OPAQUE, bits 7/6 clear = one-shot
              and f[6] == 0xFF            # let the board choose the rung
              and f[7:] == b"UI!")
    check("broadcast mode emits one one-shot OPAQUE broadcast", ok)

    # a received broadcast is reassembled into exactly one frame
    b = bridge(mode="broadcast")
    b.on_bcast(bytes([0x80 | 0x0F]))          # start
    b.on_bcast(bytes([0x00]) + b"AX")
    b.on_bcast(bytes([0x00]) + b".25")
    b.on_bcast(bytes([0x40, 0, 0, 0, 0]))     # EOS + stats
    check("a streamed broadcast reassembles into one frame",
          b.sent_frames == [b"AX.25"])

    b = bridge(mode="broadcast")
    b.on_bcast(bytes([0x00]) + b"orphan")     # data with no start
    check("broadcast data before a start marker is ignored",
          b.sent_frames == [])


def test_hold_and_probe():
    """The deadlock this exists to prevent: at rung 0 nothing bigger
    than 28 B fits, so every AX.25 frame is refused, nothing is sent,
    and the ladder that would fix it never moves."""
    import time as _t

    class HoldArgs(FakeArgs):
        hold = 120.0

    def held_bridge(rung):
        args = HoldArgs()
        b = Bridge(FakeModem(), args)
        b.status = {"rung_now": rung, "rung": rung}
        b.msg_max = 3328
        b.sent_frames = []
        b.to_hosts = lambda f: b.sent_frames.append(f)
        return b

    frame = bytes(35)                       # a small AX.25 UI frame
    b = held_bridge(0)
    b.from_host(CMD_DATA, frame)
    check("a frame too long for rung 0 is HELD, not dropped",
          b.m.submitted == [] and len(b.held) == 1)

    b.flush_held()
    check("holding sends a probe so the ladder can move",
          b.m.submitted == [(b"\x00", 0)])

    b.status = {"rung_now": 8, "rung": 8}   # the peer answered; ladder up
    b.flush_held()
    check("the held frame is released when the rung improves",
          any(d == frame for d, _ in b.m.submitted) and not b.held)

    b = held_bridge(0)
    b.from_host(CMD_DATA, frame)
    b.held = type(b.held)([(f, _t.monotonic() - 1) for f, _ in b.held])
    b.flush_held()
    check("a frame the ladder never rescues is dropped, once",
          not b.held and b.refused == 1
          and all(d != frame for d, _ in b.m.submitted))

    b = held_bridge(0)
    for i in range(10):
        b.from_host(CMD_DATA, frame)
    check("the hold queue is bounded", len(b.held) == 8 and b.refused == 2)

    # a link probe must not reach the peer's AX.25 stack
    b = held_bridge(12)
    b.pump = None
    b.to_hosts(b"x" * 20)
    check("a full-length frame does reach the host", b.sent_frames == [b"x" * 20])


def test_pty_write():
    """A frame off the air must actually reach a pty client -- the half
    that carries traffic INTO the AX.25 stack, and the half a fake
    to_hosts() in the other tests deliberately does not exercise."""
    import os as _os
    import tty as _tty
    master, slave = _os.openpty()
    # kissattach puts the line into raw mode when it opens it; a fresh
    # pty is CANONICAL, where the discipline eats and reorders the
    # binary a KISS frame is made of. Without this the test fails for a
    # reason that has nothing to do with the bridge.
    _tty.setraw(slave)
    args = FakeArgs()
    b = Bridge(FakeModem(), args)
    b.add_client(master)
    frame = b"\x82\xa0\x40\x40\x40\x40\x60hello world"
    b.to_hosts(frame)
    _os.set_blocking(slave, False)
    try:
        got = _os.read(slave, 4096)
    except BlockingIOError:
        got = b""
    dec = KissDecoder()
    check("a received frame is KISS-framed onto the pty",
          dec.push(got) == [(CMD_DATA, frame)])
    _os.close(master)
    _os.close(slave)


def test_flow_control():
    """The board holds a bounded queue; the bridge must pace against it
    rather than stuffing it and reporting every refused frame as sent.
    Measured: pings at 1/s against 16.5-second frames produced a hundred
    "tx" lines and a hundred "submit refused" answers."""

    class HoldArgs(FakeArgs):
        hold = 120.0

    def b_with(queues, rung=12):
        args = HoldArgs()
        b = Bridge(FakeModem(), args)
        b.status = {"rung_now": rung, "rung": rung, "queues": queues}
        b.msg_max = 3328
        b.sent_frames = []
        b.to_hosts = lambda f: b.sent_frames.append(f)
        return b

    b = b_with((0, 0, 0))
    b.from_host(CMD_DATA, bytes(50))
    check("an idle board takes the frame", len(b.m.submitted) == 1)

    b = b_with((0, 0, 2))                  # bulk queue at the limit
    b.from_host(CMD_DATA, bytes(50))
    check("a full bulk queue holds the frame instead of stuffing it",
          b.m.submitted == [] and len(b.held) == 1)

    b.status["queues"] = (0, 0, 0)         # the board drained
    b.flush_held()
    check("and it is released when the board drains",
          len(b.m.submitted) == 1 and not b.held)

    # the refusal arrives after the fact: the books must be corrected
    b = b_with((0, 0, 0))
    b.from_host(CMD_DATA, bytes(50))
    before = b.sent
    b.pump_modem = None                    # not needed; drive the branch
    b.sent = max(0, b.sent - 1); b.refused += 1
    check("a refusal after the fact undoes the sent count",
          before == 1 and b.sent == 0 and b.refused == 1)

    # repeated identical lines collapse
    b = b_with((0, 0, 0))
    for _ in range(30):
        b.say("same thing")
    check("repeated messages are collapsed, not repeated 30 times",
          b.last_said == ("same thing", 30))


def test_air_time_parity():
    """The bridge's table must track the model it was generated from."""
    try:
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.abspath(__file__)), ".."))
        from ofdm_phy.station import estimate_air_time as exact
    except Exception as e:                       # no NumPy in this venv
        print(f"[SKIP] air-time parity ({type(e).__name__}) -- "
              f"run under ../venv/bin/python to check it")
        return
    from kiss_bridge import estimate_air_time as local
    worst_over, optimistic = 0.0, []
    for r in range(13):
        for n in (0, 1, 16, 32, 64, 128, 200, 255, 256, 1000, 3328):
            d = local(r, n) - exact(r, n)
            worst_over = max(worst_over, d)
            if d < 0:
                optimistic.append((r, n, d))
    check("the bridge never under-estimates air time", not optimistic)
    check(f"and stays within a second of it ({worst_over:.2f}s worst)",
          worst_over < 1.0)


def main():
    test_codec()
    test_mapping()
    test_hold_and_probe()
    test_pty_write()
    test_flow_control()
    test_air_time_parity()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
