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
          got == [(0, CMD_DATA, payload)])

    # a stream, split at every possible boundary, must decode identically
    wire = kiss_encode(b"one") + kiss_encode(b"two", cmd=CMD_TXDELAY)
    ok = True
    for cut in range(len(wire) + 1):
        d = KissDecoder()
        out = d.push(wire[:cut]) + d.push(wire[cut:])
        if out != [(0, CMD_DATA, b"one"), (0, CMD_TXDELAY, b"two")]:
            ok = False
    check("frames decode the same however the stream is split", ok)

    d = KissDecoder()
    check("leading garbage before the first FEND is discarded",
          d.push(b"junk" + kiss_encode(b"hi")) == [(0, CMD_DATA, b"hi")])
    d = KissDecoder()
    check("repeated FENDs and empty frames are tolerated",
          d.push(bytes([FEND, FEND, FEND]) + kiss_encode(b"x"))
          == [(0, CMD_DATA, b"x")])
    d = KissDecoder()
    check("the port nibble is decoded",
          d.push(kiss_encode(b"p", port=3)) == [(3, CMD_DATA, b"p")])


# ---------------- mapping ----------------

def test_mapping():
    b = bridge()
    b.from_host(0, CMD_DATA, b"\x82\xa0\x40hello")
    check("a data frame becomes one station message, bytes unchanged",
          b.m.submitted == [(b"\x82\xa0\x40hello", 2)])

    b = bridge()
    for cmd in (CMD_TXDELAY, 2, 3, 4, 5, 6, CMD_RETURN):
        b.from_host(0, cmd, b"\x20")
    check("TXDELAY/P/SlotTime/TXtail/duplex/hardware/Return send nothing",
          b.m.submitted == [] and b.m.written == [])

    b = bridge()
    b.from_host(1, CMD_DATA, b"x")
    check("a frame for another KISS port is dropped", b.m.submitted == [])

    b = bridge()
    b.from_host(0, CMD_DATA, b"")
    check("an empty frame is not transmitted", b.m.submitted == [])

    # air time, not byte count, is the limit
    b = bridge(rung=0)
    b.from_host(0, CMD_DATA, bytes(256))
    check("256 B at rung 0 (278 s of air) is refused",
          b.m.submitted == [] and b.refused == 1)
    b = bridge(rung=12)
    b.from_host(0, CMD_DATA, bytes(256))
    check("the same frame at rung 12 (2.6 s) goes out",
          len(b.m.submitted) == 1 and b.refused == 0)
    b = bridge(rung=0)
    b.from_host(0, CMD_DATA, bytes(16))
    check("a short frame still fits at rung 0", len(b.m.submitted) == 1)

    b = bridge()
    b.msg_max = 200
    b.from_host(0, CMD_DATA, bytes(256))
    check("a frame over the board's msg_max is refused",
          b.m.submitted == [] and b.refused == 1)

    # broadcast mode: one frame -> one one-shot UP_CMD_BCAST, ptype OPAQUE
    b = bridge(mode="broadcast")
    b.from_host(0, CMD_DATA, b"UI!")
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
    test_air_time_parity()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
