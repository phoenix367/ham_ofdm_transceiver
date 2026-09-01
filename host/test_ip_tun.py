#!/usr/bin/env python3
"""ip_tun policy tests -- no root, no board, no TUN device.

    ./test_ip_tun.py

The device half is three lines of ioctl; the half worth testing is what
the tunnel decides to send, drop and probe for.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ip_tun import Tunnel                                    # noqa: E402

PASS = FAIL = 0


def check(name, ok):
    global PASS, FAIL
    print(f"[{'PASS' if ok else 'FAIL'}] {name}")
    if ok:
        PASS += 1
    else:
        FAIL += 1


class Args:
    qos = 2
    max_air = 45.0
    msg_max = 3328
    verbose = False


class FakeModem:
    def __init__(self):
        self.submitted = []

    def submit(self, data, qos=2):
        self.submitted.append((bytes(data), qos))


def tunnel(rung=12, queues=(0, 0, 0)):
    t = Tunnel(FakeModem(), Args())
    t.status = {"rung_now": rung, "rung": rung, "queues": queues}
    return t


def test_send_and_drop():
    t = tunnel()
    t.on_packet(bytes(500))
    check("a packet that fits is submitted whole, unwrapped",
          t.m.submitted == [(bytes(500), 2)] and t.sent == 1)

    t = tunnel()
    t.on_packet(b"")
    check("an empty read is not a packet", t.m.submitted == [])

    t = tunnel()
    t.args.msg_max = 256
    t.on_packet(bytes(500))
    check("a packet over the board's message limit is dropped, loudly",
          t.m.submitted == [] and t.dropped == 1)

    # IP is allowed to lose packets; holding a stale one is worse
    t = tunnel(rung=0)
    t.on_packet(bytes(1000))
    check("a packet too slow for the current rung is DROPPED, not queued",
          t.m.submitted == [] and t.dropped == 1)
    check("and that marks the ladder as the blocker", t.stuck is True)

    t = tunnel(queues=(0, 0, 2))
    t.on_packet(bytes(100))
    check("a full board queue drops too", t.m.submitted == [] and t.dropped == 1)
    check("but that is congestion, not the ladder", t.stuck is False)


def test_pacing():
    t = tunnel()
    for _ in range(4):
        t.on_packet(bytes(100))
    check("in-flight credit bounds a burst to the gate",
          len(t.m.submitted) == 2 and t.dropped == 2)
    t.on_status({"rung_now": 12, "rung": 12, "queues": (0, 0, 0)})
    check("a status refreshes the credit", t.inflight == 0)
    t.on_packet(bytes(100))
    check("and traffic resumes", len(t.m.submitted) == 3)


def test_probe():
    t = tunnel(rung=0)
    check("no probe while nothing is blocked", not t.probe_due(1000.0))
    t.on_packet(bytes(1000))               # blocked by the rung
    check("a ladder-blocked drop asks for a probe", t.probe_due(1000.0))
    check("and not again immediately", not t.probe_due(1001.0))
    t.send_probe()
    check("the probe is one byte, out of band of the data queue",
          t.m.submitted[-1] == (b"\x00", 0))

    t = tunnel(queues=(0, 0, 2))
    t.on_packet(bytes(100))                # blocked by congestion
    check("a congestion drop does NOT probe -- more traffic is not the fix",
          not t.probe_due(1000.0))


def test_receive():
    t = tunnel()
    pkt = bytes(range(20)) + b"payload"
    check("a full-size message comes back as a packet", t.on_message(pkt) == pkt)
    check("and is counted", t.rcvd == 1)
    check("a probe-sized message is not written to the interface",
          t.on_message(b"\x00") is None and t.rcvd == 1)


def main():
    test_send_and_drop()
    test_pacing()
    test_probe()
    test_receive()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
