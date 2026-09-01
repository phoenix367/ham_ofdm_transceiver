#!/usr/bin/env python3
"""Host-library tests -- no hardware.

    ./test_modem.py

Covers the device-selection path, which is where the two intermittent
"cannot find the board" failures came from: a control transfer that
transiently fails while the board is pushing, and pyusb caching that
failure so every later attempt fails instantly.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ofdm_modem import _read_serial, Parser, encode, MAX_PAYLOAD  # noqa: E402

PASS = FAIL = 0


def check(name, ok):
    global PASS, FAIL
    print(f"[{'PASS' if ok else 'FAIL'}] {name}")
    if ok:
        PASS += 1
    else:
        FAIL += 1


class FakeDev:
    """Enough of a pyusb device to exercise the langid cache."""

    def __init__(self):
        self.iSerialNumber = 3
        self._langids = None


def poisoning_get_string(fail_times):
    """Mimics pyusb: a failed fetch leaves _langids = (), and while it is
    set every call raises immediately -- no traffic, no chance."""
    state = {"left": fail_times}

    def get_string(dev, index):
        if dev._langids == ():
            raise ValueError("The device has no langid")
        if state["left"] > 0:
            state["left"] -= 1
            dev._langids = ()          # what pyusb leaves behind
            raise ValueError("The device has no langid")
        return "240041000551333438363436"
    return get_string


def test_serial_read():
    d = FakeDev()
    got = _read_serial(d, poisoning_get_string(1), tries=4, delay=0)
    check("a first-touch failure is retried, not believed",
          got == "240041000551333438363436")

    d = FakeDev()
    got = _read_serial(d, poisoning_get_string(3), tries=4, delay=0)
    check("and repeated failures are still recovered inside the budget",
          got == "240041000551333438363436")

    # the point of the fix: without clearing the cache, a retry is a
    # no-op, so this must fail if the un-poisoning is ever removed
    d = FakeDev()
    gs = poisoning_get_string(1)
    d._langids = ()
    check("a poisoned cache alone makes get_string fail",
          _read_serial(d, gs, tries=1, delay=0) is None)

    d = FakeDev()
    got = _read_serial(d, poisoning_get_string(99), tries=3, delay=0)
    check("a device that never answers returns None, it does not raise",
          got is None)

    d = FakeDev()
    calls = {"n": 0}

    def counting(dev, index):
        calls["n"] += 1
        raise ValueError("nope")
    _read_serial(d, counting, tries=5, delay=0)
    check("the retry budget is honoured exactly", calls["n"] == 5)


def test_framing():
    """The frame cap must track the device's UP_MAX_PAYLOAD (3336): it
    was left at the protocol's original 1024 while messages grew, and
    the parser silently dropped every file part."""
    check("MAX_PAYLOAD matches UP_MAX_PAYLOAD", MAX_PAYLOAD == 3336)
    big = bytes(range(256)) * 13                      # 3328 B, a full message
    p = Parser()
    frames = p.push(encode(0x82, big))
    check("a full-size message frames and parses back",
          frames == [(0x82, big)] and p.resyncs == 0)


def main():
    test_serial_read()
    test_framing()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
