#!/usr/bin/env python3
"""End-to-end test of the host driver against the real device-side C.

No hardware: cport/build/usb_modem_emu runs the actual usb_modem.c and
usb_proto.c over a pipe, so a passing run exercises both ends of the
protocol as shipped. What it does NOT cover is the USB peripheral driver
itself, which cannot be tested off-target.

    ./host/test_ofdm_modem.py
"""
import os
import struct
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ofdm_modem import (OfdmModem, Parser, encode, decode_status,  # noqa
                        CMD_PING, EVT_STATUS, EVT_DIAG, RSP_PONG)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EMU = os.path.join(ROOT, "cport", "build", "usb_modem_emu")

npass = nfail = 0


def check(name, ok, detail=""):
    global npass, nfail
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
    if ok:
        npass += 1
    else:
        nfail += 1


def main():
    global npass, nfail
    if not os.path.exists(EMU):
        print(f"emulator missing: {EMU}\nrun: make -C cport usbemu", file=sys.stderr)
        return 2

    with OfdmModem(emulate=[EMU]) as m:
        info = m.info()
        check("CMD_INFO answered", info is not None)
        if info:
            check("info fields are sane",
                  info["proto_ver"] == 1 and info["n_modes"] == 3
                  and info["sample_rate"] == 12000 and len(info["serial"]) == 24,
                  f"fw {info['fw']} serial {info['serial']}")
            # 24 uppercase hex characters. Not .isupper(): a UID that
            # happens to be all digits has no cased characters at all,
            # which is exactly what this device's is.
            check("serial is the device's unique ID, not a port number",
                  len(info["serial"]) == 24
                  and all(c in "0123456789ABCDEF" for c in info["serial"]))

        check("ping round trip", m.ping(0xCAFEBABE))
        check("a second ping with a different token still matches",
              m.ping(0x00C0FFEE))

        # submit a message; the station accepts it and the queue depth moves
        m.submit(b"HELLO OVER USB", qos=2)
        seen = {}
        for _ in range(8):
            for name, payload in m.events(timeout=0.3):
                seen.setdefault(name, []).append(payload)
        check("status frames arrive unprompted", "status" in seen,
              f"{len(seen.get('status', []))} seen")
        st = seen.get("status", [None])[-1]
        if st:
            check("submitted message shows in a queue depth",
                  sum(st["queues"]) >= 1 or st["pending"],
                  f"queues={st['queues']} pending={st['pending']}")

        check("no resyncs on a clean link", m.parser.resyncs == 0,
              f"{m.parser.resyncs}")

    # --- the parser is the part a real bus will stress ---
    p = Parser()
    frames = b"".join(encode(CMD_PING, struct.pack("<I", i)) for i in range(4))
    out = []
    for i in range(len(frames)):          # one byte at a time
        out += p.push(frames[i:i + 1])
    check("host parser reassembles frames split byte by byte",
          len(out) == 4 and p.resyncs == 0)

    p = Parser()
    junk = b"\x00\xff\xa5\x11\x5a\xa5"
    out = p.push(junk + encode(CMD_PING, b"\x01\x02\x03\x04"))
    check("host parser resyncs past garbage and keeps the next frame",
          len(out) == 1 and out[0][0] == CMD_PING and p.resyncs > 0)

    p = Parser()
    out = p.push(b"\xa5\x5a\x86\xff\xff" + encode(CMD_PING, b"\x09\x09\x09\x09"))
    check("host parser rejects an impossible length instead of stalling",
          len(out) == 1 and p.resyncs > 0)

    print(f"\n{npass} passed, {nfail} failed")
    return 1 if nfail else 0


if __name__ == "__main__":
    sys.exit(main())
