#!/usr/bin/env python3
"""Checks board_console's file envelope against app.c's byte layout.

The two programs only interoperate if this envelope is identical, and
nothing at run time would tell us it is not -- a mismatch just looks
like a peer sending unparseable messages. So the offsets app.c uses are
asserted here directly:

    env[0]                       magic
    env[1 .. 1+len("FILE:"))     "FILE:"
    env[1+5 ..]                  basename, NUL terminated
    env[head-4 .. head-2)        part index, little-endian
    env[head-2 .. head)          n_parts, little-endian
    env[head ..]                 data
    head = 1 + len("FILE:") + len(basename) + 1 + 4

That is the WIDE envelope (magic 0x03 raw / 0x04 deflated), which both
consoles send. The byte-indexed one (0x01 / 0x02, head ... + 2) is
still accepted on receive; a case below feeds one in.
"""
import os
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import board_console as bc                            # noqa: E402

FAIL = 0


def check(name, ok):
    global FAIL
    print(f"[{'PASS' if ok else 'FAIL'}] {name}")
    if not ok:
        FAIL += 1


class FakeModem:
    def __init__(self):
        self.sent = []

    def submit(self, data, qos=2):
        self.sent.append((qos, data))

    def info(self):
        return None


def build(tmpdir, name, payload, msg_max=256, compress=True):
    path = os.path.join(tmpdir, name)
    with open(path, "wb") as f:
        f.write(payload)
    m = FakeModem()
    con = bc.Console(m, "T", msg_max, compress)
    con.cmd_sendfile(path)
    # pump_file only tops the queue up to INFLIGHT, so drain it the way
    # a run of status events would
    while con.pending:
        before = len(m.sent)
        con.pump_file({"queues": (0, 0, 0)})
        if len(m.sent) == before:
            raise AssertionError("pump_file made no progress")
    return con, [d for _, d in m.sent]


def main(tmpdir):
    # --- 1. exact byte layout, matching app.c ---------------------
    con, parts = build(tmpdir, "hello.txt", b"A" * 50, compress=False)
    env = parts[0]
    base = b"hello.txt"
    head = 1 + len(bc.FILE_TAG) + len(base) + 1 + 4
    check("magic is 0x03 for an uncompressed file (wide envelope)",
          env[0] == bc.FILE_MAGIC_W)
    check('tag is "FILE:" at offset 1',
          env[1:1 + len(bc.FILE_TAG)] == bc.FILE_TAG)
    check("basename follows the tag, NUL terminated",
          env[1 + len(bc.FILE_TAG):1 + len(bc.FILE_TAG) + len(base)] == base
          and env[1 + len(bc.FILE_TAG) + len(base)] == 0)
    check("part index at head-4 (LE16), n_parts at head-2 (LE16)",
          env[head - 4:head - 2] == b"\0\0" and env[head - 2:head] == b"\1\0")
    check("data starts at head", env[head:] == b"A" * 50)
    check("one part for a small file", len(parts) == 1)

    # --- 2. the part size respects the board's message limit ------
    big = bytes((i * 7 + 3) & 0xFF for i in range(4000))   # poorly compressible
    con, parts = build(tmpdir, "big.bin", big, msg_max=256, compress=False)
    check("no part exceeds the board's message limit",
          all(len(p) <= 256 for p in parts))
    off = 1 + len(bc.FILE_TAG) + len(b"big.bin") + 1
    check("every part carries the same n_parts",
          len({p[off + 2:off + 4] for p in parts}) == 1)
    check("part indices are 0..n-1 in order",
          [int.from_bytes(p[off:off + 2], "little") for p in parts]
          == list(range(len(parts))))

    # --- 3. round trip through the receive path -------------------
    for label, payload, comp in (("uncompressed", big, False),
                                 ("deflated", b"B" * 9000, True)):
        con, parts = build(tmpdir, "rt.bin", payload, compress=comp)
        cwd = os.getcwd()
        os.chdir(tmpdir)
        try:
            rx = bc.Console(FakeModem(), "R", 256, True)
            for p in parts:
                rx.on_file_part(p)
            got = open("rx_rt.bin", "rb").read()
        finally:
            os.chdir(cwd)
        check(f"round trip reproduces the file ({label}, {len(parts)} parts)",
              got == payload)

    # --- 4. compression is used only when it helps ----------------
    con, parts = build(tmpdir, "z.bin", b"C" * 20000, compress=True)
    check("magic is 0x04 when DEFLATE shrinks the file",
          parts[0][0] == bc.FILE_MAGIC_WZ)
    incompressible = os.urandom(2000)
    con, parts = build(tmpdir, "r.bin", incompressible, compress=True)
    check("magic stays 0x03 when DEFLATE would not shrink it",
          parts[0][0] == bc.FILE_MAGIC_W)

    # --- 4b. more than 255 parts, which the byte index could not ---
    # 68 kB of noise at 256-byte messages is ~290 parts: the case a PNG
    # hit on the two-board stand ("needs 288 parts, cap is 255")
    noise = os.urandom(68143)
    con, parts = build(tmpdir, "big68k.bin", noise, compress=True)
    check("a 68 kB incompressible file splits into more than 255 parts",
          len(parts) > 255)
    cwd = os.getcwd()
    os.chdir(tmpdir)
    try:
        rx = bc.Console(FakeModem(), "R", 256, True)
        for p in parts:
            rx.on_file_part(p)
        got = open("rx_big68k.bin", "rb").read()
    finally:
        os.chdir(cwd)
    check(f"and round-trips byte-exact ({len(parts)} parts)", got == noise)

    # --- 4c. the byte-indexed envelope is still received -----------
    legacy = [bytes([bc.FILE_MAGIC]) + bc.FILE_TAG + b"old.bin\0"
              + bytes([i, 3]) + bytes([i]) * 100 for i in range(3)]
    os.chdir(tmpdir)
    try:
        rx = bc.Console(FakeModem(), "R", 256, True)
        for p in legacy:
            rx.on_file_part(p)
        got = open("rx_old.bin", "rb").read()
    finally:
        os.chdir(cwd)
    check("a legacy 0x01 envelope (part(1) n_parts(1)) still reassembles",
          got == b"".join(bytes([i]) * 100 for i in range(3)))

    # --- 5. a name that leaves no room is refused, not truncated --
    m = FakeModem()
    con = bc.Console(m, "T", 32, False)
    long_path = os.path.join(tmpdir, "x" * 40)
    open(long_path, "wb").write(b"hi")
    con.cmd_sendfile(long_path)
    check("a name too long for the message limit is refused",
          m.sent == [] and not con.pending)

    # --- 6. pacing never exceeds INFLIGHT in the board's queue ----
    m = FakeModem()
    con = bc.Console(m, "T", 256, False)
    path = os.path.join(tmpdir, "paced.bin")
    open(path, "wb").write(big)
    con.cmd_sendfile(path)
    con.pump_file({"queues": (0, 0, 0)})
    check("first fill hands over exactly INFLIGHT parts",
          len(m.sent) == bc.INFLIGHT)
    con.pump_file({"queues": (0, 0, bc.INFLIGHT)})
    check("a full queue gets nothing more", len(m.sent) == bc.INFLIGHT)
    con.pump_file({"queues": (0, 0, 1)})
    check("room for 3 hands over 3 more",
          len(m.sent) == bc.INFLIGHT + 3)

    print("\n" + ("FAILED" if FAIL else "all ok"))
    return 1 if FAIL else 0


if __name__ == "__main__":
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        sys.exit(main(td))
