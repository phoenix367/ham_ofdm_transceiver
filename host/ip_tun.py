#!/usr/bin/env python3
"""IP over the OFDM link, with no AX.25 in the path.

    sudo ./ip_tun.py --serial <uid> --local 10.73.0.1/24 --peer 10.73.0.2
    sudo ip netns exec radio2 \\
         ./ip_tun.py --serial <other> --local 10.73.0.2/24 --peer 10.73.0.1

Why this exists
---------------
The kernel's AX.25 stack is not network-namespace aware: its receive
handler drops frames arriving on a device outside the initial namespace.
Measured on this stand, with the far interface in a namespace, the
device counted 82 received packets while the IP stack reported "0 ICMP
messages received" -- the frames crossed the radio and died one layer
above it. That rules out namespaces AND containers (a container's
isolation IS a network namespace) for IP over `kiss_bridge.py`, and
leaves a second machine as the only option.

TUN devices have no such restriction. This tool carries IP packets
directly as station messages, so both ends can live on one host, in
namespaces or containers, and the AX.25 layer is not involved at all.
It also saves the 16-byte AX.25 header on every packet and the whole
paclen/fragmentation question.

Use kiss_bridge.py when you want to interoperate with packet-radio
software. Use this when you want IP.

What to expect
--------------
An MTU-sized packet is one station message with its own acknowledgment:
1000 bytes is about 9 s of air at rung 12, so roughly 110 B/s and
seconds of latency. TCP will work and will feel awful; ICMP, UDP and
anything patient are fine.
"""
import argparse
import fcntl
import os
import select
import signal
import struct
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ofdm_modem import OfdmModem, encode, USBTimeoutError   # noqa: E402
from kiss_bridge import estimate_air_time, QUEUE_HIGH       # noqa: E402

TUNSETIFF = 0x400454CA
IFF_TUN = 0x0001
IFF_NO_PI = 0x1000        # raw IP packets, no 4-byte prefix


def tun_open(name):
    """A TUN device, and the fd to move packets through it."""
    fd = os.open("/dev/net/tun", os.O_RDWR)
    ifr = struct.pack("16sH22s", name.encode(), IFF_TUN | IFF_NO_PI, b"")
    fcntl.ioctl(fd, TUNSETIFF, ifr)
    return fd


def ip(*args):
    subprocess.run(["ip", *args], check=True)


class Tunnel:
    """The policy half: what goes out, what is dropped, and why.

    Separated from the device so it can be tested without root or a
    board (host/test_ip_tun.py).
    """

    def __init__(self, modem, args):
        self.m = modem
        self.args = args
        self.status = {}
        self.inflight = 0          # submits since the last status frame
        self.sent = self.rcvd = self.dropped = 0
        self.drops_said = 0.0
        self.next_probe = 0.0
        self.stuck = False         # last drop was the ladder, not the queue

    # -- what the board is doing ---------------------------------------
    def rung(self):
        r = self.status.get("rung_now")
        if r is None:
            r = self.status.get("rung", -1)
        return r

    def air_time(self, n):
        return estimate_air_time(self.rung(), n)

    def board_full(self):
        q = self.status.get("queues")
        depth = q[min(self.args.qos, 2)] if q else 0
        return depth + self.inflight >= QUEUE_HIGH

    def wants_read(self):
        """Should the tunnel take another packet from the kernel now?

        NOT reading is the whole trick. The first version read every
        packet and discarded what would not fit, which throws away the
        kernel's queue and hands TCP a loss for every packet of its
        opening burst -- Linux starts with ten. Measured: 1 kB took
        116.8 s (8.8 B/s) with 5 and 7 application drops, most of them
        in the first second, and the last segment cost 77 s of
        retransmit backoff.

        Leaving the packets in the interface queue instead gives TCP
        what it expects: backpressure, and AQM drops from the qdisc
        rather than a black hole. Bound the queue (txqueuelen) so this
        does not become bufferbloat.
        """
        if self.board_full():
            self.stuck = False
            return False
        if self.air_time(self.args.mtu) > self.args.max_air:
            self.stuck = True          # the ladder, not congestion
            return False
        return True

    # -- the two directions --------------------------------------------
    def on_packet(self, pkt):
        """An IP packet from the kernel, heading for the radio.

        IP is allowed to lose packets and every sender above it
        retransmits, so a packet that cannot go now is DROPPED rather
        than queued: holding it would deliver something stale minutes
        later and call it success. (kiss_bridge holds instead, because
        an AX.25 UI frame has nobody to retry it.)
        """
        if not pkt:
            return
        if len(pkt) > self.args.msg_max:
            self.dropped += 1
            self.say(f"dropped a {len(pkt)} B packet: over the board's "
                     f"{self.args.msg_max} B message limit (lower the MTU)")
            return
        # Defensive only: the loop does not read while these hold, so
        # reaching here means the situation changed under us.
        if self.board_full():
            self.dropped += 1
            self.stuck = False
            self.note_drops()
            return
        air = self.air_time(len(pkt))
        if air > self.args.max_air:
            self.dropped += 1
            self.stuck = True
            self.note_drops()
            return
        try:
            self.m.submit(pkt, qos=self.args.qos)
        except USBTimeoutError:
            self.dropped += 1
            self.log("the board did not accept a write (busy decoding)")
            return
        self.sent += 1
        self.inflight += 1
        self.log(f"tx {len(pkt)} B, {air:.1f}s at rung {self.rung()}")

    def on_message(self, data):
        """A station message from the peer: an IP packet, or a probe."""
        if len(data) < 20:         # smaller than an IPv4 header
            self.log(f"ignoring a {len(data)} B message (a link probe)")
            return None
        self.rcvd += 1
        self.log(f"rx {len(data)} B")
        return data

    # -- housekeeping ---------------------------------------------------
    def probe_due(self, now):
        """Only when the LADDER is what is blocking traffic.

        A cold link sits at rung 0, where nothing MTU-sized fits, so
        without a nudge nothing is ever sent and the ladder -- which
        only moves on exchanges -- never comes up. Congestion is a
        different problem and more traffic is not its answer.
        """
        if not self.stuck or now < self.next_probe:
            return False
        self.next_probe = now + 30.0
        return True

    def send_probe(self):
        try:
            self.m.submit(b"\x00", qos=0)
            self.log("probe sent -- bringing the ladder up")
        except USBTimeoutError:
            pass

    def note_drops(self):
        now = time.monotonic()
        if now - self.drops_said < 10.0:
            return
        self.drops_said = now
        why = (f"nothing MTU-sized fits at rung {self.rung()}"
               if self.stuck else "the board's queue is full")
        self.say(f"dropping packets -- {why} ({self.dropped} so far)")

    def log(self, msg):
        if self.args.verbose:
            print(f"{time.strftime('%H:%M:%S')} [tun] {msg}", flush=True)

    def say(self, msg):
        print(f"{time.strftime('%H:%M:%S')} [tun] {msg}", flush=True)

    def on_status(self, st):
        was = self.rung()
        self.status = st
        self.inflight = 0          # the report now accounts for them
        now = self.rung()
        if now != was:
            # Announced, not verbose-only: on a cold link this is the
            # difference between "nothing works" and "the ladder is
            # still climbing, wait". A 64-byte ping is 110 s of air at
            # rung 0 and 1.5 s at rung 12.
            fits = estimate_air_time(now, self.args.mtu)
            self.say(f"rung {was} -> {now} "
                     f"(an MTU-sized packet is {fits:.0f}s of air)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--serial", help="board UID (omit if only one attached)")
    ap.add_argument("--emulate", help="run the emulator binary instead")
    ap.add_argument("--dev", default="ofdm0", help="TUN device name")
    ap.add_argument("--local", required=True, metavar="A.B.C.D/N",
                    help="address for this end")
    ap.add_argument("--peer", metavar="A.B.C.D",
                    help="the far end (adds a host route through the tunnel)")
    ap.add_argument("--mtu", type=int, default=1000,
                    help="bigger is more efficient and slower per packet: "
                         "1000 B is ~9 s of air at rung 12 (default 1000)")
    ap.add_argument("--qos", type=int, default=2, choices=(0, 1, 2))
    ap.add_argument("--qlen", type=int, default=8,
                    help="interface queue, in packets: the backpressure "
                         "TCP needs, bounded so it is not bufferbloat")
    ap.add_argument("--max-air", type=float, default=45.0,
                    help="drop packets that would take longer than this")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    # a killed tunnel should say what it did, not disappear
    signal.signal(signal.SIGTERM, lambda *a: sys.exit(0))

    if os.geteuid() != 0:
        print("ip_tun: needs root (it creates a network device) -- "
              "try sudo", file=sys.stderr)
        return 1

    try:
        modem = (OfdmModem(emulate=args.emulate.split()) if args.emulate
                 else OfdmModem(serial=args.serial))
    except RuntimeError as e:
        print(f"ip_tun: {e}", file=sys.stderr)
        return 1

    info = modem.info() or {}
    args.msg_max = info.get("msg_max") or 256
    if args.mtu > args.msg_max:
        print(f"ip_tun: mtu {args.mtu} exceeds the board's {args.msg_max} B "
              f"message limit; using {args.msg_max}", file=sys.stderr)
        args.mtu = args.msg_max

    fd = tun_open(args.dev)
    ip("addr", "add", args.local, "dev", args.dev)
    ip("link", "set", args.dev, "mtu", str(args.mtu), "up")
    # Eight packets is about a minute of air at rung 12 -- enough for
    # TCP to keep a window in flight, short enough that a queued packet
    # is still worth delivering when it reaches the front.
    ip("link", "set", args.dev, "txqueuelen", str(args.qlen))
    if args.peer:
        # explicit host route: with both ends on one host in different
        # namespaces, this is what sends the packet to the radio
        subprocess.run(["ip", "route", "replace", args.peer,
                        "dev", args.dev], check=False)

    t = Tunnel(modem, args)
    # flush=True everywhere: stdout redirected to a file is block
    # buffered, so these lines sat in the buffer until something else
    # flushed -- and a SIGTERM discards it. An empty log then looks
    # exactly like a process that never started.
    print(f"[tun] {args.dev} up: {args.local} mtu {args.mtu}, "
          f"board fw {info.get('fw', '?')}, {args.msg_max} B messages",
          flush=True)
    if args.peer:
        print(f"[tun] try: ping -c 3 -i 10 -W 60 -s 64 {args.peer}",
              flush=True)

    last_ping = 0.0
    try:
        while True:
            now = time.monotonic()
            if now - last_ping >= 1.0:
                try:
                    modem.t.write(encode(0x04, int(now).to_bytes(4, "little")))
                except USBTimeoutError:
                    pass
                last_ping = now
            # Only offer the tunnel fd to select when the board can
            # take another packet: unread packets wait in the kernel's
            # queue, where TCP can see the backpressure.
            watch = [fd] if t.wants_read() else []
            r, _, _ = select.select(watch, [], [], 0.05)
            if r:
                t.on_packet(os.read(fd, 65535))
            for kind, payload in modem.events(timeout=0.05, poke=False):
                if kind == "status" and payload:
                    t.on_status(payload)
                elif kind == "message":
                    pkt = t.on_message(payload["data"])
                    if pkt:
                        os.write(fd, pkt)
                elif kind == "log":
                    t.log(f"board: {payload}")
            if t.probe_due(time.monotonic()):
                t.send_probe()
    except (KeyboardInterrupt, SystemExit):
        print(f"\n[tun] {t.sent} packet(s) sent, {t.rcvd} received, "
              f"{t.dropped} dropped", flush=True)
    finally:
        os.close(fd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
