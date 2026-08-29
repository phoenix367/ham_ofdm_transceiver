#!/usr/bin/env python3
"""TCP <-> serial bridge for the ESP32 remote_bitbang probe.

OpenOCD's remote_bitbang driver connects to a TCP or UNIX socket; the
probe is on a serial port. This is the ten lines in between. (socat would
do it too -- `socat TCP-LISTEN:3335,reuseaddr /dev/ttyUSB0,raw,b921600`
-- but it is not installed here and this needs nothing beyond stdlib.)

Latency, not bandwidth, is what matters: every read request from OpenOCD
blocks until the reply lands, so TCP_NODELAY is set on the socket and the
serial side is read one chunk at a time with no buffering delay.

    ./rbb_bridge.py --port /dev/ttyUSB0 --baud 921600 --listen 3335

Then in OpenOCD:

    adapter driver remote_bitbang
    remote_bitbang host localhost
    remote_bitbang port 3335
"""

import argparse
import os
import select
import socket
import sys
import termios
import time

BAUDS = {115200: termios.B115200, 230400: termios.B230400,
         460800: termios.B460800, 921600: termios.B921600}


def open_serial(path, baud):
    """Raw 8N1, no flow control, no modem-control games."""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    a = termios.tcgetattr(fd)
    # iflag oflag cflag lflag ispeed ospeed cc
    a[0] = 0
    a[1] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[3] = 0
    a[4] = a[5] = BAUDS[baud]
    a[6] = list(a[6])
    a[6][termios.VMIN] = 0
    a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600, choices=sorted(BAUDS))
    ap.add_argument("--listen", type=int, default=3335)
    ap.add_argument("--settle", type=float, default=1.5,
                    help="seconds to let the ESP32 boot and go quiet "
                         "(opening the port resets it on a devkit)")
    args = ap.parse_args()

    fd = open_serial(args.port, args.baud)
    # Opening the port toggles DTR/RTS, which on a devkit's auto-reset
    # circuit reboots the ESP32. Wait it out, then drop the ROM banner so
    # OpenOCD never sees it as a protocol reply.
    time.sleep(args.settle)
    termios.tcflush(fd, termios.TCIOFLUSH)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", args.listen))
    srv.listen(1)
    print(f"bridge: {args.port}@{args.baud} <-> tcp/{args.listen}  "
          f"(waiting for openocd)", flush=True)

    while True:
        conn, _ = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        termios.tcflush(fd, termios.TCIOFLUSH)
        print("bridge: openocd connected", flush=True)
        n_tx = n_rx = 0
        try:
            while True:
                r, _, _ = select.select([conn, fd], [], [])
                if conn in r:
                    data = conn.recv(4096)
                    if not data:
                        break
                    os.write(fd, data)
                    n_tx += len(data)
                if fd in r:
                    data = os.read(fd, 4096)
                    if data:
                        conn.sendall(data)
                        n_rx += len(data)
        except (ConnectionResetError, BrokenPipeError, OSError) as e:
            print(f"bridge: {e}", file=sys.stderr)
        finally:
            conn.close()
            print(f"bridge: closed ({n_tx} B to probe, {n_rx} B back)",
                  flush=True)


if __name__ == "__main__":
    main()
