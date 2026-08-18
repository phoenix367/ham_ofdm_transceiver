#!/usr/bin/env python3
"""Channel configuration CLI for the virtual driver's ctl.sock.

Usage:
  ./chanctl.py                       # print current config
  ./chanctl.py snr_db=-5 fading_hz=0.2 delay_ms=25
"""

import json
import socket
import sys

DEFAULT = "/tmp/ofdmchan/ctl.sock"


def main():
    args = sys.argv[1:]
    path = DEFAULT
    if args and args[0].startswith("--dir="):
        path = args.pop(0).split("=", 1)[1] + "/ctl.sock"
    req = {}
    for a in args:
        k, v = a.split("=", 1)
        req[k] = float(v)
    if not req:
        req = {"get": True}
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(path)
    s.sendall((json.dumps(req) + "\n").encode())
    print(s.makefile().readline().strip())


if __name__ == "__main__":
    main()
