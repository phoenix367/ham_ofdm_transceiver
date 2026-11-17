#!/usr/bin/env python3
"""Sweep the board's radio-mic level with real modem frames, off the air.

    ./venv/bin/python tools/baofeng/mic_sweep.py --dir /tmp/sweep

Per `mic_gain` level: the HackRF sends a one-fragment text message
(`modem_frame.py --text`, EXTREME, a fresh 2-bit seq each time so the
board never takes it for a duplicate) into the radio the board listens
to; the board delivers it over USB and keys its acknowledgement through
its own radio; the HackRF records that reply and `reply_decode.py`
decodes it with the boards' receiver. One table at the end: deviation,
and the decoded mode and SNR, flat and de-emphasised.

The board gets `ptt_lead 1500` for the run: the HackRF needs ~1 s to
switch from TX to RX, and a reply recorded without the start of its
preamble decodes as a bad channel (see reply_decode.py). Restored to 0
afterwards, mic_gain to `--restore-gain`. Run from the repository root.
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "host"))
sys.path.insert(0, str(REPO / "tools/baofeng"))
import ofdm_modem as om                      # noqa: E402
from reply_decode import decode              # noqa: E402

ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
ap.add_argument("--dir", required=True, help="where frames and recordings go")
ap.add_argument("--serial", default="320047000851333438363436",
                help="the board wired to the radio (default: board1)")
ap.add_argument("--freq", default="479.975e6")
ap.add_argument("--levels", default="0,-3,-6,-9,-12,-15,-18")
ap.add_argument("--text", default="hello 73")
ap.add_argument("--restore-gain", type=int, default=-9)
a = ap.parse_args()
D = Path(a.dir)
D.mkdir(parents=True, exist_ok=True)
LEVELS = [int(v) for v in a.levels.split(",")]
py = sys.executable
for q in range(4):
    subprocess.run([py, str(REPO / "tools/baofeng/modem_frame.py"), "--mode", "extreme",
                    "--text", a.text, "--seq", str(q), "-o", str(D / f"msg_s{q}.npy")],
                   check=True, stdout=subprocess.DEVNULL)
t0 = time.time()


def log(s):
    print(f"[{time.time() - t0:6.1f} s] {s}", flush=True)


def poll(m, until):
    while not until():
        for name, p in m.events(timeout=0.3):
            if name == "message":
                log(f"  delivered {p['data']!r}")


with om.OfdmModem(serial=a.serial) as m:
    m.config("rx_input", 1)
    m.config("ptt_lead", 1500)
    for k, g in enumerate(LEVELS):
        seq = (k + 1) % 4
        m.config("mic_gain", g)
        log(f"mic_gain {g} dB, seq {seq}")
        tx = subprocess.Popen([py, str(REPO / "tools/baofeng/nfm_tone_tx.py"), "--freq", a.freq,
                               "--audio", str(D / f"msg_s{seq}.npy")],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, cwd=REPO)
        poll(m, lambda: tx.poll() is not None)
        rx = subprocess.Popen([py, str(REPO / "tools/baofeng/ptt_carrier_check.py"),
                               "--listen", "24", "--iq", str(D / f"reply_g{-g}.npy")],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, cwd=REPO)
        poll(m, lambda: rx.poll() is not None)
    m.config("mic_gain", a.restore_gain)
    m.config("ptt_lead", 0)
    time.sleep(0.3)

print("\n mic_gain | dev rms | 99.9% peak | flat               | de-emphasised")
for g in LEVELS:
    r = decode(str(D / f"reply_g{-g}.npy"), quiet=True)
    cell = lambda v: ", ".join(f"{md} {s:.1f} dB" for md, s in r["decodes"][v]) or "NO DECODE"
    print(f" {g:+4d} dB  | {r['dev_rms']:5.0f} Hz | {r['dev_peak']:6.0f} Hz  | "
          f"{cell('flat'):18s} | {cell('de-emph')}")
