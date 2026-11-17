#!/usr/bin/env python3
"""Build one OFDM link frame as 12 kHz audio, for nfm_tone_tx.py --audio.

    ./venv/bin/python tools/baofeng/modem_frame.py --mode extreme -o frame.npy
    ./venv/bin/python tools/baofeng/nfm_tone_tx.py --freq 479.975e6 --audio frame.npy

The frame is what a peer STATION would send: a Data packet whose 20
reserved bits are a link-control word. That matters because the radio
firmware hands every decoded frame to the station as a Data frame -- a
Beacon packet would be read as a garbage control word. The default is a
NO_DATA frame (flags 2): the station counts it and measures its SNR
(beacon rx_decodes / last_snr_q8) but does not reply, so nothing keys.
`--text` sends a real one-fragment message instead; the station then
delivers it AND transmits an acknowledgement (PTT on PB14; audio on PA4 for the wire
and PA5 for a radio's mic, see docs/console.md).

An idle board listens on EXTREME only (follow_rung), so that is the
default mode; NORMAL/ROBUST frames are only detected once the board has
decoded something and raised its request.
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from ofdm_phy.link import LinkControl                         # noqa: E402
from ofdm_phy.modes import LinkMode, make_modem               # noqa: E402
from ofdm_phy.packets import CCSpeed, Data, ModType           # noqa: E402
from ofdm_phy.transceiver import Transceiver                  # noqa: E402

FLAG_LAST_FRAGMENT, FLAG_NO_DATA = 1, 2    # cport/src/station.h

ap = argparse.ArgumentParser()
ap.add_argument("--mode", choices=["normal", "robust", "extreme"], default="extreme")
ap.add_argument("--mod", choices=["bpsk", "qpsk"], default="bpsk")
ap.add_argument("--rate", choices=["1/3", "1/2", "2/3", "3/4"], default="1/3")
ap.add_argument("--text", default=None,
                help="send this as a message (the board will REPLY and key PTT)")
ap.add_argument("--seq", type=int, default=1, choices=range(4),
                help="the message's 2-bit sequence number; repeat sends need a NEW "
                     "one or the board treats the frame as a duplicate (default 1)")
ap.add_argument("--net-key", type=int, default=0)
ap.add_argument("-o", "--out", required=True, help=".npy, float, 12 kHz")
a = ap.parse_args()

mode = {"normal": LinkMode.NORMAL, "robust": LinkMode.ROBUST,
        "extreme": LinkMode.EXTREME}[a.mode]
mod = {"bpsk": ModType.BPSK, "qpsk": ModType.QPSK}[a.mod]
spd = {"1/3": CCSpeed.R13, "1/2": CCSpeed.R12, "2/3": CCSpeed.R23,
       "3/4": CCSpeed.R34}[a.rate]
if a.text is None:
    lc, payload = LinkControl(seq=0, ack=0, flags=FLAG_NO_DATA), b""
else:
    lc, payload = LinkControl(seq=a.seq, ack=0, flags=FLAG_LAST_FRAGMENT), a.text.encode()
m = make_modem(mode)
x = Transceiver(modem=m, net_key=a.net_key).build_frame(
    Data(reserved=lc.pack(), payload=payload), mod=mod, spd=spd)
x = (x / np.max(np.abs(x))).astype(np.float32)
np.save(a.out, x)
print(f"{a.out}: {a.mode.upper()} {a.mod.upper()} {a.rate}, "
      f"{'NO_DATA' if a.text is None else f'{a.text!r} seq {a.seq}'}, "
      f"{len(x)} samples = {len(x) / m.sample_rate:.2f} s at {m.sample_rate} Hz, "
      f"peak-normalised")
