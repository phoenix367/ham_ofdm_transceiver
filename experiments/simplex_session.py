"""Whole-system test: two LinkStations on ONE simplex channel.

Both stations share a single frequency with half-duplex radios: carrier
sense before transmitting, deaf while transmitting, collisions garble both
frames, random backoff after timeouts. Traffic flows in BOTH directions with
QoS classes; the channel SNR is asymmetric and time-varying with a mid-
session fade. Every frame goes through the real PHY.

Pass criteria: every submitted message is delivered bit-exact on the far
side, higher-priority QoS classes arrive first, and the session terminates
without deadlock.

Run:  python experiments/simplex_session.py
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ofdm_phy import simulate_channel
from ofdm_phy.station import LinkStation, FS

TICK = 0.1
T_LIMIT = 900.0


def snr_to_b(t):
    # deep fade at t=130..180, right in the middle of the bulk transfer,
    # to exercise timeouts, loss fallback and recovery on a shared channel
    if t < 100:
        return -13.0 + (t / 100.0) * 9.0
    if t < 130:
        return -4.0
    if t < 180:
        return -14.0
    return min(-14.0 + (t - 180) / 60.0 * 8.0, -6.0)


def snr_to_a(t):
    return snr_to_b(t) + 2.0


MESSAGES_A = [
    (b"QRT IN 5 MIN", "control"),
    (b"TNX FER REPORT, QSL VIA BURO 73", "interactive"),
    (bytes(np.random.default_rng(7).integers(0, 256, 400, dtype=np.uint8)), "bulk"),
]
MESSAGES_B = [
    (b"UR RST 559 QSB, PSE QRS + AGN UR QTH?", "interactive"),
]


def main():
    rng = np.random.default_rng(2026)
    A = LinkStation("A", np.random.default_rng(1))
    B = LinkStation("B", np.random.default_rng(2))
    for payload, qos in MESSAGES_A:
        A.submit(payload, qos)
    for payload, qos in MESSAGES_B:
        B.submit(payload, qos)

    peers = {A: B, B: A}
    snr_of = {A: snr_to_a, B: snr_to_b}  # SNR *toward* that station
    active = []  # in-flight transmissions
    collisions = 0
    events = []

    t = 0.0
    while t < T_LIMIT:
        # --- complete transmissions
        for tx in [x for x in active if t >= x["end"]]:
            active.remove(tx)
            owner = tx["owner"]
            owner.on_tx_end(tx["end"])
            peer = peers[owner]
            if tx["collided"]:
                continue
            snr = snr_of[peer]((tx["start"] + tx["end"]) / 2)
            rx = simulate_channel(tx["sig"].astype(np.float64),
                                  int(rng.integers(300, 1200)),
                                  float(rng.uniform(-60, 60)), FS,
                                  snr_db=snr, rng=rng)
            done = peer.rx_frame(rx, tx["end"])
            for msg in done:
                events.append((tx["end"], f"{peer.name} received {len(msg)} B "
                                          f"({msg[:24]!r}{'...' if len(msg) > 24 else ''})"))
                print(f"t={tx['end']:6.1f}s  {events[-1][1]}")

        # --- poll stations (random order each tick)
        stations = [A, B] if rng.random() < 0.5 else [B, A]
        for st in stations:
            if any(x["owner"] is st for x in active):
                continue  # transmitting, radio deaf
            busy = len(active) > 0
            sig = st.poll_tx(t, busy)
            if sig is not None:
                new = {"owner": st, "start": t, "end": t + len(sig) / FS,
                       "sig": sig, "collided": False}
                for other in active:  # simultaneous keying -> both garbled
                    other["collided"] = True
                    new["collided"] = True
                if new["collided"]:
                    collisions += 1
                active.append(new)

        if (len(B.delivered) == len(MESSAGES_A)
                and len(A.delivered) == len(MESSAGES_B)
                and not active and not A.has_traffic() and not B.has_traffic()):
            break
        t += TICK

    # --- verification
    print(f"\nsession ended at t={t:.1f} s, collisions: {collisions}")
    for st in (A, B):
        s = st.stats
        print(f"{st.name}: tx {s.tx_frames} frames, rx {s.rx_frames}, "
              f"retransmissions {s.retransmissions}, timeouts {s.timeouts}")

    ok = True
    # priority order: control before interactive before bulk
    expect_b = [m[0] for m in sorted(MESSAGES_A,
                                     key=lambda m: ["control", "interactive", "bulk"].index(m[1]))]
    if B.delivered == expect_b:
        print("PASS: B received all of A's messages, bit-exact, in QoS priority order")
    else:
        print(f"FAIL: B.delivered mismatch ({len(B.delivered)} msgs)")
        ok = False
    if A.delivered == [m[0] for m in MESSAGES_B]:
        print("PASS: A received B's message bit-exact")
    else:
        print(f"FAIL: A.delivered mismatch ({len(A.delivered)} msgs)")
        ok = False

    total_payload = sum(len(m[0]) for m in MESSAGES_A + MESSAGES_B)
    print(f"total payload both directions: {total_payload} B in {t:.0f} s "
          f"-> {8 * total_payload / t:.1f} bit/s effective (shared channel)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
