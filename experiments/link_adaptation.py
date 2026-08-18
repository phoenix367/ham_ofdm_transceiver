"""Two-station link-adaptation simulation over a time-varying, asymmetric
channel, using the real PHY for every frame.

Station A pushes bulk data to station B; B answers with ACK-only frames.
Both piggyback the 20-bit link-control word (seq/ack/req_rung/SNR report) in
every frame. Receivers request rungs (RX-driven adaptation with hysteresis);
transmitters obey requests, decayed by staleness and cut by loss fallback.
Both directions bootstrap at rung 0 (EXTREME).

Channel timeline (SNR toward B; toward A is +3 dB):
  0..150 s   deep start -16 dB, slow improvement to -3 dB
  150..250 s good and stable (-3 dB)
  250 s      sudden fade to -12.5 dB (tests loss fallback)
  250..420 s recovery to -6 dB

Run:  python experiments/link_adaptation.py
Outputs: results/link_adaptation.png + a turn-by-turn log
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ofdm_phy import Transceiver, Data, make_modem, simulate_channel
from ofdm_phy.link import LADDER, LinkControl, LinkController, max_payload_bytes
from ofdm_phy.transceiver import DemodError

FS = 12000
TURNAROUND = 0.25
BULK_BYTES = 1500


def snr_to_b(t):
    if t < 150:
        return -16.0 + (t / 150.0) * 13.0
    if t < 250:
        return -3.0
    return min(-12.5 + (t - 250) / 170.0 * 6.5, -6.0)


def snr_to_a(t):
    return snr_to_b(t) + 3.0  # asymmetric link


class Station:
    def __init__(self, name, rng):
        self.name = name
        self.rng = rng
        self.ctl = LinkController()
        self.seq = 0
        self.last_rx_seq = 0
        self.queue = b""
        self.pending = None  # chunk awaiting ack
        self.delivered = 0
        self.rx_bytes = b""

    def build(self, now):
        rung_idx = self.ctl.tx_rung(now)
        rung = LADDER[rung_idx]
        if self.pending is None and self.queue:
            n = max_payload_bytes(rung)
            self.pending, self.queue = self.queue[:n], self.queue[n:]
            self.seq = (self.seq + 1) & 3
        payload = self.pending if self.pending is not None else b"\x00"

        lc = LinkControl(seq=self.seq, ack=self.last_rx_seq,
                         req_rung=self.ctl.rx_request(now),
                         snr_db=self.ctl.filtered_snr(now))
        pkt = Data(reserved=lc.pack(), payload=payload)
        trx = Transceiver(make_modem(rung.mode))
        sig = trx.build_frame(pkt, mod=rung.mod, spd=rung.spd)
        return sig, rung_idx, lc, len(payload)

    def on_reply(self, lc_or_none):
        """ARQ: did the peer's link-control word acknowledge my seq?"""
        if lc_or_none is not None and lc_or_none.ack == self.seq:
            if self.pending is not None:
                self.delivered += len(self.pending)
                self.pending = None
            self.ctl.on_ack()
        else:
            self.ctl.on_timeout()


def transmit(sig, snr_db, rng):
    rx = simulate_channel(sig, int(rng.integers(300, 1200)),
                          float(rng.uniform(-60, 60)), FS, snr_db=snr_db, rng=rng)
    try:
        pkt, stats, mode = Transceiver().demod_frame_auto(rx)
        return pkt, stats
    except DemodError:
        return None, None


def main():
    rng = np.random.default_rng(42)
    A = Station("A", rng)
    B = Station("B", rng)
    A.queue = bytes(rng.integers(0, 256, BULK_BYTES, dtype=np.uint8))

    t = 0.0
    log = []  # (t, dir, rung, snr_true, ok, bytes)
    turn = 0
    while t < 420 and (A.queue or A.pending is not None):
        # --- A transmits
        sig, rung_a, lc_a, nbytes = A.build(t)
        air = len(sig) / FS
        snr = snr_to_b(t + air / 2)
        pkt, stats = transmit(sig, snr, rng)
        ok = pkt is not None
        if ok:
            lc = LinkControl.unpack(pkt.reserved)
            B.ctl.on_rx_frame(stats.snr_db, lc, t)
            B.last_rx_seq = lc.seq
            B.on_reply(lc)  # piggybacked ack for B's previous frame
        log.append((t, "A->B", rung_a, snr, ok, nbytes))
        t += air + TURNAROUND

        # --- B replies (ack-only)
        sig, rung_b, lc_b, _ = B.build(t)
        air = len(sig) / FS
        snr = snr_to_a(t + air / 2)
        pkt, stats = transmit(sig, snr, rng)
        ok_b = pkt is not None
        if ok_b:
            lc = LinkControl.unpack(pkt.reserved)
            A.ctl.on_rx_frame(stats.snr_db, lc, t)
            A.last_rx_seq = lc.seq
            A.on_reply(lc)
        else:
            A.on_reply(None)
        log.append((t, "B->A", rung_b, snr, ok_b, 0))
        t += air + TURNAROUND
        turn += 1

        if turn % 10 == 0:
            print(f"t={t:6.1f}s  A rung {rung_a} ({'ok' if ok else 'LOST'})  "
                  f"B rung {rung_b} ({'ok' if ok_b else 'LOST'})  "
                  f"delivered {A.delivered}/{BULK_BYTES} B")

    print(f"\nfinished at t={t:.1f} s: delivered {A.delivered}/{BULK_BYTES} bytes, "
          f"{turn} turns, goodput {8 * A.delivered / t:.1f} bit/s "
          f"(incl. ACK overhead and turnarounds)")
    losses = sum(1 for e in log if not e[4])
    print(f"frame losses: {losses}/{len(log)}")

    plot(log, t)


def plot(log, t_end):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    tt = np.linspace(0, t_end, 500)
    axes[0].plot(tt, [snr_to_b(x) for x in tt], label="SNR toward B (A->B link)")
    axes[0].plot(tt, [snr_to_a(x) for x in tt], label="SNR toward A (B->A link)")
    for r in LADDER:
        axes[0].axhline(r.sens_db, color="gray", lw=0.4, alpha=0.5)
    axes[0].set_ylabel("SNR, dB")
    axes[0].legend(fontsize=9)
    axes[0].set_title("Two-station link adaptation (rung sensitivities in gray)")

    for d, color in (("A->B", "tab:blue"), ("B->A", "tab:orange")):
        ev = [(e[0], e[2], e[4]) for e in log if e[1] == d]
        axes[1].step([e[0] for e in ev], [e[1] for e in ev],
                     where="post", color=color, label=f"{d} rung")
    # mark losses
    for d, color in (("A->B", "tab:blue"), ("B->A", "tab:orange")):
        lost = [(e[0], e[2]) for e in log if e[1] == d and not e[4]]
        if lost:
            axes[1].scatter([x[0] for x in lost], [x[1] for x in lost],
                            marker="x", color="red", zorder=3,
                            label=f"{d} lost" if d == "A->B" else None)
    axes[1].set_ylabel("ladder rung")
    axes[1].set_yticks(range(len(LADDER)))
    axes[1].legend(fontsize=9)
    axes[1].grid(alpha=0.3)

    tot = 0
    xs, ys = [0], [0]
    for e in log:
        if e[1] == "A->B" and e[4]:
            tot += e[5]
            xs.append(e[0])
            ys.append(tot)
    axes[2].step(xs, ys, where="post")
    axes[2].set_ylabel("delivered bytes")
    axes[2].set_xlabel("time, s")
    axes[2].grid(alpha=0.3)

    fig.tight_layout()
    out = ROOT / "results" / "link_adaptation.png"
    fig.savefig(out, dpi=130)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
