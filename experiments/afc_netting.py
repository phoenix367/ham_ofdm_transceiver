"""AFC / frequency netting: the receiver measures the peer's carrier offset
per frame and requests a correction in the link-control word (5-bit field,
+-120 Hz per frame, 8 Hz steps); the peer trims its shared reference, damped
by 0.5 because both sides act on stale measurements. Larger offsets converge
iteratively; ongoing drift is tracked by repeated small corrections.

Scenario: two worn-TCXO rigs on 14.2 MHz -- +12 ppm vs -8 ppm -> +284 Hz
initial offset (near the modem's +-375 Hz limit) plus opposing thermal
drifts (+0.05 / -0.04 Hz/s).

Run:  python experiments/afc_netting.py
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ofdm_phy.rf import StationRF, rf_link, expected_cfo_hz
from ofdm_phy.station import LinkStation, FS

RF_A = StationRF("A", ppm=+12.0, drift_hz_per_s=+0.05, rf_carrier_hz=14.2e6)
RF_B = StationRF("B", ppm=-8.0, drift_hz_per_s=-0.04, rf_carrier_hz=14.2e6)


def run(limit_a, limit_b, anchor_a=False, label="", quiet=False):
    global RF_A, RF_B
    RF_A = StationRF("A", ppm=+12.0, drift_hz_per_s=+0.05, rf_carrier_hz=14.2e6)
    RF_B = StationRF("B", ppm=-8.0, drift_hz_per_s=-0.04, rf_carrier_hz=14.2e6)
    rng = np.random.default_rng(7)
    A = LinkStation("A", np.random.default_rng(1),
                    freq_trim_cb=lambda hz: setattr(RF_A, "trim_hz", RF_A.trim_hz + hz),
                    afc_max_trim_hz=limit_a, afc_anchor=anchor_a)
    B = LinkStation("B", np.random.default_rng(2),
                    freq_trim_cb=lambda hz: setattr(RF_B, "trim_hz", RF_B.trim_hz + hz),
                    afc_max_trim_hz=limit_b)
    A.submit(bytes(rng.integers(0, 256, 200, dtype=np.uint8)), "bulk")

    rf_of = {A: RF_A, B: RF_B}
    peers = {A: B, B: A}
    active = None
    t = 0.0
    cfos = []
    trim_violation = False
    while t < 420 and (A.has_traffic() or active):
        if active and t >= active["end"]:
            st, sig, end = active["owner"], active["sig"], active["end"]
            st.on_tx_end(end)
            peer = peers[st]
            rx = rf_link(sig.astype(np.float64), rf_of[st], rf_of[peer], t0=active["start"],
                         snr_db=-4.0, fading_doppler_hz=0.15,
                         bit_flip_prob=0.001, erasure_prob=0.02, rng=rng)
            n0 = peer.stats.rx_frames
            peer.rx_frame(rx, end)
            if peer.stats.rx_frames > n0:
                cfos.append((end, abs(peer._last_cfo_hz)))
            if abs(RF_A.trim_hz) > limit_a + 1e-6 or abs(RF_B.trim_hz) > limit_b + 1e-6:
                trim_violation = True
            active = None
        if active is None:
            for st in (A, B):
                sig = st.poll_tx(t, False)
                if sig is not None:
                    active = {"owner": st, "sig": sig, "start": t, "end": t + len(sig) / FS}
                    break
        t += 0.1

    delivered = sum(len(m) for m in B.delivered)
    final = [c for _, c in cfos[-3:]]  # end-of-session state
    print(f"[{label}] delivered {delivered}/200 B; trims A {RF_A.trim_hz:+.0f} "
          f"(limit {limit_a:.0f}{', ANCHOR' if anchor_a else ''}) / "
          f"B {RF_B.trim_hz:+.0f} (limit {limit_b:.0f}); "
          f"final |cfo| {np.mean(final):.1f} Hz; budget respected: {not trim_violation}")
    return delivered == 200 and not trim_violation, float(np.mean(final))


def main():
    rng = np.random.default_rng(7)
    A = LinkStation("A", np.random.default_rng(1),
                    freq_trim_cb=lambda hz: setattr(RF_A, "trim_hz", RF_A.trim_hz + hz))
    B = LinkStation("B", np.random.default_rng(2),
                    freq_trim_cb=lambda hz: setattr(RF_B, "trim_hz", RF_B.trim_hz + hz))
    A.submit(bytes(rng.integers(0, 256, 400, dtype=np.uint8)), "bulk")

    initial_offset = expected_cfo_hz(RF_A, RF_B, 0)
    rf_of = {A: RF_A, B: RF_B}
    peers = {A: B, B: A}
    active = None
    t = 0.0
    print(f"initial offset A->B: {expected_cfo_hz(RF_A, RF_B, 0):+.1f} Hz "
          f"(drifts {RF_A.drift_hz_per_s:+.2f}/{RF_B.drift_hz_per_s:+.2f} Hz/s)\n")
    print(f"{'t, s':>7} {'dir':>5} {'measured cfo':>13} {'trim A':>8} {'trim B':>8} "
          f"{'true offset':>12}")

    last_abs_cfo = []
    while t < 500 and (A.has_traffic() or active):
        if active and t >= active["end"]:
            st, sig, end = active["owner"], active["sig"], active["end"]
            st.on_tx_end(end)
            peer = peers[st]
            rx = rf_link(sig.astype(np.float64), rf_of[st], rf_of[peer], t0=active["start"],
                         snr_db=-4.0, fading_doppler_hz=0.15,
                         bit_flip_prob=0.001, erasure_prob=0.02, rng=rng)
            n0 = peer.stats.rx_frames
            peer.rx_frame(rx, end)
            if peer.stats.rx_frames > n0:
                cfo = peer._last_cfo_hz
                print(f"{end:7.1f} {st.name+'->'+peer.name:>5} {cfo:>+12.1f}H "
                      f"{RF_A.trim_hz:>+7.1f}H {RF_B.trim_hz:>+7.1f}H "
                      f"{expected_cfo_hz(RF_A, RF_B, end):>+11.1f}H")
                last_abs_cfo.append((end, abs(cfo)))
            active = None
        if active is None:
            for st in (A, B):
                sig = st.poll_tx(t, False)
                if sig is not None:
                    active = {"owner": st, "sig": sig, "start": t, "end": t + len(sig) / FS}
                    break
        t += 0.1

    delivered = sum(len(m) for m in B.delivered)
    # convergence: from the first measurement inside the AFC deadband onward,
    # the offset must never leave a small band again (drift is being tracked)
    conv_t = next((tt for tt, c in last_abs_cfo if c < LinkStation.AFC_DEADBAND_HZ), None)
    tail = [c for tt, c in last_abs_cfo if conv_t is not None and tt >= conv_t]
    steady = [c for tt, c in last_abs_cfo if tt >= t - 60]
    print(f"\ndelivered {delivered}/400 B in {t:.0f} s; "
          f"AFC trims applied: A {getattr(A.stats, 'afc_trims', 0)}, "
          f"B {getattr(B.stats, 'afc_trims', 0)}")
    print(f"netted at t={conv_t:.0f} s (initial offset {initial_offset:+.0f} Hz); "
          f"settling max {max(tail):.1f} Hz, steady-state (last 60 s) max {max(steady):.1f} Hz")
    ok = (delivered == 400 and conv_t is not None
          and max(tail) < 40.0 and max(steady) < 15.0)
    print("PASS: link netted and stayed netted under drift" if ok else "FAIL")

    # --- trim-budget guard scenarios
    print("\n=== hard trim budget ===")
    ok2, cfo2 = run(150.0, 150.0, label="default budgets 150/150")
    ok3, cfo3 = run(40.0, 40.0, label="tight budgets 40/40  ")
    ok4, cfo4 = run(0.0, 300.0, anchor_a=True, label="A anchored, B nets   ")
    # tight budgets can only correct 2*40=80 of ~284 Hz -> residual stays,
    # but the budget must hold and the link must still deliver
    guard_ok = ok2 and ok3 and ok4 and cfo2 < 20 and cfo3 > 100 and cfo4 < 25
    print("PASS: budgets respected; tight budget leaves residual (by design); "
          "anchor pins the frequency" if guard_ok else "FAIL (guard scenarios)")
    sys.exit(0 if (ok and guard_ok) else 1)


if __name__ == "__main__":
    main()
