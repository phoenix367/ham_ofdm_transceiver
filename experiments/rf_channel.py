"""Validate the RF layer: SSB modulator/demodulator transparency, physically
emerging CFO from per-station LO errors, and decoding through the full RF
chain with fading and noise.

Run:  python experiments/rf_channel.py
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ofdm_phy import Transceiver, Data, ModType, CCSpeed, LinkMode, make_modem
from ofdm_phy.rf import StationRF, rf_link, expected_cfo_hz

PASSED = 0
FAILED = 0


def check(name, ok, detail=""):
    global PASSED, FAILED
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))
    PASSED += ok
    FAILED += not ok


def main():
    rng = np.random.default_rng(17)
    trx = Transceiver(make_modem(LinkMode.NORMAL))
    pkt = Data(reserved=123, payload=b"CQ CQ de R9FEU")
    sig = trx.build_frame(pkt, mod=ModType.BPSK, spd=CCSpeed.R13)

    # --- (a) transparency: perfect LOs, clean channel ----------------------
    perfect = StationRF("P", ppm=0.0)
    audio = rf_link(sig, perfect, perfect, 0.0, snr_db=40.0,
                    channel_response=np.array([1.0]), rng=rng)
    dec, st = trx.demod_frame(audio)
    check("SSB mod/demod transparency (clean decode)", dec == pkt,
          f"CFO est {st.cfo_hz:+.2f} Hz, BER {st.ber * 100:.2f}%")
    check("transparency CFO ~ 0", abs(st.cfo_hz) < 2.0, f"{st.cfo_hz:+.2f} Hz")

    # --- (b) CFO emerges from LO ppm errors --------------------------------
    cases = [
        (7.1e6, +6.0, -8.0),    # 80/40m-class rig errors -> ~ +99 Hz
        (7.1e6, -11.0, +4.0),   # -> ~ -107 Hz
        (14.2e6, +12.0, -8.0),  # 20 m, worn TCXO -> ~ +284 Hz (near limit)
    ]
    for f_rf, ppm_a, ppm_b in cases:
        a = StationRF("A", ppm=ppm_a, rf_carrier_hz=f_rf)
        b = StationRF("B", ppm=ppm_b, rf_carrier_hz=f_rf)
        exp = expected_cfo_hz(a, b, 0.0)
        audio = rf_link(sig, a, b, 0.0, snr_db=30.0,
                        channel_response=np.array([1.0]), rng=rng)
        try:
            dec, st = trx.demod_frame(audio)
            ok = dec == pkt and abs(st.cfo_hz - exp) < 5.0
            check(f"LO offset {f_rf/1e6:.1f} MHz {ppm_a:+.0f}/{ppm_b:+.0f} ppm", ok,
                  f"expected {exp:+.1f} Hz, measured {st.cfo_hz:+.1f} Hz")
        except Exception as exc:
            check(f"LO offset {f_rf/1e6:.1f} MHz {ppm_a:+.0f}/{ppm_b:+.0f} ppm",
                  False, repr(exc))

    # --- (c) full RF chain: noise, fading, multipath, drift ----------------
    a = StationRF("A", ppm=+6.0, drift_hz_per_s=+0.03)
    b = StationRF("B", ppm=-8.0, drift_hz_per_s=-0.02)
    ok = 0
    n = 10
    for i in range(n):
        p = Data(reserved=123, payload=rng.bytes(27))
        s = trx.build_frame(p, mod=ModType.BPSK, spd=CCSpeed.R13)
        audio = rf_link(s, a, b, t0=30.0 * i, snr_db=-5.0,
                        fading_doppler_hz=0.1,
                        bit_flip_prob=0.001, erasure_prob=0.02, rng=rng)
        try:
            d, st = trx.demod_frame(audio, check_crc=False)
            ok += d == p
        except Exception:
            pass
    check("full RF chain @ -5 dB, fading + drift", ok >= 7, f"{ok}/{n} decoded")

    print(f"\n{PASSED} passed, {FAILED} failed")
    sys.exit(1 if FAILED else 0)


if __name__ == "__main__":
    main()
