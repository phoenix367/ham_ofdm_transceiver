"""FullOFDMModem (article's Newman comb, FFT-peak CFO estimator) vs
STFOFDMModem (802.11-STF-style comb, tones every 8 bins, delay-and-correlate
CFO estimator) over the full +-300 Hz CFO range claimed by the protocol.

Run:  python experiments/stf_vs_newman.py
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.signal import hilbert
from ofdm_phy import (Transceiver, Data, ModType, CCSpeed, simulate_channel,
                      FullOFDMModem, STFOFDMModem)

fs = 12000
PRE_LEN = 4384
N_TRIALS = 150

for name, modem_cls in (("Newman/FFT-peak", FullOFDMModem), ("STF/delay-corr", STFOFDMModem)):
    trx = Transceiver(modem_cls())
    for snr in (-7, -8, -9):
        rng = np.random.default_rng(99)
        det_fail = bad_t = ok = 0
        cfo_err = []
        t_det = 0.0
        for _ in range(N_TRIALS):
            pkt = Data(reserved=123, payload=rng.bytes(27))
            ts = int(rng.integers(200, 1500))
            cf = float(rng.uniform(-300, 300))
            sig = trx.build_frame(pkt, mod=ModType.BPSK, spd=CCSpeed.R13)
            rx = simulate_channel(sig, ts, cf, fs, snr_db=snr, rng=rng)
            x = hilbert(rx).astype(np.complex64)
            t0 = time.perf_counter()
            det = trx.modem.detect_preamble(x)
            t_det += time.perf_counter() - t0
            if det is None:
                det_fail += 1
                continue
            if abs(det[0] - (ts + PRE_LEN)) > 8:
                bad_t += 1
            cfo_err.append(abs(det[1] - cf))
            try:
                dec, _ = trx.demod_frame(rx, check_crc=False)
                ok += dec == pkt
            except Exception:
                pass
        ce = np.array(cfo_err)
        print(f"{name:16s} SNR {snr:+d}: det_fail {det_fail:3d}  bad_timing {bad_t:3d}  "
              f"cfo med/p90 {np.median(ce):5.2f}/{np.percentile(ce, 90):6.2f} Hz  "
              f"decoded {ok:3d}/{N_TRIALS}  ({1e3 * t_det / N_TRIALS:.1f} ms/detect)")
    print()
