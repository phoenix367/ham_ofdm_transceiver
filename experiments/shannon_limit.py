"""Reproduce Tables 1 and 2: Shannon-Hartley channel capacity for the 2100 Hz
band and protocol efficiency vs the Shannon limit.

Run:  python experiments/shannon_limit.py
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ofdm_phy import Transceiver, ModType

BANDWIDTH = 2400 - 300  # Hz
NYQUIST_CAP = 2 * BANDWIDTH  # upper bound, bit/s


def shannon_limit(snr_db: float) -> float:
    snr_lin = 10 ** (snr_db / 10)
    return BANDWIDTH * np.log2(1 + snr_lin)


def main():
    trx = Transceiver()
    rate_bpsk = trx.channel_bit_rate(ModType.BPSK)
    rate_qpsk = trx.channel_bit_rate(ModType.QPSK)

    sym_duration = trx.modem.symbol_len / trx.modem.sample_rate
    print(f"OFDM symbol duration (CP + 4 tiles): {sym_duration:.4f} s")
    print(f"Channel rate BPSK: {rate_bpsk:.4f} bit/s (article: 353.9823)")
    print(f"Channel rate QPSK: {rate_qpsk:.4f} bit/s (article: 707.9646)")

    print("\nTable 1 - Shannon limit vs SNR (2100 Hz band)")
    print(f"{'SNR (dB)':>8} | {'SNR (lin)':>9} | {'Shannon limit (bit/s)':>22}")
    for snr in range(5, -10, -1):
        lim = shannon_limit(snr)
        note = f" (max: {NYQUIST_CAP:.2f})" if lim > NYQUIST_CAP else ""
        print(f"{snr:>8} | {10 ** (snr / 10):>9.4f} | {lim:>22.2f}{note}")

    print("\nTable 2 - protocol efficiency vs the Shannon limit")
    print(f"{'SNR (dB)':>8} | {'Limit':>8} | {'Mod':>4} | {'Rate (bit/s)':>12} | {'Eff (%)':>7}")
    rows = []
    for snr in range(5, -10, -1):
        lim = min(shannon_limit(snr), NYQUIST_CAP)
        mod = ModType.QPSK if snr >= 0 else ModType.BPSK
        rate = rate_qpsk if mod is ModType.QPSK else rate_bpsk
        eff = 100 * rate / lim
        rows.append((snr, lim, mod.name, rate, eff))
        print(f"{snr:>8} | {lim:>8.2f} | {mod.name:>4} | {rate:>12.2f} | {eff:>7.2f}")

    print("\nUser data rates after FEC:")
    from ofdm_phy import CCSpeed
    for mod in ModType:
        rates = [trx.data_bit_rate(mod, spd) for spd in CCSpeed]
        print(f"  {mod.name}: " + ", ".join(f"{s.name}={r:.0f}" for s, r in zip(CCSpeed, rates)) + " bit/s")


if __name__ == "__main__":
    main()
