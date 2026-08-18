"""On-air test simulation with SNR-adaptive link modes.

The channel degrades over three segments (-5 dB -> -10 dB -> -17.5 dB). For
each segment the transmitter picks the link mode, modulation and coding rate
with select_mode(); the receiver decodes the single continuous audio stream
blind -- demod_frame_auto() identifies each frame's mode from its Zadoff-Chu
preamble root -- and logs packets in the article's decoder-log format.

Run:  python experiments/ota_demo.py
"""

import logging
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ofdm_phy import (
    Transceiver, Beacon, Data, LinkMode, make_modem, select_mode, simulate_channel,
)
from ofdm_phy.transceiver import CODECS, MAPPERS, DemodError

logging.basicConfig(format="%(asctime)s [%(levelname)7s] %(message)s",
                    datefmt="%H:%M:%S", level=logging.INFO)
log = logging.getLogger(__name__)

FS = 12000


def main():
    rng = np.random.default_rng(2026)

    beacon = Beacon(callsign="R9FEU", qth="LO88CA")
    segments = [
        # (channel SNR, packets for this segment)
        (-5.0, [beacon, Data(reserved=123, payload=b"    Though this be madness,")]),
        (-10.0, [beacon, Data(reserved=123, payload=b" yet there is method in't. ")]),
        (-17.5, [Data(reserved=123, payload=b"CQ CQ de R9FEU")]),
    ]

    # --- transmitter: pick the mode per segment, build one audio stream
    gap = np.zeros(int(0.3 * FS))
    rx_parts = []
    sent = 0
    for snr_db, packets in segments:
        mode, mod, spd = select_mode(snr_db)
        trx = Transceiver(make_modem(mode))
        rate = trx.data_bit_rate(mod, spd)
        log.info(f"TX segment @ {snr_db:+.1f} dB -> {mode.name} {mod.name} "
                 f"{['1/3', '1/2', '2/3', '3/4'][spd.value]} ({rate:.1f} bit/s)")

        tx_stream = np.concatenate(
            [np.concatenate([trx.build_frame(p, mod=mod, spd=spd), gap]) for p in packets])
        sent += len(packets)

        rx_parts.append(simulate_channel(
            tx_stream, time_shift=int(rng.integers(500, 2000)), freq_shift_hz=float(rng.uniform(-80, 80)),
            sample_rate=FS, snr_db=snr_db, rng=rng))

    rx_stream = np.concatenate(rx_parts)
    log.info(f"RX stream: {len(rx_stream) / FS:.1f} s of audio, {sent} packets sent")

    # --- receiver: blind windowed decode, mode auto-detected per frame
    def decode_earliest(chunk):
        """demod_frame_auto locks onto the best frame in the window; keep
        rescanning the region before it until no earlier frame is found."""
        found = None
        limit = len(chunk)
        while True:
            try:
                packet, stats, mode = Transceiver().demod_frame_auto(chunk[:limit])
            except DemodError:
                return found
            found = (packet, stats, mode)
            modem = make_modem(mode)
            preamble_len = 3 * modem._newman_preamble_tile * modem.fft_bins + modem.symbol_len
            new_limit = stats.start_sample - preamble_len
            if new_limit <= FS // 2:
                return found
            limit = new_limit

    window = int(40 * FS)  # covers the longest (EXTREME) frame
    decoded = 0
    pos = 0
    while pos < len(rx_stream) - FS // 2:
        chunk = rx_stream[pos:pos + window]
        result = decode_earliest(chunk)
        if result is None:
            log.warning("Demod failed: no frame in window")
            pos += window // 3
            continue
        packet, stats, mode = result

        decoded += 1
        log.info(f"Mode: {mode.name}")
        log.info(f"Packet: {packet}")
        log.info("Es/N0 carriers: " + ", ".join(f"{v:.3f}dB" for v in stats.es_n0_carriers_db))
        log.info(f"Es/N0: {stats.es_n0_db:.3f}dB")
        log.info(f"SNR: {stats.snr_db:.3f}dB")
        log.info(f"BER: {stats.ber * 100:.2f}%")

        # advance to the end of the decoded frame
        modem = make_modem(mode)
        hdr = stats.header
        mapper = MAPPERS[hdr.mod]
        coded_len = CODECS[hdr.spd].calc_cc_elements(hdr.len)
        n_data_syms = -(-coded_len // (modem.data_carriers_len * mapper.MU))
        pos += stats.start_sample + (6 + n_data_syms) * modem.symbol_len

    print(f"\nDecoded {decoded}/{sent} packets")


if __name__ == "__main__":
    main()
