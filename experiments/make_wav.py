"""Record modem frames to WAV files (12 kHz, 16-bit mono) and verify them by
decoding the WAV back.

  results/extreme_cq.wav        clean TX frame -- feed this to a transceiver's
                                audio input (AGC off, as the article requires)
  results/extreme_cq_-18dB.wav  the same frame through the simulated channel
                                at -18 dB SNR -- what an RX recording sounds
                                like at the EXTREME sensitivity limit

Run:  python experiments/make_wav.py [--mode EXTREME] [--text "CQ CQ de R9FEU"]
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.io import wavfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ofdm_phy import (
    Transceiver, Data, ModType, CCSpeed, LinkMode, make_modem, simulate_channel,
)

FS = 12000


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=[m.name for m in LinkMode], default="EXTREME")
    ap.add_argument("--text", default="CQ CQ de R9FEU")
    ap.add_argument("--snr", type=float, default=-18.0, help="SNR for the impaired copy")
    args = ap.parse_args()

    mode = LinkMode[args.mode]
    results = ROOT / "results"
    results.mkdir(exist_ok=True)

    trx = Transceiver(make_modem(mode))
    packet = Data(reserved=123, payload=args.text.encode())

    # lead-in/out silence so soundcard start/stop doesn't clip the preamble
    pad = np.zeros(FS // 2, dtype=np.int16)
    frame = trx.build_frame_int16(packet, mod=ModType.BPSK, spd=CCSpeed.R13)
    clean = np.concatenate([pad, frame, pad])

    stem = f"{mode.name.lower()}_cq"
    clean_path = results / f"{stem}.wav"
    wavfile.write(clean_path, FS, clean)
    print(f"saved {clean_path}  ({len(clean) / FS:.1f} s, "
          f"{clean_path.stat().st_size / 1024:.0f} KiB)")

    # channel-impaired copy at the mode's sensitivity limit
    rng = np.random.default_rng(73)
    rx = simulate_channel(frame.astype(np.float64), time_shift=FS // 2,
                          freq_shift_hz=12.0, sample_rate=FS, snr_db=args.snr, rng=rng)
    rx16 = (rx / np.max(np.abs(rx)) * 0.9 * 32767).astype(np.int16)
    noisy_path = results / f"{stem}_{args.snr:+.0f}dB.wav"
    wavfile.write(noisy_path, FS, rx16)
    print(f"saved {noisy_path}  ({len(rx16) / FS:.1f} s)")

    # verify: read both WAVs back and decode
    for path in (clean_path, noisy_path):
        fs_read, samples = wavfile.read(path)
        assert fs_read == FS
        try:
            decoded, stats, det_mode = Transceiver().demod_frame_auto(
                samples.astype(np.float64) / 32768.0)
            ok = decoded == packet
            print(f"decode {path.name}: {'OK' if ok else 'MISMATCH'} "
                  f"[{det_mode.name}] {decoded} "
                  f"(SNR est {stats.snr_db:+.1f} dB, BER {stats.ber * 100:.2f}%)")
        except Exception as exc:
            print(f"decode {path.name}: FAILED ({exc})")


if __name__ == "__main__":
    main()
