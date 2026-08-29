#!/usr/bin/env python3
"""Pull the analog loopback stand's buffers off the board and characterise
the path: what the DAC played, what the ADC heard, and the gain, delay and
noise between them -- independently of whether the decoder liked it.

    ./bench/analog_loop_dump.py              # after a run of analog_loop.elf

Reads the beacon at 0x20000000 for the buffer addresses and lengths,
dump_image's both over JTAG, then fits cap ~ g * play(n - d) + dc over
the frame region. The residual after that fit is everything the analog
path added: quantisation, buffer noise, pickup, distortion. Its power
against the fitted signal is the loop's SNR, which is the number that
decides whether a NORMAL / ROBUST / EXTREME frame can survive it.
"""
import os
import struct
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "..", "..", "tools", "esp32-probe", "stm32h7-rbb.cfg")
OUT = "/tmp/analog_loop"
os.makedirs(OUT, exist_ok=True)

FIELDS = ["magic", "stage", "tim_hz", "fs_mhz", "adc_ldo", "adc_cal",
          "adc_rdy", "adc_conv_ns", "n_play", "n_cap", "isr_count",
          "isr_max_cyc", "cap_min", "cap_max", "cap_mean", "play_addr",
          "cap_addr", "ev_type", "ev_start", "ev_cfo", "bits_ok", "fault",
          "icsr", "cfsr", "hfsr", "stray_irq", "stray_count"]
SIGNED = {"cap_min", "cap_max", "cap_mean", "ev_type", "ev_start", "ev_cfo",
          "bits_ok"}


def openocd(*cmds):
    argv = ["openocd", "-f", CFG, "-c", "init"]
    for c in cmds:
        argv += ["-c", c]
    argv += ["-c", "exit"]
    # openocd writes its log -- including mdw's output -- to stderr
    r = subprocess.run(argv, capture_output=True, text=True, timeout=600)
    return r.stdout + "\n" + r.stderr + "\n"


def read_beacon():
    out = openocd(f"mdw 0x20000000 {len(FIELDS)}")
    words = []
    for line in out.splitlines():
        if line.startswith("0x2000"):
            words += [int(x, 16) for x in line.split()[1:]]
    b = {}
    for k, v in zip(FIELDS, words):
        b[k] = v - (1 << 32) if (k in SIGNED and v >= (1 << 31)) else v
    return b


def main():
    b = read_beacon()
    if b.get("magic") != 0xA10C0DE5:
        print(f"no beacon (magic {b.get('magic', 0):#x}) -- is analog_loop.elf running?")
        return 1
    n = b["n_cap"]
    print(f"beacon: stage {b['stage']}  fs {b['fs_mhz']/1000:.3f} Hz  "
          f"adc conv {b['adc_conv_ns']/1000:.1f} us  n_cap {n}  "
          f"fault {b['fault']}  ev_type {b['ev_type']}  bits_ok {b['bits_ok']}")

    play_bin = os.path.join(OUT, "play.bin")
    cap_bin = os.path.join(OUT, "cap.bin")
    openocd(f"dump_image {play_bin} {b['play_addr']:#x} {2*n}",
            f"dump_image {cap_bin} {b['cap_addr']:#x} {2*n}")
    play = np.fromfile(play_bin, dtype="<i2").astype(float)
    cap = np.fromfile(cap_bin, dtype="<i2").astype(float)
    if len(play) != n or len(cap) != n:
        print(f"short dump: play {len(play)} cap {len(cap)} of {n}")
        return 1
    # the firmware DC-removed cap before decoding; play is as generated,
    # so scale play the way the ISR did (3/4 into the DAC, unity back)
    play_dac = play * 0.75

    # --- delay: cross-correlate over the whole record ---
    x = play_dac - play_dac.mean()
    y = cap - cap.mean()
    if x.std() == 0:
        print("play buffer is silent -- nothing to fit")
        return 1
    corr = np.correlate(y, x, mode="full")
    lag = int(np.argmax(np.abs(corr))) - (len(x) - 1)
    # --- gain + dc by least squares on the aligned overlap ---
    if lag >= 0:
        xs, ys = x[: n - lag], y[lag:]
    else:
        xs, ys = x[-lag:], y[: n + lag]
    A = np.vstack([xs, np.ones_like(xs)]).T
    (g, dc), *_ = np.linalg.lstsq(A, ys, rcond=None)
    resid = ys - (g * xs + dc)
    sig_p = np.mean((g * xs) ** 2)
    noise_p = np.mean(resid ** 2)
    snr_db = 10 * np.log10(sig_p / noise_p) if noise_p > 0 else float("inf")
    peak_corr = np.max(np.abs(corr)) / (np.linalg.norm(x) * np.linalg.norm(y) + 1e-12)

    print()
    print(f"play:  rms {play_dac.std():8.1f}  peak {np.abs(play_dac).max():6.0f}  (int16 units, at the DAC)")
    print(f"cap:   rms {cap.std():8.1f}  peak {np.abs(cap).max():6.0f}  raw mean {b['cap_mean']} "
          f"(ADC units; 32768 = mid-rail 1.65 V)")
    print(f"fit:   gain {g:7.4f} ({20*np.log10(abs(g)) if g else float('-inf'):+.2f} dB)   "
          f"delay {lag:+d} samples ({lag/12.0:+.1f} ms)   corr {peak_corr:.4f}")
    print(f"noise: residual rms {np.sqrt(noise_p):7.1f}  ->  loop SNR {snr_db:6.1f} dB")
    print()
    if peak_corr < 0.5:
        print("VERDICT: the ADC is NOT hearing the DAC (correlation {:.2f}). "
              "Is PA4 shorted to PA6?".format(peak_corr))
    else:
        print(f"VERDICT: path is live -- gain {20*np.log10(abs(g)):+.1f} dB, "
              f"SNR {snr_db:.0f} dB. "
              + ("Decoder agreed: frame decoded, payload bit-exact."
                 if b["bits_ok"] else
                 f"Decoder did NOT (ev_type {b['ev_type']})."))
    np.save(os.path.join(OUT, "play.npy"), play_dac)
    np.save(os.path.join(OUT, "cap.npy"), cap)
    print(f"buffers saved in {OUT}/ (play.npy, cap.npy)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
