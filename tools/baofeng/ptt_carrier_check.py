#!/usr/bin/env python3
"""Key the Baofeng through a board's PB14 (PTT driver) over JTAG and check
that the HackRF hears the carrier.

    Baofeng (479.975 MHz NFM) -> 60 dB pad -> HackRF One (RX only)
    board PB14 -> PTT driver -> Baofeng PTT

The HackRF records in 10 ms blocks while ONE OpenOCD session drives PB14:
output-low (2 s), high (KEY_S), low again. Every step is time-stamped, so
the carrier can be attributed to the pin state rather than assumed: with
an active-high driver it appears only in the KEY window. The firmware keeps
running -- the pin is written over the AHB-AP, nothing is halted or
flashed -- and PB14 is left as a push-pull output LOW afterwards.

Levels: 5 W = +37 dBm, -60 dB = -23 dBm at the HackRF (limit -5 dBm), so
RX runs at minimum gain with the 14 dB amp OFF.

Needs `make -C cport bridge` up and no other OpenOCD session.

    ./venv/bin/python tools/baofeng/ptt_carrier_check.py --chip stmB
"""
import argparse
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
CFG = REPO / "tools/esp32-probe/stm32h7-rbb-dual.cfg"

RCC_AHB4ENR = 0x580244E0
GPIOB = 0x58020400
MODER, IDR, ODR, BSRR = GPIOB + 0x00, GPIOB + 0x10, GPIOB + 0x14, GPIOB + 0x18
PIN = 14
SET, RESET = 1 << PIN, 1 << (PIN + 16)


def openocd_cmds(chip, low_s, key_s, tail_s):
    t = f"targets {chip}.cpu0"
    rd = (f'echo [format "MARK state ODR14=%d IDR14=%d MODER14=%d" '
          f'[expr {{([lindex [read_memory {ODR:#x} 32 1] 0]>>{PIN})&1}}] '
          f'[expr {{([lindex [read_memory {IDR:#x} 32 1] 0]>>{PIN})&1}}] '
          f'[expr {{([lindex [read_memory {MODER:#x} 32 1] 0]>>{2*PIN})&3}}]]')
    return [
        "init", t,
        f"mmw {RCC_AHB4ENR:#x} 0x2 0",                 # GPIOB clock
        f"mww {BSRR:#x} {RESET:#x}",                     # ODR14 = 0 first
        f"mmw {MODER:#x} {1 << 2*PIN:#x} {2 << 2*PIN:#x}",  # mode 01 = output
        "echo {MARK low}", rd, f"sleep {int(low_s*1000)}",
        f"mww {BSRR:#x} {SET:#x}", "echo {MARK key}", rd,
        f"sleep {int(key_s*1000)}",
        f"mww {BSRR:#x} {RESET:#x}", "echo {MARK unkey}", rd,
        f"sleep {int(tail_s*1000)}", "echo {MARK done}", "exit",
    ]


def emergency_unkey(chip):
    cmds = ["init", f"targets {chip}.cpu0", f"mww {BSRR:#x} {RESET:#x}", "exit"]
    args = ["openocd", "-f", str(CFG)] + sum((["-c", c] for c in cmds), [])
    subprocess.run(args, cwd=REPO / "cport", timeout=60,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--chip", default="stmB", choices=["stmA", "stmB"])
    ap.add_argument("--freq", type=float, default=479.975e6)
    ap.add_argument("--offset", type=float, default=200e3,
                    help="LO this far below the channel (keeps DC off it)")
    ap.add_argument("--rate", type=float, default=2e6)
    ap.add_argument("--lna", type=float, default=0.0)
    ap.add_argument("--vga", type=float, default=8.0)
    ap.add_argument("--key-s", type=float, default=3.0)
    ap.add_argument("--save", default=None, metavar="NPZ",
                    help="save per-block time/power arrays for offline analysis")
    ap.add_argument("--listen", type=float, default=0.0, metavar="S",
                    help="listen only for S seconds (no JTAG, nothing keyed): "
                         "press the radio's PTT by hand meanwhile")
    ap.add_argument("--iq", default=None, metavar="NPY",
                    help="save the channel's IQ (mixed to baseband, boxcar-"
                         "decimated to 50 kS/s) for offline FM demodulation: "
                         "sub-tone, deviation, exact carrier frequency")
    a = ap.parse_args()

    import SoapySDR
    from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CF32

    sdr = SoapySDR.Device(dict(driver="hackrf"))
    sdr.setSampleRate(SOAPY_SDR_RX, 0, a.rate)
    sdr.setFrequency(SOAPY_SDR_RX, 0, a.freq - a.offset)
    sdr.setGain(SOAPY_SDR_RX, 0, "AMP", 0.0)
    sdr.setGain(SOAPY_SDR_RX, 0, "LNA", a.lna)
    sdr.setGain(SOAPY_SDR_RX, 0, "VGA", a.vga)
    print(f"HackRF RX: LO {sdr.getFrequency(SOAPY_SDR_RX, 0)/1e6:.6f} MHz, "
          f"channel {a.freq/1e6:.6f} MHz at +{a.offset/1e3:.0f} kHz, "
          f"AMP {sdr.getGain(SOAPY_SDR_RX, 0, 'AMP'):.0f} "
          f"LNA {sdr.getGain(SOAPY_SDR_RX, 0, 'LNA'):.0f} "
          f"VGA {sdr.getGain(SOAPY_SDR_RX, 0, 'VGA'):.0f} dB", flush=True)

    n = int(a.rate / 100)                          # 10 ms blocks
    f = np.fft.fftfreq(n, 1 / a.rate)
    win = np.hanning(n).astype(np.float32)
    chan = np.abs(f - a.offset) <= 15e3
    ref = (f > a.offset + 100e3) & (f < a.offset + 400e3)
    # anywhere in the capture (minus DC and the anti-alias skirts): finds a
    # carrier that is NOT on the expected channel, e.g. a repeater offset
    wb = (np.abs(f) > 50e3) & (np.abs(f) < 0.45 * a.rate)
    rows = []          # t, P_chan, P_ref/bin, f_peak, max|x|, wide peak/median, f_wide
    marks = []
    st = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
    sdr.activateStream(st)
    t0 = time.monotonic()
    proc = None

    def reader(p):
        for line in p.stdout:
            if "MARK" in line or "Error" in line:
                marks.append((time.monotonic() - t0, line.strip()))

    buf = np.empty(n, np.complex64)
    dec = int(round(a.rate / 50e3))
    assert n % dec == 0
    iq_blocks, ph = [], 0.0
    mix_k = np.arange(n)
    try:
        while True:
            now = time.monotonic() - t0
            if a.listen and now > a.listen:
                break
            if proc is None and now > 1.5 and not a.listen:
                cmds = openocd_cmds(a.chip, 2.0, a.key_s, 2.0)
                proc = subprocess.Popen(
                    ["openocd", "-f", str(CFG)] + sum((["-c", c] for c in cmds), []),
                    cwd=REPO / "cport", stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True, bufsize=1)
                threading.Thread(target=reader, args=(proc,), daemon=True).start()
            if proc is not None and proc.poll() is not None:
                if now > (marks[-1][0] if marks else 0) + 1.0:
                    break
            if now > max(90.0, a.listen + 5.0):
                break
            got = 0
            while got < n:
                r = sdr.readStream(st, [buf[got:]], n - got, timeoutUs=1000000)
                if r.ret > 0:
                    got += r.ret
            x = buf
            if a.iq:
                # continuous-phase mix of the channel to 0 Hz, then a boxcar
                # over `dec` samples (first null at 50 kHz, fine for NFM)
                m = np.exp(-1j * (ph + 2 * np.pi * a.offset / a.rate * mix_k))
                ph = float((ph + 2 * np.pi * a.offset / a.rate * n) % (2 * np.pi))
                iq_blocks.append((x * m).reshape(-1, dec).mean(axis=1)
                                 .astype(np.complex64))
            s = np.abs(np.fft.fft(x * win)) ** 2
            k = np.argmax(np.where(chan, s, 0))
            kw = np.argmax(np.where(wb, s, 0))
            rows.append((time.monotonic() - t0, s[chan].sum(), s[ref].mean(),
                         f[k], np.max(np.abs(x)),
                         s[kw] / np.median(s[wb]), f[kw]))
    finally:
        sdr.deactivateStream(st)
        sdr.closeStream(st)
        if proc is not None:
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                proc.kill()
            if proc.returncode != 0 or not any("MARK unkey" in m for _, m in marks):
                print("!! OpenOCD sequence did not finish -- sending emergency "
                      "unkey", flush=True)
                emergency_unkey(a.chip)

    if a.iq:
        np.save(a.iq, np.concatenate(iq_blocks))
        print(f"IQ: {a.iq}, {len(iq_blocks) * n / dec / 50e3:.1f} s at 50 kS/s, "
              f"channel at 0 Hz", flush=True)
    r = np.array(rows)
    t, pc, pr, fpk, mx, wpk, fw = r.T
    if a.save:
        np.savez(a.save, t=t, pc=pc, pr=pr, fpk=fpk, mx=mx, nbin=chan.sum(),
                 blk=n / a.rate)
    nbin = chan.sum()
    snr = 10 * np.log10(pc / (pr * nbin))           # channel power / noise in the same width
    for tm, m in marks:
        print(f"  t={tm:6.2f} s  {m}")
    if a.listen:
        lo_mhz = (a.freq - a.offset) / 1e6
        snr_c = 10 * np.log10(pc / (pr * nbin))
        on = snr_c > 10
        print(f"\n  listened {t[-1]:.1f} s; blocks with >10 dB in the channel: "
              f"{on.sum()} of {len(t)}")
        if on.any():
            edges = np.flatnonzero(np.diff(np.r_[0, on.astype(int), 0]))
            blk = n / a.rate                      # one block of SAMPLE time
            for s0, s1 in zip(edges[::2], edges[1::2]):
                # duration in sample time, not wall clock: the stamps are
                # when Python got round to each block, which drifts with
                # its buffering. Edge blocks count by their power relative
                # to the segment's own level -- a block half-filled with
                # carrier carries half its power -- so the edges resolve
                # to ~1 ms rather than one 10 ms block.
                lvl = np.median(pc[s0:s1])
                lo, hi = max(0, s0 - 1), min(len(pc), s1 + 1)
                frac = np.clip((pc[lo:hi] - pr[lo:hi] * nbin) / lvl, 0, 1)
                print(f"    carrier at {t[s0]:6.2f} s (wall)  duration "
                      f"{frac.sum() * blk:7.3f} s (sample time)  "
                      f"{snr_c[s0:s1].max():+.1f} dB, at "
                      f"{np.median(fpk[s0:s1]) - a.offset:+.0f} Hz, max|iq| {mx[s0:s1].max():.3f}")
        j = int(np.argmax(wpk))
        print(f"  strongest bin anywhere: {10*np.log10(wpk[j]):+.1f} dB at "
              f"{lo_mhz + fw[j]/1e6:.4f} MHz, t={t[j]:.2f} s (noise-only max is ~+14)")
        return 0 if on.any() else 2
    print("\n  100 ms averages: t[s]  chan/noise[dB]  peak offset[Hz]  max|iq|")
    for i in range(0, len(t), 10):
        sl = slice(i, i + 10)
        print(f"    {t[sl].mean():6.2f}  {10*np.log10(np.mean(pc[sl]/(pr[sl]*nbin))):+7.1f}"
              f"  {np.median(fpk[sl]) - a.offset:+9.0f}  {mx[sl].max():.3f}")

    mk = {m.split()[1]: tm for tm, m in marks if m.startswith("MARK") and len(m.split()) > 1}
    if not {"low", "key", "unkey"} <= mk.keys():
        print("\nsequence incomplete; marks:", marks)
        return 1
    # OpenOCD prints the mark right after the write lands; allow 0.15 s slop
    w = lambda lo, hi: (t > lo + 0.15) & (t < hi - 0.15)
    pre, key, post = w(mk["low"], mk["key"]), w(mk["key"], mk["unkey"]), w(mk["unkey"], t[-1])
    # MEDIAN of the per-block ratio: the SDR samples lag the wall clock by
    # its USB buffering, so a window edge can catch a few blocks of the
    # neighbouring state -- a linear mean let 2 carrier blocks lift the
    # "after" window to +15.7 dB and the verdict missed a clean +27.8 dB key
    db = lambda m: float(np.median(10 * np.log10(pc[m] / (pr[m] * nbin)))) if m.any() else float("nan")
    print(f"\n  PB14 low  (before): {db(pre):+6.1f} dB in the 30 kHz channel vs noise")
    print(f"  PB14 HIGH (keyed) : {db(key):+6.1f} dB, carrier at "
          f"{np.median(fpk[key]) - a.offset:+.0f} Hz from {a.freq/1e6:.6f} MHz "
          f"(HackRF's own clock error included), max|iq| {mx[key].max():.3f}")
    print(f"  PB14 low  (after) : {db(post):+6.1f} dB")
    lo_mhz = (a.freq - a.offset) / 1e6
    print(f"\n  strongest bin ANYWHERE in {lo_mhz - 0.45*a.rate/1e6:.3f}.."
          f"{lo_mhz + 0.45*a.rate/1e6:.3f} MHz (peak/median, DC excluded):")
    for name, m in (("low  (before)", pre), ("HIGH (keyed) ", key), ("low  (after) ", post)):
        if m.any():
            j = np.flatnonzero(m)[np.argmax(wpk[m])]
            print(f"    {name}: {10*np.log10(wpk[j]):+6.1f} dB at "
                  f"{lo_mhz + fw[j]/1e6:.4f} MHz")
    heard = db(key) - max(db(pre), db(post)) > 20
    print(f"\n  verdict: {'CARRIER HEARD while keyed' if heard else 'no carrier attributable to PB14'}"
          f" on {a.chip}")
    return 0 if heard else 2


if __name__ == "__main__":
    sys.exit(main())
