#!/usr/bin/env python3
"""ngspice simulation of the Baofeng speaker -> STM32H743 PA6 front end
(or PA0 with `config rx_input 1`; see R_SW_CASES).

    speaker+ -C1-R1-+-R3-+- PA6 (ADC1_INP3)      Vb = VDDA/2 (10k/10k, 10u)
                    R2   C2   BAT54S to VDDA/GND
                    Vb   GND

Measured input (2026-11-15): 6 V p-p at the Baofeng speaker for a 1 kHz
tone at full deviation (+-2.5 kHz), volume as set on the stand.

Runs, each a separate ngspice batch job:
  ac      speaker -> PA6 and VDDA -> PA6, 1 Hz..1 MHz; variant A (one RC
          pole, as proposed) and B (a second RC section, for a class-D
          amplifier's switching carrier)
  steady  6 V p-p, 1 kHz, with the ADC's sample-and-hold modelled: switch
          + sampling cap, closed for the sampling window every 83.33 us
          (12 kHz), the cap RESET to 0 V between samples (the worst case
          for kickback). A duplicate network with no ADC is the reference;
          the error is read at the END of each sampling window, after the
          bias has settled, and split into gain, offset, a fixed sampling
          delay (~R_ADC*C_ADC) and what is left.
  startup power-up from all-zero caps: how long until the bias settles
  overload larger inputs (14 and 20 V p-p): where the clamps hold PA6 and
          how much current they take

ADC sampling model: C_ADC from the datasheet (DS12110 Rev 7, 4 pF typ);
R_ADC is not in the datasheet, so the steady run brackets it (940 ohm,
ST's forum figure; 4 kohm, fitted to the datasheet's sampling-time table)
at both possible sampling windows -- see the constants below. 27 nF at the
pin dominates either way, which the steady run quantifies.

    ./venv/bin/python tools/baofeng/frontend_sim.py [--keep DIR]

~2 min: the steady run is four 330 ms transients at a 50 ns step (the
bias needs 300 ms to settle under the ADC's load; see T_SETTLE).
"""
import argparse
import os
import subprocess
import sys
import tempfile

import numpy as np

VDDA = 3.3
LSB = VDDA / 65536            # 16-bit
FS = 12000.0                  # the firmware's converter tick
# ADC sampling model, from DS12110 Rev 7 (STM32H742/743 datasheet). Our
# parts are rev V (DBGMCU_IDC REV_ID 0x2003, cport/FEASIBILITY.md).
#
# C_ADC "internal sample and hold capacitor": 4 pF typ -- Table 184 (rev V)
# and Table 87 (rev Y) agree. No min/max given.
C_SH = 4e-12
# R_ADC (sampling switch) is NOT in the datasheet. PA6 is a SLOW channel
# (Table 185 notes 4-5: fast = PC0..PC3, PA0, PA1 only). Two values bracket
# it:
#    940 ohm  ST community moderator's figure for slow channels
#   4000 ohm  the effective value that reproduces Table 185's slow-channel
#             minimum sampling times (47 pF pin cap + 4 pF, full-scale step
#             to 1/2 LSB): fits 3.6-4.0 kohm per resolution. 940 ohm
#             settles ~3.5x faster than the table allows, so 4 k is the
#             conservative one.
# The firmware can also sample PA0 (`config rx_input 1`, so the front end
# can sit beside the board-to-board wire on PA6). PA0 is a FAST direct
# channel (same moderator: ~450 ohm), below both cases, so the slow-channel
# results here bound it.
R_SW_CASES = (940, 4000)
# Sampling window: SMPR 64.5 cycles of f_ADC = per_ck (64 MHz HSI) / PRESC 8
# = 8 MHz -> 8.06 us. Rev V may divide the ADC kernel clock by a further 2
# (-> 16.1 us); not verified on our boards, so both are run. Shorter is the
# worst case for settling.
T_SAMPLE_CASES = (64.5 / 8e6, 64.5 / 4e6)
# The steady run starts from the DC operating point WITHOUT the ADC (the
# switches are open at t=0). The ADC then draws ~C_ADC*V*fs = 79 nA on
# average, which shifts the bias through VB's 5k*10u = 50 ms time
# constant: a 25 ms run caught that offset still drifting and the fits
# read the drift as residual. Settle 6 tau, then analyse 30 tone periods
# (an integer number, so the fit's DC and 1 kHz columns are orthogonal).
T_SETTLE = 0.30
T_ANALYSE = 0.030

# BAT54-like Schottky: Vf ~0.33 V @ 1 mA, ~0.41 V @ 10 mA (datasheet-typical)
BAT54 = ".model BAT54 D(IS=5e-9 N=1.05 RS=1.6 CJO=10p M=0.4 VJ=0.38 " \
        "BV=30 IBV=1e-5 TT=5n EG=0.69 XTI=2)"
# generic rail-to-rail op-amp (STM32H7 OPAMP class, ASSUMED figures):
# A0 100 dB, GBW 7 MHz (dominant pole 70 Hz), output limited to the rails,
# 50 ohm output resistance
OPA = """
.subckt OPA inp inm out vdd
Eg x 0 inp inm 1e5
Rp x y 1k
Cp y 0 2.274u
Bo w 0 V = max(0.01, min(v(vdd) - 0.01, v(y)))
Ro w out 50
.ends
"""


def network(sfx, src, variant="A"):
    """the front end, nodes suffixed so several copies can share a deck"""
    s = sfx
    lines = [
        f"C1{s} {src} n1{s} 2.2u",
        f"R1{s} n1{s} a{s} 3.9k",
        f"R2{s} a{s} vb{s} 2.0k",
        f"RB1{s} vdda vb{s} 10k",
        f"RB2{s} vb{s} 0 10k",
        f"CB{s} vb{s} 0 10u",
    ]
    if variant == "B":
        # unity-gain Sallen-Key, Butterworth-ish (f0 5.1 kHz, Q 0.73),
        # driven from node a (Thevenin 1.32k of R1||R2 adds to R5).
        # A passive second RC section CANNOT do this job: with 1.32k of
        # source resistance, any section that keeps the corner >= 4.2 kHz
        # only reaches ~-40 dB at 300 kHz (E12 search, 2026-11-15).
        lines += [f"R5{s} a{s} m{s} 8.2k",
                  f"R6{s} m{s} p{s} 10k",
                  f"CF{s} m{s} o{s} 4.7n",
                  f"CG{s} p{s} 0 2.2n",
                  f"XU{s} p{s} o{s} o{s} vdda OPA",   # follower
                  f"R3{s} o{s} pa6{s} 100"]
    else:
        lines += [f"R3{s} a{s} pa6{s} 100"]
    lines += [
        f"C2{s} pa6{s} 0 27n",
        f"D1{s} pa6{s} vdda BAT54",         # BAT54S upper diode
        f"D2{s} 0 pa6{s} BAT54",            # BAT54S lower diode
    ]
    return lines


def run(deck, outdir, name):
    path = os.path.join(outdir, name + ".cir")
    with open(path, "w") as f:
        f.write(deck)
    r = subprocess.run(["ngspice", "-b", path], cwd=outdir,
                       capture_output=True, text=True)
    log = r.stdout + r.stderr
    # "aborted" too: a stalled transient exits 0, writes a truncated data
    # file and says only "Timestep too small ... aborted"
    if r.returncode != 0 or any(k in log for k in ("rror", "aborted")):
        bad = [l for l in log.splitlines()
               if any(k in l for k in ("rror", "aborted", "too small"))]
        if r.returncode != 0 or bad:
            sys.exit(f"ngspice failed on {name}:\n" + "\n".join(bad[:10]))
    return path


def load(path):
    return np.loadtxt(path)


def ac_runs(outdir):
    res = {}
    for variant in ("A", "B"):
        for inp in ("spk", "vdda"):
            src_ac = "AC 1" if inp == "spk" else "AC 0"
            vdd_ac = "AC 1" if inp == "vdda" else "AC 0"
            deck = "\n".join([
                f"* AC {variant} {inp}",
                f"VDDA vdda 0 DC {VDDA} {vdd_ac}",
                f"VSPK spk 0 DC 0 {src_ac}",
                *network("", "spk", variant),
                BAT54, OPA,
                ".control",
                "ac dec 100 1 3meg",
                f"wrdata ac_{variant}_{inp}.dat vdb(pa6)",
                "quit",
                ".endc", ".end", ""])
            run(deck, outdir, f"ac_{variant}_{inp}")
            d = load(os.path.join(outdir, f"ac_{variant}_{inp}.dat"))
            res[(variant, inp)] = (d[:, 0], d[:, 1])
    return res


def at(f, y, f0):
    return float(np.interp(np.log10(f0), np.log10(f), y))


def corner(f, y, ref_db, lo):
    """-3 dB frequency below (lo=True) or above the passband"""
    k = np.argmax(y)
    if lo:
        seg_f, seg_y = f[:k + 1], y[:k + 1]
    else:
        seg_f, seg_y = f[k:], y[k:]
    t = ref_db - 3.0
    i = np.flatnonzero(np.diff(np.sign(seg_y - t)))[0]
    x0, x1 = np.log10(seg_f[i]), np.log10(seg_f[i + 1])
    y0, y1 = seg_y[i], seg_y[i + 1]
    return 10 ** (x0 + (t - y0) * (x1 - x0) / (y1 - y0))


def steady_run(outdir, r_sw, t_sample, vpp=6.0):
    per = 1.0 / FS
    tag = f"steady_{r_sw}_{t_sample*1e6:.0f}us"
    deck = "\n".join([
        "* steady state with the ADC sample-and-hold, plus an unloaded twin",
        f"VDDA vdda 0 DC {VDDA}",
        f"VSPK spk 0 DC 0 SIN(0 {vpp/2} 1k)",
        *network("", "spk"),
        *network("i", "spk"),                # reference: no ADC attached
        f"S1 pa6 csh samp 0 SWADC",
        f"CSH csh 0 {C_SH:.4e}",
        "S2 csh 0 rst 0 SWRST",
        f"VSAMP samp 0 PULSE(0 1 0 20n 20n {t_sample:.4e} {per:.6e})",
        f"VRST rst 0 PULSE(0 1 {t_sample + 1e-6:.4e} 20n 20n "
        f"{per - t_sample - 3e-6:.4e} {per:.6e})",
        f".model SWADC SW(VT=0.5 VH=0.05 RON={r_sw} ROFF=1e9)",
        ".model SWRST SW(VT=0.5 VH=0.05 RON=10 ROFF=1e9)",
        BAT54,
        # tight enough for sub-LSB (50 uV) differences; 1e-15/1e-9 with
        # 5 ns edges into 1e12 ohm stalled at "timestep too small"
        # Gear integration: trapezoidal stalled at "timestep too small"
        # on the sampling-switch edges (tested: series R on VDDA alone
        # did not help, gear alone did)
        ".options method=gear reltol=1e-6 abstol=1e-12 vntol=1e-8",
        ".control",
        # only the analysis window is stored (tstart); the max step stays
        # 50 ns throughout
        f"tran 0.05u {T_SETTLE + T_ANALYSE + 1e-4:.4f} {T_SETTLE:.4f} 0.05u",
        f"wrdata {tag}.dat v(pa6) v(csh) v(pa6i) v(samp)",
        "quit",
        ".endc", ".end", ""])
    run(deck, outdir, tag)
    d = load(os.path.join(outdir, f"{tag}.dat"))
    t, pa6, csh, pa6i = d[:, 0], d[:, 1], d[:, 3], d[:, 5]
    # value held at the END of each sampling window, over a whole number
    # of tone periods once the bias has settled
    k = np.arange(int(round(T_SETTLE * FS)),
                  int(round((T_SETTLE + T_ANALYSE) * FS)))
    te = k / FS + t_sample - 2e-9
    held = np.interp(te, t, csh)
    ideal = np.interp(te, t, pa6i)
    err = held - ideal
    return dict(t=t, pa6=pa6, pa6i=pa6i, held=held, ideal=ideal, err=err)


def startup_run(outdir):
    deck = "\n".join([
        "* power-up from all-zero capacitors",
        f"VDDA vdda 0 PWL(0 0 10u {VDDA})",
        "VSPK spk 0 DC 0 SIN(0 3 1k)",
        *network("", "spk"),
        BAT54,
        ".control",
        "tran 20u 1.0 0 20u uic",
        "wrdata startup.dat v(pa6) v(vb)",
        "quit",
        ".endc", ".end", ""])
    run(deck, outdir, "startup")
    d = load(os.path.join(outdir, "startup.dat"))
    return d[:, 0], d[:, 1], d[:, 3]


def overload_run(outdir, vpp):
    deck = "\n".join([
        f"* overload {vpp} V p-p",
        f"VDDA vdda 0 DC {VDDA}",
        f"VSPK spk 0 DC 0 SIN(0 {vpp/2} 1k)",
        *network("", "spk"),
        BAT54,
        ".control",
        # device-internal currents are only kept if saved BEFORE the
        # analysis; without this they read 0 while PA6 sits clamped
        "save v(pa6) i(VDDA) @d1[id] @d2[id]",
        "tran 1u 10m 0 1u",
        f"wrdata ovl_{int(vpp)}.dat v(pa6) i(VDDA) @d1[id] @d2[id]",
        "quit",
        ".endc", ".end", ""])
    run(deck, outdir, f"ovl_{int(vpp)}")
    d = load(os.path.join(outdir, f"ovl_{int(vpp)}.dat"))
    t = d[:, 0]
    m = t > 5e-3
    return d[m, 1], d[m, 5], d[m, 7]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--keep", default=None,
                    help="directory for decks and data (default: a temp dir)")
    a = ap.parse_args()
    outdir = a.keep or tempfile.mkdtemp(prefix="frontend_sim_")
    os.makedirs(outdir, exist_ok=True)
    print(f"decks and data in {outdir}\n")

    # ---- AC ----
    ac = ac_runs(outdir)
    print("AC response, speaker -> PA6")
    print("  variant  passband   -3 dB lo  -3 dB hi   @6 kHz  @12 kHz  @300 kHz")
    for v in ("A", "B"):
        f, y = ac[(v, "spk")]
        ref = at(f, y, 1000)
        print(f"  {v}       {ref:+6.2f} dB  {corner(f, y, ref, True):6.1f} Hz "
              f"{corner(f, y, ref, False)/1e3:6.2f} kHz "
              f"{at(f, y, 6e3) - ref:+6.1f}  {at(f, y, 12e3) - ref:+6.1f}  "
              f"{at(f, y, 300e3) - ref:+7.1f}   (dB re passband)")
    print("  B = A with an active 2nd-order Sallen-Key (op-amp) instead of the RC")
    for v in ("A", "B"):
        f, y = ac[(v, "spk")]
        res = 7.0 * 10 ** (at(f, y, 300e3) / 20)       # 7 V p-p ASSUMED
        print(f"  {v}: a 7 V p-p class-D carrier at 300 kHz (assumed) leaves "
              f"{res*1e3:.2f} mV p-p at PA6 = {20*np.log10(res/VDDA):+.0f} dBFS")
    print("  supply rejection, VDDA -> PA6 (bias divider + 10 uF):")
    for v in ("A",):
        f, y = ac[(v, "vdda")]
        print("   " + "  ".join(f"{fx:>6} {at(f, y, fx):+6.1f} dB"
                                for fx in (50, 300, 1000, 2700)))

    # ---- steady state with the ADC ----
    first = True
    for r_sw in R_SW_CASES:
        for t_sample in T_SAMPLE_CASES:
            s = steady_run(outdir, r_sw, t_sample)
            if first:
                pk = s["pa6i"]
                vpp_pin = pk.max() - pk.min()
                dc = 0.5 * (pk.max() + pk.min())
                dbfs = 20 * np.log10(vpp_pin / VDDA)
                print(f"\n6 V p-p, 1 kHz at the speaker (full deviation):")
                print(f"  PA6: {vpp_pin:.3f} V p-p around {dc:.3f} V  "
                      f"({pk.min():.3f}..{pk.max():.3f} V) = {dbfs:+.1f} dBFS "
                      f"of the {VDDA} V span")
                print(f"  ADC sampling: C_ADC {C_SH*1e12:.0f} pF (DS12110 "
                      f"typ), cap reset to 0 V between samples (worst case);"
                      f"\n  error = held value - unloaded twin, at the end "
                      f"of each window")
                print("    R_ADC  window    mean   max|err|  gain error  "
                      "offset  | resid: after gain+off   delay  after delay")
                first = False
            e = s["err"]
            held, ideal = s["held"], s["ideal"]
            # split the sampling error into a gain part and an offset
            # (linear, harmless), then a 1 kHz QUADRATURE part -- the cap
            # trails a moving input through the switch by ~R_ADC*C_ADC, a
            # fixed sampling delay, also harmless -- and what is left
            tt = np.arange(len(ideal)) / FS
            A = np.vstack([ideal - ideal.mean(), np.ones_like(ideal)]).T
            coef, *_ = np.linalg.lstsq(A, held, rcond=None)
            resid = held - A @ coef
            Q = np.vstack([np.cos(2e3 * np.pi * tt), np.sin(2e3 * np.pi * tt)]).T
            q, *_ = np.linalg.lstsq(Q, resid, rcond=None)
            resid2 = resid - Q @ q
            # quadrature amplitude -> the time shift that produces it
            delay = np.hypot(*q) / (2e3 * np.pi * vpp_pin / 2)
            ref = vpp_pin / 2 / np.sqrt(2)
            rr, rr2 = (np.sqrt(np.mean(r ** 2)) for r in (resid, resid2))
            # settling check: the mean error must not move across the
            # window (it did by ~LSBs in a 25 ms run)
            h = len(e) // 2
            drift = e[h:].mean() - e[:h].mean()
            if abs(drift) > 0.1 * LSB:
                print(f"    WARNING: mean error still moving "
                      f"{drift/LSB:+.2f} LSB across the window -- "
                      f"raise T_SETTLE")
            print(f"    {r_sw:5d} {t_sample*1e6:5.2f} us "
                  f"{e.mean()/LSB:+5.1f} LSB {np.abs(e).max()/LSB:5.1f} LSB "
                  f"{100*(coef[0]-1):+.4f} % "
                  f"{(coef[1]-ideal.mean())/LSB:+5.1f} LSB | "
                  f"{rr/LSB:4.2f} LSB rms ({20*np.log10(rr/ref):.0f} dB) "
                  f"{delay*1e9:5.1f} ns  {rr2/LSB:4.2f} LSB rms "
                  f"({20*np.log10(rr2/ref):.0f} dB)")

    # ---- start-up ----
    t, v, vb = startup_run(outdir)
    # ngspice's steps are not uniform (tiny ones right after uic): put the
    # trace on a 20 us grid before taking 1 ms (= one tone period) means
    tg = np.arange(0.0, t[-1], 20e-6)
    vg = np.interp(tg, t, v)
    blk = 50
    means = np.convolve(vg, np.ones(blk) / blk, mode="valid")
    tm = tg[:len(means)]
    ok = np.abs(means - VDDA / 2) < 0.01 * VDDA / 2
    settle = tm[np.flatnonzero(~ok)[-1] + 1] if (~ok).any() else 0.0
    print(f"\npower-up: PA6's mean settles within 1 % of {VDDA/2:.2f} V after "
          f"{settle*1e3:.0f} ms (the firmware ignores the ADC for its first "
          f"500 ms)")

    # ---- overload ----
    print("\noverload (the clamps):")
    for vpp in (14.0, 20.0):
        v, i1, i2 = overload_run(outdir, vpp)
        print(f"  {vpp:4.0f} V p-p: PA6 {v.min():+.3f} .. {v.max():+.3f} V, "
              f"clamp current up to {1e3*max(i1.max(), i2.max()):.2f} mA "
              f"(to VDDA {1e3*i1.max():.2f}, to GND {1e3*i2.max():.2f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
