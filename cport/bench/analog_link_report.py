#!/usr/bin/env python3
"""Format the two beacons from a two-board analog-link run.

Reads the output of the `link-run` OpenOCD script on stdin, picks out
the two `mdw` dumps, and prints what actually happened -- including the
one number the single-board loopback could never produce: the sample
rate offset between the two boards, in ppm.

OpenOCD writes `mdw` output to stderr, so the caller must redirect 2>&1.
"""
import re
import sys

FIELDS = [
    "magic", "role", "stage", "tim_hz", "fs_mhz",
    "adc_ldo", "adc_cal", "adc_rdy", "adc_conv_ns",
    "n_play", "n_cap", "reps_done", "isr_count", "isr_max_cyc",
    "cap_min", "cap_max", "cap_mean", "buf_addr",
    "n_events", "n_decoded", "n_ok", "ev_start", "ev_cfo",
    "iser0", "iser1", "iser2", "iser3", "early_stray_irq", "early_stray_n",
    "fault", "icsr", "cfsr", "hfsr", "stray_irq", "stray_count",
] + [f"start{i}" for i in range(8)]
MAGIC = 0xA10C11A4
STAGES = {1: "entered", 2: "role set", 3: "frame built", 4: "peripherals",
          5: "timer", 6: "running", 7: "ran", 8: "decoded",
          0xE0: "NO ROLE SET", 0xEE: "generator refused"}


def parse(text):
    """{'TX': {...}, 'RX': {...}} from the marked mdw dumps."""
    out, cur, words = {}, None, []
    for line in text.splitlines():
        m = re.search(r"===\s*(TX|RX)\s+beacon\s*===", line)
        if m:
            if cur and words:
                out[cur] = words
            cur, words = m.group(1), []
            continue
        m = re.match(r"^0x[0-9a-f]{8}:\s+((?:[0-9a-f]{8}\s*)+)", line.strip())
        if m and cur:
            words += [int(w, 16) for w in m.group(1).split()]
    if cur and words:
        out[cur] = words
    return {k: dict(zip(FIELDS, v)) for k, v in out.items()}


def s32(v):
    return v - (1 << 32) if v >= (1 << 31) else v


def show(tag, b):
    print(f"--- {tag} (role {b['role']}) ---")
    if b["magic"] != MAGIC:
        print(f"  magic {b['magic']:#010x} -- want {MAGIC:#010x};"
              " image not running or not loaded")
        return False
    st = b["stage"]
    print(f"  stage        {st} ({STAGES.get(st, '?')})")
    print(f"  TIM6 clock   {b['tim_hz']} Hz"
          f"   ->  fs {b['fs_mhz'] / 1000.0:.3f} Hz")
    print(f"  ISR          {b['isr_count']} calls, worst {b['isr_max_cyc']} cyc"
          f" ({b['isr_max_cyc'] / 400.0:.1f} us of an 83.3 us period)")
    if b["role"] == 1:
        if b["stage"] >= 7:
            print(f"  played       {b['n_play']} samples x {b['reps_done']} reps")
        else:
            # reps_done is only written when the run loop ends, and the
            # transmitter is normally still going when we halt it
            part = b["isr_count"] / float(b["n_play"] or 1)
            print(f"  playing      {b['n_play']} samples,"
                  f" {part:.1f} reps done when halted (still transmitting)")
    else:
        print(f"  ADC          ldo {b['adc_ldo']} cal {b['adc_cal']}"
              f" rdy {b['adc_rdy']} (1 = ok, 2 = timed out),"
              f" conv {b['adc_conv_ns']} ns")
        print(f"  captured     {b['n_cap']} samples")
        lo, hi, mid = s32(b["cap_min"]), s32(b["cap_max"]), s32(b["cap_mean"])
        print(f"  ADC range    {lo}..{hi} of 0..65535, mean {mid}"
              f"  ({(hi - lo) * 3.3 / 65536.0:.3f} V pk-pk"
              f" about {mid * 3.3 / 65536.0:.3f} V)")
        print(f"  events       {s32(b['n_events'])} raw,"
              f" {s32(b['n_decoded'])} decoded,"
              f" {s32(b['n_ok'])} BIT-EXACT")
        if s32(b["n_ok"]):
            cfo_hz = s32(b["ev_cfo"]) * 12000.0 / 2.0 ** 32
            print(f"  first good   start {s32(b['ev_start'])},"
                  f" residual CFO {cfo_hz:+.2f} Hz")
    if b["early_stray_n"] or b["stray_count"]:
        print(f"  STRAY IRQ     {b['early_stray_irq']} during bring-up"
              f" (x{b['early_stray_n']}),"
              f" {b['stray_irq']} x{b['stray_count']} total"
              " -- a stray MASKS its source")
    inherited = [b[f"iser{i}"] for i in range(4)]
    if any(inherited):
        print("  inherited NVIC " + " ".join(f"{v:#010x}" for v in inherited)
              + "  (enables left by the displaced firmware)")
    if b["fault"]:
        print(f"  FAULT        {b['fault']:#x} icsr {b['icsr']:#x}"
              f" cfsr {b['cfsr']:#x} hfsr {b['hfsr']:#x}")
    if b["stray_count"]:
        print(f"  stray irq    {b['stray_irq']} x{b['stray_count']}")
    return True


def main():
    text = sys.stdin.read()
    got = parse(text)
    if not got:
        print("no beacons found in the OpenOCD output")
        print(text[-2000:])
        return 1
    ok = True
    for tag in ("TX", "RX"):
        if tag in got:
            ok &= show(tag, got[tag])
        else:
            print(f"--- {tag}: no beacon ---")
            ok = False
    rx = got.get("RX")
    if rx and s32(rx.get("n_ok", 0)) >= 2 and "TX" in got:
        st = [s32(rx[f"start{i}"]) for i in range(min(8, s32(rx["n_ok"])))]
        gaps = [b - a for a, b in zip(st, st[1:])]
        n_play = got["TX"]["n_play"]
        if gaps and n_play:
            span = st[-1] - st[0]                 # total, not per-gap
            expected = n_play * len(gaps)
            # the receiver reports a start to the nearest sample, so the
            # whole span is worth +-1 sample -- quote that, do not quote
            # a ppm figure finer than the measurement can support
            res = 1e6 / span
            ppm = (span - expected) / float(expected) * 1e6
            print("\n=== sample-rate offset, measured on the wire ===")
            print(f"  frame starts {st}, spacing {gaps}")
            print(f"  span {span} samples against {expected} transmitted"
                  f"  ->  {ppm:+.0f} ppm, resolution +-{res:.0f} ppm")
            if abs(ppm) <= res:
                print(f"  i.e. the two clocks agree to within {res:.0f} ppm --"
                      " a bound, not a measured offset.")
            print(f"  Only {len(gaps) + 1} frames fit the {rx['n_cap']}-sample"
                  " window, so the span is short; a tighter figure needs a\n"
                  "  longer capture, which needs streaming decode rather than"
                  " a buffer.")
            print("  Comparing the boards' TIM6 clocks cannot show this at"
                  " all: each board's\n  TIM6 and DWT come off the SAME PLL,"
                  " so that ratio is exact on both\n  boards however far"
                  " apart their crystals are -- it reads 0 ppm by"
                  " construction.")

    if rx and rx.get("magic") == MAGIC:
        n = s32(rx["n_ok"])
        print("\nRESULT: " + (f"{n} frame(s) crossed the wire bit-exact"
                              if n > 0 else
                              "no frame decoded -- see the ADC range above"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
