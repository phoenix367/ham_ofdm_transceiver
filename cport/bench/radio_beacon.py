#!/usr/bin/env python3
"""Dump and interpret the radio firmware's beacon from one or both boards.

    ./bench/radio_beacon.py            # both boards, interpreted
    ./bench/radio_beacon.py --board b  # just stmB
    ./bench/radio_beacon.py --raw      # named fields, no verdicts

Needs the JTAG bridge up (`make bridge`) and both boards wired as the
daisy-chain (stm32h7-rbb-dual.cfg). The beacon lives at 0x24000000 in
usb/usb_radio_main.c; keep the FIELDS list below in step with its
beacon_t -- the struct is append-only for exactly this reason.

This parser existed only as inline shell python while the streamed-burst
failure was being chased; every retyping risked a field-order slip, and
one of them (reading tx_short from the wrong offset) cost a round of
confusion. Checked in so the layout lives in one place.
"""
import argparse
import re
import subprocess
import sys

CFG = "../tools/esp32-probe/stm32h7-rbb-dual.cfg"
MAGIC = 0x0AD10BEE
N_WORDS = 87

FIELDS = [
    "magic", "stage", "mounted", "ms", "rx_bytes", "tx_bytes", "isr_count",
    "tx_frames", "rx_decodes", "cap_overruns", "tx_underruns", "tx_faults",
    "adc_ready", "rxs_ready", "last_rung", "last_snr_q8",
    "loops", "rx_samples", "push_ms_max",
    "rx_active_mask", "my_req", "follow_changes",
    "burst_starts", "burst_blocks", "burst_misses", "burst_refused",
    "ring_miss",
    "tx_short", "tx_last_pulled", "tx_last_total",
    "last_miss_type", "last_miss_bits",
    "miss_lead_hi", "miss_lead_lo", "miss_snr_q8", "miss_start",
    "walk_base_lo", "walk_step", "miss_cs", "keyups",
    "keyup_ms0", "keyup_ms1", "keyup_ms2", "keyup_ms3",
    "keyup_cs0", "keyup_cs1", "keyup_cs2", "keyup_cs3",
    "keyup_floor0", "keyup_floor1", "keyup_floor2", "keyup_floor3",
    "bc_tx_groups", "bc_tx_ms", "bc_rx_frames", "bc_rx_lost",
    "ev_n", "ev_neg", "ev_last", "ev_last_ms", "ev_cap_ovr",
    "cs_peak", "cs_peak_ms",
] + ["ev%d_%s" % (i, k) for i in range(8) for k in ("ms", "what", "start")]
STAGES = {1: "entered", 2: "supply", 3: "analog", 4: "receivers",
          5: "tusb", 6: "loop (not mounted)", 7: "MOUNTED"}
MODES = ["NORMAL", "ROBUST", "EXTREME"]


def s32(v):
    return v - (1 << 32) if v >= (1 << 31) else v


def read_board(label):
    """label 'a'/'b' -> dict of fields, or None."""
    tgt = "stm%s.cpu0" % label.upper()
    cmd = ["openocd", "-f", CFG, "-c", "init", "-c", "targets " + tgt,
           "-c", "mdw 0x24000000 %d" % N_WORDS, "-c", "exit"]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True,
                             timeout=240)
    except (OSError, subprocess.TimeoutExpired) as e:
        print("openocd failed for %s: %s" % (tgt, e), file=sys.stderr)
        return None
    words = []
    for line in (out.stdout + out.stderr).splitlines():
        m = re.match(r"^0x24000[0-9a-f]{3}:\s+((?:[0-9a-f]{8}\s*)+)", line)
        if m:
            words += [int(w, 16) for w in m.group(1).split()]
    if len(words) < N_WORDS:
        print("no beacon from %s -- bridge up? boards wired?" % tgt,
              file=sys.stderr)
        return None
    return dict(zip(FIELDS, words[:N_WORDS]))


def show(label, d, raw):
    print("=== stm%s ===" % label.upper())
    if d["magic"] != MAGIC:
        print("  magic %#010x, want %#010x -- not the radio firmware, or "
              "not booted (make flash-radio-%s, then make reset)"
              % (d["magic"], MAGIC, label))
        return
    if raw:
        for k in FIELDS:
            print("  %-14s %d" % (k, d[k]))
        return

    st = d["stage"]
    print("  up %.0f s   stage %s   ISR %d" %
          (d["ms"] / 1000.0, STAGES.get(st, st), d["isr_count"]))
    act = "+".join(MODES[i] for i in range(3)
                   if (d["rx_active_mask"] >> i) & 1) or "none"
    print("  station: tx_frames %d  rx_decodes %d  my_req %d  "
          "listening %s" % (d["tx_frames"], d["rx_decodes"],
                            d["my_req"], act))
    print("  bursts: starts %d  blocks %d  misses %d  refused %d" %
          (d["burst_starts"], d["burst_blocks"], d["burst_misses"],
           d["burst_refused"]))
    ev = d["ev_last"]
    print("  events: %d (%d failed)  last: mode %s type %d typ %d at %.0f s"
          % (d["ev_n"], d["ev_neg"], MODES[(ev >> 24) & 3],
             s32(((ev >> 16) & 0xFF) << 24) >> 24, ev & 0xFF,
             d["ev_last_ms"] / 1000.0))
    print("  broadcast: %d group(s) keyed (last %.0f s)  %d frame(s) heard, "
          "%d lost" % (d["bc_tx_groups"], d["bc_tx_ms"] / 1000.0,
                       d["bc_rx_frames"], d["bc_rx_lost"]))
    print("  loudest thing heard: cs %d at %.0f s (quiet ~2e4, carrier ~2e8)"
          % (d["cs_peak"], d["cs_peak_ms"] / 1000.0))
    if d["ev_n"]:
        print("  last events (newest last):")
        order = sorted(range(8), key=lambda i: d["ev%d_ms" % i])
        for i in order:
            w = d["ev%d_what" % i]
            if not d["ev%d_ms" % i]:
                continue
            ty = (w >> 24) & 0xF
            print("    %7.1f s  %-7s type %+d typ %2d  start %d  drops %d"
                  % (d["ev%d_ms" % i] / 1000.0, MODES[(w >> 28) & 3],
                     ty - 16 if ty > 7 else ty, (w >> 16) & 0xFF,
                     d["ev%d_start" % i], w & 0xFFFF))

    # ---- verdicts, each a measured failure signature -----------------
    bad = 0
    if not d["adc_ready"] or not d["rxs_ready"]:
        bad += 1
        print("  !! adc_ready %d rxs_ready %d -- bring-up failed" %
              (d["adc_ready"], d["rxs_ready"]))
    if d["cap_overruns"]:
        bad += 1
        print("  !! cap_overruns %d -- capture FIFO smaller than the "
              "decoder's worst blocking burst (see push_ms_max %d ms; "
              "the commit stall was measured at 2283 ms)" %
              (d["cap_overruns"], d["push_ms_max"]))
    if d["tx_underruns"]:
        bad += 1
        print("  !! tx_underruns %d -- DAC starved mid-frame (pre-fill "
              "regression? each one puts a mid-rail sample on air)" %
              d["tx_underruns"])
    if d["tx_short"]:
        bad += 1
        print("  !! tx_short %d, last %d/%d -- the generator rendered "
              "fewer samples than build() promised the station: the "
              "peer heard a TRUNCATED frame" %
              (d["tx_short"], d["tx_last_pulled"], d["tx_last_total"]))
    if d["tx_faults"]:
        bad += 1
        print("  !! tx_faults %d -- a receive phase took the arena "
              "mid-transmission (half-duplex violation)" % d["tx_faults"])
    if d["ring_miss"]:
        bad += 1
        print("  !! ring_miss %d -- decodes abandoned because the raw "
              "ring had been overwritten" % d["ring_miss"])
    if d["keyups"] and all(d["keyup_cs%d" % i] == 0
                           for i in range(min(4, d["keyups"]))):
        bad += 1
        print("  !! every recorded key-up saw cs=0 -- CARRIER SENSE IS "
              "DEAD (a quiet wire reads ~2e4: the peer's DAC holds "
              "mid-rail). This exact signature was the uncalled "
              "note_busy_isr regression.")
    if d["burst_misses"]:
        hi, lo = d["miss_lead_hi"], d["miss_lead_lo"]
        b44 = (hi << 12) | (lo & 0xFFF)
        sub = [(b44 >> 16) & 0xFF, (b44 >> 8) & 0xFF, b44 & 0xFF]
        snr = s32(d["miss_snr_q8"]) / 256.0
        sane = (sub[0] & 0x80) and 0 < (sub[0] & 0x7F) < 32 \
            and 0 < sub[2] <= 253
        print("  first walk miss: snr %+.1f dB  cs %d  lead sub %s" %
              (snr, d["miss_cs"], " ".join("%#04x" % x for x in sub)))
        if snr < -10 and not sane:
            print("     -> garbage bits at noise SNR: the SIGNAL WAS "
                  "GONE mid-stream. Either this board keyed over it "
                  "(check keyup times vs the stream) or the peer's "
                  "carrier stopped (check ITS tx_short/underruns).")
        elif sane:
            print("     -> subheader decodes sanely: aligned but noisy "
                  "-- a real channel problem, not a firmware one.")
    if not bad and not d["burst_misses"]:
        print("  clean.")
    print("  key-ups %d, last (ms cs floor): %s" %
          (d["keyups"],
           "  ".join("%d %d %d" % (d["keyup_ms%d" % i],
                                   d["keyup_cs%d" % i],
                                   d["keyup_floor%d" % i])
                     for i in range(min(4, d["keyups"])))))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--board", choices=["a", "b", "both"], default="both")
    ap.add_argument("--raw", action="store_true",
                    help="named fields only, no interpretation")
    args = ap.parse_args()
    boards = ["a", "b"] if args.board == "both" else [args.board]
    rc = 0
    for b in boards:
        d = read_board(b)
        if d is None:
            rc = 1
            continue
        show(b, d, args.raw)
    return rc


if __name__ == "__main__":
    sys.exit(main())
