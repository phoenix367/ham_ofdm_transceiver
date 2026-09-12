---
name: read-beacons
description: Read and interpret the two boards' firmware beacons over JTAG - counters, events, key-ups, verdicts - and, when a verdict is not enough, dump a board's raw capture ring and replay it through the host receiver to attribute a failure to a code change or to the wire. Use after any board run, when a console shows a timeout or a miss, or when a number on the boards must be checked against the host.
---

# Reading the board beacons

The radio firmware (`cport/usb/usb_radio_main.c`) keeps an append-only
`beacon_t` at 0x24000000 with every counter the boards cannot print:
frames, decodes, capture overruns, DAC underruns, arena faults, burst
walks, the last miss, the last four key-ups with carrier sense, and a
ring of the last receive events. `cport/bench/radio_beacon.py` reads it
over the JTAG bridge without halting anything and prints one verdict
per measured failure signature.

```bash
cd cport && make bridge                      # no-op if already listening on :3335
../venv/bin/python bench/radio_beacon.py     # both boards, interpreted
../venv/bin/python bench/radio_beacon.py --board b --raw   # one board, every field
```

## Rules, each one earned

- **One OpenOCD session at a time.** The beacon read, a capture-ring
  dump and `make flash-radio-*` all go through the same bridge; a
  second session fails with "Error on socket ... Bad file descriptor"
  and reads like a dead probe. The consoles do NOT conflict -- they
  are USB -- so beacons can be read during a transfer.
- **`make bridge` first, always.** A flash attempt without it returns
  in four seconds with the socket error and NO "flashed and reset"
  line; the boards keep running the old image and everything after
  measures the wrong firmware. Check for the line.
- **Never `reset` a target from the probe**: nSRST is shared and
  restarts both boards. Per-board reset is SYSRESETREQ
  (`target-jtag`). Beacon reads need neither.
- **`FIELDS` in `radio_beacon.py` mirrors `beacon_t`.** Append only,
  on both sides, in the same order; a field-order slip once had
  `tx_short` read from the wrong offset.
- **`--raw` when a number is wanted.** The interpreted view rounds and
  omits; `--raw` prints every field by name.

## What the blocks mean

```
station: tx_frames 7  rx_decodes 33  my_req 12  listening NORMAL+EXTREME
bursts: starts 4  blocks 26  misses 0  refused 0
events: 42 (9 failed)  last: mode NORMAL type -1 typ 5 at 314 s
loudest thing heard: cs 130495759 at 149 s (quiet ~2e4, carrier ~2e8)
LED (PA1): solid (host attached, idle)
last events (newest last): <time> <mode> type <t> typ <p> start <abs> drops <n>
clean.
key-ups 7, last (ms cs floor): 138040 13911 3837 ...
```

- `rx_decodes` counts every decoded block, burst blocks included --
  a receiver of one 8-block stream shows 8 more decodes than the
  sender shows frames. `bursts: starts N blocks M misses 0` is a
  healthy streamed transfer; `misses > 0` or `blocks 0` with
  `starts > 0` is the walk failing (see `test-radio`, ladder step 1).
- `events` are receive attempts: `type 1` decoded, `-1` header CRC,
  `-2` bad version or oversize block, `-3` data CRC; `typ` is the
  packet type (4 DATA, 5 EXT_DATA, 6 BCAST, 0 BEACON). `start` is the
  absolute sample index of the header, so consecutive entries give
  the spacing of what the board heard.
- **Failed events on an idle wire are normal.** The quiet wire
  produces a header attempt every ~8 s at EXTREME on some runs and
  almost none on others (measured 30 of 37 events failed in one run,
  4 of 11 in the next, same firmware, same wire). They cost 0.27 s of
  listening at NORMAL and 4 s at EXTREME each. Attribute before
  blaming a change: see the replay below.
- `loudest thing heard` is carrier-sense mean-square; ~2e4 is a quiet
  wire, ~2e8 the peer's carrier. A value that is nearly constant
  across seconds while frames decode is a parked or floating DC
  level, not the wire (CLAUDE.md, mid-rail park).
- `key-ups` list the last four transmissions with the carrier-sense
  reading and its floor at the moment of keying. `cs=0` at every
  key-up is dead carrier sense; a `cs` far above the floor means the
  board keyed over something.
- `clean.` means none of the measured signatures fired: `adc_ready`/
  `rxs_ready`, `cap_overruns`, `tx_underruns`, `tx_short`,
  `tx_faults`, `ring_miss`, `keyup cs=0`. Each verdict names its
  measured cause when it fires.
- The beacon does not carry the notch bank or the preemption counter.
  Those are host-harness figures (`interf_rx`, `END ... preempts
  notches`); on the boards infer a notch only from the host replay.

## Two readings measure a rate

The counters are cumulative since boot. Read twice a known interval
apart and difference them: events per minute, failed attempts per
minute, decodes per transfer. `up N s` gives the interval when the
boards were not touched in between.

## When a verdict is not enough: dump the capture ring and replay it

The firmware keeps the last `CAP_N` = 65536 raw ADC samples (5.5 s)
in `g_cap`. Dumping it gives the host the exact audio the board
heard, and the host streaming receiver (`make -C cport interfrx`)
decodes it with the same code the board runs -- which turns "the
board did X" into a repeatable A/B.

```bash
cd cport
arm-none-eabi-nm -S build/usb_radio.elf | grep -E ' g_cap$| g_cap_w$'   # address, size
openocd -f ../tools/esp32-probe/stm32h7-rbb-dual.cfg -c init \
  -c "targets stmB.cpu0" -c "dump_image /tmp/capB.bin 0x24003084 131072" \
  -c "targets stmA.cpu0" -c "dump_image /tmp/capA.bin 0x24003084 131072" -c exit 2>&1 | grep -a dumped
```

- The bridge moves 0.64 kB/s: **200 s per 128 kB**, so run it in the
  background and do nothing else on the bridge meanwhile. Take the
  address from `nm` of the image that is ON the board, not the one
  just built.
- The ring is not rewound: the dump is rotated at `g_cap_w & (CAP_N-1)`
  and carries one discontinuity. For a stationary wire that does not
  matter; for a frame it does -- read `g_cap_w` (4 bytes before
  `g_cap`) and rotate.
- 5.5 s is shorter than an EXTREME tone window plus commit; loop a
  quiet-wire capture 6x to give the receiver 33 s (the seams are
  harmless on noise).

```bash
../venv/bin/python -c "import numpy as np; x=np.fromfile('/tmp/capB.bin',dtype=np.int16); np.tile(x,6).tofile('/tmp/capB_x6.raw')"
for m in 0 2; do ./build/interf_rx -m $m -k 90 /tmp/capB_x6.raw | awk '$1=="EV"{n[$3]++} $1=="END"{e=$0} END{for(k in n) printf "type %s x%d  ",k,n[k]; print e}'; done
```

To attribute a difference to a code change, build the host receiver
from the earlier sources and run the same file through both:

```bash
git stash push -q -- cport/src cport/gen_vectors.py ofdm_phy
( cd cport && gcc -std=c99 -O2 -Isrc -DMAX_LLRS=4416 -DINTERF_OLD -o build/interf_rx_old \
    bench/interf_rx.c src/rx_stream.c src/rx_detect.c src/rx_demod.c src/tx.c src/fft.c \
    src/dsp.c src/bits.c src/conv.c src/ldpc.c src/packets.c src/link.c src/arena.c -lm )
git stash pop -q
```

`-DINTERF_OLD` compiles the harness without the accessors newer
sources added (`rxs_preempts`, `rxs_notches`, `packets_set_net_key`).
Identical event counts on the same capture through both binaries is
the proof that the wire, not the receiver, changed -- that is how the
idle-wire attempts above were cleared (12 NORMAL / 4 EXTREME commits
through both, no notches). Stash and pop the whole tree, and check
`git status` afterwards: a pop that failed leaves the sources old.

## Related

- `test-radio`: the triage ladder that decides whether to read beacons
  at all (host harness first).
- `drive-boards`: scripted transfers whose consoles the beacon
  complements.
- `target-jtag`: the probe, the bridge, per-board reset, and reading
  arbitrary memory.
