---
name: test-radio
description: Diagnose the two-board radio link (STM32 DAC->wire->ADC, USB modem firmware) - run a transfer between the boards, read both beacons with named fields and failure verdicts, and triage with the host burst harness BEFORE blaming hardware. Use when messages, files or streamed bursts fail between the boards, or after changing rx_stream/station/usb_radio_main.
---

# Two-board radio diagnostics

The stand: two STM32H743s cross-wired PA4(DAC)->PA6(ADC) both ways,
one ESP32 JTAG probe daisy-chained through both, both running the
flash-resident radio firmware (`usb/usb_radio_main.c`).

## The triage ladder -- in this order

The streamed-burst failure took a day because plausible theories
(clock drift, quantization, resync) all pointed at hardware, and every
one was wrong. The ladder below is the order that actually converges.

**1. Host harness first.** Before any board cycle (7 min each),
reproduce off-board with the firmware's exact waveform and walk logic:

```bash
cd cport && make robust     # the whole reliability gate: carrier-sense
                            # scenario suite + the input-abuse decode
                            # matrix (DC, steps, clip, hum, impulses,
                            # stuck converter) -- every line must pass
./build/burst_repro -q -dcb -dc -16577 -step -hum -imp 5   # one-off combos
./build/burst_repro -f 100 -r 7 -n 3 0   # the frag size that found MAX_LLRS
```

Carrier sense is `src/csense.c` and the DC blocker `src/dcblock.h` --
shared and host-tested (`make cstest`), because every field failure of
the 8 kB stress campaign lived in carrier sense while the demodulator
survived the full abuse matrix bare.

If the harness fails, the bug is in the shared C code -- fix it on the
host where a cycle is seconds. If the harness passes everything, the
bug is in the FIRMWARE's interaction (ISR, FIFOs, arena, carrier
sense) -- that is what the beacon is for. `-q` DAC/ADC requantization,
`-d` mid-rail DC, trailing number = sample-clock ppm; the DSP was
measured to survive all of them at once (8/8 at 56 ppm).

**2. Run a transfer with diagnostics on.** Two consoles, `--serial`
picks the board (`board_console.py --list`):

```bash
# terminal B (receiver side)
demoapp/board_console.py --serial <B> --name B   # then: config diag_stream 1
# terminal A
demoapp/board_console.py --serial <A> --name A   # config diag_stream 1; sendfile <f>
```

`diag BURST_STREAM a=<blocks>` = a streamed window went out;
`BURST_ACKRX a=<acked> b=<total>` = what the bitmap acked (a=8 b=11 is
a healthy 8-block stream); `BURST_SOFF a=2` NOACK / `a=3` TIMEOUT =
streaming abandoned; `RX ... d=<snr*10>` on the receiver shows every
decode. A transfer that works per-frame but dies streamed is a
firmware-interaction bug, not a channel one -- see the ladder's step 1.

**3. Read both beacons.** Needs the bridge (`make bridge`):

```bash
cd cport && ../venv/bin/python bench/radio_beacon.py      # or python3
```

The tool prints per-board verdicts for every failure signature that
has actually been measured on this stand:

| signature | meaning (measured cause) |
|---|---|
| `keyup cs=0` at every key-up | carrier sense dead (the uncalled `note_busy_isr` regression); quiet wire reads ~2e4 |
| walk miss at ~-30 dB, garbage lead bits | signal vanished mid-stream: this board keyed over it, or the peer's carrier stopped |
| walk miss with sane subheader (`0x8N n fs`) | aligned but noisy -- a real channel problem |
| `tx_short > 0` | generator rendered fewer samples than `build()` promised -- peer heard a truncated frame (the FIFO-capacity double-count bug) |
| `cap_overruns > 0` | capture FIFO smaller than the decoder's worst blocking burst (2283 ms commit measured) |
| `burst_starts > 0, blocks = 0` | stream detected, every continued block failed -- run step 1 with the engage's `frag_size` (`BURST_ENGAGE b=`); frag >= ~100 with `MAX_LLRS` too small decodes 0/N |
| `tx_faults > 0` | half-duplex arena violation |

**4. Only then JTAG-halt things.** `target-jtag` / `measure-target`
skills; per-board reset is SYSRESETREQ (`targets stmA.cpu0; mww
0xE000ED0C 0x05FA0004`), never `reset` -- nSRST is shared and restarts
both boards.

## Rebuild / reflash

```bash
cd cport && make flash-radio-a flash-radio-b \
    TINYUSB=/tmp/tinyusb CMSIS_H7=/tmp/cmsis_h7 CMSIS5=/tmp/cmsis5
```

Flashing one board does not disturb the other (`ab_flash` halts only
its target). AXI is full to within ~450 B -- the linker errors on
overflow; ISR-hot or oversized objects go to DTCM via the linker
script (`g_d64`, `g_bring` are there already).

## Invariants that bite here (full list in CLAUDE.md)

- The beacon struct is APPEND-ONLY and `radio_beacon.py`'s FIELDS list
  must match it -- the inline-parser era produced one field-order slip.
- A receiver whose walk expects blocks holds its transmitter; block 0
  of a stream carries the ack request on purpose.
- Assert every scripted patch replacement -- the carrier-sense
  regression was a silent `str.replace` no-op.
