---
name: test-analog
description: Run the analog loopback test stand - a NORMAL frame from DAC1 (PA4) through a jumper into ADC1 (PA6) on one STM32H743, decoded on the board - and characterise the path from the host. Use to prove the DSP chain over real converters, or to bring up a second board's audio link.
---

# Analog loopback: DAC -> wire -> ADC on one board

The stand generates a whole frame first, plays it at 12 kHz while
recording the ADC sample for sample, then decodes the recording -- the
three phases are sequential because a station is half duplex and the
transmitter shares the receiver's arena.

```bash
cd cport && make analogfw                      # RAM-resident, 52 kB
# load + run via target-jtag (~85 s), ends in bkpt with the beacon filled
../venv/bin/python bench/analog_loop_dump.py   # pull both buffers, fit the path
```

## What a good run reads

```
beacon: stage 7  fs 11999.743 Hz  adc conv 19.2 us  n_cap 18432  fault 0  ev_type 1  bits_ok 1
fit:   gain  0.9992 (-0.01 dB)   delay +0 samples   corr 1.0000
noise: residual rms 7.8  ->  loop SNR 61.9 dB
VERDICT: path is live ... Decoder agreed: frame decoded, payload bit-exact.
```

62 dB is the 12-bit DAC's quantisation floor, so anything near it means
the wire and both converters are fine. The stand's clocks are
**measured** (TIM6 against DWT, ADC conversion against DWT), not
assumed; `fs` in the beacon is what was achieved.

## Reading the fit

- `corr < 0.5`, gain around -30 dB, delay 0: **no wire** -- but not
  silence either. That is crosstalk from PA4 into a floating PA6, and it
  proves the DAC is producing the frame before any wire exists.
- `corr ~1`, `bits_ok 0`: **the path is fine and the receiver is not.**
  Take the saved `/tmp/analog_loop/cap.npy` and push it through the host
  receiver; if the CLEAN digital waveform fails the same way, it was
  never analog. That is exactly how the streaming receiver's missing
  lag-N/2 unwrap was found (see `CLAUDE.md`).
- Gain well below 0 dB with `corr ~1`: check the DAC is not clipping at
  the rails (playback is at 3/4 scale for this reason).

## Pins and wiring

| ESP32-free STM32 pin | role |
|---|---|
| **PA4** DAC1_OUT1 | plays the frame, 0.5-3.1 V around 1.65 V |
| **PA6** ADC12_INP3 | records it; PA3 (INP15) / PC0 (INP10) are one-constant swaps |

A plain jumper. **Plugging it can reset the target** through the probe's
NRST line (see `target-jtag`); if the beacon reads garbage afterwards,
reload.

## Next step

Two boards: one transmits, the other records, with independent clocks.
The fit's `delay` and a slow drift in it become the sample-rate mismatch
measurement; the beacon's `fs` on each board is the absolute check the
one-board stand cannot make.
