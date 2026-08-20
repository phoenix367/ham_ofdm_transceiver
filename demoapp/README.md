# Demo application: console messenger over a virtual HF channel

Two interactive station apps exchange text messages through a simulated
channel with real (or scaled) time, running the project's **C fixed-point
stack end to end**: fixed transmitter, three-mode streaming receiver
(`cport/rx_stream.c`), and the link-layer station (QoS, ARQ, rate
adaptation, simplex access from `cport/station.c`).

## Components

| File | Role |
|---|---|
| `driver.py` | virtual channel daemon: exposes the devices, moves 12 kHz int16 audio in paced ticks, applies propagation delay + Rayleigh fading + AWGN, enforces half-duplex |
| `app.c` → `build/ofdm_console` | the console station app |
| `chanctl.py` | channel configuration CLI (third device) |
| `smoke_test.sh` | automated end-to-end test at 25× time scale |

The driver exports three Unix-socket "devices" (default `/tmp/ofdmchan/`):
`s1.sock` and `s2.sock` are full-duplex raw int16 audio streams (one per
station), `ctl.sock` takes newline-delimited JSON configuration.

## Usage

```bash
make                                  # builds against ../cport

# terminal 0: the channel
python3 driver.py                     # real time; add --time-scale 10 to hurry
python3 driver.py --audio             # ...and HEAR it: left ear = station 1's
                                      # receiver, right ear = station 2's
                                      # (needs sounddevice; --volume 0.3)

# terminals 1 and 2: the stations
./build/ofdm_console /tmp/ofdmchan/s1.sock Alice
./build/ofdm_console /tmp/ofdmchan/s2.sock Bob

# terminal 3: degrade the channel mid-session
python3 chanctl.py snr_db=-5 fading_hz=0.2 delay_ms=25
python3 chanctl.py                    # show current config
```

App commands: `send <text>` (interactive class), `sendfile <path>`
(file transfer over the bulk class; files are split into 3 KB parts,
each one burst-ARQ transfer — up to ~24 KB per file with the default
queue depth), `bulk <n>` (n-byte test pattern), `tune <hz>` (manual LO fine-tune
through the station's trim-budget accounting — the virtual channel has
no trimmable oscillator, so the registered actuator stub only logs what
real hardware would be asked to do; the station on `s1.sock` registers
as the frequency anchor, the other as the trimming side), `debug on|off` (live
diagnostic event stream: every TX/RX, timeout, rung change with the
controller inputs that caused it, and burst state transition), `status`
(rung, SNR, CFO, queues, channel busy, plus the full link-controller
snapshot: cap, peer request/report, loss streak, request/rx age,
learned rung offset), `stats` (frame counters), `quit`.

Bulk transfers use the burst-ARQ extension (window of 8 frames per
bitmap acknowledgment) at NORMAL rungs and above — a file no longer
pays a full round trip per 25-byte fragment. On top of that, fragment
size scales with the engage-time rung (200 bytes at rung ≥ 10, 100 at
≥ 7, else 25) using extended `PKT_TYP_EXT_DATA` frames, so a good
channel amortizes the ~0.64 s preamble+header over up to 200 payload
bytes instead of 25: the smoke test's 5 KB file went from 211
transmitted frames (legacy) to ~45. The 3 KB part size keeps every
part inside the burst protocol's 127-fragment cap, so large files
never silently fall back to stop-and-wait (they did before this split
existed). Below NORMAL rungs the station still uses legacy
stop-and-wait — bursting 20-second EXTREME frames would be pointless.

All console events carry wall-clock timestamps. Received files are
stored in the app's working directory as `rx_<basename>` (basename only —
the peer cannot traverse paths); files travel in a one-byte-magic
envelope, so text messages are unaffected.

Expect radio pacing: the first exchange bootstraps at EXTREME
(~20 s of air time per frame in real time), after which the measured SNR
drives the ladder up and frames shrink to ~1 s. `--time-scale N` runs the
whole world N× faster; the apps clock the protocol from received samples,
so behaviour is identical at any scale.

## Real radio: `sdr_driver.py` (HackRF One and other SoapySDR devices)

The apps talk to a *driver* over a socket, so the channel behind it can be
swapped without touching the C stack or the console apps:

```bash
# no hardware: two stations cross-connected through the full SSB path
python3 sdr_driver.py --loopback --rate 240000 --time-scale 2 --snr-db 18

# verify the signal path (chunk continuity, round-trip fidelity, and a
# real OFDM frame pushed through TX -> RX and decoded)
python3 sdr_driver.py --selftest

# a real radio, one station per device
python3 sdr_driver.py --device driver=hackrf --freq 7.05e6 --rate 2.4e6
./build/ofdm_console /tmp/ofdmsdr/s1.sock MyCall
```

Signal path (USB, the model in `ofdm_phy/rf.py`): audio -> analytic signal
(the same 63-tap Hilbert design the receiver uses) -> interpolate to the
SDR rate -> int8 I/Q; and back the other way, where taking the real part
of the decimated baseband *is* an SSB product detector. Tune the radio to
the **suppressed carrier**: the 300-2400 Hz audio band then lands at
carrier+300..carrier+2400 Hz, and the LO error between two stations shows
up as exactly the audio-band CFO the modem already tracks.

Practical notes, in order of how much they bite:

- **Drive level matters more than anything else.** `--tx-ref` is the audio
  peak mapped to the DAC's full scale, i.e. the mic-gain knob. The C
  transmitter's frames run at rms ~14000 and touch int16 full scale, so
  the default is 32768; setting it lower clips the waveform against the
  int8 rail and nothing decodes (an 18 dB overdrive during development
  splattered ~9 % of the power out of band and killed every frame).
- **Sample rate** must be an integer multiple of 12 kHz. HackRF's minimum
  is 2 Msps, so 2.4 Msps (200x) is the natural choice.
- **CPU**: the SDR path costs roughly 20x the virtual channel, so it runs
  at about real time -- `--time-scale 2` is realistic for the loopback on
  a desktop, not the 25x `driver.py` sustains.
- **HackRF at HF is a weak receiver**: 8-bit ADC, no preselector, and the
  narrowest analog filter is 1.75 MHz, so it digitises +-875 kHz of a
  crowded band at once. Use a bandpass preselector for on-air receive.
- **TX is ~10 dBm and unfiltered.** Bench work over coax and attenuators
  first; on air needs a low-pass filter (harmonics), a licence, and
  realistically an external PA.
- **Clock**: a free-running HackRF is +-20 ppm (~140 Hz at 7 MHz), inside
  the modem's +-375 Hz range, but EXTREME integrates coherently over
  0.69 s symbols across a 44 s frame -- feed CLKIN from a 10 MHz
  TCXO/GPSDO for the slow modes.

`./sdr_smoke_test.sh` runs the two console stations over the loopback path
end to end, and `sdr_bringup.py` exercises a real radio:

```bash
python3 sdr_bringup.py --list                 # devices + their --device string
python3 sdr_bringup.py --rx --seconds 2       # levels, DC, spectrum, audio
python3 sdr_bringup.py --tx --i-have-a-dummy-load
```

### Measured on a HackRF One

Bring-up on real hardware confirmed the receive and transmit plumbing
(exact 2.400000 Msps, no timeouts over 4.8 M samples, 0.009 % clipping,
I/Q imbalance 0.00 dB, TX accepted every sample with air time matching to
50 ms) and produced four fixes worth knowing about:

- **Offset tuning is mandatory.** With the LO on the suppressed carrier
  the DC/LO leakage lands inside the 3.5 kHz passband and the recovered
  audio came out 98 % DC (DC 258.3 of an rms of 258.7). The device
  reports no hardware DC correction and no AGC, so `--if-offset` now
  defaults to 50 kHz on real hardware: the radio tunes below the carrier
  and the driver's NCO puts the signal back, leaving the spike far
  outside the decimation passband (DC 0.0 afterwards).
- **Set the gain stages, not the aggregate.** SoapySDR's aggregate
  `setGain` barely moved the level (32 -> 62 dB changed the I/Q rms by
  3 %), while LNA/VGA/AMP individually span rms 0.010 -> 0.94. Defaults
  are now `--lna 40 --vga 40 --amp 0`, worth 32 dB of working level
  (audio rms 5 -> 197); raise `--vga` until `--rx` reports clipping.
- **Wait for the transmit buffer to drain.** `writeStream` returns as
  soon as samples are queued -- 0.5 s of a 1.18 s frame was still unsent
  when it returned -- so switching straight back to receive truncated
  every frame. The half-duplex switch now waits out the queued air time.
- **Stream format is CF32**, not CS8; SoapySDR converts to the device's
  native int8 itself.

The transmitted waveform itself was checked on a spectrum analyser
connected to the antenna port (72 s of continuous EXTREME OFDM at
2.4 Msps, VGA 20, amp off, carrier placed at 7.000 MHz with the LO
offset-tuned 50 kHz away): a clean block from ~7.0000 to ~7.0025 MHz,
matching the designed occupied bandwidth of 23 subcarriers at 93.75 Hz
(carrier+300 .. carrier+2400 Hz), with no splatter beyond it. That
confirms the absolute tuning and the offset-tuning NCO arithmetic, that
the drive level is not over-driving the int8 DAC, and that the
modulation really is single-sideband -- a drive or sideband error shows
up as splatter or a mirror image. Reproduce with:

```bash
python3 sdr_bringup.py --tx --i-have-a-dummy-load --tx-mode extreme \
    --repeat 4 --freq 6.95e6 --if-offset 50000 --vga 20 --amp 0
```

(`--tx-mode extreme` transmits one ~18 s *continuous* frame rather than
short bursts, so the analyser sees the true occupied bandwidth instead
of the splatter of a pulsed signal.)

`tinysa.py` automates that measurement against a tinySA / tinySA Ultra
on USB -- it sweeps with the transmitter off, starts a transmission,
max-holds several sweeps, and writes a CSV and a plot:

```bash
python3 tinysa.py --measure-tx --sweep 6.94e6 7.01e6 --rbw 3 \
    --freq 7.0e6 --vga 20 --save results/sdr_tx_spectrum
```

Measured that way (carrier 7.000 MHz, VGA 20, amp off, 3 kHz RBW;
`results/sdr_tx_spectrum.csv` / `.png`):

| quantity | measured |
|---|---|
| signal peak | −38.2 dBm at 7.00176 MHz |
| −3 dB bandwidth | 2.42 kHz (design 2.1 kHz, plus 3 kHz RBW broadening) |
| LO leakage spur at 6.95 MHz | −73.1 dBm = **−35 dBc** |
| 2nd / 3rd / 4th harmonic | **−55 / −54 / −61 dBc** |
| wideband floor, TX on vs off | −92.6 vs −108.7 dBm (+16 dB) |
| signal above ambient | +61 dB |

Two of those are worth acting on before anything goes on air. The
harmonics at ~−55 dBc are the low-pass filter argument in numbers, and
they will get worse as the drive is raised. The LO spur at −35 dBc,
50 kHz from the signal, is the price of the offset tuning that keeps DC
leakage out of the *receive* passband -- a transmit-side DAC offset
calibration would remove it, and until then it is a spurious emission to
be aware of.

### Decoding off the air

The one thing the loopback could never prove is that a receiver recovers
frames from a real RF signal -- one half-duplex radio cannot hear itself.
An external transmitter closes that gap: a laptop sound card into an AM
modulator fed by a 7 MHz generator, through a 40 dB pad into the SDR.

```bash
python3 sdr_am_test.py --make-wav results/am_test.wav   # play this
python3 sdr_am_test.py --receive --carrier 7.0e6 --seconds 45
```

AM needs its own detector, and the reason is worth stating: our receiver
is SSB, so it tunes to a *suppressed* carrier and takes the real part of
the baseband. Feed that a double-sideband AM signal and the lower
sideband folds straight onto the upper one, while any frequency error
between the generator and the SDR multiplies the audio by a slow cosine
that splits every subcarrier in two. `sdr_am_test.py` therefore recovers
the carrier (AM supplies one), derotates by it so the carrier sits at
exactly 0 Hz and 0 phase, and only then takes the real part -- classic
synchronous detection, which also makes the generator's frequency error
irrelevant.

`sdr_waterfall.py` plots what the demodulator actually received
(`results/sdr_waterfall.png`): the whole capture plus each frame close
up, with the tone comb, ZC symbol, header and data spans computed from
the decoded header rather than fitted. The analysis FFT is 128 bins at
12 kHz -- the modem's own 93.75 Hz subcarrier grid -- so the Newman comb
is directly visible lighting only a few bins. Two things that make the
picture readable: `RxStats.start_sample` reports where the *header*
starts, so the frame begins one preamble earlier, and the ZC symbol
occupies all 23 carriers, so it looks like data unless drawn as part of
the preamble.

**Result** (recording kept as `results/am_rx_offair.wav`):

| frame | payload | SNR off air |
|---|---|---|
| NORMAL QPSK 1/2, 1.0 s | `RF TEST ONE` | +13.6 dB |
| NORMAL BPSK 1/3, 1.8 s | `RF TEST TWO` | +14.4 dB |
| ROBUST BPSK 1/3, 7.3 s | `RF TEST THREE` | +8.5 dB |

**All three transmitted frames recovered** from a real 7 MHz RF path,
across two link modes and two modulations, through the SDR, the
decimation chain and the OFDM demodulator. The decoded CFO is zero
because the carrier lock removes it. ROBUST reads a lower SNR by
design: its 16x tiling spreads the same energy over a longer symbol, so
the per-symbol estimate drops while the decoded margin grows.

(A coarse first scan found only two of the three -- it advanced 4 s
after each hit and \texttt{demod\_frame} returns one frame per window,
so it stepped over the 1.0 s QPSK frame. Worth knowing if you write
your own scanner: the miss was in the harness, not the link.) Note what this does and does not show: the
*receive* chain is validated over RF, but the transmitter here is an AM
modulator rather than our own SDR, so a full transmit-to-receive loop
still wants a second radio.

Set the modulation depth on the 1 kHz alignment tone the WAV starts
with: it is generated at the same *peak* as the data frames, and the
waveform's ~8 dB peak-to-average means a clipping modulator splatters it
exactly as an over-driven transmitter would.

### How hard can you drive it?

`sdr_drive_sweep.py` keys the radio with continuous OFDM at each TX VGA
setting and measures the fundamental, the shoulders 5-10 kHz outside the
occupied band, and the 2nd/3rd harmonics
(`results/sdr_drive_sweep.csv` / `.png`):

| VGA | output | shoulders | 2nd harm. | 3rd harm. |
|---|---|---|---|---|
| 0 dB | −58.6 dBm | −42/−43 dBc | −36 dBc | −35 dBc |
| 8 | −51.1 | −48/−49 | −42 | −42 |
| **16** | **−41.6** | **−51/−48** | **−50** | **−52** |
| 24 | −30.6 | −42/−42 | −56 | −63 |
| 32 | −24.1 | −39/−38 | −51 | −66 |
| 40 | −12.6 | −36/−27 | −48 | −64 |
| 47 | −7.1 | **−22/−25** | **−38** | −47 |

Read it from the middle outwards. The rows below VGA 16 are
*measurement*-limited, not transmitter-limited: at −58 dBm the shoulders
and harmonics being reported are the analyser's own noise floor. From
VGA 24 upwards the degradation is real amplifier compression, and the
waveform's ~8 dB PAPR is what provokes it: by VGA 47 the shoulders reach
−22 dBc and the 2nd harmonic −38 dBc, both worse than the −43 dBc
typically required of spurious emissions.

The important asymmetry: **a low-pass filter fixes the harmonics but
does nothing for the shoulders**, which are spectral regrowth a few kHz
from the carrier, inside any filter's passband. So the useful operating
window with this waveform is roughly **VGA 16-32 (−42 to −24 dBm)**,
where shoulders stay below about −38 dBc. Anything approaching the
HackRF's maximum is splattering on the neighbours regardless of what you
bolt on the output -- if you want real power, take a clean −30 dBm from
here into a *linear* external PA rather than winding this one up.

### Receiver calibration against a known source

The tinySA's **CAL output** is a reference tone at a documented level, so
feeding it into the SDR (here through a 40 dB pad) calibrates the receive
chain in absolute terms -- `sdr_rx_calibrate.py`, results in
`results/sdr_rx_calibration.csv`.  Note the free-running signal generator
(`mode low output`) is *not* usable for this: its level setting had no
measurable effect at the receiver across 56 dB, and it sweeps unless
pinned to zero span.  The CAL output just works.

At 10 MHz, −25 dBm nominal through 40 dB of pad (−65 dBm at the SDR):

| measurement | result |
|---|---|
| frequency error | −33.1 Hz at 10 MHz = **−3.31 ppm**, repeatable to 0.0 Hz |
| | i.e. −23 Hz at 7 MHz, against the modem's ±375 Hz acquisition range |
| linearity (VGA stepped 14→30 dB) | slope **1.002 dB/dB**, residual 0.03 dB |
| receiver noise, 6 kHz band | **−95.5 dBm** at the antenna port (NF ≈ 40 dB) |

and hence the first *absolute* sensitivities in the project, rather than
the relative SNR everything else is quoted in:

| rung | mode | rate | sensitivity |
|---|---|---|---|
| 0 | EXTREME BPSK 1/3 | 7.8 bit/s | **−113.4 dBm** |
| 4 | NORMAL BPSK 1/3 | 117.6 bit/s | −103.1 dBm |
| 12 | NORMAL QAM16 3/4 | 1059 bit/s | −90.8 dBm |

Two readings follow. The free-running HackRF's frequency error is
**negligible** for this waveform -- 3.3 ppm against a design range that
tolerates 50 -- so the CLKIN reference matters for EXTREME's coherent
integration, not for acquisition. And the 40 dB noise figure is bad
enough to matter but not fatal: against ITU-R P.372 atmospheric noise in
a 6 kHz band, external noise still dominates by +9 dB at a quiet rural
site, +17 dB typical rural and +24 dB in residential QRM. So the link
budget's assumption of an externally noise-limited receiver holds --
with only about 9 dB of margin at the quietest sites, where a
preselector and LNA would start to pay for themselves.

(The absolute numbers inherit the CAL output's −25 dBm nominal level;
`--cal-dbm` corrects them all if your unit differs.)

**A killed process used to leave the radio keyed.** Terminating the
transmit process without shutting the stream down left the HackRF
radiating at −37.7 dBm; opening a receive stream dropped it to
−98.7 dBm, 61 dB lower. `sdr_bringup.py` now traps SIGTERM/SIGINT and
unkeys in a `finally`, and `tinysa.py` stops the transmitter with SIGINT
and then forces the radio into receive before trusting any baseline
sweep. Worth remembering if you drive the radio from your own scripts.

CPU is not a constraint: 4.4 % of one core for transmit and 3.1 % for
receive per audio-second at 2.4 Msps.

Two honest limitations from the same session: a band survey (0.7-13 MHz)
found a medium-wave broadcaster 79.5 dB over the floor but *identical*
noise at 5/7/9.6/13 MHz, i.e. HF sits at the HackRF's own noise floor
rather than the atmospheric noise the link budget assumes -- a
preselector and LNA are needed for real HF receive. And while the transmitted
*spectrum* is confirmed clean and correctly placed, proving that a
receiver actually decodes it off the air still needs a second radio --
one half-duplex device cannot hear itself (phase 3).

## Design notes

- **Protocol time = samples received / 12 kHz.** Wall pacing (and
  `--time-scale`) never skews timeouts or air-time estimates.
- **`--audio` makes the sound card the pacing clock**: the driver's
  blocking stereo writes replace the sleep-based scheduler, so playback
  and the channel are sample-locked by construction (forces real time;
  `--time-scale` is ignored). Native 12 kHz output is used when the
  backend supports it, with a linear-interpolated 48 kHz fallback.
  What you hear: the EXTREME bootstrap is a long steady warble (~20 s per
  frame) alternating between ears; after adaptation the exchange becomes
  quick chirps, and turning the SNR down with `chanctl.py` brings the
  noise floor up audibly around them.
- **Carrier sense is relative**: the app tracks a min-EWMA noise floor and
  declares busy above 3× that RMS — an absolute threshold breaks the
  moment the channel noise changes (found by this demo's own smoke test).
- **Half-duplex** is enforced by the driver: a station transmitting hears
  only its noise floor.
- The noise level is sized from the running RMS of the *delivered* signal
  at the configured `snr_db`, so SNR keeps its meaning as the channel and
  fading change.
- The three streaming receivers (one per link mode) exploit the PHY's
  self-labeling preambles: only the sender's mode locks.
