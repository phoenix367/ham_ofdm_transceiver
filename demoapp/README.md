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
