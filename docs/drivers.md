# Channel drivers

The console applications speak one thing only: **12 kHz int16 audio over a
full-duplex byte stream**. Everything that differs between a simulation, a
real SDR, and a real board lives behind that interface, in a *driver*.
Nothing in `cport/` or the consoles changes when the channel does.

```
ofdm_console  <--audio socket-->  driver.py       simulated HF channel
ofdm_console  <--audio socket-->  sdr_driver.py   SSB over a real SDR
ofdm_console  <--USB frames--->   STM32 board     the modem IS the device
```

The third row is different in kind: with a board attached the DSP and the
station run **on the device**, and the host speaks the
[USB protocol](usb-protocol.md) instead of raw audio. See
[console.md](console.md) for that mode.

## `demoapp/driver.py` — the virtual channel

Exposes two station devices and a control device as Unix sockets
(default `/tmp/ofdmchan/`):

| device | protocol |
|---|---|
| `s1.sock`, `s2.sock` | raw int16 LE audio, both directions, one station each |
| `ctl.sock` | newline-delimited JSON configuration |

Channel model per direction, independently realized: propagation delay
(`delay_ms`) → Rayleigh fading (Doppler-band-limited complex gain,
`fading_hz`, 0 disables) → AWGN at `snr_db` relative to the running RMS of
active signal → half-duplex mute (a transmitting station hears only its
own noise floor).

Configuration at runtime through `chanctl.py`:

```bash
python3 driver.py --dir /tmp/ofdmchan --time-scale 8 &
./chanctl.py                         # print current config
./chanctl.py snr_db=-5 fading_hz=0.2 delay_ms=25
```

**The time-scale trap**: pacing is `TICK` samples every
`TICK/fs/time_scale` wall seconds. Protocol time is sample-derived, so
above what the host can decode in real time the two stations' clocks
drift apart and the transmitter times out before the receiver finishes.
Measured: 22 timeouts at 25×, 0 at 8× on the same transfer. Drop the
scale before debugging a timeout.

## `demoapp/sdr_driver.py` — SSB over a real SDR

The same socket interface, but the audio crosses a real radio (HackRF
One, or anything SoapySDR drives):

- **TX**: 12 kHz real audio → analytic signal (63-tap Hilbert, the same
  design the receiver uses) → interpolate to the SDR rate → optional NCO
  offset → int8 I/Q.
- **RX**: int8 I/Q → optional NCO → decimate to 12 kHz → real part
  (which *is* an SSB product detector) → int16 audio.

Tune the radio to the **suppressed-carrier** frequency; the 300–2400 Hz
audio band then rides `carrier+300..carrier+2400` on air, and LO error
between stations appears as exactly the audio-band CFO the modem tracks.

Two settings that are easy to get wrong (both measured, see the report):

- `--tx-ref` maps the audio peak to DAC full scale. The C transmitter
  reaches int16 full scale; a lower reference clips against the int8
  rail and nothing decodes.
- The SDR path costs ~20× the virtual channel; `--time-scale 2` is the
  ceiling, not 25.

`--selftest` and `--loopback` exercise the whole SSB/resampling/int8
path without hardware.

## `demoapp/usb_host.c` — the USB host driver

The C console's transport to a real board (libusb, vendor interface):

```c
int     usbh_list(void);                    /* enumerate boards, print serials */
usbh_t *usbh_open(const char *serial);      /* claim by 96-bit UID serial */
int     usbh_read(usbh_t*, void*, int cap, int timeout_ms);   /* 0 = quiet */
int     usbh_write(usbh_t*, const void*, int n);
int     usbh_stale(usbh_t*);                /* bytes drained at open */
void    usbh_close(usbh_t*);
```

Two rules earned by measurement (report §12.6):

- **Reopen drains, never resets.** The device pushes status frames
  continuously; a `clear_halt` on the IN endpoint while a transfer is
  armed desynchronizes TinyUSB from the hardware and wedges the
  endpoint. `usbh_open` reads until 150 ms of quiet and discards
  (`usbh_stale` reports how much).
- **The serial string read is retried, on a budget that outlasts a
  decode.** It is a control transfer, and one issued while the device
  is pushing bulk transiently fails — these boards push status at 2 Hz
  forever. The budget is 10 × 300 ms because the board stops servicing
  USB for as long as its worst blocking decode, 2283 ms measured; the
  original 4 × 50 ms could not outlast one. The Python host had no
  retry at all (3 opens in 12 failed) and, worse, pyusb caches a failed
  langid fetch as `()` so every later call raises instantly — a retry
  there must clear `dev._langids` or it is a no-op. `host/test_modem.py`
  pins both.
- **Writes wait 5 s.** The board stops servicing USB for as long as its
  worst blocking decode -- 2283 ms measured on the part -- so a shorter
  deadline turns a healthy board mid-frame into a write error. A
  timeout that moved zero bytes is retried once; a partial one never
  is, because repeating a frame's head desyncs the device's parser.

Device access needs the udev rule: `cport/usb/README.md` (install
`host/99-ofdm-modem.rules`).

## `host/kiss_bridge.py` — KISS, for existing packet software

A KISS TNC on one side, this modem on the other, so `kissattach`, APRS
clients and anything else that speaks KISS can use the link:

```bash
./kiss_bridge.py --tcp 8001        # Dire Wolf-style KISS over TCP
./kiss_bridge.py --pty             # prints a /dev/pts/N for kissattach
./kiss_bridge.py --mode broadcast  # frames go out non-ARQ, like AX.25 UI
```

It is a **host program on purpose**. The board already speaks a richer
protocol — rung, SNR, peer capabilities, die temperature, broadcast
pacing, diagnostics — and a KISS TNC has no way to report any of it;
putting KISS in the firmware would duplicate the transport, add a
fourth wire format to keep in step, and present the board as a dumb
TNC. Nothing in `cport/` changes for this.

### Using it with `kissattach`

`kissattach`'s second argument is a **port name from
`/etc/ax25/axports`**, not a label — an undefined one gives
`cannot find port radio in axports`, which is the first thing everyone
hits. Define it (the file ships with only comments):

```
# name  callsign   speed  paclen  window  description
ofdm    N0CALL-1   9600   200     2       OFDM modem
```

`paclen 200` is the value the air-time arithmetic below argues for;
`speed` is ignored on a pty. Then:

```bash
./kiss_bridge.py --pty --pty-link /tmp/kiss0 --serial <uid>
sudo kissattach /tmp/kiss0 ofdm        # brings up ax0
```

`--pty-link` matters because the pts number is allocated per run: the
symlink gives axports entries, scripts and unit files a name that
survives a restart, and it is removed on exit (including on SIGTERM, so
nothing is left pointing at a dead pts).

One host program per board: a console and a bridge cannot both hold the
same modem, and trying says so.

One KISS data frame is one AX.25 frame, boundaries preserved, in either
of two mappings: `message` (default) rides the station's ARQ
point-to-point, `broadcast` sends one non-ARQ transmission per frame —
the connectionless case AX.25 UI actually describes.

Three rules it enforces, all from the air-time arithmetic:

- **Air time, not byte count, is the limit.** A 256-byte frame is 2.6 s
  at rung 12, 5.1 s at rung 8, 18.4 s at rung 4 and **278.6 s at
  rung 0** — past every carrier-sense constant the station has. Frames
  over `--max-air` (45 s, the station's own fragment ceiling) are
  refused with a line saying why, rather than keying for four minutes.
  The estimate comes from a table generated from
  `ofdm_phy.station.estimate_air_time` and is asserted never to be
  optimistic (`host/test_kiss.py`) — the model itself is not imported,
  so the bridge needs only `pyusb` and starts in any environment that
  can reach the board.
- **`paclen` 200 is the sweet spot**: a full AX.25 frame is then ~216 B,
  inside the 255-byte single-frame payload cap, and 3.0 s at rung 10.
- **Connected-mode AX.25 is not supported.** Two ARQ engines with
  independent timers, one adapting the rung underneath the other, is a
  layering accident. Send UI frames and let this link do reliability.

### IP over the link

`host/ax25_ip.sh` sets up, tears down and inspects an IP configuration
across the two AX.25 interfaces:

```bash
sudo ./ax25_ip.sh up        # addresses, namespace, static ARP
     ./ax25_ip.sh status    # what is configured, and the counters
     ./ax25_ip.sh ping      # link-appropriate timing
sudo ./ax25_ip.sh down
```

**The trap it exists for:** both interfaces are on one host, so the
kernel routes between their addresses through loopback and never
touches the radio — a ping that "succeeds" in 40 µs while the tx
counter never moves. The script puts the remote interface in a network
namespace, which is what makes it a second host; `status` prints the
counters, because those, not the ping, are what prove the traffic was
real. Teardown returns the interface to the root namespace *before*
deleting the namespace, since deleting one destroys the devices left
inside it and this one belongs to a running `kissattach`.

Defaults are `10.73.0.1/24` and `.2` (not 44-net, which is really
allocated) at MTU 200, matching the `paclen` advice above so one packet
is one transmission. Expect **seconds per round trip**: ICMP and UDP
are fine, TCP's retransmit timers are not built for this.

**mkiss marks its first two frames after an attach.** The Linux KISS
line discipline probes the TNC for checksum support: the first frame it
sends carries the SMACK flag (bit 7 of the type byte) and the second
FLEX (bit 5), each with a 2-byte CRC appended, after which it falls
back to plain KISS permanently. Those bits live in the same nibble as
the port number, so a bridge that reads that nibble as a port sees
traffic for "port 8" and "port 2" and drops the first thing anyone
sends — measured on a live attach. The bridge strips the CRC, answers
plain, and only rejects genuine other-port traffic.

TXDELAY/P/SlotTime/TXtail/FullDuplex/SetHardware are accepted and
ignored: the station's carrier sense and turnaround are measured, not
configured from the host. `host/test_kiss.py` covers the codec and
every mapping decision without hardware.

Measured end to end, two ways. Through a TCP KISS client: a 48-byte
AX.25 UI frame crossed the two boards byte-identically in 3.4 s, 1.4 s
of it air at rung 12. Through the real Linux stack — two bridges, two
`kissattach`, `ax0` and `ax1` — a `beacon` on port 1 arrives on the
peer interface, including **the first frame after an attach**, which is
the mkiss probe case above; a second, 51-byte frame followed in under
15 s. (The interface byte counters differ by a few bytes between tx and
rx: kernel accounting around the KISS framing, not the payload —
byte-exactness is what the TCP measurement pins.)

## `host/ofdm_modem.py` — the Python host library

The same transport in Python (`OfdmModem`), with two backends: a real
board over pyusb (`_UsbTransport`, same drain-don't-reset opening rule)
or the hardware-free emulator as a subprocess (`_PipeTransport`,
`emulate=`). `encode()/Parser` implement the framing;
`decode_info()/decode_status()` the payloads; `events()` yields decoded
frames as `(kind, payload)`. Its `MAX_PAYLOAD` must track
`UP_MAX_PAYLOAD` in `cport/src/usb_proto.h`: a frame declaring more
than the cap is dropped as garbage, so a stale copy loses exactly the
big frames (a file part) while everything small keeps working.
`demoapp/board_console.py` is its
reference user; `demoapp/test_board_console.py` pins its file envelope
byte-for-byte against `app.c`.
