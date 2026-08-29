---
name: test-demoapp
description: Run the two-station demonstration end to end over the virtual channel - text plus a file transfer through the full C stack, byte-compared on arrival. Use after any change to cport/src/station.c, link.c, the streaming receiver, or demoapp/app.c; it is the only test that exercises the protocol against itself.
---

# The demo app, end to end

```bash
cd demoapp && make && SCALE=8 ./smoke_test.sh
```

Two console stations over `driver.py`'s simulated channel (18 dB SNR,
10 ms delay): S1 sends a text message, then a 5000-byte file as a
multi-part burst; the script byte-compares the received file. Expect:

```
02:44:08 [S2] << message (22 bytes): HELLO FROM STATION ONE
file transfer: rx_payload.bin matches (5000 bytes, multi-part burst)
SMOKE TEST PASS: text + file delivered, timestamps present
```

About two minutes at `SCALE=8`. The stats lines vary run to run --
`timeouts 0..4` on S1 and a few retransmissions are normal on this
channel; a PASS with `timeouts 22` is not (see the trap below).

## The time-scale trap

**Do not debug a timeout at `SCALE=25`.** Protocol time is
sample-derived, so above what the host can decode in real time the two
stations' clocks drift APART -- the transmitting side stays current, the
decoding side lags, and the transmitter times out before the receiver
has finished the burst. Signature: every timeout on one station, zero
on the other, each followed at once by the reply. Measured 22 timeouts
at 25x, 0 at 8x, same transfer. Drop the scale first.

## A bigger transfer, by hand

```bash
rm -rf /tmp/t && mkdir /tmp/t && head -c 14162 /dev/urandom > /tmp/t/big.bin
python3 driver.py --dir /tmp/t --time-scale 8 & sleep 3
python3 chanctl.py --dir=/tmp/t snr_db=18 delay_ms=10
# in two terminals (or the pattern in smoke_test.sh):
#   S2: ./build/ofdm_console /tmp/t/s2.sock S2      -> status ... quit
#   S1: ./build/ofdm_console /tmp/t/s1.sock S1      -> sendfile /tmp/t/big.bin
cmp /tmp/t/big.bin /tmp/t/rx_big.bin && echo byte-identical
```

`status` prints queue depths and the message store (`peak N of 12`);
`stats` prints tx/rx/retransmissions/timeouts. `debug on` streams the
diagnostic events. Fading: `chanctl.py fading_hz=0.5`.

## What it does NOT test

Real converters (`test-analog`), the USB host link (`test-usb`), or the
SDR path (`sdr_smoke_test.sh`, which needs `--tx-ref` set right and a
time scale of ~2x, not 25x).
