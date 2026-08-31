---
name: drive-boards
description: Run a measurement on the two-board USB stand from a script, with no human at a keyboard - non-interactive consoles, background jobs, safe teardown, and the primers a run needs to be valid. Use for any transfer, throughput or broadcast run on the boards; test-radio is for triaging one that failed.
---

# Driving the stand from a script

The consoles are interactive programs, but a measurement run is not:
feed them on stdin, log stdout, and end with a marker the caller can
grep for.

```bash
CON=/home/ivan/projects/ofdm_transceiver_proto/demoapp/build/ofdm_console
A=320047000851333438363436   # or: $CON --list
B=240041000551333438363436

rm -rf /tmp/arx && mkdir -p /tmp/arx   # B's own cwd, emptied: rx_* files
                                      # collide, and a stale one compares clean
( sleep 1; echo "config diag_stream 1"; sleep 900; echo quit ) \
    | (cd /tmp/arx && $CON --usb $B B) > /tmp/b.log 2>&1 &

( sleep 2; echo "bulk 8";              # primer: see below
  sleep 20; echo "sendfile /tmp/f.bin";
  sleep 860; echo "status"; sleep 5; echo quit ) \
    | $CON --usb $A A > /tmp/a.log 2>&1

echo RUN-DONE
```

Run the whole thing in the background (`run_in_background`) and poll
its output; foreground `sleep` is blocked in this harness, so waits go
*inside* the script or into an until-loop.

## Rules, each one earned

- **Kill every console before attaching or flashing.** A stale process
  still holds the interface: "cannot claim the modem interface", and
  `make flash-radio-*` fails the same way.

  ```bash
  pgrep -x ofdm_console          # see them
  pkill -x ofdm_console || true  # -x matches the process NAME
  ```

  Match the name, not the command line: `pgrep -f 'ofdm_console --usb'`
  also matches the shell that is running the check, and a kill loop
  built from it can shoot its own harness. Any kill loop occasionally
  exits 144 and takes the rest of a `&&` chain with it -- tolerate it,
  then verify the state and redo what was skipped.

  Avoid `$1` and friends anywhere in a skill body: the loader
  substitutes them when it renders the file, so what gets read is not
  what is on disk.
- **Size the window past the transfer.** A 68 kB file at rung 12 is
  ~10 min; a 73 kB broadcast is 22. Consoles killed mid-transfer look
  exactly like a link failure. Estimate first (the board logs an ETA
  for broadcasts), then add half.
- **Prime the capability handshake with bulk.** `bulk 8` triggers it;
  an interactive `send` does not, and a first bulk transfer to a
  stranger splits its parts conservatively (window 8) because the
  record arrives too late to size it. A run that wants the ceilings
  measured needs the primer first.
- **B gets its own working directory.** `rx_<name>` and
  `rx_broadcast.bin` land in the console's cwd, and two runs in the
  same directory silently compare the wrong file.
- **`grep -c` prints 0 and exits 1**, which breaks `&&` chains -- add
  `|| true` or restructure.
- **Assert every scripted patch replacement.** A Python `str.replace`
  that matched nothing is silent; that is how carrier sense was left
  dead for a whole campaign. Write files only after all asserts pass,
  then verify with `grep`.

## A known-good run, for calibration

6000 random bytes, both boards idle beforehand (2026-09-01):

```
02:53:01  bulk 8 primer queued
02:54:31  status: rung 12, SNR +16.2 dB, peer record complete
          (stream ext ldpc burst bcast bcstats, 3328 B, window 16, ceiling 12)
02:54:33  sendfile -> 2 parts of 3184 B     (16 x 200 - envelope head)
02:55:25  stored as rx_f.bin, byte-identical
          tx 11 frames, timeouts 0, retx 0, usb resyncs 0
```

52 s for 6000 bytes is 115 B/s, which is the documented ladder figure
independently reproduced. Both consoles print "drained N stale bytes"
at attach -- that is the drain-don't-reset opening rule working, not a
fault. A `status` taken minutes after the last exchange reads
`SNR -99.0 dB`: the sentinel `ctl_filtered_snr` returns once every
sample has aged past `SNR_MAX_AGE_S`, with the rung and peer record
still intact. Not a link failure.

## Reading the result

`status` (rung, SNR, queues, peer record), `stats` (counters), `cmp`
for byte-exactness, and the board beacons for anything the consoles
cannot see (`test-radio`). Time-scale traps do not apply here -- the
boards run in real time; that is `test-demoapp`'s problem, not this
stand's.
