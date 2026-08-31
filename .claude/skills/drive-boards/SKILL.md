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

mkdir -p /tmp/arx                     # B's own cwd -- rx_* files collide
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
  ps -eo pid,cmd | grep '[o]fdm_console --usb' | awk '{print $1}' > /tmp/pids
  while read p; do kill $p; done < /tmp/pids || true
  ```

  The loop occasionally exits 144 and takes the rest of a `&&` chain
  with it -- tolerate it, then verify the state and redo what was
  skipped.
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

## Reading the result

`status` (rung, SNR, queues, peer record), `stats` (counters), `cmp`
for byte-exactness, and the board beacons for anything the consoles
cannot see (`test-radio`). Time-scale traps do not apply here -- the
boards run in real time; that is `test-demoapp`'s problem, not this
stand's.
