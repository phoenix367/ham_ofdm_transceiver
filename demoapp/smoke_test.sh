#!/bin/bash
# End-to-end smoke test: driver + two app instances, a text message must
# cross the simulated channel (EXTREME bootstrap -> adapted rungs).
set -u
cd "$(dirname "$0")"

DIR=$(mktemp -d)
LOG=$DIR/logs
mkdir -p "$LOG"

# DRIVER_CMD lets the same test run against a different channel back end
# (see sdr_smoke_test.sh, which points it at sdr_driver.py --loopback)
DRIVER_CMD=${DRIVER_CMD:-"python3 driver.py"}
SCALE=${SCALE:-25}
$DRIVER_CMD --dir "$DIR" --time-scale $SCALE > "$LOG/driver.log" 2>&1 &
DRV=$!
trap 'kill $DRV 2>/dev/null; rm -rf "$DIR"' EXIT
sleep 1.5

# control device: set + read back the channel config
CTL=$(python3 chanctl.py --dir="$DIR" snr_db=18 delay_ms=10)
echo "$CTL" | grep -q '"snr_db": 18' || { echo "CTL FAIL: $CTL"; exit 1; }
echo "ctl ok: $CTL"

# a payload file for the transfer leg
head -c 5000 /dev/urandom > "$DIR/payload.bin"
APP=$(pwd)/build/ofdm_console

( sleep 1; echo "status"; sleep 75; echo "stats"; echo "quit" ) \
    | (cd "$LOG" && "$APP" "$DIR/s2.sock" S2 > s2.log 2>&1) &
P2=$!
( sleep 2; echo "send HELLO FROM STATION ONE"; sleep 12; \
  echo "sendfile $DIR/payload.bin"; sleep 62; echo "stats"; echo "quit" ) \
    | (cd "$LOG" && "$APP" "$DIR/s1.sock" S1 > s1.log 2>&1) &
P1=$!

wait $P1 $P2 2>/dev/null

echo "--- driver:"; tail -3 "$LOG/driver.log"
echo "--- S1:"; grep -E "frame at rung|stats" "$LOG/s1.log" | tail -6
echo "--- S2:"; grep -E "message|stats" "$LOG/s2.log" | tail -4

OK=1
grep -q "HELLO FROM STATION ONE" "$LOG/s2.log" || { echo "text FAIL"; OK=0; }
if cmp -s "$DIR/payload.bin" "$LOG/rx_payload.bin"; then
    echo "file transfer: rx_payload.bin matches (5000 bytes, multi-part burst)"
else
    echo "file FAIL"; OK=0
fi
grep -qE "^[0-9]{2}:[0-9]{2}:[0-9]{2} \[S2\] << " "$LOG/s2.log" \
    || { echo "timestamp FAIL"; OK=0; }

if [ "$OK" = 1 ]; then
    echo "SMOKE TEST PASS: text + file delivered, timestamps present"
    exit 0
else
    echo "SMOKE TEST FAIL"
    echo "--- full S1:"; cat "$LOG/s1.log"
    echo "--- full S2:"; cat "$LOG/s2.log"
    exit 1
fi
