#!/bin/bash
# End-to-end check of the SDR signal path with the real console stations:
# sdr_driver.py --loopback cross-connects them through SSB modulation,
# resampling, int8 quantization and SSB demodulation -- everything a real
# HackRF would do except the device calls themselves.
#
# Two deliberate differences from smoke_test.sh:
#   * a reduced sample rate (240 ksps rather than the 2.4 Msps a HackRF
#     would use) -- identical code path, only the resampling factors differ
#   * a much lower time scale: the SDR path costs ~20x the virtual channel,
#     so the world cannot be accelerated 25x. At 2x everything keeps up.
set -u
cd "$(dirname "$0")"

DIR=$(mktemp -d)
LOG=$DIR/logs
mkdir -p "$LOG"
SCALE=${SCALE:-2}
RATE=${RATE:-240000}

python3 sdr_driver.py --loopback --rate "$RATE" --dir "$DIR" \
    --time-scale "$SCALE" --snr-db 18 > "$LOG/driver.log" 2>&1 &
DRV=$!
trap 'kill $DRV 2>/dev/null; rm -rf "$DIR"' EXIT
sleep 2

CTL=$(python3 chanctl.py --dir="$DIR" snr_db=18)
echo "$CTL" | grep -q '"snr_db": 18' || { echo "CTL FAIL: $CTL"; exit 1; }
echo "ctl ok: $CTL"

APP=$(pwd)/build/ofdm_console
( sleep 1; sleep 150; echo "stats"; echo "quit" ) \
    | (cd "$LOG" && "$APP" "$DIR/s2.sock" S2 > s2.log 2>&1) &
P2=$!
( sleep 2; echo "send HELLO OVER THE AIR"; sleep 150; echo "stats";
  echo "quit" ) \
    | (cd "$LOG" && "$APP" "$DIR/s1.sock" S1 > s1.log 2>&1) &
P1=$!
wait $P1 $P2 2>/dev/null

echo "--- S1:"; grep -E ">> frame|stats:" "$LOG/s1.log" | tail -4
echo "--- S2:"; grep -E "<< message|stats:" "$LOG/s2.log" | tail -3

if grep -q "HELLO OVER THE AIR" "$LOG/s2.log"; then
    echo "SDR SMOKE TEST PASS: message crossed the full SSB/SDR signal path"
    exit 0
fi
echo "SDR SMOKE TEST FAIL"
echo "--- full S1:"; cat "$LOG/s1.log"
echo "--- full S2:"; cat "$LOG/s2.log"
exit 1
