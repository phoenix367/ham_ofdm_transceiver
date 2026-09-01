#!/usr/bin/env bash
# Measure the modem's own file-transfer throughput between two boards.
#
# This is the figure the technical report quotes (114 B/s at rung 12):
# the station's file transfer, which streams a whole selective-repeat
# window per transmission and amortises the acknowledgment over it.
# Traffic through the KISS bridge gets none of that -- every frame is
# its own message with its own ack -- so it runs at roughly half this.
# Compare like with like.
#
#   ./measure_sendfile.sh                 8 kB, both boards autodetected
#   ./measure_sendfile.sh -n 32768        a longer run
#   ./measure_sendfile.sh -a <tx> -b <rx> pick the direction
#   ./measure_sendfile.sh -W              skip the warm-up (measure a cold
#                                         link: the ladder starts at rung 0)
#
# It refuses to run while a bridge or console holds a board: one host
# program per board.
set -euo pipefail

BYTES=8192
WARM=1
WARM_WAIT=75
TIMEOUT=600
TX_SERIAL=""
RX_SERIAL=""
CON=${CON:-$(dirname "$(readlink -f "$0")")/../demoapp/build/ofdm_console}

usage() { sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }

while getopts "n:a:b:t:Wh" o; do
    case "$o" in
        n) BYTES=$OPTARG ;;
        a) TX_SERIAL=$OPTARG ;;
        b) RX_SERIAL=$OPTARG ;;
        t) TIMEOUT=$OPTARG ;;
        W) WARM=0 ;;
        *) usage ;;
    esac
done

die() { echo "measure: $*" >&2; exit 1; }

# The console prints "HH:MM:SS [TX] rung 12 (last tx 0)  SNR ..." and may
# prefix a "> " prompt; the rung can also read "none yet" before the
# first transmission. Checked against captured logs, where the obvious
# patterns returned nothing at all.
rung_of() {
    grep -aoE "\] rung [0-9]+" "$1" | tail -1 | awk '{print $NF}'
}

[ -x "$CON" ] || die "console not built: $CON (make -C demoapp)"

# One host program per board -- a running bridge or console holds the
# interface and this would fail with a libusb claim error halfway in.
if pgrep -f "[k]iss_bridge.py" >/dev/null || pgrep -x ofdm_console >/dev/null
then
    echo "measure: a bridge or console is holding a board. Stop them first:" >&2
    pgrep -af "[k]iss_bridge.py" >&2 || true
    pgrep -af "[o]fdm_console" >&2 || true
    exit 1
fi

if [ -z "$TX_SERIAL" ] || [ -z "$RX_SERIAL" ]; then
    mapfile -t FOUND < <("$CON" --list 2>/dev/null | awk '/serial/{print $NF}')
    [ "${#FOUND[@]}" -ge 2 ] || die "need two boards; found ${#FOUND[@]}"
    TX_SERIAL=${TX_SERIAL:-${FOUND[0]}}
    RX_SERIAL=${RX_SERIAL:-${FOUND[1]}}
fi
[ "$TX_SERIAL" != "$RX_SERIAL" ] || die "sender and receiver are the same board"

WORK=$(mktemp -d)
RXDIR="$WORK/rx"                     # the receiver stores into its own cwd
mkdir -p "$RXDIR"
SRC="$WORK/payload.bin"
cleanup() {
    exec 3>&- 4>&- 2>/dev/null || true
    [ -n "${TXPID:-}" ] && kill "$TXPID" 2>/dev/null || true
    [ -n "${RXPID:-}" ] && kill "$RXPID" 2>/dev/null || true
    sleep 0.3
    rm -rf "$WORK"
}
trap cleanup EXIT

# Random bytes on purpose: the console DEFLATEs a file before splitting
# it, and compressible data would measure zlib rather than the radio.
head -c "$BYTES" /dev/urandom > "$SRC"

mkfifo "$WORK/tx.in" "$WORK/rx.in"
( cd "$RXDIR" && "$CON" --usb "$RX_SERIAL" RX ) < "$WORK/rx.in" \
    > "$WORK/rx.log" 2>&1 &
RXPID=$!
"$CON" --usb "$TX_SERIAL" TX < "$WORK/tx.in" > "$WORK/tx.log" 2>&1 &
TXPID=$!
exec 3> "$WORK/tx.in"
exec 4> "$WORK/rx.in"
sleep 3
grep -q "attached to board" "$WORK/tx.log" || die "sender did not attach"
grep -q "attached to board" "$WORK/rx.log" || die "receiver did not attach"

if [ "$WARM" = 1 ]; then
    # The ladder decays about one rung per 90 s of silence, so a cold
    # link starts at rung 0 and the first frames are 20 s each. A small
    # bulk item also triggers the capability handshake, which is what
    # lets the transfer size its parts to the peer's window.
    echo "measure: warming the link (${WARM_WAIT}s) -- use -W to measure cold"
    echo "bulk 8" >&3
    sleep "$WARM_WAIT"
fi

echo "status" >&3
sleep 2
RUNG=$(rung_of "$WORK/tx.log")

echo "measure: sending $BYTES bytes at rung ${RUNG:-?}"
T0=$(date +%s.%N)
echo "sendfile $SRC" >&3

DEADLINE=$(( $(date +%s) + TIMEOUT ))
while ! grep -aq "stored as" "$WORK/rx.log"; do
    [ "$(date +%s)" -lt "$DEADLINE" ] || {
        echo "measure: TIMED OUT after ${TIMEOUT}s -- last lines:" >&2
        tail -3 "$WORK/tx.log" "$WORK/rx.log" >&2
        exit 1
    }
    sleep 1
done
T1=$(date +%s.%N)

echo "stats" >&3
echo "status" >&3
sleep 2

RX_FILE=$(ls "$RXDIR"/rx_* 2>/dev/null | head -1)
[ -n "$RX_FILE" ] || die "receiver reported a file but none is on disk"
cmp -s "$SRC" "$RX_FILE" && EXACT="byte-identical" || EXACT="MISMATCH"

ONAIR=$(sed -n 's/.*-> \([0-9]*\) on air.*/\1/p' "$WORK/tx.log" | tail -1)
RATIO=$(sed -n 's/.*, \([0-9.]*\)x,.*/\1/p' "$WORK/tx.log" | tail -1)
STATS=$(grep -aoE "tx [0-9]+ +rx [0-9]+ +timeouts [0-9]+ +retx [0-9]+" \
            "$WORK/tx.log" | tail -1)
RUNG_END=$(rung_of "$WORK/tx.log")

python3 - "$T0" "$T1" "$BYTES" "${ONAIR:-0}" <<'PY'
import sys
t0, t1, n, onair = float(sys.argv[1]), float(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
d = t1 - t0
print(f"\n  {n} bytes in {d:.1f} s  ->  {n/d:.1f} B/s user payload")
if onair:
    print(f"  {onair} bytes on air ->  {onair/d:.1f} B/s over the channel")
PY
echo "  rung $RUNG at the start, $RUNG_END at the end"
if [ -n "$RATIO" ]; then
    echo "  compression ${RATIO}x"
else
    echo "  incompressible (as intended: this measures the radio, not zlib)"
fi
echo "  $EXACT"
echo "  sender counters: ${STATS:-none}"
echo
echo "  (through the KISS bridge the same link runs at roughly half this:"
echo "   every frame is its own message with its own acknowledgment)"

echo quit >&3
echo quit >&4
sleep 1
[ "$EXACT" = "byte-identical" ] || exit 1
