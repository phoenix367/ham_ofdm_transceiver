#!/usr/bin/env bash
# Measure throughput THROUGH the AX.25 path: the KISS bridges, the
# stations, and the radio between them. Leave the bridges running --
# this measures them.
#
#   ./measure_ax25.sh ping [-c count] [-s bytes]   round trip, no root
#   sudo ./measure_ax25.sh tcp [-n bytes]          one-way bulk, needs root
#                                                  (the receiver runs in
#                                                   the namespace)
#
# Both modes read the AX.25 interface counters before and after, which
# is the link-layer truth: how many frames actually went out, how many
# arrived, and what the payload cost in air frames. The application
# number alone cannot tell a slow link from a lossy one.
#
# Set up the addressing first with ax25_ip.sh up.
#
# What to expect: a 216-byte frame is ~2.5 s of air at rung 12, and
# every frame is its own station message with its own acknowledgment,
# so tens of bytes per second is the right order. The station's own file
# transfer does about twice this (host/measure_sendfile.sh) because it
# streams a whole window per transmission -- that difference is the
# price of speaking someone else's protocol, not a fault.
set -euo pipefail

LOCAL_IF=${LOCAL_IF:-ax0}
REMOTE_IF=${REMOTE_IF:-ax1}
NS=${NS:-radio2}
REMOTE_IP=${REMOTE_IP:-10.73.0.2}
PORT=${PORT:-5001}

WORK_PING=$(mktemp)
trap 'rm -f "$WORK_PING"' EXIT

MODE=${1:-}
shift || true
COUNT=5
SIZE=150
BYTES=2048
TIMEOUT=900

while getopts "c:s:n:t:" o; do
    case "$o" in
        c) COUNT=$OPTARG ;;
        s) SIZE=$OPTARG ;;
        n) BYTES=$OPTARG ;;
        t) TIMEOUT=$OPTARG ;;
        *) ;;
    esac
done

die() { echo "measure-ax25: $*" >&2; exit 1; }

counters() {   # -> "packets bytes" for one direction
    local iface=$1 dir=$2 ns=${3:-} out
    if [ -n "$ns" ]; then
        out=$(ip netns exec "$ns" ip -s link show "$iface")
    else
        out=$(ip -s link show "$iface")
    fi
    echo "$out" | awk -v d="$dir" '
        $1 == d":" {getline; print $2, $1}'
}

check_setup() {
    ip -br addr show "$LOCAL_IF" 2>/dev/null | grep -q "[0-9]" \
        || die "$LOCAL_IF has no address -- run: sudo ./ax25_ip.sh up"
    ip netns list 2>/dev/null | awk '{print $1}' | grep -qx "$NS" \
        || die "namespace $NS is missing -- run: sudo ./ax25_ip.sh up"
}

report() {   # payload_bytes seconds  tx_before tx_after  rx_before rx_after
    python3 - "$@" <<'PY'
import sys
n, secs = int(sys.argv[1]), float(sys.argv[2])
txp0, txb0, txp1, txb1 = (int(x) for x in sys.argv[3:7])
rxp0, rxb0, rxp1, rxb1 = (int(x) for x in sys.argv[7:11])
txp, txb = txp1 - txp0, txb1 - txb0
rxp, rxb = rxp1 - rxp0, rxb1 - rxb0
if n:
    print(f"\n  {n} bytes in {secs:.1f} s  ->  {n / secs:.1f} B/s through AX.25")
else:
    print(f"\n  nothing completed in {secs:.1f} s")
print(f"  air: {txp} frame(s) out ({txb} B on the interface), "
      f"{rxp} in ({rxb} B)")
if txp:
    if n:
        print(f"  {txb / n:.2f}x interface bytes per payload byte "
              f"(AX.25 headers, and anything retransmitted)")
    lost = txp - rxp
    if lost > 0:
        print(f"  {lost} of {txp} frames did not come back "
              f"({100.0 * lost / txp:.0f} %)")
PY
}

case "$MODE" in
ping)
    check_setup
    read -r TXP0 TXB0 <<<"$(counters "$LOCAL_IF" TX)"
    read -r RXP0 RXB0 <<<"$(counters "$LOCAL_IF" RX)"
    MTU=$(cat "/sys/class/net/$LOCAL_IF/mtu" 2>/dev/null || echo 0)
    if [ "$MTU" -gt 0 ] && [ "$((SIZE + 28))" -gt "$MTU" ]; then
        echo "measure-ax25: -s $SIZE makes a $((SIZE + 28)) B packet, over"
        echo "              the $MTU B MTU: it would be sent as two IP"
        echo "              fragments and BOTH must survive for a reply."
        echo "              Use -s $((MTU - 28)) or less."
        exit 1
    fi
    echo "measure-ax25: $COUNT pings of $SIZE B to $REMOTE_IP"
    echo "              (each is one frame out and one back; seconds per"
    echo "               round trip is normal here)"
    T0=$(date +%s.%N)
    # -i 3 keeps the offered rate under what the link carries: faster
    # than that and the bridge drops the excess, which measures the
    # backlog rather than the link.
    ping -c "$COUNT" -i 3 -W 120 -s "$SIZE" "$REMOTE_IP" \
        | tee "$WORK_PING" || true
    T1=$(date +%s.%N)
    GOT=$(sed -n 's/.*, \([0-9]*\) received.*/\1/p' "$WORK_PING")
    GOT=${GOT:-0}
    read -r TXP1 TXB1 <<<"$(counters "$LOCAL_IF" TX)"
    read -r RXP1 RXB1 <<<"$(counters "$LOCAL_IF" RX)"
    if [ "$GOT" -eq 0 ]; then
        echo
        echo "  NO round trips completed -- there is no throughput to"
        echo "  report. Frames left this side (see the counters below);"
        echo "  nothing came back. Check the far side with:"
        echo "      sudo $0 check"
        echo
    fi
    # a completed ping carries the payload twice: out and back
    report "$((GOT * SIZE * 2))" \
           "$(python3 -c "print($T1-$T0)")" \
           "$TXP0" "$TXB0" "$TXP1" "$TXB1" "$RXP0" "$RXB0" "$RXP1" "$RXB1"
    ;;
tcp)
    [ "$(id -u)" -eq 0 ] || die "tcp mode needs root (the receiver runs \
inside the $NS namespace) -- try: sudo $0 tcp"
    check_setup
    WORK=$(mktemp -d); trap 'rm -rf "$WORK"' EXIT
    head -c "$BYTES" /dev/urandom > "$WORK/tx.bin"
    read -r TXP0 TXB0 <<<"$(counters "$LOCAL_IF" TX)"
    read -r RXP0 RXB0 <<<"$(counters "$LOCAL_IF" RX)"

    ip netns exec "$NS" nc -l -p "$PORT" > "$WORK/rx.bin" &
    NCPID=$!
    sleep 1
    echo "measure-ax25: $BYTES bytes over TCP to $REMOTE_IP:$PORT"
    echo "              TCP's timers assume milliseconds; a lost frame"
    echo "              costs seconds here, so this is 'what an ordinary"
    echo "              program gets', not the link's ceiling."
    T0=$(date +%s.%N)
    nc -q 5 -w "$TIMEOUT" "$REMOTE_IP" "$PORT" < "$WORK/tx.bin" || true
    # the sender returns when TCP has accepted the data; wait for the
    # receiver to have all of it on disk
    DEADLINE=$(( $(date +%s) + TIMEOUT ))
    while [ "$(stat -c %s "$WORK/rx.bin" 2>/dev/null || echo 0)" -lt "$BYTES" ]
    do
        [ "$(date +%s)" -lt "$DEADLINE" ] || { echo "measure-ax25: timed out" >&2; break; }
        sleep 1
    done
    T1=$(date +%s.%N)
    kill "$NCPID" 2>/dev/null || true
    read -r TXP1 TXB1 <<<"$(counters "$LOCAL_IF" TX)"
    read -r RXP1 RXB1 <<<"$(counters "$LOCAL_IF" RX)"
    GOT=$(stat -c %s "$WORK/rx.bin" 2>/dev/null || echo 0)
    report "$GOT" "$(python3 -c "print($T1-$T0)")" \
           "$TXP0" "$TXB0" "$TXP1" "$TXB1" "$RXP0" "$RXB0" "$RXP1" "$RXB1"
    if cmp -s "$WORK/tx.bin" "$WORK/rx.bin"; then
        echo "  byte-identical"
    else
        echo "  INCOMPLETE: $GOT of $BYTES bytes arrived"
    fi
    ;;
check)
    [ "$(id -u)" -eq 0 ] || die "check needs root -- try: sudo $0 check"
    echo "root namespace:"
    ip -br addr show "$LOCAL_IF" | sed 's/^/  /'
    echo "  routes:  $(ip route | grep -c "$LOCAL_IF") via $LOCAL_IF"
    echo "  arp:"
    arp -H ax25 -n 2>/dev/null | grep -E "^[0-9]" | sed 's/^/    /' \
        || echo "    (none -- the peer cannot be addressed from here)"
    echo "namespace $NS:"
    ip netns exec "$NS" ip -br addr show "$REMOTE_IF" | sed 's/^/  /'
    echo "  routes:  $(ip netns exec "$NS" ip route | grep -c "$REMOTE_IF") via $REMOTE_IF"
    echo "  arp:"
    ip netns exec "$NS" arp -H ax25 -n 2>/dev/null | grep -E "^[0-9]" \
        | sed 's/^/    /' || {
        echo "    (NONE -- this is why replies never come back: the far"
        echo "     side cannot turn 10.73.0.1 into a callsign. Fix with"
        echo "     sudo ip netns exec $NS arp -H ax25 -i $REMOTE_IF \\"
        echo "          -s <local ip> <local callsign>"
        echo "     or just re-run: sudo ./ax25_ip.sh up)"; }
    echo "does the far side's IP stack see them?"
    echo "  (frames counted by the interface but 0 ICMP received means"
    echo "   they are dropped BETWEEN the device and IP -- which is what"
    echo "   a network namespace does to AX.25: the protocol is not"
    echo "   namespace-aware and its receive handler discards frames"
    echo "   arriving on a device outside the initial namespace)"
    ip netns exec "$NS" netstat -s 2>/dev/null \
        | grep -iE "echo request|echo replies|ICMP messages received" \
        | sed 's/^/  /' || echo "  (netstat unavailable)"
    echo "  interface totals (packets bytes):"
    echo "    rx $(counters "$REMOTE_IF" RX "$NS")   tx $(counters "$REMOTE_IF" TX "$NS")"
    echo "counters:"
    echo "  $LOCAL_IF  tx $(counters "$LOCAL_IF" TX)   rx $(counters "$LOCAL_IF" RX)"
    echo "  $REMOTE_IF  tx $(counters "$REMOTE_IF" TX "$NS")   rx $(counters "$REMOTE_IF" RX "$NS")"
    ;;
*)
    sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
