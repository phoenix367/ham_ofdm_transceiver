#!/usr/bin/env bash
# End-to-end test of ip_tun.py: both ends on this host, IP across the
# radio, in one command.
#
#   sudo ./ip_tun_test.sh              ping + a 2 kB TCP transfer
#   sudo ./ip_tun_test.sh -n 4096      more bytes
#   sudo ./ip_tun_test.sh -m 500       a smaller MTU (faster per packet)
#
# It starts one tunnel in this namespace and one in a namespace of its
# own -- which TUN allows and the kernel's AX.25 does not, the whole
# reason ip_tun exists -- pings, moves bytes with nc, and tears both
# ends down. Nothing else may hold the boards: stop any bridge or
# console first.
set -euo pipefail

HERE=$(dirname "$(readlink -f "$0")")
PY=${PY:-$HERE/../venv/bin/python}
CON=$HERE/../demoapp/build/ofdm_console
NS=${NS:-tuntest}
DEV=${DEV:-ofdm0}
LOCAL=10.99.0.1
REMOTE=10.99.0.2
BYTES=2048
MTU=1000
PORT=5099

while getopts "n:m:" o; do
    case "$o" in
        n) BYTES=$OPTARG ;;
        m) MTU=$OPTARG ;;
        *) sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
    esac
done

die() { echo "ip_tun_test: $*" >&2; exit 1; }
[ "$(id -u)" -eq 0 ] || die "needs root (it creates network devices) -- use sudo"
[ -x "$PY" ] || die "no venv python at $PY"

pgrep -f "[k]iss_bridge.py" >/dev/null && die "a KISS bridge is holding a board"
pgrep -x ofdm_console >/dev/null && die "a console is holding a board"

mapfile -t BOARDS < <("$CON" --list 2>/dev/null | awk '/serial/{print $NF}')
[ "${#BOARDS[@]}" -ge 2 ] || die "need two boards, found ${#BOARDS[@]}"
A=${BOARDS[0]}; B=${BOARDS[1]}

WORK=$(mktemp -d)
cleanup() {
    set +e
    [ -n "${PIDA:-}" ] && kill "$PIDA" 2>/dev/null
    [ -n "${PIDB:-}" ] && kill "$PIDB" 2>/dev/null
    sleep 1
    ip netns del "$NS" 2>/dev/null
    echo
    echo "--- tunnel A said ---"; tail -6 "$WORK/a.log" 2>/dev/null
    echo "--- tunnel B said ---"; tail -6 "$WORK/b.log" 2>/dev/null
    rm -rf "$WORK"
}
trap cleanup EXIT

ip netns list | awk '{print $1}' | grep -qx "$NS" || ip netns add "$NS"

echo "ip_tun_test: A=$A (this namespace)  B=$B (namespace $NS)  mtu $MTU"
"$PY" "$HERE/ip_tun.py" --serial "$A" --dev "$DEV" \
      --local "$LOCAL/24" --peer "$REMOTE" --mtu "$MTU" -v \
      > "$WORK/a.log" 2>&1 &
PIDA=$!
ip netns exec "$NS" "$PY" "$HERE/ip_tun.py" --serial "$B" --dev "$DEV" \
      --local "$REMOTE/24" --peer "$LOCAL" --mtu "$MTU" -v \
      > "$WORK/b.log" 2>&1 &
PIDB=$!

for i in $(seq 1 20); do
    sleep 1
    grep -q "up:" "$WORK/a.log" && grep -q "up:" "$WORK/b.log" && break
done
grep -q "up:" "$WORK/a.log" || { cat "$WORK/a.log"; die "tunnel A did not start"; }
grep -q "up:" "$WORK/b.log" || { cat "$WORK/b.log"; die "tunnel B did not start"; }
echo "ip_tun_test: both ends up"
ip -br addr show "$DEV" | sed 's/^/  /'
ip netns exec "$NS" ip -br addr show "$DEV" | sed 's/^/  /'

echo
echo "ip_tun_test: ping (a cold link starts at rung 0 -- the first may be"
echo "             dropped while the tunnel probes the ladder up)"
ping -c 4 -i 12 -W 90 -s 64 "$REMOTE" || true

echo
echo "ip_tun_test: $BYTES bytes over TCP"
head -c "$BYTES" /dev/urandom > "$WORK/tx.bin"
ip netns exec "$NS" nc -l -p "$PORT" > "$WORK/rx.bin" &
NCPID=$!
sleep 1
T0=$(date +%s.%N)
nc -q 5 -w 900 "$REMOTE" "$PORT" < "$WORK/tx.bin" || true
for i in $(seq 1 900); do
    [ "$(stat -c %s "$WORK/rx.bin" 2>/dev/null || echo 0)" -ge "$BYTES" ] && break
    sleep 1
done
T1=$(date +%s.%N)
kill "$NCPID" 2>/dev/null || true

GOT=$(stat -c %s "$WORK/rx.bin" 2>/dev/null || echo 0)
"$PY" - "$T0" "$T1" "$GOT" "$BYTES" <<'PY'
import sys
t0, t1, got, want = float(sys.argv[1]), float(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
d = t1 - t0
print(f"\n  {got} of {want} bytes in {d:.1f} s", end="")
print(f"  ->  {got/d:.1f} B/s" if got else "  (nothing arrived)")
PY
cmp -s "$WORK/tx.bin" "$WORK/rx.bin" && echo "  byte-identical" \
    || echo "  INCOMPLETE"
