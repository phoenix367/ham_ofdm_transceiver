#!/usr/bin/env bash
# IP over the OFDM link: set up, tear down, inspect.
#
# Works on the two AX.25 interfaces that a pair of kiss_bridge.py +
# kissattach sessions creates (see docs/drivers.md).
#
# READ THIS FIRST -- THE NAMESPACE DOES NOT WORK FOR AX.25
#   Measured on this stand: with the remote interface in a namespace,
#   frames arrive (the device's rx counter climbs) and the IP stack
#   never sees one -- "0 ICMP messages received" against 82 received
#   packets. AX.25 is not network-namespace aware; its receive handler
#   drops frames arriving on a device outside the initial namespace, so
#   nothing is ever answered.
#
#   There is no host-local way around it. To run IP over this link, put
#   one end on a SECOND MACHINE (or a VM with the board passed through).
#   `up` still configures both ends and is useful for the addressing and
#   the ARP entries; just do not expect a reply from the namespace side.
#
# THE TRAP THIS EXISTS FOR
#   Both interfaces live on the same host, so the kernel routes between
#   their addresses through loopback and never touches the radio: a ping
#   that "succeeds" in 40 microseconds while the interface's tx counter
#   never moves. The remote end therefore goes into a network namespace,
#   which is what makes it a genuinely separate host. Everything else
#   here is ordinary addressing.
#
#   Verify the traffic is real by watching the counters, not the ping:
#   `ax25_ip.sh status` prints them before and after.
#
# Usage:
#   sudo ./ax25_ip.sh up          assign addresses, namespace, static ARP
#   sudo ./ax25_ip.sh down        undo it, in the order that is safe
#        ./ax25_ip.sh status      what is configured, and the counters
#        ./ax25_ip.sh ping [n]    n pings with link-appropriate timing
#
# Knobs (environment): LOCAL_IF REMOTE_IF NS NET PREFIX MTU
set -euo pipefail

LOCAL_IF=${LOCAL_IF:-ax0}
REMOTE_IF=${REMOTE_IF:-ax1}
NS=${NS:-radio2}
NET=${NET:-10.73.0}
LOCAL_IP=${LOCAL_IP:-$NET.1}
REMOTE_IP=${REMOTE_IP:-$NET.2}
PREFIX=${PREFIX:-24}
# One packet, one transmission: a full 255-byte AX.25 frame exceeds the
# 255-byte single-frame payload cap and costs an extra ack cycle. Keep
# paclen in /etc/ax25/axports at the same value.
MTU=${MTU:-200}

die() { echo "ax25_ip: $*" >&2; exit 1; }
have_ns() { ip netns list 2>/dev/null | awk '{print $1}' | grep -qx "$NS"; }
in_ns() { ip netns exec "$NS" ip link show "$1" >/dev/null 2>&1; }
in_root() { ip link show "$1" >/dev/null 2>&1; }

need_root() {
    [ "$(id -u)" -eq 0 ] || die "needs root -- run with sudo"
}

# The AX.25 callsign of an interface, as the kernel reports it. Read it
# rather than hardcoding: it comes from /etc/ax25/axports and the ARP
# entries below must match it exactly.
callsign_of() {
    local iface=$1 ns=${2:-}
    if [ -n "$ns" ]; then
        ip netns exec "$ns" ip -d link show "$iface" 2>/dev/null \
            | awk '/link\/ax25/{print $2; exit}'
    else
        ip -d link show "$iface" 2>/dev/null \
            | awk '/link\/ax25/{print $2; exit}'
    fi
}

v4_of() {   # iface [ns] -> "a.b.c.d/nn" or "(no address)"
    local iface=$1 ns=${2:-} a
    if [ -n "$ns" ]; then
        a=$(ip netns exec "$ns" ip -4 -br addr show "$iface" 2>/dev/null \
            | awk '{print $3}')
    else
        a=$(ip -4 -br addr show "$iface" 2>/dev/null | awk '{print $3}')
    fi
    echo "${a:-(no address)}"
}

counters() {   # iface [ns] -> "rx N / tx N"
    local iface=$1 ns=${2:-} out
    if [ -n "$ns" ]; then
        out=$(ip netns exec "$ns" ip -s link show "$iface" 2>/dev/null)
    else
        out=$(ip -s link show "$iface" 2>/dev/null)
    fi
    echo "$out" | awk '/RX:/{getline; rx=$2} /TX:/{getline; tx=$2}
                       END{printf "rx %s / tx %s", rx, tx}'
}

do_up() {
    need_root
    in_root "$LOCAL_IF" || die "$LOCAL_IF not found -- is kissattach running?"

    local local_call remote_call
    local_call=$(callsign_of "$LOCAL_IF")
    [ -n "$local_call" ] || die "$LOCAL_IF is not an AX.25 interface"

    # read the remote callsign BEFORE the move: afterwards it is only
    # reachable from inside the namespace
    if in_root "$REMOTE_IF"; then
        remote_call=$(callsign_of "$REMOTE_IF")
    elif have_ns && in_ns "$REMOTE_IF"; then
        remote_call=$(callsign_of "$REMOTE_IF" "$NS")
    else
        die "$REMOTE_IF not found in the root namespace or in $NS"
    fi
    [ -n "$remote_call" ] || die "$REMOTE_IF is not an AX.25 interface"

    have_ns || ip netns add "$NS"
    if in_root "$REMOTE_IF"; then
        ip link set "$REMOTE_IF" netns "$NS"
    fi

    ip addr replace "$LOCAL_IP/$PREFIX" dev "$LOCAL_IF"
    ip link set "$LOCAL_IF" mtu "$MTU" up

    ip netns exec "$NS" ip link set lo up
    ip netns exec "$NS" ip addr replace "$REMOTE_IP/$PREFIX" dev "$REMOTE_IF"
    ip netns exec "$NS" ip link set "$REMOTE_IF" mtu "$MTU" up

    # Static ARP both ways. ARP would work, but a broadcast resolution
    # over a link whose frames take seconds is a poor way to start.
    arp -H ax25 -i "$LOCAL_IF" -s "$REMOTE_IP" "$remote_call"
    ip netns exec "$NS" arp -H ax25 -i "$REMOTE_IF" -s "$LOCAL_IP" "$local_call"

    echo "ax25_ip: up"
    echo "  $LOCAL_IF  $LOCAL_IP/$PREFIX  ($local_call)   root namespace"
    echo "  $REMOTE_IF  $REMOTE_IP/$PREFIX  ($remote_call)   namespace $NS"
    echo "  mtu $MTU -- keep paclen in /etc/ax25/axports the same"
    echo
    echo "  try:    $0 ping"
    echo "  watch:  $0 status     (the counters, not the ping, prove it)"
}

do_down() {
    need_root
    # ORDER MATTERS: bring the interface home before deleting the
    # namespace. Deleting a namespace destroys the devices left in it,
    # and this one belongs to a running kissattach.
    if have_ns && in_ns "$REMOTE_IF"; then
        ip netns exec "$NS" ip addr flush dev "$REMOTE_IF" || true
        ip netns exec "$NS" ip link set "$REMOTE_IF" netns 1
    fi
    have_ns && ip netns del "$NS" || true
    if in_root "$LOCAL_IF"; then
        arp -H ax25 -i "$LOCAL_IF" -d "$REMOTE_IP" 2>/dev/null || true
        ip addr flush dev "$LOCAL_IF" || true
    fi
    echo "ax25_ip: down ($REMOTE_IF returned to the root namespace,"
    echo "               namespace $NS removed, addresses flushed)"
}

do_status() {
    local ns_state="absent"
    have_ns && ns_state="present"
    echo "namespace $NS: $ns_state"
    if in_root "$LOCAL_IF"; then
        echo "  $LOCAL_IF ($(callsign_of "$LOCAL_IF")) $(v4_of "$LOCAL_IF")"
        echo "      $(counters "$LOCAL_IF")   mtu $(cat /sys/class/net/$LOCAL_IF/mtu 2>/dev/null)"
    else
        echo "  $LOCAL_IF: not present in the root namespace"
    fi
    if have_ns && in_ns "$REMOTE_IF"; then
        echo "  $REMOTE_IF ($(callsign_of "$REMOTE_IF" "$NS")) $(v4_of "$REMOTE_IF" "$NS")  [in $NS]"
        echo "      $(counters "$REMOTE_IF" "$NS")"
    elif in_root "$REMOTE_IF"; then
        echo "  $REMOTE_IF: still in the ROOT namespace -- IP between the two"
        echo "      would never reach the radio. Run '$0 up'."
    else
        echo "  $REMOTE_IF: not present"
    fi
    local arps
    arps=$(arp -H ax25 -n 2>/dev/null \
           | grep -E '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+' || true)
    echo "  arp:"
    if [ -n "$arps" ]; then
        echo "$arps" | sed 's/^/      /'
    else
        echo "      (none -- 'up' adds one each way)"
    fi
}

do_ping() {
    local n=${1:-3}
    echo "ax25_ip: $n ping(s) to $REMOTE_IP -- seconds per round trip is"
    echo "         normal here; tens of seconds if the ladder has decayed"
    echo "         and the bridge is holding the frame while it probes."
    ping -c "$n" -i 15 -W 60 -s 16 "$REMOTE_IP"
}

case "${1:-}" in
    up)     do_up ;;
    down)   do_down ;;
    status) do_status ;;
    ping)   shift; do_ping "${1:-3}" ;;
    *)      awk 'NR>1 && /^#/{sub(/^# ?/,""); print; next} NR>1{exit}' "$0"
            exit 1 ;;
esac
