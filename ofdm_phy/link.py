"""Link adaptation: rate ladder, link-control word, per-direction controller.

Two half-duplex stations form two directed links (Tx1->Rx2, Tx2->Rx1), each
governed by its RECEIVER: only the receiver knows its own noise floor, so it
requests the transmission rung, and the request rides in every frame's
20-bit link-control word. Modes are self-labeling at the PHY (per-mode ZC
preamble roots + header mod/spd fields), so a mode switch can never deadlock
the link -- worst case a frame is lost, the transmitter's loss fallback
kicks in, and both sides re-converge at rung 0 (EXTREME), which works
whenever anything works.

Link-control word (packed into Data.reserved, 20 bits):
    seq(2) | ack(2) | req_rung(4) | snr_report(4) | freq_corr(5) | flags(3)
SNR report: 2 dB steps over -24 .. +6 dB. freq_corr: the AFC/netting request
"shift your carrier by this many Hz", signed, 8 Hz steps, +-120 Hz per frame
(larger offsets converge iteratively). Stop-and-wait ARQ needs only 2-bit
sequence numbers.
"""

import collections
import typing
from dataclasses import dataclass, field

from .modes import LinkMode
from .packets import ModType, CCSpeed


@dataclass(frozen=True)
class Rung:
    mode: LinkMode
    mod: ModType
    spd: CCSpeed
    sens_db: float      # PER<=10% sensitivity, article channel, recal RX
    user_rate: float    # bit/s after FEC
    measured: bool      # False = interpolated from neighbouring measurements


# Dominated rungs (slower AND less sensitive than a neighbour) are omitted,
# e.g. NORMAL BPSK 2/3 (-5.2 dB, 235 bit/s) loses to NORMAL QPSK 1/3
# (-5.6 dB, 235 bit/s).
LADDER = [
    Rung(LinkMode.EXTREME, ModType.BPSK, CCSpeed.R13, -17.9, 7.8, True),
    Rung(LinkMode.ROBUST, ModType.BPSK, CCSpeed.R13, -11.7, 30.8, True),
    Rung(LinkMode.ROBUST, ModType.BPSK, CCSpeed.R12, -11.8, 46.2, True),
    Rung(LinkMode.ROBUST, ModType.QPSK, CCSpeed.R13, -11.3, 61.5, True),
    Rung(LinkMode.NORMAL, ModType.BPSK, CCSpeed.R13, -7.6, 117.6, True),
    Rung(LinkMode.NORMAL, ModType.BPSK, CCSpeed.R12, -7.3, 176.5, True),
    Rung(LinkMode.NORMAL, ModType.QPSK, CCSpeed.R13, -7.0, 235.3, True),
    Rung(LinkMode.NORMAL, ModType.QPSK, CCSpeed.R12, -5.3, 352.9, True),
    Rung(LinkMode.NORMAL, ModType.QPSK, CCSpeed.R23, -3.8, 470.6, True),
    Rung(LinkMode.NORMAL, ModType.QPSK, CCSpeed.R34, -2.2, 529.4, True),
    Rung(LinkMode.NORMAL, ModType.QAM16, CCSpeed.R12, 0.7, 705.9, True),
    Rung(LinkMode.NORMAL, ModType.QAM16, CCSpeed.R23, 2.6, 941.2, True),
    Rung(LinkMode.NORMAL, ModType.QAM16, CCSpeed.R34, 4.7, 1058.8, True),
]


# --- link-control word ------------------------------------------------------

FREQ_STEP_HZ = 8.0
FREQ_MAX_HZ = 15 * FREQ_STEP_HZ  # +-120 Hz per frame


@dataclass
class LinkControl:
    seq: int = 0
    ack: int = 0
    req_rung: int = 0
    snr_db: float = -24.0
    freq_corr_hz: float = 0.0  # "peer, shift your carrier by this much"
    flags: int = 0

    def pack(self) -> int:
        snr_q = max(0, min(15, int(round((self.snr_db + 24.0) / 2))))
        f_q = max(-15, min(15, int(round(self.freq_corr_hz / FREQ_STEP_HZ)))) + 15
        return ((self.seq & 3) << 18) | ((self.ack & 3) << 16) | \
               ((self.req_rung & 15) << 12) | (snr_q << 8) | \
               (f_q << 3) | (self.flags & 7)

    @classmethod
    def unpack(cls, v: int) -> "LinkControl":
        return cls(seq=(v >> 18) & 3, ack=(v >> 16) & 3,
                   req_rung=(v >> 12) & 15,
                   snr_db=((v >> 8) & 15) * 2.0 - 24.0,
                   freq_corr_hz=(((v >> 3) & 31) - 15) * FREQ_STEP_HZ,
                   flags=v & 7)


# --- controller -------------------------------------------------------------

@dataclass
class LinkController:
    """One per station; manages the receive-side request for the inbound
    link and the transmit-side rung choice for the outbound link."""

    margin_up: float = 2.5      # dB above rung sensitivity to move up
    margin_keep: float = 1.0    # dB below which the request drops
    stale_s: float = 90.0       # peer-request decay interval
    rx_stale_s: float = 45.0    # inbound-silence decay interval
    history_len: int = 5

    # RX side (inbound link)
    _snr_hist: typing.Deque = field(default_factory=lambda: collections.deque(maxlen=5))
    _my_req: int = 0
    last_rx_time: float = -1e9

    # TX side (outbound link)
    peer_req: int = 0
    peer_req_time: float = -1e9
    peer_report_db: float = -99.0  # peer's measured SNR of MY signal
    consecutive_losses: int = 0
    # learned per-rung sensitivity corrections: the static table comes from
    # one channel model; real deployments differ, so every ack/timeout
    # nudges the effective sensitivity of the rung that was used
    rung_offset_db: list = field(default_factory=lambda: [0.0] * len(LADDER))

    # --- receive side ------------------------------------------------------

    snr_max_age_s: float = 60.0

    def on_rx_frame(self, snr_db: float, lc: LinkControl, now: float):
        """Called for every successfully decoded inbound frame."""
        self._snr_hist.append((now, snr_db))
        self.last_rx_time = now
        self.peer_req = lc.req_rung
        self.peer_req_time = now
        self.peer_report_db = lc.snr_db
        self.consecutive_losses = 0

    def filtered_snr(self, now: float = None) -> float:
        """Fade-aware statistic: second-lowest of the recent, non-expired
        history (a low percentile, not the mean -- QSB punishes averages).
        Age-windowing matters: pre-fade measurements must not resurrect a
        high request after a dropout."""
        vals = [s for (t, s) in self._snr_hist
                if now is None or now - t <= self.snr_max_age_s]
        if not vals:
            return -99.0
        h = sorted(vals)
        return h[1] if len(h) >= 3 else h[0]

    def rx_request(self, now: float = None) -> int:
        """The rung I ask the peer to transmit at (with hysteresis).

        Inbound silence decays the request: measurements go stale the moment
        frames stop decoding, yet the request still travels in our own
        (possibly still-working) outbound frames with a fresh timestamp -- so
        the receiver itself must treat "I hear nothing" as evidence."""
        if now is not None and now - self.last_rx_time > self.rx_stale_s:
            decay = int((now - self.last_rx_time) // self.rx_stale_s)
            self._my_req = max(0, self._my_req - decay)
            self._snr_hist.clear()  # stale measurements must not come back
            return self._my_req

        snr = self.filtered_snr(now)
        best_up = 0
        for i, r in enumerate(LADDER):
            if snr >= r.sens_db + self.margin_up:
                best_up = i
        cur = LADDER[self._my_req]
        if best_up > self._my_req:
            self._my_req = best_up           # fast start / upshift
        elif snr < cur.sens_db + self.margin_keep:
            down = 0
            for i, r in enumerate(LADDER):   # highest rung still holding margin
                if snr >= r.sens_db + self.margin_keep:
                    down = i
            self._my_req = min(self._my_req, down)
        return self._my_req

    # --- transmit side -----------------------------------------------------

    def on_ack(self):
        self.consecutive_losses = 0

    def on_timeout(self):
        self.consecutive_losses += 1

    def note_outcome(self, rung: int, ok: bool):
        """Learn the deployment's real sensitivities: a loss at a rung makes
        it effectively less sensitive (+0.7 dB, asymmetric fast-up/slow-down),
        a success slowly earns the margin back (-0.15 dB)."""
        if ok:
            self.rung_offset_db[rung] = max(0.0, self.rung_offset_db[rung] - 0.15)
        else:
            self.rung_offset_db[rung] = min(6.0, self.rung_offset_db[rung] + 0.7)

    def tx_rung(self, now: float) -> int:
        """Rung for my next outbound frame: the peer's request, capped by the
        peer's fresh SNR report of my signal (the report reacts faster than
        request decay when my frames stop arriving -- it clamps to the floor
        as soon as the peer hears nothing), decayed by staleness, and cut by
        the loss-fallback ladder."""
        rung = self.peer_req

        cap = 0
        for i, r in enumerate(LADDER):
            if self.peer_report_db >= r.sens_db + self.rung_offset_db[i] + self.margin_keep:
                cap = i
        rung = min(rung, cap)

        age = now - self.peer_req_time
        if age > self.stale_s:
            rung = max(0, rung - int(age // self.stale_s))
        if self.consecutive_losses >= 4:
            rung = 0
        elif self.consecutive_losses >= 2:
            rung = max(0, rung - 2)
        return rung

    def tx_rung_for_class(self, now: float, qos: str = "bulk") -> int:
        """QoS classes are margin policy: control/interactive traffic rides
        one rung below the bulk choice (extra margin, shorter frames)."""
        rung = self.tx_rung(now)
        if qos in ("control", "interactive"):
            rung = max(0, rung - 1)
        return rung


def max_payload_bytes(rung: Rung, max_air_s: float = 8.0) -> int:
    """Air-time budget -> payload cap (latency QoS lever); the packet format
    itself caps payloads at 27 bytes."""
    return max(1, min(27, int(rung.user_rate * max_air_s / 8)))
