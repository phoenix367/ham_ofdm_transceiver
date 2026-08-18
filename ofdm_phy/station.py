"""Complete link-layer station: QoS queues, segmentation/reassembly,
stop-and-wait ARQ, rate adaptation, and simplex channel access.

Simplex operation (both stations on one frequency, half-duplex radios):

- carrier sense: never transmit while the channel is busy; a timeout that
  expires while the channel is busy does NOT count as a loss (the peer may
  simply be answering at a slower rung than expected);
- listen window: after sending a frame that expects a reply, wait
  turnaround + estimated reply air time + margin before concluding loss;
- collision recovery: a timeout draws a random backoff before retransmit,
  breaking the symmetry of two stations that keyed up together;
- a station replies when the peer's frame carried data (to ack it) or when
  it has data of its own; two idle stations fall silent -- no ack-of-ack
  ping-pong.

The 3-bit LC flags: bit0 = last fragment of a message, bit1 = no-data
(ack-only) frame, bit2 = priority stream (control/interactive fragments,
which may PREEMPT an in-progress bulk message at fragment boundaries; the
receiver reassembles the two streams independently).

API (driven by a scheduler or a soundcard loop):
    submit(payload, qos)              queue application data
    poll_tx(t, channel_busy) -> sig   ask for a transmission (or None)
    on_tx_end(t)                      notify: my transmission finished
    rx_frame(samples, t)              feed a received audio burst
"""

import collections
import typing
from dataclasses import dataclass, field

import numpy as np

from .link import (LADDER, LinkControl, LinkController, max_payload_bytes,
                   FREQ_MAX_HZ)
from .modes import make_modem
from .packets import Data, Header
from .transceiver import Transceiver, CODECS, MAPPERS, HEADER_CODEC, HEADER_MAPPER, DemodError

FLAG_LAST_FRAGMENT = 1
FLAG_NO_DATA = 2
FLAG_PRIO_STREAM = 4  # fragment belongs to the control/interactive stream

FS = 12000
QOS_CLASSES = ("control", "interactive", "bulk")
QOS_MAX_AIR_S = {"control": 4.0, "interactive": 6.0, "bulk": 8.0}


def estimate_air_time(rung_idx: int, payload_len: int) -> float:
    """Deterministic frame air time for (rung, payload bytes) -- used for
    reply timeouts; no PHY invocation needed."""
    rung = LADDER[rung_idx]
    m = make_modem(rung.mode)
    preamble = 3 * m._newman_preamble_tile * m.fft_bins + m.symbol_len
    n_hdr = -(-HEADER_CODEC.calc_cc_elements(Header.PACKET_SIZE) //
              (m.data_carriers_len * HEADER_MAPPER.MU))
    bits = 20 + 8 * payload_len + 16
    coded = CODECS[rung.spd].calc_cc_elements(bits)
    n_data = -(-coded // (m.data_carriers_len * MAPPERS[rung.mod].MU))
    return (preamble + (n_hdr + n_data) * m.symbol_len) / FS

def _payload_cap(rung_idx: int, max_air_s: float) -> int:
    """Air-time budget -> payload cap, judged on the WHOLE frame.

    At the low rungs the fixed preamble+header (16.8 s at EXTREME) already
    exceeds every QoS budget, so splitting a message cannot bring a frame
    under budget -- it only pays that fixed cost once per fragment
    (measured: a 22-byte message at rung 0 went out as six 5-byte frames,
    ~4x the air time of one 38 s frame). Where the fixed cost already
    blows the budget, send the largest packet the format allows.
    """
    if estimate_air_time(rung_idx, 1) >= max_air_s:
        return 27
    return max_payload_bytes(LADDER[rung_idx], max_air_s)



@dataclass
class StationStats:
    tx_frames: int = 0
    rx_frames: int = 0
    retransmissions: int = 0
    timeouts: int = 0
    rung_trace: list = field(default_factory=list)  # (t, rung)


class LinkStation:
    AFC_DEADBAND_HZ = 12.0
    AFC_GAIN = 0.5  # damped: both sides act on stale measurements

    def __init__(self, name: str, rng: np.random.Generator,
                 turnaround: float = 0.3, timeout_margin: float = 2.0,
                 backoff_range: typing.Tuple[float, float] = (1.0, 6.0),
                 freq_trim_cb: typing.Callable[[float], None] = None,
                 afc_max_trim_hz: float = 150.0, afc_anchor: bool = False,
                 phy=None):
        # phy: optional PHY adapter (e.g. ofdm_phy.fixed.FixedPHY) providing
        # build_frame(pkt, mode, mod, spd) and demod_frame_auto(samples,
        # prev_data_llrs) -- None uses the float Transceiver chain.
        # freq_trim_cb(hz): shift this station's carrier (CAT/reference trim);
        # None disables AFC netting.
        # afc_max_trim_hz: HARD cumulative trim budget -- the netting loop
        # only observes the differential offset, so without a bound the
        # pair's common frequency random-walks over a long session (out of
        # the channel slot / bystanders' passbands / the rig's trim range).
        # Requests beyond the budget are partially honored or refused; the
        # peer's own headroom carries the rest, and any residual is left to
        # the modem's +-300 Hz tolerance.
        # afc_anchor: this station never trims (peers net onto it) -- pins
        # the absolute frequency, e.g. for the CQ caller.
        self.name = name
        self.rng = rng
        self.phy = phy
        self.turnaround = turnaround
        self.timeout_margin = timeout_margin
        self.backoff_range = backoff_range

        self.ctl = LinkController()
        self.queues = {q: collections.deque() for q in QOS_CLASSES}
        # two transmit streams: priority (control/interactive) may preempt
        # bulk at fragment boundaries
        self.cur_prio: typing.Optional[dict] = None
        self.cur_bulk: typing.Optional[dict] = None
        self.pending: typing.Optional[dict] = None  # unacked fragment
        self.seq = 0
        self.last_rx_seq = None  # None = nothing received yet (seq 0..7 are
                                 # all legitimate values -- no magic sentinel)

        self.await_until: typing.Optional[float] = None
        self.not_before = 0.0

        self.rx_assembly = {0: bytearray(), 1: bytearray()}  # per stream
        self.delivered: list = []  # completed inbound messages (bytes)
        self._harq_llrs = None  # failed-decode LLRs for chase combining
        self.stats = StationStats()
        self._modems: dict = {}
        self._reply_due = False
        self._expects_reply = False
        self._reply_rung_guess = 0
        self.freq_trim_cb = freq_trim_cb
        self.afc_max_trim_hz = afc_max_trim_hz
        self.afc_anchor = afc_anchor
        self._afc_total_hz = 0.0
        self._last_cfo_hz = None  # peer offset measured on the last rx frame

    # --- application side ---------------------------------------------------

    def submit(self, payload: bytes, qos: str = "bulk"):
        assert qos in QOS_CLASSES
        self.queues[qos].append(bytes(payload))

    def has_traffic(self) -> bool:
        return (self.pending is not None or self.cur_prio is not None
                or self.cur_bulk is not None or any(self.queues.values()))

    # --- transmit side ------------------------------------------------------

    def _trx(self, rung_idx: int) -> Transceiver:
        mode = LADDER[rung_idx].mode
        if mode not in self._modems:
            self._modems[mode] = Transceiver(make_modem(mode))
        return self._modems[mode]

    def _take_fragment(self, rung_idx: int) -> typing.Optional[dict]:
        """Pop the next fragment to send. The priority stream (control /
        interactive) preempts an in-progress bulk message at fragment
        boundaries -- no head-of-line blocking behind a long bulk transfer."""
        if self.pending is not None:
            return self.pending
        if self.cur_prio is None:
            for q in ("control", "interactive"):
                if self.queues[q]:
                    self.cur_prio = {"data": self.queues[q].popleft(), "qos": q, "off": 0}
                    break
        if self.cur_prio is None and self.cur_bulk is None and self.queues["bulk"]:
            self.cur_bulk = {"data": self.queues["bulk"].popleft(), "qos": "bulk", "off": 0}

        src = self.cur_prio if self.cur_prio is not None else self.cur_bulk
        if src is None:
            return None
        stream = 1 if src is self.cur_prio else 0
        cap = _payload_cap(rung_idx, QOS_MAX_AIR_S[src["qos"]])
        off = src["off"]
        chunk = src["data"][off:off + cap]
        last = off + len(chunk) >= len(src["data"])
        self.seq = (self.seq + 1) & 3
        self.pending = {"chunk": chunk, "last": last, "qos": src["qos"],
                        "seq": self.seq, "stream": stream, "first_try": True}
        return self.pending

    def poll_tx(self, t: float, channel_busy: bool):
        """Returns audio samples to transmit now, or None."""
        if channel_busy:
            return None  # carrier sense; also keeps a pending timeout alive
        if t < self.not_before:
            return None
        if self.await_until is not None:
            if t < self.await_until:
                return None
            # reply timeout on an idle channel -> loss
            self.await_until = None
            self.stats.timeouts += 1
            self.ctl.on_timeout()
            if getattr(self, "_last_tx_rung", None) is not None:
                self.ctl.note_outcome(self._last_tx_rung, False)
            self.not_before = t + float(self.rng.uniform(*self.backoff_range))
            if self.pending is not None:
                self.pending["first_try"] = False
            return None  # transmit on the next poll after backoff

        owes_ack = self.last_rx_seq is not None and self._reply_due
        if not owes_ack and not self.has_traffic():
            return None

        qos = self.pending["qos"] if self.pending else \
            next((q for q in QOS_CLASSES if self.queues[q]), "control")
        rung_idx = self.ctl.tx_rung_for_class(t, qos if self.has_traffic() else "control")

        frag = self._take_fragment(rung_idx)
        if frag is not None:
            if not frag.get("first_try", True):
                self.stats.retransmissions += 1
            payload = frag["chunk"] if frag["chunk"] else b"\x00"
            flags = (FLAG_LAST_FRAGMENT if frag["last"] else 0) | \
                    (FLAG_PRIO_STREAM if frag.get("stream") else 0)
            seq = frag["seq"]
            expects_reply = True
        else:
            payload = b"\x00"
            flags = FLAG_NO_DATA
            seq = self.seq
            expects_reply = False

        # AFC/netting: ask the peer to move its carrier onto ours when the
        # measured offset exceeds the deadband
        freq_req = 0.0
        if self.freq_trim_cb is not None and self._last_cfo_hz is not None \
                and abs(self._last_cfo_hz) > self.AFC_DEADBAND_HZ:
            freq_req = float(np.clip(-self._last_cfo_hz, -FREQ_MAX_HZ, FREQ_MAX_HZ))

        lc = LinkControl(seq=seq, ack=self.last_rx_seq if self.last_rx_seq is not None else 0,
                         req_rung=self.ctl.rx_request(t),
                         snr_db=self.ctl.filtered_snr(t),
                         freq_corr_hz=freq_req, flags=flags)
        pkt = Data(reserved=lc.pack(), payload=payload)
        rung = LADDER[rung_idx]
        if self.phy is not None:
            sig = self.phy.build_frame(pkt, mode=rung.mode, mod=rung.mod,
                                       spd=rung.spd)
        else:
            sig = self._trx(rung_idx).build_frame(pkt, mod=rung.mod, spd=rung.spd)

        self.stats.tx_frames += 1
        self.stats.rung_trace.append((t, rung_idx))
        self._last_tx_rung = rung_idx
        self._reply_due = False
        self._expects_reply = expects_reply
        self._reply_rung_guess = self.ctl.rx_request(t)
        return sig

    def on_tx_end(self, t: float):
        if self._expects_reply:
            reply_air = estimate_air_time(self._reply_rung_guess, 1)
            self.await_until = t + self.turnaround + reply_air + self.timeout_margin
        else:
            self.await_until = None
        self.not_before = t + self.turnaround / 2

    # --- receive side -------------------------------------------------------

    def rx_frame(self, samples: np.ndarray, t: float):
        """Feed one received burst; returns list of completed messages."""
        try:
            if self.phy is not None:
                pkt, stats, mode = self.phy.demod_frame_auto(
                    samples, prev_data_llrs=self._harq_llrs)
            else:
                pkt, stats, mode = Transceiver().demod_frame_auto(
                    samples, prev_data_llrs=self._harq_llrs, llr_recal="auto")
        except DemodError as exc:
            # keep the failed attempt's LLRs for chase combining with the
            # expected retransmission (CRC gates a wrong guess)
            if exc.data_llrs is not None:
                self._harq_llrs = exc.data_llrs
            return []
        if stats.harq_combined:
            self.stats.harq_combines = getattr(self.stats, "harq_combines", 0) + 1
        self._harq_llrs = None

        lc = LinkControl.unpack(pkt.reserved)
        self.stats.rx_frames += 1
        self.ctl.on_rx_frame(stats.snr_db, lc, t)
        self.await_until = None  # got a frame; the exchange continues
        self._last_cfo_hz = stats.cfo_hz

        # apply the peer's netting request to our reference: damped, and
        # clamped to the cumulative trim budget (anchors never trim)
        if self.freq_trim_cb is not None and lc.freq_corr_hz != 0.0 \
                and not self.afc_anchor:
            delta = self.AFC_GAIN * lc.freq_corr_hz
            new_total = float(np.clip(self._afc_total_hz + delta,
                                      -self.afc_max_trim_hz, self.afc_max_trim_hz))
            delta = new_total - self._afc_total_hz
            if delta != 0.0:
                self.freq_trim_cb(delta)
                self._afc_total_hz = new_total
                self.stats.afc_trims = getattr(self.stats, "afc_trims", 0) + 1

        done = []
        # ARQ: does their ack cover my pending fragment?
        if self.pending is not None and lc.ack == self.pending["seq"]:
            src = self.cur_prio if self.pending.get("stream") else self.cur_bulk
            src["off"] += len(self.pending["chunk"])
            if self.pending["last"]:
                if self.pending.get("stream"):
                    self.cur_prio = None
                else:
                    self.cur_bulk = None
            self.pending = None
            self.ctl.on_ack()
            if getattr(self, "_last_tx_rung", None) is not None:
                self.ctl.note_outcome(self._last_tx_rung, True)

        if not (lc.flags & FLAG_NO_DATA):
            if lc.seq != self.last_rx_seq:  # not a duplicate
                stream = 1 if lc.flags & FLAG_PRIO_STREAM else 0
                self.rx_assembly[stream].extend(pkt.payload)
                if lc.flags & FLAG_LAST_FRAGMENT:
                    done.append(bytes(self.rx_assembly[stream]))
                    self.delivered.append(bytes(self.rx_assembly[stream]))
                    self.rx_assembly[stream] = bytearray()
            self.last_rx_seq = lc.seq
            self._reply_due = True  # data frames must be answered (ack)
        return done
