"""Broadcast (non-ARQ) mode: speech and telemetry with no retransmission.

The transport already exists -- a broadcast is a streamed burst
(`Transceiver.build_stream`, see docs/phy.md) with the acknowledgment
machinery subtracted: no ack request, no window, no selective repeat,
no reply timer. What is left to define is how a receiver that was not
listening at the start can join, and what it reports back.

    [preamble][header][f0 SYNC][f1][f2]..[fN]   <- one stream group
    [preamble][header][f0 SYNC][f1][f2]..[fN]   <- the next, N frames later

Every group re-sends the preamble and a SYNC frame carrying the stream
descriptor, so a receiver that tunes in mid-transmission acquires on the
next group rather than waiting for the broadcast to end. That is the one
thing a broadcast needs that a burst does not: a burst has exactly one
listener who was already there.

Framing is two bytes per frame:

    byte 0  bit 7  SYNC   opens a stream group; the descriptor (one
                          byte of payload type) follows the length
            bit 6  EOS    last frame of the broadcast
            bits 5-0      sequence, mod 64 -- for loss statistics only,
                          never for retransmission
    byte 1         valid payload bytes in THIS frame

The per-frame length costs ~4% at 26-byte frames and buys a property
that matters when nothing is retransmitted: every frame is
self-delimiting. Carrying the length only on the EOS frame would be
cheaper and would mis-size the payload whenever that one frame is the
one that gets lost.

Deliberately NOT copied from RTP, whose 12-byte header would be 91 ms of
air time at the top rung: no timestamp (this link's delay is
deterministic, so a frame's position in the stream is its timestamp), no
SSRC (point-to-multipoint among a handful of stations, and the
descriptor names the source), no per-packet payload type (it rides the
SYNC frame instead). What is kept is what a receiver cannot work out for
itself: sequence, stream boundaries, and payload type.

Rate reality: speech needs the top of the ladder. Codec2 at 700 bit/s
fits rung 11-12 (941-1059 bit/s user rate, +2.6/+4.7 dB); Codec2 450
fits rung 8-9. Below rung 8 this carries telemetry only, all the way
down to EXTREME's 7.8 bit/s.
"""

import typing
from dataclasses import dataclass

import numpy as np
from scipy.signal import hilbert

from .ldpc import LDPCCodec
from .modes import LinkMode, make_modem
from .packets import Data, ModType, CCSpeed, PacketType
from .transceiver import (Transceiver, DemodError, CODECS, MAPPERS,
                          HEADER_CODEC, HEADER_MAPPER, STREAM_RESYNC_EVERY)
from .packets import Header
from .fixed.tx import FixedTransmitter
from .fixed.rx import FixedReceiver

# Minimum samples of pre-roll a decoder needs BEFORE a preamble. Measured
# on the integer chain: with 300 samples it mis-times by 33 -- one more
# than the 32-sample cyclic prefix, so the frame is lost to ISI -- while
# 500 or more lands within a sample. The walk pads the slice when the
# recording itself does not provide this much, which is exactly the case
# for a group sitting near the start of a capture.
MIN_PREROLL = 1024

SYNC = 0x80
EOS = 0x40
SEQ_MASK = 0x3F
SEQ_MOD = 64

# payload types carried in the SYNC descriptor
PT_TELEMETRY = 0
PT_CODEC2_700 = 1
PT_CODEC2_450 = 2
PT_OPAQUE = 15


@dataclass
class BroadcastStats:
    """What a receiver can say about a broadcast it heard. This is the
    whole feedback path in non-ARQ mode -- there are no acknowledgments,
    so the sender learns only what a report like this tells it.

    One honest limit: `delivery` is measured over the ACQUIRED SPAN, not
    over what the sender sent. Losses show up as sequence gaps between
    decoded frames, so groups missed before the first acquisition or
    after the last are invisible here -- a receiver that caught only the
    tail of a broadcast reports 100%. The sender knows how much it sent
    and can compare; a bounded broadcast could also carry a total in the
    descriptor. For live speech there is no total to carry."""
    groups: int = 0        # stream groups acquired
    frames_ok: int = 0     # frames decoded
    frames_lost: int = 0   # inferred from sequence gaps
    bytes_out: int = 0
    snr_sum: float = 0.0
    ptype: int = -1
    saw_eos: bool = False

    @property
    def snr_db(self) -> float:
        return self.snr_sum / self.frames_ok if self.frames_ok else -99.0

    @property
    def expected(self) -> int:
        return self.frames_ok + self.frames_lost

    @property
    def delivery(self) -> float:
        return self.frames_ok / self.expected if self.expected else 0.0

    def report(self) -> str:
        return (f"{self.frames_ok}/{self.expected} frames in span "
                f"({100.0 * self.delivery:.0f}%), {self.bytes_out} B, "
                f"SNR {self.snr_db:+.1f} dB, {self.groups} groups"
                f"{'' if self.saw_eos else ', no EOS'}")


class _FloatBackend:
    """The float chain (Transceiver.build_stream / demod_stream)."""

    kind = "float"

    def __init__(self, mode, llr_recal="auto"):
        self.trx = Transceiver(make_modem(mode))
        self.trx.llr_recal = llr_recal
        self.modem = self.trx.modem

    def coerce(self, audio):
        return np.asarray(audio, dtype=np.float64)

    def build_group(self, frames, mod, spd, fec, resync):
        return self.trx.build_stream(frames, mod=mod, spd=spd,
                                     resync_every=resync, fec=fec,
                                     typ=PacketType.BCAST)

    def detect(self, seg):
        return self.modem.detect_preamble(
            hilbert(np.asarray(seg, dtype=np.float64)).astype(np.complex64))

    def decode_group(self, audio, n_blocks, resync):
        packets, st = self.trx.demod_stream(audio, n_blocks=n_blocks,
                                            resync_every=resync)
        return packets, [b.snr_db for b in st.blocks], st.header, st.start_sample


class _FixedBackend:
    """The integer chain (FixedTransmitter / FixedReceiver) -- the RTL
    reference model. Same walk, same framing; only the arithmetic under
    it differs."""

    kind = "fixed"

    def __init__(self, mode, llr_recal="auto"):
        self.tx = FixedTransmitter(mode)
        self.rx = FixedReceiver(mode, calibrate=llr_recal is not None)
        self.modem = self.rx._m

    def coerce(self, audio):
        return np.asarray(audio, dtype=np.int64)

    def build_group(self, frames, mod, spd, fec, resync):
        return self.tx.build_stream(frames, mod=mod, spd=spd,
                                    resync_every=resync, fec=fec,
                                    typ=PacketType.BCAST)

    def detect(self, seg):
        # The integer detector answers in ANALYTIC index space, which the
        # Hilbert FIR delays by its 31-sample group delay relative to the
        # input samples. Inside FixedReceiver.receive() that cancels --
        # detection and demodulation both index the same analytic arrays
        # -- but a walk over a recording mixes the two spaces, so the
        # offset has to come back out here or every group start lands 31
        # samples late and the error compounds through the advance.
        i_arr, q_arr = self.rx.hilbert.analytic(
            np.asarray(seg, dtype=np.int64))
        det = self.rx._detect(i_arr, q_arr)
        if det is None:
            return None
        return det[0] - self.rx.hilbert.delay, det[1]

    def decode_group(self, audio, n_blocks, resync):
        packets, header, info = self.rx.receive_stream(
            np.asarray(audio, dtype=np.int64), n_blocks=n_blocks,
            resync_every=resync)
        # NOTE the SNR figures on this path inherit a pre-existing limit
        # of the integer estimator, whose -7.2 dB constant was measured
        # for BPSK 1/3 near its sensitivity edge: outside that regime it
        # reads low (QPSK 1/2 at a +8 dB channel reports about -7 dB,
        # matching what FixedReceiver.receive() reports for the same
        # frame, so this is the estimator and not the stream path). The
        # delivery counts -- which are what a reception report is
        # actually for -- are unaffected.
        snrs = [s if s is not None else -99.0 for s in info["snrs"]]
        # same analytic-vs-sample correction as detect(): the walk advances
        # in sample space
        return packets, snrs, header, info["start"] - self.rx.hilbert.delay


def _backend(kind, mode, llr_recal):
    if kind == "fixed":
        return _FixedBackend(mode, llr_recal)
    return _FloatBackend(mode, llr_recal)


class BroadcastTx:
    """Cuts a byte stream into fixed-size frames and emits one audio
    buffer per stream group.

    group: frames behind one preamble. Bigger amortizes the preamble
    harder but delays how long a late listener waits to join, and
    exposes more of the stream to a single fade -- the same trade the
    burst window makes, without an acknowledgment to inform it.
    """

    def __init__(self, mode: LinkMode = LinkMode.NORMAL,
                 mod: ModType = ModType.QAM16, spd: CCSpeed = CCSpeed.R34,
                 group: int = 8, frame_bytes: int = 26,
                 fec: str = "cc", chain: str = "float"):
        assert 2 <= frame_bytes <= 27, "Data payload caps at 27 bytes"
        self.be = _backend(chain, mode, None)
        self.trx = getattr(self.be, "trx", None) or self.be.tx
        self.mode, self.mod, self.spd, self.fec = mode, mod, spd, fec
        self.group = group
        self.frame_bytes = frame_bytes

    def _cap(self, sync: bool) -> int:
        return self.frame_bytes - 2 - (1 if sync else 0)

    def _pack(self, flags: int, seq: int, body: bytes,
              ptype: int = None) -> Data:
        head = bytes([flags | (seq % SEQ_MOD), len(body)])
        if ptype is not None:
            head += bytes([ptype])
        pad = self.frame_bytes - len(head) - len(body)
        assert pad >= 0, "frame overflow"
        return Data(reserved=0, payload=head + body + bytes(pad))

    def _plan(self, data_len: int) -> typing.List[typing.List[int]]:
        """Group -> list of payload byte counts. A SYNC frame spends one
        byte on the descriptor, so it carries one less than the rest;
        the FRAME is still the same size (padded), which is what the
        streaming transport requires."""
        groups, off, first_of_group = [], 0, True
        cur = []
        while off < data_len or not groups and not cur:
            cap = self._cap(first_of_group)
            take = min(cap, data_len - off)
            cur.append(take)
            off += take
            first_of_group = False
            if len(cur) == self.group or off >= data_len:
                groups.append(cur)
                cur, first_of_group = [], True
            if off >= data_len:
                break
        if cur:
            groups.append(cur)
        return groups

    def build(self, data: bytes, ptype: int = PT_OPAQUE
              ) -> typing.List[np.ndarray]:
        """Returns one audio buffer per stream group. A real transmitter
        keys once and concatenates these; keeping them separate lets a
        caller insert listening gaps between groups."""
        plan = self._plan(len(data))
        out, seq, off = [], 0, 0
        for gi, sizes in enumerate(plan):
            frames = []
            for fi, take in enumerate(sizes):
                body = data[off:off + take]
                off += take
                flags = SYNC if fi == 0 else 0
                if gi == len(plan) - 1 and fi == len(sizes) - 1:
                    flags |= EOS
                frames.append(self._pack(flags, seq, body,
                                         ptype if fi == 0 else None))
                seq += 1
            out.append(self.be.build_group(frames, self.mod, self.spd,
                                           self.fec, STREAM_RESYNC_EVERY))
        return out

    def air_time(self, data_len: int) -> float:
        """Seconds on air for a payload of this size (planning helper)."""
        pkt_bits = len(Data(reserved=0,
                            payload=bytes(self.frame_bytes)).encode())
        lay = Transceiver(make_modem(self.mode)).stream_layout
        return sum(lay(pkt_bits, self.mod, self.spd, len(sizes),
                       fec=self.fec)["seconds"]
                   for sizes in self._plan(data_len))


class BroadcastRx:
    """Walks a recording, decodes every stream group it can find, and
    reassembles the payload. Nothing is ever retransmitted: a lost frame
    leaves a hole, and the hole is counted.

    LLR recalibration is on by default here, as it is in the link layer.
    It matters more in broadcast than in ARQ, because a frame the
    decoder cannot recover is simply gone -- measured on telemetry at
    QPSK 1/2, payload recovered went from 30.6% to 69.8% at -4.5 dB and
    from 3.4% to 12.0% at -6 dB. Above -3 dB, where delivery is already
    near perfect, it changes nothing. The map is gated to MU<=2, so
    16-QAM speech is unaffected -- it was never trained for that
    constellation."""

    def __init__(self, mode: LinkMode = LinkMode.NORMAL,
                 group: int = 8, llr_recal="auto", chain: str = "float"):
        self.be = _backend(chain, mode, llr_recal)
        self.group = group
        self._extent = 0  # preamble start -> group end, learned once

    def _next_preamble(self, audio: np.ndarray, pos: int,
                       win: int = 0) -> int:
        """Absolute sample where the next group's preamble starts, or -1.

        Detection runs on a window too short to hold two preambles
        (preamble + header). This matters: the composite detector takes a
        global argmax, so given a slice containing several groups it
        picks the strongest preamble rather than the first, and every
        group before it is silently lost. Measured -- 300 samples of
        lead-in was enough to make it skip a whole group.
        """
        m = self.be.modem
        pre = sum(len(c) for c in m.gen_preamble())
        hdr = 6 * m.symbol_len
        # Preambles sit exactly one extent apart, so ANY window shorter
        # than an extent holds at most one -- which means the window can
        # be widened well past the minimum once the extent is known. That
        # matters for the integer detector, whose noise floor is a median
        # over the window: starved of blocks it misses preambles the
        # float detector finds, which showed up as the walk acquiring one
        # group and then stopping dead.
        if win <= 0:
            win = pre + hdr
        stride = hdr
        while pos + pre < len(audio):
            det = self.be.detect(audio[pos:pos + win])
            if det is not None:
                start = pos + det[0] - pre
                # a lock whose preamble begins at or before the window
                # start is one we have already walked past: accepting it
                # would return the same position forever
                if start >= pos:
                    return start
            pos += stride
        return -1

    def receive(self, audio: np.ndarray
                ) -> typing.Tuple[bytes, BroadcastStats]:
        st = BroadcastStats()
        out = bytearray()
        m = self.be.modem
        audio = self.be.coerce(audio)
        pre = sum(len(c) for c in m.gen_preamble())
        pos, last_seq = 0, None
        restarted = False

        while pos + pre < len(audio):
            win = max(0, self._extent - m.symbol_len)
            start = self._next_preamble(audio, pos, win)
            if start < 0:
                break
            # Bound the decode slice to one group once the geometry is
            # known. demod_stream re-detects over whatever slice it is
            # handed, so an unbounded one can commit to a later preamble
            # and skip this group (measured: 1 seed in 12 even at
            # +11 dB). Searching forward for the next preamble to find
            # the bound does NOT work -- from inside a group the
            # detector false-locks on data -- but every group has the
            # same extent, so one successful decode supplies it.
            # Hand the decoder a slice that starts a little BEFORE the
            # located preamble. decode_group re-detects internally, and
            # _next_preamble's answer carries a few samples of jitter --
            # measured at 34 on the integer chain -- so a slice starting
            # exactly on the reported position can begin mid-preamble,
            # where the internal detector cannot lock and the whole group
            # is lost. The lead-in is safe because the window is still
            # bounded to about one extent.
            lead = max(m.symbol_len, MIN_PREROLL)
            ds = start - lead
            pad = 0
            if ds < 0:
                pad, ds = -ds, 0
            end = len(audio) if not self._extent \
                else min(len(audio), ds + self._extent + lead)
            seg = audio[ds:end]
            if pad:
                seg = np.concatenate([np.zeros(pad, dtype=seg.dtype), seg])
            try:
                packets, snrs, hdr, start_sample = self.be.decode_group(
                    seg, self.group, STREAM_RESYNC_EVERY)
            except DemodError:
                pos = max(start + m.symbol_len, pos + m.symbol_len)
                continue
            if not any(p is not None for p in packets):
                pos = max(start + m.symbol_len, pos + m.symbol_len)
                continue

            st.groups += 1
            for snr, pkt in zip(snrs, packets):
                if pkt is None:
                    continue
                body = bytes(pkt.payload)
                if len(body) < 2:
                    continue
                flags, seq = body[0] & ~SEQ_MASK, body[0] & SEQ_MASK
                dlen = body[1]
                # sequence gaps are the only loss signal there is; a
                # whole missed group shows up as a gap of `group`
                if last_seq is not None:
                    gap = (seq - last_seq - 1) % SEQ_MOD
                    if 0 < gap < SEQ_MOD // 2:
                        st.frames_lost += gap
                last_seq = seq
                st.frames_ok += 1
                st.snr_sum += snr
                head = 2
                if flags & SYNC:
                    if len(body) > 2:
                        st.ptype = body[2]
                    head = 3
                if flags & EOS:
                    st.saw_eos = True
                out += body[head:head + dlen]

            glen = self._group_len(hdr)
            if not self._extent:
                # The FIRST decode has to run unbounded -- the geometry
                # is not known yet -- so it can skip ahead exactly as
                # described above, losing every group before the one it
                # locked. Measured on a late joiner: groups 2 and 3 of a
                # 7-group broadcast silently gone. So once one group has
                # supplied the extent, start the walk over with the
                # bound in place.
                # preamble + header + blocks. Deriving this from
                # start_sample instead would inflate it whenever the
                # first, unbounded decode skipped ahead -- start_sample
                # is only one preamble's worth when the slice begins on
                # the preamble, and an inflated extent makes the bound
                # useless.
                self._extent = pre + glen
                if not restarted:
                    restarted = True
                    st = BroadcastStats()
                    out = bytearray()
                    pos, last_seq = 0, None
                    continue
            # Resume a guard band BEFORE the next group's preamble. The
            # detectors carry a sample or two of jitter, so an advance
            # computed from this group's start can land just PAST the next
            # preamble, and _next_preamble refuses a lock behind `pos` (it
            # has to, or it would return the same position forever) --
            # measured on the integer chain as whole groups lost to a
            # one-sample overshoot. Lead-in is safe here precisely because
            # the search window is bounded to less than one extent and so
            # cannot contain the group after next; that was NOT true of
            # the earlier unbounded search, where lead-in made the
            # detector skip ahead instead.
            # the pad shifted every position inside the slice right by
            # `pad`, so take it back out before advancing
            nxt = ds + (start_sample - pad) + glen - m.symbol_len
            pos = max(nxt, pos + m.symbol_len)  # never stand still

        st.bytes_out = len(out)
        return bytes(out), st

    def _group_len(self, hdr) -> int:
        """Samples from the group's header start to the end of its last
        block -- laid out exactly as demod_stream walks it."""
        m = self.be.modem
        codec = LDPCCodec if hdr.ver == 2 else CODECS[hdr.spd]
        mapper = MAPPERS[hdr.mod]
        n_hdr = -(-HEADER_CODEC.calc_cc_elements(Header.PACKET_SIZE)
                  // (m.data_carriers_len * HEADER_MAPPER.MU))
        n_data = -(-codec.calc_cc_elements(hdr.len)
                   // (m.data_carriers_len * mapper.MU))
        n_resync = ((self.group - 1) // STREAM_RESYNC_EVERY
                    if STREAM_RESYNC_EVERY else 0)
        return (n_hdr + self.group * n_data + n_resync) * m.symbol_len
