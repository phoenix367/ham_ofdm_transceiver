"""Fixed-point transmitter: packet bits -> int16 audio samples.

Datapath: Q15 constellation -> fixed-point IFFT (Q15 twiddles, 1/N per-stage
scaling) -> CP/tiling (copies) -> preamble ROMs (precomputed int16 tables,
exactly how RTL stores them) -> 6 dB clip (threshold = 2x RMS via integer
sqrt) -> 33-tap Q15 low-pass FIR -> static x8 output gain.

The bit pipeline (CRC, convolutional code, puncturing, LDPC accumulator
encoding, interleaver, scrambler) is reused from the float package -- it is
already pure integer.
"""

import numpy as np
from scipy.signal import firwin

from ..coding import CCLTEBPSK_13
from ..ldpc import LDPCCodec
from ..interleaver import interleave
from ..mapping import BPSKMapper, QPSKMapper, QAM16Mapper
from ..modes import LinkMode, make_modem
from ..packets import Header, Beacon, PacketType, ModType, CCSpeed
from ..scrambler import scramble
from ..transceiver import CODECS, MAPPERS, HEADER_CODEC, HEADER_MAPPER
from .fxp import Q15, sat, rshift_round, isqrt_i64
from .fft import ifft_fixed

Q15_MAX = (1 << Q15) - 1
# Q15 constellation points
BPSK_POINTS = {0: (-Q15_MAX, 0), 1: (Q15_MAX, 0)}
QPSK_AMP = int(round(Q15_MAX / np.sqrt(2.0)))
# 16-QAM: Gray-coded per axis, unit average power => levels {+-1, +-3}/sqrt(10)
QAM16_AMP = int(round(Q15_MAX / np.sqrt(10.0)))
QAM16_GRAY_LEVEL = {(0, 0): -3, (0, 1): -1, (1, 1): 1, (1, 0): 3}

OUTPUT_GAIN_SHIFT = 3  # static x8, sized for ~0.85 FS peaks after clipping
LPF_TAPS = 33


class FixedTransmitter:
    def __init__(self, mode: LinkMode = LinkMode.NORMAL):
        self.mode = mode
        self._m = make_modem(mode)  # float modem supplies constants only

        # pilot ROM: Q15 quantized ZC pilot values
        pilots = self._m._get_pilot_values()
        self.pilot_rom = (np.round(pilots.real * Q15_MAX).astype(np.int64),
                          np.round(pilots.imag * Q15_MAX).astype(np.int64))

        # preamble ROM: the float preamble, real part, scaled to the same
        # Q15 domain as the IFFT output of Q15 constellation bins
        pre = np.concatenate([blk.real for blk in self._m.gen_preamble()])
        self.preamble_rom = np.round(pre * Q15_MAX).astype(np.int64)
        # the ZC symbol alone (CP + sym_tile tiles) closes the preamble; a
        # streamed burst re-emits just this block as a resync marker
        self.zc_rom = self.preamble_rom[-self._m.symbol_len:]

        taps = firwin(LPF_TAPS, 3000.0, fs=self._m.sample_rate)
        self.lpf_q15 = np.round(taps * Q15_MAX).astype(np.int64)

    # --- OFDM symbol -------------------------------------------------------

    def _map_bits(self, bits, mapper):
        if mapper is BPSKMapper:
            pts = [BPSK_POINTS[int(b)] for b in bits]
        elif mapper is QPSKMapper:
            pairs = np.asarray(bits).reshape(-1, 2)
            pts = [(QPSK_AMP * (2 * int(b0) - 1), QPSK_AMP * (2 * int(b1) - 1))
                   for b0, b1 in pairs]
        elif mapper is QAM16Mapper:
            quads = np.asarray(bits).reshape(-1, 4)
            pts = [(QAM16_AMP * QAM16_GRAY_LEVEL[(int(b0), int(b1))],
                    QAM16_AMP * QAM16_GRAY_LEVEL[(int(b2), int(b3))])
                   for b0, b1, b2, b3 in quads]
        else:
            raise ValueError(f"unsupported mapper {mapper!r}")
        re = np.array([p[0] for p in pts], dtype=np.int64)
        im = np.array([p[1] for p in pts], dtype=np.int64)
        return re, im

    def _modulate_symbol(self, bits, mapper):
        m = self._m
        n = m.fft_bins

        sub_re = np.zeros(len(m.channel_indices), dtype=np.int64)
        sub_im = np.zeros(len(m.channel_indices), dtype=np.int64)
        d_re, d_im = self._map_bits(bits, mapper)
        sub_re[m._data_local_indices] = d_re
        sub_im[m._data_local_indices] = d_im
        sub_re[m._pilot_local_indices] = self.pilot_rom[0]
        sub_im[m._pilot_local_indices] = self.pilot_rom[1]

        sym_re = np.zeros(n, dtype=np.int64)
        sym_im = np.zeros(n, dtype=np.int64)
        sym_re[m.channel_indices] = sub_re
        sym_im[m.channel_indices] = sub_im
        sym_re[n - m.channel_indices] = sub_re
        sym_im[n - m.channel_indices] = -sub_im

        t_re, _ = ifft_fixed(sym_re, sym_im)  # Hermitian -> real output

        tiled = np.tile(t_re, m.sym_tile)
        return np.concatenate([tiled[-m.cyclic_prefix:], tiled])

    # --- frame -------------------------------------------------------------

    def _encode_block(self, bits, codec, mapper):
        coded = codec.encode(bits.astype(np.uint8))
        capacity = self._m.data_carriers_len * mapper.MU
        n_syms = (len(coded) + capacity - 1) // capacity
        padded = np.pad(coded, (0, n_syms * capacity - len(coded)))
        return scramble(interleave(padded, self._m.data_carriers_len)).reshape(n_syms, capacity)

    def build_frame(self, packet, mod: ModType = ModType.BPSK,
                    spd: CCSpeed = CCSpeed.R13, fec: str = "cc") -> np.ndarray:
        """Returns int16 audio samples.

        fec="ldpc" codes the DATA block with the rate-1/3 IRA LDPC
        (signalled via header ver=2, mirroring the float transmitter); the
        header itself stays convolutional. The LDPC accumulator encoding is
        already pure integer, so the bit pipeline needs no fixed-point twin.
        """
        pkt_bits = packet.encode()
        assert len(pkt_bits) <= 255, "packet exceeds the 8-bit header len field"
        typ = PacketType.BEACON if isinstance(packet, Beacon) else PacketType.DATA
        header = Header(ver=2 if fec == "ldpc" else 1, typ=typ, mod=mod,
                        spd=spd, len=len(pkt_bits))
        data_codec = LDPCCodec if fec == "ldpc" else CODECS[spd]

        chunks = [self.preamble_rom]
        for row in self._encode_block(header.encode(), HEADER_CODEC, HEADER_MAPPER):
            chunks.append(self._modulate_symbol(row, HEADER_MAPPER))
        for row in self._encode_block(pkt_bits, data_codec, MAPPERS[mod]):
            chunks.append(self._modulate_symbol(row, MAPPERS[mod]))

        return self._finish(np.concatenate(chunks))

    def _finish(self, signal: np.ndarray) -> np.ndarray:
        """Clip, filter and scale -- shared by frames and bursts, and applied
        over the WHOLE waveform (the clip threshold is a signal-wide RMS)."""
        # clip at RMS + ~6 dB (threshold = 2x RMS, integer sqrt of mean square)
        mean_sq = int(np.sum(signal * signal) // len(signal))
        thr = 2 * isqrt_i64(mean_sq)
        clipped = np.clip(signal, -thr, thr)

        # Q15 low-pass FIR (causal; the 16-sample group delay is harmless)
        filtered = rshift_round(np.convolve(clipped, self.lpf_q15, mode="same"), Q15)

        return sat(filtered << OUTPUT_GAIN_SHIFT, 16).astype(np.int16)

    def build_stream(self, packets, mod: ModType = ModType.BPSK,
                     spd: CCSpeed = CCSpeed.R13, resync_every: int = 4,
                     fec: str = "cc", typ=None) -> np.ndarray:
        """Streamed burst: one preamble and one header for N equal blocks.

        Fixed-point twin of Transceiver.build_stream (see docs/phy.md). The
        ZC symbol re-emitted every `resync_every` blocks is the preamble's
        own ZC block straight out of ROM -- no recomputation, which is the
        whole point on an MCU.
        """
        assert len(packets) >= 1, "a stream needs at least one block"
        bits = [p.encode() for p in packets]
        pkt_bits = len(bits[0])
        assert all(len(b) == pkt_bits for b in bits), \
            "stream blocks must all encode to the same length"
        assert pkt_bits <= 255, "packet exceeds the 8-bit header len field"
        types = {PacketType.BEACON if isinstance(p, Beacon) else PacketType.DATA
                 for p in packets}
        assert len(types) == 1, "stream blocks must all be the same packet type"
        # a caller may override the advertised type -- broadcast frames are
        # Data-shaped but must not reach the ARQ reassembler
        if typ is not None:
            types = {typ}

        header = Header(ver=2 if fec == "ldpc" else 1, typ=types.pop(),
                        mod=mod, spd=spd, len=pkt_bits)
        data_codec = LDPCCodec if fec == "ldpc" else CODECS[spd]

        chunks = [self.preamble_rom]
        for row in self._encode_block(header.encode(), HEADER_CODEC, HEADER_MAPPER):
            chunks.append(self._modulate_symbol(row, HEADER_MAPPER))
        for k, block_bits in enumerate(bits):
            if resync_every and k and k % resync_every == 0:
                chunks.append(self.zc_rom)
            for row in self._encode_block(block_bits, data_codec, MAPPERS[mod]):
                chunks.append(self._modulate_symbol(row, MAPPERS[mod]))

        return self._finish(np.concatenate(chunks))
