"""Full PHY frame transceiver.

TX chain: packet -> FEC (conv. code, optional puncturing) -> interleave ->
scramble -> PSK map -> OFDM symbols (pilots, CP, 4x tiling) -> preamble
(Newman tones + Zadoff-Chu) -> clip-and-filter.

RX chain: analytic signal -> two-stage preamble detection + CFO correction ->
per-symbol soft demodulation (channel estimation, MMSE equalization, LLRs) ->
descramble -> deinterleave -> soft Viterbi -> CRC check -> packet.
"""

import typing
from dataclasses import dataclass, field

import numpy as np
from scipy.signal import hilbert

from .coding import CCLTEBPSK_13, CCLTEBPSK_12, CCLTEBPSK_23, CCLTEBPSK_34
from .ldpc import LDPCCodec
from .interleaver import interleave, deinterleave
from .mapping import BPSKMapper, QPSKMapper, QAM16Mapper
from .ofdm import FullOFDMModem, freq_shift
from .packets import (
    Header, Beacon, Data, PacketType, ModType, CCSpeed, PACKET_CLASSES,
)
from .papr import clip_and_filter
from .scrambler import scramble, descramble

CODECS = {
    CCSpeed.R13: CCLTEBPSK_13,
    CCSpeed.R12: CCLTEBPSK_12,
    CCSpeed.R23: CCLTEBPSK_23,
    CCSpeed.R34: CCLTEBPSK_34,
}

MAPPERS = {
    ModType.BPSK: BPSKMapper,
    ModType.QPSK: QPSKMapper,
    ModType.QAM16: QAM16Mapper,
}

HEADER_CODEC = CCLTEBPSK_13
HEADER_MAPPER = BPSKMapper

# Measured monotone LLR reliability map (trained: NORMAL BPSK 1/3 at -7..-9 dB
# over the article channel, 44k samples; see experiments/llr_calibration.py).
# The raw front-end LLRs are shape-miscalibrated: weak LLRs are ~4x more
# reliable than they claim (per-carrier EsN0 weighting over-spreads
# confidence) and the +-20 clip overstates the top. Mapping each |L| to the
# log-odds of its EMPIRICAL error rate fixes both, and capping at the
# data-justified maximum also defuses confidently-wrong bits from BSC flips
# and fades. Worth ~1.5-2 dB at the BPSK sensitivity edge; NOT valid for
# 16-QAM (different LLR shape), hence the MU <= 2 gate at the use site.
_RECAL_MIDS = np.array([0.1, 0.7, 1.2, 1.7, 2.5, 3.7, 5.3, 7.5, 10.6, 15.8])
_RECAL_OUTS = np.array([1.8, 3.0, 3.6, 4.1, 4.6, 5.9, 6.7, 7.4, 7.4, 7.4])


def default_llr_recal(llrs: np.ndarray) -> np.ndarray:
    return np.sign(llrs) * np.interp(np.abs(llrs), _RECAL_MIDS, _RECAL_OUTS)


class DemodError(Exception):
    """PHY demodulation failure. For data-stage failures the exception
    carries the decoded header and the raw data-block LLRs so the link layer
    can chase-combine them with a retransmission."""

    def __init__(self, msg, header=None, data_llrs=None):
        super().__init__(msg)
        self.header = header
        self.data_llrs = data_llrs


@dataclass
class RxStats:
    header: Header = None
    es_n0_carriers_db: np.ndarray = None
    es_n0_db: float = 0.0
    snr_db: float = 0.0
    ber: float = 0.0
    start_sample: int = 0
    cfo_hz: float = 0.0
    harq_combined: bool = False  # decode needed chase combining
    llr_alpha: float = 1.0       # header-fitted LLR temperature


class Transceiver:
    def __init__(self, modem: FullOFDMModem = None,
                 papr_cutoff_db: float = 6.0, papr_filter_hz: float = 3000.0):
        self._modem = modem if modem is not None else FullOFDMModem()
        self._papr_cutoff_db = papr_cutoff_db
        self._papr_filter_hz = papr_filter_hz
        # LLR recalibration: None = raw (article-faithful), "auto" = apply
        # the measured default map to BPSK/QPSK data blocks, or a callable
        self.llr_recal = None

    @property
    def modem(self):
        return self._modem

    # --- shared helpers ----------------------------------------------------

    def _block_capacity(self, mapper) -> int:
        return self._modem.data_carriers_len * mapper.MU

    def _num_symbols(self, coded_len: int, mapper) -> int:
        capacity = self._block_capacity(mapper)
        return (coded_len + capacity - 1) // capacity

    def _encode_block(self, bits: np.ndarray, codec, mapper) -> np.ndarray:
        """FEC-encode, pad to a whole number of OFDM symbols, interleave and
        scramble. Returns (n_syms, capacity) bit matrix."""
        coded = codec.encode(bits.astype(np.uint8))
        capacity = self._block_capacity(mapper)
        n_syms = (len(coded) + capacity - 1) // capacity

        padded = np.pad(coded, (0, n_syms * capacity - len(coded)), mode="constant")
        interleaved = interleave(padded, self._modem.data_carriers_len)
        scrambled = scramble(interleaved)

        return scrambled.reshape(n_syms, capacity)

    def _decode_block(self, llrs: np.ndarray, codec, bits_count: int) -> np.ndarray:
        """Inverse of _encode_block: descramble, deinterleave, crop the pad,
        Viterbi-decode."""
        coded_len = codec.calc_cc_elements(bits_count)

        descrambled = descramble(llrs)
        deinterleaved = deinterleave(descrambled, self._modem.data_carriers_len)
        cropped = deinterleaved[:coded_len]

        if codec is LDPCCodec:
            return codec.decode(cropped, bits_count, spa=LDPCCodec.USE_SPA)
        return codec.decode(cropped, bits_count)

    # --- transmitter -------------------------------------------------------

    def build_frame(self, packet: typing.Union[Beacon, Data],
                    mod: ModType = ModType.BPSK, spd: CCSpeed = CCSpeed.R13,
                    clip: bool = True, fec: str = "cc") -> np.ndarray:
        """Build the complete baseband frame (float64 audio samples).

        fec="ldpc" codes the DATA block with the rate-1/3 IRA LDPC instead of
        the convolutional code (signalled via header ver=2; the header itself
        stays convolutional -- 25 bits is below LDPC's useful block size).
        """
        m = self._modem

        pkt_bits = packet.encode()
        assert len(pkt_bits) <= 255, "packet exceeds the 8-bit header len field"
        typ = PacketType.BEACON if isinstance(packet, Beacon) else PacketType.DATA
        header = Header(ver=2 if fec == "ldpc" else 1, typ=typ, mod=mod,
                        spd=spd, len=len(pkt_bits))
        data_codec = LDPCCodec if fec == "ldpc" else CODECS[spd]

        chunks = list(m.gen_preamble())

        m.set_mapper(HEADER_MAPPER)
        for row in self._encode_block(header.encode(), HEADER_CODEC, HEADER_MAPPER):
            chunks.append(m.modulate_symbol_cp(row))

        m.set_mapper(MAPPERS[mod])
        for row in self._encode_block(pkt_bits, data_codec, MAPPERS[mod]):
            chunks.append(m.modulate_symbol_cp(row))

        signal_full = np.concatenate(chunks).real.astype(np.float64)

        if clip:
            signal_full = clip_and_filter(
                signal_full,
                papr_cutoff_db=self._papr_cutoff_db,
                cutoff_freq_hz=self._papr_filter_hz,
                fs_hz=m.sample_rate,
            )

        return signal_full

    def build_frame_int16(self, *args, **kwargs) -> np.ndarray:
        signal_caf = self.build_frame(*args, **kwargs)
        peak = np.max(np.abs(signal_caf))
        return (signal_caf / peak * 0.9 * 32767).astype(np.int16)

    # --- receiver ----------------------------------------------------------

    def _demod_symbols(self, signal: np.ndarray, pos: int, n_syms: int):
        m = self._modem
        sym_len = m.symbol_len

        if pos + n_syms * sym_len > len(signal):
            raise DemodError("signal too short")

        llrs = []
        es_n0_carriers = []
        es_n0_list = []
        for i in range(n_syms):
            sym = signal[pos + i * sym_len: pos + (i + 1) * sym_len]
            es_n0_pc, es_n0, llr = m.demodulate_symbol_cp_soft(sym)
            llrs.append(llr)
            es_n0_carriers.append(es_n0_pc)
            es_n0_list.append(es_n0)

        return np.concatenate(llrs), np.mean(es_n0_carriers, axis=0), float(np.mean(es_n0_list))

    def demod_frame(self, real_signal: np.ndarray, check_crc: bool = True,
                    prev_data_llrs: np.ndarray = None,
                    ) -> typing.Tuple[typing.Union[Beacon, Data], RxStats]:
        """Detect, synchronize and decode one frame from real audio samples.

        prev_data_llrs: raw data-block LLRs of a previously FAILED frame with
        the same header signature (chase-combining HARQ). If the fresh LLRs
        fail CRC, the sum of both attempts is tried -- worth ~3 dB on a
        retransmission. The CRC gates it, so a wrong guess costs nothing.
        """
        m = self._modem

        analytic = hilbert(np.asarray(real_signal, dtype=np.float64)).astype(np.complex64)

        det = m.detect_preamble(analytic)
        if det is None:
            raise DemodError("no preamble")

        start, cfo = det
        corrected = freq_shift(m.sample_rate, analytic, cfo)
        # tolerate a small positive timing slip on the final symbol
        corrected = np.concatenate([corrected, np.zeros(m.symbol_len, dtype=corrected.dtype)])

        # ---- header (always BPSK, rate 1/3)
        m.set_mapper(HEADER_MAPPER)
        n_hdr_syms = self._num_symbols(HEADER_CODEC.calc_cc_elements(Header.PACKET_SIZE), HEADER_MAPPER)

        try:
            hdr_llrs, _, _ = self._demod_symbols(corrected, start, n_hdr_syms)
            hdr_bits = self._decode_block(hdr_llrs, HEADER_CODEC, Header.PACKET_SIZE)
            header = Header.decode(hdr_bits, check_crc=check_crc)
        except DemodError:
            raise
        except Exception as exc:
            raise DemodError("head") from exc

        # front-end LLR calibration: the decoded header supplies 96 known
        # bits; Gaussian consistency (Var = 2*Mean for a true LLR) yields the
        # frame's temperature alpha = 2*E[L*x]/Var[L*x]. The decision-directed
        # EsN0 estimate is overconfident at low SNR, so alpha < 1 there.
        # Scaling is a no-op for Viterbi/min-sum but makes sum-product and
        # HARQ combining weights meaningful.
        ref_hdr = 2.0 * self._encode_block(hdr_bits, HEADER_CODEC,
                                           HEADER_MAPPER).ravel().astype(np.float64) - 1.0
        lx = hdr_llrs[:len(ref_hdr)] * ref_hdr
        v = float(np.var(lx))
        llr_alpha = float(np.clip(2.0 * np.mean(lx) / v, 0.05, 4.0)) if v > 1e-9 else 1.0

        pos = start + n_hdr_syms * m.symbol_len

        # ---- data block (modulation/rate from the header)
        codec = LDPCCodec if header.ver == 2 else CODECS[header.spd]
        mapper = MAPPERS[header.mod]
        m.set_mapper(mapper)

        coded_len = codec.calc_cc_elements(header.len)
        n_data_syms = self._num_symbols(coded_len, mapper)

        llrs, es_n0_pc, es_n0 = self._demod_symbols(corrected, pos, n_data_syms)
        llrs = llrs * llr_alpha  # calibrated scale (also keeps HARQ sums fair)
        if self.llr_recal == "auto":
            if mapper.MU <= 2:  # the map is trained on BPSK/QPSK statistics
                llrs = default_llr_recal(llrs)
        elif callable(self.llr_recal):
            llrs = self.llr_recal(llrs)

        def attempt(llr_stream):
            b = self._decode_block(llr_stream, codec, header.len)
            return PACKET_CLASSES[header.typ].decode(b, check_crc=check_crc), b

        packet = bits = None
        combined = False
        try:
            packet, bits = attempt(llrs)
        except Exception:
            pass
        if packet is None and prev_data_llrs is not None \
                and len(prev_data_llrs) == len(llrs):
            try:  # chase combining with the stored failed attempt
                packet, bits = attempt(llrs + prev_data_llrs)
                combined = True
            except Exception:
                pass
        if packet is None:
            raise DemodError("data", header=header, data_llrs=llrs)

        # channel BER: re-encode the decoded bits and compare with the hard
        # decisions taken from the raw LLR stream
        ref_stream = self._encode_block(bits, codec, mapper).ravel()
        rx_hard = (llrs > 0).astype(np.uint8)
        ber = float(np.mean(ref_stream != rx_hard))

        es_n0_db = 10 * np.log10(max(es_n0, 1e-12))
        stats = RxStats(
            header=header,
            es_n0_carriers_db=10 * np.log10(np.maximum(es_n0_pc, 1e-12)),
            es_n0_db=es_n0_db,
            snr_db=es_n0_db - 10 * np.log10(m.sym_tile),
            ber=ber,
            start_sample=start,
            cfo_hz=cfo,
            harq_combined=combined,
            llr_alpha=llr_alpha,
        )

        return packet, stats

    _AUTO_TRX_CACHE: typing.ClassVar[dict] = {}

    def demod_frame_auto(self, real_signal: np.ndarray, check_crc: bool = True,
                         prev_data_llrs: np.ndarray = None, llr_recal=None):
        """Try each link mode's modem in turn; the per-mode ZC preamble root
        means only the transmitted mode's matched filter locks. Returns
        (packet, stats, mode)."""
        from .modes import LinkMode, make_modem

        last_exc = None
        for mode in LinkMode:
            if mode not in self._AUTO_TRX_CACHE:
                self._AUTO_TRX_CACHE[mode] = Transceiver(make_modem(mode))
            trx = self._AUTO_TRX_CACHE[mode]
            trx.llr_recal = llr_recal
            try:
                packet, stats = trx.demod_frame(real_signal, check_crc=check_crc,
                                                prev_data_llrs=prev_data_llrs)
                return packet, stats, mode
            except DemodError as exc:
                if exc.data_llrs is not None or last_exc is None \
                        or last_exc.data_llrs is None:
                    last_exc = exc
        raise last_exc if last_exc is not None else DemodError("no preamble")

    # --- rates -------------------------------------------------------------

    def channel_bit_rate(self, mod: ModType) -> float:
        """Raw channel rate (before FEC), bits/second."""
        m = self._modem
        return self._block_capacity(MAPPERS[mod]) * m.sample_rate / m.symbol_len

    def data_bit_rate(self, mod: ModType, spd: CCSpeed) -> float:
        """Net user data rate (after FEC overhead), bits/second."""
        rates = {CCSpeed.R13: 1 / 3, CCSpeed.R12: 1 / 2, CCSpeed.R23: 2 / 3, CCSpeed.R34: 3 / 4}
        return self.channel_bit_rate(mod) * rates[spd]
