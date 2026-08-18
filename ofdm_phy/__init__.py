"""OFDM amateur-radio PHY prototype.

Reproduction of the PHY layer described in the Habr article
"Разработка цифрового радиолюбительского протокола на базе OFDM. PHY-уровень"
(https://habr.com/ru/articles/1070804/).

Audio-band OFDM (12 kHz sample rate, 128-bin FFT, 300-2400 Hz, 23 subcarriers,
7 Zadoff-Chu pilots, 4x symbol tiling), BPSK/QPSK mapping, rate-1/3 convolutional
FEC with puncturing to 1/2, 2/3, 3/4, soft-decision Viterbi decoding, block
interleaving, LFSR scrambling, Newman-tone + Zadoff-Chu two-stage preamble and
clip-and-filter PAPR reduction.
"""

from .crc import crc8_lte, crc16_ccitt
from .packets import (
    PacketType, ModType, CCSpeed, BeaconMode,
    Header, Beacon, Data, PacketCRCMissmatch,
    callsign_to_int, int_to_callsign, qth6_to_int, int_to_qth6,
)
from .mapping import PSKMapper, BPSKMapper, QPSKMapper, QAM16Mapper
from .coding import ConvCodec, ConvCodecPunctured, CCLTEBPSK, CCLTEBPSK_13, CCLTEBPSK_12, CCLTEBPSK_23, CCLTEBPSK_34
from .interleaver import interleave, deinterleave
from .scrambler import scramble, descramble, DEFAULT_SEED
from .papr import clip_and_filter
from .ofdm import OFDMModem, TiledOFDMModem, FullOFDMModem, STFOFDMModem, freq_shift, DEFAULT_SAMPLE_RATE
from .channel import simulate_channel
from .modes import LinkMode, ModeSpec, MODE_SPECS, make_modem, select_mode
from .transceiver import Transceiver, RxStats
from .link import LADDER, LinkControl, LinkController
from .station import LinkStation
from .ldpc import LDPCCodec
