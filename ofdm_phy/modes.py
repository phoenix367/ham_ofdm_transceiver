"""SNR-adaptive link modes.

Each mode scales the coherent-accumulation tile factor (and the preambles
with it), trading bitrate for sensitivity in ~6 dB steps:

  NORMAL   4x tiles   353 bit/s channel   sensitivity ~ -7 dB    (the article)
  ROBUST  16x tiles    88 bit/s channel   sensitivity ~ -11.5 dB
  EXTREME 64x tiles    22 bit/s channel   sensitivity ~ -18 dB

The receiver cannot learn the tile factor from the header (it needs the tile
factor to demodulate the header), so each mode uses a distinct Zadoff-Chu
preamble root: the preamble itself identifies the mode, and
`Transceiver.demod_frame_auto` simply tries the modes in turn -- a
wrong-root matched filter does not correlate.

Shannon context: at -20 dB the 2100 Hz band carries at most ~30 bit/s;
EXTREME runs at 22 bit/s (78% of capacity), so ~-19 dB is the practical
floor for this waveform and -20.7 dB the theoretical one.
"""

from dataclasses import dataclass
from enum import Enum

from .ofdm import FullOFDMModem, STFOFDMModem
from .packets import ModType, CCSpeed


class LinkMode(Enum):
    NORMAL = 0
    ROBUST = 1
    EXTREME = 2


@dataclass(frozen=True)
class ModeSpec:
    sym_tile: int
    newman_tile: int          # tone preamble tiles (first tone lasts 2x this)
    zc_count: int             # ZC symbols in the sync preamble
    zc_root: int              # distinct per mode -> preamble identifies mode
    newman_threshold: float   # tone-contrast detection threshold
    freq_range: float         # per-symbol residual-CFO search range, Hz
    detect_fft_len: int       # tone detection FFT block length
    zc_group: int             # ZC symbols correlated coherently per kernel
    zc_threshold: float       # ZC normalized-correlation base threshold


MODE_SPECS = {
    LinkMode.NORMAL: ModeSpec(sym_tile=4, newman_tile=10, zc_count=4, zc_root=17,
                              newman_threshold=1.6, freq_range=8.0,
                              detect_fft_len=128, zc_group=1, zc_threshold=0.2),
    LinkMode.ROBUST: ModeSpec(sym_tile=16, newman_tile=40, zc_count=16, zc_root=19,
                              newman_threshold=1.5, freq_range=8.0,
                              detect_fft_len=512, zc_group=2, zc_threshold=0.08),
    LinkMode.EXTREME: ModeSpec(sym_tile=64, newman_tile=160, zc_count=64, zc_root=21,
                               newman_threshold=1.30, freq_range=25.0,
                               detect_fft_len=512, zc_group=4, zc_threshold=0.05),
}


def make_modem(mode: LinkMode, stf: bool = False, **overrides) -> FullOFDMModem:
    spec = MODE_SPECS[mode]
    cls = STFOFDMModem if stf else FullOFDMModem
    kwargs = dict(
        sym_tile=spec.sym_tile,
        newman_preamble_tile=spec.newman_tile,
        zc_preamble_count=spec.zc_count,
        zc_preamble_root=spec.zc_root,
        newman_threshold=spec.newman_threshold,
        demod_freq_range=spec.freq_range,
        detect_fft_len=spec.detect_fft_len,
        zc_coherent_group=spec.zc_group,
        detection_threshold=spec.zc_threshold,
    )
    kwargs.update(overrides)
    return cls(**kwargs)


def select_mode(snr_db: float, margin_db: float = 1.0):
    """Pick (mode, modulation, coding rate) for an expected channel SNR.

    Thresholds are the measured PER<=10% sensitivities shifted by margin_db.
    """
    s = snr_db - margin_db
    if s >= 4.2:
        return LinkMode.NORMAL, ModType.QAM16, CCSpeed.R34
    if s >= 2.4:
        return LinkMode.NORMAL, ModType.QAM16, CCSpeed.R23
    if s >= 0.1:
        return LinkMode.NORMAL, ModType.QAM16, CCSpeed.R12
    if s >= -2:
        return LinkMode.NORMAL, ModType.QPSK, CCSpeed.R12
    if s >= -4:
        return LinkMode.NORMAL, ModType.QPSK, CCSpeed.R13
    if s >= -6:
        return LinkMode.NORMAL, ModType.BPSK, CCSpeed.R12
    if s >= -7:
        return LinkMode.NORMAL, ModType.BPSK, CCSpeed.R13
    if s >= -11.5:
        return LinkMode.ROBUST, ModType.BPSK, CCSpeed.R13
    return LinkMode.EXTREME, ModType.BPSK, CCSpeed.R13
