"""Fixed-point (integer-only) reference model of the modem for RTL work.

Every DSP operation runs in integer arithmetic with explicit bit widths and
rounding, mirroring what an FPGA/ASIC datapath would do:

  fxp.py      Q-format primitives: saturation, rounding shifts, complex mult
  fft.py      radix-2 DIT FFT, Q15 twiddle ROM, per-stage scaling (1/N),
              block-floating-point wrapper
  dsp.py      FIR Hilbert transformer (analytic signal), NCO with sine LUT
              (CFO as a 32-bit phase-increment word), CORDIC atan2/magnitude
  viterbi.py  integer soft-decision Viterbi (int8 LLRs, int32 path metrics)
  tx.py       FixedTransmitter -> int16 audio samples
  rx.py       FixedReceiver: detection, sync, demod, decode on int16 input

Bit-exact float paths (CRC, packets, scrambler, interleaver, puncturing) are
reused from the parent package -- they are already integer arithmetic.

The receiver covers the full frame family: convolutional and LDPC (header
ver=2, integer min-sum, alpha=0.75 as x-(x>>2)) FEC, BPSK/QPSK/16-QAM
(integer max-log demap, one divider per 16-QAM symbol), HARQ chase combining
(DemodError carries the integer LLRs; receive(prev_data_llrs=...) combines,
CRC-gated), and an optional calibrated-LLR mode (calibrate=True): an integer
header-based temperature fit (one divider) plus a 32-entry reliability ROM
measured on the fixed chain itself. Measured: the default 6-bit
peak-normalized quantization already acts as a compressive recalibration --
the fixed RX matches or beats the recalibrated float chain at -9/-10 dB --
so calibrate mode's value is the STABLE CROSS-FRAME SCALE that makes HARQ
combining legitimate, not extra PER. The repacked LC word is payload-level
and passes through untouched.
"""

from .fxp import Q15, sat, rshift_round, cmul_q15
from .fft import fft_fixed, ifft_fixed, fft_bfp
from .dsp import HilbertFIR, NCO, cordic_atan2, cordic_mag, hz_to_phase_word
from .viterbi import viterbi_decode_int
from .tx import FixedTransmitter
from .rx import FixedReceiver, FixedRxStats
from .phy import FixedPHY
