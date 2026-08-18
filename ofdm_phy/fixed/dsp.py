"""RTL-style DSP blocks: FIR Hilbert transformer, NCO, CORDIC.

CFO convention: frequency lives everywhere as a 32-bit phase-increment word
(one turn = 2^32), never as a float. hz_to_phase_word()/phase_word_to_hz()
convert only at the API boundary and in logs.
"""

import numpy as np
from scipy.signal import remez

from .fxp import Q15, rshift_round

PHASE_BITS = 32
PHASE_ONE = 1 << PHASE_BITS  # one full turn


def hz_to_phase_word(freq_hz: float, sample_rate: int) -> int:
    return int(round(freq_hz / sample_rate * PHASE_ONE))


def phase_word_to_hz(word: int, sample_rate: int) -> float:
    # interpret as signed
    if word >= PHASE_ONE // 2:
        word -= PHASE_ONE
    return word * sample_rate / PHASE_ONE


class HilbertFIR:
    """63-tap type-III FIR Hilbert transformer (Q15 taps). Produces the
    analytic signal: I = input delayed by the 31-sample group delay,
    Q = FIR output. int16 in, int16-range int64 out."""

    NUM_TAPS = 63

    def __init__(self):
        taps = remez(self.NUM_TAPS, [0.02, 0.48], [1.0], type="hilbert", fs=1.0)
        # scipy's convention yields -sin for a cos input; negate so the
        # analytic signal I + jQ carries positive frequencies
        self.taps_q15 = np.round(-taps * ((1 << Q15) - 1)).astype(np.int64)
        self.delay = (self.NUM_TAPS - 1) // 2

    def analytic(self, samples):
        """Returns (i_arr, q_arr) of the same length as the input (edges are
        transient, as in hardware)."""
        x = np.asarray(samples, dtype=np.int64)
        padded = np.concatenate([np.zeros(self.NUM_TAPS - 1, dtype=np.int64), x])
        # direct-form FIR: int16 x Q15 -> int32 products, int38 accumulator
        q = np.convolve(padded, self.taps_q15, mode="valid")
        q = rshift_round(q, Q15)

        i = np.concatenate([np.zeros(self.delay, dtype=np.int64), x])[:len(x)]
        q = q[len(q) - len(x):]
        # align: q[n] corresponds to input n - delay as well
        return i, q


class NCO:
    """Numerically controlled oscillator: 32-bit phase accumulator, 4096-entry
    Q15 sine/cosine ROM addressed by the top 12 phase bits."""

    LUT_BITS = 12

    _COS = None
    _SIN = None

    @classmethod
    def _rom(cls):
        if cls._COS is None:
            k = np.arange(1 << cls.LUT_BITS)
            ang = 2.0 * np.pi * k / (1 << cls.LUT_BITS)
            cls._COS = np.round(np.cos(ang) * ((1 << Q15) - 1)).astype(np.int64)
            cls._SIN = np.round(np.sin(ang) * ((1 << Q15) - 1)).astype(np.int64)
        return cls._COS, cls._SIN

    @classmethod
    def derotate(cls, i_arr, q_arr, phase_word: int, start_phase: int = 0):
        """Multiply the stream by exp(-j*phase): compensates a +phase_word/
        sample CFO. Vectorized reference of the per-sample accumulator."""
        cos_rom, sin_rom = cls._rom()
        n = len(i_arr)
        phases = (start_phase + phase_word * np.arange(n, dtype=np.int64)) & (PHASE_ONE - 1)
        idx = phases >> (PHASE_BITS - cls.LUT_BITS)
        c = cos_rom[idx]
        s = sin_rom[idx]
        i64 = np.asarray(i_arr, dtype=np.int64)
        q64 = np.asarray(q_arr, dtype=np.int64)
        # (i + jq) * (c - js)
        out_i = rshift_round(i64 * c + q64 * s, Q15)
        out_q = rshift_round(q64 * c - i64 * s, Q15)
        return out_i, out_q


_CORDIC_ITERS = 16
# atan(2^-i) in phase-word units (2^32 = one turn)
_CORDIC_ATAN = [int(round(np.arctan(2.0 ** -i) / (2.0 * np.pi) * PHASE_ONE))
                for i in range(_CORDIC_ITERS)]
_CORDIC_GAIN_Q15 = int(round((1 << Q15) / np.prod(
    [np.sqrt(1 + 2.0 ** (-2 * i)) for i in range(_CORDIC_ITERS)])))


def cordic_atan2(y: int, x: int):
    """Vectoring-mode CORDIC. Returns (angle_word, magnitude): the angle in
    32-bit phase-word units (signed, +-half turn) and |x + jy| (int)."""
    x = int(x)
    y = int(y)
    angle = 0
    if x < 0:  # rotate into the right half-plane
        if y >= 0:
            x, y = y, -x
            angle = PHASE_ONE // 4
        else:
            x, y = -y, x
            angle = -(PHASE_ONE // 4)

    for i in range(_CORDIC_ITERS):
        if y > 0:
            x, y = x + (y >> i), y - (x >> i)
            angle += _CORDIC_ATAN[i]
        else:
            x, y = x - (y >> i), y + (x >> i)
            angle -= _CORDIC_ATAN[i]

    mag = (x * _CORDIC_GAIN_Q15) >> Q15
    return angle, mag


def cordic_mag(y: int, x: int) -> int:
    """Magnitude via CORDIC vectoring (angle discarded)."""
    return cordic_atan2(y, x)[1]
