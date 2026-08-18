"""Fixed-point primitives.

Conventions: samples and coefficients are Q15 (int16 range) unless stated;
intermediates are int64 numpy arrays (RTL widths are noted at each use site).
Rounding is round-half-up via add-before-shift, saturation is symmetric
clipping -- the two standard RTL choices.
"""

import numpy as np

Q15 = 15
Q15_ONE = 1 << Q15  # 32768


def sat(x, bits: int):
    """Saturate to a signed `bits`-wide integer."""
    lo = -(1 << (bits - 1))
    hi = (1 << (bits - 1)) - 1
    return np.clip(x, lo, hi)


def rshift_round(x, shift: int):
    """Arithmetic right shift with round-half-up (add 2^(shift-1) first)."""
    if shift <= 0:
        return np.asarray(x) << (-shift)
    return (np.asarray(x) + (1 << (shift - 1))) >> shift


def cmul_q15(ar, ai, br, bi):
    """(ar + j*ai) * (br + j*bi) with Q15 coefficients b: products are
    int16*int16 -> int32, rounded back by 15. Returns int64 arrays."""
    ar = np.asarray(ar, dtype=np.int64)
    ai = np.asarray(ai, dtype=np.int64)
    re = rshift_round(ar * br - ai * bi, Q15)
    im = rshift_round(ar * bi + ai * br, Q15)
    return re, im


def block_exponent(x_re, x_im, target_bits: int = 14):
    """Block-floating-point prescale: left-shift so the largest |component|
    occupies `target_bits` bits (RTL: priority encoder on the block max).
    Returns the shift (may be negative for large inputs)."""
    m = int(max(np.max(np.abs(x_re)), np.max(np.abs(x_im)), 1))
    return target_bits - m.bit_length()


def isqrt_i64(x: int) -> int:
    """Integer square root (RTL: non-restoring root extractor)."""
    import math
    return math.isqrt(int(x))
