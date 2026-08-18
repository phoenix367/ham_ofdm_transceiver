"""Radix-2 DIT fixed-point FFT with a Q15 twiddle ROM.

Each butterfly stage right-shifts by one (with rounding), so an N-point
transform carries a built-in 1/N scaling -- overflow-free with int16-range
inputs and exactly numpy's `ifft` normalization when used as the inverse.
The inverse is the conjugate trick: ifft(x) = conj(fft(conj(x))).

fft_bfp() wraps the transform in block floating point: the input block is
pre-shifted to fill the dynamic range (RTL: priority encoder + barrel
shifter) and the applied exponent is returned alongside the spectrum.
"""

import numpy as np

from .fxp import Q15, rshift_round, block_exponent

_TWIDDLE_ROM = {}


def _twiddles(n: int):
    """Q15 twiddle ROM for an n-point transform (cos, -sin of 2*pi*k/n)."""
    if n not in _TWIDDLE_ROM:
        k = np.arange(n // 2)
        ang = 2.0 * np.pi * k / n
        _TWIDDLE_ROM[n] = (
            np.round(np.cos(ang) * ((1 << Q15) - 1)).astype(np.int64),
            np.round(-np.sin(ang) * ((1 << Q15) - 1)).astype(np.int64),
        )
    return _TWIDDLE_ROM[n]


def _bit_reverse_indices(n: int):
    bits = n.bit_length() - 1
    idx = np.arange(n)
    rev = np.zeros(n, dtype=np.int64)
    for b in range(bits):
        rev |= ((idx >> b) & 1) << (bits - 1 - b)
    return rev


def fft_fixed(x_re, x_im):
    """Fixed-point DFT with 1/N scaling. int-array in, int64 arrays out."""
    n = len(x_re)
    wr_rom, wi_rom = _twiddles(n)

    rev = _bit_reverse_indices(n)
    re = np.asarray(x_re, dtype=np.int64)[rev].copy()
    im = np.asarray(x_im, dtype=np.int64)[rev].copy()

    half = 1
    while half < n:
        step = n // (2 * half)
        for start in range(0, n, 2 * half):
            k = np.arange(half)
            i0 = start + k
            i1 = i0 + half
            wr = wr_rom[k * step]
            wi = wi_rom[k * step]

            # butterfly: t = w * x[i1]; x[i1] = x[i0] - t; x[i0] += t
            tr = rshift_round(re[i1] * wr - im[i1] * wi, Q15)
            ti = rshift_round(re[i1] * wi + im[i1] * wr, Q15)

            # per-stage >>1 keeps the datapath width flat (1/N total)
            re[i1] = rshift_round(re[i0] - tr, 1)
            im[i1] = rshift_round(im[i0] - ti, 1)
            re[i0] = rshift_round(re[i0] + tr, 1)
            im[i0] = rshift_round(im[i0] + ti, 1)
        half *= 2

    return re, im


def ifft_fixed(x_re, x_im):
    """Inverse via the conjugate trick; carries the same 1/N scaling, i.e.
    matches numpy.fft.ifft for Q15 inputs."""
    re, im = fft_fixed(x_re, -np.asarray(x_im, dtype=np.int64))
    return re, -im


def fft_bfp(x_re, x_im, target_bits: int = 14):
    """Block-floating-point forward FFT: returns (re, im, exp) where the
    spectrum is scaled by 2**exp relative to fft_fixed of the raw input."""
    exp = block_exponent(x_re, x_im, target_bits)
    if exp >= 0:
        re_in = np.asarray(x_re, dtype=np.int64) << exp
        im_in = np.asarray(x_im, dtype=np.int64) << exp
    else:
        re_in = rshift_round(np.asarray(x_re, dtype=np.int64), -exp)
        im_in = rshift_round(np.asarray(x_im, dtype=np.int64), -exp)
    re, im = fft_fixed(re_in, im_in)
    return re, im, exp
