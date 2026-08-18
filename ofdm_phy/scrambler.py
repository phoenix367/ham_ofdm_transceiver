"""15-bit LFSR scrambler (feedback taps 7 and 4, i.e. bits 6 and 3).

`scramble` XORs hard bits with the PRBS; `descramble` operates on soft bits
(LLRs) by flipping their sign where the PRBS bit is 1.
"""

import typing

import numpy as np
import numpy.typing as npt

DEFAULT_SEED = 0x5A


def scrambler(data: npt.NDArray, seed: int,
              operator: typing.Callable[[int, typing.Any], typing.Any]) -> npt.NDArray:
    scrambled = np.zeros_like(data)

    lfsr = seed & 0x7FFF
    if lfsr == 0:
        lfsr = 1

    for i in range(len(data)):
        fb = ((lfsr >> 6) ^ (lfsr >> 3)) & 1
        prbs_bit = lfsr & 1
        scrambled[i] = operator(prbs_bit, data[i])
        lfsr = ((lfsr << 1) | fb) & 0x7FFF

    return scrambled


def scramble(bits: npt.NDArray[np.uint8], seed: int = DEFAULT_SEED) -> npt.NDArray[np.uint8]:
    return scrambler(bits, seed, operator=lambda prbs_bit, value: value ^ prbs_bit)


def descramble(llr_bits: npt.NDArray[np.float64], seed: int = DEFAULT_SEED) -> npt.NDArray[np.float64]:
    return scrambler(llr_bits, seed, operator=lambda prbs_bit, value: value * (-1.0 if prbs_bit == 1 else 1.0))
