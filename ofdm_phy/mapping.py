"""PSK bit-to-symbol mappers (BPSK, QPSK) as defined in the article."""

from abc import ABC

import numpy as np
import numpy.typing as npt


class PSKMapper(ABC):
    MU: int = 0
    _MAPPING_TABLE: dict = {}

    @classmethod
    def _mapping_table(cls) -> dict:
        return cls._MAPPING_TABLE

    @classmethod
    def map(cls, bits: npt.NDArray) -> npt.NDArray:
        m_table = cls._mapping_table()
        return np.array([m_table[tuple(b)] for b in bits])


class BPSKMapper(PSKMapper):
    MU = 1

    _MAPPING_TABLE = {
        (0,): -1 + 0j,
        (1,): 1 + 0j,
    }


class QPSKMapper(PSKMapper):
    MU = 2

    _NORM = np.sqrt(2.0)
    _MAPPING_TABLE = {
        (0, 0): (-1.0 - 1j) / _NORM,
        (0, 1): (-1.0 + 1j) / _NORM,
        (1, 1): (1.0 + 1j) / _NORM,
        (1, 0): (1.0 - 1j) / _NORM,
    }


def _qam16_table():
    a = 1.0 / np.sqrt(10.0)
    gray = {(0, 0): -3.0, (0, 1): -1.0, (1, 1): 1.0, (1, 0): 3.0}
    return {(b0, b1, b2, b3): (gray[(b0, b1)] + 1j * gray[(b2, b3)]) * a
            for b0 in (0, 1) for b1 in (0, 1)
            for b2 in (0, 1) for b3 in (0, 1)}


class QAM16Mapper(PSKMapper):
    """Gray-coded 16-QAM, unit average power. Bits (b0, b1) select the I
    level, (b2, b3) the Q level: (0,0)->-3, (0,1)->-1, (1,1)->+1, (1,0)->+3
    (in units of 1/sqrt(10))."""

    MU = 4

    _MAPPING_TABLE = _qam16_table()
