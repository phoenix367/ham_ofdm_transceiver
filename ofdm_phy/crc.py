"""CRC-8 (LTE) and CRC-16 (CCITT/Wi-Fi) over bit arrays, as in the article."""

import numpy as np
import numpy.typing as npt

POLY_ITU_LTE = 0x07
POLY_CCITT_WIFI = 0x1021


def crc(bits: npt.NDArray[np.uint8], polynomial: int, seed: int, crc_bits: int) -> int:
    mask = (1 << crc_bits) - 1

    crc = seed & mask
    for bit in bits:
        crc_msb = (crc >> (crc_bits - 1)) & 1

        crc = (crc << 1) & mask

        if crc_msb ^ (bit & 1):
            crc ^= polynomial

    return crc


def crc8(bits: npt.NDArray[np.uint8], polynomial: int) -> int:
    return crc(bits, polynomial, 0xff, 8)


def crc16(bits: npt.NDArray[np.uint8], polynomial: int) -> int:
    return crc(bits, polynomial, 0xffff, 16)


def crc8_lte(bits: npt.NDArray[np.uint8]) -> int:
    return crc8(bits, POLY_ITU_LTE)


def crc16_ccitt(bits: npt.NDArray[np.uint8]) -> int:
    return crc16(bits, POLY_CCITT_WIFI)
