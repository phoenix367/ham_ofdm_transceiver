"""Packet layer: Header, Beacon and Data packets with CRC-8/CRC-16 protection.

Bit layouts follow the article:
  Header: ver(2) typ(3) mod(2) spd(2) len(8) + CRC-8      -> 25 bits
  Beacon: callsign(53, Base38) qth(25) mode(4) + CRC-8    -> 90 bits
  Data:   reserved(20) payload(8..216) + CRC-16
"""

import typing
from dataclasses import dataclass
from enum import Enum

import numpy as np
import numpy.typing as npt

from .crc import crc8_lte, crc16_ccitt


class PacketCRCMissmatch(Exception):
    pass


class PacketType(Enum):
    BEACON = 0
    DATA = 4


class ModType(Enum):
    BPSK = 0
    QPSK = 1
    QAM16 = 2


class CCSpeed(Enum):
    R13 = 0
    R12 = 1
    R23 = 2
    R34 = 3


class BeaconMode(Enum):
    BEACON = 15


# --- Base38 callsign codec -------------------------------------------------

ALPHABET = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/"
BASE_CALL = len(ALPHABET)
MAX_CALL_LEN = 10


def callsign_to_int(callsign: str) -> int:
    s = callsign.strip().upper()[:MAX_CALL_LEN].ljust(MAX_CALL_LEN, " ")
    val = 0
    for char in s:
        val = val * BASE_CALL + ALPHABET.index(char)
    return val


def int_to_callsign(val: int) -> str:
    chars = []
    for _ in range(MAX_CALL_LEN):
        chars.append(ALPHABET[val % BASE_CALL])
        val //= BASE_CALL
    return "".join(reversed(chars)).strip()


# --- 6-character QTH locator codec (FT8-style, extended to 6 chars) --------

def qth6_to_int(qth: str) -> int:
    q = qth.upper()
    assert len(q) == 6

    off = ord('A')
    lon_field = ord(q[0]) - off
    lat_field = ord(q[1]) - off
    lon_sq = int(q[2])
    lat_sq = int(q[3])
    lon_sub = ord(q[4]) - off
    lat_sub = ord(q[5]) - off

    val = lon_field
    val = val * 18 + lat_field
    val = val * 10 + lon_sq
    val = val * 10 + lat_sq
    val = val * 24 + lon_sub
    val = val * 24 + lat_sub
    return val


def int_to_qth6(val: int) -> str:
    lat_sub = val % 24
    val //= 24
    lon_sub = val % 24
    val //= 24
    lat_sq = val % 10
    val //= 10
    lon_sq = val % 10
    val //= 10
    lat_field = val % 18
    val //= 18
    lon_field = val % 18

    off = ord('A')
    return f"{chr(lon_field + off)}{chr(lat_field + off)}{lon_sq}{lat_sq}{chr(lon_sub + off)}{chr(lat_sub + off)}"


# --- Packet base classes ---------------------------------------------------

class PacketBase:
    @staticmethod
    def _to_bits(val: int, count: int) -> npt.NDArray[np.uint8]:
        val = val & ((1 << count) - 1)
        bytes_count = (count + 7) // 8
        val_bytes = val.to_bytes(bytes_count, byteorder='big')
        val_array = np.frombuffer(val_bytes, dtype=np.uint8)
        return np.unpackbits(val_array)[-count:]

    @staticmethod
    def _from_bits(bits: npt.NDArray[np.uint8]) -> int:
        pad_width = (8 - (len(bits) % 8)) % 8
        padded_bits = np.pad(bits, (pad_width, 0), mode="constant", constant_values=0)
        bytes_array = np.packbits(padded_bits)
        return int.from_bytes(bytes_array.tobytes(), byteorder='big')

    @classmethod
    def _calc_crc(cls, data: npt.NDArray[np.uint8]) -> int:
        raise NotImplementedError

    @classmethod
    def _append_crc(cls, data: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint8]:
        crc_val = cls._calc_crc(data)
        return np.concatenate([data, cls._to_bits(crc_val, cls.CRC_SIZE)])

    @classmethod
    def _check_crc(cls, data: npt.NDArray[np.uint8], crc_data: npt.NDArray[np.uint8],
                   raise_exception: bool = True) -> bool:
        crc_our = cls._calc_crc(data)
        crc_their = cls._from_bits(crc_data)

        ok = crc_our == crc_their

        if raise_exception and not ok:
            raise PacketCRCMissmatch("Packet CRC missmatch")

        return ok

    @classmethod
    def _extract_data(cls, bits: npt.NDArray[np.uint8], check_crc: bool) -> npt.NDArray[np.uint8]:
        data = bits[:-cls.CRC_SIZE]
        crc_data = bits[-cls.CRC_SIZE:]
        if check_crc:
            cls._check_crc(data, crc_data)
        return data


class PacketCRC8(PacketBase):
    CRC_SIZE: typing.ClassVar[int] = 8

    @classmethod
    def _calc_crc(cls, data: npt.NDArray[np.uint8]) -> int:
        return crc8_lte(data)


class PacketCRC16(PacketBase):
    CRC_SIZE: typing.ClassVar[int] = 16

    @classmethod
    def _calc_crc(cls, data: npt.NDArray[np.uint8]) -> int:
        return crc16_ccitt(data)


# --- Concrete packets ------------------------------------------------------

@dataclass
class Header(PacketCRC8):
    PACKET_SIZE: typing.ClassVar[int] = 17 + PacketCRC8.CRC_SIZE

    ver: int
    typ: PacketType
    mod: ModType
    spd: CCSpeed
    len: int

    def encode(self) -> npt.NDArray[np.uint8]:
        ver = self._to_bits(self.ver, 2)
        typ = self._to_bits(self.typ.value, 3)
        mod = self._to_bits(self.mod.value, 2)
        spd = self._to_bits(self.spd.value, 2)
        length = self._to_bits(self.len, 8)

        data = np.concatenate([ver, typ, mod, spd, length])

        packet = self._append_crc(data)
        return packet

    @classmethod
    def decode(cls, bits: npt.NDArray[np.uint8], check_crc: bool = True) -> 'Header':
        assert bits.shape == (cls.PACKET_SIZE,)

        data = cls._extract_data(bits, check_crc)

        ver = cls._from_bits(data[0:2])
        typ = cls._from_bits(data[2:5])
        mod = cls._from_bits(data[5:7])
        spd = cls._from_bits(data[7:9])
        length = cls._from_bits(data[9:17])

        return cls(ver, PacketType(typ), ModType(mod), CCSpeed(spd), length)


@dataclass
class Beacon(PacketCRC8):
    PACKET_SIZE: typing.ClassVar[int] = 82 + PacketCRC8.CRC_SIZE

    callsign: str
    qth: str
    mode: BeaconMode = BeaconMode.BEACON

    def encode(self) -> npt.NDArray[np.uint8]:
        call_val = callsign_to_int(self.callsign)
        qth_val = qth6_to_int(self.qth)

        call_bits = self._to_bits(call_val, 53)
        qth_bits = self._to_bits(qth_val, 25)
        mode_bits = self._to_bits(self.mode.value, 4)

        data = np.concatenate([call_bits, qth_bits, mode_bits])

        packet = self._append_crc(data)
        return packet

    @classmethod
    def decode(cls, bits: npt.NDArray, check_crc: bool = True) -> 'Beacon':
        assert bits.shape == (cls.PACKET_SIZE,)

        data = cls._extract_data(bits, check_crc)

        call_bits = data[0:53]
        qth_bits = data[53:78]
        flag_bits = data[78:82]

        call_val = cls._from_bits(call_bits)
        qth_val = cls._from_bits(qth_bits)
        mode = BeaconMode(cls._from_bits(flag_bits))

        return cls(int_to_callsign(call_val), int_to_qth6(qth_val), mode)


@dataclass
class Data(PacketCRC16):
    reserved: int
    payload: bytes

    def encode(self) -> npt.NDArray[np.uint8]:
        reserved = self._to_bits(self.reserved, 20)
        payload = np.unpackbits(np.frombuffer(self.payload, dtype=np.uint8))

        data = np.concatenate([reserved, payload])

        packet = self._append_crc(data)
        return packet

    @classmethod
    def decode(cls, bits: npt.NDArray[np.uint8], check_crc: bool = True) -> 'Data':
        assert bits.shape >= (44,)

        data = cls._extract_data(bits, check_crc)

        reserved = cls._from_bits(data[0:20])
        payload = np.packbits(data[20:]).tobytes()

        return cls(reserved, payload)


PACKET_CLASSES = {
    PacketType.BEACON: Beacon,
    PacketType.DATA: Data,
}
