"""Verify the implementation against every worked example printed in the article.

Run:  python experiments/verify_article.py
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ofdm_phy import (
    Header, Beacon, Data, PacketType, ModType, CCSpeed, BeaconMode,
    callsign_to_int, int_to_callsign, qth6_to_int, int_to_qth6,
    crc8_lte, crc16_ccitt,
    CCLTEBPSK_13, CCLTEBPSK_12, CCLTEBPSK_23, CCLTEBPSK_34,
    interleave, deinterleave, scramble, descramble,
    FullOFDMModem,
)

PASSED = 0
FAILED = 0


def check(name, actual, expected):
    global PASSED, FAILED
    ok = actual == expected
    if isinstance(ok, np.ndarray):
        ok = bool(ok.all())
    status = "PASS" if ok else "FAIL"
    if ok:
        PASSED += 1
    else:
        FAILED += 1
    print(f"[{status}] {name}")
    if not ok:
        print(f"       expected: {expected}")
        print(f"       actual:   {actual}")


def bits_str(bits):
    return "".join(str(int(b)) for b in bits)


def str_bits(s):
    return np.array([int(c) for c in s], dtype=np.uint8)


# --- modem geometry (section: Концепция протокола) --------------------------

modem = FullOFDMModem()
check("subcarrier spacing = 93.75 Hz", modem._freq_step, 93.75)
check("channel bins = 3..25 (23 carriers)", list(modem.channel_indices), list(range(3, 26)))
check("center bin = 14 (1312.5 Hz)", modem._bin_center, 14)
check("pilot bins = [3, 6, 10, 14, 17, 21, 25]", list(modem.pilot_carriers), [3, 6, 10, 14, 17, 21, 25])
check("data carriers = 16", modem.data_carriers_len, 16)
check("cyclic prefix = 32 samples (25%)", modem.cyclic_prefix, 32)
check("tiled symbol = CP + 4*128 samples", modem.symbol_len, 32 + 4 * 128)
check("newman tone bins = [8, 12, 16, 20]", list(modem._newman_preamble_bins), [8, 12, 16, 20])
check("newman shifted bins = [10, 14, 18, 22]",
      list(modem._newman_preamble_bins + 2), [10, 14, 18, 22])

# --- callsign / QTH codecs --------------------------------------------------

check("callsign_to_int('R9FEU') = 4671407026402144", callsign_to_int("R9FEU"), 4671407026402144)
check("int_to_callsign round-trip", int_to_callsign(4671407026402144), "R9FEU")
check("qth6_to_int('LO88CA') = 12261936", qth6_to_int("LO88CA"), 12261936)
check("int_to_qth6 round-trip", int_to_qth6(12261936), "LO88CA")

# --- CRC --------------------------------------------------------------------

crc8_input = str_bits(format(0b1000010011000100111101011010001101000111100110110000001011101100011000000001110000, "082b"))
check("crc8_lte example = 0b10101101", crc8_lte(crc8_input), 0b10101101)

# the CRC-16 example input is a Data packet body: reserved(20) + payload
# "    Though this be madness," (216 bits) = 236 bits total
CRC16_EXAMPLE = (
    "0000001100000101011100100000001000000010000000100000010101000110100001101111"
    "0111010101100111011010000010000001110100011010000110100101110011001000000110"
    "0010011001010010000001101101011000010110010001101110011001010111001101110011"
    "00101100")
crc16_input = str_bits(CRC16_EXAMPLE)
check("crc16 example input is 236 bits", len(crc16_input), 236)
check("crc16_ccitt example = 0b1001001100011111", crc16_ccitt(crc16_input), 0b1001001100011111)
check("crc16 example payload decodes to article text",
      np.packbits(crc16_input[20:]).tobytes(), b"    Though this be madness,")
check("crc16 example equals Data(reserved=12375, ...).encode() body",
      bits_str(Data(reserved=12375, payload=b"    Though this be madness,").encode()[:236]),
      CRC16_EXAMPLE)

# --- Header packet ----------------------------------------------------------

hdr = Header(ver=1, typ=PacketType.BEACON, mod=ModType.BPSK, spd=CCSpeed.R13, len=90)
hdr_bits = hdr.encode()
check("Header(ver=1, BEACON, BPSK, R13, len=90) bits",
      bits_str(hdr_bits), "0100000000101101010011110")
check("Header round-trip", Header.decode(hdr_bits), hdr)

# --- Beacon packet ----------------------------------------------------------

bcn = Beacon(callsign="R9FEU", qth="LO88CA", mode=BeaconMode.BEACON)
bcn_bits = bcn.encode()
check("Beacon packet size = 90 bits", len(bcn_bits), 90)
check("Beacon callsign field bits",
      bits_str(bcn_bits[:53]), "10000100110001001111010110100011010001111001101100000")
check("Beacon QTH field bits", bits_str(bcn_bits[53:78]), "0101110110001101000110000")
check("Beacon round-trip", Beacon.decode(bcn_bits), bcn)

# --- Data packet ------------------------------------------------------------

dat = Data(reserved=123, payload=b"    Though this be madness,")
dat_bits = dat.encode()
check("Data packet size = 20+216+16 bits", len(dat_bits), 252)
check("Data round-trip", Data.decode(dat_bits), dat)

# --- Convolutional code -----------------------------------------------------

impulse = CCLTEBPSK_13._encode(np.array([1, 0, 0, 0, 0, 0, 0], dtype=np.uint8))
check("CC impulse response (rate 1/3)", bits_str(impulse), "111100001110111011111")

check("CC codec constants", (CCLTEBPSK_13.SPEED, CCLTEBPSK_13.PAD_LEN, CCLTEBPSK_13.NUM_STATES), (3, 6, 64))

header_bits = str_bits("0100000000101101010011110")
enc13 = CCLTEBPSK_13.encode(header_bits)
check("header CC rate 1/3 (93 bits)", bits_str(enc13),
      "000111100001110111011111000000111100110101010011010010110001010011001011100011101011100111000")
enc12 = CCLTEBPSK_12.encode(header_bits)
check("header CC rate 1/2 (62 bits)", bits_str(enc12),
      "00111001111101110000111011110101010011010101000110011001101100")
enc23 = CCLTEBPSK_23.encode(header_bits)
check("header CC rate 2/3 (47 bits)", bits_str(enc23),
      "00110011101100011111101001011001000010010010100")
enc34 = CCLTEBPSK_34.encode(header_bits)
check("header CC rate 3/4 (42 bits)", bits_str(enc34),
      "001100110110001111100100110001001001011100")

# soft-decision Viterbi round-trip for every rate (clean channel)
for codec, enc in ((CCLTEBPSK_13, enc13), (CCLTEBPSK_12, enc12),
                   (CCLTEBPSK_23, enc23), (CCLTEBPSK_34, enc34)):
    soft = 2.0 * enc.astype(np.float64) - 1.0  # LLR: +1 for 1, -1 for 0
    dec = codec.decode(soft, len(header_bits))
    check(f"Viterbi round-trip {codec.__name__}", bits_str(dec), bits_str(header_bits))

# noisy Viterbi round-trip (rate 1/3, a few sign flips)
rng = np.random.default_rng(1)
soft = 2.0 * enc13.astype(np.float64) - 1.0
flip = rng.choice(len(soft), size=8, replace=False)
soft[flip] *= -1
check("Viterbi corrects 8/93 hard errors (rate 1/3)",
      bits_str(CCLTEBPSK_13.decode(soft, len(header_bits))), bits_str(header_bits))

# --- Interleaver ------------------------------------------------------------

idx = interleave(np.arange(32), 16)
check("interleave 32 bits over 16 carriers",
      list(idx), [0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23,
                  8, 24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31])
check("deinterleave inverts interleave",
      list(deinterleave(idx, 16)), list(np.arange(32)))

# --- Scrambler --------------------------------------------------------------

src = str_bits("011011011110000001110001111111111000110110011111000011001001000000101100100010100110011010111000")
expect = "011001110101111011100101100100011011001000011000011101011111010010101101100110010111000111100011"
scr = scramble(src)
check("scrambler example sequence", bits_str(scr), expect)
check("scramble is an involution", bits_str(scramble(scr)), bits_str(src))

llr = 2.0 * scr.astype(np.float64) - 1.0
descr_hard = (descramble(llr) > 0).astype(np.uint8)
check("descramble (soft) recovers source", bits_str(descr_hard), bits_str(src))

# --- Zadoff-Chu -------------------------------------------------------------

zc = FullOFDMModem._gen_zc_seq(17, 23)
check("ZC sequence constant amplitude", bool(np.allclose(np.abs(zc), 1.0)), True)
acf = np.array([np.abs(np.sum(zc * np.conj(np.roll(zc, k)))) for k in range(23)])
check("ZC periodic ACF is a delta function",
      bool(np.isclose(acf[0], 23) and np.all(acf[1:] < 1e-6)), True)

# ----------------------------------------------------------------------------

print(f"\n{PASSED} passed, {FAILED} failed")
sys.exit(1 if FAILED else 0)
