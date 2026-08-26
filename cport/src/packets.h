/* Packet bit layouts (TX side) -- C twin of ofdm_phy/packets.py.
 * Header: ver(2) typ(3) mod(2) spd(2) len(8) + CRC-8  -> 25 bits
 * Data:   reserved(20) payload(8*n) + CRC-16          -> 36 + 8*n bits
 * All fields MSB-first. */
#ifndef OFDM_PACKETS_H
#define OFDM_PACKETS_H

#include <stdint.h>

#define HEADER_BITS 25
#define PKT_TYP_BEACON 0
#define PKT_TYP_DATA 4
/* extended data frame: header `len` counts payload BYTES (not packet
 * bits) -> payloads up to 255 bytes; conv FEC only (LDPC K=256 is too
 * small). C-stack extension beyond the article's format. */
#define PKT_TYP_EXT_DATA 5
/* broadcast (non-ARQ): Data-shaped, but the reserved field is NOT a link
 * control word and the frame must never reach the ARQ reassembler */
#define PKT_TYP_BCAST 6

/* packet bit count from a decoded header (type-dependent len units) */
#define PKT_BITS_FROM_HDR(typ, len) \
    ((typ) == PKT_TYP_EXT_DATA ? 36 + 8 * (len) : (len))

void header_encode(int ver, int typ, int mod, int spd, int len,
                   uint8_t out[HEADER_BITS]);

/* returns bit count = 20 + 8*payload_len + 16 */
int data_encode(uint32_t reserved, const uint8_t *payload, int payload_len,
                uint8_t *out);

/* parse + CRC-check a received header; returns 0 or -1 on CRC mismatch */
int header_decode(const uint8_t bits[HEADER_BITS], int *ver, int *typ,
                  int *mod, int *spd, int *len);

/* CRC-16 check over a Data packet's bits (reserved+payload+crc) */
int data_check_crc(const uint8_t *bits, int nbits);

#endif /* OFDM_PACKETS_H */
