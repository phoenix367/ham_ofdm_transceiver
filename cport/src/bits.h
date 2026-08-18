/* Bit pipeline: CRC-8/16, LFSR scrambler, block interleaver -- C twin of
 * ofdm_phy/{crc,scrambler,interleaver}.py. */
#ifndef OFDM_BITS_H
#define OFDM_BITS_H

#include <stdint.h>

#define SCRAMBLER_SEED 0x5A

uint32_t crc8_lte(const uint8_t *bits, int n);
uint32_t crc16_ccitt(const uint8_t *bits, int n);

/* XOR hard bits with the 15-bit LFSR PRBS (taps 7 and 4), in place ok */
void scramble_bits(const uint8_t *in, int n, uint8_t *out);

/* flip LLR signs where the PRBS bit is 1 (soft descramble), in place ok */
void descramble_llrs(const int64_t *in, int n, int64_t *out);

/* transpose block interleaver over num_carriers columns (pruned) */
void interleave_u8(const uint8_t *in, int n, int num_carriers, uint8_t *out);
void deinterleave_i64(const int64_t *in, int n, int num_carriers, int64_t *out);

#endif /* OFDM_BITS_H */
