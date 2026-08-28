/* Bit pipeline: CRC-8/16, LFSR scrambler, block interleaver -- C twin of
 * ofdm_phy/{crc,scrambler,interleaver}.py. */
#ifndef OFDM_BITS_H
#define OFDM_BITS_H

#include <stdint.h>

/* LLR sample type -- see rx_internal.h for the rationale and the
 * widening rule for products. */
#ifndef OFDM_LLR_T
#define OFDM_LLR_T
typedef int32_t llr_t;
#endif

#define SCRAMBLER_SEED 0x5A

uint32_t crc8_lte(const uint8_t *bits, int n);
uint32_t crc16_ccitt(const uint8_t *bits, int n);

/* XOR hard bits with the 15-bit LFSR PRBS (taps 7 and 4), in place ok */
void scramble_bits(const uint8_t *in, int n, uint8_t *out);

/* flip LLR signs where the PRBS bit is 1 (soft descramble), in place ok */
void descramble_llrs(const llr_t *in, int n, llr_t *out);

/* transpose block interleaver over num_carriers columns (pruned) */
void interleave_u8(const uint8_t *in, int n, int num_carriers, uint8_t *out);
void deinterleave_i64(const llr_t *in, int n, int num_carriers, llr_t *out);

#endif /* OFDM_BITS_H */
