#include "bits.h"

static uint32_t crc_bits(const uint8_t *bits, int n, uint32_t poly,
                         uint32_t seed, int width)
{
    uint32_t mask = ((uint32_t)1 << width) - 1;
    uint32_t c = seed & mask;
    int i;
    for (i = 0; i < n; i++) {
        uint32_t msb = (c >> (width - 1)) & 1;
        c = (c << 1) & mask;
        if (msb ^ (bits[i] & 1))
            c ^= poly;
    }
    return c;
}

uint32_t crc8_lte(const uint8_t *bits, int n)
{
    return crc_bits(bits, n, 0x07, 0xff, 8);
}

uint32_t crc16_ccitt(const uint8_t *bits, int n)
{
    return crc_bits(bits, n, 0x1021, 0xffff, 16);
}

/* 15-bit LFSR, feedback = bit6 ^ bit3, PRBS output = bit0 */
static uint32_t lfsr_init(void)
{
    uint32_t l = SCRAMBLER_SEED & 0x7FFF;
    return l ? l : 1;
}

static uint32_t lfsr_step(uint32_t *lfsr, uint32_t *prbs_bit)
{
    uint32_t fb = ((*lfsr >> 6) ^ (*lfsr >> 3)) & 1;
    *prbs_bit = *lfsr & 1;
    *lfsr = ((*lfsr << 1) | fb) & 0x7FFF;
    return *prbs_bit;
}

void scramble_bits(const uint8_t *in, int n, uint8_t *out)
{
    uint32_t lfsr = lfsr_init(), p;
    int i;
    for (i = 0; i < n; i++) {
        lfsr_step(&lfsr, &p);
        out[i] = in[i] ^ (uint8_t)p;
    }
}

void descramble_llrs(const llr_t *in, int n, llr_t *out)
{
    uint32_t lfsr = lfsr_init(), p;
    int i;
    for (i = 0; i < n; i++) {
        lfsr_step(&lfsr, &p);
        out[i] = p ? -in[i] : in[i];
    }
}

/* interleaved read order: column-major walk of an R x C row-major block,
 * skipping cells beyond n (R = ceil(n / C)) */
void interleave_u8(const uint8_t *in, int n, int num_carriers, uint8_t *out)
{
    int rows = (n + num_carriers - 1) / num_carriers;
    int c, r, k = 0;
    for (c = 0; c < num_carriers; c++)
        for (r = 0; r < rows; r++) {
            int idx = r * num_carriers + c;
            if (idx < n)
                out[k++] = in[idx];
        }
}

void deinterleave_i64(const llr_t *in, int n, int num_carriers, llr_t *out)
{
    int rows = (n + num_carriers - 1) / num_carriers;
    int c, r, k = 0;
    for (c = 0; c < num_carriers; c++)
        for (r = 0; r < rows; r++) {
            int idx = r * num_carriers + c;
            if (idx < n)
                out[idx] = in[k++];
        }
}
