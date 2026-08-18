#include "packets.h"
#include "bits.h"

static uint8_t *put_bits(uint8_t *out, uint32_t val, int count)
{
    int i;
    for (i = count - 1; i >= 0; i--)
        *out++ = (uint8_t)((val >> i) & 1);
    return out;
}

void header_encode(int ver, int typ, int mod, int spd, int len,
                   uint8_t out[HEADER_BITS])
{
    uint8_t *p = out;
    p = put_bits(p, (uint32_t)ver, 2);
    p = put_bits(p, (uint32_t)typ, 3);
    p = put_bits(p, (uint32_t)mod, 2);
    p = put_bits(p, (uint32_t)spd, 2);
    p = put_bits(p, (uint32_t)len, 8);
    put_bits(p, crc8_lte(out, 17), 8);
}

int data_encode(uint32_t reserved, const uint8_t *payload, int payload_len,
                uint8_t *out)
{
    uint8_t *p = out;
    int i;
    p = put_bits(p, reserved, 20);
    for (i = 0; i < payload_len; i++)
        p = put_bits(p, payload[i], 8);
    put_bits(p, crc16_ccitt(out, 20 + 8 * payload_len), 16);
    return 20 + 8 * payload_len + 16;
}

static uint32_t get_bits(const uint8_t *bits, int off, int count)
{
    uint32_t v = 0;
    int i;
    for (i = 0; i < count; i++)
        v = (v << 1) | (bits[off + i] & 1);
    return v;
}

int header_decode(const uint8_t bits[HEADER_BITS], int *ver, int *typ,
                  int *mod, int *spd, int *len)
{
    if (crc8_lte(bits, 17) != get_bits(bits, 17, 8))
        return -1;
    *ver = (int)get_bits(bits, 0, 2);
    *typ = (int)get_bits(bits, 2, 3);
    *mod = (int)get_bits(bits, 5, 2);
    *spd = (int)get_bits(bits, 7, 2);
    *len = (int)get_bits(bits, 9, 8);
    return 0;
}

int data_check_crc(const uint8_t *bits, int nbits)
{
    if (nbits < 36)
        return -1;
    return crc16_ccitt(bits, nbits - 16) == get_bits(bits, nbits - 16, 16)
               ? 0 : -1;
}
