/* Rate-1/3 IRA LDPC (N=768, K=256) -- C twin of ofdm_phy/ldpc.py's
 * integer paths: accumulator encoder + integer normalized min-sum
 * (alpha = 0.75 as x - (x >> 2)). The graph is dumped from the Python
 * model (rom_ldpc.h), never rebuilt. Shortening: info tail is known-zero,
 * not transmitted, pinned to a strong "zero" LLR at the decoder. */
#ifndef OFDM_LDPC_H
#define OFDM_LDPC_H

#include <stdint.h>

/* LLR sample type -- see rx_internal.h. */
#ifndef OFDM_LLR_T
#define OFDM_LLR_T
typedef int32_t llr_t;
#endif

/* transmitted coded length for bits_count info bits: N - (K - k) */
int ldpc_cc_elements(int bits_count);

/* encode bits_count info bits; out receives ldpc_cc_elements() bits */
void ldpc_encode(const uint8_t *bits, int bits_count, uint8_t *out);

/* integer min-sum decode: soft (positive = logical 1) of length
 * ldpc_cc_elements(bits_count); out receives bits_count bits */
void ldpc_decode_int(const llr_t *soft, int soft_len, int bits_count,
                     uint8_t *out);

#endif /* OFDM_LDPC_H */
