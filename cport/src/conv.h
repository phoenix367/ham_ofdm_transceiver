/* Rate-1/3 K=7 convolutional code (133/171/165 octal) with puncturing to
 * 1/2, 2/3, 3/4 and integer soft-decision Viterbi -- C twin of
 * ofdm_phy/coding.py + ofdm_phy/fixed/viterbi.py. */
#ifndef OFDM_CONV_H
#define OFDM_CONV_H

#include <stdint.h>

#define CONV_K 7
#define CONV_SPEED 3
#define CONV_PAD (CONV_K - 1)
#define CONV_STATES 64
#define CONV_MAX_STEPS_PUB 2112 /* keep = conv.c CONV_MAX_STEPS */
#define LLR_MAX 31 /* 6-bit soft bits */

typedef enum { CC_R13 = 0, CC_R12, CC_R23, CC_R34 } cc_rate_t;

/* RX-side coded-stream length (the model's calc_cc_elements: whole
 * puncture periods -- may exceed the encoder's actual output, e.g. 144 vs
 * 142 for 100 bits at rate 3/4; the decoder tolerates the difference) */
int conv_cc_elements(cc_rate_t rate, int bits_count);

/* TX-side actual encoded length (mask sum over bits_count+tail elements) */
int conv_encoded_len(cc_rate_t rate, int bits_count);

/* tail-padded encode + puncture; out must hold conv_encoded_len() bits */
void conv_encode(cc_rate_t rate, const uint8_t *bits, int bits_count,
                 uint8_t *out);

/* integer max-log Viterbi over depunctured (zeros injected) soft bits.
 * soft: soft_len LLRs (positive = logical 1); unfilled puncture slots are
 * zero. out: bits_count decoded bits. work: traceback scratch of at
 * least (bits_count + CONV_PAD) * CONV_STATES / 8 bytes (1 bit per
 * state per step). */
void conv_decode(cc_rate_t rate, const int64_t *soft, int soft_len,
                 int bits_count, uint8_t *out, uint8_t *work);

#endif /* OFDM_CONV_H */
