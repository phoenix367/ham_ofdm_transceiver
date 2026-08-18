/* Fixed-point transmitter -- C twin of ofdm_phy/fixed/tx.py.
 * Convolutional frames (ver=1); LDPC (ver=2) lands with ldpc.c. */
#ifndef OFDM_TX_H
#define OFDM_TX_H

#include <stdint.h>
#include "conv.h"

typedef enum { MODE_NORMAL = 0, MODE_ROBUST = 1, MODE_EXTREME = 2 } link_mode_t;
typedef enum { MOD_BPSK = 0, MOD_QPSK = 1, MOD_QAM16 = 2 } mod_type_t;

/* frame length in samples for a pkt_bits_n-bit packet (buffer sizing) */
int tx_frame_len(link_mode_t mode, int pkt_bits_n, mod_type_t mod,
                 cc_rate_t spd);
int tx_frame_len_ex(link_mode_t mode, int pkt_bits_n, mod_type_t mod,
                    cc_rate_t spd, int use_ldpc);

/* build a complete frame: preamble + BPSK/R13 header + data block.
 * pkt_bits: encoded packet (packets.h), typ: PKT_TYP_*. use_ldpc codes
 * the data block with the IRA LDPC (header ver=2). Returns the sample
 * count (== tx_frame_len_ex) or -1 on a bad argument. */
int tx_build_frame(link_mode_t mode, const uint8_t *pkt_bits, int pkt_bits_n,
                   int typ, mod_type_t mod, cc_rate_t spd, int16_t *out);
int tx_build_frame_ex(link_mode_t mode, const uint8_t *pkt_bits,
                      int pkt_bits_n, int typ, mod_type_t mod, cc_rate_t spd,
                      int use_ldpc, int16_t *out);

#endif /* OFDM_TX_H */
