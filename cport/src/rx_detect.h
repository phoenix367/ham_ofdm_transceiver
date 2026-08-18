/* Two-stage frame detection -- C twin of the detection half of
 * ofdm_phy/fixed/rx.py: block-spectrogram tone contrast (coarse CFO by
 * mask-shift grid + lag-N residual), then group-coherent ZC matched
 * filtering with fractional-CFO hypotheses (fine timing + CFO).
 *
 * Host reference operates frame-at-once like the Python model; the MCU
 * streaming state machine wraps these primitives later. */
#ifndef OFDM_RX_DETECT_H
#define OFDM_RX_DETECT_H

#include <stdint.h>
#include "tx.h" /* link_mode_t */

/* returns 0 and fills (start, cfo_word) on detection, -1 on no lock */
int rx_detect(link_mode_t mode, const int64_t *i_arr, const int64_t *q_arr,
              int n, int *start, int64_t *cfo_word);

/* ZC stage alone, on an already-derotated window (streaming reuse):
 * returns 0 + (time within window, residual cfo word incl. hypothesis) */
int rx_detect_zc_window(link_mode_t mode, const int64_t *i_arr,
                        const int64_t *q_arr, int n, int *time_out,
                        int64_t *word_out);

/* lag-N phase estimate of a derotated segment -> residual phase word */
int64_t rx_lag_n_word(const int64_t *di, const int64_t *dq, int n);

#endif /* OFDM_RX_DETECT_H */
