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
#include "dsp.h"
#include "tx.h" /* link_mode_t */

/* returns 0 and fills (start, cfo_word) on detection, -1 on no lock */
int rx_detect(link_mode_t mode, const samp_t *i_arr, const samp_t *q_arr,
              int n, int *start, int64_t *cfo_word);

/* ZC stage alone, on an already-derotated window (streaming reuse):
 * returns 0 + (time within window, residual cfo word incl. hypothesis) */
/* A pull source for the ZC scan.
 *
 * The scan sweeps up to 70688 samples at EXTREME (5.9 s of audio), but
 * its LIVE window is only preamble_len+1: at position p the correlation
 * reads [p, p+klen) and the energy term reads m=p-(ng-1)klen and
 * m+preamble_len, which is the same leading edge because ng*klen ==
 * preamble_len exactly. So the caller need not materialise the window --
 * it hands over a fetch and the scan keeps a bounded slice of it. */
typedef struct {
    void *ctx;
    /* write n derotated analytic samples starting at index k (k+n <= n_win) */
    void (*fetch)(void *ctx, int k, int n, samp_t *di, samp_t *dq);
} zc_src_t;

int rx_detect_zc_src(link_mode_t mode, const zc_src_t *src, int n,
                     int *time_out, int64_t *word_out);

int rx_detect_zc_window(link_mode_t mode, const samp_t *i_arr,
                        const samp_t *q_arr, int n, int *time_out,
                        int64_t *word_out);

/* lag-N phase estimate of a derotated segment -> residual phase word */
int64_t rx_lag_n_word(const samp_t *di, const samp_t *dq, int n);

/* Same correlations, pulled instead of buffered: the lags are 128 and 64,
 * so these need a 128-sample delay line, not the segment. */
int64_t rx_lag_n_word_src(const zc_src_t *src, int n);
int64_t rx_residual_word_src(const zc_src_t *src, int n);

#endif /* OFDM_RX_DETECT_H */
