/* Channel impairment for the board's own transmit output -- a debug
 * mode. AWGN at a set SNR, and optionally Rayleigh fading (one flat
 * tap, or two equal taps with a delay: the Watterson HF model with a
 * one-pole Doppler spectrum), applied to the DAC samples before they
 * leave the board, so the peer sees a channel instead of a cross-wire.
 *
 * Integer only, and SATURATING: every stage that could overflow
 * (fading gain x sample, the two-path sum, the added noise) is computed
 * in int64 and clamped to int16 at the output; `sat` counts the clamps.
 * Nothing here touches the station or the receiver -- it is a filter
 * between the streaming transmitter and the DAC FIFO.
 *
 *   SNR        chan_snr (dB, CHANIMP_OFF = no noise). Relative to the
 *              CLEAN signal's mean power, tracked as an EWMA of s^2 over
 *              ~4096 samples (the transmitter's per-sample power is
 *              equalised between preamble and data by design, so the
 *              estimate settles within the first symbol).
 *   fading     chan_fade (Doppler spread, centi-Hz; 0 = no fading).
 *              Each tap is a complex Gaussian AR(1) process updated
 *              every CHANIMP_STEP samples (100 Hz) and linearly
 *              interpolated between updates; the one-pole's 3 dB width
 *              is set to the spread. Mean gain is 1 whatever the number
 *              of taps, so chan_snr stays the MEAN SNR.
 *   delay      chan_delay (second path, units of 0.1 ms; 0 = one tap).
 *              CCIR 520 "good" is 0.5 ms / 0.1 Hz, "moderate" 1 ms /
 *              0.5 Hz, "poor" 2 ms / 1 Hz.
 *
 * Fading runs 6 dB below the clean level (a full-scale waveform
 * through Rayleigh peaks would clamp at every peak otherwise) with the
 * noise scaled the same, so chan_snr is the mean SNR either way; the
 * peer sees 6 dB less absolute level, which the cross-wire's +22 dB
 * has room for. Fading needs the analytic signal (a complex tap on a
 * real passband signal), which costs the Hilbert FIR's group delay: the output runs
 * HILBERT_DELAY (+ the path delay) samples behind the input, so a
 * transmission must be FLUSHED at its end (chanimp_tail/chanimp_flush)
 * or its last symbol loses its tail. AWGN alone has no delay. */
#ifndef CHANIMP_H
#define CHANIMP_H

#include <stdint.h>

#define CHANIMP_OFF       999   /* chan_snr value meaning "no noise" */
#define CHANIMP_STEP      120   /* fading update period, samples (100 Hz) */
#define CHANIMP_HIST      256   /* clean-input ring: FIR + max delay */
#define CHANIMP_MAX_DELAY 120   /* 10 ms at 12 kHz */
#define CHANIMP_MAX_FADE_CHZ 800 /* 8 Hz: the one-pole's alpha stays < 0.5 */

typedef struct {
    /* settings */
    int snr_db;             /* CHANIMP_OFF = noise off */
    int fade_chz;           /* 0 = fading off */
    int delay_100us;        /* 0 = single tap */
    /* noise */
    uint32_t rng;
    int64_t ms;             /* EWMA of the clean s^2 */
    int ms_init;
    int32_t gain_q15;       /* 10^(-snr/20), 0 when off */
    int32_t sigma;          /* noise rms, LSB */
    int sig_left;           /* samples until sigma is refreshed */
    /* fading */
    int paths, delay;       /* taps in use, path-2 delay in samples */
    int32_t alpha_q15;      /* AR(1) pole */
    int32_t innov_q15;      /* innovation rms, Q15 */
    int32_t h_re[2], h_im[2];   /* tap targets, Q15 */
    int32_t c_re[2], c_im[2];   /* current (interpolated) taps, Q15 */
    int32_t d_re[2], d_im[2];   /* per-sample increments, Q15 */
    int step_left;
    int16_t hist[CHANIMP_HIST]; /* clean input */
    int32_t ai[CHANIMP_HIST];   /* analytic I (delayed input) */
    int32_t aq[CHANIMP_HIST];   /* analytic Q (Hilbert FIR) */
    uint32_t n;                 /* samples seen since start */
    int tail;                   /* samples still to flush at the end */
    /* accounting */
    uint32_t sat;               /* output clamps */
    uint32_t samples;           /* samples impaired since init */
} chanimp_t;

void chanimp_init(chanimp_t *c, uint32_t seed);
void chanimp_set_snr(chanimp_t *c, int snr_db);       /* CHANIMP_OFF = off */
void chanimp_set_fade(chanimp_t *c, int fade_chz);    /* 0 = off */
void chanimp_set_delay(chanimp_t *c, int delay_100us);/* 0 = one tap */
int  chanimp_active(const chanimp_t *c);              /* anything on? */

/* at key-up: forget the previous transmission (rings, power estimate),
 * draw the taps from their stationary distribution */
void chanimp_start(chanimp_t *c);
/* impair n samples in place */
void chanimp_apply(chanimp_t *c, int16_t *x, int n);
/* after the generator ran dry: how many delayed samples are still
 * inside, and emit up to cap of them (returns the count written) */
int  chanimp_tail(const chanimp_t *c);
int  chanimp_flush(chanimp_t *c, int16_t *out, int cap);

/* one unit-variance-ish Gaussian sample: sum of 12 uniforms, rms 65536
 * (Q16), bounded at +-6 sigma. Exposed for the tests. */
int32_t chanimp_gauss(uint32_t *rng);

#endif
