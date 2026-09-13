/* Channel impairment for the transmit output: AWGN + Rayleigh fading,
 * integer and saturating. See chanimp.h. */
#include <math.h>
#include <string.h>

#include "chanimp.h"
#include "dsp.h"
#include "fxp.h"
#include "rom_tables.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define CH_FS 12000
#define CH_SIGMA_PERIOD 256     /* samples between noise-rms refreshes */
#define CH_EWMA_SHIFT 12        /* clean-power EWMA: ~4096 samples */

static uint32_t xorshift32(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

int32_t chanimp_gauss(uint32_t *rng)
{
    /* Irwin-Hall: 12 uniforms on [-32768, 32767], variance 12 * 2^32/12
     * = 2^32, so the sum has rms 65536 (Q16) and is bounded at +-6 sigma
     * -- far enough out for BER work down to 1e-9. Two uniforms per
     * xorshift word. */
    int32_t acc = 0;
    int i;
    for (i = 0; i < 6; i++) {
        uint32_t x = xorshift32(rng);
        acc += (int32_t)(x & 0xFFFFu) - 32768;
        acc += (int32_t)(x >> 16) - 32768;
    }
    return acc;
}

static int16_t ch_sat16(chanimp_t *c, int64_t v)
{
    if (v > 32767) {
        c->sat++;
        return 32767;
    }
    if (v < -32768) {
        c->sat++;
        return -32768;
    }
    return (int16_t)v;
}

static void fade_params(chanimp_t *c)
{
    double alpha, sig_h, sig_g;
    c->paths = (c->fade_chz > 0 && c->delay > 0) ? 2 : 1;
    if (c->fade_chz <= 0) {
        c->alpha_q15 = 0;
        c->innov_q15 = 0;
        return;
    }
    /* one-pole AR(1) at CH_FS/CHANIMP_STEP updates per second: 3 dB
     * width ~ (F/2pi) * alpha, so alpha = 2pi * fd / F */
    alpha = 2.0 * M_PI * (c->fade_chz / 100.0) / ((double)CH_FS / CHANIMP_STEP);
    if (alpha > 0.5)
        alpha = 0.5;
    c->alpha_q15 = (int32_t)(32768.0 * alpha + 0.5);
    if (c->alpha_q15 < 1)
        c->alpha_q15 = 1;
    alpha = c->alpha_q15 / 32768.0;
    /* stationary variance of the AR(1) is sig_g^2 * alpha / (2 - alpha)
     * PER COMPONENT; want |h|^2 = re^2 + im^2 to average 1/paths per
     * tap, so each component gets 1/(2 paths) and the sum of taps has
     * unit mean gain -- chan_snr stays the mean SNR */
    sig_h = 1.0 / sqrt(2.0 * (double)c->paths);
    sig_g = sig_h * sqrt((2.0 - alpha) / alpha);
    c->innov_q15 = (int32_t)(32768.0 * sig_g + 0.5);
}

void chanimp_init(chanimp_t *c, uint32_t seed)
{
    memset(c, 0, sizeof(*c));
    c->rng = seed ? seed : 0x9E3779B9u;
    c->snr_db = CHANIMP_OFF;
    c->paths = 1;
}

void chanimp_set_snr(chanimp_t *c, int snr_db)
{
    if (snr_db == CHANIMP_OFF) {
        c->snr_db = CHANIMP_OFF;
        c->gain_q15 = 0;
        c->sigma = 0;
        return;
    }
    if (snr_db < -60)
        snr_db = -60;
    if (snr_db > 60)
        snr_db = 60;
    c->snr_db = snr_db;
    c->gain_q15 = (int32_t)(32768.0 * pow(10.0, -snr_db / 20.0) + 0.5);
    c->sig_left = 0;            /* refresh on the next sample */
}

void chanimp_set_fade(chanimp_t *c, int fade_chz)
{
    if (fade_chz < 0)
        fade_chz = 0;
    if (fade_chz > CHANIMP_MAX_FADE_CHZ)
        fade_chz = CHANIMP_MAX_FADE_CHZ;
    c->fade_chz = fade_chz;
    fade_params(c);
}

void chanimp_set_delay(chanimp_t *c, int delay_100us)
{
    int d;
    if (delay_100us < 0)
        delay_100us = 0;
    d = delay_100us * (CH_FS / 10) / 1000;   /* 0.1 ms = 1.2 samples */
    if (d > CHANIMP_MAX_DELAY)
        d = CHANIMP_MAX_DELAY;
    c->delay_100us = delay_100us;
    c->delay = d;
    fade_params(c);
}

int chanimp_active(const chanimp_t *c)
{
    return c->gain_q15 != 0 || c->fade_chz > 0;
}

/* FADING HEADROOM. The transmitter uses the whole int16 range, and a
 * Rayleigh tap reaches 2x the mean amplitude 1.8 % of the time and 3x
 * 0.01 % -- through a unit-mean channel a full-scale waveform would
 * clamp at every peak and the "channel" would be a clipper. The fading
 * path therefore runs CH_FADE_SHIFT = 6 dB down, and the noise with it,
 * so chan_snr is still the mean SNR at the peer; |h| > 2 still clamps,
 * rarely, and is counted. AWGN alone has no headroom to give: it is
 * added to the waveform as transmitted. */
#define CH_FADE_SHIFT 1

static void refresh_sigma(chanimp_t *c)
{
    int64_t rms = isqrt_i64(c->ms);
    int shift = Q15 + (c->fade_chz > 0 ? CH_FADE_SHIFT : 0);
    c->sigma = (int32_t)((rms * c->gain_q15) >> shift);
    c->sig_left = CH_SIGMA_PERIOD;
}

static void fade_step(chanimp_t *c)
{
    int k;
    for (k = 0; k < c->paths; k++) {
        int32_t g_re = (int32_t)(((int64_t)c->innov_q15 * chanimp_gauss(&c->rng)) >> 16);
        int32_t g_im = (int32_t)(((int64_t)c->innov_q15 * chanimp_gauss(&c->rng)) >> 16);
        c->h_re[k] += (int32_t)(((int64_t)c->alpha_q15 * (g_re - c->h_re[k])) >> 15);
        c->h_im[k] += (int32_t)(((int64_t)c->alpha_q15 * (g_im - c->h_im[k])) >> 15);
        /* glide to the new target over the step: no 100 Hz staircase
         * in the gain, which would be a 100 Hz sideband on the air */
        c->d_re[k] = (c->h_re[k] - c->c_re[k]) / CHANIMP_STEP;
        c->d_im[k] = (c->h_im[k] - c->c_im[k]) / CHANIMP_STEP;
    }
    c->step_left = CHANIMP_STEP;
}

void chanimp_start(chanimp_t *c)
{
    int k;
    memset(c->hist, 0, sizeof(c->hist));
    memset(c->ai, 0, sizeof(c->ai));
    memset(c->aq, 0, sizeof(c->aq));
    c->n = 0;
    c->ms = 0;
    c->ms_init = 0;
    c->sigma = 0;
    c->sig_left = 0;
    c->tail = (c->fade_chz > 0) ? HILBERT_DELAY + c->delay : 0;
    for (k = 0; k < 2; k++) {
        /* draw from the stationary distribution: no warm-up ramp */
        int32_t sig_h_q15 = (int32_t)(32768.0 / sqrt(2.0 * (double)c->paths) + 0.5);
        c->h_re[k] = (int32_t)(((int64_t)sig_h_q15 * chanimp_gauss(&c->rng)) >> 16);
        c->h_im[k] = (int32_t)(((int64_t)sig_h_q15 * chanimp_gauss(&c->rng)) >> 16);
        c->c_re[k] = c->h_re[k];
        c->c_im[k] = c->h_im[k];
        c->d_re[k] = c->d_im[k] = 0;
    }
    c->step_left = CHANIMP_STEP;
}

void chanimp_apply(chanimp_t *c, int16_t *x, int n)
{
    int fading = c->fade_chz > 0;
    int i;

    if (!chanimp_active(c) || n <= 0)
        return;
    if (!c->ms_init) {
        /* the first chunk is the whole estimate until the EWMA has
         * something to work with */
        int64_t acc = 0;
        for (i = 0; i < n; i++)
            acc += (int64_t)x[i] * x[i];
        c->ms = acc / n;
        c->ms_init = 1;
        refresh_sigma(c);
    }
    for (i = 0; i < n; i++) {
        int32_t s = x[i];
        int64_t y;

        c->ms += (((int64_t)s * s) - c->ms) >> CH_EWMA_SHIFT;
        if (--c->sig_left <= 0)
            refresh_sigma(c);

        if (fading) {
            uint32_t idx = c->n & (CHANIMP_HIST - 1);
            int64_t acc = 0;
            int k;
            c->hist[idx] = (int16_t)s;
            /* Hilbert FIR over the ring: q[n] = sum taps[N-1-k] s[n-(N-1)+k].
             * Slots not yet written are zero (cleared at start), which
             * is the zero prehistory dsp.c's frame-at-once version
             * assumes. */
            for (k = 0; k < HILBERT_TAPS_N; k++)
                acc += (int64_t)HILBERT_TAPS[HILBERT_TAPS_N - 1 - k]
                       * c->hist[(c->n - (HILBERT_TAPS_N - 1) + (uint32_t)k)
                                 & (CHANIMP_HIST - 1)];
            c->ai[idx] = c->hist[(c->n - HILBERT_DELAY) & (CHANIMP_HIST - 1)];
            c->aq[idx] = (int32_t)rshift_round(acc, Q15);
            /* y = sum_k Re{ h_k * a[n - d_k] } */
            y = 0;
            for (k = 0; k < c->paths; k++) {
                uint32_t j = (c->n - (k ? (uint32_t)c->delay : 0u))
                             & (CHANIMP_HIST - 1);
                y += (int64_t)c->c_re[k] * c->ai[j]
                     - (int64_t)c->c_im[k] * c->aq[j];
            }
            y = rshift_round(y, Q15 + CH_FADE_SHIFT);
            for (k = 0; k < c->paths; k++) {
                c->c_re[k] += c->d_re[k];
                c->c_im[k] += c->d_im[k];
            }
            if (--c->step_left <= 0)
                fade_step(c);
            c->n++;
        } else {
            y = s;
        }
        if (c->gain_q15)
            y += ((int64_t)c->sigma * chanimp_gauss(&c->rng)) >> 16;
        x[i] = ch_sat16(c, y);
        c->samples++;
    }
}

int chanimp_tail(const chanimp_t *c)
{
    return chanimp_active(c) ? c->tail : 0;
}

int chanimp_flush(chanimp_t *c, int16_t *out, int cap)
{
    int m = c->tail;
    if (!chanimp_active(c) || m <= 0 || cap <= 0)
        return 0;
    if (m > cap)
        m = cap;
    memset(out, 0, (size_t)m * sizeof(*out));
    chanimp_apply(c, out, m);
    c->tail -= m;
    return m;
}
