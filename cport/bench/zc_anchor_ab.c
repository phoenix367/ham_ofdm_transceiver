/* Does anchoring the ZC scan at the tone field's end cost sensitivity?
 *
 * The anchoring cut EXTREME acquisition 4x and the raw ring nearly in
 * half, but it narrows the EXTREME search from 62497 candidate
 * offsets to 8225 -- and a narrower search is exactly the kind of change
 * trades sensitivity for cost without saying so. The C suite could not
 * answer it: its only noisy case is NORMAL at -5 dB against golden
 * samples, so there is no EXTREME sweep and no false-alarm count.
 *
 * This runs both arms over BYTE-IDENTICAL waveforms (same seed, same
 * noise, same CFO) and counts frames decoded, plus a noise-only pass
 * for false alarms. Build twice:
 *
 *   gcc ... -DZC_ANCHOR_LEGACY ... -o zc_ab_legacy
 *   gcc ...                   ... -o zc_ab_anchored
 *
 * The SNR here is referenced to the full-band RMS of the transmitted
 * waveform, NOT the article's convention -- the number that matters is
 * the DIFFERENCE between the arms on the same input, not its absolute
 * value.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "tx.h"
#include "dsp.h"
#include "packets.h"
#include "rx_stream.h"

#define FS 12000.0
#define MAXN 700000

static int16_t clean[MAXN], noisy[MAXN];
static samp_t ai[MAXN], aq[MAXN], bi[MAXN], bq[MAXN];
static uint8_t pkt[280];

static uint64_t rng_s;
static void rseed(uint64_t s) { rng_s = s ? s : 1; }
static uint32_t rnext(void)
{
    rng_s = rng_s * 6364136223846793005ULL + 1442695040888963407ULL;
    return (uint32_t)(rng_s >> 33);
}
/* Box-Muller, so the noise is actually Gaussian rather than merely
 * zero-mean: a uniform or triangular surrogate flatters a threshold
 * detector at exactly the SNRs this sweep is about. */
static double rnorm(void)
{
    double u1 = (rnext() + 1.0) / 4294967296.0;
    double u2 = (rnext() + 1.0) / 4294967296.0;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/* frequency-shift a real waveform by hz, via its analytic signal */
static void apply_cfo(int16_t *x, int n, double hz)
{
    int64_t word = (int64_t)(-hz / FS * 4294967296.0);
    int k;
    hilbert_analytic(x, n, ai, aq);
    nco_derotate(ai, aq, n, word, 0, bi, bq);
    for (k = 0; k < n; k++) {
        int64_t v = bi[k];
        x[k] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
    }
}

static double rms_of(const int16_t *x, int n)
{
    double acc = 0.0;
    int k;
    for (k = 0; k < n; k++)
        acc += (double)x[k] * x[k];
    return sqrt(acc / n);
}

/* one trial: returns 1 if the frame decoded with the right payload */
static int trial(int n, int pkt_n, double snr_db, double cfo_hz,
                 uint64_t seed, int noise_only)
{
    rxs_t *r;
    rxs_event_t ev;
    double sig, sigma;
    int k, pos, got = 0, ok = 0;

    memcpy(noisy, clean, sizeof(int16_t) * (size_t)n);
    if (!noise_only && cfo_hz != 0.0)
        apply_cfo(noisy, n, cfo_hz);
    sig = rms_of(noisy, n);
    if (noise_only)
        memset(noisy, 0, sizeof(int16_t) * (size_t)n);
    sigma = sig / pow(10.0, snr_db / 20.0);

    rseed(seed);
    for (k = 0; k < n; k++) {
        double v = noisy[k] + sigma * rnorm();
        noisy[k] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
    }

    r = rxs_open(MODE_EXTREME, 0);
    for (pos = 0; pos < n; pos += 512) {
        int c = n - pos < 512 ? n - pos : 512;
        if (rxs_push(r, noisy + pos, c, &ev)) {
            got++;
            if (ev.type == 1 && memcmp(ev.bits, pkt, (size_t)pkt_n) == 0)
                ok = 1;
        }
    }
    if (rxs_flush(r, &ev)) {
        got++;
        if (ev.type == 1 && memcmp(ev.bits, pkt, (size_t)pkt_n) == 0)
            ok = 1;
    }
    return noise_only ? got : ok;
}

int main(int argc, char **argv)
{
    static const double SNRS[] = { -8, -10, -11, -11.5, -12, -12.5, -13, -14 };
    int trials = argc > 1 ? atoi(argv[1]) : 20;
    uint8_t pay[27];
    int pkt_n, n, i, t;

    for (i = 0; i < 27; i++)
        pay[i] = (uint8_t)(i * 7 + 3);
    pkt_n = data_encode(123, pay, 27, pkt);
    n = tx_build_frame(MODE_EXTREME, pkt, pkt_n, PKT_TYP_DATA, MOD_BPSK,
                       CC_R13, clean + 2000);
    if (n <= 0) { printf("tx failed\n"); return 1; }
    memset(clean, 0, 2000 * sizeof(int16_t));
    n += 2000;

#ifdef ZC_ANCHOR_LEGACY
    printf("arm: LEGACY (scan from cs_abs, full tone field)\n");
#else
    printf("arm: ANCHORED (tone field end +- 8 blocks)\n");
#endif
    printf("EXTREME BPSK 1/3, %d samples/frame, %d trials/point\n",
           n, trials);
    printf("%8s %10s %10s\n", "SNR dB", "decoded", "rate");
    for (i = 0; i < (int)(sizeof(SNRS) / sizeof(SNRS[0])); i++) {
        int ok = 0;
        for (t = 0; t < trials; t++) {
            /* seed depends on SNR and trial ONLY -- so both arms see
             * byte-identical waveforms and the comparison is paired */
            uint64_t seed = 0x5eedULL * 1000003ULL
                            + (uint64_t)(int)(SNRS[i] * 10) * 7919ULL
                            + (uint64_t)t * 104729ULL;
            double cfo = -18.0 + 6.0 * (t % 7);   /* spread across bins */
            ok += trial(n, pkt_n, SNRS[i], cfo, seed, 0);
        }
        printf("%8.1f %10d %9.0f%%\n", SNRS[i], ok, 100.0 * ok / trials);
        fflush(stdout);
    }
    {   /* noise only: any event at all is a false alarm */
        int fa = 0, runs = trials;
        for (t = 0; t < runs; t++)
            fa += trial(n, pkt_n, 0.0, 0.0,
                        0xf00dULL + (uint64_t)t * 104729ULL, 1);
        printf("noise-only false alarms: %d event(s) in %d runs\n", fa, runs);
    }
    return 0;
}
