/* chanimp: AWGN + Rayleigh fading on the transmit output, integer and
 * saturating. Checks the noise lands at the requested SNR, that the
 * Gaussian has the tails it claims, that saturation clamps instead of
 * wrapping, that fading conserves mean power and actually fades, and
 * that a golden NORMAL frame survives the impairment where it should
 * and dies where it should. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../src/chanimp.h"
#include "../src/dsp.h"
#include "../src/tx.h"
#include "../src/rx_stream.h"
#include "../src/packets.h"
#include "test_vectors.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int g_pass, g_fail;

static void check(const char *name, int ok)
{
    printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (ok)
        g_pass++;
    else
        g_fail++;
}

static int16_t g_sig[1500000];   /* 125 s: the fading test wants 120 */
static int16_t g_ref[1500000];

static double rms_diff(const int16_t *a, const int16_t *b, int n, int lag)
{
    /* rms of a[i] - b[i - lag] over the overlap */
    double acc = 0.0;
    int i, m = 0;
    for (i = lag; i < n; i++) {
        double d = (double)a[i] - (double)b[i - lag];
        acc += d * d;
        m++;
    }
    return sqrt(acc / (m ? m : 1));
}

static double rms_of(const int16_t *a, int n)
{
    double acc = 0.0;
    int i;
    for (i = 0; i < n; i++)
        acc += (double)a[i] * a[i];
    return sqrt(acc / n);
}

static void sine(int16_t *x, int n, double amp, double hz)
{
    int i;
    for (i = 0; i < n; i++)
        x[i] = (int16_t)lrint(amp * sin(2 * M_PI * hz * i / 12000.0));
}

/* decode one impaired NORMAL frame with the streaming receiver */
static int decodes(const int16_t *s, int n, const uint8_t *pkt, int pkt_n)
{
    rxs_t *r = rxs_open(MODE_NORMAL, 0);
    rxs_event_t ev;
    int pos = 0, got = 0;
    while (pos < n) {
        int c = 160;
        if (c > n - pos)
            c = n - pos;
        got = rxs_push(r, s + pos, c, &ev);
        pos += c;
        if (got && ev.type == 1)
            break;
    }
    if (!(got && ev.type == 1))
        got = rxs_flush(r, &ev);
    return got && ev.type == 1 && ev.pkt_bits_n == pkt_n
           && memcmp(ev.bits, pkt, (size_t)pkt_n) == 0;
}

int main(void)
{
    chanimp_t c;
    int n, i;

    /* ---- the Gaussian: rms 65536, bounded, roughly symmetric ---- */
    {
        uint32_t rng = 12345;
        double acc = 0.0, sum = 0.0;
        int32_t mx = 0;
        int tails = 0;
        for (i = 0; i < 200000; i++) {
            int32_t g = chanimp_gauss(&rng);
            acc += (double)g * g;
            sum += g;
            if (g > mx) mx = g;
            if (-g > mx) mx = -g;
            if (g > 3 * 65536 || g < -3 * 65536)
                tails++;
        }
        acc = sqrt(acc / 200000.0);
        check("gauss: rms within 2 % of 65536",
              fabs(acc / 65536.0 - 1.0) < 0.02);
        check("gauss: mean within 1 % of rms", fabs(sum / 200000.0) < 655.0);
        /* P(|g| > 3 sigma) = 0.27 % for a true Gaussian; Irwin-Hall(12)
         * is a little light in the tails: accept 0.15..0.35 % */
        check("gauss: 3-sigma tail between 0.15 and 0.35 %",
              tails > 300 && tails < 700);
        check("gauss: never beyond 6 sigma", mx <= 6 * 65536);
    }

    /* ---- AWGN at +10 dB on a 7071-rms sine: measured SNR, no delay ---- */
    {
        n = 48000;
        sine(g_ref, n, 10000.0, 1000.0);
        memcpy(g_sig, g_ref, (size_t)n * sizeof(int16_t));
        chanimp_init(&c, 7);
        chanimp_set_snr(&c, 10);
        chanimp_start(&c);
        chanimp_apply(&c, g_sig, n);
        {
            double nz = rms_diff(g_sig, g_ref, n, 0);
            double snr = 20.0 * log10(rms_of(g_ref, n) / nz);
            printf("  awgn: asked 10 dB, measured %.2f dB, sat %u\n", snr, c.sat);
            check("awgn: measured SNR within 0.5 dB of the setting",
                  fabs(snr - 10.0) < 0.5);
            check("awgn: no saturation at +10 dB on a half-scale sine", c.sat == 0);
            check("awgn: no delay (noise is the only difference)",
                  rms_diff(g_sig, g_ref, n, 0) < rms_diff(g_sig, g_ref, n, 1));
        }
        /* -20 dB on a full-scale sine: saturates, stays int16, counts */
        sine(g_ref, n, 32000.0, 1000.0);
        memcpy(g_sig, g_ref, (size_t)n * sizeof(int16_t));
        chanimp_set_snr(&c, -20);
        chanimp_start(&c);
        chanimp_apply(&c, g_sig, n);
        {
            int hits = 0;
            for (i = 0; i < n; i++)
                if (g_sig[i] == 32767 || g_sig[i] == -32768)
                    hits++;
            printf("  awgn -20 dB full scale: %u clamps, %d rail samples\n", c.sat, hits);
            /* a sample may land ON the rail without clamping */
            check("awgn: -20 dB on full scale clamps (counted) instead of wrapping",
                  c.sat > 1000 && hits >= (int)c.sat && hits - (int)c.sat < 10);
        }
        chanimp_set_snr(&c, CHANIMP_OFF);
        check("awgn: CHANIMP_OFF is inactive", !chanimp_active(&c));
    }

    /* ---- fading: unit mean power, real fades, the right delay ---- */
    {
        double p_in, p_out, p_min = 1e30, p_blk;
        int blocks = 0, deep = 0;
        n = 12000 * 120;    /* 120 s: ~120 independent fades at 1 Hz */
        if (n > (int)(sizeof(g_sig) / sizeof(g_sig[0])))
            n = (int)(sizeof(g_sig) / sizeof(g_sig[0]));
        chanimp_init(&c, 99);
        chanimp_set_fade(&c, 100);      /* 1 Hz */
        chanimp_set_delay(&c, 10);      /* 1 ms -> two taps */
        check("fade: 1 ms delay is 12 samples, two taps",
              c.delay == 12 && c.paths == 2);
        chanimp_start(&c);
        {
            /* stream it in 1000-sample chunks like the FIFO fill does */
            int pos = 0;
            sine(g_ref, n, 8000.0, 1500.0);
            memcpy(g_sig, g_ref, (size_t)n * sizeof(int16_t));
            while (pos < n) {
                int ch = n - pos < 1000 ? n - pos : 1000;
                chanimp_apply(&c, g_sig + pos, ch);
                pos += ch;
            }
        }
        p_in = rms_of(g_ref, n);
        p_out = rms_of(g_sig + HILBERT_DELAY, n - HILBERT_DELAY);
        /* 30 ms blocks: a fade at 1 Hz Doppler lasts ~100 ms, and a
         * 250 ms average would hide it under its neighbours */
        for (i = 0; i + 360 <= n; i += 360) {
            p_blk = rms_of(g_sig + i, 360);
            if (p_blk < p_min)
                p_min = p_blk;
            if (p_blk < 0.3 * 0.5 * p_in)
                deep++;
            blocks++;
        }
        p_in *= 0.5;    /* the fading path runs 6 dB down (CH_FADE_SHIFT) */
        printf("  fade: in rms/2 %.0f, out rms %.0f (%.2f), deepest 30 ms block %.2f, "
               "%d/%d blocks under -10 dB, sat %u\n",
               p_in, p_out, p_out / p_in, p_min / p_in, deep, blocks, c.sat);
        check("fade: mean power conserved (at the 6 dB headroom) within 25 % over 120 s",
              p_out / p_in > 0.75 && p_out / p_in < 1.25);
        check("fade: it fades (30 ms blocks under -10 dB exist)", deep > 0);
        check("fade: and it is not stuck (deepest block below 0.5x)", p_min / p_in < 0.5);
        /* a flat single tap at a tiny Doppler is nearly a constant
         * complex gain: the output is the input through the Hilbert
         * delay, scaled and phase-shifted -- check the delay by
         * correlation against the reference */
        chanimp_init(&c, 5);
        chanimp_set_fade(&c, 1);
        chanimp_set_delay(&c, 0);
        chanimp_start(&c);
        n = 24000;
        sine(g_ref, n, 8000.0, 1500.0);
        memcpy(g_sig, g_ref, (size_t)n * sizeof(int16_t));
        chanimp_apply(&c, g_sig, n);
        {
            /* the envelope of a 1500 Hz tone through a complex gain is
             * constant: compare block rms at the start and the end */
            double a = rms_of(g_sig + 100, 2000), b = rms_of(g_sig + 20000, 2000);
            printf("  slow flat tap: rms %.0f at start, %.0f at end\n", a, b);
            check("fade: a 0.01 Hz single tap is steady over 2 s",
                  fabs(a - b) / (a + b + 1.0) < 0.05);
        }
        check("fade: tail to flush is the Hilbert delay plus the path delay",
              chanimp_tail(&c) == HILBERT_DELAY);
        {
            int16_t tl[64];
            int m = chanimp_flush(&c, tl, 64);
            check("fade: flush emits exactly the tail and then nothing",
                  m == HILBERT_DELAY && chanimp_flush(&c, tl, 64) == 0);
        }
    }

    /* ---- a real frame: decodes through the channel, dies in the noise ---- */
    {
        int pkt_n = (int)sizeof(TX_NORM_BPSK_PKT);
        int lead = 2000, m, pos;
        n = tx_build_frame((link_mode_t)TX_NORM_BPSK_MODE, TX_NORM_BPSK_PKT, pkt_n,
                           PKT_TYP_DATA, (mod_type_t)TX_NORM_BPSK_MOD,
                           (cc_rate_t)TX_NORM_BPSK_SPD, g_ref + lead);
        memset(g_ref, 0, (size_t)lead * sizeof(int16_t));
        memset(g_ref + lead + n, 0, 2000 * sizeof(int16_t));
        check("frame: clean golden frame decodes", decodes(g_ref, lead + n + 2000, TX_NORM_BPSK_PKT, pkt_n));

        /* +20 dB, CCIR "moderate" fading (1 ms, 0.5 Hz), flushed tail */
        memcpy(g_sig, g_ref, (size_t)(lead + n + 2000) * sizeof(int16_t));
        chanimp_init(&c, 31);
        chanimp_set_snr(&c, 20);
        chanimp_set_fade(&c, 50);
        chanimp_set_delay(&c, 10);
        chanimp_start(&c);
        for (pos = 0; pos < lead + n; pos += 1000) {
            int ch = lead + n - pos < 1000 ? lead + n - pos : 1000;
            chanimp_apply(&c, g_sig + pos, ch);
        }
        m = chanimp_flush(&c, g_sig + lead + n, 2000);
        printf("  frame: +20 dB moderate fading, flushed %d, sat %u\n", m, c.sat);
        check("frame: decodes at +20 dB through moderate fading",
              decodes(g_sig, lead + n + 2000, TX_NORM_BPSK_PKT, pkt_n));
        check("frame: the flush is the delay line (31 + 12 samples)", m == HILBERT_DELAY + 12);

        /* -25 dB AWGN: nothing to decode */
        memcpy(g_sig, g_ref, (size_t)(lead + n + 2000) * sizeof(int16_t));
        chanimp_init(&c, 32);
        chanimp_set_snr(&c, -25);
        chanimp_start(&c);
        chanimp_apply(&c, g_sig, lead + n + 2000);
        check("frame: does not decode at -25 dB",
              !decodes(g_sig, lead + n + 2000, TX_NORM_BPSK_PKT, pkt_n));
    }

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
