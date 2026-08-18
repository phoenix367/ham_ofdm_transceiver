/* Measurement harness (plan §6): host timing of every pipeline stage plus
 * whole-session streaming real-time factors, and analytic MAC counts per
 * stage. Host wall time is a proxy -- the FEASIBILITY verdicts scale the
 * analytic MAC numbers to Cortex-M DSP throughput; this harness's job is
 * to confirm the hotspot ranking and catch anything the analysis missed. */
#define _POSIX_C_SOURCE 199309L /* clock_gettime under -std=c99 */

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "../src/packets.h"
#include "../src/tx.h"
#include "../src/rx_demod.h"
#include "../src/rx_detect.h"
#include "../src/rx_stream.h"
#include "../src/rx_internal.h"
#include "../src/conv.h"
#include "../src/ldpc.h"
#include "../src/dsp.h"

#define FS 12000.0

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

static uint32_t g_lcg = 987654321u;

static int16_t noise_sample(int amp)
{
    g_lcg = g_lcg * 1103515245u + 12345u;
    return (int16_t)((int)((g_lcg >> 16) % (uint32_t)(2 * amp)) - amp);
}

static int16_t g_sess[900000];
static uint8_t g_pkt[512];
static int64_t g_i64a[600000], g_i64b[600000];

static const char *MODE_NAMES[3] = { "NORMAL", "ROBUST", "EXTREME" };

/* one mode's scenario: idle-search load + framed-session real-time factor */
static void bench_mode(link_mode_t mode)
{
    int pkt_n, frame_n, sess_n, k, got = 0;
    double t0, idle_s, sess_s, audio_s;
    rxs_event_t ev;
    rxs_t *r;

    /* 1. idle search on noise, 30 s of audio */
    r = rxs_open(mode, 0);
    for (k = 0; k < 360000; k++)
        g_sess[k] = noise_sample(900);
    t0 = now_s();
    rxs_push(r, g_sess, 360000, &ev);
    idle_s = now_s() - t0;

    /* 2. noise + frame + noise session */
    pkt_n = data_encode(123, (const uint8_t *)"BENCH PAYLOAD 27 BYTES LONG",
                        27, g_pkt);
    frame_n = tx_build_frame(mode, g_pkt, pkt_n, PKT_TYP_DATA, MOD_BPSK,
                             CC_R13, g_sess + 6000);
    for (k = 0; k < 6000; k++)
        g_sess[k] = noise_sample(40);
    for (k = 0; k < 6000; k++)
        g_sess[6000 + frame_n + k] = noise_sample(40);
    sess_n = 12000 + frame_n;
    audio_s = (double)sess_n / FS;

    r = rxs_open(mode, 0);
    t0 = now_s();
    {
        int pos = 0;
        while (pos < sess_n) {
            int c = sess_n - pos > 4096 ? 4096 : sess_n - pos;
            got = rxs_push(r, g_sess + pos, c, &ev);
            pos += c;
            if (got && ev.type == 1)
                break; /* negative events are acquisition retries */
        }
        if (!(got && ev.type == 1))
            got = rxs_flush(r, &ev);
    }
    sess_s = now_s() - t0;

    printf("%-8s idle-search %7.1f ms/audio-s   framed session %7.1f ms/audio-s"
           "  (frame %4.1f s, decoded=%s)\n",
           MODE_NAMES[mode], idle_s / 30.0 * 1e3, sess_s / audio_s * 1e3,
           (double)frame_n / FS,
           (got == 1 && ev.type == 1
            && memcmp(ev.bits, g_pkt, (size_t)pkt_n) == 0) ? "yes" : "NO");
    if (!(got == 1 && ev.type == 1))
        printf("         (debug: got=%d ev.type=%d start=%d)\n", got, ev.type,
               ev.start_abs);
}

/* micro-benches of the burst stages */
static void bench_stages(void)
{
    int pkt_n, frame_n, k, reps;
    double t0, dt;
    rxd_t rd;
    static int64_t llr[64];

    pkt_n = data_encode(7, (const uint8_t *)"STAGE", 5, g_pkt);
    frame_n = tx_build_frame(MODE_EXTREME, g_pkt, pkt_n, PKT_TYP_DATA,
                             MOD_BPSK, CC_R13, g_sess + 700);
    memset(g_sess, 0, 700 * sizeof(int16_t));

    /* hilbert throughput */
    t0 = now_s();
    for (reps = 0; reps < 20; reps++)
        hilbert_analytic(g_sess, 12000, g_i64a, g_i64b);
    dt = now_s() - t0;
    printf("hilbert           %7.2f Msamp/s (%.1f ms per audio-s)\n",
           20.0 * 12000.0 / dt / 1e6, dt / 20.0 * 1e3);

    /* full acquisition burst (tone over capture + ZC), frame-at-once */
    hilbert_analytic(g_sess, 700 + frame_n, g_i64a, g_i64b);
    {
        int start;
        int64_t cfo;
        t0 = now_s();
        rx_detect(MODE_EXTREME, g_i64a, g_i64b, 700 + frame_n, &start, &cfo);
        dt = now_s() - t0;
        printf("acquisition burst %7.1f ms (EXTREME frame-at-once: tone+ZC)\n",
               dt * 1e3);

        /* first-symbol frequency search: gated vs full grid */
        rxd_init(&rd, MODE_EXTREME);
        rd.last_hyp = -1;
        t0 = now_s();
        rxd_demod_symbol(&rd, g_i64a + start, g_i64b + start, start, cfo, 1,
                         0, 0, llr);
        dt = now_s() - t0;
        printf("first symbol      %7.1f ms (gated coarse/fine, EXTREME)\n",
               dt * 1e3);

        rd.coarse_enabled = 0;
        rd.last_hyp = -1;
        t0 = now_s();
        rxd_demod_symbol(&rd, g_i64a + start, g_i64b + start, start, cfo, 1,
                         0, 0, llr);
        dt = now_s() - t0;
        printf("first symbol      %7.1f ms (full 275-hyp grid, EXTREME)\n",
               dt * 1e3);

        /* tracked symbol (5 hypotheses) */
        {
            int win[5] = { 135, 136, 137, 138, 139 };
            t0 = now_s();
            for (reps = 0; reps < 10; reps++)
                rxd_demod_symbol(&rd, g_i64a + start, g_i64b + start, start,
                                 cfo, 1, win, 5, llr);
            dt = (now_s() - t0) / 10.0;
            printf("tracked symbol    %7.2f ms (5 hyps, EXTREME, per 685 ms)\n",
                   dt * 1e3);
        }
    }

    /* decoders, worst-case block sizes */
    {
        static int64_t soft[1024];
        static uint8_t out[256], work[64 * 272];
        int n = conv_cc_elements(CC_R13, 255);
        for (k = 0; k < n; k++)
            soft[k] = (int64_t)(noise_sample(31));
        t0 = now_s();
        for (reps = 0; reps < 50; reps++)
            conv_decode(CC_R13, soft, n, 255, out, work);
        dt = (now_s() - t0) / 50.0;
        printf("viterbi 255 bits  %7.2f ms\n", dt * 1e3);

        n = ldpc_cc_elements(255);
        for (k = 0; k < n; k++)
            soft[k] = (int64_t)(noise_sample(31));
        t0 = now_s();
        for (reps = 0; reps < 10; reps++)
            ldpc_decode_int(soft, n, 255, out);
        dt = (now_s() - t0) / 10.0;
        printf("ldpc random LLRs  %7.2f ms (non-converging, 60 iters)\n",
               dt * 1e3);
    }

    /* TX, worst case */
    t0 = now_s();
    tx_build_frame(MODE_EXTREME, g_pkt, pkt_n, PKT_TYP_DATA, MOD_BPSK,
                   CC_R13, g_sess);
    dt = now_s() - t0;
    printf("tx EXTREME frame  %7.1f ms (%.1f s of audio)\n", dt * 1e3,
           (double)tx_frame_len(MODE_EXTREME, pkt_n, MOD_BPSK, CC_R13) / FS);
}

static void print_macs(void)
{
    printf("\nanalytic MAC budget (from dimensions, real-time at 12 kHz):\n");
    printf("  hilbert         0.4 MMAC/s continuous (63-tap, ~32 nonzero)\n");
    printf("  tone detection  <=1 MMAC/s continuous (FFT(B)/block + dots)\n");
    printf("  ZC burst EXTREME ~230 MMAC over 5.8 s preamble (~40 MMAC/s)\n");
    printf("  first symbol    9 MMAC full grid / ~1.6 MMAC gated (EXTREME)\n");
    printf("  tracked symbol  ~0.9 MMAC per 685 ms (EXTREME)\n");
}

int main(void)
{
    printf("host: single core, wall time (see FEASIBILITY.md for the\n"
           "Cortex-M scaling of the analytic MAC numbers)\n\n");
    bench_mode(MODE_NORMAL);
    bench_mode(MODE_ROBUST);
    bench_mode(MODE_EXTREME);
    printf("\n");
    bench_stages();
    print_macs();
    return 0;
}
