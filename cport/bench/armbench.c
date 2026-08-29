/* On-target cycle counts for the DSP primitives, via the Cortex-M7's
 * DWT cycle counter.
 *
 * FEASIBILITY.md's Cortex-M projections scale ANALYTIC MAC counts
 * against an assumed DSP throughput ("SMLAD-class, x2 overhead margin:
 * M7 @480 MHz ~= 400 MMAC/s"). That assumption is the weakest link in
 * the whole document. This measures the primitives instead.
 *
 * Runs entirely from RAM -- OpenOCD loads it, points the core at
 * _start, and reads the results back out. The target's own flash is
 * never touched, so the board keeps whatever firmware it had.
 *
 * No printf: results land in a struct at a fixed DTCM address, which
 * OpenOCD dumps afterwards. That keeps the image small enough to live
 * in ITCM, which is where the real firmware would run from anyway --
 * measuring a bus instead of an algorithm would defeat the point.
 *
 * Each case reports the MINIMUM over its repetitions. The minimum is
 * the honest estimator here: interrupts and refills can only ever add
 * cycles, never remove them.
 */

#include <stdint.h>
#include <string.h>

#include "fft.h"
#include "dsp.h"
#include "conv.h"
#include "ldpc.h"
#include "rom_modes.h"
#include "packets.h"
#include "tx.h"
#include "rx_stream.h"

/* --- Cortex-M7 debug/trace registers ---------------------------------- */
#define DEMCR    (*(volatile uint32_t *)0xE000EDFCu)
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYC  (*(volatile uint32_t *)0xE0001004u)
#define DEMCR_TRCENA (1u << 24)
#define DWT_CYCCNTENA 1u

#define SCB_CCR    (*(volatile uint32_t *)0xE000ED14u)
#define SCB_CSSELR (*(volatile uint32_t *)0xE000ED84u)
#define SCB_CCSIDR (*(volatile uint32_t *)0xE000ED80u)
#define SCB_DCISW  (*(volatile uint32_t *)0xE000EF60u)
#define SCB_DCCISW (*(volatile uint32_t *)0xE000EF74u)
#define CCR_DC (1u << 16)
#define MPU_CTRL (*(volatile uint32_t *)0xE000ED94u)

/* The core is NOT reset between bench runs -- OpenOCD halts, loads, sets
 * PC and resumes -- so CCR survives from whatever ran before. A
 * build-time "cache off" variant therefore proves nothing: it inherits
 * the previous image's enabled cache. Toggle it here instead, inside one
 * image, and record CCR so the state is visible in the results. */
static void dcache_set(int on)
{
    uint32_t ccsidr, sets, ways, sw, w;

    __asm__ volatile("dsb");
    SCB_CSSELR = 0;
    __asm__ volatile("dsb");
    ccsidr = SCB_CCSIDR;
    sets = (ccsidr >> 13) & 0x7FFFu;
    ways = (ccsidr >> 3) & 0x3FFu;
    for (sw = 0; sw <= sets; sw++)
        for (w = 0; w <= ways; w++)
            SCB_DCCISW = (w << 30) | (sw << 5);   /* clean + invalidate */
    __asm__ volatile("dsb");
    if (on)
        SCB_CCR |= CCR_DC;
    else
        SCB_CCR &= ~CCR_DC;
    __asm__ volatile("dsb; isb");
}

#define CASE_NAME_LEN 28
#define MAX_CASES 20

typedef struct {
    char name[CASE_NAME_LEN];
    uint32_t cycles;   /* minimum over reps, overhead subtracted */
    uint32_t units;    /* samples/bits/blocks the case covers */
} bench_case_t;

/* Read back by OpenOCD. `magic` last-written is the completion flag. */
typedef struct {
    uint32_t magic;
    uint32_t n_cases;
    uint32_t overhead;   /* cycles for an empty measurement */
    uint32_t cpu_hz_hint;
    uint32_t mpu_ctrl_seen;
    bench_case_t c[MAX_CASES];
} bench_out_t;

#define BENCH_MAGIC 0xB6C51CE5u

/* Fixed address so the host can find it without parsing the ELF. DTCM,
 * which is neither cached nor buffered -- so what the core wrote is what
 * the debugger reads, with no cache maintenance in between. */
bench_out_t g_out __attribute__((section(".results"), used));

static bench_out_t *const out = &g_out;

static inline uint32_t cyc(void) { return DWT_CYC; }

static void record(const char *name, uint32_t cycles, uint32_t units)
{
    bench_case_t *c;
    int i;
    if (out->n_cases >= MAX_CASES)
        return;
    c = &out->c[out->n_cases++];
    for (i = 0; i < CASE_NAME_LEN - 1 && name[i]; i++)
        c->name[i] = name[i];
    c->name[i] = 0;
    c->cycles = cycles;
    c->units = units;
}

/* --- working buffers. Sized for the worst case each primitive sees --- */
static int64_t f_re[FFT_BINS], f_im[FFT_BINS];
static int16_t raw[4096];
static samp_t ai[4096], aq[4096], bi[4096], bq[4096];
static llr_t soft[2048];
static uint8_t bits_in[256], bits_out[256];
static uint8_t vwork[CONV_STATES / 8 * CONV_MAX_STEPS_PUB];
static uint8_t coded[2048];

/* The three-memory ZC A/B that used to live here is done and recorded
 * (FEASIBILITY.md, "Where the ZC window lives does NOT matter"): all
 * three SRAMs measured an identical 3.27 cyc/MAC once the inherited MPU
 * was disabled. Its 192 kB of windows is now needed by the real
 * streaming receiver below, so it has been removed rather than kept as
 * a monument. */

/* deterministic filler -- an LCG, so every run measures the same data */
static uint32_t lcg = 22695477u;
static int32_t noise(int32_t amp)
{
    lcg = lcg * 1103515245u + 12345u;
    return (int32_t)((lcg >> 16) % (uint32_t)(2 * amp)) - amp;
}

/* --- the real thing: a whole EXTREME frame through the streaming
 * receiver, on target -------------------------------------------------
 *
 * Everything above is a primitive. This is the stage FEASIBILITY.md's
 * projections are actually about, and it is measured by piping the
 * streaming TRANSMITTER straight into the streaming RECEIVER: txs_pull
 * generates a chunk (untimed), rxs_push consumes it (timed). Nothing
 * holds a frame -- at EXTREME that would be ~950 kB of int16 -- which is
 * the same reason the port streams in the first place.
 *
 * Cycles are split at the preamble boundary: everything the receiver
 * spends before PREAMBLE_LEN_EXTREME is acquisition (tone search, ZC
 * lock, CFO), everything after is header and data demodulation. */

#define RX_CHUNK 512

#define ZC_COMMIT_ABS 131072   /* just past the measured 124478 lookback */
static uint64_t g_tone_cyc, g_acq_cyc, g_demod_cyc;
static uint32_t g_tone_samp, g_acq_samp, g_demod_samp;
static int g_ev_type, g_ev_seen, g_ev_n;
static uint32_t g_ev_at, g_hwm, g_miss, g_total;

static void bench_frame(void)
{
    static uint8_t pkt[280];
    uint8_t payload[27];
    int pkt_n, total = 0, got, i;
    int64_t abs_n = 0;
    rxs_t *r;
    txs_t *t;
    static int16_t chunk[RX_CHUNK];
    static uint8_t save[16384];
    const void *blob;
    int blob_n = 0;
    rxs_event_t ev;

    for (i = 0; i < 27; i++)
        payload[i] = (uint8_t)(i * 7 + 3);
    pkt_n = data_encode(123, payload, 27, pkt);

    r = rxs_open(MODE_EXTREME, 0);
    t = txs_open(MODE_EXTREME, pkt, pkt_n, 1, PKT_TYP_DATA,
                 MOD_BPSK, CC_R13, 0, 0, &total);
    if (!r || !t)
        return;

    /* The generator lives in the arena the receiver is about to scribble
     * on, so its state is lifted out and put back around each push. This
     * is untimed, and it is why txs_state_blob() exists (tx.h). Without
     * it the very first rxs_push claims the arena and the next txs_pull
     * correctly refuses to continue -- which is exactly what the guard
     * is for, and what this bench first ran into. */
    blob = txs_state_blob(&blob_n);
    while ((got = txs_pull(t, chunk, RX_CHUNK)) > 0) {
        uint32_t t0, d;
        int hit;

        memcpy(save, blob, (size_t)blob_n);
        t0 = cyc();
        hit = rxs_push(r, chunk, got, &ev);
        d = cyc() - t0;
        txs_state_restore(save, blob_n);
        /* Three phases, split where the receiver actually changes job.
         * NOT at the preamble boundary: the tone detector takes the
         * argmax over the whole above-threshold region, so it cannot
         * commit until roughly TWO tone fields in (measured ring
         * lookback 124478, which is exactly that). Until then it is
         * folding 512-sample blocks into summaries; after it, it runs
         * the ZC scan and then demodulates symbols -- reading back
         * through g_raw, which is why the ring has to be that deep. */
        if (abs_n < PREAMBLE_LEN_EXTREME) {
            g_tone_cyc += d;  g_tone_samp += (uint32_t)got;
        } else if (abs_n < ZC_COMMIT_ABS) {
            g_acq_cyc += d;   g_acq_samp += (uint32_t)got;
        } else {
            g_demod_cyc += d; g_demod_samp += (uint32_t)got;
        }
        if (hit) {
            g_ev_n++;
            if (!g_ev_seen) {
                g_ev_seen = 1;
                g_ev_type = ev.type;
                g_ev_at = (uint32_t)abs_n;
            }
        }
        abs_n += got;
    }
    if (rxs_flush(r, &ev) && !g_ev_seen) {
        g_ev_seen = 1;
        g_ev_type = ev.type;
        g_ev_at = (uint32_t)abs_n;
    }
    g_hwm = (uint32_t)rxs_ring_hwm(r);
    g_miss = (uint32_t)rxs_ring_miss(r);
    g_total = (uint32_t)total;
}

int main(void)
{
    uint32_t t0, best, k, rep;
    int i, exp;

    /* D2 SRAM (SRAM1/2/3) is clock-gated and resets to OFF: without this
     * every access to 0x30000000 faults, which reads as "the memory is
     * not there". RCC_AHB2ENR bits 31:29. */
    *(volatile uint32_t *)0x580244DCu |= (7u << 29);
    (void)*(volatile uint32_t *)0x580244DCu;   /* ensure the write landed */

    memset(out, 0, sizeof(*out));
    DEMCR |= DEMCR_TRCENA;
    DWT_CYC = 0;
    DWT_CTRL |= DWT_CYCCNTENA;

    /* measurement overhead: two register reads with nothing between */
    best = 0xFFFFFFFFu;
    for (rep = 0; rep < 64; rep++) {
        t0 = cyc();
        k = cyc() - t0;
        if (k < best)
            best = k;
    }
    out->overhead = best;

/* variadic: a measured body may contain commas at brace depth 0 */
#define MEASURE(reps, units, label, ...)                                    \
    do {                                                                    \
        uint32_t _b = 0xFFFFFFFFu, _r, _t, _d;                              \
        for (_r = 0; _r < (uint32_t)(reps); _r++) {                         \
            _t = cyc();                                                     \
            __VA_ARGS__;                                                    \
            _d = cyc() - _t;                                                \
            if (_d < _b) _b = _d;                                           \
        }                                                                   \
        record(label, _b - out->overhead, (uint32_t)(units));               \
    } while (0)

    for (i = 0; i < 4096; i++) {
        raw[i] = (int16_t)noise(12000);
        ai[i] = noise(30000);
        aq[i] = noise(30000);
    }
    for (i = 0; i < 2048; i++)
        soft[i] = noise(120);
    for (i = 0; i < 256; i++)
        bits_in[i] = (uint8_t)(i & 1);

    /* ---- FFT: the core of demodulation and of the tone detector ---- */
    MEASURE(64, FFT_BINS, "fft_bfp 128", {
        for (i = 0; i < FFT_BINS; i++) { f_re[i] = ai[i]; f_im[i] = aq[i]; }
        fft_bfp(f_re, f_im, FFT_BINS, 30, &exp);
    });
    MEASURE(64, FFT_BINS, "ifft_fixed 128", {
        for (i = 0; i < FFT_BINS; i++) { f_re[i] = ai[i]; f_im[i] = aq[i]; }
        ifft_fixed(f_re, f_im, FFT_BINS);
    });

    /* ---- per-sample front end: runs continuously, so it sets the
     * floor on idle load ---- */
    MEASURE(16, 4096, "hilbert_analytic /samp", {
        hilbert_analytic(raw, 4096, ai, aq);
    });
    MEASURE(64, 4096, "nco_derotate /samp", {
        nco_derotate(ai, aq, 4096, 1234567, 0, bi, bq);
    });

    /* ---- CORDIC: one per lag correlation, not per sample ---- */
    MEASURE(256, 1, "cordic_atan2", {
        int64_t ang, mag;
        cordic_atan2(123456789, 987654321, &ang, &mag);
    });

    /* ---- decoders, worst-case block sizes ---- */
    {
        int n = conv_cc_elements(CC_R13, 255);
        MEASURE(16, 255, "viterbi 255 bits r1/3", {
            conv_decode(CC_R13, soft, n, 255, bits_out, vwork);
        });
    }
    {
        int n = ldpc_cc_elements(128);
        for (i = 0; i < n && i < 2048; i++)
            soft[i] = noise(120);
        MEASURE(8, 128, "ldpc min-sum 128 bits", {
            ldpc_decode_int(soft, n, 128, bits_out);
        });
        MEASURE(16, 128, "ldpc encode 128 bits", {
            ldpc_encode(bits_in, 128, coded);
        });
    }
    MEASURE(64, 255, "conv encode 255 bits r1/3", {
        conv_encode(CC_R13, bits_in, 255, coded);
    });

    /* ---- the ZC acquisition inner loop, which dominates EXTREME.
     * A complex multiply-accumulate over the correlation kernel is what
     * the MMAC budget in FEASIBILITY.md actually counts. ---- */
    /* The board's resident firmware leaves an MPU region over AXI-SRAM
     * marked shareable, which on a Cortex-M7 (no coherency unit) makes
     * it effectively uncached and cost 2.3x. Measured, then removed --
     * see FEASIBILITY.md. Disable it so AXI is cacheable, as a real
     * deployment would configure deliberately. */
    out->mpu_ctrl_seen = MPU_CTRL;
    MPU_CTRL = 0;
    __asm__ volatile("dsb; isb");
    dcache_set(1);

    /* ---- the stage the projections are actually about ---- */
    bench_frame();
    record("tone search (kcyc)", (uint32_t)(g_tone_cyc / 1000), g_tone_samp);
    record("EXTREME acq (kcyc)", (uint32_t)(g_acq_cyc / 1000), g_acq_samp);
    record("EXTREME demod (kcyc)", (uint32_t)(g_demod_cyc / 1000),
           g_demod_samp);
    record("frame ev type+8", (uint32_t)(g_ev_type + 8), (uint32_t)g_ev_seen);
    record("events / first at", (uint32_t)g_ev_n, g_ev_at);
    record("ring hwm / miss", g_hwm, g_miss);
    record("tx frame samples", g_total, 1);

    out->cpu_hz_hint = SCB_CCR;   /* bit16 = DC, bit17 = IC, as measured */
    out->magic = BENCH_MAGIC;   /* written last: the completion flag */

    __asm__ volatile("bkpt #0");
    for (;;)
        ;
}
