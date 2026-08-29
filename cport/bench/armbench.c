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

/* --- Cortex-M7 debug/trace registers ---------------------------------- */
#define DEMCR    (*(volatile uint32_t *)0xE000EDFCu)
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYC  (*(volatile uint32_t *)0xE0001004u)
#define DEMCR_TRCENA (1u << 24)
#define DWT_CYCCNTENA 1u

#define CASE_NAME_LEN 28
#define MAX_CASES 16

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
/* DTCM-resident twins of the correlation operands (see the MAC cases) */
#define DTCM_BSS __attribute__((section(".dtcm_bss")))
static samp_t di[2048] DTCM_BSS, dq[2048] DTCM_BSS;
static samp_t ei[2048] DTCM_BSS, eq[2048] DTCM_BSS;

/* deterministic filler -- an LCG, so every run measures the same data */
static uint32_t lcg = 22695477u;
static int32_t noise(int32_t amp)
{
    lcg = lcg * 1103515245u + 12345u;
    return (int32_t)((lcg >> 16) % (uint32_t)(2 * amp)) - amp;
}

int main(void)
{
    uint32_t t0, best, k, rep;
    int i, exp;

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
    MEASURE(64, 2048, "cplx MAC 2048 AXI", {
        int64_t rr = 0, ri = 0;
        for (i = 0; i < 2048; i++) {
            rr += (int64_t)ai[i] * bi[i] + (int64_t)aq[i] * bq[i];
            ri += (int64_t)ai[i] * bq[i] - (int64_t)aq[i] * bi[i];
        }
        f_re[0] = rr; f_im[0] = ri;
    });

    /* Same loop, operands in DTCM instead of AXI-SRAM. The M7's D-cache
     * is 16 kB and the four AXI arrays above are 64 kB of streamed data,
     * so that case misses on essentially every line. This one is zero
     * wait states with no cache involved, which separates the arithmetic
     * cost from the memory cost -- and the receiver streams buffers far
     * larger than 16 kB, so which of the two dominates decides whether
     * the design is compute- or memory-bound on this part. */
    for (i = 0; i < 2048; i++) {
        di[i] = ai[i]; dq[i] = aq[i];
        ei[i] = bi[i]; eq[i] = bq[i];
    }
    MEASURE(64, 2048, "cplx MAC 2048 DTCM", {
        int64_t rr = 0, ri = 0;
        for (i = 0; i < 2048; i++) {
            rr += (int64_t)di[i] * ei[i] + (int64_t)dq[i] * eq[i];
            ri += (int64_t)di[i] * eq[i] - (int64_t)dq[i] * ei[i];
        }
        f_re[0] = rr; f_im[0] = ri;
    });

    out->cpu_hz_hint = 0;
    out->magic = BENCH_MAGIC;   /* written last: the completion flag */

    __asm__ volatile("bkpt #0");
    for (;;)
        ;
}
