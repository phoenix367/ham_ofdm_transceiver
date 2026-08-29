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

/* --- the ZC scan, in each of the three memories it could live in ------
 *
 * This is the case that decides the port: EXTREME acquisition is a
 * sliding correlation whose working set (the slide window) is far larger
 * than the M7's 16 kB D-cache, so where the window lives sets the cost.
 * The three arrays below are identical in size and content and differ
 * only in placement -- DTCM (zero wait state), AXI-SRAM (D1, cached),
 * and D2 SRAM (further out on the AHB matrix, and where `g_raw` actually
 * sits today). */
#define WIN_N   8192            /* samples; 2 arrays x 4 B = 64 kB */
#define KLEN     512            /* ZC correlation kernel, ZC_G * FFT_BINS */
#define N_OFF   4096            /* offsets swept; touches ~37 kB > 16 kB */

#define DTCM_BSS __attribute__((section(".dtcm_bss")))
#define D2_BSS   __attribute__((section(".d2_bss")))

static samp_t wi_dtcm[WIN_N] DTCM_BSS, wq_dtcm[WIN_N] DTCM_BSS;
static samp_t kr_k[KLEN] DTCM_BSS, ki_k[KLEN] DTCM_BSS;  /* kernel: always hot */
static samp_t wi_axi[WIN_N], wq_axi[WIN_N];
static samp_t wi_d2[WIN_N] D2_BSS, wq_d2[WIN_N] D2_BSS;

/* Volatile sink. Without it gcc infers zc_correlate() is pure, sees its
 * result is never read, and deletes all three calls -- which measured as
 * exactly 0 cycles, the one result that is obviously a bug rather than a
 * surprise. */
static volatile int64_t g_sink;

/* forward: assigned inside the correlator, see the note there */

/* noinline and pointer-taking, so all three cases execute the SAME
 * instructions and only the addresses differ */
static int64_t __attribute__((noinline))
zc_correlate(const samp_t *wi, const samp_t *wq,
             const samp_t *kr, const samp_t *ki, int n_off, int klen)
{
    int64_t acc = 0;
    int m, k;

    for (m = 0; m < n_off; m++) {
        int64_t rr = 0, ri = 0;
        for (k = 0; k < klen; k++) {
            rr += (int64_t)wi[m + k] * kr[k] + (int64_t)wq[m + k] * ki[k];
            ri += (int64_t)wq[m + k] * kr[k] - (int64_t)wi[m + k] * ki[k];
        }
        rr >>= 16;
        ri >>= 16;
        acc += rr * rr + ri * ri;
    }
    /* The volatile store is what keeps this function honest. Marked pure
     * (no side effects, result written to a dead array) gcc first deleted
     * the calls outright, and then -- once a volatile sink was assigned
     * OUTSIDE -- hoisted them clear of the two volatile CYCCNT reads, so
     * both attempts measured ~0 cycles. A side effect INSIDE the timed
     * region is what pins it there; a "memory" clobber does not, because
     * a pure call has no memory effects to order against. */
    g_sink = acc;
    return acc;
}

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
    /* ---- the decisive case: identical correlation, three memories ---- */
    for (i = 0; i < WIN_N; i++) {
        samp_t vi = noise(20000), vq = noise(20000);
        wi_dtcm[i] = vi;  wq_dtcm[i] = vq;
        wi_axi[i]  = vi;  wq_axi[i]  = vq;
        wi_d2[i]   = vi;  wq_d2[i]   = vq;
    }
    for (i = 0; i < KLEN; i++) {
        kr_k[i] = noise(20000);
        ki_k[i] = noise(20000);
    }
    dcache_set(1);   /* do not inherit the previous image's cache state */
    {
        const uint32_t macs = (uint32_t)N_OFF * KLEN * 4;
        MEASURE(2, macs, "ZC corr, window in DTCM", {
            g_sink = zc_correlate(wi_dtcm, wq_dtcm, kr_k, ki_k, N_OFF, KLEN);
        });
        MEASURE(2, macs, "ZC corr, window in AXI", {
            g_sink = zc_correlate(wi_axi, wq_axi, kr_k, ki_k, N_OFF, KLEN);
        });
        MEASURE(2, macs, "ZC corr, window in D2", {
            g_sink = zc_correlate(wi_d2, wq_d2, kr_k, ki_k, N_OFF, KLEN);
        });

        /* Same three, with the D-cache explicitly OFF. The inner loop
         * re-reads 512 samples per offset and advances by one, so the
         * reuse is ~99.8 % -- if the cache were doing anything, these
         * should be far worse than the three above. */
        dcache_set(0);
        MEASURE(2, macs, "ZC DTCM, DC off", {
            g_sink = zc_correlate(wi_dtcm, wq_dtcm, kr_k, ki_k, N_OFF, KLEN);
        });
        MEASURE(2, macs, "ZC AXI, DC off", {
            g_sink = zc_correlate(wi_axi, wq_axi, kr_k, ki_k, N_OFF, KLEN);
        });
        MEASURE(2, macs, "ZC D2, DC off", {
            g_sink = zc_correlate(wi_d2, wq_d2, kr_k, ki_k, N_OFF, KLEN);
        });
        dcache_set(1);

        /* The AXI case above is slower than D2 for a reason that has
         * nothing to do with the H743: the firmware resident on this
         * board leaves an MPU region covering AXI-SRAM (base 0x24000000,
         * 512 kB) with TEX=0 C=1 B=0 S=1 -- Normal, write-through, and
         * SHAREABLE. The Cortex-M7 has no cache coherency unit, so a
         * shareable Normal region is effectively uncached, which is
         * exactly what "the D-cache wins 1.00x on AXI" measured.
         *
         * We inherit that config because the core is never reset. Turn
         * the MPU off and the default map applies (Normal, write-back,
         * write-allocate, non-shareable), which is what a deployment
         * would configure deliberately. */
        out->mpu_ctrl_seen = MPU_CTRL;
        MPU_CTRL = 0;
        __asm__ volatile("dsb; isb");
        dcache_set(1);
        MEASURE(2, macs, "ZC AXI, MPU off", {
            g_sink = zc_correlate(wi_axi, wq_axi, kr_k, ki_k, N_OFF, KLEN);
        });
        MEASURE(2, macs, "ZC D2, MPU off", {
            g_sink = zc_correlate(wi_d2, wq_d2, kr_k, ki_k, N_OFF, KLEN);
        });
    }

    out->cpu_hz_hint = SCB_CCR;   /* bit16 = DC, bit17 = IC, as measured */
    out->magic = BENCH_MAGIC;   /* written last: the completion flag */

    __asm__ volatile("bkpt #0");
    for (;;)
        ;
}
