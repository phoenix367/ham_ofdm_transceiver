/* FIR Hilbert transformer, NCO, CORDIC -- C twin of ofdm_phy/fixed/dsp.py.
 * CFO convention: 32-bit phase-increment word (one turn = 2^32). */
#ifndef OFDM_DSP_H
#define OFDM_DSP_H

#include <stdint.h>

/* Analytic sample type.
 *
 * These derive from int16 audio through a unity-gain Hilbert FIR;
 * measured peak across every suite is 43077 (17 bits), so int32 keeps a
 * ~49000x margin at half the storage.
 *
 * Products of TWO samp_t must be widened explicitly -- 43077 squared is
 * 1.9e9 and sits right against INT32_MAX. On Cortex-M7 the widening
 * multiply is SMULL/SMLAL, the same instruction count as MUL/MLA, so
 * being explicit costs nothing. */
#ifndef OFDM_SAMP_T
#define OFDM_SAMP_T
typedef int32_t samp_t;
#endif

#define PHASE_BITS 32
#define NCO_LUT_BITS 12
#define HILBERT_TAPS_N 63
#define HILBERT_DELAY 31

/* analytic signal: I = input delayed by the group delay, Q = FIR output.
 * out_i/out_q are n samples each (edges transient, as in hardware). */
void hilbert_analytic(const int16_t *x, int n, samp_t *out_i, samp_t *out_q);

/* multiply the stream by exp(-j*phase): compensates +word/sample CFO.
 * word is the signed phase increment; start_phase the initial accumulator. */
void nco_derotate(const samp_t *in_i, const samp_t *in_q, int n,
                  int64_t phase_word, uint32_t start_phase,
                  samp_t *out_i, samp_t *out_q);

/* vectoring-mode CORDIC: angle in signed phase-word units, |x + jy| */
void cordic_atan2(int64_t y, int64_t x, int64_t *angle, int64_t *mag);

/* STATIONARY-CARRIER NOTCH (the float model's FullOFDMModem.notch, the
 * fixed model's dsp.Notch -- integer twins, bit-exact). Second-order
 * real IIR, NOTCH_BW_HZ wide, centred on a 32-bit phase word:
 *   y = x - 2c x1 + x2 + 2rc y1 - r^2 y2,  r = 1 - pi*BW/fs
 * Coefficients in Q14 (b1 = -c_q15 is exactly -2c in Q14), states are
 * sample values, the accumulator is int64, the output rounds and
 * saturates to int16 -- the ring stores int16. */
#define NOTCH_MAX 3
#define NOTCH_R_Q14 16255      /* r = 1 - pi*30/12000 = 0.992146 */
#define NOTCH_R2_Q14 16128     /* r^2 */
#define NOTCH_TICK 4096        /* samples per release evaluation: long
                                * enough (~34 degrees of freedom in the
                                * 30-Hz slice) that white noise cannot
                                * trip the 1/32 ratio by fluctuation --
                                * at 256 it did, every few ticks */
#define NOTCH_IDLE_TICKS 4     /* ~1.4 s of the carrier gone -> release */
#define NOTCH_REL_DIV 64       /* released when rejected < input/64 (dsp.c) */
#define NOTCH_MIN_REJ (NOTCH_TICK * 16) /* rejected rms < 4 LSB counts as gone:
                                         * with the input silent the notch's
                                         * own ringing would otherwise hold
                                         * it forever */

/* FREQUENCY TRACKING. The streaming carrier finder estimates a carrier's
 * frequency once, from one tone window of noisy bins -- at EXTREME
 * that left it a few Hz off, the notch attenuated the carrier by only
 * ~10 dB and the arm stayed at 35-63 % PER from ISR 0 dB up, where the
 * float model, estimating over the whole recording, decodes. The
 * rejected component of a notch, in - out, IS the carrier (attenuated
 * only by the mismatch), so each active notch mixes it down with an
 * NCO at its own frequency and reads the residual rotation: the phase
 * of the mixed sum over NOTCH_SUB samples, unwrapped from one sub-block
 * to the next (+-half a turn per 256 samples = +-23 Hz, a full
 * detection bin), summed over the tick and divided by the samples,
 * is the per-sample frequency error in phase-word units. Half of it
 * is applied at every tick while the carrier is present (rejected
 * power above the release ratio), clamped to one 512-point bin per
 * tick, by retuning the coefficients with the filter states kept. */
#define NOTCH_SUB 256
#define NOTCH_TRACK_MAX_WORD ((uint32_t)(4294967296.0 / 512.0))

typedef struct {
    int32_t b1, a1, a2;
    int32_t x1, x2, y1, y2;
} notch_t;

typedef struct {
    notch_t f[NOTCH_MAX];
    uint32_t word[NOTCH_MAX];
    int active[NOTCH_MAX];
    int idle[NOTCH_MAX];
    int64_t rej[NOTCH_MAX];  /* (in - out)^2 of each stage this tick */
    int64_t tot;             /* input^2 this tick */
    int n_tick;
    /* frequency tracking (see NOTCH_SUB) */
    uint32_t ph[NOTCH_MAX];          /* mixer NCO phase */
    int64_t sub_re[NOTCH_MAX], sub_im[NOTCH_MAX];
    int sub_n[NOTCH_MAX];
    int64_t last_ang[NOTCH_MAX];
    int have_ang[NOTCH_MAX];
    int64_t dphi[NOTCH_MAX];         /* unwrapped phase advance this tick */
    int n_dphi[NOTCH_MAX];
    int64_t retunes[NOTCH_MAX];      /* diagnostic */
} notch_bank_t;

void notch_init(notch_t *f, uint32_t word);
/* new centre frequency, filter states kept (a small transient, no reset) */
void notch_retune(notch_t *f, uint32_t word);
int16_t notch_step(notch_t *f, int16_t x);
void notch_bank_clear(notch_bank_t *b);
/* add a notch at word, or refresh one within NOTCH_MERGE_WORD of it;
 * returns 2 if a notch was ADDED, 1 if an existing one was refreshed,
 * 0 if the bank is full */
int notch_bank_request(notch_bank_t *b, uint32_t word);
int notch_bank_active(const notch_bank_t *b);
/* is a notch already within tol_word of word? (refreshes its idle count) */
int notch_bank_near(notch_bank_t *b, uint32_t word, uint32_t tol_word);
/* run one sample through every active notch (and the release monitor) */
int16_t notch_bank_push(notch_bank_t *b, int16_t x);
/* analytic signal of the bank's output: hilbert_analytic on the notched
 * samples, bit-exact with notching the array first */
void hilbert_analytic_notched(const int16_t *x, int n, notch_bank_t *b,
                              samp_t *out_i, samp_t *out_q);

#endif /* OFDM_DSP_H */
