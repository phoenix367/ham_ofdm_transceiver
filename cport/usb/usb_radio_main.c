/* Flash-resident USB modem WITH A RADIO: the station behind the USB
 * endpoints transmits and receives through the board's own converters.
 *
 * usb_flash_main.c binds a stub PHY -- everything above the PHY is real
 * but nothing reaches the air. analog_link.c proved the opposite half:
 * frames cross a wire between two boards' converters, but with no link
 * layer and no host. This image is the two of them in one, which makes a
 * board a complete station a host can drive over USB and a peer can hear
 * over a wire.
 *
 *   DAC1_OUT1 (PA4) --- wire ---> ADC1_INP3 (PA6) on the peer, and back
 *   TIM6 at 12 kHz drives both directions; a common ground is required.
 *
 * ## Why the transmitter never renders a frame
 *
 * station_phy_t::build is frame-at-once: it is handed a buffer and
 * returns the sample count. demoapp does exactly that with a 600000-
 * sample (1.2 MB) buffer. An EXTREME frame is ~38 s = ~456000 samples =
 * 912 kB, so that shape is impossible here.
 *
 * It is also unnecessary. The station never reads or writes those
 * samples -- it only passes the buffer through to build() and hands the
 * returned COUNT to the caller (checked: `out`/`out_cap` appear nowhere
 * else in station_poll_tx). So build() here opens a STREAMING generator
 * and returns the total it reports, without writing a sample; the main
 * loop then pulls from that generator into a small FIFO as the ISR
 * drains it. Bit-identical to the frame-at-once path -- a 1-block burst
 * with no resync is the same waveform as a frame, which both test suites
 * assert.
 *
 * ## Half duplex is not a simplification here, it is the contract
 *
 * The streaming transmitter's generator state lives in the shared arena
 * ACROSS txs_pull calls, while the receiver's scratch is call-scoped.
 * That is only safe because a station never does both at once
 * (arena.h). So this image must never call rxs_push while a
 * transmission is in flight -- and does not: the ISR feeds the capture
 * FIFO only when not transmitting, the FIFO is dropped when a
 * transmission starts, and txs_faulted() is checked as a backstop that
 * would catch the violation rather than let it corrupt a waveform.
 *
 * ## The ADC does not block the ISR
 *
 * analog_link.c started a conversion and waited for it inside the tick
 * (19.2 us of an 83.3 us period, 23%). That was fine when the decoder
 * ran afterwards, and is not fine when the decoder has to keep up in
 * real time. Here the ISR reads the conversion STARTED BY THE PREVIOUS
 * TICK and immediately starts the next, so it never waits: the sample
 * is one tick (83 us) old, a constant delay the receiver cannot tell
 * from cable length.
 */

#include <stdint.h>
#include <string.h>

#include "tusb.h"
#include "usb_proto.h"
#include "usb_desc.h"
#include "usb_modem.h"
#include "station.h"
#include "link.h"
#include "tx.h"
#include "rx_stream.h"

void ofdm_usb_bsp_init(int rhport);
int  ofdm_usb_bsp_supply_ready(void);

/* --- beacon at 0x24000000 ------------------------------------------- */
#define BEACON_MAGIC 0x0AD10BEEu
typedef struct {
    uint32_t magic, stage, mounted, ms, rx_bytes, tx_bytes, isr_count;
    uint32_t tx_frames, rx_decodes, cap_overruns, tx_underruns, tx_faults;
    uint32_t adc_ready, rxs_ready;
    int32_t  last_rung, last_snr_q8;
    uint32_t loops, rx_samples, push_ms_max;
    uint32_t rx_active_mask, my_req, follow_changes;
    uint32_t burst_starts, burst_blocks, burst_misses;
    uint32_t burst_refused, ring_miss;
    uint32_t tx_short, tx_last_pulled, tx_last_total;
    int32_t  last_miss_type, last_miss_bits;
    /* forensics of the FIRST failed continued block since boot:
     * the leading 44 decoded bits (20-bit LC word + 3 subheader
     * bytes -- all predictable), packed MSB-first into two words,
     * the SNR the demod saw, and the walk geometry. The host
     * harness decodes 8/8 under every modeled impairment, so
     * whatever kills block 1 exists only on the boards -- make
     * the boards say what they decoded. */
    uint32_t miss_lead_hi, miss_lead_lo;  /* bits 0..31, 32..43 */
    int32_t  miss_snr_q8, miss_start;
    uint32_t walk_base_lo, walk_step;     /* block-0 end, length */
    uint32_t miss_cs;                     /* cs_mean at first miss */
    /* the last four key-ups: when, and what carrier sense saw.
     * B transmitted over A's live stream while poll_tx is gated
     * on channel_busy() -- so either CS answered idle against
     * 2e8 of mean-square signal, or the key-up came at a moment
     * the air was genuinely quiet. This is the recorder that
     * stops that being a mystery a second time. */
    uint32_t keyups;
    uint32_t keyup_ms[4], keyup_cs[4], keyup_floor[4];
} beacon_t;
volatile beacon_t g_beacon __attribute__((section(".results"), used));
enum { ST_ENTER = 1, ST_SUPPLY, ST_ANALOG, ST_RXS, ST_TUSB, ST_LOOP,
       ST_MOUNTED };

/* --- core ----------------------------------------------------------- */
#define SYST_CSR  (*(volatile uint32_t *)0xE000E010u)
#define SYST_RVR  (*(volatile uint32_t *)0xE000E014u)
#define SYST_CVR  (*(volatile uint32_t *)0xE000E018u)
#define NVIC_ISER ((volatile uint32_t *)0xE000E100u)
#define NVIC_IPR  ((volatile uint8_t  *)0xE000E400u)
#define IRQ_OTG_FS 101
#define IRQ_TIM6_DAC 54

static volatile uint32_t g_ms;
static void note_busy_isr(int16_t v);  /* carrier sense, defined below */

void SysTick_Handler(void) { g_ms++; }

void OTG_FS_Handler(void)
{
    g_beacon.isr_count++;
    tud_int_handler(0);
}

static double now_s(void) { return (double)g_ms * 0.001; }

/* Push the beacon out to memory.
 *
 * With the D-cache enabled the debugger does NOT see dirty lines -- it
 * reads through the AHB-AP, not the cache -- so every counter here would
 * read stale or zero over JTAG. Two 32-byte lines, called once per
 * millisecond, which is far cheaper than the confusion of debugging a
 * beacon that lies. */
#define SCB_DCCMVAC (*(volatile uint32_t *)0xE000EF68u)
static void beacon_flush(void)
{
    uint32_t a = (uint32_t)(uintptr_t)&g_beacon & ~31u;
    uint32_t end = (uint32_t)(uintptr_t)&g_beacon + sizeof(g_beacon);
    __asm__ volatile("dsb");
    for (; a < end; a += 32u)
        SCB_DCCMVAC = a;
    __asm__ volatile("dsb");
}

/* --- peripherals (register set as in bench/analog_link.c) ------------ */
#define RCC_BASE      0x58024400u
#define RCC_D3CCIPR   (*(volatile uint32_t *)(RCC_BASE + 0x58u))
#define RCC_AHB1ENR   (*(volatile uint32_t *)(RCC_BASE + 0xD8u))
#define RCC_AHB4ENR   (*(volatile uint32_t *)(RCC_BASE + 0xE0u))
#define RCC_APB1LENR  (*(volatile uint32_t *)(RCC_BASE + 0xE8u))

#define TIM6_BASE     0x40001000u
#define TIM6_CR1      (*(volatile uint32_t *)(TIM6_BASE + 0x00u))
#define TIM6_DIER     (*(volatile uint32_t *)(TIM6_BASE + 0x0Cu))
#define TIM6_SR       (*(volatile uint32_t *)(TIM6_BASE + 0x10u))
#define TIM6_EGR      (*(volatile uint32_t *)(TIM6_BASE + 0x14u))
#define TIM6_PSC      (*(volatile uint32_t *)(TIM6_BASE + 0x28u))
#define TIM6_ARR      (*(volatile uint32_t *)(TIM6_BASE + 0x2Cu))

#define DAC1_BASE     0x40007400u
#define DAC_CR        (*(volatile uint32_t *)(DAC1_BASE + 0x00u))
#define DAC_DHR12R1   (*(volatile uint32_t *)(DAC1_BASE + 0x08u))
#define DAC_MCR       (*(volatile uint32_t *)(DAC1_BASE + 0x3Cu))

#define ADC1_BASE     0x40022000u
#define ADC_ISR_R     (*(volatile uint32_t *)(ADC1_BASE + 0x00u))
#define ADC_CR        (*(volatile uint32_t *)(ADC1_BASE + 0x08u))
#define ADC_CFGR      (*(volatile uint32_t *)(ADC1_BASE + 0x0Cu))
#define ADC_SMPR1     (*(volatile uint32_t *)(ADC1_BASE + 0x14u))
#define ADC_PCSEL     (*(volatile uint32_t *)(ADC1_BASE + 0x1Cu))
#define ADC_SQR1      (*(volatile uint32_t *)(ADC1_BASE + 0x30u))
#define ADC_DR        (*(volatile uint32_t *)(ADC1_BASE + 0x40u))
#define ADC_CCR       (*(volatile uint32_t *)(ADC1_BASE + 0x308u))
#define ADC_CHANNEL   3                      /* ADC12_INP3 = PA6 */

/* APB1 timer clock, measured on the part by bench/analog_link.c as
 * 199999720 Hz. TIM6 divides it to 12 kHz; both boards land on the same
 * divisor, so what remains between them is the crystal difference. */
#define TIM_HZ  200000000u
#define FS_HZ   12000u

/* --- the two FIFOs between the ISR and the main loop ----------------- */
/* Capture: sized by the receiver's WORST BURST, not by a frame.
 *
 * The decoder's cost is wildly uneven. Quiet blocks are cheap; the
 * commit at the end of a frame -- acquisition, demodulation of every
 * tiled symbol, Viterbi -- is one long blocking call. Measured on this
 * part at EXTREME: 2283 ms in a single rxs_push, against a 19.5 s
 * frame. That is only ~12% average duty, comfortably real time, but it
 * arrives all at once, so what matters is whether the FIFO can hold the
 * samples that keep arriving THROUGH the burst.
 *
 * 16384 samples (1.37 s) could not: the receiver dropped 33036 samples
 * mid-frame and decoded nothing. 65536 is 5.5 s, more than twice the
 * worst burst measured. cap_overruns and push_ms_max in the beacon are
 * how this is checked rather than assumed. */
#define CAP_N 65536
#define TXF_N 2048

static int16_t g_cap[CAP_N];
static volatile uint32_t g_cap_w, g_cap_r;
static int16_t g_txf[TXF_N];
static volatile uint32_t g_txf_w, g_txf_r;
static volatile int g_tx_on;      /* ISR: 1 = drive DAC, 0 = sample ADC */

static void tim6_isr(void)
{
    TIM6_SR = 0;
    if (g_tx_on) {
        uint32_t r = g_txf_r;
        if (r != g_txf_w) {
            /* 3/4 scale: the DAC's buffer cannot reach the rails and the
             * transmitter reaches int16 full scale (same mapping the
             * analog bench measured a clean path with). */
            int32_t s = g_txf[r & (TXF_N - 1)];
            DAC_DHR12R1 = (uint32_t)(2048 + ((s * 3) >> 6));
            g_txf_r = r + 1;
        } else {
            DAC_DHR12R1 = 2048;
            g_beacon.tx_underruns++;   /* the main loop fell behind */
        }
    } else {
        uint32_t w = g_cap_w;
        /* the conversion started last tick is finished long since */
        int16_t v = (int16_t)(ADC_DR - 32768u);
        /* carrier sense on LIVE samples. This call was meant to arrive
         * with the note_busy_isr commit and did not -- the edit's
         * pattern had the wrong indentation and replaced nothing, so
         * g_cs_mean stayed 0 and channel_busy() answered idle FOREVER.
         * That is what let this station answer block 0's ack request
         * over the top of the peer's still-running stream. Caught by
         * the key-up recorder: cs=0 at every key-up, when a quiet wire
         * (the peer's DAC holds mid-rail) reads ~2e4. */
        note_busy_isr(v);
        if ((uint32_t)(w - g_cap_r) < CAP_N)
            g_cap[w & (CAP_N - 1)] = v, g_cap_w = w + 1;
        else
            g_beacon.cap_overruns++;
        ADC_CR |= (1u << 2);           /* ADSTART: next one, do not wait */
    }
}

void TIM6_DAC_Handler(void) { tim6_isr(); }

/* --- bring-up -------------------------------------------------------- */
static int adc_init(void)
{
    uint32_t guard;
    RCC_AHB1ENR |= (1u << 5);                       /* ADC12EN */
    RCC_D3CCIPR = (RCC_D3CCIPR & ~(3u << 16)) | (2u << 16);  /* per_ck */
    ADC_CCR = (4u << 18);                           /* async, /8 */
    ADC_CR &= ~(1u << 29);                          /* DEEPPWD off */
    ADC_CR |= (1u << 28);                           /* ADVREGEN */
    for (guard = 0; guard < 4000000u; guard++)
        if (ADC_ISR_R & (1u << 12))                 /* LDORDY */
            break;
    for (guard = 0; guard < 200000u; guard++)
        __asm__ volatile("nop");
    ADC_CR |= (1u << 8);                            /* BOOST */
    ADC_CR |= (1u << 31);                           /* ADCAL */
    for (guard = 0; guard < 40000000u; guard++)
        if (!(ADC_CR & (1u << 31)))
            break;
    ADC_CFGR = 0;                                   /* 16-bit, single, sw */
    ADC_PCSEL = 1u << ADC_CHANNEL;                  /* H7: or read garbage */
    ADC_SQR1 = (uint32_t)ADC_CHANNEL << 6;
    ADC_SMPR1 = 5u << (3 * ADC_CHANNEL);            /* 64.5 cycles */
    ADC_ISR_R = 1u;                                 /* clear ADRDY */
    ADC_CR |= 1u;                                   /* ADEN */
    for (guard = 0; guard < 4000000u; guard++)
        if (ADC_ISR_R & 1u)
            return 1;
    return 0;
}

static void dac_init(void)
{
    uint32_t g;
    RCC_APB1LENR |= (1u << 29);                     /* DAC12EN */
    DAC_MCR = 0;                                    /* buffer on, to pin */
    DAC_DHR12R1 = 2048;
    DAC_CR = 1u;                                    /* EN1 */
    for (g = 0; g < 200000u; g++)
        __asm__ volatile("nop");
}

static void tim6_init(void)
{
    RCC_APB1LENR |= (1u << 4);                      /* TIM6EN */
    TIM6_CR1 = 0;
    TIM6_PSC = 0;
    TIM6_ARR = (TIM_HZ + FS_HZ / 2u) / FS_HZ - 1u;
    TIM6_EGR = 1;
    TIM6_SR = 0;
    TIM6_DIER = 1;                                  /* UIE */
    NVIC_IPR[IRQ_TIM6_DAC] = 0x40;                  /* above USB's 0x80 */
    NVIC_ISER[IRQ_TIM6_DAC >> 5] = 1u << (IRQ_TIM6_DAC & 31);
    TIM6_CR1 = 1;                                   /* CEN */
}

/* --- the PHY --------------------------------------------------------- */
static station_t g_st;
static usb_modem_t g_modem;
static txs_t *g_txs;                 /* live generator, NULL when idle */
static int g_tx_total, g_tx_pulled;

/* Opens a generator and reports its length WITHOUT writing a sample --
 * see the file header for why that is both necessary and safe. */
static int phy_build_stream(const uint8_t *blocks, int pkt_n, int n_blocks,
                            int typ, int rung, int resync_every)
{
    int total = 0;
    if (g_txs)
        return -1;                    /* already transmitting */
    g_txs = txs_open(ladder_mode(rung), blocks, pkt_n, n_blocks, typ,
                     ladder_mod(rung), ladder_spd(rung), resync_every, 0,
                     &total);
    if (!g_txs || total <= 0) {
        g_txs = 0;
        return -1;
    }
    g_tx_total = total;
    g_tx_pulled = 0;
    return total;
}

static int phy_build(void *c, const uint8_t *b, int n, int typ, int rung,
                     int16_t *out, int cap)
{
    (void)c; (void)out; (void)cap;    /* never written: we stream instead */
    return phy_build_stream(b, n, 1, typ, rung, 0);
}

static int phy_build_burst(void *c, const uint8_t *blocks, int pkt_n,
                           int n_blocks, int typ, int rung, int resync_every,
                           int16_t *out, int cap)
{
    (void)c; (void)out; (void)cap;
    return phy_build_stream(blocks, pkt_n, n_blocks, typ, rung,
                            resync_every);
}

static int phy_recv_unused(void *c, const int16_t *s, int n, uint8_t *bits,
                           int *nb, double *snr, double *cfo, int *harq,
                           const int64_t *pl, int pn, int64_t *lo, int *ln)
{
    (void)c;(void)s;(void)n;(void)bits;(void)nb;(void)snr;(void)cfo;
    (void)harq;(void)pl;(void)pn;(void)lo;(void)ln;
    return -1;                        /* decoded through rxs_push instead */
}

/* Pull from the live generator into the DAC FIFO until it is within
 * `slack` samples of full, or the generator is exhausted (which clears
 * g_txs). ONE implementation on purpose: this arithmetic was written
 * twice and the second copy was wrong -- it passed
 *     TXF_N - (w - r) - (w & (TXF_N-1))
 * as the capacity, which with r = 0 is TXF_N - 2w: it double-counts the
 * write index against the free space and reaches zero once the FIFO is
 * half full. txs_pull then returned 0, the caller read that as "frame
 * finished" and dropped the generator, and the board transmitted a
 * TRUNCATED frame. It hid at EXTREME, where the first pull fills the
 * whole FIFO and the loop breaks before the second iteration, and for
 * frames shorter than the FIFO, where the generator really was
 * exhausted -- so the link bootstrapped happily, climbed to a rung
 * whose frames are longer than the FIFO but pulled in small pieces, and
 * then stopped being decodable at all. */
static void tx_fill(uint32_t slack)
{
    while (g_txs && (uint32_t)(g_txf_w - g_txf_r) + slack < TXF_N) {
        int wi = (int)(g_txf_w & (TXF_N - 1));
        int room = (int)(TXF_N - (g_txf_w - g_txf_r));
        int lin = (int)TXF_N - wi;      /* contiguous, to the wrap */
        int got;
        if (room > lin)
            room = lin;
        if (room <= 0)
            break;
        got = txs_pull(g_txs, g_txf + wi, room);
        if (got <= 0) {
            g_txs = 0;                  /* generator exhausted */
            break;
        }
        g_txf_w += (uint32_t)got;
        g_tx_pulled += got;
    }
}

static rxs_t *g_rxs[3];   /* declared here: burst_advance() needs it */
static volatile uint32_t g_cs_mean;  /* defined with carrier sense below */

/* --- streamed bursts -------------------------------------------------
 *
 * phy.receive_burst stays NULL, and not for want of trying: the station
 * only ever reaches it through phy.receive(), the FRAME-AT-ONCE entry
 * point that is handed a whole recording. This firmware never has one
 * -- it decodes as samples arrive -- and buffering a burst would be the
 * 912 kB problem that phy.build already refuses. The streaming receiver
 * has its own continuation instead: after a block whose packet says
 * more follow, rxs_continue_burst() takes the next block from the
 * deterministic offset after this one rather than hunting for a
 * preamble. Same arrangement demoapp uses.
 *
 * The continuation MUST know where to stop. While stepping through
 * blocks the receiver is not running the preamble detector, so chasing
 * blocks that were never sent makes it deaf for one block-time each --
 * measured in demoapp as a ~12 s hole that ate the peer's next burst
 * and produced an endless retransmit loop. The stop signal is the
 * ack-request bit on the burst's last block, tested as "set AND not the
 * first block of this stream" because the first block carries it too
 * (for peers that cannot stream at all). A consecutive-failure bound
 * covers the case where that last block is itself the one that did not
 * decode. */
static int g_burst_left[3], g_burst_miss[3];
/* A walk in progress HOLDS this station's transmitter (below), so
 * it must not be able to last forever: if the peer dies mid-
 * stream no further events fire, burst_left never reaches 0, and
 * without a deadline the hold would mute us for good. */
static uint32_t g_burst_deadline[3];

static int frame_is_streamed(const uint8_t *bits, int nbits, int *ack_req)
{
    lc_word_t lc;
    uint32_t reserved = 0;
    int i, v = 0;

    if ((nbits - 36) / 8 < BURST_SUBHDR)
        return 0;
    for (i = 0; i < 20; i++)
        reserved = (reserved << 1) | (bits[i] & 1);
    lc_unpack(reserved, &lc);
    if (lc.flags != FLAG_BURST_DATA)
        return 0;
    for (i = 0; i < 8; i++)
        v = (v << 1) | (bits[20 + i] & 1);
    if (ack_req)                      /* sub-header byte 1, bit 7 */
        *ack_req = bits[20 + 8] & 1;
    return (v & BURST_SUB_STREAMED) != 0;
}

/* One decoded block (or one that failed) on receiver m. Returns with
 * g_burst_left[m] set to however many blocks are still expected. */
static void burst_advance(int m, const rxs_event_t *ev)
{
    int ackreq = 0, streamed;

    streamed = ev->type == 1
               && frame_is_streamed(ev->bits, ev->pkt_bits_n, &ackreq);
    if (g_burst_left[m] > 0 || streamed)
        g_burst_deadline[m] = g_ms + 15000u;
    if (streamed && g_burst_left[m] == 0) {
        g_burst_left[m] = BURST_STREAM_MAX - 1;
        g_burst_miss[m] = 0;
        g_beacon.burst_starts++;
        /* geometry of the walk this arms: block 0's start and the step
         * the continuation will take from its end */
        g_beacon.walk_base_lo = (uint32_t)ev->start_abs;
        g_beacon.walk_step = (uint32_t)ev->pkt_bits_n;
    } else if (streamed) {
        g_burst_miss[m] = 0;
        g_beacon.burst_blocks++;
        if (ackreq)
            g_burst_left[m] = 0;      /* the last block: burst complete */
        else
            g_burst_left[m]--;
    } else if (g_burst_left[m] > 0) {
        /* a block we could not decode: keep going, so one bad block does
         * not cost the tail -- but not forever */
        g_beacon.burst_misses++;
        /* what KIND of miss: a failed decode (type != 1) or a decode
         * that did not look like a streamed block (type 1, not
         * streamed)? The two want different fixes. */
        g_beacon.last_miss_type = ev->type;
        g_beacon.last_miss_bits = ev->type == 1 ? ev->pkt_bits_n : -1;
        if (g_beacon.miss_lead_hi == 0 && g_beacon.miss_lead_lo == 0) {
            uint32_t hi = 0, lo = 0;
            int b;
            for (b = 0; b < 32; b++)
                hi = (hi << 1) | (ev->bits[b] & 1u);
            for (b = 32; b < 44; b++)
                lo = (lo << 1) | (ev->bits[b] & 1u);
            g_beacon.miss_lead_hi = hi;
            g_beacon.miss_lead_lo = lo;
            g_beacon.miss_snr_q8 = (int32_t)(ev->snr_db * 256.0);
            g_beacon.miss_start = ev->start_abs;
            g_beacon.miss_cs = g_cs_mean;
        }
        if (++g_burst_miss[m] >= 2)
            g_burst_left[m] = 0;
        else
            g_burst_left[m]--;
    }
    if (g_burst_left[m] > 0
        && !rxs_continue_burst(g_rxs[m], BURST_STREAM_RESYNC)) {
        g_beacon.burst_refused++;   /* the decoder would not arm */
        g_burst_left[m] = 0;
    }
    g_beacon.ring_miss = (uint32_t)(rxs_ring_miss(g_rxs[0])
                                    + rxs_ring_miss(g_rxs[1])
                                    + rxs_ring_miss(g_rxs[2]));
}

/* --- carrier sense --------------------------------------------------- */
/* Same shape as demoapp -- a 40 ms power window against a floor that
 * drops instantly and climbs slowly, plus the rebase that stops a step
 * rise in the noise floor from freezing carrier sense at BUSY (measured
 * there at 82 s of dead air after a -37 dB step); CS_REBASE_S must stay
 * above the longest frame, 38 s at EXTREME.
 *
 * MEASURED IN THE ISR, not on the decoder's feed. demoapp accumulates
 * over the samples it pushes into the receiver, which is the same thing
 * there because it pushes in real time. Here it is not: a single
 * rxs_push can block for 2.4 s while it commits a frame, so the pushed
 * stream runs seconds behind the air. Carrier sense computed on it
 * reports the channel as it was, not as it is -- and this station
 * therefore keyed up in the middle of a peer's streamed burst, dropped
 * its own capture FIFO at key-up, and lost the rest of the burst.
 * Measured: 8 transmissions during one transfer, every streamed burst
 * abandoned after two blocks (burst_starts 2, burst_blocks 0).
 *
 * The ISR keeps the ring and an int64 accumulator and publishes the
 * mean as one 32-bit store, which the main loop can read without
 * tearing. */
#define BUSY_WIN 480
#define BUSY_RATIO_SQ 9.0
#define CS_REBASE_S 60.0
static int16_t g_bring[BUSY_WIN];
static int g_bpos;
static int64_t g_bacc;
static volatile uint32_t g_cs_mean;      /* published by the ISR */
static double g_floor = 1e9, g_busy_since = -1.0;

/* called from the 12 kHz tick, one live sample at a time */
static void note_busy_isr(int16_t v)
{
    int16_t old = g_bring[g_bpos];
    g_bacc += (int64_t)v * v - (int64_t)old * old;
    g_bring[g_bpos] = v;
    if (++g_bpos >= BUSY_WIN)
        g_bpos = 0;
    g_cs_mean = (uint32_t)(g_bacc / BUSY_WIN);
}

static int channel_busy(double now)
{
    double p = (double)g_cs_mean;
    static uint32_t climb_ms;
    int busy;
    if (p < g_floor) {
        g_floor = p;
    } else if ((uint32_t)(g_ms - climb_ms) >= 40u) {
        /* The climb rate is DEFINED per 40 ms window (demoapp's cadence,
         * 0.05% per block), not per call: this function runs at 1 kHz
         * here against demoapp's ~25 Hz, and multiplying per call made
         * the floor rise 40x too fast -- x1.65 per second of continuous
         * signal instead of x1.013. */
        g_floor *= 1.0005;
        climb_ms = g_ms;
    }
    if (g_floor < 25.0)
        g_floor = 25.0;
    busy = p > BUSY_RATIO_SQ * g_floor;
    if (!busy)
        g_busy_since = -1.0;
    else if (g_busy_since < 0.0)
        g_busy_since = now;
    else if (now - g_busy_since > CS_REBASE_S) {
        g_floor = p;
        g_busy_since = -1.0;
        busy = 0;
    }
    return busy;
}

/* --- receivers, one per mode over the shared raw ring ----------------
 *
 * Which modes to listen for is a CPU BUDGET, not a free choice. Each
 * open receiver runs its own detector over every sample, and the
 * EXTREME one is much the most expensive (its detection window is 64x
 * a NORMAL symbol). demoapp opens all three because a workstation can
 * afford to; this part measurably cannot -- see the header of
 * follow_rung(). Bit 0 NORMAL, bit 1 ROBUST, bit 2 EXTREME.
 *
 * All of them are OPENED; which are ACTIVE follows the negotiated rung
 * (see follow_rung). A muted instance still consumes every sample --
 * it has to, the raw ring is shared and indexed by abs_n -- but skips
 * the per-block detection, which is where the cost is. */
#ifndef OFDM_RX_MODES
#define OFDM_RX_MODES 0x7
#endif
/* Seconds of silence after which the mode we last heard the peer on
 * stops being evidence of anything. Above the ladder's own staleness
 * decay, so the request has already fallen back to EXTREME by then. */
#define RX_MODE_STALE_S 120.0

/* Listen where the peer is actually going to transmit.
 *
 * The peer transmits at the rung WE asked it for -- ctl.my_req, the
 * inbound half of the link control word -- so that is the mode to
 * listen for. Two more are kept active alongside it, and both matter:
 *
 *   EXTREME, always. It is rung 0, so it is the bootstrap; it is where
 *   the ladder decays to after RX_STALE_S of silence; and it is what a
 *   peer that cannot hear us falls back to. Muting it would make the
 *   link undiscoverable and unrecoverable exactly when recovery is
 *   needed.
 *
 *   The mode we LAST decoded on. my_req is a request, not an
 *   observation: the peer only moves once it has received it, so
 *   between our raising the request and the peer acting on it, the peer
 *   is still transmitting at the old mode. Dropping that mode the
 *   instant we ask for a different one makes us deaf for exactly the
 *   exchange that would have confirmed the change -- we would time out,
 *   the request would decay, and the link would oscillate. Keeping it
 *   until the peer actually moves (or until it goes stale) costs one
 *   muted-to-active detector during the handover and nothing after.
 *
 * In the settled bootstrap state -- my_req 0, last decode EXTREME --
 * this is a single active detector, which is what makes the whole thing
 * fit real time on this part. */
static int g_last_rx_mode = -1;
static double g_last_rx_t = -1e9;

static void follow_rung(double now)
{
    int want = 1 << (int)MODE_EXTREME;
    int i, req = g_st.ctl.my_req;

    if (req < 0)
        req = 0;
    want |= 1 << (int)ladder_mode(req);
    if (g_last_rx_mode >= 0 && now - g_last_rx_t < RX_MODE_STALE_S)
        want |= 1 << g_last_rx_mode;
    /* A receiver walking a burst is not running the preamble detector,
     * and muting rearms the search -- so muting one mid-burst would
     * throw away the rest of the transfer. Hold it until the walk
     * ends, which it always does (ack-request bit, or the
     * consecutive-failure bound). */
    for (i = 0; i < 3; i++)
        if (g_burst_left[i] > 0)
            want |= 1 << i;

    if ((uint32_t)want != g_beacon.rx_active_mask)
        g_beacon.follow_changes++;
    g_beacon.rx_active_mask = (uint32_t)want;
    g_beacon.my_req = (uint32_t)req;
    for (i = 0; i < 3; i++)
        if (g_rxs[i])
            rxs_set_active(g_rxs[i], (want >> i) & 1);
}

int main(void)
{
    static uint8_t in[64], out[64];
    static uint8_t air_dummy[64];     /* build() never writes; see header */
    static const uint8_t *const UID = (const uint8_t *)0x1FF1E800u;
    station_phy_t phy;
    double t, last_status = 0.0;
    uint32_t slow = 0;
    int i, n;

    memset((void *)&g_beacon, 0, sizeof(g_beacon));
    g_beacon.magic = BEACON_MAGIC;
    g_beacon.stage = ST_ENTER;

    SYST_RVR = 400000u - 1u;
    SYST_CVR = 0;
    SYST_CSR = 7u;

    g_beacon.stage = ST_SUPPLY;
    while (!ofdm_usb_bsp_supply_ready())
        ;
    ofdm_usb_bsp_init(0);

    g_beacon.stage = ST_ANALOG;
    RCC_AHB4ENR |= (1u << 0);         /* GPIOA: PA4/PA6 stay analog */
    dac_init();
    g_beacon.adc_ready = (uint32_t)adc_init();
    ADC_CR |= (1u << 2);              /* prime the first conversion */
    tim6_init();

    g_beacon.stage = ST_RXS;
    g_beacon.rxs_ready = 1u;
    if (OFDM_RX_MODES & 1) {
        g_rxs[0] = rxs_open(MODE_NORMAL, 0);
        if (!g_rxs[0]) g_beacon.rxs_ready = 0u;
    }
    if (OFDM_RX_MODES & 2) {
        g_rxs[1] = rxs_open(MODE_ROBUST, 0);
        if (!g_rxs[1]) g_beacon.rxs_ready = 0u;
    }
    if (OFDM_RX_MODES & 4) {
        g_rxs[2] = rxs_open(MODE_EXTREME, 0);
        if (!g_rxs[2]) g_beacon.rxs_ready = 0u;
    }

    memset(&phy, 0, sizeof(phy));
    phy.build = phy_build;
    phy.receive = phy_recv_unused;
    phy.build_burst = phy_build_burst;
    /* receive_burst stays NULL BY DESIGN, not for want of an
     * implementation: the station only reaches it from phy.receive(),
     * the frame-at-once path that is handed a whole recording, and this
     * firmware never has one. Streamed reception is done where it
     * belongs for a streaming receiver -- burst_advance() above. */
    station_init(&g_st, &phy, 0x5EEDu);
    /* Streamed windows need BOTH of these, and setting only the second
     * gets you nothing: burst ARQ itself is gated on burst_window >= 2
     * (station.c), and a station left at the default runs legacy
     * stop-and-wait, so burst_stream never comes into play. Measured
     * with burst_stream alone: a 6-part file crossed the wire correctly
     * in 68 per-frame transmissions with burst_starts still 0.
     *
     * The host can still turn streaming off with UP_CFG_BURST_STREAM or
     * resize the window with UP_CFG_BURST_WINDOW, and the station falls
     * back by itself if a peer turns out not to follow a stream
     * (ST_SOFF_NOACK). */
    /* Burst windows and streamed windows: ON. They were shipped OFF
     * when every streamed transfer died, and the post-mortem found
     * three real causes, none of them the burst machinery itself:
     * MAX_LLRS=1024 overflowing g_d64 on any frag_size >= 100 (host
     * repro: 0/3 blocks at 1024, 3/3 at the default), carrier sense
     * dead since its ISR hook was never actually inserted, and the
     * station answering block 0's ack request over the peer's live
     * stream (fixed by the walk hold below). With all three fixed the
     * same 1200-byte file crosses byte-exact in ~21 s streamed against
     * ~2 min per-frame, one 8-block stream acking 8 of 11 frags at
     * once. The fallbacks (bitmap ack, ST_SOFF_*) remain for peers and
     * channels that cannot stream.
     *
     * Selective-repeat windows with a bitmap ack: ON. Streaming a whole
     * window behind ONE preamble: OFF, and measured rather than
     * assumed. The transmit half works -- BURST_STREAM reports 8 blocks
     * in one 116448-sample transmission and tx_short confirms every
     * promised sample reached the DAC -- and the receiver detects the
     * stream marker and arms the continuation (burst_starts 2,
     * burst_refused 0). But every continued block then fails to decode
     * (burst_blocks 0, all misses type -3), with the samples present
     * (ring_miss 0) and no capture overrun. Ruled out by measurement:
     * truncated transmit, ring overwrite, arming refusal, the resync
     * step-over (identical with BURST_STREAM_RESYNC 0), and the rung
     * floor (the stream ran at rung 6, NORMAL, above BURST_MIN_RUNG).
     *
     * The one thing this stand has that demoapp does not is two
     * INDEPENDENT sample clocks, and rxs_continue_burst says in its own
     * comment that it does not re-lock on the resync ZC, being "benign
     * ... because an open-loop NORMAL stream was measured to hold far
     * longer than any burst lasts" -- measured where both ends share
     * one clock. That is consistent with what is seen but is NOT
     * demonstrated here, so it is written down as the open question,
     * not as the answer.
     *
     * Left off because the cost is real: the station spends two windows
     * failing, takes the timeouts, and drags the rung down before
     * ST_SOFF_TIMEOUT puts it back on per-frame bursts -- which deliver
     * the file byte-exact. `config burst_stream 1` from the console
     * turns it on for further work.
     *
     * The WINDOW went back to the default for a separate and more
     * serious reason. burst_window = 8 works while the rung holds, and
     * acked 8 of 11 fragments in one bitmap. But frag_size is fixed at
     * engage and uniform for the whole transfer, so when the rung
     * collapses mid-transfer the station keeps sending fragments sized
     * for the rung it engaged at: measured, one transmission of
     * 2693120 samples -- 224 SECONDS of air -- after a window engaged
     * at rung 12 and the link fell back to EXTREME. Nothing recovers
     * from that inside a transfer: the peer cannot reply for four
     * minutes, timers expire, and the transfer stalls having delivered
     * one part. Without the window the same file crossed byte-exact.
     *
     * (That 224-second-frame hazard -- frag_size fixed at engage, rung
     * collapsing mid-transfer -- was itself a symptom: the collapse was
     * driven by the failures above poisoning the controller.) */
    g_st.burst_window = BURST_STREAM_MAX;
    g_st.burst_stream = 1;
    usb_modem_init(&g_modem, &g_st, UID, 0x0200,
                   UP_CAP_LDPC | UP_CAP_EXT_FRAMES);
    g_st.diag_cb = usb_modem_diag;
    g_st.diag_ctx = &g_modem;

    tusb_init();
    NVIC_IPR[IRQ_OTG_FS] = 0x80;
    NVIC_ISER[IRQ_OTG_FS >> 5] = 1u << (IRQ_OTG_FS & 31);
    g_beacon.stage = ST_TUSB;

    for (;;) {
        tud_task();
        g_beacon.loops++;
        g_beacon.stage = tud_mounted() ? ST_MOUNTED : ST_LOOP;
        g_beacon.mounted = tud_mounted() ? 1u : 0u;
        g_beacon.ms = g_ms;
        t = now_s();

        if (tud_vendor_available()) {
            uint32_t k = tud_vendor_read(in, sizeof(in));
            g_beacon.rx_bytes += k;
            usb_modem_rx(&g_modem, in, (int)k);
        }

        /* ---- transmit: keep the DAC FIFO fed ---- */
        if (g_txs) {
            tx_fill(256);
            if (txs_faulted())
                g_beacon.tx_faults++;
        }
        if (g_tx_on && !g_txs && g_txf_r == g_txf_w) {
            /* every generated sample has left the DAC.
             *
             * phy.build reported g_tx_total to the station WITHOUT
             * rendering it, so nothing else would notice if the
             * generator then produced a different number of samples --
             * the station would believe it transmitted a frame the air
             * never carried. Check it here, where both numbers exist. */
            g_beacon.tx_last_pulled = (uint32_t)g_tx_pulled;
            g_beacon.tx_last_total = (uint32_t)g_tx_total;
            if (g_tx_pulled != g_tx_total)
                g_beacon.tx_short++;
            g_tx_on = 0;
            g_cap_r = g_cap_w;            /* drop what leaked in */
            station_on_tx_end(&g_st, t);
        }

        /* ---- receive: only when not transmitting (arena, half duplex)
         *
         * One chunk per pass through the loop, not a drain: an
         * acquisition costs far more than a quiet block, and USB has to
         * stay serviced through it. The FIFO is what carries the
         * backlog, and cap_overruns says if it ever was not enough. */
        if (!g_tx_on && g_ms > 500u) {   /* let the analog path settle */
            if (g_cap_r != g_cap_w) {
                int16_t chunk[256];
                uint32_t avail = g_cap_w - g_cap_r;
                int m = avail > 256u ? 256 : (int)avail;
                int k;
                for (k = 0; k < m; k++)
                    chunk[k] = g_cap[(g_cap_r + (uint32_t)k) & (CAP_N - 1)];
                uint32_t t0 = g_ms, dt;
                g_cap_r += (uint32_t)m;
                g_beacon.rx_samples += (uint32_t)m;
                for (i = 0; i < 3; i++) {
                    rxs_event_t ev;
                    if (!g_rxs[i] || !rxs_push(g_rxs[i], chunk, m, &ev))
                        continue;
                    /* every event, decoded or not, moves the burst
                     * walk on: a failed block must still be counted or
                     * the receiver steps out of phase with the sender */
                    burst_advance(i, &ev);
                    if (ev.type != 1)
                        continue;
                    g_beacon.rx_decodes++;
                    g_beacon.last_snr_q8 = (int32_t)(ev.snr_db * 256.0);
                    g_last_rx_mode = i;      /* the index IS the mode */
                    g_last_rx_t = t;
                    station_on_decoded(&g_st, ev.bits, ev.pkt_bits_n,
                                       ev.snr_db,
                                       (double)ev.cfo_word * 12000.0
                                           / 4294967296.0,
                                       0, t);
                }
                dt = g_ms - t0;
                if (dt > g_beacon.push_ms_max)
                    g_beacon.push_ms_max = dt;
            }
        }

        /* ---- the station, at ~1 kHz ---- */
        if ((uint32_t)(g_ms - slow) >= 1u) {
            slow = g_ms;
            if (g_ms <= 500u)
                g_cap_r = g_cap_w;       /* discard the settling transient */
            follow_rung(t);
            /* expire wedged walks (peer died mid-stream: no events) */
            {
                int i2;
                for (i2 = 0; i2 < 3; i2++)
                    if (g_burst_left[i2] > 0
                        && (int32_t)(g_ms - g_burst_deadline[i2]) > 0)
                        g_burst_left[i2] = 0;
            }
            /* A receiver that KNOWS more blocks are coming must not
             * transmit over them. Measured on the stand without this:
             * block 0 of a stream carries the ack request (by design,
             * for peers that cannot stream), B's station answered it
             * ~2 s into A's 9.75 s transmission, key-up dropped B's own
             * capture, and the walk decoded noise at -30 dB -- every
             * streamed burst died this way while A's transmit counters
             * showed a clean carrier. The ack goes out when the walk
             * ends, which is exactly the contract: one ack, after every
             * block of the burst has been processed. */
            if (!g_tx_on && !g_txs
                && !(g_burst_left[0] | g_burst_left[1] | g_burst_left[2])) {
                int air = station_poll_tx(&g_st, t, channel_busy(t),
                                          (int16_t *)air_dummy,
                                          1 << 24);
                if (air > 0 && g_txs) {
                    /* Fill the FIFO BEFORE the ISR starts draining it.
                     * Arming an empty FIFO underran it every time the
                     * carrier started -- measured 166 underruns across 3
                     * frames, ~55 per frame, i.e. once per transmission
                     * rather than a throughput problem. Each underrun
                     * puts a mid-rail sample on the air. */
                    g_txf_r = g_txf_w = 0;
                    tx_fill(0);
                    /* build() opened the generator; start the carrier.
                     * Drop what was captured before now: those samples
                     * pre-date the transmission, and feeding them after
                     * it ends would hand the receiver a stream whose
                     * time order does not match the air. */
                    g_cap_r = g_cap_w;
                    {
                        uint32_t ki = g_beacon.keyups & 3u;
                        g_beacon.keyup_ms[ki] = g_ms;
                        g_beacon.keyup_cs[ki] = g_cs_mean;
                        g_beacon.keyup_floor[ki] = (uint32_t)g_floor;
                        g_beacon.keyups++;
                    }
                    g_tx_on = 1;
                    g_beacon.tx_frames++;
                    g_beacon.last_rung = g_st.stats.last_rung;
                } else if (g_txs) {
                    /* build() opened a generator but the station did not
                     * transmit after all. Abandoning it here matters: a
                     * generator left open makes `!g_txs` false forever
                     * and the station never transmits again. */
                    g_txs = 0;
                }
            }
            usb_modem_tick(&g_modem, t, t - last_status >= 0.5);
            if (t - last_status >= 0.5)
                last_status = t;
            beacon_flush();
        }

        while (tud_vendor_write_available() > 0 &&
               (n = usb_modem_poll(&g_modem, out,
                     (int)(tud_vendor_write_available() < sizeof(out)
                           ? tud_vendor_write_available() : sizeof(out)))) > 0) {
            tud_vendor_write(out, (uint32_t)n);
            tud_vendor_write_flush();
            g_beacon.tx_bytes += (uint32_t)n;
        }
    }
}
