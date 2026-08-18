/* Link-layer station -- C twin of ofdm_phy/station.py: QoS queues with
 * priority-stream preemption, segmentation/reassembly, stop-and-wait ARQ
 * with HARQ hooks, rate adaptation, AFC netting, simplex channel access.
 * PHY behind callbacks so it runs over the C fixed PHY or a test stub. */
#ifndef OFDM_STATION_H
#define OFDM_STATION_H

#include <stdint.h>
#include "link.h"

#define QOS_CONTROL 0
#define QOS_INTERACTIVE 1
#define QOS_BULK 2

/* buffer sizing (overridable at build time, e.g. -DST_MSG_MAX=4096 for
 * the demo app's file transfers; cport defaults stay MCU-modest) */
#ifndef ST_MAX_MSGS
#define ST_MAX_MSGS 8
#endif
#ifndef ST_MSG_MAX
#define ST_MSG_MAX 256
#endif
#ifndef ST_ASM_MAX
#define ST_ASM_MAX 4096
#endif
#ifndef ST_DELIVERED_MAX
#define ST_DELIVERED_MAX 16
#endif
#define ST_LLR_MAX 1024

#define FLAG_LAST_FRAGMENT 1
#define FLAG_NO_DATA 2
#define FLAG_PRIO_STREAM 4

/* burst (selective-repeat) extension: flag combinations that are
 * impossible in the legacy protocol become the two burst frame types */
#define FLAG_BURST_DATA (FLAG_NO_DATA | FLAG_PRIO_STREAM)   /* 6 */
#define FLAG_BURST_ACK (FLAG_NO_DATA | FLAG_LAST_FRAGMENT)  /* 3 */
#define BURST_FRAG_SIZE 25   /* minimum fragment size (low rungs) */
#define BURST_SUBHDR 3       /* [frag_idx][ack_req<<7|total][frag_size] */
#define BURST_MAX_FRAGS 127
#define BURST_MIN_RUNG 4     /* engage at NORMAL rungs and above */

/* per-transfer fragment size, fixed at engage time (uniform size ->
 * random-access placement at idx*frag_size). Larger fragments ride
 * EXT_DATA frames (255-byte payload cap) and amortize the fixed
 * preamble+header cost -- worth ~3x goodput at the top rungs. */
static inline int burst_frag_size_for_rung(int rung_idx)
{
    return rung_idx >= 10 ? 200 : (rung_idx >= 7 ? 100 : BURST_FRAG_SIZE);
}

typedef struct {
    void *ctx;
    /* build a frame at rung; returns sample count (<= out_cap) or <0 */
    int (*build)(void *ctx, const uint8_t *pkt_bits, int pkt_n, int typ,
                 int rung, int16_t *out, int out_cap);
    /* auto-mode receive; 0 = decoded. HARQ pointers may be NULL. */
    int (*receive)(void *ctx, const int16_t *s, int n,
                   uint8_t *pkt_bits, int *pkt_bits_n,
                   double *snr_db, double *cfo_hz, int *harq_combined,
                   const int64_t *prev_llrs, int prev_n,
                   int64_t *llrs_out, int *llrs_n);
} station_phy_t;

typedef struct {
    int tx_frames, rx_frames, retransmissions, timeouts, harq_combines,
        afc_trims;
    int last_rung;
} station_stats_t;

typedef struct {
    uint8_t data[ST_MSG_MAX];
    int len, off, qos, active;
} st_msg_t;

typedef struct {
    uint8_t chunk[32];
    int chunk_len, last, qos, seq, stream, first_try, active;
} st_frag_t;

typedef struct {
    link_ctl_t ctl;
    station_phy_t phy;

    uint8_t qdata[3][ST_MAX_MSGS][ST_MSG_MAX];
    int qlen[3][ST_MAX_MSGS];
    int qhead[3], qcount[3];

    st_msg_t cur_prio, cur_bulk;
    st_frag_t pending;
    int seq, last_rx_seq; /* last_rx_seq: -1 = nothing received yet */

    /* burst ARQ: 0/1 = legacy stop-and-wait; >=2 = selective-repeat
     * window for the bulk stream (send burst_window frames, then one
     * bitmap acknowledgment) */
    int burst_window;
    struct {
        int active, id, n, last_len, window_left, cursor;
        int frag_size; /* uniform per transfer */
        int miss; /* consecutive ack-window misses (1st is forgiven) */
        uint8_t acked[16];
        uint8_t sent[16]; /* sent in the current window (no re-send
                             within a window before the ack arrives) */
    } btx;
    struct {
        int active, id, n, last_len, ack_due, done;
        int frag_size; /* learned from the sub-header */
        uint8_t have[16];
    } brx;

    double await_until; /* <0 = none */
    double not_before;
    int reply_due, expects_reply, reply_rung_guess, last_tx_rung;
    int reply_len_guess; /* expected reply payload bytes (ack bitmap) */

    uint8_t assembly[2][ST_ASM_MAX];
    int assembly_len[2];
    uint8_t delivered[ST_DELIVERED_MAX][ST_MSG_MAX];
    int delivered_len[ST_DELIVERED_MAX];
    int delivered_n;

    int64_t harq_llrs[ST_LLR_MAX];
    int harq_llrs_n; /* 0 = none stored */

    /* AFC */
    void (*freq_trim_cb)(void *ctx, double hz);
    void *trim_ctx;
    double afc_max_trim_hz, afc_total_hz;
    int afc_anchor;
    double last_cfo_hz;
    int has_cfo;

    double turnaround, timeout_margin, backoff_lo, backoff_hi;
    uint64_t rng;
    station_stats_t stats;

    /* diagnostics */
    void (*diag_cb)(void *ctx, int ev, int a, int b, int c, int d,
                    double t);
    void *diag_ctx;
    int diag_last_rung; /* last rung reported via ST_EV_RUNG */
} station_t;

void station_init(station_t *st, const station_phy_t *phy, uint64_t seed);
int station_submit(station_t *st, const uint8_t *data, int len, int qos);
int station_has_traffic(const station_t *st);
/* returns sample count to transmit now, or 0 */
int station_poll_tx(station_t *st, double t, int channel_busy,
                    int16_t *out, int out_cap);
void station_on_tx_end(station_t *st, double t);
/* returns number of messages completed by this burst */
int station_rx_frame(station_t *st, const int16_t *samples, int n, double t);

/* protocol entry after an external PHY decode (e.g. the streaming
 * receiver's events): same semantics as station_rx_frame past the PHY */
int station_on_decoded(station_t *st, const uint8_t *pkt_bits, int pkt_n,
                       double snr_db, double cfo_hz, int harq_combined,
                       double t);

double estimate_air_time(int rung_idx, int payload_len);

/* --- external oscillator fine-tune endpoint ---------------------------
 * In a real setup the reference oscillator is trimmable (VCTCXO DAC,
 * PLL fractional-N word, rig CAT clarifier, ...). Register the actuator
 * here: trim_cb(ctx, hz) must nudge the local LO by the RELATIVE amount
 * `hz` (positive = tune up). The station's AFC netting then drives it
 * from the peer's LC-word frequency requests, keeping the accumulated
 * trim inside +-max_trim_hz. anchor=1 marks this station as the
 * frequency reference: it never auto-trims from peer requests (exactly
 * one side of a link should anchor, or both LOs walk off together);
 * manual trims below still work on an anchor. Pass max_trim_hz <= 0 to
 * keep the current budget (default 150 Hz). */
void station_set_freq_trim(station_t *st,
                           void (*trim_cb)(void *ctx, double hz),
                           void *ctx, double max_trim_hz, int anchor);

/* operator/manual fine-tune through the same budget accounting: clamps
 * so the accumulated trim stays inside the budget, actuates the
 * callback, and returns the delta actually applied (0 if none). */
double station_freq_trim(station_t *st, double hz);

/* accumulated trim currently applied to the oscillator, Hz */
double station_freq_trim_total(const station_t *st);

/* --- diagnostic event stream ------------------------------------------
 * Every internal state switch is reported through an optional callback
 * (args a..d are event-specific, t is protocol time). Costs nothing when
 * unset; meant for debug consoles and flight recorders. */
enum {
    ST_EV_TX = 0,       /* a=rung b=typ c=lc.flags d=payload bytes */
    ST_EV_RX,           /* a=lc.flags b=lc.seq c=lc.ack d=snr_db*10 */
    ST_EV_TIMEOUT,      /* a=losses (after) b=rung c=1 forgiven 1st miss */
    ST_EV_RUNG,         /* a=old b=new c=losses d=cap (change on tx path) */
    ST_EV_BURST_ENGAGE, /* a=nfrags b=frag_size c=transfer id */
    ST_EV_BURST_FRAG,   /* a=frag idx b=ack_req c=window_left (before) */
    ST_EV_BURST_ACKTX,  /* a=transfer id b=bitmap bytes */
    ST_EV_BURST_ACKRX,  /* a=frags acked (total) b=nfrags */
    ST_EV_BURST_PROBE,  /* timeout inside a burst -> 1-frame probe */
    ST_EV_BURST_DONE,   /* a=0 tx-side complete, 1 rx-side delivered */
};
void station_set_diag(station_t *st,
                      void (*cb)(void *ctx, int ev, int a, int b, int c,
                                 int d, double t),
                      void *ctx);
const char *station_diag_name(int ev); /* short label, e.g. "RUNG" */

#endif /* OFDM_STATION_H */
