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
/* Slots in the shared message store. Every message the station holds --
 * queued, in flight, or delivered and not yet drained -- occupies one.
 *
 * Before this, each of the 42 positions that COULD hold a message owned
 * ST_MSG_MAX bytes outright: 3x8 queue slots, 16 delivered entries, and
 * the two current messages. That sized RAM by the number of ADDRESSABLE
 * positions rather than by how many messages can exist at once, and at
 * ST_MSG_MAX=4096 it was 172 kB inside a station_t that is otherwise
 * about 25 kB. The positions are still all there; they now hold a slot
 * index instead of a copy.
 *
 * Slots are fixed size and never move, so a message stays contiguous
 * and `data + idx*frag_size` fragment arithmetic still works. A full
 * store fails station_submit, which is the same back-pressure a full
 * queue has always applied. Measured peak across the C suites and the
 * demoapp file transfers is reported in `pool_hwm`. */
#ifndef ST_POOL_SLOTS
#define ST_POOL_SLOTS 12
#endif
#define ST_LLR_MAX 1024

#define FLAG_LAST_FRAGMENT 1
#define FLAG_NO_DATA 2
#define FLAG_PRIO_STREAM 4

/* burst (selective-repeat) extension: flag combinations that are
 * impossible in the legacy protocol become the two burst frame types */
#define FLAG_BURST_DATA (FLAG_NO_DATA | FLAG_PRIO_STREAM)   /* 6 */
#define FLAG_BURST_ACK (FLAG_NO_DATA | FLAG_LAST_FRAGMENT)  /* 3 */
/* Capability handshake. A third impossible flag combination carries a
 * 10-byte record of what a station can do, so the peer's abilities are
 * DECLARED instead of discovered by failing at them (streaming used to
 * be learned from a window that came back one-eighth acked, and the
 * fragment size guessed from OUR limits rather than the receiver's).
 *
 *   A -> B  CAPS            when A has bulk to send and knows nothing
 *   B -> A  CAPS | CAP_ACK  B stores A's record, answers with its own
 *   A -> B  (any frame)     A's ack of B's seq is the third leg: B now
 *                           knows A holds B's record
 *
 * An older peer reads flags 7 as no-data (it ignores the payload and
 * does not reply), so the probe is FORGIVEN by the rate controller and
 * after CAPS_TRIES the peer is assumed legacy and today's defaults
 * apply -- the fallback property streaming already relied on. */
#define FLAG_CAPS (FLAG_NO_DATA | FLAG_LAST_FRAGMENT | FLAG_PRIO_STREAM) /* 7 */
#define CAPS_VER 1
#define CAPS_LEN 10          /* ver flags msg_max(2) win pool fw(2) frags
                              * max_rung+1 (0 = unspecified: older record) */
#define CAP_STREAM (1 << 0)  /* can follow streamed burst windows */
#define CAP_EXT    (1 << 1)  /* EXT_DATA frames (255-byte payloads) */
#define CAP_LDPC   (1 << 2)
#define CAP_BURST  (1 << 3)  /* selective-repeat bursts at all */
#define CAP_BCAST  (1 << 4)  /* receives broadcasts (firmware sets it) */
#define CAP_BC_STATS (1 << 5) /* will ANSWER a broadcast's end-of-block
                               * marker with a stats frame (typ BCSTAT):
                               * frames ok, lost, SNR, desired rung */
#define CAP_ACK    (1 << 7)  /* this record answers yours */
#define CAPS_TRIES 2         /* unanswered probes before "legacy peer" */
#define CAPS_RETRY_S 300.0   /* ...and how long that verdict holds */
#define CAPS_STALE_S 900.0   /* a record older than this is re-asked */

typedef struct {
    int valid;      /* a record has been received */
    int legacy;     /* probes went unanswered: assume nothing */
    int flags, msg_max, win_max, pool_slots, fw_ver, max_frags;
    int max_rung;   /* fastest rung the peer accepts; -1 = unspecified */
    double t;       /* when the record arrived */
} st_caps_t;

#define BURST_FRAG_SIZE 25   /* minimum fragment size (low rungs) */
#define BURST_SUBHDR 3       /* [streamed<<7|frag_idx][ack_req<<7|total][frag_size] */
#define BURST_MAX_FRAGS 127
#define BURST_MIN_RUNG 4     /* engage at NORMAL rungs and above */

/* streamed bursts: a whole window behind ONE preamble+header instead of
 * one preamble per fragment (docs/phy.md). Bit 7 of the sub-header's
 * index byte marks a fragment as part of a stream -- frag_idx only ever
 * needs 7 bits (BURST_MAX_FRAGS 127), and the packet bytes are otherwise
 * unchanged, so a receiver without streaming support still decodes the
 * first block of the burst as an ordinary fragment. */
#define BURST_SUB_STREAMED 0x80
#ifndef BURST_STREAM_RESYNC
#define BURST_STREAM_RESYNC 4  /* ZC resync period, in blocks */
#endif
/* blocks per stream. Sizes two static buffers (TX packet bits ~2.1 kB
 * each, RX reassembly 2.6 kB each), so raising it costs ~4.7 kB/block --
 * check FEASIBILITY.md's RAM budget before turning it up on an MCU. */
#ifndef BURST_STREAM_MAX
#define BURST_STREAM_MAX 8
#endif

/* No burst fragment may be SENT whose air time exceeds this, whatever
 * rung the transfer was engaged at. frag_size is fixed at engage and
 * uniform per transfer, so a transfer engaged at rung 10 with 203-byte
 * frags that collapses to rung 0 would otherwise emit 224-second
 * frames -- which violate every carrier-sense time constant at once
 * (the peer's noise floor climbs through the busy threshold after
 * ~176 s of continuous signal, and the rebase fires at 300 s), so the
 * peer keys over the frame and the link death-spirals. Measured on the
 * two-board 8 kB stress transfer. 45 s clears every legitimately
 * engaged fragment (engage sizes frags FOR its rung) and sits under
 * both constants. */
#define BURST_FRAG_MAX_AIR_S 45.0
#define BURST_STREAM_MIN 2     /* below this a stream saves nothing */
/* consecutive streamed windows that deliver ~nothing before the station
 * gives up and reverts to per-frame bursts for the rest of the transfer */
#define BURST_STREAM_STRIKES 2
/* Reply-timer allowance after a streamed window: the peer commits the
 * whole stream in one rxs_push (~2.5 s measured) and then waits out
 * carrier sense before its bitmap -- a stall, not a latency the RFC-6298
 * estimator should learn (13/34 windows timed out at the learned 1.2 s
 * budget on the 68 kB stand run; the acks arrived ~5 s later). */
#define RTO_STREAM_COMMIT_S 5.0

/* One acknowledgment covers a whole window, so a single transmission
 * must never outlast a plausible fade -- everything in it is exposed
 * before any of it is acked. This caps the window by air time whatever
 * else asks for. */
#define BURST_WIN_MAX_AIR_S 30.0

/* why a station stopped streaming (ST_EV_BURST_SOFF payload) */
#define ST_SOFF_BUILD   1  /* the PHY refused to build the burst */
#define ST_SOFF_NOACK   2  /* windows streamed, ~nothing came back acked */
#define ST_SOFF_TIMEOUT 3  /* streamed windows kept timing out */

/* Only ST_SOFF_NOACK is a statement about the PEER: a bitmap that acks
 * one fragment out of a window of eight is what a receiver unable to
 * follow a stream looks like, and that does not change between
 * transfers. ST_SOFF_TIMEOUT and ST_SOFF_BUILD are statements about the
 * CHANNEL and the local buffers, so they stay per-transfer -- measured:
 * a fading channel produces ST_SOFF_TIMEOUT against a peer that streams
 * perfectly well, and remembering that would disable streaming forever
 * on a capable link.
 *
 * The peer verdict is not permanent either: a deep fade can forge the
 * NOACK signature, so streaming is probed again after this many burst
 * transfers. Wrong "peer cannot stream" costs one window per retry
 * period; a peer that really cannot costs two windows per period
 * instead of two per transfer. */
#define PEER_STREAM_RETRY 8

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

    /* OPTIONAL streamed bursts. Either may be NULL -- the station then
     * uses per-frame bursts, which is also where it falls back to when a
     * stream stops delivering. build_burst emits n_blocks equal-size
     * packets behind one preamble+header (returns samples, <=0 on
     * refusal); receive_burst decodes as many blocks as the buffer holds,
     * writing pkt_bits block-major and one CRC flag per block, and
     * returns the block count examined (<0 = no lock). */
    int (*build_burst)(void *ctx, const uint8_t *blocks, int pkt_n,
                       int n_blocks, int typ, int rung, int resync_every,
                       int16_t *out, int out_cap);
    int (*receive_burst)(void *ctx, const int16_t *s, int n, int max_blocks,
                         int resync_every, uint8_t *pkt_bits, int *pkt_bits_n,
                         int *ok_flags, double *snr_db, double *cfo_hz);
} station_phy_t;

typedef struct {
    int tx_frames, rx_frames, retransmissions, timeouts, harq_combines,
        afc_trims;
    int last_rung;
    /* the window a burst actually engaged with, kept as a STATISTIC:
     * btx.win itself is dropped when the burst disengages, and reading
     * it afterwards only worked while that state was leaking */
    int last_burst_win;
} station_stats_t;

typedef struct {
    int slot;  /* index into station_t.pool, -1 = holds nothing */
    int len, off, qos, active;
} st_msg_t;

typedef struct {
    uint8_t chunk[32];
    int chunk_len, last, qos, seq, stream, first_try, active;
} st_frag_t;

typedef struct {
    link_ctl_t ctl;
    station_phy_t phy;

    /* the shared message store and its free list (see ST_POOL_SLOTS) */
    uint8_t pool[ST_POOL_SLOTS][ST_MSG_MAX];
    int pool_next[ST_POOL_SLOTS];
    int pool_head;             /* free-list head, -1 = exhausted */
    int pool_used, pool_hwm;   /* live slots, and the peak ever live */

    int qslot[3][ST_MAX_MSGS]; /* -1 where the queue position is empty */
    int qlen[3][ST_MAX_MSGS];
    int qhead[3], qcount[3];

    st_msg_t cur_prio, cur_bulk;
    st_frag_t pending;
    int seq, last_rx_seq; /* last_rx_seq: -1 = nothing received yet */

    /* burst ARQ: 0/1 = legacy stop-and-wait; >=2 = selective-repeat
     * window for the bulk stream (send burst_window frames, then one
     * bitmap acknowledgment) */
    int burst_window;
    /* stream whole windows behind one preamble when the PHY offers
     * build_burst/receive_burst. Opt-in: 0 keeps the per-frame burst
     * behaviour (and is what every window degrades to on failure). */
    int burst_stream;
    /* peer capability, remembered across transfers (see
     * PEER_STREAM_RETRY): 1 = believed able to follow streamed bursts */
    int peer_stream_ok, peer_stream_retry;
    /* capability handshake (see FLAG_CAPS) */
    st_caps_t peer;        /* what the peer declared */
    int my_caps;           /* what we declare (CAP_*, set by the caller) */
    /* operator knobs, declared in the record and enforced locally:
     * my_win_max caps the streamed window we ASK a peer to follow (and
     * the one we send); my_max_rung is the fastest rung we transmit at
     * or request -- the UP_CFG_RUNG_CEILING key, which was documented
     * for a year and implemented nowhere */
    int my_win_max;        /* 1..BURST_STREAM_MAX */
    int my_max_rung;       /* 0..ladder_n()-1 */
    int caps_reply_due;    /* leg 2 owed */
    int caps_sent;         /* our record has gone out at least once */
    int caps_confirmed;    /* the peer acked the frame that carried it */
    int caps_inflight;     /* a CAPS frame awaits its reply (forgiven) */
    int caps_tries, caps_seq;
    int caps_kick;         /* fire the caps probe without bulk waiting
                            * (a held broadcast probes with CAPS: the
                            * exchange moves the ladder AND fills the
                            * record) -- consumed by the next probe */
    int fw_ver;            /* advertised in the record; caller sets */
    int caps_disabled;     /* test knob: behave like a peer that predates
                            * the handshake (ignore CAPS, never send) */
    double caps_next_t;    /* not before: retry pacing, legacy re-probe */
    struct {
        int active, id, n, last_len, window_left, cursor;
        int frag_size; /* uniform per transfer */
        int miss; /* consecutive ack-window misses (1st is forgiven) */
        uint8_t acked[16];
        uint8_t sent[16]; /* sent in the current window (no re-send
                             within a window before the ack arrives) */
        /* streaming state: stream_ok goes to 0 for the rest of the
         * transfer once the evidence says the peer is not following the
         * stream, and every later window is sent as separate frames */
        int stream_ok, streamed_n, stream_strikes;
        /* fragments behind one acknowledgment for this transfer, fixed
         * at engage: min(operator ceiling, buffer cap, fragment count),
         * then clipped by the air-time cap. Sizing it to the transfer is
         * what makes a short transfer cost exactly one acknowledgment
         * (measured: 9 transmissions -> 6 on a 14 KB file). Resizing it
         * DURING a transfer was tried and reverted -- see the timeout
         * path in station.c. */
        int win;
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

    /* Adaptive reply timer. The budget splits in two: the reply's AIR
     * TIME is computable physics (estimate_air_time, and it swings 40x
     * across the ladder), while everything else -- peer turnaround,
     * decode time, carrier-sense wait, scheduling, and the error in
     * reply_rung_guess -- is latency we can only measure. So the air
     * term stays exact and only the overhead is smoothed, RFC 6298
     * style, with Karn's rule (no sample from an exchange that involved
     * a retransmission) and exponential backoff on loss.
     * turnaround + timeout_margin remain the bootstrap value. */
    double tx_end_t, rto_air_est, rto_srtt, rto_rttvar, rto_backoff;
    int rto_have, rto_pending, rto_ambiguous;

    uint8_t assembly[2][ST_ASM_MAX];
    int assembly_len[2];
    int delivered_slot[ST_DELIVERED_MAX];
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

/* --- delivered messages -----------------------------------------------
 * Completed inbound messages accumulate in a bounded log; the caller
 * reads entries [0, delivered_n) and then releases them. Payloads live
 * in the shared store, so the log MUST be reset through this call --
 * zeroing delivered_n by hand strands the slots. */
const uint8_t *station_delivered(const station_t *st, int i);
void station_delivered_reset(station_t *st);

/* Drop the bulk message in flight together with its burst state, and
 * return its storage. A transfer can be abandoned from outside: an
 * operator cancel, or setting up a fresh one over the top. */
void msg_release_for_test(station_t *st);   /* tests only */
void station_abort_bulk(station_t *st);

/* Sizing diagnostics for ST_POOL_SLOTS, process-wide across every
 * station: the peak slots ever live, and the number of allocations
 * refused because the store was full (must be 0). */
int station_pool_peak(void);
int station_pool_refused(void);
/* slots released twice -- must be 0; see pool_free() for why it matters */
int station_pool_double_free(void);

/* Slots still free. A caller about to submit several messages at once
 * should check this the way it checks queue depth -- the store is a
 * capacity like any other, and refusing up front beats a partial
 * submission half way through a file. */
int station_pool_free(const station_t *st);
int station_has_traffic(const station_t *st);
/* The rung the next bulk frame would go out at, as of t: the controller
 * clamped by the operator ceiling and the peer's declared one. This is
 * NOT stats.last_rung, which is a record of the last frame actually
 * transmitted and can be hours stale -- a station idle overnight
 * reported "rung 12" and then sent at rung 0. */
int station_tx_rung(const station_t *st, double t);
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
/* air time of one streamed window (the fixed preamble+header once, then
 * n_blocks of data) -- exposed for tests and air-time planning */
double stream_air_time_pub(int rung_idx, int payload_len, int n_blocks);
/* test hook: force the streaming fallback with a given reason */
void burst_stream_off_pub(station_t *st, int reason, double t);

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
    ST_EV_BURST_STREAM, /* a=blocks streamed b=samples c=resync period */
    ST_EV_BURST_SRX,    /* a=blocks decoded b=blocks examined */
    ST_EV_BURST_SOFF,   /* streaming abandoned: a=ST_SOFF_* */
    ST_EV_RTO,          /* reply timer: a=srtt ms b=rttvar ms c=budget ms
                           d=air-time term ms */
    ST_EV_BURST_WIN,    /* window chosen at engage: a=ceiling b=used
                           c=fragments d=burst air time, s */
    ST_EV_BURST_REFRAG, /* frag air time over cap at the CURRENT rung:
                         * burst disengaged, legacy path carries the
                         * message. a=frag_size b=rung c=nfrags */
    ST_EV_CAPS,         /* a=0 sent b=1 sent as reply 2=received
                         * 3=peer assumed legacy; b=flags c=msg_max
                         * d=win_max */
};
void station_set_diag(station_t *st,
                      void (*cb)(void *ctx, int ev, int a, int b, int c,
                                 int d, double t),
                      void *ctx);
const char *station_diag_name(int ev);
/* Render one diagnostic event as a human-readable line ("burst engage:
 * 2 frag(s) x 200 B, transfer 1" rather than "a=2 b=200 c=1"), using
 * the field semantics documented on the enum above. Every console
 * should print THIS -- the raw a..d numbers are write-only. */
void station_diag_format(int ev, int a, int b, int c, int d,
                         char *out, int cap); /* short label, e.g. "RUNG" */

#endif /* OFDM_STATION_H */
