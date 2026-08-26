#include <string.h>

#include "station.h"
#include "packets.h"
#include "rom_modes.h"

static const double QOS_MAX_AIR_S[3] = { 4.0, 6.0, 8.0 };
static const int DET_T[3] = { DET_T_NORMAL, DET_T_ROBUST, DET_T_EXTREME };
static const int SYM_TILE[3] = { SYM_TILE_NORMAL, SYM_TILE_ROBUST,
                                 SYM_TILE_EXTREME };

#define AFC_DEADBAND_HZ 12.0
#define AFC_GAIN 0.5

/* Adaptive reply timer (RFC 6298 shape, applied to the OVERHEAD only --
 * the air-time term is computed exactly, see station.h). */
#define RTO_ALPHA 0.125      /* srtt gain */
#define RTO_BETA 0.25        /* rttvar gain */
#define RTO_K 4.0            /* variance multiplier */
#define RTO_MIN_SLACK 0.5    /* s, floor on the learned overhead */
#define RTO_MAX_SLACK 30.0   /* s, ceiling (EXTREME turnarounds are slow) */
#define RTO_MAX_BACKOFF 8.0

static double rto_slack(const station_t *st)
{
    double s = st->rto_have ? st->rto_srtt + RTO_K * st->rto_rttvar
                            : st->turnaround + st->timeout_margin;
    s *= st->rto_backoff > 1.0 ? st->rto_backoff : 1.0;
    if (s < RTO_MIN_SLACK)
        s = RTO_MIN_SLACK;
    if (s > RTO_MAX_SLACK)
        s = RTO_MAX_SLACK;
    return s;
}

/* One round-trip observation, Karn-gated by the caller. */
static void rto_sample(station_t *st, double overhead)
{
    if (overhead < 0.0)
        overhead = 0.0;
    if (!st->rto_have) {
        st->rto_srtt = overhead;
        st->rto_rttvar = overhead / 2.0;
        st->rto_have = 1;
    } else {
        double d = st->rto_srtt - overhead;
        if (d < 0.0)
            d = -d;
        st->rto_rttvar = (1.0 - RTO_BETA) * st->rto_rttvar + RTO_BETA * d;
        st->rto_srtt = (1.0 - RTO_ALPHA) * st->rto_srtt + RTO_ALPHA * overhead;
    }
    st->rto_backoff = 1.0;
    st->peer_stream_ok = 1; /* optimistic until a bitmap says otherwise */
}

double estimate_air_time(int rung_idx, int payload_len)
{
    link_mode_t mode = ladder_mode(rung_idx);
    int symbol_len = CP_LEN + SYM_TILE[mode] * FFT_BINS;
    int preamble = 3 * DET_T[mode] * FFT_BINS + symbol_len;
    int n_hdr = (conv_cc_elements(CC_R13, HEADER_BITS) + N_DATA_CARRIERS - 1)
                / N_DATA_CARRIERS;
    int bits = 20 + 8 * payload_len + 16;
    int mu = ladder_mod(rung_idx) == MOD_QAM16
                 ? 4 : (ladder_mod(rung_idx) == MOD_QPSK ? 2 : 1);
    int cap = N_DATA_CARRIERS * mu;
    int coded = conv_cc_elements(ladder_spd(rung_idx), bits);
    int n_data = (coded + cap - 1) / cap;
    return (double)(preamble + (n_hdr + n_data) * symbol_len) / 12000.0;
}

static double rng_uniform(station_t *st, double lo, double hi)
{
    st->rng = st->rng * 6364136223846793005ull + 1442695040888963407ull;
    return lo + (hi - lo) * ((double)(st->rng >> 11) / 9007199254740992.0);
}

void station_init(station_t *st, const station_phy_t *phy, uint64_t seed)
{
    memset(st, 0, sizeof(*st));
    ctl_init(&st->ctl);
    st->phy = *phy;
    st->last_rx_seq = -1;
    st->last_tx_rung = -1;
    st->await_until = -1.0;
    st->turnaround = 0.3;
    st->timeout_margin = 2.0;
    st->backoff_lo = 1.0;
    st->backoff_hi = 6.0;
    st->afc_max_trim_hz = 150.0;
    st->rng = seed ? seed : 1;
    st->diag_last_rung = -1;
    st->rto_backoff = 1.0;
    st->peer_stream_ok = 1; /* optimistic until a bitmap says otherwise */
}

static void diag(station_t *st, int ev, int a, int b, int c, int d,
                 double t)
{
    if (st->diag_cb)
        st->diag_cb(st->diag_ctx, ev, a, b, c, d, t);
}

void station_set_diag(station_t *st,
                      void (*cb)(void *ctx, int ev, int a, int b, int c,
                                 int d, double t),
                      void *ctx)
{
    st->diag_cb = cb;
    st->diag_ctx = ctx;
}

const char *station_diag_name(int ev)
{
    static const char *names[] = {
        "TX", "RX", "TIMEOUT", "RUNG", "BURST_ENGAGE", "BURST_FRAG",
        "BURST_ACKTX", "BURST_ACKRX", "BURST_PROBE", "BURST_DONE",
        "BURST_STREAM", "BURST_SRX", "BURST_SOFF", "RTO",
        "BURST_WIN",
    };
    return ev >= 0 && ev < (int)(sizeof(names) / sizeof(names[0]))
               ? names[ev]
               : "?";
}

/* report a tx-rung change with the controller inputs that caused it */
static void diag_rung(station_t *st, int rung, double t)
{
    if (st->diag_cb && rung != st->diag_last_rung) {
        link_diag_t d;
        ctl_diag(&st->ctl, t, &d);
        diag(st, ST_EV_RUNG, st->diag_last_rung, rung, d.losses, d.cap, t);
    }
    st->diag_last_rung = rung;
}

void station_set_freq_trim(station_t *st,
                           void (*trim_cb)(void *ctx, double hz),
                           void *ctx, double max_trim_hz, int anchor)
{
    st->freq_trim_cb = trim_cb;
    st->trim_ctx = ctx;
    if (max_trim_hz > 0.0)
        st->afc_max_trim_hz = max_trim_hz;
    st->afc_anchor = anchor;
    st->afc_total_hz = 0.0; /* new actuator starts at its rest frequency */
}

double station_freq_trim(station_t *st, double hz)
{
    double nt = st->afc_total_hz + hz, delta;
    if (nt > st->afc_max_trim_hz)
        nt = st->afc_max_trim_hz;
    if (nt < -st->afc_max_trim_hz)
        nt = -st->afc_max_trim_hz;
    delta = nt - st->afc_total_hz;
    if (delta != 0.0) {
        if (st->freq_trim_cb)
            st->freq_trim_cb(st->trim_ctx, delta);
        st->afc_total_hz = nt;
    }
    return delta;
}

double station_freq_trim_total(const station_t *st)
{
    return st->afc_total_hz;
}

int station_submit(station_t *st, const uint8_t *data, int len, int qos)
{
    int slot;
    if (len > ST_MSG_MAX || st->qcount[qos] >= ST_MAX_MSGS)
        return -1;
    slot = (st->qhead[qos] + st->qcount[qos]) % ST_MAX_MSGS;
    memcpy(st->qdata[qos][slot], data, (size_t)len);
    st->qlen[qos][slot] = len;
    st->qcount[qos]++;
    return 0;
}

int station_has_traffic(const station_t *st)
{
    return st->pending.active || st->cur_prio.active || st->cur_bulk.active
           || st->btx.active || st->qcount[0] || st->qcount[1]
           || st->qcount[2];
}

/* ---- burst (selective-repeat) helpers ---- */

static void pop_msg(station_t *st, int qos, st_msg_t *dst);

static int burst_all_acked(const station_t *st)
{
    int i;
    for (i = 0; i < st->btx.n; i++)
        if (!(st->btx.acked[i >> 3] & (1 << (i & 7))))
            return 0;
    return 1;
}

/* next fragment that is neither acked nor already sent this window */
static int burst_next_candidate(const station_t *st, int from)
{
    int i, idx;
    for (i = 0; i < st->btx.n; i++) {
        idx = (from + i) % st->btx.n;
        if (!(st->btx.acked[idx >> 3] & (1 << (idx & 7)))
            && !(st->btx.sent[idx >> 3] & (1 << (idx & 7))))
            return idx;
    }
    return -1;
}

static int burst_candidates_left(const station_t *st)
{
    int i, c = 0;
    for (i = 0; i < st->btx.n; i++)
        if (!(st->btx.acked[i >> 3] & (1 << (i & 7)))
            && !(st->btx.sent[i >> 3] & (1 << (i & 7))))
            c++;
    return c;
}

/* Air time of one streamed window: the fixed preamble+header is paid
 * once, then n_blocks data blocks. (estimate_air_time charges the fixed
 * cost per frame, so subtract it back out for the blocks after the
 * first -- that saving is the whole point of streaming.) */
static double stream_air_time(int rung_idx, int payload_len, int n_blocks)
{
    double one = estimate_air_time(rung_idx, payload_len);
    double fixed = estimate_air_time(rung_idx, 0);
    double data = one - fixed;
    if (data < 0.0)
        data = 0.0;
    return fixed + data * (double)n_blocks;
}

double stream_air_time_pub(int rung_idx, int payload_len, int n_blocks)
{
    return stream_air_time(rung_idx, payload_len, n_blocks);
}

/* Largest window that stays inside the air-time cap. Never returns 0:
 * one fragment must always be sendable, whatever it costs. */
static int burst_win_air_cap(int rung_idx, int payload_len, int want)
{
    while (want > 1
           && stream_air_time(rung_idx, payload_len, want)
                  > BURST_WIN_MAX_AIR_S)
        want--;
    return want;
}

/* engage burst mode for the current/next bulk message if eligible */
static int burst_try_engage(station_t *st, int rung_idx)
{
    int n;
    if (st->burst_window < 2 || st->btx.active || st->pending.active
        || st->cur_prio.active || st->qcount[0] || st->qcount[1])
        return 0;
    if (rung_idx < BURST_MIN_RUNG)
        return 0;
    if (!st->cur_bulk.active) {
        if (!st->qcount[QOS_BULK])
            return 0;
        pop_msg(st, QOS_BULK, &st->cur_bulk);
    }
    {
        int fs = burst_frag_size_for_rung(rung_idx);
        n = (st->cur_bulk.len + fs - 1) / fs;
        if (n > BURST_MAX_FRAGS)
            return 0; /* too large for one transfer -> legacy fragments */
        st->btx.frag_size = fs;
        st->btx.last_len = st->cur_bulk.len - (n - 1) * fs;
    }
    st->btx.active = 1;
    st->btx.id = (st->btx.id + 1) & 3;
    st->btx.n = n;
    {   /* start optimistic: cover the whole transfer if the operator
         * ceiling and the air-time cap allow it, so a short transfer on
         * a good channel costs exactly one acknowledgment */
        int w = st->burst_window;
        if (w > BURST_STREAM_MAX)
            w = BURST_STREAM_MAX;
        if (w > n)
            w = n;
        st->btx.win = burst_win_air_cap(rung_idx, st->btx.frag_size, w);
        diag(st, ST_EV_BURST_WIN, st->burst_window, st->btx.win, n,
             (int)stream_air_time(rung_idx, st->btx.frag_size,
                                  st->btx.win), 0.0);
    }
    st->btx.window_left = st->btx.win;
    st->btx.cursor = 0;
    st->btx.miss = 0;
    {   /* a peer that has shown it cannot follow a stream is not asked
         * again for a while -- without this the station pays two failed
         * windows re-learning the same thing on every transfer */
        int can = st->burst_stream && st->phy.build_burst != 0;
        if (!st->peer_stream_ok) {
            if (st->peer_stream_retry > 0)
                st->peer_stream_retry--;
            if (st->peer_stream_retry == 0)
                st->peer_stream_ok = 1; /* time to probe again */
            else
                can = 0;
        }
        st->btx.stream_ok = can;
    }
    st->btx.streamed_n = 0;
    st->btx.stream_strikes = 0;
    memset(st->btx.acked, 0, sizeof(st->btx.acked));
    memset(st->btx.sent, 0, sizeof(st->btx.sent));
    return 1;
}

static void pop_msg(station_t *st, int qos, st_msg_t *dst)
{
    int slot = st->qhead[qos];
    memcpy(dst->data, st->qdata[qos][slot], (size_t)st->qlen[qos][slot]);
    dst->len = st->qlen[qos][slot];
    dst->off = 0;
    dst->qos = qos;
    dst->active = 1;
    st->qhead[qos] = (st->qhead[qos] + 1) % ST_MAX_MSGS;
    st->qcount[qos]--;
}

/* Air-time budget -> payload cap.  The budget has to be judged on the
 * WHOLE frame, not on the payload's own transmission time: at the low
 * rungs the fixed preamble+header (16.8 s at EXTREME) already exceeds
 * every QoS budget, so splitting a message cannot bring a frame under
 * budget -- it only pays that fixed cost once per fragment.  Measured
 * before this guard: a 22-byte message at rung 0 went out as six
 * 5-byte frames (6x16.8 s of preamble) instead of one 38 s frame,
 * ~4x the air time.  Where the fixed cost already blows the budget,
 * fragmenting buys nothing, so send the largest packet the format
 * allows; elsewhere the original rate-based cap is unchanged. */
static int payload_cap(int rung_idx, double max_air_s)
{
    if (estimate_air_time(rung_idx, 1) >= max_air_s)
        return 27;
    return link_max_payload_bytes(rung_idx, max_air_s);
}

/* ---- streamed bursts ----
 * A whole window behind one preamble+header instead of one preamble per
 * fragment. Everything above the PHY is unchanged: the packets are the
 * same bytes a per-frame burst would send, with bit 7 of the sub-header's
 * index byte marking them as streamed.
 *
 * Deliberate restriction: a stream carries only FULL-SIZE fragments. The
 * receiver learns the message length from the last fragment's own length
 * (`brx.last_len`), and every block of a stream must be the same size for
 * one header to describe them, so the short final fragment always travels
 * as an ordinary frame. It costs one frame per transfer and keeps the
 * packet format byte-identical. */
static int burst_stream_ready(const station_t *st)
{
    return st->burst_stream && st->btx.stream_ok && st->phy.build_burst
           && st->btx.window_left >= BURST_STREAM_MIN;
}

/* give up on streaming for the rest of this transfer */
static void burst_stream_off(station_t *st, int reason, double t)
{
    int sticky = reason == ST_SOFF_NOACK;
    if (!st->btx.stream_ok)
        return;
    st->btx.stream_ok = 0;
    st->btx.streamed_n = 0;
    if (sticky) { /* a peer property, not a channel one -- remember it */
        st->peer_stream_ok = 0;
        st->peer_stream_retry = PEER_STREAM_RETRY;
    }
    diag(st, ST_EV_BURST_SOFF, reason, sticky, st->peer_stream_retry, 0, t);
}

void burst_stream_off_pub(station_t *st, int reason, double t)
{
    burst_stream_off(st, reason, t);
}

/* bits in one streamed packet: 20 reserved + payload + 16 CRC */
#define ST_STREAM_PKT_BITS (36 + 8 * (BURST_SUBHDR + 253))

/* Build and hand over one streamed window. Returns the sample count, or 0
 * to mean "not this time" -- every such exit leaves the burst state
 * untouched so the caller just sends the next fragment as its own frame. */
static int burst_send_stream(station_t *st, int rung_idx, int16_t *out,
                             int out_cap, double t)
{
    static uint8_t blocks[BURST_STREAM_MAX * ST_STREAM_PKT_BITS];
    uint8_t payload[BURST_SUBHDR + 253];
    int idxs[BURST_STREAM_MAX];
    lc_word_t lc;
    int fs = st->btx.frag_size;
    int pkt_n = 36 + 8 * (BURST_SUBHDR + fs);
    int count = 0, i, k, n;

    if (fs > 253 || pkt_n > ST_STREAM_PKT_BITS)
        return 0;

    /* full-size fragments not yet acked and not yet sent in this window;
     * the short tail is excluded on purpose (see the note above) */
    for (i = 0; i < st->btx.n && count < BURST_STREAM_MAX
                && count < st->btx.window_left; i++) {
        int idx = (st->btx.cursor + i) % st->btx.n;
        if (st->btx.acked[idx >> 3] & (1 << (idx & 7)))
            continue;
        if (st->btx.sent[idx >> 3] & (1 << (idx & 7)))
            continue;
        if (idx == st->btx.n - 1 && st->btx.last_len != fs)
            continue;
        idxs[count++] = idx;
    }
    if (count < BURST_STREAM_MIN)
        return 0;

    lc.seq = st->btx.id;
    lc.ack = st->last_rx_seq >= 0 ? st->last_rx_seq : 0;
    lc.req_rung = ctl_rx_request(&st->ctl, t);
    lc.snr_db = ctl_filtered_snr(&st->ctl, t);
    lc.freq_corr_hz = 0.0;
    lc.flags = FLAG_BURST_DATA;
    for (k = 0; k < count; k++) {
        int idx = idxs[k];
        payload[0] = (uint8_t)(idx | BURST_SUB_STREAMED);
        /* the ack request rides on the last block AND on the first: a peer
         * that cannot follow the stream decodes only block 0, and without
         * this it would never reply at all -- the sender would learn of
         * the failure by timeout instead of by a bitmap that says
         * "1 of 8", which is the signal the fallback wants. A peer that
         * does follow the stream still sends exactly one ack, after every
         * block of the burst has been processed. */
        payload[1] = (uint8_t)((k == count - 1 || k == 0 ? 0x80 : 0)
                               | st->btx.n);
        payload[2] = (uint8_t)fs;
        memcpy(payload + BURST_SUBHDR, st->cur_bulk.data + idx * fs,
               (size_t)fs);
        n = data_encode(lc_pack(&lc), payload, BURST_SUBHDR + fs,
                        blocks + (size_t)k * pkt_n);
        if (n != pkt_n)
            return 0; /* size assumption broken: send frames instead */
    }

    n = st->phy.build_burst(st->phy.ctx, blocks, pkt_n, count,
                            PKT_TYP_EXT_DATA, rung_idx, BURST_STREAM_RESYNC,
                            out, out_cap);
    if (n <= 0) {
        /* the PHY refused (too long for the buffer, bad rung, ...) --
         * one strike is enough here, the refusal is deterministic */
        burst_stream_off(st, ST_SOFF_BUILD, t);
        return 0;
    }

    for (k = 0; k < count; k++) {
        st->btx.sent[idxs[k] >> 3] |= (uint8_t)(1 << (idxs[k] & 7));
        /* report the ack_req bit that actually went out, not just
         * "is last": the receiver uses it as the end-of-burst signal, so
         * a diagnostic that disagrees with the wire is worse than none */
        diag(st, ST_EV_BURST_FRAG, idxs[k], k == count - 1 || k == 0,
             st->btx.window_left, 0, t);
    }
    st->btx.cursor = idxs[count - 1] + 1;
    st->btx.window_left = 0; /* the last block carried the ack request */
    st->btx.streamed_n = count;
    diag_rung(st, rung_idx, t);
    diag(st, ST_EV_BURST_STREAM, count, n, BURST_STREAM_RESYNC, 0, t);
    diag(st, ST_EV_TX, rung_idx, PKT_TYP_EXT_DATA, lc.flags,
         BURST_SUBHDR + fs, t);
    st->stats.tx_frames++;
    st->stats.last_rung = rung_idx;
    st->last_tx_rung = rung_idx;
    st->reply_due = 0;
    st->expects_reply = 1;
    st->reply_rung_guess = ctl_rx_request(&st->ctl, t) - 2;
    if (st->reply_rung_guess < 0)
        st->reply_rung_guess = 0;
    st->reply_len_guess = (st->btx.n + 7) / 8;
    return n;
}

/* priority stream preempts bulk at fragment boundaries */
static st_frag_t *take_fragment(station_t *st, int rung_idx)
{
    st_msg_t *src;
    int cap, chunk_len, q;

    if (st->pending.active)
        return &st->pending;
    if (!st->cur_prio.active)
        for (q = QOS_CONTROL; q <= QOS_INTERACTIVE; q++)
            if (st->qcount[q]) {
                pop_msg(st, q, &st->cur_prio);
                break;
            }
    if (!st->cur_prio.active && !st->cur_bulk.active && st->qcount[QOS_BULK])
        pop_msg(st, QOS_BULK, &st->cur_bulk);

    src = st->cur_prio.active ? &st->cur_prio
                              : (st->cur_bulk.active ? &st->cur_bulk : 0);
    if (!src)
        return 0;

    cap = payload_cap(rung_idx, QOS_MAX_AIR_S[src->qos]);
    chunk_len = src->len - src->off;
    if (chunk_len > cap)
        chunk_len = cap;
    st->seq = (st->seq + 1) & 3;
    memcpy(st->pending.chunk, src->data + src->off, (size_t)chunk_len);
    st->pending.chunk_len = chunk_len;
    st->pending.last = src->off + chunk_len >= src->len;
    st->pending.qos = src->qos;
    st->pending.seq = st->seq;
    st->pending.stream = src == &st->cur_prio;
    st->pending.first_try = 1;
    st->pending.active = 1;
    return &st->pending;
}

int station_poll_tx(station_t *st, double t, int channel_busy,
                    int16_t *out, int out_cap)
{
    lc_word_t lc;
    static uint8_t pkt_bits[2600]; /* EXT frames: up to 2076 bits */
    uint8_t payload[256];
    int payload_len, pkt_n, rung_idx, qos, flags, seq, expects_reply, n;
    st_frag_t *frag;
    double freq_req = 0.0;

    if (channel_busy)
        return 0; /* carrier sense; keeps a pending timeout alive */
    if (t < st->not_before)
        return 0;
    if (st->await_until >= 0.0) {
        if (t < st->await_until)
            return 0;
        st->await_until = -1.0; /* timeout on an idle channel -> loss */
        st->stats.timeouts++;
        st->rto_backoff = st->rto_backoff < 1.0
                              ? 2.0 : st->rto_backoff * 2.0;
        if (st->rto_backoff > RTO_MAX_BACKOFF)
            st->rto_backoff = RTO_MAX_BACKOFF;
        st->rto_ambiguous = 1; /* Karn: whatever answers next is ambiguous */
        st->rto_pending = 0;
        /* a burst ack window's FIRST miss is forgiven: the bitmap ack is
         * routinely a little late (peer decodes the long frame, waits for
         * carrier release, replies at ITS control rung), and penalizing
         * the controller for that poisons rung offsets and walks
         * consecutive_losses toward the >=4 hard rung-0 -- the observed
         * "rung 0 right after sendfile". The probe still goes out; only
         * a repeated miss counts as a real loss. */
        if (st->btx.active && st->btx.miss == 0) {
            st->btx.miss = 1;
            diag(st, ST_EV_TIMEOUT, st->ctl.consecutive_losses,
                 st->last_tx_rung, 1, 0, t);
        } else {
            if (st->btx.active)
                st->btx.miss++;
            ctl_on_timeout(&st->ctl);
            diag(st, ST_EV_TIMEOUT, st->ctl.consecutive_losses,
                 st->last_tx_rung, 0, 0, t);
            if (st->last_tx_rung >= 0)
                ctl_note_outcome(&st->ctl, st->last_tx_rung, 0);
        }
        st->not_before = t + rng_uniform(st, st->backoff_lo, st->backoff_hi);
        if (st->pending.active)
            st->pending.first_try = 0;
        if (st->btx.active) {
            /* A streamed window that timed out counts toward abandoning
             * streaming for this transfer -- including the forgiven
             * first timeout, because forgiveness is about not poisoning
             * the rate controller, not about discarding the evidence.
             * (Clearing streamed_n on the forgiven miss would hide the
             * second strike, since by then the window is long gone.)
             *
             * MEASURED, do not "improve" this into a gentler response:
             * halving the window on each timeout instead of striking out
             * makes a fading channel dramatically worse -- 196
             * transmissions and 72 timeouts against 134 and 9 for the
             * strike-out path, because the window collapses to 1 and
             * never recovers while streaming keeps being attempted. When
             * a fade is what is breaking the burst, the right answer is
             * to stop streaming, not to stream less. */
            if (st->btx.streamed_n > 0) {
                if (++st->btx.stream_strikes >= BURST_STREAM_STRIKES)
                    burst_stream_off(st, ST_SOFF_TIMEOUT, t);
                st->btx.streamed_n = 0;
            }
            st->btx.window_left = 1; /* probe: one frame, ack requested */
            memset(st->btx.sent, 0, sizeof(st->btx.sent));
            diag(st, ST_EV_BURST_PROBE, st->btx.id, 0, 0, 0, t);
        }
        return 0;
    }

    {
        int owes_ack = (st->last_rx_seq >= 0 && st->reply_due)
                       || st->brx.ack_due;
        if (!owes_ack && !station_has_traffic(st))
            return 0;
    }

    /* burst acknowledgment duty (bitmap of received fragments) */
    if (st->brx.ack_due) {
        int nb = (st->brx.n + 7) / 8;
        memcpy(payload, st->brx.have, (size_t)nb);
        lc.seq = st->seq;
        lc.ack = st->brx.id;
        lc.req_rung = ctl_rx_request(&st->ctl, t);
        lc.snr_db = ctl_filtered_snr(&st->ctl, t);
        lc.freq_corr_hz = 0.0;
        lc.flags = FLAG_BURST_ACK;
        pkt_n = data_encode(lc_pack(&lc), payload, nb, pkt_bits);
        rung_idx = ctl_tx_rung_for_class(&st->ctl, t, QOS_CONTROL);
        n = st->phy.build(st->phy.ctx, pkt_bits, pkt_n, PKT_TYP_DATA,
                          rung_idx, out, out_cap);
        if (n <= 0)
            return 0;
        st->brx.ack_due = 0;
        diag_rung(st, rung_idx, t);
        diag(st, ST_EV_BURST_ACKTX, st->brx.id, nb, 0, 0, t);
        diag(st, ST_EV_TX, rung_idx, PKT_TYP_DATA, lc.flags, nb, t);
        st->stats.tx_frames++;
        st->stats.last_rung = rung_idx;
        st->last_tx_rung = rung_idx;
        st->reply_due = 0;
        st->expects_reply = 0;
        return n;
    }

    /* burst transmit: up to window_left back-to-back fragments, the last
     * one carrying the ack request */
    rung_idx = ctl_tx_rung_for_class(&st->ctl, t, QOS_BULK);
    if (!st->btx.active && burst_try_engage(st, rung_idx))
        diag(st, ST_EV_BURST_ENGAGE, st->btx.n, st->btx.frag_size,
             st->btx.id, 0, t);
    if (burst_stream_ready(st)) {
        n = burst_send_stream(st, rung_idx, out, out_cap, t);
        if (n > 0)
            return n; /* 0 = not this time; fall through to one frame */
    }
    if (st->btx.active && st->btx.window_left > 0) {
        int idx = burst_next_candidate(st, st->btx.cursor);
        int flen, ack_req;
        if (idx < 0) { /* window exhausted; wait for the ack */
            st->btx.window_left = 0;
        } else {
            flen = idx == st->btx.n - 1 ? st->btx.last_len
                                        : st->btx.frag_size;
            ack_req = st->btx.window_left == 1
                      || burst_candidates_left(st) == 1;
            payload[0] = (uint8_t)idx;
            payload[1] = (uint8_t)((ack_req ? 0x80 : 0) | st->btx.n);
            payload[2] = (uint8_t)st->btx.frag_size;
            memcpy(payload + BURST_SUBHDR,
                   st->cur_bulk.data + idx * st->btx.frag_size,
                   (size_t)flen);
            lc.seq = st->btx.id;
            lc.ack = st->last_rx_seq >= 0 ? st->last_rx_seq : 0;
            lc.req_rung = ctl_rx_request(&st->ctl, t);
            lc.snr_db = ctl_filtered_snr(&st->ctl, t);
            lc.freq_corr_hz = 0.0;
            lc.flags = FLAG_BURST_DATA;
            pkt_n = data_encode(lc_pack(&lc), payload, BURST_SUBHDR + flen,
                                pkt_bits);
            n = st->phy.build(st->phy.ctx, pkt_bits, pkt_n,
                              PKT_TYP_EXT_DATA, rung_idx, out, out_cap);
            if (n <= 0)
                return 0;
            diag_rung(st, rung_idx, t);
            diag(st, ST_EV_BURST_FRAG, idx, ack_req, st->btx.window_left,
                 0, t);
            diag(st, ST_EV_TX, rung_idx, PKT_TYP_EXT_DATA, lc.flags,
                 BURST_SUBHDR + flen, t);
            st->btx.cursor = idx + 1;
            st->btx.sent[idx >> 3] |= (uint8_t)(1 << (idx & 7));
            st->btx.window_left--;
            if (ack_req)
                st->btx.window_left = 0;
            st->stats.tx_frames++;
            st->stats.last_rung = rung_idx;
            st->last_tx_rung = rung_idx;
            st->reply_due = 0;
            st->expects_reply = ack_req;
            /* the bitmap ack comes at the PEER's control rung, which can
             * sit below what we request -- budget two rungs conservative
             * and for the actual bitmap size, not a 1-byte frame */
            st->reply_rung_guess = ctl_rx_request(&st->ctl, t) - 2;
            if (st->reply_rung_guess < 0)
                st->reply_rung_guess = 0;
            st->reply_len_guess = (st->btx.n + 7) / 8;
            return n;
        }
    }

    if (st->pending.active)
        qos = st->pending.qos;
    else if (st->qcount[0])
        qos = QOS_CONTROL;
    else if (st->qcount[1])
        qos = QOS_INTERACTIVE;
    else if (st->qcount[2])
        qos = QOS_BULK;
    else
        qos = QOS_CONTROL;
    rung_idx = ctl_tx_rung_for_class(
        &st->ctl, t, station_has_traffic(st) ? qos : QOS_CONTROL);

    frag = take_fragment(st, rung_idx);
    if (frag) {
        if (!frag->first_try)
            st->stats.retransmissions++;
        if (frag->chunk_len > 0) {
            memcpy(payload, frag->chunk, (size_t)frag->chunk_len);
            payload_len = frag->chunk_len;
        } else {
            payload[0] = 0;
            payload_len = 1;
        }
        flags = (frag->last ? FLAG_LAST_FRAGMENT : 0)
                | (frag->stream ? FLAG_PRIO_STREAM : 0);
        seq = frag->seq;
        expects_reply = 1;
    } else {
        payload[0] = 0;
        payload_len = 1;
        flags = FLAG_NO_DATA;
        seq = st->seq;
        expects_reply = 0;
    }

    if (st->freq_trim_cb && st->has_cfo
        && (st->last_cfo_hz > AFC_DEADBAND_HZ
            || st->last_cfo_hz < -AFC_DEADBAND_HZ)) {
        freq_req = -st->last_cfo_hz;
        if (freq_req > FREQ_MAX_HZ)
            freq_req = FREQ_MAX_HZ;
        if (freq_req < -FREQ_MAX_HZ)
            freq_req = -FREQ_MAX_HZ;
    }

    lc.seq = seq;
    lc.ack = st->last_rx_seq >= 0 ? st->last_rx_seq : 0;
    lc.req_rung = ctl_rx_request(&st->ctl, t);
    lc.snr_db = ctl_filtered_snr(&st->ctl, t);
    lc.freq_corr_hz = freq_req;
    lc.flags = flags;

    pkt_n = data_encode(lc_pack(&lc), payload, payload_len, pkt_bits);
    n = st->phy.build(st->phy.ctx, pkt_bits, pkt_n, PKT_TYP_DATA, rung_idx,
                      out, out_cap);
    if (n <= 0)
        return 0;

    diag_rung(st, rung_idx, t);
    diag(st, ST_EV_TX, rung_idx, PKT_TYP_DATA, lc.flags, payload_len, t);
    st->stats.tx_frames++;
    st->stats.last_rung = rung_idx;
    st->last_tx_rung = rung_idx;
    st->reply_due = 0;
    st->expects_reply = expects_reply;
    st->reply_rung_guess = ctl_rx_request(&st->ctl, t);
    st->reply_len_guess = 1;
    return n;
}

void station_on_tx_end(station_t *st, double t)
{
    st->tx_end_t = t;
    if (st->expects_reply) {
        double slack = rto_slack(st);
        st->rto_air_est = estimate_air_time(st->reply_rung_guess,
                                            st->reply_len_guess > 0
                                                ? st->reply_len_guess : 1);
        st->await_until = t + st->rto_air_est + slack;
        st->rto_pending = 1;
        diag(st, ST_EV_RTO, (int)(st->rto_srtt * 1000.0),
             (int)(st->rto_rttvar * 1000.0),
             (int)((st->rto_air_est + slack) * 1000.0),
             (int)(st->rto_air_est * 1000.0), t);
    } else {
        st->await_until = -1.0;
        st->rto_pending = 0;
    }
    st->not_before = t + st->turnaround / 2.0;
}

/* Does a decoded packet say "more blocks follow me in this recording"?
 * Only the streamed-fragment marker does, and only on a burst fragment. */
static int frame_is_streamed(const uint8_t *pkt_bits, int pkt_n)
{
    lc_word_t lc;
    uint32_t reserved = 0;
    int i, v = 0;

    if ((pkt_n - 36) / 8 < BURST_SUBHDR)
        return 0;
    for (i = 0; i < 20; i++)
        reserved = (reserved << 1) | (pkt_bits[i] & 1);
    lc_unpack(reserved, &lc);
    if (lc.flags != FLAG_BURST_DATA)
        return 0;
    for (i = 0; i < 8; i++)
        v = (v << 1) | (pkt_bits[20 + i] & 1);
    return (v & BURST_SUB_STREAMED) != 0;
}

/* Decode a streamed burst: re-run the recording through the PHY's burst
 * receiver and feed every block that passed CRC to the ordinary fragment
 * path. Block 0 is demodulated twice (once to discover the marker) --
 * cheap next to the burst itself, and it keeps the single-frame path,
 * which every other frame type uses, completely untouched. */
static int station_rx_stream(station_t *st, const int16_t *samples, int n,
                             double t)
{
    static uint8_t blocks[BURST_STREAM_MAX * 2600];
    static int ok[BURST_STREAM_MAX];
    int pkt_n = 0, k, got, nok, done = 0;
    double snr_db = -30.0, cfo_hz = 0.0;

    got = st->phy.receive_burst(st->phy.ctx, samples, n, BURST_STREAM_MAX,
                                BURST_STREAM_RESYNC, blocks, &pkt_n, ok,
                                &snr_db, &cfo_hz);
    if (got <= 0 || pkt_n <= 0)
        return -1; /* no lock: let the caller use the single-frame result */
    if (got > BURST_STREAM_MAX)
        got = BURST_STREAM_MAX;

    nok = 0;
    for (k = 0; k < got; k++)
        nok += ok[k] ? 1 : 0;
    if (nok == 0)
        return -1; /* nothing survived: keep the single-frame decode */

    diag(st, ST_EV_BURST_SRX, nok, got, 0, 0, t);
    for (k = 0; k < got; k++) {
        if (!ok[k])
            continue; /* one bad block costs only itself */
        if (station_on_decoded(st, blocks + (size_t)k * 2600, pkt_n, snr_db,
                               cfo_hz, 0, t))
            done = 1;
    }
    return done;
}

int station_rx_frame(station_t *st, const int16_t *samples, int n, double t)
{
    static uint8_t pkt_bits[2600];
    int pkt_n = 0, harq_combined = 0;
    double snr_db = -30.0, cfo_hz = 0.0;

    if (st->phy.receive(st->phy.ctx, samples, n, pkt_bits, &pkt_n, &snr_db,
                        &cfo_hz, &harq_combined,
                        st->harq_llrs_n ? st->harq_llrs : 0, st->harq_llrs_n,
                        st->harq_llrs, &st->harq_llrs_n) != 0)
        return 0; /* failed decode: fresh LLRs (if any) kept for HARQ */

    /* a streamed burst carries more blocks after this one; without a
     * burst receiver we simply keep the first block, and the sender's
     * bitmap ack will tell it to stop streaming */
    if (st->phy.receive_burst && frame_is_streamed(pkt_bits, pkt_n)) {
        int r = station_rx_stream(st, samples, n, t);
        if (r >= 0)
            return r;
    }

    return station_on_decoded(st, pkt_bits, pkt_n, snr_db, cfo_hz,
                              harq_combined, t);
}

int station_on_decoded(station_t *st, const uint8_t *pkt_bits, int pkt_n,
                       double snr_db, double cfo_hz, int harq_combined,
                       double t)
{
    int done = 0;
    lc_word_t lc;
    uint32_t reserved;
    int i, payload_len;

    if (harq_combined)
        st->stats.harq_combines++;
    st->harq_llrs_n = 0;

    reserved = 0;
    for (i = 0; i < 20; i++)
        reserved = (reserved << 1) | (pkt_bits[i] & 1);
    lc_unpack(reserved, &lc);
    payload_len = (pkt_n - 36) / 8;

    st->stats.rx_frames++;
    ctl_on_rx_frame(&st->ctl, snr_db, &lc, t);
    diag(st, ST_EV_RX, lc.flags, lc.seq, lc.ack, (int)(snr_db * 10.0), t);
    /* round-trip observation: everything beyond the reply's computed air
     * time is the overhead we are trying to learn. Karn's rule -- an
     * exchange that involved a retransmission cannot be attributed to a
     * particular transmission, so it updates nothing (and leaves the
     * backoff in place until a clean exchange clears it). */
    if (st->rto_pending) {
        if (!st->rto_ambiguous)
            rto_sample(st, (t - st->tx_end_t) - st->rto_air_est);
        st->rto_ambiguous = 0;
        st->rto_pending = 0;
    }
    st->await_until = -1.0; /* got a frame; the exchange continues */
    st->last_cfo_hz = cfo_hz;
    st->has_cfo = 1;

    if (st->freq_trim_cb && lc.freq_corr_hz != 0.0 && !st->afc_anchor) {
        double delta = AFC_GAIN * lc.freq_corr_hz;
        double nt = st->afc_total_hz + delta;
        if (nt > st->afc_max_trim_hz)
            nt = st->afc_max_trim_hz;
        if (nt < -st->afc_max_trim_hz)
            nt = -st->afc_max_trim_hz;
        delta = nt - st->afc_total_hz;
        if (delta != 0.0) {
            st->freq_trim_cb(st->trim_ctx, delta);
            st->afc_total_hz = nt;
            st->stats.afc_trims++;
        }
    }

    /* burst ACK: bitmap of received fragments for transfer lc.ack */
    if (lc.flags == FLAG_BURST_ACK) {
        if (st->btx.active && lc.ack == st->btx.id) {
            int nb = (st->btx.n + 7) / 8, b, i2, before = 0, na = 0;
            for (i2 = 0; i2 < st->btx.n; i2++)
                if (st->btx.acked[i2 >> 3] & (1 << (i2 & 7)))
                    before++;
            for (b = 0; b < nb && 20 + 8 * b + 8 <= pkt_n - 16; b++) {
                int v = 0, j;
                for (j = 0; j < 8; j++)
                    v = (v << 1) | (pkt_bits[20 + 8 * b + j] & 1);
                st->btx.acked[b] |= (uint8_t)v;
            }
            for (i2 = 0; i2 < st->btx.n; i2++)
                if (st->btx.acked[i2 >> 3] & (1 << (i2 & 7)))
                    na++;
            diag(st, ST_EV_BURST_ACKRX, na, st->btx.n, 0, 0, t);
            /* did the stream actually land? A peer that cannot follow one
             * decodes only its first block, so a window of >=3 that gains
             * at most one fragment is the signature of a receiver reading
             * the burst as a single frame. */
            if (st->btx.streamed_n > 0) {
                if (na - before <= 1 && st->btx.streamed_n >= 3) {
                    if (++st->btx.stream_strikes >= BURST_STREAM_STRIKES)
                        burst_stream_off(st, ST_SOFF_NOACK, t);
                } else {
                    st->btx.stream_strikes = 0;
                }
                st->btx.streamed_n = 0;
            }
            st->btx.miss = 0;
            if (burst_all_acked(st)) {
                diag(st, ST_EV_BURST_DONE, 0, st->btx.id, 0, 0, t);
                st->btx.active = 0;
                st->cur_bulk.active = 0;
                ctl_on_ack(&st->ctl);
                if (st->last_tx_rung >= 0)
                    ctl_note_outcome(&st->ctl, st->last_tx_rung, 1);
            } else {
                st->btx.window_left = st->btx.win > 0 ? st->btx.win
                                                      : st->burst_window;
                memset(st->btx.sent, 0, sizeof(st->btx.sent));
            }
        }
        return 0;
    }

    /* burst data fragment: place by index, ack on request */
    if (lc.flags == FLAG_BURST_DATA) {
        int plen = (pkt_n - 36) / 8, j, b2;
        uint8_t hdr2[BURST_SUBHDR];
        if (plen < BURST_SUBHDR)
            return 0;
        for (j = 0; j < BURST_SUBHDR; j++) {
            int v = 0, b;
            for (b = 0; b < 8; b++)
                v = (v << 1) | (pkt_bits[20 + 8 * j + b] & 1);
            hdr2[j] = (uint8_t)v;
        }
        {
            /* bit 7 of the index byte marks a streamed fragment; the rest
             * of the packet is identical to a per-frame burst fragment,
             * so this branch handles both without caring which it was */
            int idx = hdr2[0] & ~BURST_SUB_STREAMED;
            int ack_req = hdr2[1] >> 7, total = hdr2[1] & 0x7F;
            int fs = hdr2[2];
            int dlen = plen - BURST_SUBHDR;
            if (total < 1 || total > BURST_MAX_FRAGS || idx >= total
                || fs < 1 || fs > 253)
                return 0;
            if (!st->brx.active || st->brx.id != lc.seq
                || st->brx.n != total || st->brx.frag_size != fs) {
                memset(&st->brx, 0, sizeof(st->brx));
                st->brx.active = 1;
                st->brx.id = lc.seq;
                st->brx.n = total;
                st->brx.frag_size = fs;
            }
            if (idx * fs + dlen <= ST_ASM_MAX) {
                for (j = 0; j < dlen; j++) {
                    int v = 0;
                    for (b2 = 0; b2 < 8; b2++)
                        v = (v << 1)
                            | (pkt_bits[20 + 8 * BURST_SUBHDR + 8 * j + b2]
                               & 1);
                    st->assembly[0][idx * fs + j] = (uint8_t)v;
                }
                st->brx.have[idx >> 3] |= (uint8_t)(1 << (idx & 7));
                if (idx == total - 1)
                    st->brx.last_len = dlen;
            }
            if (ack_req)
                st->brx.ack_due = 1;
            /* complete? deliver once, keep state for duplicate acks */
            if (!st->brx.done && st->brx.last_len > 0) {
                int all = 1, i2;
                for (i2 = 0; i2 < total; i2++)
                    if (!(st->brx.have[i2 >> 3] & (1 << (i2 & 7))))
                        all = 0;
                if (all) {
                    int mlen = (total - 1) * st->brx.frag_size
                               + st->brx.last_len;
                    diag(st, ST_EV_BURST_DONE, 1, st->brx.id, 0, 0, t);
                    st->brx.done = 1;
                    if (st->delivered_n < ST_DELIVERED_MAX
                        && mlen <= ST_MSG_MAX) {
                        memcpy(st->delivered[st->delivered_n],
                               st->assembly[0], (size_t)mlen);
                        st->delivered_len[st->delivered_n] = mlen;
                        st->delivered_n++;
                    }
                    return 1;
                }
            }
        }
        return 0;
    }

    /* ARQ: does their ack cover my pending fragment? */
    if (st->pending.active && lc.ack == st->pending.seq) {
        st_msg_t *src = st->pending.stream ? &st->cur_prio : &st->cur_bulk;
        src->off += st->pending.chunk_len;
        if (st->pending.last)
            src->active = 0;
        st->pending.active = 0;
        ctl_on_ack(&st->ctl);
        if (st->last_tx_rung >= 0)
            ctl_note_outcome(&st->ctl, st->last_tx_rung, 1);
    }

    if (!(lc.flags & FLAG_NO_DATA)) {
        if (lc.seq != st->last_rx_seq) { /* not a duplicate */
            int stream = (lc.flags & FLAG_PRIO_STREAM) ? 1 : 0;
            int alen = st->assembly_len[stream];
            for (i = 0; i < payload_len && alen < ST_ASM_MAX; i++) {
                int b, byte = 0;
                for (b = 0; b < 8; b++)
                    byte = (byte << 1) | (pkt_bits[20 + 8 * i + b] & 1);
                st->assembly[stream][alen++] = (uint8_t)byte;
            }
            st->assembly_len[stream] = alen;
            if (lc.flags & FLAG_LAST_FRAGMENT) {
                if (st->delivered_n < ST_DELIVERED_MAX
                    && alen <= ST_MSG_MAX) {
                    memcpy(st->delivered[st->delivered_n],
                           st->assembly[stream], (size_t)alen);
                    st->delivered_len[st->delivered_n] = alen;
                    st->delivered_n++;
                }
                st->assembly_len[stream] = 0;
                done++;
            }
        }
        st->last_rx_seq = lc.seq;
        st->reply_due = 1; /* data frames must be answered */
    }
    return done;
}
