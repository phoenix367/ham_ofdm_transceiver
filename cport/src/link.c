#include <math.h>
#include <string.h>

#include "link.h"
#include "rom_link.h"

#define MARGIN_UP 2.5
#define MARGIN_KEEP 1.0
#define STALE_S 90.0
#define RX_STALE_S 45.0
#define SNR_MAX_AGE_S 60.0

int ladder_n(void) { return LADDER_N; }
link_mode_t ladder_mode(int r) { return (link_mode_t)LADDER_MODE[r]; }
mod_type_t ladder_mod(int r) { return (mod_type_t)LADDER_MOD[r]; }
cc_rate_t ladder_spd(int r) { return (cc_rate_t)LADDER_SPD[r]; }
double ladder_sens_db(int r) { return LADDER_SENS[r]; }
double ladder_rate(int r) { return LADDER_RATE[r]; }

/* python round() = round-half-even = C rint() in the default mode */
uint32_t lc_pack(const lc_word_t *lc)
{
    int snr_q = LC_SNR_IS_NONE(lc->snr_db)
                    ? 0                       /* no measurement to report */
                    : (int)rint((lc->snr_db + 24.0) / 2.0);
    int f_q = (int)rint(lc->freq_corr_hz / FREQ_STEP_HZ);
    if (snr_q < 1 && !LC_SNR_IS_NONE(lc->snr_db))
        snr_q = 1;                            /* a real one is never code 0 */
    if (snr_q > 15) snr_q = 15;
    if (f_q < -15) f_q = -15;
    if (f_q > 15) f_q = 15;
    f_q += 15;
    return ((uint32_t)(lc->seq & 3) << 18) | ((uint32_t)(lc->ack & 3) << 16)
           | ((uint32_t)(lc->req_rung & 15) << 12)
           | ((uint32_t)snr_q << 8) | ((uint32_t)f_q << 3)
           | (uint32_t)(lc->flags & 7);
}

void lc_unpack(uint32_t v, lc_word_t *lc)
{
    lc->seq = (int)((v >> 18) & 3);
    lc->ack = (int)((v >> 16) & 3);
    lc->req_rung = (int)((v >> 12) & 15);
    lc->snr_db = ((v >> 8) & 15) == 0
                     ? LC_SNR_NONE
                     : (double)((v >> 8) & 15) * 2.0 - 24.0;
    lc->freq_corr_hz = (double)((int)((v >> 3) & 31) - 15) * FREQ_STEP_HZ;
    lc->flags = (int)(v & 7);
}

void ctl_init(link_ctl_t *c)
{
    memset(c, 0, sizeof(*c));
    c->last_rx_time = -1e9;
    c->peer_req_time = -1e9;
    c->peer_report_db = LC_SNR_NONE;
    c->req_decay_t = -1e9;
}

void ctl_on_rx_frame(link_ctl_t *c, double snr_db, const lc_word_t *lc,
                     double now)
{
    /* deque(maxlen=5): ring overwrite */
    int slot = (c->snr_hist_head + c->snr_hist_n) % SNR_HIST_LEN;
    if (c->snr_hist_n == SNR_HIST_LEN)
        c->snr_hist_head = (c->snr_hist_head + 1) % SNR_HIST_LEN;
    else
        c->snr_hist_n++;
    c->snr_hist_t[slot] = now;
    c->snr_hist_v[slot] = snr_db;
    c->last_rx_time = now;
    c->peer_req = lc->req_rung;
    c->peer_req_time = now;
    c->peer_report_db = lc->snr_db;
    c->consecutive_losses = 0;
}

double ctl_filtered_snr(const link_ctl_t *c, double now)
{
    double vals[SNR_HIST_LEN];
    int n = 0, i, j;
    for (i = 0; i < c->snr_hist_n; i++) {
        int s = (c->snr_hist_head + i) % SNR_HIST_LEN;
        if (now - c->snr_hist_t[s] <= SNR_MAX_AGE_S)
            vals[n++] = c->snr_hist_v[s];
    }
    if (n == 0)
        return -99.0;
    for (i = 1; i < n; i++) { /* insertion sort ascending */
        double v = vals[i];
        for (j = i - 1; j >= 0 && vals[j] > v; j--)
            vals[j + 1] = vals[j];
        vals[j + 1] = v;
    }
    return n >= 3 ? vals[1] : vals[0];
}

int ctl_rx_request(link_ctl_t *c, double now)
{
    double snr;
    int i, best_up = 0;

    /* Decay follows ELAPSED TIME, not the number of calls. This used
     * to subtract the whole silence every time while leaving
     * last_rx_time alone, and station_poll_tx asks twice per frame (the
     * request that goes on the wire, then the reply-rung guess):
     * measured, three calls at ONE instant walked my_req 12 -> 11 ->
     * 10 -> 9, so a few retransmissions inside a fade collapsed the
     * request to rung 0 and the peer was asked for EXTREME. Charging
     * the decay against req_decay_t makes repeated calls idempotent. */
    {
        double base = c->req_decay_t > c->last_rx_time ? c->req_decay_t
                                                       : c->last_rx_time;
        if (now - base > RX_STALE_S) {
            int decay = (int)floor((now - base) / RX_STALE_S);
            c->my_req -= decay;
            if (c->my_req < 0)
                c->my_req = 0;
            c->req_decay_t = base + decay * RX_STALE_S;
            c->snr_hist_n = 0; /* stale measurements must not come back */
            c->snr_hist_head = 0;
            return c->my_req;
        }
        if (now - c->last_rx_time > RX_STALE_S)
            return c->my_req;   /* this silence is already charged */
    }

    snr = ctl_filtered_snr(c, now);
    for (i = 0; i < LADDER_N; i++)
        if (snr >= LADDER_SENS[i] + MARGIN_UP)
            best_up = i;
    if (best_up > c->my_req) {
        c->my_req = best_up;
    } else if (snr < LADDER_SENS[c->my_req] + MARGIN_KEEP) {
        int down = 0;
        for (i = 0; i < LADDER_N; i++)
            if (snr >= LADDER_SENS[i] + MARGIN_KEEP)
                down = i;
        if (down < c->my_req)
            c->my_req = down;
    }
    return c->my_req;
}

void ctl_on_ack(link_ctl_t *c) { c->consecutive_losses = 0; }
void ctl_on_timeout(link_ctl_t *c) { c->consecutive_losses++; }

void ctl_note_outcome(link_ctl_t *c, int rung, int ok)
{
    if (ok) {
        c->rung_offset_db[rung] -= 0.15;
        if (c->rung_offset_db[rung] < 0.0)
            c->rung_offset_db[rung] = 0.0;
    } else {
        c->rung_offset_db[rung] += 0.7;
        if (c->rung_offset_db[rung] > 6.0)
            c->rung_offset_db[rung] = 6.0;
    }
}

int ctl_tx_rung(const link_ctl_t *c, double now)
{
    int rung = c->peer_req, cap = 0, i;
    double age;

    /* The cap applies only to a report the peer actually made. "I heard
     * you badly" and "I have not heard anything lately" are different
     * facts: the first must slow us down, the second is what every gap
     * in a conversation looks like. A peer that has really gone away is
     * still caught by the staleness decay below. */
    if (!LC_SNR_IS_NONE(c->peer_report_db)) {
        for (i = 0; i < LADDER_N; i++)
            if (c->peer_report_db
                >= LADDER_SENS[i] + c->rung_offset_db[i] + MARGIN_KEEP)
                cap = i;
        if (cap < rung)
            rung = cap;
    }

    age = now - c->peer_req_time;
    if (age > STALE_S) {
        rung -= (int)floor(age / STALE_S);
        if (rung < 0)
            rung = 0;
    }
    if (c->consecutive_losses >= 4)
        rung = 0;
    else if (c->consecutive_losses >= 2) {
        rung -= 2;
        if (rung < 0)
            rung = 0;
    }
    return rung;
}

void ctl_diag(const link_ctl_t *c, double now, link_diag_t *d)
{
    int i;
    d->rung = ctl_tx_rung(c, now);
    d->peer_req = c->peer_req;
    /* -1 = the peer reported no measurement, so no cap applies. It is
     * NOT the same as a cap of 0, and printing it as one is what made
     * this whole class of defect hard to read in the first place. */
    d->cap = -1;
    if (!LC_SNR_IS_NONE(c->peer_report_db)) {
        d->cap = 0;
        for (i = 0; i < LADDER_N; i++)
            if (c->peer_report_db
                >= LADDER_SENS[i] + c->rung_offset_db[i] + MARGIN_KEEP)
                d->cap = i;
    }
    d->losses = c->consecutive_losses;
    d->my_req = c->my_req;
    d->peer_report_db = c->peer_report_db;
    d->filtered_snr = ctl_filtered_snr(c, now);
    d->req_age_s = now - c->peer_req_time;
    d->rx_age_s = now - c->last_rx_time;
    d->offset_db = c->rung_offset_db[d->rung];
}

int ctl_tx_rung_for_class(const link_ctl_t *c, double now, int qos)
{
    int rung = ctl_tx_rung(c, now);
    if (qos <= 1) { /* control / interactive */
        rung -= 1;
        if (rung < 0)
            rung = 0;
    }
    return rung;
}

int link_max_payload_bytes(int rung, double max_air_s)
{
    int v = (int)(LADDER_RATE[rung] * max_air_s / 8.0);
    if (v > 27)
        v = 27;
    if (v < 1)
        v = 1;
    return v;
}
