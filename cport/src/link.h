/* Link adaptation -- C twin of ofdm_phy/link.py: rate ladder, 20-bit
 * link-control word, per-direction controller. Control plane: doubles are
 * allowed here (dB values and times, as in the model). */
#ifndef OFDM_LINK_H
#define OFDM_LINK_H

#include <stdint.h>
#include "tx.h" /* link_mode_t, mod_type_t */
#include "conv.h"

#define FREQ_STEP_HZ 8.0
#define FREQ_MAX_HZ 120.0
#define SNR_HIST_LEN 5

int ladder_n(void);
link_mode_t ladder_mode(int rung);
mod_type_t ladder_mod(int rung);
cc_rate_t ladder_spd(int rung);
double ladder_sens_db(int rung);
double ladder_rate(int rung);

typedef struct {
    int seq, ack, req_rung, flags;
    double snr_db, freq_corr_hz;
} lc_word_t;

uint32_t lc_pack(const lc_word_t *lc);
void lc_unpack(uint32_t v, lc_word_t *lc);

typedef struct {
    /* rx side (inbound link) */
    double snr_hist_t[SNR_HIST_LEN];
    double snr_hist_v[SNR_HIST_LEN];
    int snr_hist_n, snr_hist_head;
    int my_req;
    double last_rx_time;
    /* tx side (outbound link) */
    int peer_req;
    double peer_req_time;
    double peer_report_db;
    int consecutive_losses;
    double rung_offset_db[16];
} link_ctl_t;

void ctl_init(link_ctl_t *c);
void ctl_on_rx_frame(link_ctl_t *c, double snr_db, const lc_word_t *lc,
                     double now);
double ctl_filtered_snr(const link_ctl_t *c, double now);
int ctl_rx_request(link_ctl_t *c, double now);
void ctl_on_ack(link_ctl_t *c);
void ctl_on_timeout(link_ctl_t *c);
void ctl_note_outcome(link_ctl_t *c, int rung, int ok);
int ctl_tx_rung(const link_ctl_t *c, double now);
/* qos: 0 = control, 1 = interactive, 2 = bulk */
int ctl_tx_rung_for_class(const link_ctl_t *c, double now, int qos);

int link_max_payload_bytes(int rung, double max_air_s);

/* diagnostic snapshot of the tx-rung decision: every input that can pull
 * the rung down, plus the result. Read-only; for debug displays. */
typedef struct {
    int rung;              /* resulting ctl_tx_rung(now) */
    int peer_req;          /* rung the peer asked us to use */
    int cap;               /* cap from peer_report_db vs sens+offset */
    int losses;            /* consecutive timeouts (>=2: -2, >=4: rung 0) */
    int my_req;            /* what we ask the peer for */
    double peer_report_db; /* SNR the peer last reported */
    double filtered_snr;   /* our rx-side filtered SNR */
    double req_age_s;      /* age of the peer's request (stale: decay) */
    double rx_age_s;       /* age of our last received frame */
    double offset_db;      /* learned penalty of the resulting rung */
} link_diag_t;
void ctl_diag(const link_ctl_t *c, double now, link_diag_t *d);

#endif /* OFDM_LINK_H */
