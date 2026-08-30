#include <string.h>

#include "usb_modem.h"

/* Staging ring. The host may be slow or absent; the modem must not
 * block on it and must not corrupt the queue when it fills. A dropped
 * frame is counted rather than half-written -- a partial frame would
 * desynchronise the stream, which costs far more than the frame. */
static int txq_push(usb_modem_t *m, const uint8_t *f, int n)
{
    int i;
    if (n <= 0 || n > UM_TXQ - m->txq_len) {
        m->dropped++;
        return 0;
    }
    for (i = 0; i < n; i++)
        m->txq[(m->txq_head + m->txq_len + i) % UM_TXQ] = f[i];
    m->txq_len += n;
    return n;
}

static void emit(usb_modem_t *m, uint8_t type, const void *p, int len)
{
    uint8_t f[UP_HDR_LEN + 256];
    int n;
    if (len > (int)sizeof(f) - UP_HDR_LEN)
        len = (int)sizeof(f) - UP_HDR_LEN;
    n = up_encode(type, p, len, f, (int)sizeof(f));
    if (n > 0)
        txq_push(m, f, n);
}

static int32_t get_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8)
                     | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static void on_frame(void *ctx, uint8_t type, const uint8_t *p, int len)
{
    usb_modem_t *m = (usb_modem_t *)ctx;
    uint8_t buf[UP_HDR_LEN + 32];
    int n;

    m->host_cmds++;

    switch (type) {
    case UP_CMD_INFO:
        n = up_encode_info(&m->info, buf, (int)sizeof(buf));
        if (n > 0)
            txq_push(m, buf, n);
        break;

    case UP_CMD_PING:
        if (len >= 4)
            emit(m, UP_RSP_PONG, p, 4);
        break;

    case UP_CMD_SUBMIT:
        /* payload: qos, then the message. A refusal is reported rather
         * than silently dropped -- the host queues on our behalf and has
         * to know when to stop. */
        if (len >= 2) {
            int qos = p[0];
            if (qos < 0 || qos > 2)
                qos = 2;
            if (station_submit(m->st, p + 1, len - 1, qos) != 0)
                emit(m, UP_EVT_LOG, "submit refused: queue or store full",
                     36);
        }
        break;

    case UP_CMD_CONFIG:
        if (len >= 5) {
            int32_t v = get_i32(p + 1);
            switch (p[0]) {
            case UP_CFG_BURST_WINDOW: m->st->burst_window = (int)v; break;
            case UP_CFG_BURST_STREAM: m->st->burst_stream = v ? 1 : 0; break;
            case UP_CFG_FREQ_TRIM_MHZ:
                station_freq_trim(m->st, (double)v / 1000.0);
                break;
            case UP_CFG_ANCHOR: m->st->afc_anchor = v ? 1 : 0; break;
            case UP_CFG_DIAG_STREAM: m->diag_on = v ? 1 : 0; break;
            default: break;
            }
        }
        break;

    case UP_CMD_BCAST:
        if (len >= 2 && m->bcast_cb)
            m->bcast_cb(m->bcast_ctx, p[0], p[1], p + 2, len - 2);
        break;

    case UP_CMD_RESET:
        /* Re-init the station but keep the link identity: the host's
         * handle stays valid, which is the difference between a modem
         * reset and a USB re-enumeration. */
        m->delivered_seen = 0;
        station_delivered_reset(m->st);
        break;

    default:
        break;
    }
}

void usb_modem_init(usb_modem_t *m, station_t *st, const uint8_t uid[12],
                    uint16_t fw_ver, uint32_t caps)
{
    memset(m, 0, sizeof(*m));
    m->st = st;
    up_parser_init(&m->parser);
    m->info.proto_ver = 1;
    m->info.n_modes = 3;
    m->info.fw_ver = fw_ver;
    if (uid)
        memcpy(m->info.uid, uid, 12);
    m->info.caps = caps;
    m->info.sample_rate = 12000;
}

void usb_modem_rx(usb_modem_t *m, const uint8_t *data, int n)
{
    up_parser_push(&m->parser, data, n, on_frame, m);
}

void usb_modem_tick(usb_modem_t *m, double now, int status)
{
    m->now = now;

    /* deliver anything the station completed since last time */
    while (m->delivered_seen < m->st->delivered_n) {
        const uint8_t *msg = station_delivered(m->st, m->delivered_seen);
        int len = m->st->delivered_len[m->delivered_seen];
        m->delivered_seen++;
        if (msg && len > 0 && len <= 512) {
            uint8_t body[1 + 512];
            uint8_t out[UP_HDR_LEN + 1 + 512];
            int n;
            body[0] = 2;                    /* qos: bulk */
            memcpy(body + 1, msg, (size_t)len);
            n = up_encode(UP_EVT_MESSAGE, body, len + 1, out,
                          (int)sizeof(out));
            if (n > 0)
                txq_push(m, out, n);
        }
    }
    /* the log is drained once its contents are on the wire; leaving it
     * to fill would stall delivery at ST_DELIVERED_MAX */
    if (m->delivered_seen > 0 && m->delivered_seen >= m->st->delivered_n) {
        station_delivered_reset(m->st);
        m->delivered_seen = 0;
    }

    if (status) {
        up_status_t s;
        uint8_t out[UP_HDR_LEN + 32];
        int n;
        memset(&s, 0, sizeof(s));
        s.rung = m->st->stats.last_rung;
        s.snr_q8 = (int32_t)(ctl_filtered_snr(&m->st->ctl, now)
                             * 256.0);
        s.tx_frames = (uint32_t)m->st->stats.tx_frames;
        s.rx_frames = (uint32_t)m->st->stats.rx_frames;
        s.timeouts = (uint32_t)m->st->stats.timeouts;
        s.retransmissions = (uint32_t)m->st->stats.retransmissions;
        s.q_ctl = (uint16_t)m->st->qcount[0];
        s.q_inter = (uint16_t)m->st->qcount[1];
        s.q_bulk = (uint16_t)m->st->qcount[2];
        s.pending = (uint8_t)(m->st->pending.active ? 1 : 0);
        n = up_encode_status(&s, out, (int)sizeof(out));
        if (n > 0)
            txq_push(m, out, n);
    }
}

int usb_modem_poll(usb_modem_t *m, uint8_t *out, int cap)
{
    int n = m->txq_len < cap ? m->txq_len : cap;
    int i;
    for (i = 0; i < n; i++)
        out[i] = m->txq[(m->txq_head + i) % UM_TXQ];
    m->txq_head = (m->txq_head + n) % UM_TXQ;
    m->txq_len -= n;
    return n;
}

/* Firmware-level events the station knows nothing about -- a received
 * broadcast, for one -- reach the host through the same staging ring
 * as everything else. */
void usb_modem_emit(usb_modem_t *m, uint8_t type, const void *payload,
                    int len)
{
    emit(m, type, payload, len);
}

/* The diagnostic stream is OFF until the host asks for it, and even then
 * it yields to everything else.
 *
 * A station with no radio attached times out continuously, and each
 * event was a frame. With nothing draining them they filled the 512-byte
 * endpoint buffer within milliseconds and it never recovered: the device
 * sent exactly 549 bytes and then went silent, while its loop counter
 * kept climbing and every register read healthy. Command RESPONSES were
 * stuck behind a debug firehose.
 *
 * So: opt-in, and dropped rather than queued once the staging ring is
 * half full, because a lost diagnostic costs nothing and a lost reply
 * costs the session. */
void usb_modem_diag(void *ctx, int ev, int a, int b, int c, int d,
                    double t)
{
    usb_modem_t *m = (usb_modem_t *)ctx;
    uint8_t out[UP_HDR_LEN + 21];
    int n;

    if (!m->diag_on || m->txq_len > UM_TXQ / 2) {
        m->diag_suppressed++;
        return;
    }
    n = up_encode_diag(ev, a, b, c, d, (uint32_t)(t * 1000.0),
                       out, (int)sizeof(out));
    if (n > 0)
        txq_push(m, out, n);
}
