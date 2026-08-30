/* Host link protocol codec -- see usb_proto.h.
 *
 * Deliberately free of any USB, station or platform dependency: it is
 * bytes in, frames out. That is what lets the host driver, the device
 * firmware and the test suite all share one implementation, and what
 * lets the whole protocol be tested without a USB bus in the room. */

#include <string.h>

#include "usb_proto.h"

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
           | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int up_encode(uint8_t type, const void *payload, int len,
              uint8_t *out, int out_cap)
{
    if (len < 0 || len > UP_MAX_PAYLOAD)
        return -1;
    if (out_cap < UP_HDR_LEN + len)
        return -1;
    out[0] = UP_SYNC0;
    out[1] = UP_SYNC1;
    out[2] = type;
    put_u16(out + 3, (uint16_t)len);
    if (len)
        memcpy(out + UP_HDR_LEN, payload, (size_t)len);
    return UP_HDR_LEN + len;
}

int up_encode_info(const up_info_t *info, uint8_t *out, int out_cap)
{
    uint8_t p[26];
    p[0] = info->proto_ver;
    p[1] = info->n_modes;
    put_u16(p + 2, info->fw_ver);
    memcpy(p + 4, info->uid, 12);
    put_u32(p + 16, info->caps);
    put_u32(p + 20, info->sample_rate);
    put_u16(p + 24, info->msg_max);
    return up_encode(UP_RSP_INFO, p, 26, out, out_cap);
}

int up_encode_status(const up_status_t *st, uint8_t *out, int out_cap)
{
    uint8_t p[30];
    put_u32(p + 0, (uint32_t)st->rung);
    put_u32(p + 4, (uint32_t)st->snr_q8);
    put_u32(p + 8, st->tx_frames);
    put_u32(p + 12, st->rx_frames);
    put_u32(p + 16, st->timeouts);
    put_u32(p + 20, st->retransmissions);
    put_u16(p + 24, st->q_ctl);
    put_u16(p + 26, st->q_inter);
    put_u16(p + 28, st->q_bulk);
    /* busy/pending ride in the two spare bits of nothing -- append */
    {
        uint8_t q[40];
        memcpy(q, p, 30);
        q[30] = st->busy;
        q[31] = st->pending;
        q[32] = st->peer_state;
        q[33] = st->peer_caps;
        put_u16(q + 34, st->peer_msg_max);
        q[36] = st->peer_win_max;
        q[37] = q[38] = q[39] = 0;
        return up_encode(UP_EVT_STATUS, q, 40, out, out_cap);
    }
}

int up_encode_diag(int ev, int a, int b, int c, int d, uint32_t t_ms,
                   uint8_t *out, int out_cap)
{
    uint8_t p[21];
    p[0] = (uint8_t)ev;
    put_u32(p + 1, (uint32_t)a);
    put_u32(p + 5, (uint32_t)b);
    put_u32(p + 9, (uint32_t)c);
    put_u32(p + 13, (uint32_t)d);
    put_u32(p + 17, t_ms);
    return up_encode(UP_EVT_DIAG, p, 21, out, out_cap);
}

void up_parser_init(up_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

/* Drop one byte from the front and re-examine. Only reached when the
 * sync bytes do not match, which on a healthy USB link never happens --
 * `resyncs` is therefore a fault counter, not a routine statistic. */
static void slip(up_parser_t *p)
{
    if (p->have > 1)
        memmove(p->buf, p->buf + 1, (size_t)(p->have - 1));
    if (p->have > 0)
        p->have--;
    p->resyncs++;
    p->want = 0;
}

int up_parser_push(up_parser_t *p, const uint8_t *data, int n,
                   void (*cb)(void *ctx, uint8_t type,
                              const uint8_t *payload, int len),
                   void *ctx)
{
    int delivered = 0, i = 0;

    while (i < n) {
        /* fill */
        int room = (int)sizeof(p->buf) - p->have;
        int take = n - i < room ? n - i : room;
        if (take <= 0)
            return delivered;   /* cannot happen: a frame always drains */
        memcpy(p->buf + p->have, data + i, (size_t)take);
        p->have += take;
        i += take;

        /* drain every whole frame currently buffered */
        for (;;) {
            if (p->have >= 1 && p->buf[0] != UP_SYNC0) {
                slip(p);
                continue;
            }
            if (p->have >= 2 && p->buf[1] != UP_SYNC1) {
                slip(p);
                continue;
            }
            if (p->have < UP_HDR_LEN)
                break;                      /* header incomplete */
            if (!p->want) {
                int len = get_u16(p->buf + 3);
                if (len > UP_MAX_PAYLOAD) {  /* not a real header */
                    slip(p);
                    continue;
                }
                p->want = UP_HDR_LEN + len;
            }
            if (p->have < p->want)
                break;                      /* payload incomplete */
            if (cb)
                cb(ctx, p->buf[2], p->buf + UP_HDR_LEN,
                   p->want - UP_HDR_LEN);
            delivered++;
            if (p->have > p->want)
                memmove(p->buf, p->buf + p->want,
                        (size_t)(p->have - p->want));
            p->have -= p->want;
            p->want = 0;
        }
    }
    return delivered;
}

int up_decode_info(const uint8_t *payload, int len, up_info_t *out)
{
    if (len < 24)
        return -1;
    out->proto_ver = payload[0];
    out->n_modes = payload[1];
    out->fw_ver = get_u16(payload + 2);
    memcpy(out->uid, payload + 4, 12);
    out->caps = get_u32(payload + 16);
    out->sample_rate = get_u32(payload + 20);
    out->msg_max = len >= 26 ? get_u16(payload + 24) : 0;
    return 0;
}

int up_decode_status(const uint8_t *payload, int len, up_status_t *out)
{
    if (len < 32)
        return -1;
    out->rung = (int32_t)get_u32(payload + 0);
    out->snr_q8 = (int32_t)get_u32(payload + 4);
    out->tx_frames = get_u32(payload + 8);
    out->rx_frames = get_u32(payload + 12);
    out->timeouts = get_u32(payload + 16);
    out->retransmissions = get_u32(payload + 20);
    out->q_ctl = get_u16(payload + 24);
    out->q_inter = get_u16(payload + 26);
    out->q_bulk = get_u16(payload + 28);
    out->busy = payload[30];
    out->pending = payload[31];
    if (len >= 40) {
        out->peer_state = payload[32];
        out->peer_caps = payload[33];
        out->peer_msg_max = get_u16(payload + 34);
        out->peer_win_max = payload[36];
    } else {
        out->peer_state = out->peer_caps = out->peer_win_max = 0;
        out->peer_msg_max = 0;
    }
    return 0;
}
