/* Host link protocol: framing, round trips, and the ways a byte stream
 * can go wrong. No USB bus involved -- the codec is deliberately free of
 * one, which is what makes this testable at all. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "../src/usb_proto.h"

static int g_pass, g_fail;

static void check(const char *name, int ok)
{
    printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (ok) g_pass++; else g_fail++;
}

/* --- collector for parsed frames ------------------------------------ */
#define MAXF 64
static struct { uint8_t type; int len; uint8_t data[UP_MAX_PAYLOAD]; } g_f[MAXF];
static int g_n;

static void sink(void *ctx, uint8_t type, const uint8_t *p, int len)
{
    (void)ctx;
    if (g_n >= MAXF) return;
    g_f[g_n].type = type;
    g_f[g_n].len = len;
    if (len > 0) memcpy(g_f[g_n].data, p, (size_t)len);
    g_n++;
}

static void reset(void) { g_n = 0; }

int main(void)
{
    static uint8_t buf[8192];
    up_parser_t par;
    int n;

    /* ---- a frame survives a round trip ---- */
    {
        const char *msg = "HELLO MODEM";
        n = up_encode(UP_EVT_LOG, msg, (int)strlen(msg), buf, sizeof(buf));
        reset();
        up_parser_init(&par);
        up_parser_push(&par, buf, n, sink, 0);
        check("round trip: one frame in, one out",
              n == UP_HDR_LEN + (int)strlen(msg) && g_n == 1 &&
              g_f[0].type == UP_EVT_LOG &&
              g_f[0].len == (int)strlen(msg) &&
              memcmp(g_f[0].data, msg, strlen(msg)) == 0);
    }

    /* ---- the capacity check is exact ----
     * up_encode must refuse a buffer one byte short and accept one that
     * is exactly right. An off-by-one here writes past the caller's
     * buffer, and the header length is easy to get wrong: it is FIVE
     * bytes, not four. */
    {
        uint8_t small[16];
        int payload = 8;
        int exact = up_encode(UP_CMD_PING, "abcdefgh", payload, small,
                              UP_HDR_LEN + payload);
        int over = up_encode(UP_CMD_PING, "abcdefgh", payload, small,
                             UP_HDR_LEN + payload - 1);
        check("encode: exact capacity accepted, one short refused",
              exact == UP_HDR_LEN + payload && over == -1);
    }

    /* ---- frames split across arbitrary chunk boundaries ----
     * USB delivers whatever the host asks for, and a 64-byte endpoint
     * cuts long frames wherever it likes. Feeding one byte at a time is
     * the worst case and must behave identically. */
    {
        uint8_t big[600];
        int i, k;
        for (i = 0; i < (int)sizeof(big); i++)
            big[i] = (uint8_t)(i * 31 + 7);
        n = up_encode(UP_EVT_MESSAGE, big, (int)sizeof(big), buf,
                      sizeof(buf));
        reset();
        up_parser_init(&par);
        for (k = 0; k < n; k++)
            up_parser_push(&par, buf + k, 1, sink, 0);
        check("frame reassembles when fed one byte at a time",
              g_n == 1 && g_f[0].len == (int)sizeof(big) &&
              memcmp(g_f[0].data, big, sizeof(big)) == 0 &&
              par.resyncs == 0);
    }

    /* ---- several frames coalesced into one read ---- */
    {
        int pos = 0, k;
        for (k = 0; k < 5; k++) {
            uint8_t body[3] = { (uint8_t)k, 0xAA, 0x55 };
            pos += up_encode(UP_EVT_DIAG, body, 3, buf + pos,
                             (int)sizeof(buf) - pos);
        }
        reset();
        up_parser_init(&par);
        up_parser_push(&par, buf, pos, sink, 0);
        check("five coalesced frames come out as five",
              g_n == 5 && g_f[4].data[0] == 4 && par.resyncs == 0);
    }

    /* ---- garbage before a good frame is skipped, not swallowed ----
     * The device may be mid-transmission when the host opens the
     * endpoint. Resynchronising must find the next real frame rather
     * than lose it. */
    {
        uint8_t junk[9] = { 0x00, 0xFF, 0xA5, 0x11, 0x5A, 0xA5, 0xA5,
                            0x01, 0x02 };
        int good;
        memcpy(buf, junk, sizeof(junk));
        good = up_encode(UP_CMD_PING, "wxyz", 4, buf + sizeof(junk),
                         (int)sizeof(buf) - (int)sizeof(junk));
        reset();
        up_parser_init(&par);
        up_parser_push(&par, buf, (int)sizeof(junk) + good, sink, 0);
        check("leading garbage is resynced past, frame still delivered",
              g_n == 1 && g_f[0].type == UP_CMD_PING &&
              g_f[0].len == 4 &&
              memcmp(g_f[0].data, "wxyz", 4) == 0 && par.resyncs > 0);
    }

    /* ---- a length field larger than the protocol allows is rejected ----
     * Otherwise a corrupted header stalls the parser forever waiting for
     * a frame that will never arrive. */
    {
        uint8_t evil[5] = { UP_SYNC0, UP_SYNC1, UP_EVT_LOG, 0xFF, 0xFF };
        int good;
        memcpy(buf, evil, sizeof(evil));
        good = up_encode(UP_RSP_PONG, "ok", 2, buf + sizeof(evil),
                         (int)sizeof(buf) - (int)sizeof(evil));
        reset();
        up_parser_init(&par);
        up_parser_push(&par, buf, (int)sizeof(evil) + good, sink, 0);
        check("impossible length is rejected, parser recovers",
              g_n == 1 && g_f[0].type == UP_RSP_PONG && par.resyncs > 0);
    }

    /* ---- oversize payloads are refused at encode ---- */
    {
        static uint8_t huge[UP_MAX_PAYLOAD + 1];
        check("payload above UP_MAX_PAYLOAD refused",
              up_encode(UP_EVT_MESSAGE, huge, UP_MAX_PAYLOAD + 1, buf,
                        sizeof(buf)) == -1 &&
              up_encode(UP_EVT_MESSAGE, huge, UP_MAX_PAYLOAD, buf,
                        sizeof(buf)) == UP_HDR_LEN + UP_MAX_PAYLOAD);
    }

    /* ---- zero-length payloads are legal (UP_CMD_INFO has none) ---- */
    {
        n = up_encode(UP_CMD_INFO, 0, 0, buf, sizeof(buf));
        reset();
        up_parser_init(&par);
        up_parser_push(&par, buf, n, sink, 0);
        check("zero-length frame round trips",
              n == UP_HDR_LEN && g_n == 1 && g_f[0].len == 0 &&
              g_f[0].type == UP_CMD_INFO);
    }

    /* ---- fixed-layout payloads survive, byte order included ---- */
    {
        up_info_t in, out2;
        int i;
        memset(&in, 0, sizeof(in));
        in.proto_ver = 1;
        in.n_modes = 3;
        in.fw_ver = 0x0104;
        for (i = 0; i < 12; i++)
            in.uid[i] = (uint8_t)(0x10 + i);
        in.caps = UP_CAP_LDPC | UP_CAP_BURST;
        in.sample_rate = 12000;
        n = up_encode_info(&in, buf, sizeof(buf));
        reset();
        up_parser_init(&par);
        up_parser_push(&par, buf, n, sink, 0);
        memset(&out2, 0, sizeof(out2));
        check("info payload round trips exactly",
              g_n == 1 && g_f[0].type == UP_RSP_INFO &&
              up_decode_info(g_f[0].data, g_f[0].len, &out2) == 0 &&
              out2.proto_ver == 1 && out2.n_modes == 3 &&
              out2.fw_ver == 0x0104 && out2.caps == in.caps &&
              out2.sample_rate == 12000 &&
              memcmp(out2.uid, in.uid, 12) == 0);
    }
    {
        up_status_t in, out2;
        memset(&in, 0, sizeof(in));
        in.rung = -1;                 /* the "nothing yet" value */
        in.snr_q8 = -1536;            /* -6.0 dB, and negative on the wire */
        in.tx_frames = 4000000000u;   /* past INT32_MAX, must not wrap */
        in.rx_frames = 7;
        in.q_bulk = 3;
        in.busy = 1;
        n = up_encode_status(&in, buf, sizeof(buf));
        reset();
        up_parser_init(&par);
        up_parser_push(&par, buf, n, sink, 0);
        memset(&out2, 0, sizeof(out2));
        check("status payload round trips, negatives and >2^31 intact",
              g_n == 1 &&
              up_decode_status(g_f[0].data, g_f[0].len, &out2) == 0 &&
              out2.rung == -1 && out2.snr_q8 == -1536 &&
              out2.tx_frames == 4000000000u && out2.rx_frames == 7 &&
              out2.q_bulk == 3 && out2.busy == 1);
    }

    /* ---- truncated fixed payloads are refused, not read past ---- */
    {
        up_info_t o;
        up_status_t s;
        uint8_t short_[8] = { 0 };
        check("short fixed payloads are refused",
              up_decode_info(short_, 8, &o) == -1 &&
              up_decode_status(short_, 8, &s) == -1);
    }

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
