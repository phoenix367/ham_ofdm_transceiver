/* ofdm_console: interactive station over a virtual channel device.
 *
 *   ./ofdm_console /tmp/ofdmchan/s1.sock [name]
 *
 * Runs the C fixed-point stack end to end: the streaming receiver
 * (rx_stream) on the device's continuous 12 kHz RX audio, the link-layer
 * station (station.c) for QoS/ARQ/rate adaptation, and the fixed
 * transmitter for outbound frames. Protocol time is derived from received
 * samples, so the driver's time_scale changes wall speed only.
 *
 * Commands:  send <text>      queue an interactive text message
 *            sendfile <path>  transfer a file (bulk class; deflated
 *                             first, so the on-air size is what the
 *                             queue depth limits)
 *            bulk <n>         queue an n-byte test pattern (bulk class)
 *            status           connection status (rung, SNR, CFO, queues)
 *            stats            frame counters
 *            quit
 *
 * Received files are stored as rx_<basename> in the working directory.
 */
#define _POSIX_C_SOURCE 200809L /* strnlen, localtime_r */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>
#include <zlib.h>

/* app-level envelope: files travel as
 *   \x01 FILE: <name> \0 <part_idx> <n_parts> <data>
 * split into parts small enough for one burst-ARQ transfer each
 * (127 fragments x 25 bytes); parts arrive in order (FIFO bulk queue).
 *
 * Magic \x02 is the same envelope carrying a DEFLATE stream: the file is
 * compressed once, whole, and the compressed bytes are what get split
 * into parts -- compressing each 3 KB part separately would throw away
 * most of the ratio. A distinct magic (rather than a flag inside the
 * envelope) means a peer that predates compression sees an unknown
 * message instead of misparsing a valid-looking one.
 *
 * Compression lives here, at the application layer, exactly as Winlink's
 * B2F does it: cport/ stays dependency-free and MCU-portable. An
 * embedded build would swap zlib for miniz or heatshrink; the wire
 * format is plain DEFLATE either way. */
#define FILE_MAGIC 0x01
#define FILE_MAGIC_Z 0x02
#define FILE_TAG "FILE:"
#define FILE_PART_DATA 3000
#define FILE_MAX_SRC (1 << 19) /* 512 KB source cap before compression */

#include "../cport/src/link.h"
#include "../cport/src/station.h"
#include "../cport/src/tx.h"
#include "../cport/src/rx_stream.h"
#include "../cport/src/packets.h"
#include "../cport/src/broadcast.h"

#define FS 12000.0
#define BUSY_WIN 480        /* 40 ms carrier-sense window */
#define BUSY_RATIO 3.0      /* busy when RMS > ratio x tracked noise floor */

static station_t g_st;
static rxs_t *g_rxs[3]; /* one streaming receiver per link mode */
static int g_fd = -1;
static const char *g_name = "station";

static int64_t g_rx_total;          /* samples received = protocol clock */
static double g_busy_acc;           /* running power accumulator */
static int16_t g_busy_ring[BUSY_WIN];
static int g_busy_pos;

static int g_txing;                 /* burst in flight */
static double g_tx_end_t;
static double g_last_snr = -99.0, g_last_cfo;
static int g_rx_ok, g_rx_events;

static int16_t g_frame[600000];
/* streamed bursts: blocks still expected on each mode's receiver */
static int g_burst_left[3], g_burst_miss[3];
/* broadcast (non-ARQ): frames still expected in the current group, and
 * the reassembly buffer. Each group carries its own preamble, so the
 * streaming receiver finds group STARTS unaided -- only the frames after
 * the first need rxs_continue_burst. Note the causal receiver has none of
 * the global-argmax trouble the buffer-based walk has: it commits to a
 * preamble as samples arrive and never sees two at once. */
static int g_bc_left[3];
static int g_bc_group = 4;
static uint8_t g_bc_asm[8192];
static int g_bc_len, g_bc_frames, g_bc_lost, g_bc_last_seq = -1;
static int16_t g_bc_air[600000];
static int g_bc_pending;   /* samples queued for transmission */
static int g_bc_sent;

/* ---------------- PHY glue ---------------- */

static int phy_build(void *ctx, const uint8_t *bits, int n, int typ,
                     int rung, int16_t *out, int out_cap)
{
    (void)ctx;
    if (tx_frame_len(ladder_mode(rung), n, ladder_mod(rung),
                     ladder_spd(rung)) > out_cap)
        return -1;
    return tx_build_frame(ladder_mode(rung), bits, n, typ,
                          ladder_mod(rung), ladder_spd(rung), out);
}

static int phy_build_burst(void *ctx, const uint8_t *blocks, int pkt_n,
                           int n_blocks, int typ, int rung, int resync_every,
                           int16_t *out, int out_cap)
{
    (void)ctx;
    if (tx_burst_len(ladder_mode(rung), pkt_n, ladder_mod(rung),
                     ladder_spd(rung), n_blocks, resync_every) > out_cap)
        return -1;
    return tx_build_burst(ladder_mode(rung), blocks, pkt_n, n_blocks, typ,
                          ladder_mod(rung), ladder_spd(rung), resync_every,
                          out);
}

static int phy_receive_unused(void *ctx, const int16_t *s, int n,
                              uint8_t *b, int *bn, double *snr, double *cfo,
                              int *hc, const int64_t *pl, int pn,
                              int64_t *lo, int *ln)
{
    (void)ctx; (void)s; (void)n; (void)b; (void)bn; (void)snr; (void)cfo;
    (void)hc; (void)pl; (void)pn; (void)lo; (void)ln;
    return -1; /* the app decodes via the streaming receiver instead */
}

/* Is this decoded packet part of a streamed burst? Reads the link
 * layer's marker (bit 7 of the burst sub-header's index byte); the PHY
 * itself never looks inside a payload. */
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
    if (ack_req) /* sub-header byte 1, bit 7 */
        *ack_req = bits[20 + 8] & 1;
    return (v & BURST_SUB_STREAMED) != 0;
}

/* ---------------- helpers ---------------- */

static double now_t(void)
{
    return (double)g_rx_total / FS;
}

static const char *tstamp(void)
{
    static char buf[16];
    time_t t = time(0);
    struct tm tm;
    localtime_r(&t, &tm);
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", tm.tm_hour, tm.tm_min,
             tm.tm_sec);
    return buf;
}

static double g_noise_floor = 1e9; /* min-tracking RMS^2 EWMA */
static double g_busy_since = -1.0; /* protocol time the busy run started */

/* Longer than any single transmission (the longest frame is EXTREME with a
 * 27-byte payload, 38 s), so energy that outlasts it cannot be a frame. */
#define CS_REBASE_S 60.0

static int channel_busy(void)
{
    double p = g_busy_acc / BUSY_WIN;
    double now = now_t();
    int busy;
    /* noise floor: fast to drop, slow to rise -- adapts to whatever the
     * channel's quiet level is (carrier sense must be relative; frames
     * below the noise floor are invisible to energy detection anyway) */
    if (p < g_noise_floor)
        g_noise_floor = p;
    else
        g_noise_floor *= 1.0005;
    if (g_noise_floor < 25.0)
        g_noise_floor = 25.0;
    busy = p > BUSY_RATIO * BUSY_RATIO * g_noise_floor;

    /* A step rise in channel noise (an operator dropping the SNR, a band
     * opening) would otherwise take minutes to track: the floor drops
     * instantly but climbs only 0.05% per block, so carrier sense reads
     * BUSY and the station transmits nothing -- measured at 82 s of dead
     * air after a +20 -> -17 dB step, with no losses counted either
     * (a timeout on a busy channel is deliberately not a loss). No real
     * transmission outlasts one frame, so sustained energy beyond that
     * IS the new floor: re-baseline onto it. */
    if (!busy)
        g_busy_since = -1.0;
    else if (g_busy_since < 0.0)
        g_busy_since = now;
    else if (now - g_busy_since > CS_REBASE_S) {
        g_noise_floor = p;
        g_busy_since = -1.0;
        busy = 0;
    }
    return busy;
}

static void note_busy(const int16_t *s, int n)
{
    int i;
    for (i = 0; i < n; i++) {
        int16_t old = g_busy_ring[g_busy_pos];
        g_busy_acc += (double)s[i] * s[i] - (double)old * old;
        g_busy_ring[g_busy_pos] = s[i];
        g_busy_pos = (g_busy_pos + 1) % BUSY_WIN;
    }
}

static int g_debug; /* diag event printing on/off */

static void diag_print(void *ctx, int ev, int a, int b, int c, int d,
                       double t)
{
    (void)ctx;
    if (!g_debug)
        return;
    printf("%s [%s] dbg t=%.1f %-12s", tstamp(), g_name, t,
           station_diag_name(ev));
    switch (ev) {
    case ST_EV_TX:
        printf(" rung=%d typ=%d flags=%d len=%dB", a, b, c, d);
        break;
    case ST_EV_RX:
        printf(" flags=%d seq=%d ack=%d snr=%.1fdB", a, b, c, d / 10.0);
        break;
    case ST_EV_TIMEOUT:
        printf(" losses=%d rung=%d%s", a, b,
               a >= 4 ? "  << forces rung 0" : (a >= 2 ? "  << rung -2" : ""));
        break;
    case ST_EV_RUNG: {
        link_diag_t ld;
        ctl_diag(&g_st.ctl, t, &ld);
        printf(" %d -> %d (losses=%d cap=%d peer_req=%d peer_snr=%.0fdB "
               "req_age=%.0fs)", a, b, c, d, ld.peer_req,
               ld.peer_report_db, ld.req_age_s);
        break;
    }
    case ST_EV_BURST_ENGAGE:
        printf(" nfrags=%d frag_size=%dB id=%d", a, b, c);
        break;
    case ST_EV_BURST_FRAG:
        printf(" idx=%d ack_req=%d window_left=%d", a, b, c);
        break;
    case ST_EV_BURST_ACKTX:
        printf(" id=%d bitmap=%dB", a, b);
        break;
    case ST_EV_BURST_ACKRX:
        printf(" acked=%d/%d", a, b);
        break;
    case ST_EV_BURST_DONE:
        printf(" side=%s id=%d", a ? "rx" : "tx", b);
        break;
    default:
        printf(" a=%d b=%d c=%d d=%d", a, b, c, d);
    }
    printf("\n");
    fflush(stdout);
}

static void show_status(void)
{
    printf("--- %s @ t=%.1f s (audio clock)\n", g_name, now_t());
    printf("  channel: %s   last frame: SNR %+.1f dB, CFO %+.1f Hz\n",
           channel_busy() ? "BUSY" : "idle", g_last_snr, g_last_cfo);
    printf("  carrier sense: rms^2 %.0f, noise floor %.0f (%.1f dB over "
           "floor, busy above %.1f dB)\n",
           g_busy_acc / BUSY_WIN, g_noise_floor,
           10.0 * log10((g_busy_acc / BUSY_WIN) / g_noise_floor + 1e-9),
           20.0 * log10(BUSY_RATIO));
    printf("  tx rung %d (%s %.0f bit/s)   peer requests rung %d\n",
           g_st.stats.last_rung,
           g_st.stats.last_rung >= 0 ? "ladder" : "-",
           g_st.stats.last_rung >= 0 ? ladder_rate(g_st.stats.last_rung) : 0.0,
           g_st.ctl.peer_req);
    printf("  queues: ctl %d, inter %d, bulk %d, pending=%d   "
           "delivered %d msgs\n",
           g_st.qcount[0], g_st.qcount[1], g_st.qcount[2],
           g_st.pending.active, g_st.delivered_n);
    {
        link_diag_t ld;
        ctl_diag(&g_st.ctl, now_t(), &ld);
        printf("  link ctl: rung=%d cap=%d peer_req=%d my_req=%d "
               "losses=%d\n"
               "            peer_snr=%+.1fdB my_snr=%+.1fdB "
               "req_age=%.0fs rx_age=%.0fs offset=%.1fdB\n",
               ld.rung, ld.cap, ld.peer_req, ld.my_req, ld.losses,
               ld.peer_report_db, ld.filtered_snr, ld.req_age_s,
               ld.rx_age_s, ld.offset_db);
    }
}

static void show_stats(void)
{
    printf("--- %s stats: tx %d, rx %d, retransmissions %d, timeouts %d, "
           "rx events %d (%d decoded)\n",
           g_name, g_st.stats.tx_frames, g_st.stats.rx_frames,
           g_st.stats.retransmissions, g_st.stats.timeouts, g_rx_events,
           g_rx_ok);
}

static struct {
    char path[300];
    int next_part, n_parts, total, out_total, zipped;
    z_stream z;
    FILE *f;
} g_rxfile;

/* feed one part through the inflater, writing whatever comes out.
 * Returns 1 at end of stream, 0 mid-stream, -1 on a corrupt stream. */
static int inflate_part(const uint8_t *in, int n)
{
    unsigned char out[16384];
    int rc = Z_OK;

    g_rxfile.z.next_in = (Bytef *)in;
    g_rxfile.z.avail_in = (uInt)n;
    do {
        size_t got;
        g_rxfile.z.next_out = out;
        g_rxfile.z.avail_out = (uInt)sizeof(out);
        rc = inflate(&g_rxfile.z, Z_NO_FLUSH);
        if (rc != Z_OK && rc != Z_STREAM_END && rc != Z_BUF_ERROR)
            return -1;
        got = sizeof(out) - g_rxfile.z.avail_out;
        if (got && fwrite(out, 1, got, g_rxfile.f) != got)
            return -1;
        g_rxfile.out_total += (int)got;
        if (rc == Z_BUF_ERROR)
            break; /* no progress possible: needs more input */
    } while (g_rxfile.z.avail_out == 0);
    return rc == Z_STREAM_END ? 1 : 0;
}

static void store_file(const uint8_t *msg, int len, int zipped)
{
    const char *name = (const char *)msg + 1 + strlen(FILE_TAG);
    const uint8_t *meta;
    int name_len, data_len, part, n_parts;

    name_len = (int)strnlen(name, (size_t)(len - 1 - (int)strlen(FILE_TAG)));
    meta = (const uint8_t *)name + name_len + 1;
    data_len = len - (int)(meta + 2 - msg);
    if (data_len < 0) {
        printf("\n%s [%s] << malformed file envelope\n> ", tstamp(), g_name);
        return;
    }
    part = meta[0];
    n_parts = meta[1];
    /* basename only -- no path traversal from the peer */
    {
        const char *slash = strrchr(name, '/');
        if (slash)
            name = slash + 1;
    }

    if (part == 0) {
        if (g_rxfile.f)
            fclose(g_rxfile.f);
        snprintf(g_rxfile.path, sizeof(g_rxfile.path), "rx_%s",
                 name[0] ? name : "unnamed");
        g_rxfile.f = fopen(g_rxfile.path, "wb");
        g_rxfile.next_part = 0;
        g_rxfile.n_parts = n_parts;
        g_rxfile.total = 0;
        g_rxfile.out_total = 0;
        if (g_rxfile.zipped)
            inflateEnd(&g_rxfile.z); /* a dropped transfer left one open */
        g_rxfile.zipped = zipped;
        if (zipped) {
            memset(&g_rxfile.z, 0, sizeof(g_rxfile.z));
            if (inflateInit(&g_rxfile.z) != Z_OK) {
                printf("\n%s [%s] << file '%s': inflateInit failed\n> ",
                       tstamp(), g_name, name);
                fclose(g_rxfile.f);
                g_rxfile.f = 0;
                g_rxfile.zipped = 0;
                return;
            }
        }
        if (!g_rxfile.f) {
            printf("\n%s [%s] << file '%s': cannot write %s\n> ", tstamp(),
                   g_name, name, g_rxfile.path);
            return;
        }
    }
    if (!g_rxfile.f || part != g_rxfile.next_part
        || n_parts != g_rxfile.n_parts) {
        printf("\n%s [%s] << file '%s': part %d/%d out of order, "
               "transfer dropped\n> ", tstamp(), g_name, name, part + 1,
               n_parts);
        if (g_rxfile.f) {
            fclose(g_rxfile.f);
            g_rxfile.f = 0;
        }
        return;
    }

    if (g_rxfile.zipped) {
        int rc = inflate_part(meta + 2, data_len);
        if (rc < 0) {
            printf("\n%s [%s] << file '%s': corrupt compressed stream, "
                   "transfer dropped\n> ", tstamp(), g_name, name);
            inflateEnd(&g_rxfile.z);
            g_rxfile.zipped = 0;
            fclose(g_rxfile.f);
            g_rxfile.f = 0;
            return;
        }
    } else {
        fwrite(meta + 2, 1, (size_t)data_len, g_rxfile.f);
        g_rxfile.out_total += data_len;
    }
    g_rxfile.total += data_len;
    g_rxfile.next_part++;
    if (g_rxfile.next_part >= g_rxfile.n_parts) {
        if (g_rxfile.zipped) {
            inflateEnd(&g_rxfile.z);
            g_rxfile.zipped = 0;
        }
        fclose(g_rxfile.f);
        g_rxfile.f = 0;
        if (g_rxfile.total != g_rxfile.out_total)
            printf("\n%s [%s] << file '%s' (%d bytes, %d parts, %d B on "
                   "air = %.2fx compression) stored as %s\n> ", tstamp(),
                   g_name, name, g_rxfile.out_total, n_parts,
                   g_rxfile.total,
                   (double)g_rxfile.out_total / (double)g_rxfile.total,
                   g_rxfile.path);
        else
            printf("\n%s [%s] << file '%s' (%d bytes, %d parts) stored as "
                   "%s\n> ", tstamp(), g_name, name, g_rxfile.out_total,
                   n_parts, g_rxfile.path);
    } else {
        printf("\n%s [%s] << file '%s': part %d/%d (%d bytes so far)\n> ",
               tstamp(), g_name, name, part + 1, n_parts,
               g_rxfile.out_total);
    }
}

static void handle_delivered(int before)
{
    int i;
    for (i = before; i < g_st.delivered_n; i++) {
        const uint8_t *m = g_st.delivered[i];
        int len = g_st.delivered_len[i];
        if (len > 1 + (int)strlen(FILE_TAG)
            && (m[0] == FILE_MAGIC || m[0] == FILE_MAGIC_Z)
            && !memcmp(m + 1, FILE_TAG, strlen(FILE_TAG))) {
            store_file(m, len, m[0] == FILE_MAGIC_Z);
        } else {
            printf("\n%s [%s] << message (%d bytes): ", tstamp(), g_name,
                   len);
            fwrite(m, 1, (size_t)(len < 120 ? len : 120), stdout);
            printf("\n> ");
        }
        fflush(stdout);
    }
    /* the delivered log is a bounded ring; once handled, recycle it so a
     * long-lived console session never stops recording new messages */
    if (g_st.delivered_n >= ST_DELIVERED_MAX)
        g_st.delivered_n = 0;
}

/* Queue a broadcast: one preamble+header per group of BC frames, framing
 * as ofdm_phy/broadcast.py (SYNC|EOS|seq, then this frame's length). */
static void cmd_broadcast(const char *text, int rung);

static void cmd_broadcast(const char *text, int rung)
{
    static uint8_t blocks[8 * 2600];
    uint8_t payload[26];
    int fb = 26, cap0 = 26 - 3, cap = 26 - 2;
    int len = (int)strlen(text), off = 0, seq = 0, total = 0;

    if (len <= 0)
        return;
    g_bc_pending = g_bc_sent = 0;
    while (off < len) {
        int nf = 0, pkt_n = 0, first = 1, n;
        while (nf < g_bc_group && off < len) {
            int take = (first ? cap0 : cap);
            int flags = first ? 0x80 : 0;
            if (take > len - off)
                take = len - off;
            if (off + take >= len)
                flags |= 0x40; /* EOS */
            memset(payload, 0, sizeof(payload));
            payload[0] = (uint8_t)(flags | (seq & 0x3F));
            payload[1] = (uint8_t)take;
            if (first)
                payload[2] = BC_PT_TELEMETRY;
            memcpy(payload + (first ? 3 : 2), text + off, (size_t)take);
            pkt_n = data_encode(0, payload, fb, blocks + (size_t)nf * 2600);
            off += take;
            seq++;
            nf++;
            first = 0;
        }
        {   /* pack contiguously at the packet stride the builder wants */
            static uint8_t packed[8 * 2600];
            int i;
            for (i = 0; i < nf; i++)
                memcpy(packed + (size_t)i * pkt_n, blocks + (size_t)i * 2600,
                       (size_t)pkt_n);
            n = tx_build_burst(ladder_mode(rung), packed, pkt_n, nf,
                               PKT_TYP_BCAST, ladder_mod(rung),
                               ladder_spd(rung), BURST_STREAM_RESYNC,
                               g_bc_air + g_bc_pending);
        }
        if (n <= 0) {
            /* The group did not fit the transmit buffer -- at the slow
             * rungs one EXTREME frame is already 20 s of air, so four of
             * them behind a single preamble is far past it. Shrink the
             * group and retry; a group of one is still a valid broadcast,
             * it just stops amortizing the preamble. */
            if (g_bc_group > 1) {
                g_bc_group /= 2;
                printf("%s [%s] broadcast: group too long at rung %d, "
                       "retrying with %d frame(s) per group\n", tstamp(),
                       g_name, rung, g_bc_group);
                g_bc_pending = 0;
                cmd_broadcast(text, rung);
                return;
            }
            printf("%s [%s] broadcast: will not fit at rung %d -- let the "
                   "ladder climb first\n", tstamp(), g_name, rung);
            g_bc_pending = 0;
            return;
        }
        g_bc_pending += n;
        total++;
    }
    printf("%s [%s] broadcast queued: %d bytes in %d group(s), %.1f s air "
           "at rung %d (non-ARQ, nothing will be repeated)\n", tstamp(),
           g_name, len, total, (double)g_bc_pending / FS, rung);
}

static int g_compress = 1; /* deflate files before transmitting */

static void cmd_sendfile(const char *path)
{
    static uint8_t env[ST_MSG_MAX];
    static uint8_t src[FILE_MAX_SRC], zbuf[FILE_MAX_SRC];
    const uint8_t *body;
    const char *base = strrchr(path, '/');
    FILE *f = fopen(path, "rb");
    long fsz;
    uLongf zlen = 0;
    int head, n_parts, p, q_free, magic = FILE_MAGIC, body_len;

    base = base ? base + 1 : path;
    if (!f) {
        printf("%s [%s] sendfile: cannot open %s\n", tstamp(), g_name, path);
        return;
    }
    fseek(f, 0, SEEK_END);
    fsz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (fsz < 0 || fsz > FILE_MAX_SRC) {
        printf("%s [%s] sendfile: %s is %ld bytes, cap is %d\n", tstamp(),
               g_name, path, fsz, FILE_MAX_SRC);
        fclose(f);
        return;
    }
    if (fsz > 0 && fread(src, 1, (size_t)fsz, f) != (size_t)fsz) {
        printf("%s [%s] sendfile: read error\n", tstamp(), g_name);
        fclose(f);
        return;
    }
    fclose(f);

    /* compress the whole file once; keep it only if it actually shrank
     * (already-compressed payloads inflate slightly under deflate) */
    body = src;
    body_len = (int)fsz;
    if (g_compress && fsz > 0) {
        zlen = compressBound((uLong)fsz);
        if (zlen <= sizeof(zbuf)
            && compress2(zbuf, &zlen, src, (uLong)fsz, 9) == Z_OK
            && (long)zlen < fsz) {
            body = zbuf;
            body_len = (int)zlen;
            magic = FILE_MAGIC_Z;
        }
    }

    head = 1 + (int)strlen(FILE_TAG) + (int)strlen(base) + 1 + 2;
    n_parts = body_len <= 0 ? 1
                            : (body_len + FILE_PART_DATA - 1) / FILE_PART_DATA;
    q_free = ST_MAX_MSGS - g_st.qcount[QOS_BULK];
    if (n_parts > 255 || n_parts > q_free) {
        printf("%s [%s] sendfile: %s is %ld bytes (%d on air) = %d parts, "
               "but only %d queue slots free (max %d bytes now)\n", tstamp(),
               g_name, path, fsz, body_len, n_parts, q_free,
               q_free * FILE_PART_DATA);
        return;
    }

    env[0] = (uint8_t)magic;
    memcpy(env + 1, FILE_TAG, strlen(FILE_TAG));
    memcpy(env + 1 + strlen(FILE_TAG), base, strlen(base) + 1);
    for (p = 0; p < n_parts; p++) {
        int dlen = body_len - p * FILE_PART_DATA;
        if (dlen > FILE_PART_DATA)
            dlen = FILE_PART_DATA;
        if (dlen < 0)
            dlen = 0;
        env[head - 2] = (uint8_t)p;
        env[head - 1] = (uint8_t)n_parts;
        memcpy(env + head, body + (size_t)p * FILE_PART_DATA, (size_t)dlen);
        if (station_submit(&g_st, env, head + dlen, QOS_BULK) != 0) {
            printf("%s [%s] sendfile: queue full at part %d\n", tstamp(),
                   g_name, p + 1);
            return;
        }
    }
    if (magic == FILE_MAGIC_Z)
        printf("%s [%s] queued file '%s' (%ld bytes -> %d on air, %.2fx, "
               "%d part%s, burst-ARQ bulk)\n", tstamp(), g_name, base, fsz,
               body_len, (double)fsz / (double)body_len, n_parts,
               n_parts == 1 ? "" : "s");
    else
        printf("%s [%s] queued file '%s' (%ld bytes, %d part%s, burst-ARQ "
               "bulk)\n", tstamp(), g_name, base, fsz, n_parts,
               n_parts == 1 ? "" : "s");
}

/* oscillator fine-tune actuator: the virtual channel has no trimmable
 * LO, so this stub only reports what real hardware (VCTCXO DAC, CAT
 * clarifier) would be asked to do */
static void lo_trim(void *ctx, double hz)
{
    (void)ctx;
    printf("%s [%s] LO trim %+.1f Hz (accumulated %+.1f Hz)\n", tstamp(),
           g_name, hz, station_freq_trim_total(&g_st) + hz);
}

static void handle_command(char *line)
{
    if (!strncmp(line, "send ", 5)) {
        const char *msg = line + 5;
        if (station_submit(&g_st, (const uint8_t *)msg, (int)strlen(msg),
                           QOS_INTERACTIVE) == 0)
            printf("%s [%s] queued %zu bytes (interactive)\n", tstamp(),
                   g_name, strlen(msg));
        else
            printf("%s [%s] queue full\n", tstamp(), g_name);
    } else if (!strncmp(line, "sendfile ", 9)) {
        cmd_sendfile(line + 9);
    } else if (!strncmp(line, "bulk ", 5)) {
        int n = atoi(line + 5), i;
        uint8_t buf[ST_MSG_MAX];
        if (n < 1 || n > ST_MSG_MAX) {
            printf("[%s] bulk size 1..%d\n", g_name, ST_MSG_MAX);
            return;
        }
        for (i = 0; i < n; i++)
            buf[i] = (uint8_t)(i * 13 + 7);
        if (station_submit(&g_st, buf, n, QOS_BULK) == 0)
            printf("[%s] queued %d-byte test pattern (bulk)\n", g_name, n);
    } else if (!strcmp(line, "debug on")) {
        g_debug = 1;
        printf("%s [%s] diagnostic event stream ON\n", tstamp(), g_name);
    } else if (!strcmp(line, "debug off")) {
        g_debug = 0;
        printf("%s [%s] diagnostic event stream OFF\n", tstamp(), g_name);
    } else if (!strncmp(line, "window ", 7)) {
        int w = atoi(line + 7);
        if (w < 0)
            w = 0;
        if (w > BURST_STREAM_MAX)
            w = BURST_STREAM_MAX;
        g_st.burst_window = w;
        printf("%s [%s] burst window = %d fragment%s per acknowledgment "
               "(stream cap %d)\n", tstamp(), g_name, w,
               w == 1 ? "" : "s", BURST_STREAM_MAX);
    } else if (!strncmp(line, "broadcast ", 10)) {
        cmd_broadcast(line + 10, g_st.stats.last_rung >= 0
                                     ? g_st.stats.last_rung : 7);
    } else if (!strcmp(line, "compress on")
               || !strcmp(line, "compress off")) {
        g_compress = line[9] == 'o' && line[10] == 'n';
        printf("%s [%s] file compression %s\n", tstamp(), g_name,
               g_compress ? "ON (deflate)" : "OFF");
    } else if (!strcmp(line, "stream on") || !strcmp(line, "stream off")) {
        g_st.burst_stream = line[7] == 'o' && line[8] == 'n';
        printf("%s [%s] streamed burst windows %s\n", tstamp(), g_name,
               g_st.burst_stream ? "ON" : "OFF (one preamble per fragment)");
    } else if (!strncmp(line, "tune ", 5)) {
        double d = station_freq_trim(&g_st, atof(line + 5));
        printf("%s [%s] manual LO trim: applied %+.1f Hz, total %+.1f Hz\n",
               tstamp(), g_name, d, station_freq_trim_total(&g_st));
    } else if (!strcmp(line, "status")) {
        show_status();
    } else if (!strcmp(line, "stats")) {
        show_stats();
    } else if (!strcmp(line, "quit") || !strcmp(line, "exit")) {
        exit(0);
    } else if (line[0]) {
        printf("commands: send <text> | sendfile <path> | bulk <n> | "
               "tune <hz> | debug on|off | status | stats | quit\n");
    }
}

/* ---------------- main loop ---------------- */

int main(int argc, char **argv)
{
    struct sockaddr_un sa;
    /* receive_burst stays NULL: this app decodes with the streaming
     * receiver, which continues a burst through rxs_continue_burst()
     * rather than re-running a whole recording */
    station_phy_t phy = { 0, phy_build, phy_receive_unused,
                          phy_build_burst, 0 };
    int16_t chunk[4096];

    if (argc < 2) {
        fprintf(stderr, "usage: %s <device.sock> [name]\n", argv[0]);
        return 1;
    }
    if (argc >= 3)
        g_name = argv[2];

    g_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    memset(&sa, 0, sizeof(sa));
    sa.sun_family = AF_UNIX;
    strncpy(sa.sun_path, argv[1], sizeof(sa.sun_path) - 1);
    if (connect(g_fd, (struct sockaddr *)&sa, sizeof(sa)) != 0) {
        fprintf(stderr, "connect %s: %s\n", argv[1], strerror(errno));
        return 1;
    }

    station_init(&g_st, &phy, (uint64_t)getpid());
    /* the ceiling is what the buffers allow; the station picks the
     * actual window from what the channel delivers (ST_EV_BURST_WIN) */
    g_st.burst_window = BURST_STREAM_MAX;
    g_st.burst_stream = 1; /* ...sent behind one preamble when possible */
    /* AFC endpoint: the s1-device station anchors (frequency
     * reference), the other side trims */
    station_set_freq_trim(&g_st, lo_trim, 0, 150.0,
                          strstr(argv[1], "s1") != 0);
    station_set_diag(&g_st, diag_print, 0); /* prints only with g_debug */
    /* modes are self-labeling by ZC preamble root: one streaming receiver
     * per mode, all fed the same audio -- only the sender's mode locks */
    g_rxs[0] = rxs_open(MODE_NORMAL, 0);
    g_rxs[1] = rxs_open(MODE_ROBUST, 0);
    g_rxs[2] = rxs_open(MODE_EXTREME, 0);
    printf("[%s] connected to %s -- protocol bootstraps at EXTREME; "
           "type 'send <text>'\n> ", g_name, argv[1]);
    fflush(stdout);

    for (;;) {
        fd_set rf;
        struct timeval tv = { 0, 20000 };
        FD_ZERO(&rf);
        FD_SET(g_fd, &rf);
        FD_SET(0, &rf);
        if (select(g_fd + 1, &rf, 0, 0, &tv) < 0)
            break;

        if (FD_ISSET(g_fd, &rf)) {
            ssize_t got = read(g_fd, chunk, sizeof(chunk));
            int n;
            if (got <= 0) {
                fprintf(stderr, "[%s] device closed\n", g_name);
                break;
            }
            n = (int)(got / 2);
            g_rx_total += n;
            note_busy(chunk, n);
            {
                int m;
                for (m = 0; m < 3; m++) {
                    rxs_event_t ev;
                    if (!rxs_push(g_rxs[m], chunk, n, &ev)) {
                        continue;
                    }
                    g_rx_events++;
                    /* A streamed burst: keep taking blocks from the
                     * deterministic offsets instead of hunting for the
                     * next preamble.
                     *
                     * Knowing where to STOP is the whole game. While the
                     * receiver is stepping through blocks it is not
                     * running the preamble detector, so chasing blocks
                     * that were never sent makes it deaf to the peer's
                     * next transmission for one block-time each. The
                     * transmitter marks the last block of a burst with
                     * the ack request, so that is the stop signal; the
                     * FIRST block carries it too (for peers that cannot
                     * stream at all), hence the "not the first" test. A
                     * consecutive-failure bound catches the case where
                     * the last block itself did not decode. */
                    {
                        int ackreq = 0, streamed;
                        streamed = ev.type == 1
                                   && frame_is_streamed(ev.bits,
                                                        ev.pkt_bits_n,
                                                        &ackreq);
                        if (streamed && g_burst_left[m] == 0) {
                            g_burst_left[m] = BURST_STREAM_MAX - 1;
                            g_burst_miss[m] = 0;
                        } else if (streamed) {
                            g_burst_miss[m] = 0;
                            if (ackreq)
                                g_burst_left[m] = 0; /* burst complete */
                            else
                                g_burst_left[m]--;
                        } else if (g_burst_left[m] > 0) {
                            /* a block we could not decode: keep going so
                             * one bad block does not cost the tail, but
                             * not forever */
                            if (++g_burst_miss[m] >= 2)
                                g_burst_left[m] = 0;
                            else
                                g_burst_left[m]--;
                        }
                        if (g_burst_left[m] > 0
                            && !rxs_continue_burst(g_rxs[m],
                                                   BURST_STREAM_RESYNC))
                            g_burst_left[m] = 0;
                    }
                    if (g_debug && ev.type != 1) {
                        static const char *why[] = {
                            "?", "header CRC", "bad ver", "data CRC" };
                        printf("%s [%s] dbg t=%.1f %-12s mode=%d (%s)\n",
                               tstamp(), g_name, now_t(), "RX_FAIL", m,
                               why[ev.type >= -3 && ev.type < 0
                                   ? -ev.type : 0]);
                        fflush(stdout);
                    }
                    if (ev.type == 1 && ev.hdr.typ == PKT_TYP_BCAST) {
                        /* Broadcast: Data-shaped but NOT station traffic --
                         * its reserved field is not a link-control word, so
                         * it must never reach the ARQ reassembler. */
                        const uint8_t *b = ev.bits;
                        int plen = (ev.pkt_bits_n - 36) / 8, j, v;
                        int flags = 0, seq = 0, dlen = 0, head = 2;
                        for (j = 0, v = 0; j < 8; j++)
                            v = (v << 1) | (b[20 + j] & 1);
                        flags = v & ~0x3F;
                        seq = v & 0x3F;
                        for (j = 0, v = 0; j < 8; j++)
                            v = (v << 1) | (b[28 + j] & 1);
                        dlen = v;
                        if (flags & 0x80) {
                            head = 3;
                            g_bc_left[m] = g_bc_group - 1;
                        } else if (g_bc_left[m] > 0) {
                            g_bc_left[m]--;
                        }
                        if (g_bc_last_seq >= 0) {
                            int gap = (seq - g_bc_last_seq - 1) & 0x3F;
                            if (gap > 0 && gap < 32)
                                g_bc_lost += gap;
                        }
                        g_bc_last_seq = seq;
                        g_bc_frames++;
                        if (dlen > plen - head)
                            dlen = plen - head;
                        for (j = 0; j < dlen
                             && g_bc_len < (int)sizeof(g_bc_asm); j++) {
                            int bb, val = 0;
                            for (bb = 0; bb < 8; bb++)
                                val = (val << 1)
                                      | (b[20 + 8 * (head + j) + bb] & 1);
                            g_bc_asm[g_bc_len++] = (uint8_t)val;
                        }
                        if (g_bc_left[m] > 0
                            && !rxs_continue_burst(g_rxs[m],
                                                   BURST_STREAM_RESYNC))
                            g_bc_left[m] = 0;
                        if (flags & 0x40) {
                            printf("\n%s [%s] << broadcast: %d bytes, "
                                   "%d frames, %d lost, snr %+.1f dB\n",
                                   tstamp(), g_name, g_bc_len, g_bc_frames,
                                   g_bc_lost, ev.snr_db);
                            printf("    \"%.*s\"\n> ", g_bc_len, g_bc_asm);
                            fflush(stdout);
                            g_bc_len = g_bc_frames = g_bc_lost = 0;
                            g_bc_last_seq = -1;
                        }
                    } else if (ev.type == 1) {
                        int before = g_st.delivered_n;
                        g_rx_ok++;
                        g_last_snr = ev.snr_db;
                        g_last_cfo = (double)ev.cfo_word * FS / 4294967296.0;
                        station_on_decoded(&g_st, ev.bits, ev.pkt_bits_n,
                                           ev.snr_db, g_last_cfo, 0,
                                           now_t());
                        handle_delivered(before);
                    }
                }
            }
            if (g_txing && now_t() >= g_tx_end_t) {
                station_on_tx_end(&g_st, g_tx_end_t);
                g_txing = 0;
            }
            if (!g_txing) {
                int busy = channel_busy();
                static int prev_busy = -1;
                if (g_debug && busy != prev_busy) {
                    printf("%s [%s] dbg t=%.1f %-12s rms^2=%.3g "
                           "floor=%.3g\n", tstamp(), g_name, now_t(),
                           busy ? "CS_BUSY" : "CS_IDLE",
                           g_busy_acc / BUSY_WIN, g_noise_floor);
                    fflush(stdout);
                    prev_busy = busy;
                }
                int nf = 0;
                if (g_bc_pending > 0 && !busy && !g_txing) {
                    /* a broadcast owns the channel for its duration: there
                     * is no acknowledgment to wait for and nothing to
                     * retransmit, so it just goes out */
                    nf = g_bc_pending;
                    memcpy(g_frame, g_bc_air, (size_t)nf * sizeof(int16_t));
                    g_bc_pending = 0;
                    g_bc_sent = 1;
                }
                if (nf == 0)
                    nf = station_poll_tx(&g_st, now_t(), busy,
                                         g_frame, 600000);
                if (nf > 0) {
                    ssize_t off = 0;
                    const char *p = (const char *)g_frame;
                    while (off < (ssize_t)(nf * 2)) {
                        ssize_t w = write(g_fd, p + off,
                                          (size_t)(nf * 2 - off));
                        if (w <= 0)
                            break;
                        off += w;
                    }
                    g_txing = 1;
                    g_tx_end_t = now_t() + (double)nf / FS;
                    printf("%s [%s] >> frame at rung %d (%.1f s air time)\n> ",
                           tstamp(), g_name, g_st.stats.last_rung,
                           (double)nf / FS);
                    fflush(stdout);
                }
            }
        }

        if (FD_ISSET(0, &rf)) {
            /* raw read + our own line splitting: fgets() would pull the
             * whole readable block into stdio's buffer and hand back only
             * the first line, and select() cannot see what is already
             * buffered in user space -- so any command typed (or piped)
             * behind another one sat unprocessed until the *next* input
             * arrived. */
            static char inbuf[1024];
            static size_t inlen;
            ssize_t got = read(0, inbuf + inlen, sizeof(inbuf) - inlen - 1);
            char *nl;
            if (got <= 0)
                break;
            inlen += (size_t)got;
            inbuf[inlen] = 0;
            while ((nl = memchr(inbuf, '\n', inlen)) != 0) {
                size_t used = (size_t)(nl - inbuf) + 1;
                *nl = 0;
                handle_command(inbuf);
                printf("> ");
                fflush(stdout);
                memmove(inbuf, inbuf + used, inlen - used);
                inlen -= used;
            }
            if (inlen >= sizeof(inbuf) - 1)
                inlen = 0;              /* overlong line: drop it */
        }
    }
    return 0;
}
