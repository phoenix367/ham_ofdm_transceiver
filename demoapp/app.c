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
 *            sendfile <path>  transfer a file (bulk class, <= ~4 KB)
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
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

/* app-level envelope: files travel as
 *   \x01 FILE: <name> \0 <part_idx> <n_parts> <data>
 * split into parts small enough for one burst-ARQ transfer each
 * (127 fragments x 25 bytes); parts arrive in order (FIFO bulk queue) */
#define FILE_MAGIC 0x01
#define FILE_TAG "FILE:"
#define FILE_PART_DATA 3000

#include "../cport/src/link.h"
#include "../cport/src/station.h"
#include "../cport/src/tx.h"
#include "../cport/src/rx_stream.h"
#include "../cport/src/packets.h"

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

static int phy_receive_unused(void *ctx, const int16_t *s, int n,
                              uint8_t *b, int *bn, double *snr, double *cfo,
                              int *hc, const int64_t *pl, int pn,
                              int64_t *lo, int *ln)
{
    (void)ctx; (void)s; (void)n; (void)b; (void)bn; (void)snr; (void)cfo;
    (void)hc; (void)pl; (void)pn; (void)lo; (void)ln;
    return -1; /* the app decodes via the streaming receiver instead */
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

static int channel_busy(void)
{
    double p = g_busy_acc / BUSY_WIN;
    /* noise floor: fast to drop, slow to rise -- adapts to whatever the
     * channel's quiet level is (carrier sense must be relative; frames
     * below the noise floor are invisible to energy detection anyway) */
    if (p < g_noise_floor)
        g_noise_floor = p;
    else
        g_noise_floor *= 1.0005;
    if (g_noise_floor < 25.0)
        g_noise_floor = 25.0;
    return p > BUSY_RATIO * BUSY_RATIO * g_noise_floor;
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
    int next_part, n_parts, total;
    FILE *f;
} g_rxfile;

static void store_file(const uint8_t *msg, int len)
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

    fwrite(meta + 2, 1, (size_t)data_len, g_rxfile.f);
    g_rxfile.total += data_len;
    g_rxfile.next_part++;
    if (g_rxfile.next_part >= g_rxfile.n_parts) {
        fclose(g_rxfile.f);
        g_rxfile.f = 0;
        printf("\n%s [%s] << file '%s' (%d bytes, %d parts) stored as "
               "%s\n> ", tstamp(), g_name, name, g_rxfile.total, n_parts,
               g_rxfile.path);
    } else {
        printf("\n%s [%s] << file '%s': part %d/%d (%d bytes so far)\n> ",
               tstamp(), g_name, name, part + 1, n_parts, g_rxfile.total);
    }
}

static void handle_delivered(int before)
{
    int i;
    for (i = before; i < g_st.delivered_n; i++) {
        const uint8_t *m = g_st.delivered[i];
        int len = g_st.delivered_len[i];
        if (len > 1 + (int)strlen(FILE_TAG) && m[0] == FILE_MAGIC
            && !memcmp(m + 1, FILE_TAG, strlen(FILE_TAG))) {
            store_file(m, len);
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

static void cmd_sendfile(const char *path)
{
    static uint8_t env[ST_MSG_MAX];
    const char *base = strrchr(path, '/');
    FILE *f = fopen(path, "rb");
    long fsz;
    int head, n_parts, p, q_free;

    base = base ? base + 1 : path;
    if (!f) {
        printf("%s [%s] sendfile: cannot open %s\n", tstamp(), g_name, path);
        return;
    }
    fseek(f, 0, SEEK_END);
    fsz = ftell(f);
    fseek(f, 0, SEEK_SET);

    head = 1 + (int)strlen(FILE_TAG) + (int)strlen(base) + 1 + 2;
    n_parts = fsz <= 0 ? 1 : (int)((fsz + FILE_PART_DATA - 1)
                                   / FILE_PART_DATA);
    q_free = ST_MAX_MSGS - g_st.qcount[QOS_BULK];
    if (n_parts > 255 || n_parts > q_free) {
        printf("%s [%s] sendfile: %s is %ld bytes = %d parts, but only %d "
               "queue slots free (max %d bytes now)\n", tstamp(), g_name,
               path, fsz, n_parts, q_free, q_free * FILE_PART_DATA);
        fclose(f);
        return;
    }

    env[0] = FILE_MAGIC;
    memcpy(env + 1, FILE_TAG, strlen(FILE_TAG));
    memcpy(env + 1 + strlen(FILE_TAG), base, strlen(base) + 1);
    for (p = 0; p < n_parts; p++) {
        int dlen = (int)(fsz - (long)p * FILE_PART_DATA);
        if (dlen > FILE_PART_DATA)
            dlen = FILE_PART_DATA;
        if (dlen < 0)
            dlen = 0;
        env[head - 2] = (uint8_t)p;
        env[head - 1] = (uint8_t)n_parts;
        if (fread(env + head, 1, (size_t)dlen, f) != (size_t)dlen) {
            printf("%s [%s] sendfile: read error\n", tstamp(), g_name);
            fclose(f);
            return;
        }
        if (station_submit(&g_st, env, head + dlen, QOS_BULK) != 0) {
            printf("%s [%s] sendfile: queue full at part %d\n", tstamp(),
                   g_name, p + 1);
            fclose(f);
            return;
        }
    }
    fclose(f);
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
    station_phy_t phy = { 0, phy_build, phy_receive_unused };
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
    g_st.burst_window = 8; /* selective-repeat bursts for bulk transfers */
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
                    if (!rxs_push(g_rxs[m], chunk, n, &ev))
                        continue;
                    g_rx_events++;
                    if (ev.type == 1) {
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
                int nf = station_poll_tx(&g_st, now_t(), channel_busy(),
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
            static char line[512];
            if (!fgets(line, sizeof(line), stdin))
                break;
            line[strcspn(line, "\n")] = 0;
            handle_command(line);
            printf("> ");
            fflush(stdout);
        }
    }
    return 0;
}
