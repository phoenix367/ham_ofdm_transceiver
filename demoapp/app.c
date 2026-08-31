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
#define FILE_MAGIC 0x01        /* part(1) n_parts(1): 255 parts */
#define FILE_MAGIC_Z 0x02      /* the same, DEFLATEd whole */
/* Wide envelope: part(2) n_parts(2), little-endian. The byte-sized
 * index capped a transfer at 255 x ~230 B = 58 kB over USB, which a
 * 68 kB PNG hit on the stand. Senders emit the wide form; receivers
 * take both, so an old console still receives from a new one as long
 * as the file fits its cap. */
#define FILE_MAGIC_W 0x03
#define FILE_MAGIC_WZ 0x04
#define FILE_IS_MAGIC(m) ((m) >= FILE_MAGIC && (m) <= FILE_MAGIC_WZ)
#define FILE_IS_ZIPPED(m) ((m) == FILE_MAGIC_Z || (m) == FILE_MAGIC_WZ)
#define FILE_IS_WIDE(m) ((m) >= FILE_MAGIC_W)
#define FILE_TAG "FILE:"
#define FILE_PART_DATA 3000
#define FILE_MAX_SRC (1 << 19) /* 512 KB source cap before compression */
/* Peer silence past which a bulk transfer probes before committing. Below
 * ctl_tx_rung()'s STALE_S (90 s), so the probe happens BEFORE the first
 * rung of decay rather than after it. */
#define BULK_PROBE_STALE_S 60.0

#include "../cport/src/link.h"
#include "../cport/src/station.h"
#include "../cport/src/tx.h"
#include "../cport/src/rx_stream.h"
#include "../cport/src/usb_proto.h"
#include "usb_host.h"
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
/* consecutive undecodable blocks that end a broadcast group: the EOS
 * marker rides in a frame, so losing that frame would otherwise leave the
 * receiver walking phantom blocks to the end of the advertised group */
#ifndef BC_MAX_MISS
#define BC_MAX_MISS 4
#endif
static int g_bc_miss[3];
static int g_bc_group = 4;
/* A broadcast payload goes STRAIGHT TO DISK. It used to accumulate in
 * this RAM buffer and be written once at EOS, which silently truncated
 * anything larger than the buffer: a 14162-byte file arrived as "8192
 * bytes" with nothing in the summary hinting that 42% was missing.
 * Streaming also means the open (and any failure) is reported when the
 * broadcast STARTS, not minutes later when it ends.
 *
 * The buffer remains for the text/telemetry path, which is small by
 * nature and is printed rather than stored -- it now says so when it
 * fills. */
/* A broadcast is a TRAIN of transmissions with short gaps between them
 * (the sender yields to carrier sense between turns). Those gaps read as
 * idle, so a station holding a pending reply fires into one -- and its
 * own keying makes it deaf just as the next transmission's preamble goes
 * by. Losing a group's preamble costs the WHOLE group with nothing to
 * repeat it: measured, one 0.8 s reply cost exactly 16 frames on a
 * +20 dB channel, which looked like channel errors and was not.
 *
 * So a station holds off while it is hearing a broadcast. The hold is
 * expressed as "the channel is busy" rather than as a transmit veto, so
 * the link layer does not count the resulting reply timeouts as losses
 * (a timeout on a busy channel is deliberately not a loss) and the rate
 * ladder is left alone. It spans longer than one group so that a lost
 * group does not release it mid-broadcast, and EOS clears it at once. */
#define BC_RX_HOLD_S 12.0
static double g_bc_rx_last = -1e9;

/* NOTE: there is deliberately no listener->sender reception report
 * here. It was implemented and REVERTED after measurement.
 *
 * The idea is sound and the gap is real -- a broadcast at rung 8 while
 * the listener decodes it at +7.7 dB wastes 5x the air. But on a simplex
 * channel the listener has to transmit to report, and transmitting is
 * exactly what stops it hearing. Measured against a baseline that
 * delivered 14162/14162 bytes with 0 frames lost:
 *
 *   reports every group  24 reports/10 turns, 32 lost, rung 8->11
 *   one report per turn  15 reports/16 turns, 28 lost, rung 8->7 (down)
 *
 * The second run is the damning one: rate limiting removed the storm and
 * bought nothing -- same turn count, 28 frames lost for good, and the
 * rung moved DOWN, because the listener's own deafness became losses and
 * the losses made the controller conservative.
 *
 * The root cause is not tuning: the listener infers "the sender paused"
 * from quiet on ITS clock, but it decodes BEHIND the sender (lag of
 * seconds; 11.7 s was measured on a 16-block group). Its reply therefore
 * lands inside the next transmission whatever threshold is chosen, and a
 * window wide enough to absorb the lag costs more air than the rung
 * climb saves.
 *
 * Doing this properly means the SENDER signalling an explicit report
 * slot on the wire -- a flag on the last frame of each turn, so the
 * listener answers a marker instead of inferred silence. That is a frame
 * format change across the Python/C/demoapp twins and the golden
 * vectors, not an app-layer heuristic. Until then a broadcast runs at
 * whatever rung the link last established. */

#define BC_RX_PATH "rx_broadcast.bin"
static FILE *g_bc_file;
static long g_bc_written;
static int g_bc_trunc;
static uint8_t g_bc_asm[8192];
static int g_bc_len, g_bc_frames, g_bc_lost, g_bc_last_seq = -1;
static int g_bc_ptype = -1;   /* descriptor from the SYNC frame */
static int g_bc_rx_group = 4; /* group size, read off the wire */
static int16_t g_bc_air[600000];
static int g_bc_pending;   /* samples queued for transmission */
static int g_bc_sent;

/* ---------------- PHY glue ---------------- */

/* Render a frame or burst through the streaming transmitter.
 *
 * The generator produces the waveform on demand from one 128-sample IFFT
 * tile, so nothing here holds a frame: it is what lets the C port drop
 * tx.c's 4.3 MB g_sig entirely (--gc-sections removes it once
 * tx_build_frame/tx_build_burst are unreferenced). Bit-identical to the
 * frame-at-once path, which cport/tests/test_tx.c asserts. */
static int build_streamed(int rung, const uint8_t *blocks, int pkt_n,
                          int n_blocks, int typ, int resync_every,
                          int16_t *out, int out_cap)
{
    int total = 0, got, pos = 0;
    txs_t *t = txs_open(ladder_mode(rung), blocks, pkt_n, n_blocks, typ,
                        ladder_mod(rung), ladder_spd(rung), resync_every, 0,
                        &total);

    if (!t || total < 0 || total > out_cap)
        return -1;
    while ((got = txs_pull(t, out + pos, out_cap - pos)) > 0)
        pos += got;
    return pos == total ? pos : -1;
}

static int phy_build(void *ctx, const uint8_t *bits, int n, int typ,
                     int rung, int16_t *out, int out_cap)
{
    (void)ctx;
    return build_streamed(rung, bits, n, 1, typ, 0, out, out_cap);
}

static int phy_build_burst(void *ctx, const uint8_t *blocks, int pkt_n,
                           int n_blocks, int typ, int rung, int resync_every,
                           int16_t *out, int out_cap)
{
    (void)ctx;
    return build_streamed(rung, blocks, pkt_n, n_blocks, typ, resync_every,
                          out, out_cap);
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

/* Open the broadcast sink once the descriptor says the payload is
 * opaque. Anything already buffered (frames that arrived before the
 * SYNC) is flushed into it first. */
static void bc_sink_open(void)
{
    if (g_bc_file || g_bc_ptype != BC_PT_OPAQUE)
        return;
    g_bc_file = fopen(BC_RX_PATH, "wb");
    if (!g_bc_file) {
        printf("\n%s [%s] broadcast: cannot open %s for writing: %s\n> ",
               tstamp(), g_name, BC_RX_PATH, strerror(errno));
        fflush(stdout);
        return;
    }
    if (g_bc_len > 0) {
        fwrite(g_bc_asm, 1, (size_t)g_bc_len, g_bc_file);
        g_bc_written += g_bc_len;
        g_bc_len = 0;
    }
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
    printf("  message store: %d/%d slots live, peak %d, refused %d\n",
           g_st.pool_used, ST_POOL_SLOTS, station_pool_peak(),
           station_pool_refused());
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

    int wide = FILE_IS_WIDE(msg[0]);
    name_len = (int)strnlen(name, (size_t)(len - 1 - (int)strlen(FILE_TAG)));
    meta = (const uint8_t *)name + name_len + 1;
    data_len = len - (int)(meta + (wide ? 4 : 2) - msg);
    if (data_len < 0) {
        printf("\n%s [%s] << malformed file envelope\n> ", tstamp(), g_name);
        return;
    }
    if (wide) {
        part = meta[0] | (meta[1] << 8);
        n_parts = meta[2] | (meta[3] << 8);
    } else {
        part = meta[0];
        n_parts = meta[1];
    }
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
        int rc = inflate_part(meta + (wide ? 4 : 2), data_len);
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
        fwrite(meta + (wide ? 4 : 2), 1, (size_t)data_len, g_rxfile.f);
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
        const uint8_t *m = station_delivered(&g_st, i);
        int len = g_st.delivered_len[i];
        if (len > 1 + (int)strlen(FILE_TAG)
            && FILE_IS_MAGIC(m[0])
            && !memcmp(m + 1, FILE_TAG, strlen(FILE_TAG))) {
            store_file(m, len, FILE_IS_ZIPPED(m[0]));
        } else {
            printf("\n%s [%s] << message (%d bytes): ", tstamp(), g_name,
                   len);
            fwrite(m, 1, (size_t)(len < 120 ? len : 120), stdout);
            printf("\n> ");
        }
        fflush(stdout);
    }
    /* Everything up to delivered_n has now been handled, so release it
     * immediately. Waiting for the log to fill first (which is what this
     * did) held every delivered payload in the message store until the
     * 16th message arrived -- harmless when each entry was its own
     * static array, wasteful now that they share slots. */
    station_delivered_reset(&g_st);
}


/* ---------------- USB modem mode ----------------
 *
 * `ofdm_console --usb [serial]` attaches to a BOARD instead of a
 * virtual channel. The difference is architectural, not cosmetic: a
 * socket device carries 12 kHz audio and this process runs the whole
 * PHY+station stack; a board already runs that stack in firmware and
 * speaks the message-level usb_proto (submit / message / status /
 * diag). So USB mode is a terminal onto a station that lives on the
 * other end of a cable -- the same commands, none of the DSP.
 *
 * What it shares with socket mode byte for byte is the FILE ENVELOPE
 * (magic FILE: name NUL part n_parts data, magic 2 = whole-file
 * DEFLATE), so files cross between any mix of consoles and boards.
 * Two board limits are respected: messages cap at ST_MSG_MAX (256 on
 * the boards, against this app's 4096), and with 12 pool slots a file
 * is PACED against the q_bulk depth the board reports twice a second
 * rather than dumped into the queue. */

/* The board tells us its ST_MSG_MAX in the INFO reply (2048 on the
 * radio firmware now, 256 before -- and 256 is what an older firmware
 * that does not report one gets). Once the capability handshake has
 * run, the PEER's limit applies too: a part the receiver cannot hold
 * is refused there, not here. */
#define USB_MSG_CAP 3328         /* buffer size: the largest we handle
                                  * (the radio boards' ST_MSG_MAX; run 3
                                  * of the throughput measurement split
                                  * at 2048 because this cap lagged the
                                  * boards -- the INFO value is only as
                                  * useful as the buffer behind it) */
static int g_umsg_max = 256;     /* this board's, from INFO */
static int usb_msg_max(void);
#define USB_INFLIGHT 4           /* file parts allowed in q_bulk */

static usbh_t *g_usb;
static up_parser_t g_up;
static up_status_t g_ust;
static int g_ust_valid;
/* outgoing file, split into ready-made messages */
/* enough for FILE_MAX_SRC at the smallest useful part: ~600 kB static,
 * which a host can afford and a 255-entry table could not deliver */
#define USB_PARTS_MAX 2400
static uint8_t g_uparts[USB_PARTS_MAX][USB_MSG_CAP];
static int g_upart_len[USB_PARTS_MAX];
static int g_upart_n, g_upart_sent;
static char g_upfile[128];
/* usb-mode broadcastfile: the file streams to the BOARD in chunks
 * (bit 7 of the ptype byte = more follows, bit 6 = continuation),
 * paced against the bc_free the status frame reports -- the board's
 * source buffer is 8 kB, not a file. Raw bytes, ptype OPAQUE, exactly
 * the socket-mode broadcastfile convention, so the receiver stores
 * rx_broadcast.bin whichever path carried it. */
#define UBCF_CHUNK 1024
static void usb_send_frame(uint8_t type, const void *payload, int len);
/* No file buffer and no size cap: the pump is already chunked and
 * paced against the board's bc_free, so it reads the file
 * SEQUENTIALLY -- the board never holds more than its 8 kB source
 * anyway, and a huge file is the operator's call (the board's ETA
 * line prices it). */
static FILE *g_ubcf_f;
static long g_ubcf_len, g_ubcf_off;
static int g_ubcf_rung;

static void usb_pump_bcfile(void)
{
    if (!g_ubcf_f)
        return;
    while (g_ust_valid && g_ust.bc_free >= 2 * UBCF_CHUNK) {
        uint8_t body[2 + UBCF_CHUNK];
        long left = g_ubcf_len - g_ubcf_off;
        int n = left > UBCF_CHUNK ? UBCF_CHUNK : (int)left;
        int more = g_ubcf_off + n < g_ubcf_len;
        if (n <= 0 || fread(body + 2, 1, (size_t)n, g_ubcf_f) != (size_t)n) {
            /* short read (file truncated underneath us): end the
             * broadcast cleanly rather than leave the board waiting
             * for chunks forever -- the stream is truncated, and says
             * so */
            printf("%s [%s] broadcastfile: read failed at %ld/%ld -- "
                   "broadcast truncated\n", tstamp(), g_name, g_ubcf_off,
                   g_ubcf_len);
            body[0] = (uint8_t)(0x0F | 0x40);
            body[1] = (uint8_t)g_ubcf_rung;
            body[2] = 0;
            usb_send_frame(UP_CMD_BCAST, body, 3);
            fclose(g_ubcf_f);
            g_ubcf_f = 0;
            break;
        }
        body[0] = (uint8_t)(0x0F | (more ? 0x80 : 0)
                            | (g_ubcf_off ? 0x40 : 0));
        body[1] = (uint8_t)g_ubcf_rung;
        usb_send_frame(UP_CMD_BCAST, body, n + 2);
        g_ubcf_off += n;
        g_ust.bc_free = (uint16_t)(g_ust.bc_free - n); /* until the next
                                                        * status frame */
        if (!more) {
            printf("%s [%s] broadcastfile: all %ld bytes handed to the "
                   "board (no delivery guarantee)\n", tstamp(), g_name,
                   g_ubcf_len);
            fclose(g_ubcf_f);
            g_ubcf_f = 0;
            break;
        }
    }
}

/* usb-mode broadcast reception (chunks stream in as UP_EVT_BCAST) */
static FILE *g_ubc_file;
static long g_ubc_written;
static uint8_t g_ubc_text[4096];
static int g_ubc_len, g_ubc_ptype = -1;

static int usb_msg_max(void)
{
    int m = g_umsg_max;
    if (g_ust_valid && g_ust.peer_state >= 2 && g_ust.peer_msg_max > 0
        && (int)g_ust.peer_msg_max < m)
        m = g_ust.peer_msg_max;
    return m > USB_MSG_CAP ? USB_MSG_CAP : m;
}

static void usb_send_frame(uint8_t type, const void *payload, int len)
{
    uint8_t out[UP_MAX_FRAME];
    int n = up_encode(type, payload, len, out, (int)sizeof(out));
    if (n > 0 && usbh_write(g_usb, out, n) != n)
        printf("%s [%s] usb write failed\n", tstamp(), g_name);
}

static void usb_submit(const uint8_t *data, int len, int qos)
{
    static uint8_t body[1 + USB_MSG_CAP];
    if (len > usb_msg_max()) {
        printf("%s [%s] message exceeds the board's %d-byte limit\n",
               tstamp(), g_name, usb_msg_max());
        return;
    }
    body[0] = (uint8_t)qos;
    memcpy(body + 1, data, (size_t)len);
    usb_send_frame(UP_CMD_SUBMIT, body, len + 1);
}

static void usb_pump_file(void)
{
    if (!g_upart_n || !g_ust_valid)
        return;
    while (g_upart_sent < g_upart_n && g_ust.q_bulk < USB_INFLIGHT) {
        usb_submit(g_uparts[g_upart_sent], g_upart_len[g_upart_sent],
                   QOS_BULK);
        g_ust.q_bulk++;          /* provisional; the next status corrects */
        g_upart_sent++;
    }
    if (g_upart_sent >= g_upart_n) {
        printf("\n%s [%s] file '%s': all %d part(s) handed to the board\n> ",
               tstamp(), g_name, g_upfile, g_upart_n);
        fflush(stdout);
        g_upart_n = g_upart_sent = 0;
    }
}

static void usb_sendfile(const char *path)
{
    static uint8_t src[FILE_MAX_SRC], zbuf[FILE_MAX_SRC + 4096];
    const char *base = strrchr(path, '/');
    const uint8_t *body;
    FILE *f = fopen(path, "rb");
    long fsz;
    uLongf zlen;
    int head, part_data, n_parts, i, magic = FILE_MAGIC, body_len;

    base = base ? base + 1 : path;
    if (g_upart_n) {
        printf("%s [%s] sendfile: a transfer is already in progress\n",
               tstamp(), g_name);
        if (f)
            fclose(f);
        return;
    }
    if (!f) {
        printf("%s [%s] sendfile: cannot open %s\n", tstamp(), g_name, path);
        return;
    }
    fseek(f, 0, SEEK_END);
    fsz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (fsz < 0 || fsz > FILE_MAX_SRC
        || (fsz > 0 && fread(src, 1, (size_t)fsz, f) != (size_t)fsz)) {
        printf("%s [%s] sendfile: read failed or over the %d-byte cap\n",
               tstamp(), g_name, FILE_MAX_SRC);
        fclose(f);
        return;
    }
    fclose(f);

    body = src;
    body_len = (int)fsz;
    zlen = compressBound((uLong)fsz);
    if (fsz > 0 && zlen <= sizeof(zbuf)
        && compress2(zbuf, &zlen, src, (uLong)fsz, 9) == Z_OK
        && (long)zlen < fsz) {
        body = zbuf;
        body_len = (int)zlen;
        magic = FILE_MAGIC_Z;
    }

    head = 1 + (int)strlen(FILE_TAG) + (int)strlen(base) + 1 + 4;
    part_data = usb_msg_max() - head;
    {   /* Align each part to the peer's streamed window: a part IS one
         * station message, the station fragments it at 200 B (top
         * rungs), and one window is answered by ONE acknowledgment. A
         * 2048-byte part at window 8 went out as 8+2+1 fragments =
         * three ack cycles; a window-sized part costs exactly one.
         * (win*200 is window-aligned at the mid rungs too: fragments
         * are 100 B there, so it is two full windows.) */
        int win = (g_ust_valid && g_ust.peer_state >= 2
                   && g_ust.peer_win_max > 0) ? g_ust.peer_win_max : 8;
        int target = win * 200 - head;
        if (target > 0 && part_data > target)
            part_data = target;
    }
    if (part_data <= 0) {
        printf("%s [%s] sendfile: name too long for a %d-byte message\n",
               tstamp(), g_name, usb_msg_max());
        return;
    }
    n_parts = body_len <= 0 ? 1 : (body_len + part_data - 1) / part_data;
    if (n_parts > USB_PARTS_MAX) {
        printf("%s [%s] sendfile: %d bytes on air needs %d parts, cap is "
               "%d\n", tstamp(), g_name, body_len, n_parts, USB_PARTS_MAX);
        return;
    }
    magic = magic == FILE_MAGIC_Z ? FILE_MAGIC_WZ : FILE_MAGIC_W;
    for (i = 0; i < n_parts; i++) {
        int dlen = body_len - i * part_data;
        uint8_t *env = g_uparts[i];
        if (dlen > part_data)
            dlen = part_data;
        if (dlen < 0)
            dlen = 0;
        env[0] = (uint8_t)magic;
        memcpy(env + 1, FILE_TAG, strlen(FILE_TAG));
        memcpy(env + 1 + strlen(FILE_TAG), base, strlen(base) + 1);
        env[head - 4] = (uint8_t)i;
        env[head - 3] = (uint8_t)(i >> 8);
        env[head - 2] = (uint8_t)n_parts;
        env[head - 1] = (uint8_t)(n_parts >> 8);
        memcpy(env + head, body + (size_t)i * part_data, (size_t)dlen);
        g_upart_len[i] = head + dlen;
    }
    g_upart_n = n_parts;
    g_upart_sent = 0;
    snprintf(g_upfile, sizeof(g_upfile), "%s", base);
    if (magic == FILE_MAGIC_WZ)
        printf("%s [%s] queued file '%s' (%ld bytes -> %d on air, %.2fx, "
               "%d part(s), paced against the board's queue)\n", tstamp(),
               g_name, base, fsz, body_len, (double)fsz / (double)body_len,
               n_parts);
    else
        printf("%s [%s] queued file '%s' (%ld bytes, %d part(s), paced "
               "against the board's queue)\n", tstamp(), g_name, base, fsz,
               n_parts);
    usb_pump_file();
}

static void usb_on_frame(void *ctx, uint8_t type, const uint8_t *pl, int len)
{
    (void)ctx;
    switch (type) {
    case UP_RSP_INFO: {
        up_info_t inf;
        if (up_decode_info(pl, len, &inf) == 0) {
            g_umsg_max = inf.msg_max ? inf.msg_max : 256;
            printf("%s [%s] modem: proto v%d fw %d.%d, %d modes @%u Hz, "
                   "messages up to %d B\n", tstamp(), g_name,
                   inf.proto_ver, inf.fw_ver >> 8, inf.fw_ver & 0xFF,
                   inf.n_modes, inf.sample_rate, g_umsg_max);
        }
        break;
    }
    case UP_EVT_STATUS:
        if (up_decode_status(pl, len, &g_ust) == 0) {
            g_ust_valid = 1;
            usb_pump_file();
            usb_pump_bcfile();
        }
        break;
    case UP_EVT_MESSAGE:
        if (len < 2)
            break;
        /* pl[0] = qos, then the message: classify exactly as the socket
         * path's handle_delivered does, sharing store_file() */
        if (len - 1 > 1 + (int)strlen(FILE_TAG)
            && FILE_IS_MAGIC(pl[1])
            && !memcmp(pl + 2, FILE_TAG, strlen(FILE_TAG))) {
            store_file(pl + 1, len - 1, FILE_IS_ZIPPED(pl[1]));
        } else {
            printf("\n%s [%s] << message (%d bytes): ", tstamp(), g_name,
                   len - 1);
            fwrite(pl + 1, 1, (size_t)(len - 1 < 120 ? len - 1 : 120),
                   stdout);
            printf("\n> ");
        }
        fflush(stdout);
        break;
    case UP_EVT_LOG:
        printf("\n%s [%s] board: %.*s\n> ", tstamp(), g_name, len, pl);
        fflush(stdout);
        break;
    case UP_EVT_BCAST:
        if (len < 1)
            break;
        if (pl[0] & BC_SYNC) {
            /* one start event per STREAM now (the firmware suppresses
             * per-group duplicates), so this is the moment to reset
             * the sink -- a stale open file belongs to a stream that
             * died without its EOS */
            g_ubc_ptype = pl[0] & 0x0F;
            if (g_ubc_file) {
                fclose(g_ubc_file);
                g_ubc_file = 0;
                g_ubc_written = 0;
            }
            g_ubc_len = 0;
            if (g_ubc_ptype == BC_PT_OPAQUE)
                g_ubc_file = fopen(BC_RX_PATH, "wb");
            printf("\n%s [%s] << broadcast starting (ptype %d%s)\n> ",
                   tstamp(), g_name, g_ubc_ptype,
                   g_ubc_ptype == BC_PT_OPAQUE ? ", storing" : "");
        } else if (pl[0] & BC_EOS) {
            int fo = len >= 5 ? pl[1] | (pl[2] << 8) : 0;
            int lo = len >= 5 ? pl[3] | (pl[4] << 8) : 0;
            double snr = len >= 7
                ? (int16_t)(pl[5] | (pl[6] << 8)) / 256.0 : 0.0;
            printf("\n%s [%s] << broadcast ended: %d frame(s), %d lost, "
                   "snr %+.1f dB (gaps are NOT repaired)\n", tstamp(),
                   g_name, fo, lo, snr);
            if (g_ubc_file) {
                fclose(g_ubc_file);
                g_ubc_file = 0;
                printf("    stored %ld bytes as %s\n", g_ubc_written,
                       BC_RX_PATH);
                g_ubc_written = 0;
            } else if (g_ubc_len) {
                printf("    \"%.*s\"\n", g_ubc_len, g_ubc_text);
                g_ubc_len = 0;
            }
            printf("> ");
        } else if (len > 1) {
            if (g_ubc_file) {
                fwrite(pl + 1, 1, (size_t)(len - 1), g_ubc_file);
                g_ubc_written += len - 1;
            } else {
                int room = (int)sizeof(g_ubc_text) - g_ubc_len;
                int c = len - 1 < room ? len - 1 : room;
                memcpy(g_ubc_text + g_ubc_len, pl + 1, (size_t)c);
                g_ubc_len += c;
            }
        }
        fflush(stdout);
        break;
    case UP_EVT_DIAG:
        if (len >= 21 && g_debug) {
            int32_t a, b, c, d;
            char line[160];
            memcpy(&a, pl + 1, 4); memcpy(&b, pl + 5, 4);
            memcpy(&c, pl + 9, 4); memcpy(&d, pl + 13, 4);
            station_diag_format(pl[0], a, b, c, d, line,
                                (int)sizeof(line));
            printf("\n%s [%s] . %s\n> ", tstamp(), g_name, line);
            fflush(stdout);
        }
        break;
    default:
        break;
    }
}

static const struct { const char *name; int key; } USB_CFG[] = {
    { "rung_ceiling", 1 }, { "burst_window", 2 }, { "burst_stream", 3 },
    { "freq_trim_mhz", 4 }, { "audio_tap", 5 }, { "anchor", 6 },
    { "diag_stream", 7 }, { "win_max", 8 },
};

static int usb_command(char *line)
{
    char *cmd = strtok(line, " \t\n");
    char *rest = strtok(0, "\n");
    if (!cmd)
        return 1;
    if (!strcmp(cmd, "quit") || !strcmp(cmd, "exit"))
        return 0;
    if (!strcmp(cmd, "send") && rest) {
        usb_submit((const uint8_t *)rest, (int)strlen(rest),
                   QOS_INTERACTIVE);
        printf("%s [%s] >> queued %d bytes (interactive)\n", tstamp(),
               g_name, (int)strlen(rest));
    } else if (!strcmp(cmd, "sendfile") && rest) {
        usb_sendfile(rest);
    } else if (!strcmp(cmd, "bulk") && rest) {
        int n = atoi(rest), i;
        static uint8_t pat[USB_MSG_CAP];
        if (n < 1 || n > usb_msg_max()) {
            printf("bulk: 1..%d\n", usb_msg_max());
        } else {
            for (i = 0; i < n; i++)
                pat[i] = (uint8_t)i;
            usb_submit(pat, n, QOS_BULK);
            printf("%s [%s] >> queued %d-byte pattern (bulk)\n", tstamp(),
                   g_name, n);
        }
    } else if (!strcmp(cmd, "bcastfile") && rest) {
        FILE *f;
        long sz;
        int rung = 0xFF;
        if (rest[0] == '-' && rest[1] == 'r' && rest[2] == ' ') {
            char *e;
            long v = strtol(rest + 3, &e, 10);
            if (e != rest + 3 && v >= 0 && v <= 12) {
                rung = (int)v;
                while (*e == ' ')
                    e++;
                rest = e;
            }
        }
        if (g_ubcf_f) {
            printf("bcastfile: a broadcast is already streaming to the "
                   "board\n");
        } else if (!(f = fopen(rest, "rb"))) {
            printf("bcastfile: cannot open %s\n", rest);
        } else {
            fseek(f, 0, SEEK_END);
            sz = ftell(f);
            fseek(f, 0, SEEK_SET);
            if (sz <= 0) {
                printf("bcastfile: %s is empty\n", rest);
                fclose(f);
            } else {
                if (g_ust_valid && g_ust.peer_state >= 2
                    && !(g_ust.peer_caps & 16))
                    printf("note: the known peer did not declare "
                           "broadcast reception -- other listeners may "
                           "still hear it\n");
                g_ubcf_f = f;         /* read sequentially as the board
                                       * drains; no size cap */
                g_ubcf_len = sz;
                g_ubcf_off = 0;
                g_ubcf_rung = rung;
                printf("%s [%s] broadcasting %s (%ld bytes, raw, no "
                       "delivery guarantee)\n", tstamp(), g_name, rest,
                       sz);
                usb_pump_bcfile();
            }
        }
    } else if (!strcmp(cmd, "bcast") && rest) {
        uint8_t body[2 + 1022];
        int rung = 0xFF, n;
        /* `-r N` pins the rung. Without it the board uses the rung it
         * would send the peer a frame at -- the one rung the peer is
         * certainly listening on -- and says which that was in a log
         * line. A beacon for stations never heard from wants `-r 0`. */
        if (rest[0] == '-' && rest[1] == 'r' && rest[2] == ' ') {
            char *e;
            long v = strtol(rest + 3, &e, 10);
            if (e != rest + 3 && v >= 0 && v <= 12) {
                rung = (int)v;
                while (*e == ' ')
                    e++;
                rest = e;
            }
        }
        n = (int)strlen(rest);
        if (n > 1022) {
            printf("bcast: %d bytes exceeds the %d-byte broadcast cap\n",
                   n, 1022);
        } else if (n == 0) {
            printf("bcast: nothing to send\n");
        } else {
            body[0] = BC_PT_TELEMETRY;
            body[1] = (uint8_t)rung;
            memcpy(body + 2, rest, (size_t)n);
            usb_send_frame(UP_CMD_BCAST, body, n + 2);
            char rb[24];
            if (rung == 0xFF)
                snprintf(rb, sizeof(rb), "the link's own rung");
            else
                snprintf(rb, sizeof(rb), "rung %d", rung);
            printf("%s [%s] >> broadcast queued, %d bytes at %s (non-ARQ: "
                   "nothing will be repeated)\n", tstamp(), g_name, n, rb);
        }
    } else if (!strcmp(cmd, "status")) {
        if (!g_ust_valid) {
            printf("no status yet -- the board pushes one every 0.5 s\n");
        } else {
            printf("%s [%s] rung %d  SNR %+.1f dB  %s%s\n", tstamp(),
                   g_name, g_ust.rung, g_ust.snr_q8 / 256.0,
                   g_ust.busy ? "BUSY" : "idle",
                   g_ust.pending ? "  pending-ack" : "");
            printf("  queues: ctl %u  inter %u  bulk %u\n", g_ust.q_ctl,
                   g_ust.q_inter, g_ust.q_bulk);
            if (g_ubcf_f)
                printf("  broadcastfile: %ld of %ld bytes fed to the "
                       "board (board buffer %u B free)\n", g_ubcf_off,
                       g_ubcf_len, g_ust.bc_free);
            if (g_ust.peer_state >= 2) {
                char mr[16];
                if (g_ust.peer_max_rung1)
                    snprintf(mr, sizeof(mr), "%d", g_ust.peer_max_rung1 - 1);
                else
                    snprintf(mr, sizeof(mr), "unspecified");
                printf("  peer: %s%s%s%s%smessages up to %u B, window %u, "
                       "rung ceiling %s%s\n",
                       (g_ust.peer_caps & 1) ? "stream " : "",
                       (g_ust.peer_caps & 2) ? "ext " : "",
                       (g_ust.peer_caps & 4) ? "ldpc " : "",
                       (g_ust.peer_caps & 8) ? "burst " : "",
                       (g_ust.peer_caps & 16) ? "bcast " : "",
                       g_ust.peer_msg_max, g_ust.peer_win_max, mr,
                       g_ust.peer_state == 3 ? " (handshake complete)"
                                             : " (awaiting our confirmation)");
            }
            if (g_ust.peer_state < 2)
                printf("  peer: %s\n", g_ust.peer_state == 1
                       ? "did not answer the capability probe -- legacy "
                         "defaults"
                       : "capabilities unknown (asked on the first "
                         "bulk transfer)");
        }
    } else if (!strcmp(cmd, "stats")) {
        if (g_ust_valid)
            printf("tx %u  rx %u  timeouts %u  retx %u\n", g_ust.tx_frames,
                   g_ust.rx_frames, g_ust.timeouts,
                   g_ust.retransmissions);
        if (g_upart_n)
            printf("sending '%s': %d/%d parts handed over\n", g_upfile,
                   g_upart_sent, g_upart_n);
        printf("usb resyncs %u (0 on a healthy link)\n", g_up.resyncs);
    } else if (!strcmp(cmd, "config")) {
        char *key = rest ? strtok(rest, " \t") : 0;
        char *val = key ? strtok(0, " \t") : 0;
        size_t i;
        int found = -1;
        for (i = 0; key && val
                    && i < sizeof(USB_CFG) / sizeof(USB_CFG[0]); i++)
            if (!strcmp(key, USB_CFG[i].name))
                found = USB_CFG[i].key;
        if (!key) {
            /* no arguments: ask the board -- the settings live there,
             * and a cache here would lie after a reattach */
            usb_send_frame(UP_CMD_CONFIG, 0, 0);
        } else if (found < 0) {
            printf("config keys: rung_ceiling burst_window burst_stream "
                   "freq_trim_mhz audio_tap anchor diag_stream win_max\n");
        } else {
            uint8_t body[5];
            int32_t v = atoi(val);
            body[0] = (uint8_t)found;
            memcpy(body + 1, &v, 4);
            usb_send_frame(UP_CMD_CONFIG, body, 5);
            printf("%s [%s] >> config %s = %d\n", tstamp(), g_name, key, v);
        }
    } else if (!strcmp(cmd, "debug")) {
        /* `debug [on|off]`. The events come from the BOARD, whose diag
         * stream is off by default (it would drown the command replies
         * otherwise -- see usb_modem.c), so turning prints on without
         * turning the stream on showed nothing and said so in a hint
         * nobody read. One command does both now. */
        uint8_t body[5];
        int32_t v;
        if (rest && !strcmp(rest, "on"))
            g_debug = 1;
        else if (rest && !strcmp(rest, "off"))
            g_debug = 0;
        else
            g_debug = !g_debug;
        v = g_debug;
        body[0] = 7;                            /* UP_CFG_DIAG_STREAM */
        memcpy(body + 1, &v, 4);
        usb_send_frame(UP_CMD_CONFIG, body, 5);
        printf("diag %s: the board's event stream is %s and prints are %s\n",
               g_debug ? "ON" : "OFF", g_debug ? "on" : "off",
               g_debug ? "on" : "off");
    } else if (!strcmp(cmd, "help")) {
        printf("  send <text> | sendfile <path> | bulk <n> | "
               "bcast [-r <rung>] <text> | bcastfile [-r <rung>] "
               "<path>\n  status | stats | "
               "config [<key> <val>] | "
               "debug [on|off] | quit\n");
    } else {
        printf("unknown command '%s' -- try help\n", cmd);
    }
    return 1;
}

static int usb_console(const char *serial)
{
    uint8_t buf[512];
    char inbuf[512];
    int inlen = 0;

    g_usb = usbh_open(serial);
    if (!g_usb)
        return 1;
    up_parser_init(&g_up);
    if (usbh_stale(g_usb))
        printf("[%s] drained %d stale bytes from a previous session\n",
               g_name, usbh_stale(g_usb));
    printf("[%s] attached to board %s -- the station runs ON the board; "
           "this is a terminal onto it. 'help' for commands.\n", g_name,
           usbh_serial(g_usb));
    usb_send_frame(UP_CMD_INFO, 0, 0);
    printf("> ");
    fflush(stdout);

    time_t last_ping = 0;
    for (;;) {
        fd_set rf;
        struct timeval tv = { 0, 0 };
        time_t now = time(0);
        int n;
        /* one ping a second keeps the board's "host attached" state
         * (and its LED) alive: closing this program does not unmount
         * the device, so the board would otherwise never notice */
        if (now != last_ping) {
            uint32_t tok = (uint32_t)now;
            usb_send_frame(UP_CMD_PING, &tok, 4);
            last_ping = now;
        }
        n = usbh_read(g_usb, buf, (int)sizeof(buf), 50);
        if (n < 0) {
            printf("\n[%s] usb read failed -- board unplugged?\n", g_name);
            break;
        }
        if (n > 0)
            up_parser_push(&g_up, buf, n, usb_on_frame, 0);

        FD_ZERO(&rf);
        FD_SET(0, &rf);
        if (select(1, &rf, 0, 0, &tv) > 0) {
            ssize_t got = read(0, inbuf + inlen,
                               sizeof(inbuf) - (size_t)inlen - 1);
            char *nl;
            if (got <= 0)
                break;
            inlen += (int)got;
            inbuf[inlen] = 0;
            while ((nl = strchr(inbuf, '\n')) != 0) {
                *nl = 0;
                if (!usb_command(inbuf)) {
                    usbh_close(g_usb);
                    return 0;
                }
                printf("> ");
                fflush(stdout);
                memmove(inbuf, nl + 1, strlen(nl + 1) + 1);
                inlen = (int)strlen(inbuf);
            }
        }
    }
    usbh_close(g_usb);
    return 0;
}

/* Queue a broadcast: one preamble+header per group of BC frames, framing
 * as ofdm_phy/broadcast.py (SYNC|EOS|seq, then this frame's length). */
/* The grouping is the point: everything under "delivered" is tracked by
 * the ARQ layer and retransmitted until acknowledged, while broadcast
 * repeats nothing at all. `stream` is easy to misread as the second kind
 * -- it is not, it only changes how a burst is framed on air -- so it
 * sits with the tuning knobs, not with the transfer commands. */
static void show_help(void)
{
    printf(
"\n  transfers -- DELIVERED (tracked, retransmitted until acknowledged)\n"
"    send <text>        interactive message, jumps ahead of bulk traffic\n"
"    sendfile <path>    file transfer over burst ARQ (deflated first)\n"
"    bulk <n>           queue an n-byte test pattern\n"
"\n  transfers -- NOT DELIVERED (non-ARQ: nothing is ever repeated)\n"
"    broadcast <text>       speech/telemetry style; the receiver reports\n"
"                           what it heard and losses stay lost\n"
"    broadcastfile <path>   the same, file-sourced and binary-safe; the\n"
"                           receiver stores it as rx_broadcast.bin.\n"
"                           A broadcast cannot negotiate its rate, so if\n"
"                           the link is still at the bootstrap rung the\n"
"                           payload is HELD and the station probes the\n"
"                           peer until the ladder can carry it.\n"
"\n  tuning\n"
"    stream on|off      put a whole burst window behind ONE preamble\n"
"                       instead of one per fragment (default on). This\n"
"                       does NOT affect delivery -- every fragment is\n"
"                       still acknowledged and retransmitted; it only\n"
"                       changes framing, and falls back automatically if\n"
"                       the peer cannot follow a stream.\n"
"    window <n>         fragments per acknowledgment (ceiling; the\n"
"                       station picks below it from what lands)\n"
"    compress on|off    deflate files before transmitting (default on)\n"
"    tune <hz>          manual LO trim, through the trim budget\n"
"\n  diagnostics\n"
"    debug on|off       live event stream: every TX/RX, timeout, rung\n"
"                       change with the inputs that caused it, and burst\n"
"                       state transitions\n"
"    status             rung, SNR, CFO, queues, carrier sense, controller\n"
"    stats              frame counters\n"
"    help | quit\n\n");
}

/* ---- broadcast transmit ----
 *
 * A broadcast is not one keying. A payload of any size is cut into
 * successive transmissions, each a whole number of groups that fits the
 * air buffer, with the sequence number running CONTINUOUSLY across them
 * so the receiver's loss counting still works and only the final frame
 * carries EOS. Truncating instead (the first version did) silently
 * dropped 46978 of 47001 bytes and then reported success.
 */
#define BC_MAX_TX_SAMPLES 360000   /* 30 s: one polite turn on the channel */
/* Frames behind one preamble in a broadcast.
 *
 * A 400-byte broadcast once arrived as "5 frames, 12 lost" at 16 and
 * complete at 4, which looked like a group-size limit. It was not: the
 * DSP is innocent at every group size. Capturing the audio the live
 * receiver actually saw and replaying it offline showed two unrelated
 * application bugs, both now fixed --
 *   1. g_bc_left was armed from g_bc_rx_group BEFORE the SYNC descriptor
 *      that sets it was parsed, so the first group of a broadcast walked
 *      the PREVIOUS stream's block count (4), and 4 + 1 re-acquired
 *      frame is exactly the "5 frames" that was reported;
 *   2. EOS did not stop the walk, so the short FINAL group -- whose
 *      descriptor can only advertise the nominal size -- was chased to
 *      the full 16, giving 15 phantom CRC failures.
 * For the record, what was ruled out by measurement: the C transmitter
 * is bit-identical to the Python twin at 16 blocks; the C streaming
 * receiver decodes a 16-block burst 16/16 offline; the shared sample
 * ring never overran (124478 of 147456); and a per-block LLR-temperature
 * refit is worth nothing even with oracle knowledge of the answer
 * (experiments/burst_alpha_ab.py).
 *
 * 8 is the ceiling BC_MAX_GROUP imposes (bc_receive holds a whole group
 * in a static buffer), and the compile-time assertion below keeps the
 * two from drifting apart. It is not a performance ceiling: 16 was
 * measured to deliver a 14162-byte file complete once the two bugs
 * above were fixed.
 *
 * The trade-off the size sets is a non-ARQ one. A larger group amortises
 * the preamble over more frames, but nothing is retransmitted, so a
 * missed acquisition costs the whole group rather than one frame --
 * bigger groups are cheaper per frame and more expensive per failure. */
#ifndef BC_GROUP_CAP
#define BC_GROUP_CAP 8
#endif
/* The group size travels on the wire (log2 in the SYNC descriptor), so a
 * receiver using bc_receive() must be able to hold a whole group -- that
 * walker rejects group > BC_MAX_GROUP outright. Fail the build rather
 * than emit broadcasts the C receive API cannot accept. */
typedef char bc_group_cap_fits[(BC_GROUP_CAP <= BC_MAX_GROUP
                                && BC_GROUP_CAP >= 1) ? 1 : -1];

static uint8_t g_bc_src[65536];
static int g_bc_src_len, g_bc_src_off, g_bc_seq, g_bc_rung, g_bc_ptype_tx;
/* A broadcast cannot negotiate: it carries no acknowledgment, so the
 * rung is whatever the link last established, and at the bootstrap rung
 * that is EXTREME -- tens of seconds per frame. Rather than telling the
 * operator to go and make the link by hand, hold the payload and drive
 * the ladder here: a control-class message is exactly the "say something
 * back to me" probe the rate controller needs, so no new message type is
 * required. */
static int g_bc_waiting;          /* payload held until the link is up */
static double g_bc_deadline, g_bc_next_probe;
#define BC_LINK_TIMEOUT_S 180.0
#define BC_PROBE_EVERY_S 25.0

/* Bytes a single transmission can carry at this rung -- used to tell the
 * operator up front what will happen, instead of discovering it after a
 * 42-second transmission of 23 bytes. */
static int bc_bytes_per_tx(int rung, int group)
{
    int pkt_bits = 36 + 8 * 26;
    int one = tx_burst_len(ladder_mode(rung), pkt_bits, ladder_mod(rung),
                           ladder_spd(rung), group, BURST_STREAM_RESYNC);
    int per_group = group * (26 - 2) - 1;   /* SYNC frame spends one more */
    int groups;
    if (one <= 0)
        return 0;
    groups = BC_MAX_TX_SAMPLES / one;
    return groups > 0 ? groups * per_group : 0;
}

/* Build the next transmission into g_bc_air. Returns samples, or 0 when
 * the payload is exhausted. */
static int bc_build_next(void)
{
    static uint8_t blocks[8 * 2600], packed[8 * 2600];
    uint8_t payload[26];
    int cap0 = 26 - 3, cap = 26 - 2;

    g_bc_pending = 0;
    while (g_bc_src_off < g_bc_src_len) {
        int nf = 0, pkt_n = 0, first = 1, n, i;
        while (nf < g_bc_group && g_bc_src_off < g_bc_src_len) {
            int take = first ? cap0 : cap;
            int flags = first ? 0x80 : 0;
            if (take > g_bc_src_len - g_bc_src_off)
                take = g_bc_src_len - g_bc_src_off;
            if (g_bc_src_off + take >= g_bc_src_len)
                flags |= 0x40;               /* EOS: the very last frame */
            memset(payload, 0, sizeof(payload));
            payload[0] = (uint8_t)(flags | (g_bc_seq & 0x3F));
            payload[1] = (uint8_t)take;
            if (first) {
                /* log2(group) << 4 | payload type -- the receiver cannot
                 * guess the group size, and guessing wrong costs it every
                 * frame after the first of each group */
                int gc = 0, g = g_bc_group;
                while (g > 1) { g >>= 1; gc++; }
                payload[2] = (uint8_t)((gc << 4) | (g_bc_ptype_tx & 0x0F));
            }
            memcpy(payload + (first ? 3 : 2), g_bc_src + g_bc_src_off,
                   (size_t)take);
            pkt_n = data_encode(0, payload, 26, blocks + (size_t)nf * 2600);
            g_bc_src_off += take;
            g_bc_seq++;
            nf++;
            first = 0;
        }
        for (i = 0; i < nf; i++)
            memcpy(packed + (size_t)i * pkt_n, blocks + (size_t)i * 2600,
                   (size_t)pkt_n);
        n = build_streamed(g_bc_rung, packed, pkt_n, nf, PKT_TYP_BCAST,
                           BURST_STREAM_RESYNC, g_bc_air + g_bc_pending,
                           (int)(sizeof(g_bc_air) / sizeof(g_bc_air[0]))
                               - g_bc_pending);
        if (n <= 0) {
            printf("%s [%s] broadcast: PHY refused a group at rung %d\n",
                   tstamp(), g_name, g_bc_rung);
            g_bc_src_off = g_bc_src_len;
            break;
        }
        g_bc_pending += n;
        if (g_bc_pending + n > BC_MAX_TX_SAMPLES)
            break;                            /* this turn is full */
    }
    return g_bc_pending;
}

/* Begin transmitting a held payload at the rung the link now has.
 * Returns 0 if the rung still will not carry it. */
static int bc_start(int rung)
{
    int per_tx, n_tx;

    g_bc_group = BC_GROUP_CAP;
    while (g_bc_group > 1
           && tx_burst_len(ladder_mode(rung), 36 + 8 * 26, ladder_mod(rung),
                           ladder_spd(rung), g_bc_group,
                           BURST_STREAM_RESYNC) > BC_MAX_TX_SAMPLES)
        g_bc_group /= 2;
    per_tx = bc_bytes_per_tx(rung, g_bc_group);
    if (per_tx <= 0)
        return 0;

    g_bc_src_off = 0;
    g_bc_seq = 0;
    g_bc_rung = rung;
    g_bc_pending = g_bc_sent = 0;
    g_bc_waiting = 0;
    n_tx = (g_bc_src_len + per_tx - 1) / per_tx;
    printf("\n%s [%s] broadcast starting: %d bytes at rung %d, %d frame(s) "
           "per group, ~%d transmission(s), ~%.0f s of air in total "
           "(non-ARQ, nothing will be repeated)\n> ", tstamp(), g_name,
           g_bc_src_len, rung, g_bc_group, n_tx,
           (double)n_tx * BC_MAX_TX_SAMPLES / FS);
    fflush(stdout);
    return 1;
}

/* Called from the main loop while a payload is held: probe the peer with
 * control-class traffic until the rate ladder is high enough to carry
 * the broadcast, then start it. */
static void bc_poll_link(double now)
{
    static const uint8_t probe[] = "LINK";
    int rung = g_st.stats.last_rung;

    if (!g_bc_waiting)
        return;
    if (rung >= BURST_MIN_RUNG && bc_start(rung))
        return;
    if (now > g_bc_deadline) {
        printf("\n%s [%s] broadcast: gave up after %.0f s -- the link is "
               "still at rung %d, which cannot carry a broadcast\n> ",
               tstamp(), g_name, BC_LINK_TIMEOUT_S, rung);
        fflush(stdout);
        g_bc_waiting = 0;
        g_bc_src_len = 0;
        return;
    }
    if (now >= g_bc_next_probe) {
        g_bc_next_probe = now + BC_PROBE_EVERY_S;
        /* control class: highest priority, and a frame the peer answers,
         * which is what moves the ladder */
        if (station_submit(&g_st, probe, (int)sizeof(probe) - 1,
                           QOS_CONTROL) == 0) {
            printf("\n%s [%s] broadcast: link at rung %d, probing to bring "
                   "it up...\n> ", tstamp(), g_name, rung);
            fflush(stdout);
        }
    }
}

static void cmd_broadcast_bytes(const uint8_t *data, int len, int rung,
                                int ptype)
{
    if (len <= 0)
        return;
    if (len > (int)sizeof(g_bc_src)) {
        printf("%s [%s] broadcast: %d bytes exceeds the %zu-byte cap\n",
               tstamp(), g_name, len, sizeof(g_bc_src));
        return;
    }
    memcpy(g_bc_src, data, (size_t)len);
    g_bc_src_len = len;
    g_bc_ptype_tx = ptype;
    g_bc_src_off = g_bc_seq = 0;
    g_bc_pending = g_bc_sent = 0;

    if (rung >= BURST_MIN_RUNG && bc_start(rung))
        return;
    /* Hold it and bring the link up ourselves rather than handing the
     * operator a chore. */
    g_bc_waiting = 1;
    g_bc_deadline = now_t() + BC_LINK_TIMEOUT_S;
    g_bc_next_probe = 0.0;
    printf("%s [%s] broadcast held: %d bytes waiting for the link (rung %d "
           "cannot carry it); probing the peer, giving up after %.0f s\n",
           tstamp(), g_name, len, rung, BC_LINK_TIMEOUT_S);
}

static void cmd_broadcast(const char *text, int rung)
{
    cmd_broadcast_bytes((const uint8_t *)text, (int)strlen(text), rung,
                        BC_PT_TELEMETRY);
}

/* Broadcast a file: same non-ARQ path, binary-safe. Nothing here is
 * acknowledged or repeated, so what the receiver misses is gone -- it
 * is the right carrier for telemetry or coded speech, and the wrong one
 * for anything that has to arrive intact. Use sendfile for that. */
static void cmd_broadcastfile(const char *path, int rung)
{
    static uint8_t buf[65536];
    FILE *f = fopen(path, "rb");
    long sz;
    size_t got;

    if (!f) {
        printf("%s [%s] broadcastfile: cannot open %s\n", tstamp(), g_name,
               path);
        return;
    }
    fseek(f, 0, SEEK_END);
    sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > (long)sizeof(buf)) {
        printf("%s [%s] broadcastfile: %s is %ld bytes, cap is %zu\n",
               tstamp(), g_name, path, sz, sizeof(buf));
        fclose(f);
        return;
    }
    got = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    if (got != (size_t)sz) {
        printf("%s [%s] broadcastfile: read error\n", tstamp(), g_name);
        return;
    }
    printf("%s [%s] broadcasting %s (%ld bytes, no delivery guarantee)\n",
           tstamp(), g_name, path, sz);
    cmd_broadcast_bytes(buf, (int)sz, rung, BC_PT_OPAQUE);
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

    head = 1 + (int)strlen(FILE_TAG) + (int)strlen(base) + 1 + 4;
    n_parts = body_len <= 0 ? 1
                            : (body_len + FILE_PART_DATA - 1) / FILE_PART_DATA;
    magic = magic == FILE_MAGIC_Z ? FILE_MAGIC_WZ : FILE_MAGIC_W;
    /* Negotiate before committing. ctl_tx_rung() decays the rung by one
     * step per STALE_S of peer silence, so a transfer started after a
     * quiet spell opens at a rung the link has not actually been tested
     * at -- measured, a 37.0 s frame at rung 4 where the very next
     * exchange settled at rung 12 and 5.1 s. One control frame costs
     * under a second and is answered, which is what moves the ladder, so
     * probe first and let the bulk queue behind it. */
    {
        /* peer_req_time starts at the -1e9 sentinel, so "never heard from"
         * must be TESTED for, not subtracted from -- doing the arithmetic
         * anyway printed "peer last reported 1000000058 s ago". Same rule
         * the link layer already follows for last_rx_seq. */
        int never = g_st.ctl.peer_req_time < -1e8;
        double age = never ? 0.0 : now_t() - g_st.ctl.peer_req_time;

        if (never || age > BULK_PROBE_STALE_S) {
            static const uint8_t probe[] = "LINK";
            if (station_submit(&g_st, probe, (int)sizeof(probe) - 1,
                               QOS_CONTROL) == 0) {
                if (never)
                    printf("%s [%s] sendfile: peer has not reported yet"
                           " -- probing before committing the transfer\n",
                           tstamp(), g_name);
                else
                    printf("%s [%s] sendfile: peer last reported %.0f s"
                           " ago -- probing before committing the"
                           " transfer\n", tstamp(), g_name, age);
                fflush(stdout);
            }
        }
    }

    q_free = ST_MAX_MSGS - g_st.qcount[QOS_BULK];
    /* the message store is a capacity too: check it here so a file is
     * refused whole rather than half submitted (see ST_POOL_SLOTS) */
    if (station_pool_free(&g_st) < q_free)
        q_free = station_pool_free(&g_st);
    if (n_parts > 65535 || n_parts > q_free) {
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
        env[head - 4] = (uint8_t)p;
        env[head - 3] = (uint8_t)(p >> 8);
        env[head - 2] = (uint8_t)n_parts;
        env[head - 1] = (uint8_t)(n_parts >> 8);
        memcpy(env + head, body + (size_t)p * FILE_PART_DATA, (size_t)dlen);
        if (station_submit(&g_st, env, head + dlen, QOS_BULK) != 0) {
            printf("%s [%s] sendfile: queue full at part %d\n", tstamp(),
                   g_name, p + 1);
            return;
        }
    }
    if (magic == FILE_MAGIC_WZ)
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
    } else if (!strncmp(line, "broadcastfile ", 14)) {
        cmd_broadcastfile(line + 14, g_st.stats.last_rung >= 0
                                         ? g_st.stats.last_rung : 7);
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
    } else if (!strcmp(line, "help") || !strcmp(line, "?")) {
        show_help();
    } else if (line[0]) {
        printf("%s [%s] unknown command '%s' -- type 'help'\n", tstamp(),
               g_name, line);
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
        fprintf(stderr,
                "usage: %s <device.sock> [name]     virtual channel mode\n"
                "       %s --usb [serial] [name]    USB modem mode\n"
                "       %s --list                   enumerate USB modems\n",
                argv[0], argv[0], argv[0]);
        return 1;
    }
    if (!strcmp(argv[1], "--list"))
        return usbh_list() > 0 ? 0 : 1;
    if (!strcmp(argv[1], "--usb")) {
        const char *serial = 0;
        int ai = 2;
        /* a 24-hex-char token is a serial; anything else is the name */
        if (argc > ai && strlen(argv[ai]) == 24
            && strspn(argv[ai], "0123456789abcdefABCDEF") == 24)
            serial = argv[ai++];
        if (argc > ai)
            g_name = argv[ai];
        else if (serial)
            /* FIRST 4 of the serial, not the last: the UID's tail is
             * the wafer/lot ID, identical for chips from one wafer --
             * both stand boards end in ...3436 and printed the same
             * name. The head is the die coordinate, which differs. */
            g_name = strndup(serial, 4);
        return usb_console(serial);
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
    /* rxs_open refuses a mode whose tone window does not fit the summary
     * slice sized for it -- a build-configuration error, not a runtime
     * condition, but rxs_push would dereference the NULL. */
    if (!g_rxs[0] || !g_rxs[1] || !g_rxs[2]) {
        fprintf(stderr, "[%s] rxs_open failed: summary slice too small "
                        "for a mode's tone window\n", g_name);
        return 1;
    }
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
#ifdef RXS_TRACE
            {   /* capture exactly what the live receiver saw, so the same
                 * audio can be replayed through the offline decoder */
                static FILE *cap;
                if (!cap) cap = fopen("rx_raw.bin", "wb");
                if (cap) { fwrite(chunk, 2, (size_t)n, cap); fflush(cap); }
            }
#endif
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
                    if (ev.type == -3 && ev.hdr.typ == PKT_TYP_BCAST
                        && g_bc_left[m] > 0) {
                        /* A block that failed CRC must NOT end the group.
                         * The offsets are deterministic, so step over it
                         * and keep going -- otherwise one bad block costs
                         * every frame behind it, which is how a 17-frame
                         * broadcast arrived as 5. */
                        g_bc_left[m]--;
                        g_bc_lost++;
                        g_bc_rx_last = now_t();
                        /* The EOS marker rides in a frame, so a lost EOS
                         * leaves nothing to stop on. Bound the chase the
                         * same way the ARQ burst path does. */
                        if (++g_bc_miss[m] >= BC_MAX_MISS)
                            g_bc_left[m] = 0;
                        if (g_bc_left[m] > 0
                            && !rxs_continue_burst(g_rxs[m],
                                                   BURST_STREAM_RESYNC))
                            g_bc_left[m] = 0;
                    } else if (ev.type == 1 && ev.hdr.typ == PKT_TYP_BCAST) {
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
                            /* descriptor: log2(group) << 4 | ptype.
                             * Parse it BEFORE arming g_bc_left: the group
                             * size is a property of THIS stream, so arming
                             * from the previous stream's value makes the
                             * first group of every broadcast walk the
                             * wrong number of blocks -- and the first
                             * group is the only one a listener that just
                             * tuned in actually gets. */
                            int t = 0, q;
                            head = 3;
                            for (q = 0; q < 8; q++)
                                t = (t << 1) | (b[20 + 16 + q] & 1);
                            g_bc_ptype = t & 0x0F;
                            g_bc_rx_group = 1 << (t >> 4);
                            if (g_bc_rx_group < 1
                                || g_bc_rx_group > BURST_STREAM_MAX)
                                g_bc_rx_group = 4;
                            g_bc_left[m] = g_bc_rx_group - 1;
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
                        g_bc_rx_last = now_t();
                        if (dlen > plen - head)
                            dlen = plen - head;
                        {
                            uint8_t tmp[256];
                            int nb = 0;
                            for (j = 0; j < dlen
                                 && nb < (int)sizeof(tmp); j++) {
                                int bb, val = 0;
                                for (bb = 0; bb < 8; bb++)
                                    val = (val << 1)
                                          | (b[20 + 8 * (head + j) + bb] & 1);
                                tmp[nb++] = (uint8_t)val;
                            }
                            bc_sink_open();
                            if (g_bc_file) {
                                fwrite(tmp, 1, (size_t)nb, g_bc_file);
                                g_bc_written += nb;
                            } else {
                                int room = (int)sizeof(g_bc_asm) - g_bc_len;
                                int c = nb < room ? nb : room;
                                memcpy(g_bc_asm + g_bc_len, tmp, (size_t)c);
                                g_bc_len += c;
                                if (c < nb)
                                    g_bc_trunc = 1;
                            }
                        }
                        g_bc_miss[m] = 0;
                        if (flags & 0x40)
                            /* EOS: the stream ends HERE, whatever the
                             * group descriptor promised. The final group
                             * of a broadcast is usually SHORT, but the
                             * descriptor can only advertise the nominal
                             * size (it carries log2(group)), so without
                             * this the receiver walks blocks that were
                             * never transmitted: measured 15 phantom CRC
                             * failures on a 1-frame final group, each
                             * costing a block-time of deafness. */
                            g_bc_left[m] = 0;
                        if (g_bc_left[m] > 0
                            && !rxs_continue_burst(g_rxs[m],
                                                   BURST_STREAM_RESYNC))
                            g_bc_left[m] = 0;
                        if (flags & 0x40) {
                            long total = g_bc_written + g_bc_len;
                            printf("\n%s [%s] << broadcast: %ld bytes, "
                                   "%d frames, %d lost, snr %+.1f dB\n",
                                   tstamp(), g_name, total, g_bc_frames,
                                   g_bc_lost, ev.snr_db);
                            if (g_bc_file) {
                                /* binary: printing it would be nonsense,
                                 * and it may well have holes in it --
                                 * nothing was retransmitted */
                                fclose(g_bc_file);
                                g_bc_file = NULL;
                                printf("    stored %ld bytes as %s"
                                       " (%d of the frames sent arrived;"
                                       " gaps are NOT repaired)\n> ",
                                       g_bc_written, BC_RX_PATH,
                                       g_bc_frames);
                            } else if (g_bc_ptype == BC_PT_OPAQUE) {
                                printf("    NOT stored: %s could not be"
                                       " opened (see the error above)\n> ",
                                       BC_RX_PATH);
                            } else {
                                printf("    \"%.*s\"%s\n> ", g_bc_len,
                                       g_bc_asm,
                                       g_bc_trunc ? "  [truncated: text"
                                                    " buffer full]" : "");
                            }
                            fflush(stdout);
                            g_bc_len = g_bc_frames = g_bc_lost = 0;
                            g_bc_written = 0;
                            g_bc_trunc = 0;
                            g_bc_last_seq = -1;
                            g_bc_ptype = -1;
                            g_bc_rx_last = -1e9;
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
                if (now_t() - g_bc_rx_last < BC_RX_HOLD_S)
                    busy = 1;   /* hearing someone else's broadcast */
                bc_poll_link(now_t());
                if (!g_bc_waiting && g_bc_src_off < g_bc_src_len
                    && !busy && !g_txing) {
                    /* Successive turns: build the next transmission only
                     * when the channel is free, so a long payload yields
                     * to carrier sense between turns instead of hogging
                     * the frequency in one enormous keying. */
                    nf = bc_build_next();
                    if (nf > 0) {
                        memcpy(g_frame, g_bc_air,
                               (size_t)nf * sizeof(int16_t));
                        g_bc_pending = 0;
                        g_bc_sent = 1;
                        printf("%s [%s] >> broadcast %d/%d bytes, %.1f s "
                               "air\n> ", tstamp(), g_name, g_bc_src_off,
                               g_bc_src_len, (double)nf / FS);
                        fflush(stdout);
                    }
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
