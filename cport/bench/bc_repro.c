/* Host reproduction of the FIRMWARE's broadcast path.
 *
 * The firmware builds a broadcast group itself (usb/usb_radio_main.c
 * bc_open_group) and walks it with bc_advance() through the streaming
 * receiver -- neither of which src/broadcast.c's frame-at-once
 * bc_receive() exercises. This bench is that pair, lifted verbatim, so a
 * board failure can be reproduced without boards.
 *
 *   -r N   rung (default 4, the BURST_MIN_RUNG floor bc_cmd clamps to)
 *   -q     the wire: 12-bit DAC at 3/4 scale re-read by a 16-bit ADC
 *   -len N source payload bytes (default 51)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "packets.h"
#include "link.h"
#include "station.h"
#include "broadcast.h"
#include "tx.h"
#include "rx_stream.h"

#define BC_GROUP 4
#define BC_FRAME 26
#define BC_MAX_MISS 4

static int RUNG = 4, SRC_LEN = 51, QUANT = 0, ALL_MODES = 0;

static uint8_t g_bc_src[1022];
static int g_bc_src_len, g_bc_src_off, g_bc_seq, g_bc_ptype_tx;
static uint8_t g_bc_blocks[BC_GROUP * (36 + 8 * BC_FRAME)];
/* EXTREME groups are ~25 s each: two of them plus gaps need the room */
static int16_t g_wave[600000], g_air[1400000];
static txs_t *g_txs;
static int g_tx_total;

/* --- transmitter: bc_open_group(), with phy_build_stream inlined --- */
#define BC_GROUP_MAX_AIR_S 30.0

static int bc_group_frames(void)
{
    int g = BC_GROUP;
    while (g > 1
           && stream_air_time_pub(RUNG, BC_FRAME, g) > BC_GROUP_MAX_AIR_S)
        g >>= 1;
    return g;
}

static int bc_open_group(void)
{
    uint8_t *blocks = g_bc_blocks;
    uint8_t payload[BC_FRAME];
    int pkt_n = 0, nf = 0, first = 1, cap0 = BC_FRAME - 3, cap = BC_FRAME - 2,
        grp = bc_group_frames();

    if (g_bc_src_off >= g_bc_src_len)
        return 0;
    while (nf < grp && g_bc_src_off < g_bc_src_len) {
        int take = first ? cap0 : cap;
        int flags = first ? BC_SYNC : 0;
        if (take > g_bc_src_len - g_bc_src_off)
            take = g_bc_src_len - g_bc_src_off;
        if (g_bc_src_off + take >= g_bc_src_len)
            flags |= BC_EOS;
        memset(payload, 0, sizeof(payload));
        payload[0] = (uint8_t)(flags | (g_bc_seq & BC_SEQ_MASK));
        payload[1] = (uint8_t)take;
        if (first) {
            int gc = 0, g = grp;
            while (g > 1) { g >>= 1; gc++; }
            payload[2] = (uint8_t)((gc << 4) | g_bc_ptype_tx);
        }
        memcpy(payload + (first ? 3 : 2), g_bc_src + g_bc_src_off,
               (size_t)take);
        pkt_n = data_encode(0, payload, BC_FRAME,
                            blocks + (size_t)nf * (36 + 8 * BC_FRAME));
        g_bc_src_off += take;
        g_bc_seq++;
        nf++;
        first = 0;
    }
    {
        int i;
        for (i = 1; i < nf; i++)
            memmove(blocks + (size_t)i * pkt_n,
                    blocks + (size_t)i * (36 + 8 * BC_FRAME), (size_t)pkt_n);
    }
    g_tx_total = 0;
    g_txs = txs_open(ladder_mode(RUNG), blocks, pkt_n, nf, PKT_TYP_BCAST,
                     ladder_mod(RUNG), ladder_spd(RUNG), BURST_STREAM_RESYNC,
                     0, &g_tx_total);
    printf("  group: %d frames, pkt_n %d, %d samples (rung %d, mode %d)\n",
           nf, pkt_n, g_tx_total, RUNG, (int)ladder_mode(RUNG));
    return g_txs ? g_tx_total : -1;
}

/* --- receiver: bc_advance(), USB emit replaced by a byte sink ------- */
static int g_bc_left[3], g_bc_miss[3];
static int g_bc_rx_group = 4, g_bc_ptype = -1, g_bc_last_seq = -1;
static int g_bc_frames, g_bc_lost;
static uint8_t g_out[4096];
static int g_out_n;
static rxs_t *g_rxs[3];

static int bc_advance(int m, const rxs_event_t *ev)
{
    if (ev->type == -3 && ev->hdr.typ == PKT_TYP_BCAST && g_bc_left[m] > 0) {
        g_bc_left[m]--;
        g_bc_lost++;
        if (++g_bc_miss[m] >= BC_MAX_MISS)
            g_bc_left[m] = 0;
        if (g_bc_left[m] > 0
            && !rxs_continue_burst(g_rxs[m], BURST_STREAM_RESYNC))
            g_bc_left[m] = 0;
        printf("  event: type -3 BCAST (stepped over), left %d\n",
               g_bc_left[m]);
        return 1;
    }
    if (ev->type != 1 || ev->hdr.typ != PKT_TYP_BCAST)
        return 0;
    {
        const uint8_t *b = ev->bits;
        int plen = (ev->pkt_bits_n - 36) / 8;
        int j, v, flags, seq, dlen, head = 2;

        for (j = 0, v = 0; j < 8; j++)
            v = (v << 1) | (b[20 + j] & 1);
        flags = v & ~BC_SEQ_MASK;
        seq = v & BC_SEQ_MASK;
        for (j = 0, v = 0; j < 8; j++)
            v = (v << 1) | (b[28 + j] & 1);
        dlen = v;
        if (flags & BC_SYNC) {
            int t = 0, q;
            head = 3;
            for (q = 0; q < 8; q++)
                t = (t << 1) | (b[36 + q] & 1);
            g_bc_ptype = t & 0x0F;
            g_bc_rx_group = 1 << (t >> 4);
            if (g_bc_rx_group < 1 || g_bc_rx_group > BURST_STREAM_MAX)
                g_bc_rx_group = 4;
            g_bc_left[m] = g_bc_rx_group - 1;
        } else if (g_bc_left[m] > 0) {
            g_bc_left[m]--;
        }
        if (g_bc_last_seq >= 0) {
            int gap = (seq - g_bc_last_seq - 1) & BC_SEQ_MASK;
            if (gap > 0 && gap < 32)
                g_bc_lost += gap;
        }
        g_bc_last_seq = seq;
        g_bc_frames++;
        if (dlen > plen - head)
            dlen = plen - head;
        for (j = 0; j < dlen && g_out_n < (int)sizeof(g_out); j++) {
            int bb, val = 0;
            for (bb = 0; bb < 8; bb++)
                val = (val << 1) | (b[20 + 8 * (head + j) + bb] & 1);
            g_out[g_out_n++] = (uint8_t)val;
        }
        printf("  event: type 1 BCAST seq %d flags %#x dlen %d plen %d "
               "left %d snr %+.1f\n", seq, flags, dlen, plen, g_bc_left[m],
               ev->snr_db);
        g_bc_miss[m] = 0;
        if (flags & BC_EOS)
            g_bc_left[m] = 0;
        if (g_bc_left[m] > 0
            && !rxs_continue_burst(g_rxs[m], BURST_STREAM_RESYNC))
            g_bc_left[m] = 0;
        return 1;
    }
}

static int16_t wire(int16_t s)
{
    int32_t v = s;
    if (QUANT) {
        uint32_t d = (uint32_t)(2048 + ((v * 3) >> 6));
        v = (int32_t)(d << 4) - 32768;
        v = (v * 85) >> 6;
    }
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

int main(int argc, char **argv)
{
    int i, lead = 700, pos = 0, got, air_n, groups = 0;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-r")) RUNG = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-len")) SRC_LEN = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-q")) QUANT = 1;
        else if (!strcmp(argv[i], "-all")) ALL_MODES = 1;
    }
    for (i = 0; i < SRC_LEN; i++)
        g_bc_src[i] = (uint8_t)(0x41 + (i % 26));
    g_bc_src_len = SRC_LEN;
    g_bc_ptype_tx = BC_PT_TELEMETRY;

    /* the firmware transmits one group per keying; concatenate them
     * here with a gap, which is what the wire carries */
    memset(g_air, 0, sizeof(*g_air) * (size_t)lead);
    pos = lead;
    while (g_bc_src_off < g_bc_src_len) {
        int n = 0;
        if (bc_open_group() <= 0) { printf("build failed\n"); return 1; }
        while (g_txs && (got = txs_pull(g_txs, g_wave + n,
                                        (int)(sizeof(g_wave) / 2) - n)) > 0)
            n += got;
        if (n != g_tx_total)
            printf("  WARNING: pulled %d of %d\n", n, g_tx_total);
        g_txs = 0;
        for (i = 0; i < n && pos < (int)(sizeof(g_air) / 2) - 4000; i++)
            g_air[pos++] = wire(g_wave[i]);
        memset(g_air + pos, 0, sizeof(*g_air) * 2000);   /* inter-group gap */
        pos += 2000;
        groups++;
    }
    air_n = pos + 700;
    printf("  %d group(s), %d air samples\n", groups, air_n);

    /* the firmware opens one receiver per mode over the same samples
     * (-all), which is the configuration a board runs in: the extra
     * detectors false-lock on a foreign mode's data and every lock
     * costs a decode attempt */
    g_rxs[(int)ladder_mode(RUNG)] = rxs_open(ladder_mode(RUNG), 0);
    if (ALL_MODES)
        for (i = 0; i < 3; i++)
            if (!g_rxs[i])
                g_rxs[i] = rxs_open((link_mode_t)i, 0);
    for (i = 0; i + 256 <= air_n; i += 256) {
        int m;
        for (m = 0; m < 3; m++) {
            rxs_event_t ev;
            if (!g_rxs[m] || !rxs_push(g_rxs[m], g_air + i, 256, &ev))
                continue;
            if (!bc_advance(m, &ev))
                printf("  event: mode %d type %d typ %d (not broadcast)\n",
                       m, ev.type, ev.hdr.typ);
        }
    }
    g_out[g_out_n < (int)sizeof(g_out) ? g_out_n : (int)sizeof(g_out) - 1] = 0;
    printf("RESULT rung=%d quant=%d: %d frames, %d lost, %d/%d bytes, "
           "ptype %d group %d\n  payload: \"%s\"\n", RUNG, QUANT,
           g_bc_frames, g_bc_lost, g_out_n, SRC_LEN, g_bc_ptype,
           g_bc_rx_group, (char *)g_out);
    return (g_out_n == SRC_LEN
            && memcmp(g_out, g_bc_src, (size_t)SRC_LEN) == 0) ? 0 : 1;
}
