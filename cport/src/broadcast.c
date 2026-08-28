#include <string.h>

#include "broadcast.h"
#include "rx_detect.h"
#include "rx_internal.h"
#include "packets.h"
#include "dsp.h"
#include "rom_modes.h"
#include "station.h"  /* BURST_STREAM_RESYNC */

#define BC_MAX_SAMPLES 600000
#define BC_PKT_BITS 2600

static samp_t g_bi[BC_MAX_SAMPLES], g_bq[BC_MAX_SAMPLES];
static uint8_t g_blocks[BC_MAX_GROUP * BC_PKT_BITS];
static int g_ok[BC_MAX_GROUP];

/* Preamble length for a mode: two tone fields (the first twice as long)
 * then the ZC symbol. */
static int bc_preamble_len(link_mode_t mode)
{
    static const int T[3] = { DET_T_NORMAL, DET_T_ROBUST, DET_T_EXTREME };
    static const int TILE[3] = { SYM_TILE_NORMAL, SYM_TILE_ROBUST,
                                 SYM_TILE_EXTREME };
    return 3 * T[mode] * FFT_BINS + (CP_LEN + TILE[mode] * FFT_BINS);
}

/* Absolute sample where the next group's preamble starts, at or after
 * `pos`, or -1.
 *
 * The window is deliberately too short to hold two preambles. Given a
 * longer one the detector's global argmax picks the STRONGEST preamble
 * rather than the first, and every group before it is lost without a
 * trace -- measured in the Python twin at 300 samples of lead-in being
 * enough to skip a whole group. A lock at or before the window start is
 * one already walked past; accepting it would return the same position
 * forever. */
static int bc_next_preamble(link_mode_t mode, int n, int pos, int symbol_len)
{
    int pre = bc_preamble_len(mode);
    int hdr = 6 * symbol_len;
    int win = pre + hdr, stride = hdr;

    while (pos + pre < n) {
        int start, want = win;
        int64_t cfo;
        if (pos + want > n)
            want = n - pos;
        if (rx_detect(mode, g_bi + pos, g_bq + pos, want, &start, &cfo) == 0) {
            int abs_start = pos + start - pre;
            if (abs_start >= pos)
                return abs_start;
        }
        pos += stride;
    }
    return -1;
}

/* Samples from a group's header start to the end of its last block --
 * laid out exactly as rxd_receive_burst walks it. */
static int bc_group_len(const rxd_header_t *hdr, int group, int symbol_len)
{
    int mu = hdr->mod == 2 ? 4 : (hdr->mod == 1 ? 2 : 1);
    int cap = N_DATA_CARRIERS * mu;
    int bits = PKT_BITS_FROM_HDR(hdr->typ, hdr->len);
    int coded = conv_cc_elements((cc_rate_t)hdr->spd, bits);
    int n_data = (coded + cap - 1) / cap;
    int n_hdr = (conv_cc_elements(CC_R13, HEADER_BITS) + N_DATA_CARRIERS - 1)
                / N_DATA_CARRIERS;
    int n_resync = BURST_STREAM_RESYNC > 0
                       ? (group - 1) / BURST_STREAM_RESYNC : 0;
    return (n_hdr + group * n_data + n_resync) * symbol_len;
}

int bc_receive(link_mode_t mode, const int16_t *samples, int n, int group,
               uint8_t *out, int out_cap, bc_stats_t *st)
{
    rxd_t r;
    int pos = 0, last_seq = -1, written = 0, extent = 0, restarted = 0;
    int pre, symbol_len;

    if (!st || group < 1 || group > BC_MAX_GROUP || n <= 0
        || n > BC_MAX_SAMPLES)
        return -1;
    memset(st, 0, sizeof(*st));
    st->ptype = -1;

    rxd_init(&r, mode);
    r.calibrate = 1; /* an unrecoverable frame is not retried, it is lost */
    symbol_len = r.symbol_len;
    pre = bc_preamble_len(mode);

    hilbert_analytic(samples, n, g_bi, g_bq);
    memset(g_bi + n, 0, sizeof(*g_bi) * (size_t)symbol_len);
    memset(g_bq + n, 0, sizeof(*g_bq) * (size_t)symbol_len);

    while (pos + pre < n) {
        rxd_header_t hdr;
        int start = bc_next_preamble(mode, n, pos, symbol_len);
        int end, got, k, glen, start_sample = 0, nres = 0;
        int64_t cfo = 0;

        if (start < 0)
            break;
        /* Bound the decode slice to one group once the geometry is
         * known. rxd_receive_burst re-detects over whatever slice it is
         * handed, so an unbounded one can commit to a later preamble and
         * skip this group. Searching forward for the next preamble to
         * find the bound does NOT work -- from inside a group the
         * detector false-locks on data -- but every group has the same
         * extent, so one successful decode supplies it. */
        end = extent ? start + extent : n;
        if (end > n)
            end = n;

        got = rxd_receive_burst(&r, samples + start, end - start, group,
                                BURST_STREAM_RESYNC, &hdr, g_blocks,
                                g_ok, &start_sample, &cfo, &nres);
        if (got < 0) {
            pos = start + symbol_len;
            continue;
        }
        glen = bc_group_len(&hdr, group, symbol_len);
        if (!extent) {
            /* preamble + header + blocks. Deriving this from
             * start_sample instead would inflate it whenever the first,
             * unbounded decode skipped ahead. */
            extent = pre + glen;
            if (!restarted) {
                /* the first decode had to run unbounded -- redo the walk
                 * now that the bound is available */
                restarted = 1;
                memset(st, 0, sizeof(*st));
                st->ptype = -1;
                written = 0;
                last_seq = -1;
                pos = 0;
                continue;
            }
        }

        st->groups++;
        for (k = 0; k < group; k++) {
            const uint8_t *bits;
            int bits_n, plen, flags, seq, dlen, head, j;
            if (!g_ok[k])
                continue;
            bits_n = PKT_BITS_FROM_HDR(hdr.typ, hdr.len);
            bits = g_blocks + (size_t)k * bits_n;
            plen = (bits_n - 36) / 8;
            if (plen < BC_HEAD)
                continue;
            {   /* unpack the two framing bytes */
                int b0 = 0, b1 = 0;
                for (j = 0; j < 8; j++)
                    b0 = (b0 << 1) | (bits[20 + j] & 1);
                for (j = 0; j < 8; j++)
                    b1 = (b1 << 1) | (bits[28 + j] & 1);
                flags = b0 & ~BC_SEQ_MASK;
                seq = b0 & BC_SEQ_MASK;
                dlen = b1;
            }
            /* sequence gaps are the only loss signal there is */
            if (last_seq >= 0) {
                int gap = (seq - last_seq - 1) & (BC_SEQ_MOD - 1);
                if (gap > 0 && gap < BC_SEQ_MOD / 2)
                    st->frames_lost += gap;
            }
            last_seq = seq;
            st->frames_ok++;
            st->snr_sum += r.last_snr_db;
            head = BC_HEAD;
            if (flags & BC_SYNC) {
                if (plen > BC_HEAD) {
                    int pt = 0;
                    for (j = 0; j < 8; j++)
                        pt = (pt << 1) | (bits[20 + 8 * BC_HEAD + j] & 1);
                    /* log2(group) << 4 | payload type: the group size has
                     * to be on the wire, or a receiver that guesses wrong
                     * decodes the first frame of each group and no more */
                    st->ptype = pt & 0x0F;
                    st->group = 1 << (pt >> 4);
                }
                head = BC_HEAD + 1;
            }
            if (flags & BC_EOS)
                st->saw_eos = 1;
            if (dlen > plen - head)
                dlen = plen - head;
            for (j = 0; j < dlen && written < out_cap; j++) {
                int v = 0, b;
                for (b = 0; b < 8; b++)
                    v = (v << 1) | (bits[20 + 8 * (head + j) + b] & 1);
                out[written++] = (uint8_t)v;
            }
        }

        {
            int nxt = start + start_sample + glen;
            pos = nxt > pos + symbol_len ? nxt : pos + symbol_len;
        }
    }

    st->bytes_out = written;
    return written;
}
