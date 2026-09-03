/* Link-layer validation: LC-word golden vectors, controller parity against
 * a Python-recorded op trace, and a full two-station simplex session over
 * the C fixed PHY (virtual time, deterministic channel). */
#include <stdio.h>
#include <string.h>

#include "../src/link.h"
#include "../src/station.h"
#include "../src/packets.h"
#include "../src/tx.h"
#include "../src/rx_demod.h"
#include "../src/rom_link.h"
#include "test_vectors.h"

static int g_pass, g_fail;

static void check(const char *name, int ok)
{
    printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (ok)
        g_pass++;
    else
        g_fail++;
}

/* ---------------- LC word ---------------- */

static void test_lc(void)
{
    lc_word_t lc;
    int ok = 1;
#define LC_CASE(I)                                                          \
    do {                                                                    \
        lc_unpack((uint32_t)LC_PACKED[I], &lc);                             \
        if (lc_pack(&lc) != (uint32_t)LC_PACKED[I] || lc.seq != LC##I##_SEQ \
            || lc.ack != LC##I##_ACK || lc.req_rung != LC##I##_RUNG        \
            || lc.flags != LC##I##_FLAGS || lc.snr_db != LC##I##_SNR       \
            || lc.freq_corr_hz != LC##I##_FREQ)                            \
            ok = 0;                                                        \
    } while (0)
    LC_CASE(0); LC_CASE(1); LC_CASE(2); LC_CASE(3); LC_CASE(4);
    LC_CASE(5);
    check("lc word pack/unpack (incl. half-even rounding)", ok);
    /* the two ends of the "no measurement" code, which the cases above
     * only cover as data: a real report near the floor must never be
     * mistaken for an absent one, in either direction */
    {
        lc_word_t a, b;
        memset(&a, 0, sizeof(a));
        a.snr_db = LC_SNR_NONE;
        lc_unpack(lc_pack(&a), &b);
        check("no-measurement packs to code 0 and comes back as none",
              ((lc_pack(&a) >> 8) & 15) == 0 && LC_SNR_IS_NONE(b.snr_db));
        a.snr_db = -30.0;                    /* real, below the field */
        lc_unpack(lc_pack(&a), &b);
        check("a real report below the floor clamps to -22, not to none",
              ((lc_pack(&a) >> 8) & 15) == 1 && b.snr_db == -22.0);
    }
}

/* ---------------- controller parity ---------------- */

static void test_ctl(void)
{
    link_ctl_t c;
    lc_word_t lc;
    int i, ok = 1;
    ctl_init(&c);
    for (i = 0; i < CTL_OPS_N; i++) {
        int got = -12345;
        switch (CTL_OP[i]) {
        case 0:
            memset(&lc, 0, sizeof(lc));
            lc.req_rung = CTL_B[i];
            lc.snr_db = CTL_C[i];
            ctl_on_rx_frame(&c, CTL_A[i], &lc, CTL_T[i]);
            continue;
        case 1: got = ctl_rx_request(&c, CTL_T[i]); break;
        case 2: got = ctl_tx_rung(&c, CTL_T[i]); break;
        case 3: ctl_on_timeout(&c); continue;
        case 4: ctl_on_ack(&c); continue;
        case 5: ctl_note_outcome(&c, (int)CTL_A[i], CTL_B[i]); continue;
        case 6: {
            double s = ctl_filtered_snr(&c, CTL_T[i]) * 1000.0;
            got = (int)(s >= 0 ? s + 0.5 : s - 0.5);
            break;
        }
        case 7: got = ctl_tx_rung_for_class(&c, CTL_T[i], QOS_CONTROL); break;
        }
        if (got != CTL_WANT[i]) {
            printf("  ctl op %d (code %d): got %d want %d\n", i, CTL_OP[i],
                   got, CTL_WANT[i]);
            ok = 0;
        }
    }
    check("controller parity trace vs python", ok);
}

/* ------- burst timeout forgiveness (first ack miss is not a loss) ------ */

static int stub_build(void *ctx, const uint8_t *bits, int n, int typ,
                      int rung, int16_t *out, int out_cap)
{
    (void)ctx; (void)bits; (void)n; (void)typ; (void)rung; (void)out;
    (void)out_cap;
    return 1000; /* pretend a 1000-sample frame went out */
}

static void test_burst_forgiveness(void)
{
    static station_t C;
    static int16_t air[2000];
    station_phy_t phy = { 0, stub_build, 0, 0, 0 };
    static uint8_t bulk[250];
    lc_word_t lc;
    double t = 100.0;
    int sent = 0, guard;

    station_init(&C, &phy, 3);
    C.caps_disabled = 1;              /* not what this test is about */
    C.burst_window = 8;

    /* prime the controller: peer reports +6 dB and requests rung 11 */
    memset(&lc, 0, sizeof(lc));
    lc.flags = FLAG_NO_DATA;
    lc.req_rung = 11;
    lc.snr_db = 6.0;
    ctl_on_rx_frame(&C.ctl, 6.0, &lc, t);

    station_submit(&C, bulk, 250, QOS_BULK);
    /* drain the window: frames until one requests the ack */
    for (guard = 0; guard < 20 && !C.expects_reply; guard++) {
        int n = station_poll_tx(&C, t, 0, air, 2000);
        if (n > 0) {
            sent++;
            station_on_tx_end(&C, t + 0.1);
            t += 0.2;
        } else {
            t += 0.5;
        }
    }
    check("burst window sent, ack requested",
          C.btx.active && C.expects_reply && sent >= 1);

    /* let the ack window expire: first miss must NOT count as a loss */
    t = C.await_until + 0.1;
    station_poll_tx(&C, t, 0, air, 2000);
    check("first ack miss forgiven (no controller loss)",
          C.ctl.consecutive_losses == 0 && C.btx.miss == 1
          && C.stats.timeouts == 1);

    /* probe goes out and its miss DOES count */
    for (guard = 0; guard < 20; guard++) {
        int n = station_poll_tx(&C, t, 0, air, 2000);
        if (n > 0) {
            station_on_tx_end(&C, t + 0.1);
            break;
        }
        t += 1.0;
    }
    t = C.await_until + 0.1;
    station_poll_tx(&C, t, 0, air, 2000);
    check("second miss counts as a real loss",
          C.ctl.consecutive_losses == 1 && C.btx.miss == 2);
}

/* ------------- extended (255-byte) frame TX -> RX loopback ------------- */

static int16_t g_ext_air[600000];
static uint32_t g_nlcg; /* defined below with the session harness */
static int16_t nz(void);

static void test_ext_frame(void)
{
    static uint8_t payload[255], pkt_bits[2600], rx_bits[2600];
    rxd_t r;
    rxd_header_t h;
    int k, n, pkt_n, start, ok;
    int64_t cfo;

    for (k = 0; k < 255; k++)
        payload[k] = (uint8_t)(k * 13 + 7);
    pkt_n = data_encode(0x5A5A5, payload, 255, pkt_bits);
    check("ext packet is 2076 bits", pkt_n == 36 + 8 * 255);

    n = tx_build_frame(MODE_NORMAL, pkt_bits, pkt_n, PKT_TYP_EXT_DATA,
                       MOD_QPSK, CC_R12, g_ext_air);
    check("ext frame builds", n > 0);

    /* lead-in/out noise as in the session test */
    for (k = n - 1; k >= 0; k--)
        g_ext_air[700 + k] = g_ext_air[k];
    g_nlcg = 777;
    for (k = 0; k < 700; k++)
        g_ext_air[k] = nz();
    for (k = 0; k < 700; k++)
        g_ext_air[700 + n + k] = nz();

    rxd_init(&r, MODE_NORMAL);
    ok = rxd_receive(&r, g_ext_air, n + 1400, &h, rx_bits, &start, &cfo)
         == 0;
    check("ext frame decodes", ok);
    check("ext header: typ=EXT len=255 bytes",
          ok && h.typ == PKT_TYP_EXT_DATA && h.len == 255);
    check("ext frame payload bit-exact",
          ok && memcmp(rx_bits, pkt_bits, (size_t)pkt_n) == 0);

    /* LDPC + EXT is rejected at build time */
    check("ext + LDPC rejected",
          tx_build_frame_ex(MODE_NORMAL, pkt_bits, pkt_n, PKT_TYP_EXT_DATA,
                            MOD_QPSK, CC_R12, 1, g_ext_air) < 0);
}

/* ------------- oscillator fine-tune endpoint (AFC actuator) ------------ */

static double g_lo_hz; /* the simulated VCTCXO: accumulates relative trims */

static void lo_trim_stub(void *ctx, double hz)
{
    (void)ctx;
    g_lo_hz += hz;
}

static void test_freq_trim(void)
{
    static station_t C;
    station_phy_t phy = { 0, 0, 0, 0, 0 }; /* on_decoded never touches the PHY */
    static uint8_t pkt[128];
    lc_word_t lc;
    int pkt_n;
    uint8_t one = 0;
    double d;

    station_init(&C, &phy, 1);
    g_lo_hz = 0.0;
    station_set_freq_trim(&C, lo_trim_stub, 0, 10.0, 0);

    d = station_freq_trim(&C, 7.0);
    check("manual trim applied", d == 7.0 && g_lo_hz == 7.0);
    d = station_freq_trim(&C, 7.0);
    check("manual trim clamped at the budget",
          d == 3.0 && station_freq_trim_total(&C) == 10.0);
    d = station_freq_trim(&C, -25.0);
    check("manual trim clamped low side",
          d == -20.0 && station_freq_trim_total(&C) == -10.0
          && g_lo_hz == -10.0);

    /* peer requests +40 Hz via the LC word -> AFC nets gain*40 = +20 */
    memset(&lc, 0, sizeof(lc));
    lc.flags = FLAG_NO_DATA;
    lc.snr_db = 5.0;
    lc.freq_corr_hz = 40.0; /* multiple of FREQ_STEP_HZ: survives packing */
    pkt_n = data_encode(lc_pack(&lc), &one, 1, pkt);
    station_on_decoded(&C, pkt, pkt_n, 5.0, 0.0, 0, 1.0);
    check("AFC netting drives the actuator",
          station_freq_trim_total(&C) == 10.0 && g_lo_hz == 10.0
          && C.stats.afc_trims == 1);

    /* an anchor station ignores peer requests but obeys the operator */
    station_set_freq_trim(&C, lo_trim_stub, 0, 10.0, 1);
    station_on_decoded(&C, pkt, pkt_n, 5.0, 0.0, 0, 2.0);
    check("anchor ignores peer trim requests",
          station_freq_trim_total(&C) == 0.0 && C.stats.afc_trims == 1);
    d = station_freq_trim(&C, -4.0);
    check("anchor still accepts manual trim",
          d == -4.0 && station_freq_trim_total(&C) == -4.0);
}

/* ---------------- two-station session over the C fixed PHY -------------- */

static uint8_t g_rx_bits[2600];

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

static int phy_receive(void *ctx, const int16_t *s, int n, uint8_t *pkt_bits,
                       int *pkt_bits_n, double *snr_db, double *cfo_hz,
                       int *harq_combined, const int64_t *prev_llrs,
                       int prev_n, int64_t *llrs_out, int *llrs_n)
{
    int m;
    (void)ctx;
    (void)prev_llrs;
    (void)prev_n;
    (void)llrs_out;
    if (llrs_n)
        *llrs_n = 0;
    for (m = 0; m < 3; m++) { /* try modes in turn, as demod_frame_auto */
        rxd_t r;
        rxd_header_t h;
        int start;
        int64_t cfo;
        rxd_init(&r, (link_mode_t)m);
        if (rxd_receive(&r, s, n, &h, g_rx_bits, &start, &cfo) == 0) {
            int nb = PKT_BITS_FROM_HDR(h.typ, h.len);
            memcpy(pkt_bits, g_rx_bits, (size_t)nb);
            *pkt_bits_n = nb;
            *snr_db = r.last_snr_db;
            *cfo_hz = (double)cfo * 12000.0 / 4294967296.0;
            *harq_combined = 0;
            return 0;
        }
    }
    return -1;
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

static int phy_receive_burst(void *ctx, const int16_t *s, int n,
                             int max_blocks, int resync_every,
                             uint8_t *pkt_bits, int *pkt_bits_n, int *ok,
                             double *snr_db, double *cfo_hz)
{
    static uint8_t bits[BURST_STREAM_MAX * 2600];
    int m;
    (void)ctx;
    for (m = 0; m < 3; m++) {
        rxd_t r;
        rxd_header_t h;
        int start, nres, got, k, nb;
        int64_t cfo;
        rxd_init(&r, (link_mode_t)m);
        got = rxd_receive_burst(&r, s, n, max_blocks, resync_every, &h, bits,
                                ok, &start, &cfo, &nres);
        if (got < 0)
            continue;
        nb = PKT_BITS_FROM_HDR(h.typ, h.len);
        for (k = 0; k < max_blocks; k++)
            memcpy(pkt_bits + (size_t)k * 2600, bits + (size_t)k * nb,
                   (size_t)nb);
        *pkt_bits_n = nb;
        *snr_db = r.last_snr_db;
        *cfo_hz = (double)cfo * 12000.0 / 4294967296.0;
        return max_blocks;
    }
    return -1;
}

static int16_t g_air[600000], g_rxbuf[600000];
static uint32_t g_nlcg = 5555;

static int16_t nz(void)
{
    g_nlcg = g_nlcg * 1103515245u + 12345u;
    return (int16_t)((int)((g_nlcg >> 16) % 200u) - 100);
}

static void test_session(void)
{
    static station_t A, B;
    station_phy_t phy = { 0, phy_build, phy_receive, phy_build_burst, phy_receive_burst };
    static const uint8_t msg_cq[] = "CQ DE R9FEU";
    static uint8_t msg_bulk[96];
    static const uint8_t msg_reply[] = "R R TNX DE UB1ABC 73";
    station_t *owner = 0;
    double t = 0.0, tx_end = 0.0;
    int air_n = 0, k, b_submitted = 0, ok;

    for (k = 0; k < 96; k++)
        msg_bulk[k] = (uint8_t)(k * 7 + 1);

    station_init(&A, &phy, 111);
    station_init(&B, &phy, 222);
    station_submit(&A, msg_cq, (int)sizeof(msg_cq) - 1, QOS_CONTROL);
    station_submit(&A, msg_bulk, 96, QOS_BULK);

    while (t < 1200.0) {
        if (owner && t >= tx_end) {
            station_t *peer = owner == &A ? &B : &A;
            station_on_tx_end(owner, tx_end);
            /* deliver with lead-in/out noise, as a real channel would (the
             * Hilbert transient must not chew the preamble head) */
            for (k = 0; k < 700; k++)
                g_rxbuf[k] = nz();
            for (k = 0; k < air_n; k++)
                g_rxbuf[700 + k] = (int16_t)(g_air[k] + nz());
            for (k = 0; k < 700; k++)
                g_rxbuf[700 + air_n + k] = nz();
            station_rx_frame(peer, g_rxbuf, air_n + 1400, tx_end);
            owner = 0;
            if (!b_submitted && B.delivered_n >= 2) {
                station_submit(&B, msg_reply, (int)sizeof(msg_reply) - 1,
                               QOS_INTERACTIVE);
                b_submitted = 1;
            }
        }
        if (!owner) {
            station_t *st = ((int)(t * 10) & 1) ? &A : &B;
            station_t *other = st == &A ? &B : &A;
            int n = station_poll_tx(st, t, 0, g_air, 600000);
            if (!n)
                n = station_poll_tx(other, t, 0, g_air, 600000),
                st = other;
            if (n > 0) {
                owner = st;
                air_n = n;
                tx_end = t + (double)n / 12000.0;
            }
        }
        if (!owner && !station_has_traffic(&A) && !station_has_traffic(&B)
            && B.delivered_n >= 2 && A.delivered_n >= 1)
            break;
        t += 0.1;
    }

    ok = B.delivered_n == 2 && A.delivered_n == 1
         && B.delivered_len[0] == (int)sizeof(msg_cq) - 1
         && memcmp(station_delivered(&B, 0), msg_cq, sizeof(msg_cq) - 1) == 0
         && B.delivered_len[1] == 96
         && memcmp(station_delivered(&B, 1), msg_bulk, 96) == 0
         && A.delivered_len[0] == (int)sizeof(msg_reply) - 1
         && memcmp(station_delivered(&A, 0), msg_reply, sizeof(msg_reply) - 1) == 0;
    check("two-station session: all messages bit-exact", ok);
    check("rate ladder climbed off the EXTREME bootstrap",
          A.stats.last_rung > 0 && B.stats.last_rung > 0);
    printf("  session: %.1f virtual s, A tx %d rx %d (last rung %d), "
           "B tx %d rx %d (last rung %d)\n",
           t, A.stats.tx_frames, A.stats.rx_frames, A.stats.last_rung,
           B.stats.tx_frames, B.stats.rx_frames, B.stats.last_rung);
    if (!ok)
        printf("  delivered: B %d msgs, A %d msgs\n", B.delivered_n,
               A.delivered_n);

    /* phase 2: burst (selective-repeat) bulk transfer on the adapted link */
    {
        static uint8_t big[250];
        int tx0 = A.stats.tx_frames + B.stats.tx_frames;
        /* legacy = stop-and-wait with 25-byte fragments */
        int nleg = (250 + BURST_FRAG_SIZE - 1) / BURST_FRAG_SIZE;
        int deliv0 = B.delivered_n;
        double t0 = t;
        for (k = 0; k < 250; k++)
            big[k] = (uint8_t)(k * 31 + 5);
        A.burst_window = 8;
        B.burst_window = 8;
        station_submit(&A, big, 250, QOS_BULK);
        while (t < t0 + 900.0) {
            if (owner && t >= tx_end) {
                station_t *peer = owner == &A ? &B : &A;
                station_on_tx_end(owner, tx_end);
                for (k = 0; k < 700; k++)
                    g_rxbuf[k] = nz();
                for (k = 0; k < air_n; k++)
                    g_rxbuf[700 + k] = (int16_t)(g_air[k] + nz());
                for (k = 0; k < 700; k++)
                    g_rxbuf[700 + air_n + k] = nz();
                station_rx_frame(peer, g_rxbuf, air_n + 1400, tx_end);
                owner = 0;
            }
            if (!owner) {
                station_t *st = ((int)(t * 10) & 1) ? &A : &B;
                station_t *other = st == &A ? &B : &A;
                int n = station_poll_tx(st, t, 0, g_air, 600000);
                if (!n)
                    n = station_poll_tx(other, t, 0, g_air, 600000),
                    st = other;
                if (n > 0) {
                    owner = st;
                    air_n = n;
                    tx_end = t + (double)n / 12000.0;
                }
            }
            if (!owner && B.delivered_n > deliv0 && !station_has_traffic(&A))
                break;
            t += 0.1;
        }
        ok = B.delivered_n == deliv0 + 1
             && B.delivered_len[deliv0] == 250
             && memcmp(station_delivered(&B, deliv0), big, 250) == 0;
        check("burst transfer: 250 bytes bit-exact", ok);
        {
            int used = A.stats.tx_frames + B.stats.tx_frames - tx0;
            int fs = A.btx.frag_size;
            /* legacy stop-and-wait would need 2 frames per 25-byte chunk */
            check("burst efficiency: fewer frames than stop-and-wait",
                  ok && used < 2 * nleg - 4);
            printf("  burst: 250 B as %d-byte EXT fragments in %d frames "
                   "(legacy would be ~%d), %.1f virtual s\n", fs, used,
                   2 * nleg, t - t0);
        }
    }
}

/* ---------------- streamed bursts + the fallback path ------------------- */

/* run the two stations against each other until B has a new delivery or
 * the clock runs out; returns the frames put on air */
static int pump(station_t *A, station_t *B, double *tp, double limit,
                int deliv0)
{
    station_t *owner = 0;
    double t = *tp, tx_end = 0.0;
    int air_n = 0, k, frames = 0;

    while (t < limit) {
        if (owner && t >= tx_end) {
            station_t *peer = owner == A ? B : A;
            station_on_tx_end(owner, tx_end);
            for (k = 0; k < 700; k++)
                g_rxbuf[k] = nz();
            for (k = 0; k < air_n; k++)
                g_rxbuf[700 + k] = (int16_t)(g_air[k] + nz());
            for (k = 0; k < 700; k++)
                g_rxbuf[700 + air_n + k] = nz();
            station_rx_frame(peer, g_rxbuf, air_n + 1400, tx_end);
            owner = 0;
        }
        if (!owner) {
            station_t *st = ((int)(t * 10) & 1) ? A : B;
            station_t *other = st == A ? B : A;
            int n = station_poll_tx(st, t, 0, g_air, 600000);
            if (!n)
                n = station_poll_tx(other, t, 0, g_air, 600000), st = other;
            if (n > 0) {
                owner = st;
                air_n = n;
                tx_end = t + (double)n / 12000.0;
                frames++;
            }
        }
        if (!owner && B->delivered_n > deliv0 && !station_has_traffic(A))
            break;
        t += 0.1;
    }
    *tp = t;
    return frames;
}

/* one 250-byte transfer at a mid rung (25-byte fragments -> 10 of them,
 * all full size). peer_streams=0 gives the receiver no burst decoder,
 * which is the fallback case: the transfer must still complete. */
static void stream_case(const char *name, int peer_streams, int *frames_out,
                        int *stream_off_out)
{
    static station_t A, B;
    station_phy_t pa = { 0, phy_build, phy_receive, phy_build_burst,
                         phy_receive_burst };
    station_phy_t pb = { 0, phy_build, phy_receive, phy_build_burst,
                         phy_receive_burst };
    static uint8_t big[250];
    lc_word_t lc;
    double t = 100.0;
    int k, frames, ok;

    if (!peer_streams)
        pb.receive_burst = 0;
    for (k = 0; k < 250; k++)
        big[k] = (uint8_t)(k * 31 + 5);

    station_init(&A, &pa, 4242);
    station_init(&B, &pb, 2424);
    A.burst_window = 8;
    B.burst_window = 8;
    A.burst_stream = 1;
    B.burst_stream = 1;

    /* prime both controllers at a mid rung so fragments are 25 bytes */
    memset(&lc, 0, sizeof(lc));
    lc.flags = FLAG_NO_DATA;
    lc.req_rung = 5;
    lc.snr_db = -4.0;
    ctl_on_rx_frame(&A.ctl, -4.0, &lc, t);
    ctl_on_rx_frame(&B.ctl, -4.0, &lc, t);

    station_submit(&A, big, 250, QOS_BULK);
    frames = pump(&A, &B, &t, 3000.0, 0);

    ok = B.delivered_n >= 1 && B.delivered_len[0] == 250
         && memcmp(station_delivered(&B, 0), big, 250) == 0;
    check(name, ok);
    printf("  %s: %d frames on air, frag %d B, stream_ok=%d\n", name, frames,
           A.btx.frag_size, A.btx.stream_ok);
    if (frames_out)
        *frames_out = frames;
    if (stream_off_out)
        *stream_off_out = !A.btx.stream_ok;
}

static void test_burst_stream(void)
{
    int streamed = 0, fallback = 0, off_when_streaming = 0, off_on_fallback = 0;

    stream_case("streamed burst: 250 bytes bit-exact", 1, &streamed,
                &off_when_streaming);
    stream_case("fallback (peer cannot decode streams): 250 bytes bit-exact",
                0, &fallback, &off_on_fallback);

    check("streaming needs fewer transmissions than per-frame bursts",
          streamed < fallback);
    check("a peer that cannot follow the stream turns streaming off",
          off_on_fallback);
    printf("  streamed %d transmissions vs %d after fallback\n", streamed,
           fallback);
}

/* ---------------- capability handshake ---------------- */

/* run one 250-byte bulk transfer A -> B with the given caps masks and
 * report what each side learned */
/* codecs for the next caps_case(); station_init() clears the station, so
 * a caller cannot set my_codecs before the call */
static int g_case_codecs_a, g_case_codecs_b;

static void caps_case(int a_caps, int b_caps, int b_answers,
                      station_t *A, station_t *B, int *frames_out)
{
    station_phy_t pa = { 0, phy_build, phy_receive, phy_build_burst,
                         phy_receive_burst };
    station_phy_t pb = pa;
    static uint8_t big[250];
    lc_word_t lc;
    double t = 100.0;
    int k;

    for (k = 0; k < 250; k++)
        big[k] = (uint8_t)(k * 31 + 5);
    station_init(A, &pa, 4242);
    station_init(B, &pb, 2424);
    A->burst_window = B->burst_window = 8;
    A->burst_stream = B->burst_stream = 1;
    A->my_caps = a_caps;
    B->my_caps = b_caps;
    /* station_init() zeroed the station, so these are set HERE and not
     * by the caller before the call */
    A->my_codecs = g_case_codecs_a;
    B->my_codecs = g_case_codecs_b;
    B->fw_ver = 0x0201;
    B->caps_disabled = !b_answers;    /* a peer from before the handshake */

    memset(&lc, 0, sizeof(lc));
    lc.flags = FLAG_NO_DATA;
    lc.req_rung = 5;
    lc.snr_db = -4.0;
    ctl_on_rx_frame(&A->ctl, -4.0, &lc, t);
    ctl_on_rx_frame(&B->ctl, -4.0, &lc, t);

    station_submit(A, big, 250, QOS_BULK);
    *frames_out = pump(A, B, &t, 3000.0, 0);
    check("caps: the transfer completes",
          B->delivered_n >= 1 && B->delivered_len[0] == 250
          && memcmp(station_delivered(B, 0), big, 250) == 0);
}

/* A capability kick that no probe will honour must not keep the
 * transmitter running. Found on the stand: 117 frames in 108 seconds
 * with nothing attached to either board -- an ack-only frame every air
 * time, forever, because the kick survived a caps_probe_wanted() that
 * declines on its first line for a peer already known. */
/* The burst state must not outlive the message it is sending.
 * msg_data() used to return pool[-1] for a released slot and the burst
 * paths memcpy'd it into a packet -- an out-of-bounds read that went on
 * the air (ASan-reproduced from ordinary traffic: a data frame, an ack,
 * a partial bitmap). */
static void test_burst_outlives_message(void)
{
    station_phy_t p = { 0, phy_build, phy_receive, phy_build_burst,
                        phy_receive_burst };
    static station_t S;
    static int16_t out[600000];
    static uint8_t msg[250];
    int i, n;
    double t = 100.0;

    for (i = 0; i < (int)sizeof(msg); i++)
        msg[i] = (uint8_t)i;

    /* abort while a burst is engaged: every field must go, not just
     * .active -- burst_stream_ready() does not test .active */
    station_init(&S, &p, 31337);
    S.caps_disabled = 1;          /* drive this station by hand: a
                                   * stranger peer would get a probe
                                   * before any burst */
    S.burst_window = 8;
    S.burst_stream = 1;
    S.ctl.peer_req = 10;
    S.ctl.peer_report_db = 20.0;
    S.ctl.peer_req_time = t;
    station_submit(&S, msg, (int)sizeof(msg), QOS_BULK);
    n = station_poll_tx(&S, t, 0, out, (int)(sizeof(out) / sizeof(out[0])));
    check("a burst engages", n > 0 && S.btx.active && S.btx.window_left >= 0);
    check("and the engaged window is recorded as a statistic",
          S.stats.last_burst_win > 0);
    station_abort_bulk(&S);
    check("abort drops the WHOLE burst state, not just the flag",
          !S.btx.active && S.btx.window_left == 0 && !S.btx.stream_ok);
    check("abort also drops the fragment pointing at the freed message",
          !S.pending.active);
    check("and the message's slot went back", S.cur_bulk.slot < 0
          && S.pool_used == 0);

    /* a released message under an engaged burst must produce silence,
     * not a read from pool[-1] */
    station_init(&S, &p, 31337);
    S.caps_disabled = 1;
    S.burst_window = 8;
    S.ctl.peer_req = 10;
    S.ctl.peer_report_db = 20.0;
    S.ctl.peer_req_time = t;
    station_submit(&S, msg, (int)sizeof(msg), QOS_BULK);
    station_poll_tx(&S, t, 0, out, (int)(sizeof(out) / sizeof(out[0])));
    msg_release_for_test(&S);          /* what the legacy ack path does */
    S.btx.window_left = S.btx.win;     /* what a partial bitmap does */
    n = station_poll_tx(&S, t + 1.0, 0, out,
                        (int)(sizeof(out) / sizeof(out[0])));
    check("a burst whose message was released transmits nothing", n == 0);
    check("and disengages rather than trying again", !S.btx.active);
}

/* Every frame carries an ack. The CAPS / BURST_ACK / BURST_DATA
 * branches used to return before the ARQ retire, so an ack riding on
 * one of them was discarded -- while the sender had already cleared its
 * own reply_due and believed it had answered. */
/* A completed message that cannot be stored must not be acknowledged
 * as delivered: the sender can never detect that failure. The burst
 * path gives the fragment back so the bitmap tells the truth. */
static void test_full_store_does_not_lie(void)
{
    static station_t A, B;
    static uint8_t msg[120];
    int i, frames;
    double t = 100.0;

    for (i = 0; i < (int)sizeof(msg); i++)
        msg[i] = (uint8_t)(i * 7 + 1);

    caps_case(CAP_STREAM | CAP_EXT | CAP_LDPC | CAP_BURST,
              CAP_STREAM | CAP_EXT | CAP_LDPC | CAP_BURST, 1, &A, &B,
              &frames);
    /* fill B's delivered log so the next completion cannot be stored */
    B.delivered_n = ST_DELIVERED_MAX;
    station_submit(&A, msg, (int)sizeof(msg), QOS_BULK);
    pump(&A, &B, &t, 600.0, 0);
    check("a completed transfer the receiver cannot store is NOT marked "
          "done", !B.brx.done);
    check("and the sender still holds the message rather than believing "
          "it arrived", A.cur_bulk.active || A.qcount[QOS_BULK] > 0);
}

static void test_ack_on_any_frame(void)
{
    station_phy_t p = { 0, phy_build, phy_receive, phy_build_burst,
                        phy_receive_burst };
    static station_t S;
    static int16_t out[600000];
    static uint8_t bits[2600];
    lc_word_t lc;
    double t = 100.0;
    int n, flags[] = { FLAG_CAPS, FLAG_BURST_ACK, FLAG_BURST_DATA, 0 };
    int k, ok = 1;

    for (k = 0; flags[k] != 0 || k == 3; k++) {
        station_init(&S, &p, 4242);
        S.caps_disabled = 1;
        S.ctl.peer_req = 10;
        S.ctl.peer_report_db = 20.0;
        S.ctl.peer_req_time = t;
        station_submit(&S, (const uint8_t *)"hello", 5, QOS_INTERACTIVE);
        n = station_poll_tx(&S, t, 0, out,
                            (int)(sizeof(out) / sizeof(out[0])));
        if (n <= 0 || !S.pending.active) {
            ok = 0;
            break;
        }
        /* a frame of this class, carrying an ack for what we just sent */
        memset(&lc, 0, sizeof(lc));
        lc.seq = 2;
        lc.ack = S.pending.seq;
        lc.snr_db = 10.0;
        lc.flags = flags[k];
        memset(bits, 0, sizeof(bits));
        {
            uint32_t v = lc_pack(&lc);
            int i;
            for (i = 0; i < 20; i++)
                bits[i] = (uint8_t)((v >> (19 - i)) & 1);
        }
        station_on_decoded(&S, bits, 36 + 8, 10.0, 0.0, 0, t + 1.0);
        if (S.pending.active)
            ok = 0;                 /* the ack was thrown away */
        if (flags[k] == 0)
            break;
    }
    check("an ack is honoured on CAPS, burst-ack, burst-data and plain "
          "frames alike", ok);
}

/* A burst owns cur_bulk: the legacy path must not send the same message
 * again behind its back (it did, into the same reassembly buffer, and
 * left btx.active set forever if it completed it). */
static void test_burst_owns_its_message(void)
{
    station_phy_t p = { 0, phy_build, phy_receive, phy_build_burst,
                        phy_receive_burst };
    static station_t S;
    static int16_t out[600000];
    static uint8_t msg[250];
    double t = 100.0;
    int i, n;

    for (i = 0; i < (int)sizeof(msg); i++)
        msg[i] = (uint8_t)i;
    station_init(&S, &p, 999);
    S.caps_disabled = 1;
    S.burst_window = 8;
    S.ctl.peer_req = 10;
    S.ctl.peer_report_db = 20.0;
    S.ctl.peer_req_time = t;
    station_submit(&S, msg, (int)sizeof(msg), QOS_BULK);
    station_poll_tx(&S, t, 0, out, (int)(sizeof(out) / sizeof(out[0])));
    check("a burst engages and owns the message",
          S.btx.active && S.cur_bulk.active);

    S.btx.window_left = 0;          /* the ack-requesting fragment is out */
    S.await_until = -1.0;           /* ... and any decoded frame cleared it */
    n = station_poll_tx(&S, t + 1.0, 0, out,
                        (int)(sizeof(out) / sizeof(out[0])));
    check("the legacy path does not re-send it", n == 0);
    check("and the burst is still engaged, waiting for its ack",
          S.btx.active && !S.pending.active);

    /* control traffic is still allowed through */
    station_submit(&S, (const uint8_t *)"ctl", 3, QOS_CONTROL);
    n = station_poll_tx(&S, t + 2.0, 0, out,
                        (int)(sizeof(out) / sizeof(out[0])));
    check("but control traffic still gets out", n > 0);
}

static void test_caps_kick_orphan(void)
{
    station_phy_t p = { 0, phy_build, phy_receive, phy_build_burst,
                        phy_receive_burst };
    static station_t S;
    /* rung 0 is EXTREME: one frame is ~456 000 samples, and the build
     * stub refuses a buffer that cannot hold it */
    static int16_t out[600000];
    double t = 100.0;
    int n, i, frames = 0;

    station_init(&S, &p, 7777);
    S.peer.valid = 1;                 /* the handshake already happened */
    S.peer.t = t;
    S.caps_kick = 1;                  /* ... and a stale kick survived it */

    for (i = 0; i < 20; i++) {
        n = station_poll_tx(&S, t, 0, out,
                            (int)(sizeof(out) / sizeof(out[0])));
        if (n > 0)
            frames++;
        t += 1.0;
    }
    check("an orphaned caps kick transmits nothing", frames == 0);
    check("and is cleared rather than left to fire again", !S.caps_kick);

    /* the same station still answers a frame that carried data */
    S.last_rx_seq = 1;
    S.reply_due = 1;
    n = station_poll_tx(&S, t, 0, out,
                        (int)(sizeof(out) / sizeof(out[0])));
    check("but a genuinely owed acknowledgment still goes out", n > 0);
    n = station_poll_tx(&S, t + 1.0, 0, out,
                        (int)(sizeof(out) / sizeof(out[0])));
    check("and only once", n == 0);
}

static void test_caps(void)
{
    static station_t A, B;
    int frames, plain, all = CAP_STREAM | CAP_EXT | CAP_LDPC | CAP_BURST;

    /* both modern: three legs, both sides hold the other's record */
    caps_case(all, all, 1, &A, &B, &frames);
    check("caps: A holds B's record", A.peer.valid && A.peer.flags == all
          && A.peer.msg_max == ST_MSG_MAX && A.peer.win_max == BURST_STREAM_MAX
          && A.peer.fw_ver == 0x0201);
    check("caps: B holds A's record", B.peer.valid && B.peer.flags == all);
    check("caps: both sides confirmed (third leg)",
          A.caps_confirmed && B.caps_confirmed);
    check("caps: neither side thinks the other is legacy",
          !A.peer.legacy && !B.peer.legacy);
    check("caps: a probe is not a loss for the ladder",
          A.ctl.consecutive_losses == 0);
    printf("  caps: %d frames on air, A tx %d, B tx %d\n", frames,
           A.stats.tx_frames, B.stats.tx_frames);
    plain = frames;

    /* CODEC declaration, and the compatibility contract around it.
     * The record GREW (10 -> 11 bytes) without moving CAPS_VER, so both
     * directions must still parse: a peer that predates the codec byte
     * must read as "never said" (0) rather than "supports none", and a
     * short record must not be refused outright. Getting this wrong is
     * silent -- the peer simply looks capability-less forever. */
    {
        static station_t A3, B3;
        g_case_codecs_a = CODEC_LSCODEC_25;
        g_case_codecs_b = CODEC_LSCODEC_25 | CODEC_CODEC2_700;
        caps_case(all, all, 1, &A3, &B3, &frames);
        g_case_codecs_a = g_case_codecs_b = 0;
        check("caps: codec bitmap crosses the link",
              A3.peer.codecs == (CODEC_LSCODEC_25 | CODEC_CODEC2_700)
              && B3.peer.codecs == CODEC_LSCODEC_25);
        check("caps: a codec the peer did not declare is visibly absent",
              !(A3.peer.codecs & CODEC_CODEC2_450));
    }
    {   /* a peer that never sets my_codecs declares 0 = "never said",
         * which callers must not read as "supports nothing" */
        static station_t A4, B4;
        caps_case(all, all, 1, &A4, &B4, &frames);
        check("caps: a silent peer reads as unspecified (0), not refused",
              A4.peer.valid && A4.peer.codecs == 0);
    }

    /* B declares no streaming: A must never try it, and never strike out */
    caps_case(all, all & ~CAP_STREAM, 1, &A, &B, &frames);
    check("caps: a peer that declares no streaming is never streamed to",
          A.peer.valid && !(A.peer.flags & CAP_STREAM)
          && A.peer_stream_ok == 0 && A.peer_stream_retry == -1
          && A.btx.streamed_n == 0);
    printf("  caps (no-stream peer): %d frames\n", frames);

    /* B narrows itself by configuration: a 4-window and a rung-9
     * ceiling, both declared in its record. A must respect BOTH without
     * being told anything out of band. */
    {
        static station_t A2, B2;
        station_phy_t pa = { 0, phy_build, phy_receive, phy_build_burst,
                             phy_receive_burst };
        station_phy_t pb = pa;
        static uint8_t big[250];
        lc_word_t lc;
        double t = 100.0;
        int k, fr;
        for (k = 0; k < 250; k++)
            big[k] = (uint8_t)(k * 31 + 5);
        station_init(&A2, &pa, 555);
        station_init(&B2, &pb, 556);
        A2.burst_window = B2.burst_window = 8;
        A2.burst_stream = B2.burst_stream = 1;
        B2.my_win_max = 4;
        B2.my_max_rung = 9;
        memset(&lc, 0, sizeof(lc));
        lc.flags = FLAG_NO_DATA;
        lc.req_rung = 12;             /* the controller WANTS the top */
        lc.snr_db = 10.0;
        ctl_on_rx_frame(&A2.ctl, 10.0, &lc, t);
        ctl_on_rx_frame(&B2.ctl, 10.0, &lc, t);
        station_submit(&A2, big, 250, QOS_BULK);
        fr = pump(&A2, &B2, &t, 3000.0, 0);
        (void)fr;
        check("caps: declared knobs arrive (win 4, rung ceiling 9)",
              A2.peer.valid && A2.peer.win_max == 4
              && A2.peer.max_rung == 9);
        /* the engaged window, from the statistic: btx.win itself is
         * dropped when the burst disengages, and asserting on it after
         * the transfer only worked while that state leaked */
        check("caps: the window respects the peer's declared ceiling",
              B2.delivered_n >= 1 && A2.stats.last_burst_win > 0
              && A2.stats.last_burst_win <= 4);
        check("caps: the tx rung never exceeds the peer's rung ceiling",
              A2.stats.last_rung <= 9 && A2.stats.last_rung > 0);
    }

    /* B predates the handshake: it never answers. A must give up after
     * CAPS_TRIES without charging the ladder, then run on the defaults
     * -- the transfer still completes, streamed (B does decode
     * streams, it just cannot say so). */
    caps_case(all, all, 0, &A, &B, &frames);
    check("caps: a legacy peer is detected after the probes go unanswered",
          !A.peer.valid && A.peer.legacy && A.caps_tries >= CAPS_TRIES);
    check("caps: the unanswered probes cost the ladder nothing",
          A.ctl.consecutive_losses == 0);
    check("caps: the legacy peer never sent a record", !B.caps_sent);
    printf("  caps (legacy peer): %d frames vs %d with the handshake\n",
           frames, plain);
}

/* ---------------- adaptive reply timer ---------------- */

/* Drive exchanges whose peer always answers a known interval after the
 * reply's air time would have ended, and check the budget converges onto
 * that interval instead of sitting at the fixed bootstrap guess. */
static void test_rto(void)
{
    static station_t C;
    static int16_t air[2000];
    static uint8_t pkt[128];
    station_phy_t phy = { 0, stub_build, 0, 0, 0 };
    static uint8_t msg[40];
    lc_word_t lc;
    uint8_t one = 0;
    double t = 100.0, boot, tuned_budget, backed;
    int k, pkt_n, exchanges = 0;
    const double OVERHEAD = 0.4;

    station_init(&C, &phy, 9);
    C.caps_disabled = 1;              /* not what this test is about */
    memset(&lc, 0, sizeof(lc));
    lc.flags = FLAG_NO_DATA;
    lc.req_rung = 11;
    lc.snr_db = 6.0;
    ctl_on_rx_frame(&C.ctl, 6.0, &lc, t);
    station_submit(&C, msg, (int)sizeof(msg), QOS_INTERACTIVE);

    /* the peer's reply: a plain no-data frame (it never acks, so the
     * station keeps giving us fresh exchanges to measure) */
    memset(&lc, 0, sizeof(lc));
    lc.flags = FLAG_NO_DATA;
    lc.req_rung = 11;
    lc.snr_db = 6.0;
    pkt_n = data_encode(lc_pack(&lc), &one, 1, pkt);

    boot = -1.0;
    for (k = 0; k < 200 && exchanges < 25; k++) {
        if (station_poll_tx(&C, t, 0, air, 2000) > 0) {
            t += 0.1;
            station_on_tx_end(&C, t);
            if (C.expects_reply) {
                double reply_at = C.tx_end_t + C.rto_air_est + OVERHEAD;
                if (boot < 0.0)
                    boot = C.await_until - C.tx_end_t - C.rto_air_est;
                station_on_decoded(&C, pkt, pkt_n, 6.0, 0.0, 0, reply_at);
                exchanges++;
                t = reply_at + 0.1;
            }
        } else {
            t += 0.5;
        }
    }

    check("rto: first budget is the bootstrap turnaround + margin",
          boot > 2.29 && boot < 2.31);
    check("rto: took round-trip samples", C.rto_have && exchanges >= 10);
    check("rto: learned overhead matches what the peer actually took",
          C.rto_srtt > OVERHEAD - 0.15 && C.rto_srtt < OVERHEAD + 0.15);
    tuned_budget = C.rto_srtt + 4.0 * C.rto_rttvar;
    check("rto: tuned budget is tighter than the bootstrap guess",
          tuned_budget < boot);

    /* a timeout must widen the timer, and Karn must poison the sample
     * that follows it. Send one more frame and simply do not answer it. */
    for (k = 0; k < 200 && C.await_until < 0.0; k++) {
        if (station_poll_tx(&C, t, 0, air, 2000) > 0) {
            t += 0.1;
            station_on_tx_end(&C, t);
        } else {
            t += 0.5;
        }
    }
    check("rto: a fresh transmission arms the timer", C.await_until > 0.0);
    t = C.await_until + 0.1;
    station_poll_tx(&C, t, 0, air, 2000);
    backed = C.rto_backoff;
    check("rto: a timeout backs the timer off and arms Karn",
          backed >= 2.0 && C.rto_ambiguous == 1);
    printf("  rto: bootstrap %.2f s -> learned %.2f s "
           "(srtt %.2f, var %.2f, %d samples), backoff x%.0f\n",
           boot, tuned_budget, C.rto_srtt, C.rto_rttvar, exchanges, backed);
}

/* ---------------- burst window sizing ---------------- */

/* The window is chosen once, at engage: cover the whole transfer if the
 * operator ceiling and the air-time cap allow it. Resizing it DURING a
 * transfer was implemented and then reverted -- on a fading channel,
 * halving the window on each timeout produced 196 transmissions and 72
 * timeouts where striking out of streaming produced 134 and 9. */
static void test_burst_window(void)
{
    static station_t C;
    static int16_t air[2000];
    static uint8_t bulk[250];
    station_phy_t phy = { 0, stub_build, 0, 0, 0 };
    lc_word_t lc;
    double t = 100.0;
    int k, small_win, big_win;

    /* a transfer shorter than the ceiling gets a window sized to it, so
     * it costs exactly one acknowledgment */
    station_init(&C, &phy, 77);
    C.caps_disabled = 1;              /* not what this test is about */
    C.burst_window = 8;
    memset(&lc, 0, sizeof(lc));
    lc.flags = FLAG_NO_DATA;
    lc.req_rung = 11;
    lc.snr_db = 6.0;
    ctl_on_rx_frame(&C.ctl, 6.0, &lc, t);
    station_submit(&C, bulk, (int)sizeof(bulk), QOS_BULK);
    for (k = 0; k < 20 && !C.btx.active; k++) {
        if (station_poll_tx(&C, t, 0, air, 2000) > 0)
            station_on_tx_end(&C, t += 0.1);
        else
            t += 0.5;
    }
    small_win = C.btx.win;
    check("window: a short transfer fits in one window",
          C.btx.active && small_win == C.btx.n && small_win <= C.burst_window);

    /* a transfer with more fragments than the ceiling is capped by it */
    station_init(&C, &phy, 78);
    C.caps_disabled = 1;
    C.burst_window = 4;
    memset(&lc, 0, sizeof(lc));
    lc.flags = FLAG_NO_DATA;
    lc.req_rung = 5; /* low rung -> 25-byte fragments -> 10 of them */
    lc.snr_db = -4.0;
    ctl_on_rx_frame(&C.ctl, -4.0, &lc, t);
    station_submit(&C, bulk, (int)sizeof(bulk), QOS_BULK);
    t = 100.0;
    for (k = 0; k < 40 && !C.btx.active; k++) {
        if (station_poll_tx(&C, t, 0, air, 2000) > 0)
            station_on_tx_end(&C, t += 0.1);
        else
            t += 0.5;
    }
    big_win = C.btx.win;
    check("window: a long transfer is capped by the operator ceiling",
          C.btx.active && C.btx.n > C.burst_window
          && big_win == C.burst_window);

    /* whatever is chosen, the burst must fit the air-time cap */
    check("window: the chosen window fits the air-time cap",
          stream_air_time_pub(C.last_tx_rung >= 0 ? C.last_tx_rung : 0,
                              C.btx.frag_size, big_win)
              <= BURST_WIN_MAX_AIR_S + 0.001);
    printf("  window: %d-fragment transfer -> %d, %d-fragment -> %d "
           "(ceilings 8 and 4)\n", 2, small_win, 10, big_win);
}

/* ------------- peer streaming capability is remembered ------------- */

/* engage a burst transfer and report the window the station chose */
static int engage_transfer(station_t *C, double *t, const uint8_t *msg,
                           int len)
{
    static int16_t air[2000];
    int k;
    station_abort_bulk(C); /* drop any transfer still in flight */
    station_submit(C, msg, len, QOS_BULK);
    for (k = 0; k < 40 && !C->btx.active; k++) {
        if (station_poll_tx(C, *t, 0, air, 2000) > 0)
            station_on_tx_end(C, *t += 0.1);
        else
            *t += 0.5;
    }
    return C->btx.active ? C->btx.stream_ok : -1;
}

static void prime_rung11(station_t *C, double t)
{
    lc_word_t lc;
    memset(&lc, 0, sizeof(lc));
    lc.flags = FLAG_NO_DATA;
    lc.req_rung = 11;
    lc.snr_db = 6.0;
    ctl_on_rx_frame(&C->ctl, 6.0, &lc, t);
}

static int stub_build_burst(void *ctx, const uint8_t *b, int pn, int nb,
                            int typ, int rung, int rs, int16_t *out,
                            int cap)
{
    (void)ctx; (void)b; (void)pn; (void)nb; (void)typ; (void)rung;
    (void)rs; (void)out; (void)cap;
    return 5000;
}

static void test_peer_stream_memory(void)
{
    static station_t C;
    station_phy_t phy = { 0, stub_build, 0, stub_build_burst, 0 };
    static uint8_t bulk[250];
    double t = 100.0;
    int first, after_noack, after_timeout, k, probed = -1;

    /* a NOACK verdict is about the PEER: the next transfer must not
     * bother streaming */
    station_init(&C, &phy, 101);
    C.burst_window = 8;
    C.burst_stream = 1;
    prime_rung11(&C, t);
    first = engage_transfer(&C, &t, bulk, (int)sizeof(bulk));
    check("peer stream: a fresh peer is assumed capable", first == 1);
    C.btx.stream_ok = 1;
    burst_stream_off_pub(&C, ST_SOFF_NOACK, t);
    after_noack = engage_transfer(&C, &t, bulk, (int)sizeof(bulk));
    check("peer stream: a NOACK verdict is remembered next transfer",
          after_noack == 0 && C.peer_stream_ok == 0);

    /* a TIMEOUT verdict is about the CHANNEL: it must NOT stick, or a
     * fade would disable streaming on a perfectly capable link */
    station_init(&C, &phy, 102);
    C.burst_window = 8;
    C.burst_stream = 1;
    prime_rung11(&C, t);
    engage_transfer(&C, &t, bulk, (int)sizeof(bulk));
    C.btx.stream_ok = 1;
    burst_stream_off_pub(&C, ST_SOFF_TIMEOUT, t);
    after_timeout = engage_transfer(&C, &t, bulk, (int)sizeof(bulk));
    check("peer stream: a TIMEOUT verdict does not stick to the peer",
          after_timeout == 1 && C.peer_stream_ok == 1);

    /* and the peer verdict expires, so a fade that forged the NOACK
     * signature costs one retry period, not the session */
    station_init(&C, &phy, 103);
    C.burst_window = 8;
    C.burst_stream = 1;
    prime_rung11(&C, t);
    engage_transfer(&C, &t, bulk, (int)sizeof(bulk));
    C.btx.stream_ok = 1;
    burst_stream_off_pub(&C, ST_SOFF_NOACK, t);
    for (k = 1; k <= PEER_STREAM_RETRY + 2; k++) {
        if (engage_transfer(&C, &t, bulk, (int)sizeof(bulk)) == 1) {
            probed = k;
            break;
        }
    }
    check("peer stream: the verdict expires and streaming is re-probed",
          probed == PEER_STREAM_RETRY);
    printf("  peer stream: NOACK sticks, TIMEOUT does not, re-probe after "
           "%d transfers\n", probed);
}

int main(void)
{
    test_lc();
    test_ctl();
    test_freq_trim();
    test_burst_forgiveness();
    test_ext_frame();
    test_session();
    test_burst_stream();
    test_caps();
    test_caps_kick_orphan();
    test_burst_outlives_message();
    test_full_store_does_not_lie();
    test_ack_on_any_frame();
    test_burst_owns_its_message();
    test_rto();
    test_burst_window();
    test_peer_stream_memory();
    check("message store: no allocation refused, no slot double-freed",
          station_pool_refused() == 0 && station_pool_double_free() == 0);
    printf("  message store peak %d of %d slots\n",
           station_pool_peak(), ST_POOL_SLOTS);

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
