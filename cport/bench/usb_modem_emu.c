/* The modem device, minus the USB peripheral.
 *
 * Speaks the exact wire protocol of usb_proto.h on stdin/stdout instead
 * of on two bulk endpoints. That makes the host driver testable against
 * the real device-side code with no hardware attached -- and, when
 * hardware does exist, means a protocol bug can be reproduced without
 * it. The only thing this does not exercise is the USB peripheral
 * driver itself, which is the one part that cannot be tested off-target
 * anyway.
 *
 *     ./usb_modem_emu            # framed protocol on stdin/stdout
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "usb_modem.h"
#include "usb_desc.h"

static station_t g_st;
static usb_modem_t g_m;

/* A station needs a PHY. This one transmits into the void and never
 * receives, which is all the protocol layer requires: what is being
 * tested here is the host link, not the radio. */
static int phy_build(void *c, const uint8_t *b, int n, int typ, int rung,
                     int16_t *out, int cap)
{ (void)c;(void)b;(void)typ;(void)rung;(void)out;(void)cap; return n > 0 ? 64 : 0; }
static int phy_recv(void *c, const int16_t *s, int n, uint8_t *bits,
                    int *nb, double *snr, double *cfo, int *harq,
                    const int64_t *pl, int pn, int64_t *lo, int *ln)
{ (void)c;(void)s;(void)n;(void)bits;(void)nb;(void)snr;(void)cfo;
  (void)harq;(void)pl;(void)pn;(void)lo;(void)ln; return -1; }

int main(void)
{
    uint8_t in[512], out[512];
    /* the unique ID this session read off the part over JTAG */
    static const uint8_t UID[12] = { 0x24,0x00,0x41,0x00, 0x05,0x51,0x33,0x34,
                                     0x38,0x36,0x34,0x36 };
    station_phy_t phy;
    double t = 0.0;
    int ticks = 0;

    memset(&phy, 0, sizeof(phy));
    phy.build = phy_build;
    phy.receive = phy_recv;
    station_init(&g_st, &phy, 1);
    usb_modem_init(&g_m, &g_st, UID, 0x0104,
                   UP_CAP_LDPC | UP_CAP_BURST | UP_CAP_EXT_FRAMES);
    g_st.diag_cb = usb_modem_diag;
    g_st.diag_ctx = &g_m;

    setvbuf(stdout, 0, _IONBF, 0);
    for (;;) {
        ssize_t k = read(0, in, sizeof(in));
        int n;
        if (k <= 0)
            break;
        usb_modem_rx(&g_m, in, (int)k);
        usb_modem_tick(&g_m, t, (++ticks % 4) == 0);
        t += 0.05;
        while ((n = usb_modem_poll(&g_m, out, (int)sizeof(out))) > 0)
            if (write(1, out, (size_t)n) != n)
                return 1;
    }
    return 0;
}
