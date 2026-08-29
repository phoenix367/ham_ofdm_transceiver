/* Flash-resident USB modem. Cold-boots from 0x08000000 (startup_flash.c
 * brought up the 400 MHz clock, FPU and flash vector table), enumerates
 * as the OFDM modem, and runs a real station behind the endpoints. This
 * is what "the firmware survives reset" means: power the board and it is
 * a USB device, no debugger involved.
 *
 * Two differences from the RAM bring-up image (usb_main.c):
 *   - the station clock is SysTick, not DWT. DWT's cycle counter is not
 *     clocked without a debugger attached, so a standalone image that
 *     reads it stalls the bus (proven with the heartbeat).
 *   - interrupts vector through the FLASH table: OTG_FS_Handler and
 *     SysTick_Handler here are the strong definitions the startup left
 *     weak. No RAM vector table, no VTOR games.
 */

#include <stdint.h>
#include <string.h>

#include "tusb.h"
#include "usb_proto.h"
#include "usb_desc.h"
#include "usb_modem.h"
#include "station.h"
#include "tx.h"

void ofdm_usb_bsp_init(int rhport);
int  ofdm_usb_bsp_supply_ready(void);

/* --- beacon at 0x24000000 (AXI, via .results in .bss) --------------- */
#define BEACON_MAGIC 0x05BF1A54u
typedef struct {
    uint32_t magic, stage, mounted, ms, rx_bytes, tx_bytes, isr_count;
} beacon_t;
volatile beacon_t g_beacon __attribute__((section(".results"), used));
enum { ST_ENTER = 1, ST_SUPPLY, ST_TUSB, ST_LOOP, ST_MOUNTED };

/* --- SysTick millisecond clock -------------------------------------- */
#define SYST_CSR  (*(volatile uint32_t *)0xE000E010u)
#define SYST_RVR  (*(volatile uint32_t *)0xE000E014u)
#define SYST_CVR  (*(volatile uint32_t *)0xE000E018u)
#define NVIC_ISER ((volatile uint32_t *)0xE000E100u)
#define NVIC_IPR  ((volatile uint8_t  *)0xE000E400u)
#define IRQ_OTG_FS 101

static volatile uint32_t g_ms;

void SysTick_Handler(void)   /* strong: overrides the startup weak */
{
    g_ms++;
}

void OTG_FS_Handler(void)    /* strong: the USB core's interrupt */
{
    g_beacon.isr_count++;
    tud_int_handler(0);
}

static double now_s(void)
{
    return (double)g_ms * 0.001;
}

/* --- station with a stub PHY (no radio on this board) --------------- */
static station_t g_st;
static usb_modem_t g_modem;
static int16_t g_air[4096];

static int phy_build(void *c, const uint8_t *b, int n, int typ, int rung,
                     int16_t *out, int cap)
{
    int len;
    (void)c; (void)b; (void)typ; (void)out;
    len = tx_frame_len_ex(ladder_mode(rung), n, ladder_mod(rung),
                          ladder_spd(rung), 0);
    return len <= 0 ? 0 : (len < cap ? len : cap);
}
static int phy_recv(void *c, const int16_t *s, int n, uint8_t *bits, int *nb,
                    double *snr, double *cfo, int *harq, const int64_t *pl,
                    int pn, int64_t *lo, int *ln)
{
    (void)c;(void)s;(void)n;(void)bits;(void)nb;(void)snr;(void)cfo;
    (void)harq;(void)pl;(void)pn;(void)lo;(void)ln; return -1;
}

int main(void)
{
    static uint8_t in[64], out[64];
    static const uint8_t *const UID = (const uint8_t *)0x1FF1E800u;
    station_phy_t phy;
    double t, last_status = 0.0, tx_until = 0.0;
    int tx_busy = 0, slow = 0;

    memset((void *)&g_beacon, 0, sizeof(g_beacon));
    g_beacon.magic = BEACON_MAGIC;
    g_beacon.stage = ST_ENTER;

    /* SysTick: 1 ms at 400 MHz, interrupt on. Clocked from the core, so
     * it runs standalone (unlike DWT). */
    SYST_RVR = 400000u - 1u;
    SYST_CVR = 0;
    SYST_CSR = 7u;                        /* CLKSOURCE=core, TICKINT, EN */

    g_beacon.stage = ST_SUPPLY;
    while (!ofdm_usb_bsp_supply_ready())
        ;
    ofdm_usb_bsp_init(0);

    memset(&phy, 0, sizeof(phy));
    phy.build = phy_build;
    phy.receive = phy_recv;
    station_init(&g_st, &phy, 0x5EEDu);
    usb_modem_init(&g_modem, &g_st, UID, 0x0104,
                   UP_CAP_LDPC | UP_CAP_BURST | UP_CAP_EXT_FRAMES);
    g_st.diag_cb = usb_modem_diag;
    g_st.diag_ctx = &g_modem;

    tusb_init();
    NVIC_IPR[IRQ_OTG_FS] = 0x80;
    NVIC_ISER[IRQ_OTG_FS >> 5] = 1u << (IRQ_OTG_FS & 31);
    g_beacon.stage = ST_TUSB;

    for (;;) {
        int n;
        tud_task();
        g_beacon.stage = tud_mounted() ? ST_MOUNTED : ST_LOOP;
        g_beacon.mounted = tud_mounted() ? 1u : 0u;
        g_beacon.ms = g_ms;

        if (tud_vendor_available()) {
            uint32_t k = tud_vendor_read(in, sizeof(in));
            g_beacon.rx_bytes += k;
            usb_modem_rx(&g_modem, in, (int)k);
        }

        /* run the station at ~1 kHz, not every loop */
        if ((uint32_t)(g_ms - slow) >= 1u) {
            slow = (int)g_ms;
            t = now_s();
            if (!tx_busy) {
                int air = station_poll_tx(&g_st, t, 0, g_air,
                                          (int)(sizeof(g_air)/sizeof(g_air[0])));
                if (air > 0) { tx_busy = 1; tx_until = t + (double)air/12000.0; }
            } else if (t >= tx_until) {
                station_on_tx_end(&g_st, t); tx_busy = 0;
            }
            usb_modem_tick(&g_modem, t, t - last_status >= 0.5);
            if (t - last_status >= 0.5) last_status = t;
        }

        while (tud_vendor_write_available() > 0 &&
               (n = usb_modem_poll(&g_modem, out,
                     (int)(tud_vendor_write_available() < sizeof(out)
                           ? tud_vendor_write_available() : sizeof(out)))) > 0) {
            tud_vendor_write(out, (uint32_t)n);
            tud_vendor_write_flush();
            g_beacon.tx_bytes += (uint32_t)n;
        }
    }
}
