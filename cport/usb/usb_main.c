/* RAM-resident USB bring-up: does the modem enumerate?
 *
 * Loaded over JTAG and run from RAM, exactly like bench/armbench.c --
 * the board's flash is never written, so the unit keeps whatever
 * firmware it had and a reset undoes everything here. Flashing is a
 * separate decision, taken once this has been seen to work.
 *
 * TinyUSB is driven by POLLING rather than by interrupt:
 * tud_int_handler() is called from the loop instead of from an ISR.
 * That is not a shortcut, it is the point -- a RAM-resident image has
 * no vector table, and installing one means writing VTOR, which is a
 * thing this project has already lost an afternoon to. Polling removes
 * the whole question from the first bring-up; the interrupt path
 * (ofdm_usb_bsp_irq_enable) is there for the flashed build.
 *
 * Progress is reported through a struct at a fixed DTCM address, the
 * same mechanism the cycle benchmark uses, so the host can read how far
 * enumeration got even if it never completes.
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
void ofdm_usb_bsp_irq_disable(int rhport);
int  ofdm_usb_bsp_supply_ready(void);
int  ofdm_usb_bsp_used_regulator(void);

/* --- progress beacon, read back over JTAG --------------------------- */
#define USB_BEACON_MAGIC 0x05BEAC01u

typedef struct {
    uint32_t magic;
    uint32_t stage;        /* how far bring-up got; see below */
    uint32_t mounted;      /* host completed enumeration */
    uint32_t rx_bytes, tx_bytes, frames_in, frames_out;
    uint32_t loops;
    uint32_t suspended, resumed;
    uint32_t supply_waits;   /* loops spent waiting for VDD33USB */
    uint32_t used_regulator; /* 1 = VDD33USB came from VBUS */
} usb_beacon_t;

enum {
    ST_ENTER = 1, ST_WAIT_SUPPLY, ST_BSP_DONE, ST_TUSB_INIT, ST_LOOPING,
    ST_MOUNTED
};

usb_beacon_t g_beacon __attribute__((section(".results"), used));

/* --- TinyUSB's time hooks. No SysTick here: the loop is the clock, and
 * nothing in a device-only vendor-class configuration depends on real
 * milliseconds. --- */
static volatile uint32_t g_ticks;

uint32_t tusb_time_millis_api(void)
{
    return g_ticks / 1000u;      /* coarse; only used for timeouts */
}

void tusb_time_delay_ms_api(uint32_t ms)
{
    volatile uint32_t n = ms * 10000u;
    while (n--)
        ;
}

/* --- device callbacks ---------------------------------------------- */
void tud_mount_cb(void)   { g_beacon.mounted = 1; g_beacon.stage = ST_MOUNTED; }
void tud_umount_cb(void)  { g_beacon.mounted = 0; }
void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; g_beacon.suspended++; }
void tud_resume_cb(void)  { g_beacon.resumed++; }

/* --- a real station behind the endpoints -------------------------- *
 *
 * The bring-up image answered the protocol itself, so that USB could
 * not fail for station-shaped reasons. It has, so this now runs the
 * actual link layer: usb_modem.c binds the protocol to a station_t, and
 * the host can queue messages, read status and watch the diagnostic
 * event stream over the bus.
 *
 * What is NOT here is a radio. There is no codec on this board, so the
 * PHY reports how long a frame WOULD take and transmits nothing --
 * air-time accounting and the rate ladder run for real, the samples go
 * nowhere, and the receive path always reports "nothing heard". That is
 * the honest shape of a modem with its antenna disconnected, and it
 * exercises everything the USB link is responsible for. */

static station_t g_st;
static usb_modem_t g_modem;
static int16_t g_air[4096];      /* never transmitted; see above */

static int phy_build(void *ctx, const uint8_t *bits, int n, int typ,
                     int rung, int16_t *out, int cap)
{
    int len;
    (void)ctx;
    (void)out;                   /* no radio: nothing is written */
    len = tx_frame_len_ex(ladder_mode(rung), n, ladder_mod(rung),
                          ladder_spd(rung), 0);
    if (len <= 0)
        return 0;
    (void)typ;
    return len < cap ? len : cap;   /* clamped: the caller only times it */
}

static int phy_recv(void *ctx, const int16_t *s, int n, uint8_t *bits,
                    int *nb, double *snr, double *cfo, int *harq,
                    const int64_t *pl, int pn, int64_t *lo, int *ln)
{
    (void)ctx; (void)s; (void)n; (void)bits; (void)nb; (void)snr;
    (void)cfo; (void)harq; (void)pl; (void)pn; (void)lo; (void)ln;
    return -1;                   /* nothing heard, ever */
}

/* --- a clock -------------------------------------------------------
 * The station needs real seconds for its timers. There is no SysTick
 * here, so DWT's cycle counter is the clock: it is already enabled for
 * the benchmarks, free to read, and exact. It is 32 bits and wraps
 * every ~10.7 s at 400 MHz, which the loop is far too fast to miss --
 * so wraps are accumulated rather than ignored. */
#define DEMCR    (*(volatile uint32_t *)0xE000EDFCu)
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYC  (*(volatile uint32_t *)0xE0001004u)

#ifndef OFDM_CPU_HZ
#define OFDM_CPU_HZ 400000000.0
#endif

static uint32_t s_last_cyc;
static uint64_t s_cycles;

static void clock_init(void)
{
    DEMCR |= (1u << 24);         /* TRCENA */
    DWT_CYC = 0;
    DWT_CTRL |= 1u;              /* CYCCNTENA */
    s_last_cyc = DWT_CYC;
}

static double now_s(void)
{
    uint32_t c = DWT_CYC;
    s_cycles += (uint32_t)(c - s_last_cyc);   /* wraps correctly */
    s_last_cyc = c;
    return (double)s_cycles / OFDM_CPU_HZ;
}

int main(void)
{
    static uint8_t in[64], out[64];
    station_phy_t phy;
    double t, last_status = 0.0, tx_until = 0.0;
    int tx_busy = 0, slow_div = 0, run_station = 0;
    static const uint8_t *const UID = (const uint8_t *)0x1FF1E800u;

    memset((void *)&g_beacon, 0, sizeof(g_beacon));
    g_beacon.magic = USB_BEACON_MAGIC;
    g_beacon.stage = ST_ENTER;

    g_beacon.stage = ST_WAIT_SUPPLY;
    while (!ofdm_usb_bsp_supply_ready())
        g_beacon.supply_waits++;

    g_beacon.used_regulator = (uint32_t)ofdm_usb_bsp_used_regulator();
    ofdm_usb_bsp_init(OFDM_USB_RHPORT);
    clock_init();
    g_beacon.stage = ST_BSP_DONE;

    memset(&phy, 0, sizeof(phy));
    phy.build = phy_build;
    phy.receive = phy_recv;
    station_init(&g_st, &phy, 0x5EEDu);
    usb_modem_init(&g_modem, &g_st, UID, 0x0104,
                   UP_CAP_LDPC | UP_CAP_BURST | UP_CAP_EXT_FRAMES);
    g_st.diag_cb = usb_modem_diag;      /* the event stream goes to USB */
    g_st.diag_ctx = &g_modem;

    tusb_init();
    ofdm_usb_bsp_irq_disable(OFDM_USB_RHPORT);
    __asm__ volatile("cpsid i");
    g_beacon.stage = ST_TUSB_INIT;

    for (;;) {
        int n;
        g_beacon.loops++;
        tud_int_handler(OFDM_USB_RHPORT);
        tud_task();
        if (g_beacon.stage < ST_LOOPING)
            g_beacon.stage = ST_LOOPING;

        /* Read the clock and run the station at a BOUNDED rate.
         *
         * Everything else in this loop is USB servicing, and with
         * interrupts masked tud_int_handler() only runs as often as the
         * loop does. Calling station_poll_tx() every iteration dropped
         * the loop from 653 M iterations to 6.6 M -- a hundredfold --
         * and the device then stopped delivering IN transfers after
         * filling its 512-byte endpoint buffer. The link layer's timers
         * work in milliseconds; the USB core does not. */
        if (++slow_div >= 256) {
            slow_div = 0;
            t = now_s();
            run_station = 1;
        } else {
            run_station = 0;
        }

        if (tud_vendor_available()) {
            uint32_t k = tud_vendor_read(in, sizeof(in));
            g_beacon.rx_bytes += k;
            usb_modem_rx(&g_modem, in, (int)k);
        }

        /* Let the station run. A transmission has to be ENDED as well as
         * started: without station_on_tx_end the station believes it is
         * still on the air and never moves on. There is no radio, so the
         * air time is simulated from the sample count the PHY reported. */
        if (run_station) {
            if (!tx_busy) {
                int air = station_poll_tx(&g_st, t, 0, g_air,
                                          (int)(sizeof(g_air)
                                                / sizeof(g_air[0])));
                if (air > 0) {
                    tx_busy = 1;
                    tx_until = t + (double)air / 12000.0;
                }
            } else if (t >= tx_until) {
                station_on_tx_end(&g_st, t);
                tx_busy = 0;
            }

            usb_modem_tick(&g_modem, t, t - last_status >= 0.5);
            if (t - last_status >= 0.5)
                last_status = t;
        }

        /* Ask how much the endpoint can take BEFORE taking it out of the
         * queue. usb_modem_poll() removes what it returns, so checking
         * afterwards and breaking -- which is what this did -- discards
         * those bytes, and discarding part of a frame desynchronises the
         * host's parser. Measured as a device that stopped transmitting
         * at 547 bytes while its loop counter kept climbing. */
        for (;;) {
            int room = (int)tud_vendor_write_available();
            if (room <= 0)
                break;
            if (room > (int)sizeof(out))
                room = (int)sizeof(out);
            n = usb_modem_poll(&g_modem, out, room);
            if (n <= 0)
                break;
            tud_vendor_write(out, (uint32_t)n);
            tud_vendor_write_flush();
            g_beacon.tx_bytes += (uint32_t)n;
            g_beacon.frames_out++;
        }
    }
}
