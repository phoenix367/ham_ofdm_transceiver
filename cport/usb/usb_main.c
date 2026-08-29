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

/* --- the modem, minus the station ----------------------------------
 * This image answers the protocol itself rather than driving a station:
 * the question it exists to settle is whether the device enumerates and
 * survives a round trip, and a station would only add ways for that to
 * fail for unrelated reasons. usb_modem.c binds the two together once
 * this passes. */
static up_parser_t g_parser;
static uint8_t g_out[UP_MAX_FRAME];

static void on_frame(void *ctx, uint8_t type, const uint8_t *p, int len)
{
    int n = 0;
    (void)ctx;
    g_beacon.frames_in++;
    switch (type) {
    case UP_CMD_INFO: {
        up_info_t info;
        memset(&info, 0, sizeof(info));
        info.proto_ver = 1;
        info.n_modes = 3;
        info.fw_ver = 0x0104;
        memcpy(info.uid, (const void *)0x1FF1E800u, 12);
        info.caps = UP_CAP_LDPC | UP_CAP_BURST;
        info.sample_rate = 12000;
        n = up_encode_info(&info, g_out, (int)sizeof(g_out));
        break;
    }
    case UP_CMD_PING:
        if (len >= 4)
            n = up_encode(UP_RSP_PONG, p, 4, g_out, (int)sizeof(g_out));
        break;
    default:
        n = up_encode(UP_EVT_LOG, "usb bring-up image", 18,
                      g_out, (int)sizeof(g_out));
        break;
    }
    if (n > 0 && tud_vendor_write_available() >= (uint32_t)n) {
        tud_vendor_write(g_out, (uint32_t)n);
        tud_vendor_write_flush();
        g_beacon.tx_bytes += (uint32_t)n;
        g_beacon.frames_out++;
    }
}

int main(void)
{
    static uint8_t in[64];

    memset((void *)&g_beacon, 0, sizeof(g_beacon));
    g_beacon.magic = USB_BEACON_MAGIC;
    g_beacon.stage = ST_ENTER;

    /* Wait for VDD33USB, visibly. No cable means no VBUS means no USB
     * supply, and the honest thing is to say so and keep looking rather
     * than to hang -- which also makes the device start on its own the
     * moment the cable goes in. */
    g_beacon.stage = ST_WAIT_SUPPLY;
    while (!ofdm_usb_bsp_supply_ready())
        g_beacon.supply_waits++;

    g_beacon.used_regulator = (uint32_t)ofdm_usb_bsp_used_regulator();
    ofdm_usb_bsp_init(OFDM_USB_RHPORT);
    g_beacon.stage = ST_BSP_DONE;

    up_parser_init(&g_parser);
    tusb_init();
    /* tusb_init() enabled the OTG line in the NVIC. With no vector
     * table installed, the first interrupt would vector into whatever
     * VTOR still points at -- so mask it and drive the core from the
     * loop instead. Interrupts are also masked globally, because
     * nothing else in this image wants one either. */
    ofdm_usb_bsp_irq_disable(OFDM_USB_RHPORT);
    __asm__ volatile("cpsid i");
    g_beacon.stage = ST_TUSB_INIT;

    for (;;) {
        g_ticks++;
        g_beacon.loops++;
        tud_int_handler(OFDM_USB_RHPORT);   /* polled, see the note above */
        tud_task();
        if (g_beacon.stage < ST_LOOPING)
            g_beacon.stage = ST_LOOPING;
        if (tud_vendor_available()) {
            uint32_t k = tud_vendor_read(in, sizeof(in));
            g_beacon.rx_bytes += k;
            up_parser_push(&g_parser, in, (int)k, on_frame, 0);
        }
    }
}
