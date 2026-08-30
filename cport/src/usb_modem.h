/* The modem as a USB device: binds the host protocol (usb_proto.h) to a
 * station (station.h).
 *
 * Transport-agnostic on purpose. It never touches a USB register: bytes
 * arrive through usb_modem_rx() and leave through usb_modem_poll(),
 * so the same object runs on the OTG peripheral, on a TinyUSB callback,
 * or on a pipe -- which is how it is tested (bench/usb_modem_emu.c)
 * without a bus in the room. The USB peripheral driver's whole job is to
 * move bytes between two bulk endpoints and these two calls. */
#ifndef OFDM_USB_MODEM_H
#define OFDM_USB_MODEM_H

#include <stdint.h>

#include "station.h"
#include "usb_proto.h"

#ifndef UM_TXQ
#define UM_TXQ 4096          /* device -> host staging */
#endif

typedef struct {
    station_t *st;
    up_parser_t parser;
    uint8_t txq[UM_TXQ];
    int txq_head, txq_len;   /* ring */
    uint32_t dropped;        /* frames LOST: staging ring was full */
    uint32_t diag_suppressed;/* diag events not sent because the stream
                              * is off, or shed under backpressure --
                              * NOT a loss. Kept separate because a
                              * single 'dropped' counter that mixed the
                              * two read as data loss during a wedge
                              * diagnosis when nothing had been lost. */
    up_info_t info;
    int delivered_seen;      /* how much of the delivered log we have sent */
    double now;              /* protocol time, set by the caller */
    int diag_on;             /* UP_CFG_DIAG_STREAM, default 0 */
    /* UP_CMD_BCAST lands here; the firmware owns the broadcast engine
     * (it is not station traffic -- nothing is acknowledged). NULL =
     * command ignored. */
    void (*bcast_cb)(void *ctx, int ptype, int rung, const uint8_t *data,
                     int len);
    void *bcast_ctx;
    /* commands received from the host. Non-zero means a host PROGRAM has
     * attached, which is more than the cable being plugged in: the
     * console announces itself with UP_CMD_INFO. Clear it on unmount. */
    uint32_t host_cmds;
} usb_modem_t;

void usb_modem_init(usb_modem_t *m, station_t *st, const uint8_t uid[12],
                    uint16_t fw_ver, uint32_t caps);

/* bytes from the OUT endpoint */
void usb_modem_rx(usb_modem_t *m, const uint8_t *data, int n);

/* Drain up to cap bytes for the IN endpoint; returns the count. Call
 * usb_modem_tick() first so pending events are staged. */
int usb_modem_poll(usb_modem_t *m, uint8_t *out, int cap);

/* Stage whatever the station has produced since the last call: delivered
 * messages, and a status frame if `status` is nonzero. */
void usb_modem_tick(usb_modem_t *m, double now, int status);

/* Emit one frame to the host (queued behind whatever is pending).
 * For firmware-level events that are not the station's business --
 * broadcast reception, for instance. */
void usb_modem_emit(usb_modem_t *m, uint8_t type, const void *payload,
                    int len);

/* Station diagnostic callback -- register with station_set_diag so the
 * event stream reaches the host. */
void usb_modem_diag(void *ctx, int ev, int a, int b, int c, int d,
                    double t);

#endif /* OFDM_USB_MODEM_H */
