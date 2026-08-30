#define _POSIX_C_SOURCE 200809L /* nanosleep */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <libusb-1.0/libusb.h>

#include "usb_host.h"

#define VID 0x1209
#define PID 0x0001
#define EP_OUT 0x01
#define EP_IN 0x81

struct usbh {
    libusb_context *ctx;
    libusb_device_handle *dev;
    char serial[64];
    int stale;
};

static int get_serial(libusb_device_handle *h, char *out, int cap)
{
    struct libusb_device_descriptor d;
    int tries;
    if (libusb_get_device_descriptor(libusb_get_device(h), &d) != 0
        || d.iSerialNumber == 0)
        return -1;
    /* The string read is a control transfer (langid fetch + the
     * descriptor) and transiently fails when the device is mid-traffic
     * or freshly enumerated -- measured as an intermittent
     * "<unreadable>" in --list while the peer console was streaming.
     * The serial is factory-constant, so a retry costs nothing and
     * cannot return a stale answer. */
    for (tries = 0; tries < 4; tries++) {
        if (libusb_get_string_descriptor_ascii(h, d.iSerialNumber,
                                               (unsigned char *)out,
                                               cap) > 0)
            return 0;
        {
            struct timespec ts = { 0, 50 * 1000 * 1000 };
            nanosleep(&ts, 0);
        }
    }
    return -1;
}

int usbh_list(void)
{
    libusb_context *ctx;
    libusb_device **list;
    ssize_t n, i;
    int count = 0;

    if (libusb_init(&ctx) != 0) {
        fprintf(stderr, "libusb init failed\n");
        return 0;
    }
    n = libusb_get_device_list(ctx, &list);
    for (i = 0; i < n; i++) {
        struct libusb_device_descriptor d;
        libusb_device_handle *h;
        char ser[64] = "<unreadable>";
        if (libusb_get_device_descriptor(list[i], &d) != 0
            || d.idVendor != VID || d.idProduct != PID)
            continue;
        if (libusb_open(list[i], &h) == 0) {
            get_serial(h, ser, sizeof(ser));
            libusb_close(h);
        } else {
            snprintf(ser, sizeof(ser), "<open failed -- udev rule?>");
        }
        printf("  bus %03d dev %03d  serial %s\n",
               libusb_get_bus_number(list[i]),
               libusb_get_device_address(list[i]), ser);
        count++;
    }
    libusb_free_device_list(list, 1);
    libusb_exit(ctx);
    if (!count)
        printf("no OFDM modem on the bus (%04x:%04x) -- cable? udev "
               "rule (host/99-ofdm-modem.rules)?\n", VID, PID);
    else
        printf("%d modem(s). Select one with --usb <serial>.\n", count);
    return count;
}

usbh_t *usbh_open(const char *serial)
{
    usbh_t *u = calloc(1, sizeof(*u));
    libusb_device **list;
    libusb_device_handle *found = 0;
    ssize_t n, i;
    int matches = 0;

    if (!u || libusb_init(&u->ctx) != 0) {
        fprintf(stderr, "libusb init failed\n");
        free(u);
        return 0;
    }
    n = libusb_get_device_list(u->ctx, &list);
    for (i = 0; i < n; i++) {
        struct libusb_device_descriptor d;
        libusb_device_handle *h;
        char ser[64] = "";
        if (libusb_get_device_descriptor(list[i], &d) != 0
            || d.idVendor != VID || d.idProduct != PID)
            continue;
        if (libusb_open(list[i], &h) != 0)
            continue;
        get_serial(h, ser, sizeof(ser));
        if (serial && strcmp(ser, serial) != 0) {
            libusb_close(h);
            continue;
        }
        matches++;
        if (!found) {
            found = h;
            snprintf(u->serial, sizeof(u->serial), "%s", ser);
        } else {
            libusb_close(h);
        }
    }
    libusb_free_device_list(list, 1);
    if (!found || (matches > 1 && !serial)) {
        fprintf(stderr, matches > 1
                ? "several modems attached -- pass --usb <serial> "
                  "(see --list)\n"
                : serial ? "no modem with serial %s\n"
                         : "no OFDM modem on the bus\n",
                serial);
        if (found)
            libusb_close(found);
        libusb_exit(u->ctx);
        free(u);
        return 0;
    }
    u->dev = found;
    if (libusb_claim_interface(u->dev, 0) != 0) {
        fprintf(stderr, "cannot claim the modem interface (udev rule? "
                        "another console attached?)\n");
        libusb_close(u->dev);
        libusb_exit(u->ctx);
        free(u);
        return 0;
    }
    libusb_clear_halt(u->dev, EP_OUT);   /* OUT only; see usb_host.h */
    {   /* drain: consume whatever the device armed for a dead session */
        unsigned char junk[512];
        int got, quiet = 0;
        while (quiet < 3) {              /* 3 x 50 ms of silence */
            if (libusb_bulk_transfer(u->dev, EP_IN, junk, sizeof(junk),
                                     &got, 50) == 0 && got > 0) {
                u->stale += got;
                quiet = 0;
            } else {
                quiet++;
            }
            if (u->stale > 65536)
                break;
        }
    }
    return u;
}

int usbh_read(usbh_t *u, void *buf, int cap, int timeout_ms)
{
    int got = 0;
    int rc = libusb_bulk_transfer(u->dev, EP_IN, buf, cap, &got,
                                  timeout_ms < 1 ? 1 : (unsigned)timeout_ms);
    if (rc == 0 || rc == LIBUSB_ERROR_TIMEOUT)
        return got;
    return -1;
}

int usbh_write(usbh_t *u, const void *buf, int n)
{
    int done = 0;
    return libusb_bulk_transfer(u->dev, EP_OUT, (unsigned char *)buf, n,
                                &done, 2000) == 0 && done == n ? n : -1;
}

const char *usbh_serial(usbh_t *u) { return u->serial; }
int usbh_stale(usbh_t *u) { return u->stale; }

void usbh_close(usbh_t *u)
{
    if (!u)
        return;
    libusb_release_interface(u->dev, 0);
    libusb_close(u->dev);
    libusb_exit(u->ctx);
    free(u);
}
