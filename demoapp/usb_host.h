#ifndef OFDM_USB_HOST_H
#define OFDM_USB_HOST_H

/* Minimal libusb wrapper for the OFDM USB modem (cport/src/usb_desc.h:
 * VID 0x1209 PID 0x0001, vendor class, bulk 0x01 OUT / 0x81 IN, serial
 * = the STM32's 96-bit UID). Mirrors host/ofdm_modem.py's transport,
 * including its hardest-won rule: NEVER clear_halt the IN endpoint --
 * the device pushes status unprompted so EP_IN is usually ARMED at
 * open, and TinyUSB's clear-stall desynchronises its software state
 * from the hardware (one packet escapes, then the endpoint wedges).
 * Drain instead: read until quiet, discard. */

typedef struct usbh usbh_t;

/* Print every attached modem (bus/addr/serial); returns the count. */
int usbh_list(void);

/* Open by serial, or the only attached modem when serial is NULL.
 * Prints its own errors; returns NULL on failure. */
usbh_t *usbh_open(const char *serial);

int usbh_read(usbh_t *u, void *buf, int cap, int timeout_ms); /* 0 = quiet */
int usbh_write(usbh_t *u, const void *buf, int n);
const char *usbh_serial(usbh_t *u);
int usbh_stale(usbh_t *u);           /* bytes drained at open */
void usbh_close(usbh_t *u);

#endif
