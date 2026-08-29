/* USB descriptors for the OFDM modem device.
 *
 * The point of this file is IDENTITY. A USB-serial bridge appears as
 * one more /dev/ttyACM* among however many are plugged in, with a
 * number that depends on enumeration order, so the host has to guess
 * which port is the modem and guesses wrong after a reboot. A device
 * with its own VID/PID and a per-unit serial can be opened by name:
 *
 *     libusb_open_device_with_vid_pid(ctx, OFDM_USB_VID, OFDM_USB_PID)
 *
 * and udev can give each unit a stable path (see host/99-ofdm-modem.rules)
 * regardless of what else is on the bus.
 *
 * VENDOR ID. 0x1209 is pid.codes, the shared vendor ID for open-source
 * hardware, and 0x0001 within it is explicitly the *test* product ID --
 * unallocated, for development only. It is correct here and NOT correct
 * in anything shipped: a released unit must hold a PID allocated at
 * https://pid.codes, or a vendor ID of its own. Squatting on someone
 * else's VID/PID is what makes two unrelated devices collide on a user's
 * machine, which is the exact failure this file exists to avoid.
 *
 * CLASS. Vendor-specific (0xFF) rather than CDC, so no operating system
 * binds a driver of its own and the interface is claimable directly.
 * Subclass 0x4F ('O') and protocol 0x01 identify this protocol version,
 * so a future incompatible framing can be told apart on the wire rather
 * than by trial.
 *
 * WINDOWS. Linux and macOS bind nothing to a vendor-class interface, so
 * libusb can claim it unaided. Windows needs to be told once, either by
 * an .inf pointing at WinUSB or by adding an MS OS 2.0 descriptor set.
 * That descriptor set is not here: it is a long blob that cannot be
 * verified without a Windows host to plug into, and shipping an
 * unverified one is worse than shipping none.
 */
#ifndef OFDM_USB_DESC_H
#define OFDM_USB_DESC_H

#include <stdint.h>

#define OFDM_USB_VID 0x1209   /* pid.codes */
#define OFDM_USB_PID 0x0001   /* TEST pid -- see the note above */

#define OFDM_USB_EP_OUT 0x01  /* host -> modem, bulk */
#define OFDM_USB_EP_IN  0x81  /* modem -> host, bulk */
#define OFDM_USB_EP_SIZE 64   /* full speed */

#define OFDM_USB_IF_CLASS    0xFF
#define OFDM_USB_IF_SUBCLASS 0x4F  /* 'O' */
#define OFDM_USB_IF_PROTOCOL 0x01  /* usb_proto.h framing, version 1 */

/* string descriptor indices */
#define OFDM_USB_STR_MFR    1
#define OFDM_USB_STR_PROD   2
#define OFDM_USB_STR_SERIAL 3
#define OFDM_USB_STR_IF     4

static const uint8_t OFDM_USB_DEVICE_DESC[18] = {
    18, 0x01,               /* bLength, DEVICE */
    0x00, 0x02,             /* bcdUSB 2.00 */
    OFDM_USB_IF_CLASS, 0x00, 0x00,  /* class at device level too, so the
                                     * host binds nothing before it has
                                     * read the configuration */
    64,                     /* bMaxPacketSize0 */
    (uint8_t)(OFDM_USB_VID & 0xFF), (uint8_t)(OFDM_USB_VID >> 8),
    (uint8_t)(OFDM_USB_PID & 0xFF), (uint8_t)(OFDM_USB_PID >> 8),
    0x00, 0x01,             /* bcdDevice 1.00 */
    OFDM_USB_STR_MFR, OFDM_USB_STR_PROD, OFDM_USB_STR_SERIAL,
    1                       /* bNumConfigurations */
};

#define OFDM_USB_CONFIG_LEN 32

static const uint8_t OFDM_USB_CONFIG_DESC[OFDM_USB_CONFIG_LEN] = {
    /* configuration */
    9, 0x02,
    OFDM_USB_CONFIG_LEN, 0x00,   /* wTotalLength */
    1,                           /* bNumInterfaces */
    1,                           /* bConfigurationValue */
    0,                           /* iConfiguration */
    0x80,                        /* bus powered, no remote wakeup */
    100,                         /* 200 mA */

    /* interface */
    9, 0x04,
    0,                           /* bInterfaceNumber */
    0,                           /* bAlternateSetting */
    2,                           /* bNumEndpoints */
    OFDM_USB_IF_CLASS, OFDM_USB_IF_SUBCLASS, OFDM_USB_IF_PROTOCOL,
    OFDM_USB_STR_IF,

    /* bulk OUT */
    7, 0x05, OFDM_USB_EP_OUT, 0x02,
    OFDM_USB_EP_SIZE, 0x00, 0,

    /* bulk IN */
    7, 0x05, OFDM_USB_EP_IN, 0x02,
    OFDM_USB_EP_SIZE, 0x00, 0
};

/* Serial number from the STM32 96-bit unique ID (0x1FF1E800 on an H7).
 * Per-unit and stamped at the factory, so two boards on one host are
 * always distinguishable -- which a ttyACM number is not. Writes 24 hex
 * characters plus a NUL; `out` must hold 25. */
static inline void ofdm_usb_serial(const uint8_t uid[12], char *out)
{
    static const char HEX[] = "0123456789ABCDEF";
    int i;
    for (i = 0; i < 12; i++) {
        out[2 * i] = HEX[(uid[i] >> 4) & 0xF];
        out[2 * i + 1] = HEX[uid[i] & 0xF];
    }
    out[24] = 0;
}

#endif /* OFDM_USB_DESC_H */
