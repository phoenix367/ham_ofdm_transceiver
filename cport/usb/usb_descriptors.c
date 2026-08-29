/* TinyUSB descriptor callbacks.
 *
 * The values come from usb_desc.h so that there is ONE definition of
 * this device's identity: the host driver, the udev rule and the
 * firmware cannot drift apart. TinyUSB wants them through callbacks
 * rather than as flat arrays, so this file is the adapter. */

#include "tusb.h"

#include "usb_desc.h"

/* --- device --- */
static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = OFDM_USB_IF_CLASS,   /* vendor: bind nothing */
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = OFDM_USB_VID,
    .idProduct          = OFDM_USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = OFDM_USB_STR_MFR,
    .iProduct           = OFDM_USB_STR_PROD,
    .iSerialNumber      = OFDM_USB_STR_SERIAL,
    .bNumConfigurations = 1
};

const uint8_t *tud_descriptor_device_cb(void)
{
    return (const uint8_t *)&desc_device;
}

/* --- configuration --- */
enum { ITF_VENDOR = 0, ITF_COUNT };

#define CONFIG_TOTAL_LEN (9 + 9 + 7 + 7)   /* config + interface + 2 EPs */

static const uint8_t desc_config[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    /* The interface is written out rather than built with
     * TUD_VENDOR_DESCRIPTOR, which hardcodes bInterfaceSubClass and
     * bInterfaceProtocol to zero. usb_desc.h declares 0x4F and 0x01 to
     * identify this protocol version ON THE WIRE, so that a future
     * incompatible framing can be told apart by a host before it opens
     * anything -- and a declared identity that is not actually
     * transmitted is worse than no declaration at all. lsusb showed the
     * macro's zeros, which is how this was caught. */
    9, TUSB_DESC_INTERFACE, ITF_VENDOR, 0, 2,
    OFDM_USB_IF_CLASS, OFDM_USB_IF_SUBCLASS, OFDM_USB_IF_PROTOCOL,
    OFDM_USB_STR_IF,

    7, TUSB_DESC_ENDPOINT, OFDM_USB_EP_OUT, TUSB_XFER_BULK,
       U16_TO_U8S_LE(OFDM_USB_EP_SIZE), 0,

    7, TUSB_DESC_ENDPOINT, OFDM_USB_EP_IN, TUSB_XFER_BULK,
       U16_TO_U8S_LE(OFDM_USB_EP_SIZE), 0
};

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_config;
}

/* --- strings ---
 * The serial is built at runtime from the STM32's 96-bit unique ID, so
 * every unit is distinguishable on a host that has two attached. That
 * is the property a /dev/ttyACM number does not have. */
#define UID_BASE_H7 0x1FF1E800u

static uint16_t desc_str[33];

static const char *const STR_TABLE[] = {
    [OFDM_USB_STR_MFR]  = "ofdm-transceiver-proto",
    [OFDM_USB_STR_PROD] = "OFDM Modem",
    [OFDM_USB_STR_IF]   = "OFDM modem data"
};

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    char serial[25];
    const char *s = 0;
    int n = 0, i;

    (void)langid;

    if (index == 0) {
        desc_str[1] = 0x0409;              /* en-US */
        n = 1;
    } else if (index == OFDM_USB_STR_SERIAL) {
        const uint8_t *uid = (const uint8_t *)UID_BASE_H7;
        ofdm_usb_serial(uid, serial);
        s = serial;
    } else if (index < (uint8_t)(sizeof(STR_TABLE) / sizeof(STR_TABLE[0]))) {
        s = STR_TABLE[index];
    }

    if (s) {
        for (n = 0; s[n] && n < 31; n++)
            desc_str[1 + n] = (uint16_t)s[n];
    }
    if (!s && index != 0)
        return NULL;

    /* bLength, bDescriptorType packed into the first UTF-16 unit */
    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
    (void)i;
    return desc_str;
}
