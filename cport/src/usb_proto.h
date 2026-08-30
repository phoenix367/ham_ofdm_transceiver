/* Host link protocol for the OFDM modem as a USB device.
 *
 * WHY NOT CDC-ACM. A USB-serial bridge is the obvious way to get a
 * modem onto a PC and the wrong one here: it enumerates as yet another
 * /dev/ttyACM* or /dev/ttyUSB*, indistinguishable from every other
 * adapter on the bus, so the host has to guess which port is the modem
 * -- and guesses wrong when the numbering shifts on the next boot. It
 * also reduces a packet interface to a byte stream, so framing has to
 * be reinvented on top anyway, without the reliability guarantees USB
 * already provides.
 *
 * Instead the modem is its own device class: a vendor-specific
 * interface with bulk endpoints, a fixed VID/PID, and a serial string
 * derived from the STM32's 96-bit unique ID. The host finds it by
 * identity rather than by port number, and udev can give it a stable
 * name per unit. See usb_desc.h.
 *
 * WHAT CROSSES THE LINK. The MCU runs the whole stack -- PHY, FEC, link
 * layer, ARQ -- so this carries MESSAGES and CONTROL, not audio.
 * Roughly 27 bytes per frame at up to a few frames a second: the link
 * is idle by USB standards, and its job is identity and framing rather
 * than throughput. An optional audio tap exists for debugging and is
 * off by default.
 *
 * FRAMING. Bulk transfers preserve packet boundaries only up to the
 * endpoint size, and a host may coalesce reads, so frames are
 * self-delimiting:
 *
 *     A5 5A | type:u8 | len:u16le | payload[len]     (5-byte header)
 *
 * Two sync bytes rather than one so that resynchronising after a
 * truncated transfer is cheap and unambiguous enough in practice. No
 * checksum: USB already CRCs every packet and retries, and a second
 * layer of integrity here would be answering a question the bus has
 * already answered.
 */
#ifndef OFDM_USB_PROTO_H
#define OFDM_USB_PROTO_H

#include <stdint.h>

#define UP_SYNC0 0xA5u
#define UP_SYNC1 0x5Au
#define UP_HDR_LEN 5   /* sync0 sync1 type len_lo len_hi */
/* One decoded message plus its sub-header; also the largest thing the
 * link layer will hand up in one piece. */
#ifndef UP_MAX_PAYLOAD
#define UP_MAX_PAYLOAD 1024
#endif
#define UP_MAX_FRAME (UP_HDR_LEN + UP_MAX_PAYLOAD)

/* host -> device */
enum {
    UP_CMD_INFO    = 0x01, /* -- */
    UP_CMD_SUBMIT  = 0x02, /* qos:u8, data[] */
    UP_CMD_CONFIG  = 0x03, /* key:u8, value:i32le */
    UP_CMD_PING    = 0x04, /* token:u32le */
    UP_CMD_RESET   = 0x05, /* -- : re-init the station, keep the link up */
    UP_CMD_BCAST   = 0x06  /* ptype:u8, rung:u8 (0xFF = link's last rung),
                            * data[] -- start a NON-ARQ broadcast. One
                            * command is one broadcast; nothing is ever
                            * retransmitted. */
};

/* device -> host */
enum {
    UP_RSP_INFO    = 0x81, /* see up_info_t */
    UP_EVT_MESSAGE = 0x82, /* qos:u8, data[] */
    UP_EVT_STATUS  = 0x83, /* see up_status_t */
    UP_EVT_DIAG    = 0x84, /* ev:u8, a..d:i32le, t_ms:u32le */
    UP_RSP_PONG    = 0x85, /* token:u32le, echoed */
    UP_EVT_LOG     = 0x86, /* utf-8 text, no terminator */
    UP_EVT_AUDIO   = 0x87, /* int16le samples, debug tap, off by default */
    UP_EVT_BCAST   = 0x88  /* received broadcast. flags:u8 (bit7 = start,
                            * low nibble then carries the ptype; bit6 =
                            * EOS, payload is stats: frames_ok:u16le,
                            * frames_lost:u16le, snr_q8:i16le), else
                            * payload = reassembled data bytes as they
                            * decode -- STREAMED, the host stores or
                            * prints them; gaps are never repaired. */
};

/* UP_CMD_CONFIG keys */
enum {
    UP_CFG_RUNG_CEILING = 1, /* operator cap on the rate ladder */
    UP_CFG_BURST_WINDOW = 2,
    UP_CFG_BURST_STREAM = 3, /* 0/1 */
    UP_CFG_FREQ_TRIM_MHZ= 4, /* millihertz, signed */
    UP_CFG_AUDIO_TAP    = 5, /* 0 = off, else decimation factor */
    UP_CFG_ANCHOR       = 6, /* 0/1, AFC frequency reference */
    UP_CFG_DIAG_STREAM  = 7  /* 0/1, default OFF -- see usb_modem.c */
};

/* --- payload layouts, all little-endian, all fixed size ------------- */

typedef struct {
    uint8_t  proto_ver;      /* this protocol's version, 1 */
    uint8_t  n_modes;        /* 3 */
    uint16_t fw_ver;         /* (major<<8)|minor */
    uint8_t  uid[12];        /* STM32 96-bit unique ID, as read */
    uint32_t caps;           /* UP_CAP_* */
    uint32_t sample_rate;    /* 12000 */
} up_info_t;

#define UP_CAP_LDPC       (1u << 0)
#define UP_CAP_EXT_FRAMES (1u << 1)
#define UP_CAP_BURST      (1u << 2)
#define UP_CAP_AUDIO_TAP  (1u << 3)
#define UP_CAP_BCAST      (1u << 4)

typedef struct {
    int32_t  rung;           /* current tx rung, -1 = none yet */
    int32_t  snr_q8;         /* last SNR, dB in Q8 */
    uint32_t tx_frames, rx_frames, timeouts, retransmissions;
    uint16_t q_ctl, q_inter, q_bulk;  /* queue depths */
    uint8_t  busy;           /* carrier sense */
    uint8_t  pending;        /* a frame awaiting acknowledgement */
} up_status_t;

/* --- encoding ------------------------------------------------------- */

/* Write one frame into out (>= UP_HDR_LEN + len). Returns bytes
 * written, or -1 if it would not fit UP_MAX_FRAME. */
int up_encode(uint8_t type, const void *payload, int len,
              uint8_t *out, int out_cap);

int up_encode_info(const up_info_t *info, uint8_t *out, int out_cap);
int up_encode_status(const up_status_t *st, uint8_t *out, int out_cap);
int up_encode_diag(int ev, int a, int b, int c, int d, uint32_t t_ms,
                   uint8_t *out, int out_cap);

/* --- decoding ------------------------------------------------------- */

/* Streaming parser: bytes arrive in whatever chunks the bus delivers,
 * frames come out whole. One instance per direction. */
typedef struct {
    uint8_t buf[UP_MAX_FRAME];
    int have;                /* bytes in buf */
    int want;                /* full frame length once the header is in */
    uint32_t resyncs;        /* frames dropped to a bad sync -- 0 on a
                              * healthy link; a rising count means the
                              * stream lost alignment */
} up_parser_t;

void up_parser_init(up_parser_t *p);

/* Feed n bytes; cb is called once per complete frame. Returns the
 * number of frames delivered. */
int up_parser_push(up_parser_t *p, const uint8_t *data, int n,
                   void (*cb)(void *ctx, uint8_t type,
                              const uint8_t *payload, int len),
                   void *ctx);

/* Fixed-layout payload readers; return 0 on success, -1 on a short or
 * malformed payload. */
int up_decode_info(const uint8_t *payload, int len, up_info_t *out);
int up_decode_status(const uint8_t *payload, int len, up_status_t *out);

#endif /* OFDM_USB_PROTO_H */
