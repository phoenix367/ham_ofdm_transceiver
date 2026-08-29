/* ESP32 as an OpenOCD remote_bitbang probe -- JTAG and SWD.
 *
 * OpenOCD's remote_bitbang driver speaks a one-character-per-edge ASCII
 * protocol over a socket. This implements the server side on an ESP32
 * and bit-bangs the pins, which turns a bare devkit into a debug probe
 * for any ARM target -- here an STM32H7, whose flash OpenOCD already
 * knows how to program (stm32h7x).
 *
 * Protocol (doc/manual/jtag/drivers/remote_bitbang.txt, OpenOCD master):
 *
 *   B b      blink on / off
 *   0..7     JTAG write, bit2=TCK bit1=TMS bit0=TDI
 *   R        sample TDO            -> replies '0' or '1'
 *   r s t u  reset, (trst,srst) = (0,0) (0,1) (1,0) (1,1)
 *   O o      SWDIO output enable / release to input
 *   c        sample SWDIO          -> replies '0' or '1'
 *   d e f g  SWD write, (SWCLK,SWDIO) = (0,0) (0,1) (1,0) (1,1)
 *   Z z      sleep 1 ms / 1 us     (only if use_remote_sleep is on)
 *   Q        quit
 *
 * The JTAG half works with the OpenOCD in Ubuntu (0.12). The SWD half
 * needs OpenOCD built from master -- 0.12's remote_bitbang is JTAG only,
 * which `transport select swd` will tell you plainly.
 *
 * Transport to the host is UART0, i.e. the same USB cable that programs
 * this ESP32, bridged to a TCP socket by rbb_bridge.py. Serial rather
 * than WiFi on purpose: every R and c is a round trip that OpenOCD
 * blocks on, so latency matters far more than bandwidth.
 */

/* --- pin map: all < 32 so one w1ts/w1tc register pair covers them, and
 * none of them are strapping (0,2,5,12,15) or flash (6..11) pins ---- */
#define PIN_TCK    18   /* SWCLK */
#define PIN_TMS    19   /* SWDIO */
#define PIN_TDI    21
#define PIN_TDO    22   /* input */
#define PIN_TRST   23
#define PIN_SRST   25   /* to STM32 NRST, open-drain */
#define PIN_LED     2   /* devkit LED, driven by B/b */

#define BAUD 921600

/* Direct register writes rather than digitalWrite(): one store instead
 * of a call through the HAL, which matters when every protocol
 * character is one pin edge. The *_REG macros are stable across IDF
 * versions; the `GPIO` struct's field names are not. */
#include <soc/gpio_reg.h>
#include <soc/soc.h>

static inline void pin_hi(int p) { REG_WRITE(GPIO_OUT_W1TS_REG, 1u << p); }
static inline void pin_lo(int p) { REG_WRITE(GPIO_OUT_W1TC_REG, 1u << p); }
static inline void pin_set(int p, int v) { if (v) pin_hi(p); else pin_lo(p); }
static inline int  pin_get(int p) { return (REG_READ(GPIO_IN_REG) >> p) & 1u; }

/* SWDIO is bidirectional: drive it only while OpenOCD says to. Toggling
 * the enable register is a single store, unlike pinMode(). */
static inline void swdio_drive(void)
{
    REG_WRITE(GPIO_ENABLE_W1TS_REG, 1u << PIN_TMS);
}

static inline void swdio_float(void)
{
    REG_WRITE(GPIO_ENABLE_W1TC_REG, 1u << PIN_TMS);
}

/* Replies are buffered and flushed only when the input runs dry: OpenOCD
 * pipelines requests and blocks on the first read, so batching costs it
 * nothing and saves a USB frame per character. */
static uint8_t txbuf[512];
static int txn;

static inline void reply(char c)
{
    txbuf[txn++] = (uint8_t)c;
    if (txn == sizeof(txbuf)) {
        Serial.write(txbuf, txn);
        txn = 0;
    }
}

static void flush_reply(void)
{
    if (txn) {
        Serial.write(txbuf, txn);
        txn = 0;
    }
}

void setup(void)
{
    pinMode(PIN_TCK, OUTPUT);
    pinMode(PIN_TDI, OUTPUT);
    pinMode(PIN_TRST, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_TDO, INPUT_PULLUP);
    /* NRST driven open-drain: the target may hold it low itself, and a
     * push-pull high would fight that. */
    pinMode(PIN_SRST, OUTPUT_OPEN_DRAIN);
    /* TMS/SWDIO starts DRIVEN HIGH, which is the idle state for both
     * transports: JTAG drives TMS the whole time and never asks for a
     * direction change, and SWD idles with the host driving the line
     * before it hands over for a read ('o'). Leaving it an input here
     * would silently break JTAG -- pin_set() would write the output
     * register of a pin that drives nothing. */
    pinMode(PIN_TMS, OUTPUT);

    pin_hi(PIN_TRST);   /* both resets are active LOW -- idle released */
    pin_hi(PIN_SRST);
    pin_hi(PIN_TMS);
    pin_lo(PIN_TCK);
    pin_lo(PIN_LED);

    Serial.setRxBufferSize(4096);
    Serial.begin(BAUD);
}

void loop(void)
{
    int avail = Serial.available();

    if (avail <= 0) {
        flush_reply();   /* nothing more queued: let the host proceed */
        return;
    }
    while (avail-- > 0) {
        int c = Serial.read();
        if (c < 0)
            break;
        switch (c) {
        /* ---- JTAG ---- */
        case '0': case '1': case '2': case '3':
        case '4': case '5': case '6': case '7': {
            int v = c - '0';
            pin_set(PIN_TDI, v & 1);
            pin_set(PIN_TMS, (v >> 1) & 1);
            pin_set(PIN_TCK, (v >> 2) & 1);
            break;
        }
        case 'R':
            reply(pin_get(PIN_TDO) ? '1' : '0');
            break;

        /* ---- SWD. 'd'..'g' encode (SWCLK, SWDIO) as bit1, bit0 ---- */
        case 'd': case 'e': case 'f': case 'g': {
            int v = c - 'd';
            pin_set(PIN_TMS, v & 1);
            pin_set(PIN_TCK, (v >> 1) & 1);
            break;
        }
        case 'O':
            swdio_drive();
            break;
        case 'o':
            swdio_float();
            break;
        case 'c':
            reply(pin_get(PIN_TMS) ? '1' : '0');
            break;

        /* ---- resets, both active low on the target ---- */
        case 'r': pin_hi(PIN_TRST); pin_hi(PIN_SRST); break;
        case 's': pin_hi(PIN_TRST); pin_lo(PIN_SRST); break;
        case 't': pin_lo(PIN_TRST); pin_hi(PIN_SRST); break;
        case 'u': pin_lo(PIN_TRST); pin_lo(PIN_SRST); break;

        /* ---- misc ---- */
        case 'B': pin_hi(PIN_LED); break;
        case 'b': pin_lo(PIN_LED); break;
        case 'Z': flush_reply(); delay(1); break;
        case 'z': delayMicroseconds(1); break;
        case 'Q': flush_reply(); break;
        default:  break;   /* stray byte (boot noise): ignore */
        }
    }
    flush_reply();
}
