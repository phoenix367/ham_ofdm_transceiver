/* Host test for the probe firmware's GPIO layer.
 *
 * The firmware addresses JTAG lines by MASK so that a duplicated line
 * and its second copy are switched by one store. That is easy to get
 * subtly wrong (a missed call site drives one board and not the other,
 * which on a daisy-chain looks like a dead scan chain), so model the
 * w1ts/w1tc registers and assert the actual bits for every protocol
 * character. Build both ways: DUAL_PROBE=1 must move the copies in
 * lockstep, DUAL_PROBE=0 must never touch those pins at all.
 *
 *   g++ -DDUAL_PROBE=1 -I. -o t test_pinmask.cpp && ./t
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t g_out, g_enable, g_in;
static uint32_t g_touched;   /* every bit ever written, either direction */

enum { R_W1TS, R_W1TC, R_IN, R_EN_W1TS, R_EN_W1TC };

static void reg_write(int r, uint32_t v)
{
    switch (r) {
    case R_W1TS:    g_out |= v;     g_touched |= v; break;
    case R_W1TC:    g_out &= ~v;    g_touched |= v; break;
    case R_EN_W1TS: g_enable |= v;  break;
    case R_EN_W1TC: g_enable &= ~v; break;
    }
}
static uint32_t reg_read(int r) { return r == R_IN ? g_in : 0; }

#define GPIO_OUT_W1TS_REG    R_W1TS
#define GPIO_OUT_W1TC_REG    R_W1TC
#define GPIO_IN_REG          R_IN
#define GPIO_ENABLE_W1TS_REG R_EN_W1TS
#define GPIO_ENABLE_W1TC_REG R_EN_W1TC
#define REG_WRITE(r, v) reg_write((r), (uint32_t)(v))
#define REG_READ(r)     reg_read(r)

#define OUTPUT 1
#define INPUT_PULLUP 2
#define OUTPUT_OPEN_DRAIN 3
static uint32_t g_pinmoded;
static void pinMode(int p, int) { g_pinmoded |= 1u << p; }
static void delay(unsigned) {}
static void delayMicroseconds(unsigned) {}

static const char *g_feed; static int g_feed_n, g_feed_i;
static char g_replies[256]; static int g_reply_n;
struct FakeSerial {
    int available() { return g_feed_n - g_feed_i; }
    int read() { return g_feed_i < g_feed_n ? g_feed[g_feed_i++] : -1; }
    void write(const uint8_t *b, int n)
    { for (int i = 0; i < n && g_reply_n < 255; i++) g_replies[g_reply_n++] = (char)b[i]; }
    void begin(int) {}
    void setRxBufferSize(int) {}
} Serial;

#define main ino_main_unused
#include "esp32_rbb/esp32_rbb.ino"
#undef main

static int g_fail;
static void chk(const char *what, int ok)
{
    printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) g_fail++;
}
/* every listed bit high, every other listed bit low */
static void chk_bits(const char *what, uint32_t hi, uint32_t lo)
{
    int ok = (g_out & hi) == hi && (g_out & lo) == 0;
    if (!ok)
        printf("     out=%08x want hi=%08x lo=%08x\n", g_out, hi, lo);
    chk(what, ok);
}
static void feed(const char *s)
{
    g_feed = s; g_feed_n = (int)strlen(s); g_feed_i = 0; g_reply_n = 0;
    loop();
    loop();   /* second pass: available()==0 flushes replies */
}

#define B(p) (1u << (p))
#if DUAL_PROBE
#define TCK (B(PIN_TCK) | B(PIN_TCK_B))
#define TMS (B(PIN_TMS) | B(PIN_TMS_B))
#define TRST (B(PIN_TRST) | B(PIN_TRST_B))
#else
#define TCK B(PIN_TCK)
#define TMS B(PIN_TMS)
#define TRST B(PIN_TRST)
#endif
#define TDI B(PIN_TDI)
#define SRST B(PIN_SRST)

int main(void)
{
    printf("--- DUAL_PROBE=%d ---\n", DUAL_PROBE);
    setup();

    /* idle: both resets released, TMS driven high, TCK low */
    chk_bits("setup: TRST+SRST+TMS high, TCK low", TRST | SRST | TMS, TCK);

    /* every JTAG write code: bit2=TCK bit1=TMS bit0=TDI */
    for (int v = 0; v < 8; v++) {
        char s[2] = { (char)('0' + v), 0 };
        char name[64];
        uint32_t hi = 0, lo = 0;
        ((v & 1) ? hi : lo) |= TDI;
        ((v & 2) ? hi : lo) |= TMS;
        ((v & 4) ? hi : lo) |= TCK;
        feed(s);
        snprintf(name, sizeof(name), "jtag '%c': TCK=%d TMS=%d TDI=%d together",
                 '0' + v, (v >> 2) & 1, (v >> 1) & 1, v & 1);
        chk_bits(name, hi, lo);
    }

    /* resets, both active low */
    feed("u"); chk_bits("'u': TRST low, SRST low", 0, TRST | SRST);
    feed("t"); chk_bits("'t': TRST low, SRST high", SRST, TRST);
    feed("s"); chk_bits("'s': TRST high, SRST low", TRST, SRST);
    feed("r"); chk_bits("'r': TRST high, SRST high", TRST | SRST, 0);

    /* TDO is sampled from the END of the chain, on its own pin */
    g_in = B(PIN_TDO); feed("R");
    chk("'R' reads TDO high", g_reply_n == 1 && g_replies[0] == '1');
    g_in = 0; feed("R");
    chk("'R' reads TDO low", g_reply_n == 1 && g_replies[0] == '0');

    /* SWD direction must move every copy of TMS together */
    feed("o");
    chk("'o' floats all TMS copies", (g_enable & TMS) == 0);
    feed("O");
    chk("'O' drives all TMS copies", (g_enable & TMS) == TMS);

#if DUAL_PROBE
    chk("duplicate pins configured as outputs",
        (g_pinmoded & (B(PIN_TCK_B) | B(PIN_TMS_B) | B(PIN_TRST_B)))
            == (B(PIN_TCK_B) | B(PIN_TMS_B) | B(PIN_TRST_B)));
    /* the whole point: a copy is never written without its primary */
    chk("TCK copy never diverges from TCK",
        ((g_out >> PIN_TCK) & 1) == ((g_out >> PIN_TCK_B) & 1));
#else
    /* single-board build must leave the spare pins completely alone */
    uint32_t spare = B(PIN_TCK_B) | B(PIN_TMS_B) | B(PIN_TRST_B);
    chk("single-board build never drives the spare pins",
        (g_touched & spare) == 0 && (g_pinmoded & spare) == 0);
#endif

    printf("%s\n", g_fail ? "FAILED" : "all ok");
    return g_fail ? 1 : 0;
}
