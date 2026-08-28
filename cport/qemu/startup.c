/* Bare-metal Cortex-M startup + ARM semihosting I/O for QEMU.
 *
 * Deliberately does NOT use newlib's rdimon crt0: that queries the host
 * for a stack base via SYS_HEAPINFO and relocates SP there, which lands
 * outside the region this linker script owns. Everything here is
 * explicit instead -- vector table, .data copy, .bss zero, heap, and a
 * _write that goes straight to semihosting. */
#include <stddef.h>
#include <sys/stat.h>
#include <stdio.h>

extern unsigned __data_start__, __data_end__, __bss_start__, __bss_end__;
extern unsigned __etext, _estack, __heap_start__, __heap_end__;
extern int main(void);

static inline int semihost(int op, void *arg)
{
    register int r0 __asm__("r0") = op;
    register void *r1 __asm__("r1") = arg;
    __asm__ volatile("bkpt 0xAB" : "+r"(r0) : "r"(r1) : "memory");
    return r0;
}

int _write(int fd, const char *buf, int len)
{
    /* SYS_WRITE0 rather than SYS_WRITE: the latter wants a semihosting
     * file HANDLE from SYS_OPEN(":tt"), not a POSIX fd, and passing 1
     * writes nowhere and reports success. WRITE0 is NUL-terminated, so
     * chunk through a small buffer. */
    char tmp[65];
    int done = 0, i, n;
    (void)fd;
    while (done < len) {
        n = (len - done) > 64 ? 64 : (len - done);
        for (i = 0; i < n; i++)
            tmp[i] = buf[done + i];
        tmp[n] = 0;
        semihost(0x04, tmp);
        done += n;
    }
    return len;
}

void *_sbrk(int incr)
{
    static char *cur;
    char *prev;
    if (cur == 0)
        cur = (char *)&__heap_start__;
    if (cur + incr > (char *)&__heap_end__)
        return (void *)-1;
    prev = cur;
    cur += incr;
    return prev;
}

int _close(int f) { (void)f; return -1; }
int _fstat(int f, struct stat *s) { (void)f; s->st_mode = S_IFCHR; return 0; }
int _isatty(int f) { (void)f; return 1; }
int _lseek(int f, int p, int w) { (void)f; (void)p; (void)w; return 0; }
int _read(int f, char *b, int l) { (void)f; (void)b; (void)l; return 0; }
int _getpid(void) { return 1; }
int _kill(int p, int s) { (void)p; (void)s; return -1; }

void _exit(int code)
{
    /* SYS_EXIT on 32-bit ARM takes the reason in R1 directly, not a
     * pointer to a parameter block (that is the 64-bit / EXIT_EXTENDED
     * form) -- passing a pointer exits with a bogus status. */
    (void)code;
    semihost(0x18, (void *)0x20026);
    for (;;) ;
}

/* A fault handler that spins is indistinguishable from slow code: the
 * first attempt at this burned 33 minutes at 99.9% CPU printing nothing,
 * because a fault had been caught by an infinite loop. Report and exit
 * instead -- the stacked PC is the whole diagnosis. */
void fault_report(unsigned *sp, int which)
{
    /* No printf. A fault handler that calls into the C library cannot
     * survive the conditions it exists to report -- every fault then
     * presents as the same opaque "can't escalate 3 to HardFault"
     * lockup, which is how two wrong diagnoses got made before this.
     * Hand-formatted into a static buffer, straight out via SYS_WRITE0.
     *
     * CFSR (0xE000ED28) is the answer: bit 16 UNDEFINSTR, 17 INVSTATE,
     * 24 UNALIGNED, 25 DIVBYZERO, 8..15 bus faults, 0..7 mem faults.
     * HFSR bit 30 FORCED means it escalated from one of those. */
    static char msg[] =
        "*** FAULT x pc=xxxxxxxx lr=xxxxxxxx psr=xxxxxxxx "
        "cfsr=xxxxxxxx hfsr=xxxxxxxx sp=xxxxxxxx\n";
    static const char hx[] = "0123456789abcdef";
    unsigned v[6];
    int f, i;

    v[0] = sp[6];                              /* stacked PC  */
    v[1] = sp[5];                              /* stacked LR  */
    v[2] = sp[7];                              /* stacked xPSR */
    v[3] = *(volatile unsigned *)0xE000ED28;   /* CFSR */
    v[4] = *(volatile unsigned *)0xE000ED2C;   /* HFSR */
    v[5] = (unsigned)(unsigned long)sp;
    msg[10] = (char)('0' + which);
    for (f = 0; f < 6; f++) {
        static const int at[6] = { 15, 27, 40, 54, 68, 80 };
        unsigned x = v[f];
        for (i = 7; i >= 0; i--) {
            msg[at[f] + i] = hx[x & 15u];
            x >>= 4;
        }
    }
    semihost(0x04, msg);
    semihost(0x18, (void *)0x20026);
    for (;;) ;
}

#define FAULT(name, idx)                                                  \
    __attribute__((naked)) void name(void)                                \
    {                                                                     \
        __asm__ volatile("tst lr, #4\n\t"                                 \
                         "ite eq\n\t"                                     \
                         "mrseq r0, msp\n\t"                              \
                         "mrsne r0, psp\n\t"                              \
                         "mov r1, %0\n\t"                                 \
                         "b fault_report\n" :: "I"(idx) : "r0", "r1");    \
    }
FAULT(HardFault_Handler, 0)
FAULT(MemManage_Handler, 1)
FAULT(BusFault_Handler, 2)
FAULT(UsageFault_Handler, 3)

static void hang(void) { for (;;) ; }

void Reset_Handler(void)
{
    unsigned *s = &__etext, *d = &__data_start__;

    /* The Cortex-M7 FPU is DISABLED at reset. Built -mfloat-abi=hard,
     * the very first FP instruction (main's "vpush {d8}" prologue) then
     * takes a UsageFault that escalates to HardFault -- which, with a
     * spinning fault handler, looks exactly like slow code. Grant full
     * access to CP10/CP11 before anything can use it. */
    *(volatile unsigned *)0xE000ED88 |= (0xFu << 20);
    __asm__ volatile("dsb" ::: "memory");
    __asm__ volatile("isb" ::: "memory");

    while (d < &__data_end__)
        *d++ = *s++;
    for (d = &__bss_start__; d < &__bss_end__; )
        *d++ = 0;
    /* unbuffered: these suites run for many minutes under emulation, and
     * buffered output only appears at exit -- a run that is killed part
     * way then shows nothing at all about how far it got */
    setvbuf(stdout, 0, _IONBF, 0);
    _exit(main());
}

/* 16 core exceptions + 96 external IRQ slots.
 *
 * A 16-entry table is the trap here: any exception with number >= 16 --
 * i.e. ANY peripheral interrupt -- reads past the end of it, fetches
 * whatever follows in .text, and branches there. The observed failure
 * was exactly that: stacking SUCCEEDED (SP = _estack - 0x20, one frame,
 * stack otherwise empty, so not an overflow), and the core then jumped
 * to PC=0 and locked up. It looked nondeterministic because it depends
 * on something asserting an IRQ, not on the code under test.
 *
 * Every slot is populated so an unexpected interrupt reports itself
 * instead of vectoring into space. */
__attribute__((section(".isr_vector"), used))
void (*const g_vectors[16 + 96])(void) = {
    (void (*)(void)) &_estack,
    Reset_Handler,
    hang,                 /* NMI */
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    hang, hang, hang, hang, hang, hang, hang, hang, hang,
    /* external IRQs 0..95 */
    hang, hang, hang, hang, hang, hang, hang, hang, hang, hang, hang, hang,
    hang, hang, hang, hang, hang, hang, hang, hang, hang, hang, hang, hang,
    hang, hang, hang, hang, hang, hang, hang, hang, hang, hang, hang, hang,
    hang, hang, hang, hang, hang, hang, hang, hang, hang, hang, hang, hang,
    hang, hang, hang, hang, hang, hang, hang, hang, hang, hang, hang, hang,
    hang, hang, hang, hang, hang, hang, hang, hang, hang, hang, hang, hang,
    hang, hang, hang, hang, hang, hang, hang, hang, hang, hang, hang, hang,
    hang, hang, hang, hang, hang, hang, hang, hang, hang, hang, hang, hang,
};
