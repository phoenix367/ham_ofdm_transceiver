# Running the C port on Cortex-M7 under QEMU

`make qemu SUITE=test_stream` links one golden-vector suite for Cortex-M7
and runs it under `qemu-system-arm -M mps2-an500`, with output over ARM
semihosting. It answers a question the host build cannot: does this code
produce **the same bits on ARM as on x86**?

    sudo apt install qemu-system-arm gcc-arm-none-eabi libnewlib-arm-none-eabi
    make qemu SUITE=test_primitives

## Status

Bit-identical to the host build, same golden vectors, same counts:

| suite             | x86-64 | Cortex-M7 |
|-------------------|--------|-----------|
| `test_primitives` | 11/11  | 11/11     |
| `test_bits`       | 16/16  | 16/16     |
| `test_tx`         | 10/10  | 10/10     |
| `test_stream`     | 8/8    | 8/8       |

`test_broadcast` (~37 MB), `test_rx` and `test_link` (~29 MB each) link
but do not fit: their static buffers are host-sized worst cases, mostly
in `rx_detect` (13.2 MB), `rx_demod` (9.9 MB), `broadcast` (9.6 MB) and
`tx` (4.4 MB). That is two orders of magnitude above the per-mode budget
in FEASIBILITY.md — the budget describes an intended sizing this code
does not yet express. Running the receiver on real silicon means
parameterising those buffers by mode, not just documenting them.

QEMU is functional, not cycle-accurate: it says nothing about timing.

## Four traps, each of which cost real time here

**The toolchain on PATH may not be able to link M-profile at all.** A
vendor `arm-none-eabi-gcc` (Xilinx, ST) is often Cortex-A oriented: it
compiles `-mcpu=cortex-m7` happily and then fails at link because it has
no `v7e-m` multilib. Check with `arm-none-eabi-gcc -print-multi-lib |
grep v7e-m`; the Makefile prefers `/usr/bin/arm-none-eabi-gcc` for this
reason. This is also why the older `armsize` target only ever compiled
objects — nothing here had actually been linked, let alone run.

**The Cortex-M7 FPU is disabled at reset.** Built `-mfloat-abi=hard`,
the first FP instruction — typically `vpush {d8}` in a function
prologue, before any of your code runs — takes a UsageFault that
escalates to HardFault. `startup.c` enables CP10/CP11 in `CPACR` first.

**A fault handler that spins is indistinguishable from slow code.**
Before the handlers reported anything, the above fault presented as 33
minutes at 99.9% CPU with no output, which reads exactly like heavy
emulation. The handlers here print the stacked PC and exit; feed that PC
to `arm-none-eabi-addr2line`.

**newlib's rdimon `crt0` relocates the stack.** It asks the host for a
stack base via semihosting `SYS_HEAPINFO` and lands outside the region
this linker script owns. `startup.c` therefore does its own `.data` copy,
`.bss` zero and heap setup instead of using `rdimon`, and writes through
`SYS_WRITE0` — note `SYS_WRITE` wants a handle from `SYS_OPEN(":tt")`,
not a POSIX fd, and silently writes nowhere if given `1`.

## Known cosmetic issue

`newlib-nano`'s `printf` omits `long long`, so a suite's `%lld` prints
as a literal `ld`. Add `-u _printf_ll` if that matters; it does not
affect pass or fail.
