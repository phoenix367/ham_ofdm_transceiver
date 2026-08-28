---
name: test-arm
description: Run the C port's test suites on Cortex-M7 under QEMU, and measure the linked ARM image's flash and RAM. Use to confirm a change is bit-identical on the target (not just on x86), and whenever a memory figure is claimed.
---

# Testing the C port on Cortex-M7 under QEMU

```bash
cd cport && make qemu SUITE=test_stream
```

`SUITE` is any suite in `cport/tests/`. The image boots on
`qemu-system-arm -M mps2-an500` (Cortex-M7) and reports over ARM
semihosting; expect the same `N passed, 0 failed` as the host build.

Needs `qemu-system-arm`, `gcc-arm-none-eabi` and
`libnewlib-arm-none-eabi`. The Makefile prefers `/usr/bin/arm-none-eabi-gcc`
deliberately — see the traps below.

## Which suites are worth running

- **`test_stream`** is the important one: it exercises the streaming
  receiver through NORMAL/ROBUST/EXTREME, a noisy −5 dB case with CFO
  and multipath, LDPC, and burst decoding. Takes several minutes under
  emulation, so run it in the background.
- `test_tx`, `test_primitives`, `test_bits` finish in seconds.
- `test_rx`, `test_link`, `test_broadcast` **do not fit** — they use the
  frame-at-once path, whose host-sized scratch needs ~29 MB. That is
  expected, not a regression.

Run this after any change to detection, demodulation or a numeric type.
Host tests passing is weaker evidence: undefined behaviour that gcc
resolves benignly on x86 `-O2` can behave differently on ARM `-Os`.

## Measuring the image

```bash
ARMCC=/usr/bin/arm-none-eabi-gcc
$ARMCC -std=c99 -Os -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
  -ffunction-sections -fdata-sections -Isrc -nostartfiles -specs=nano.specs \
  -T qemu/mps2.ld -Wl,--gc-sections qemu/startup.c <main>.c src/*.c -lm -o /tmp/img.elf
/usr/bin/arm-none-eabi-size /tmp/img.elf
/usr/bin/arm-none-eabi-nm --size-sort -S --radix=d /tmp/img.elf | \
  awk '$3=="b"||$3=="B"{t+=$2; if($2>30000) print $2, $4} END{print "total", t}'
```

`--gc-sections` is **required** for a meaningful number: without it the
linker keeps whole objects and the frame-at-once buffers stay resident,
overstating RAM by megabytes. Add `-DMAX_LLRS=1024` for a build without
extended frames.

Current figures live in `cport/FEASIBILITY.md`; update them there when
they move.

## Four traps, each of which presents as something else

- **A vendor `arm-none-eabi-gcc` may not link M-profile at all.** Xilinx
  and ST SDKs ship Cortex-A oriented toolchains that compile
  `-mcpu=cortex-m7` happily and fail at link with no `v7e-m` multilib.
  Check `arm-none-eabi-gcc -print-multi-lib | grep v7e-m`.
- **The Cortex-M7 FPU is disabled at reset.** A `-mfloat-abi=hard` build
  faults on the first `vpush` in a prologue. `qemu/startup.c` enables
  CP10/CP11 first.
- **VTOR must be set.** Reset is fetched from address 0 by hardware, so
  boot works without it — but every later exception vectors through
  VTOR, and a fault then cannot reach its handler. It presents as a
  lockup with no handler output at all.
- **QEMU's post-lockup register dump is not the faulting context.**
  Every register reads zero and PC is a halt placeholder. Do not
  diagnose from it. The fault handlers in `qemu/startup.c` print the
  stacked PC, CFSR and HFSR — use those, and feed the PC to
  `arm-none-eabi-addr2line`.

If a run produces no output for minutes, that is not necessarily
slowness: a fault handler that spins looks identical to slow code.
