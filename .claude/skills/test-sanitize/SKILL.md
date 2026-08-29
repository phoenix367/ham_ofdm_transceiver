---
name: test-sanitize
description: Run every C suite under AddressSanitizer and UndefinedBehaviorSanitizer, plus the host-link parser fuzz. Use after ANY change to a type width, a buffer size, an index computation, or shift arithmetic in cport/ -- the suites passing is not sufficient evidence for those, and this is where the defects they miss are caught.
---

# Sanitizing the C port

```bash
cd cport && make sanitize
```

Builds all eight suites with `-fsanitize=address,undefined
-fno-sanitize-recover=all`, runs them, then feeds the host-link parser
40 MB of random input. Ends with `sanitize: clean`; anything else is a
finding. Takes a few minutes -- run it in the background.

```
test_primitives  11 passed, 0 failed
...
test_usb         11 passed, 0 failed
fuzz_usb         fuzz: 200k random chunks, 11 frames accepted, 39965075 resyncs, no faults
sanitize: clean
```

## When it is mandatory, not optional

The golden-vector suites compare outputs `memcmp`-exactly, which sounds
airtight and is not: a read past a buffer that happens to land on
harmless memory, or a shift that the compiler resolves benignly on x86,
passes every check. Three defects in this codebase's history were
invisible to the suites and caught only here:

- a `memcpy` with a hardcoded `sizeof(int64_t)` into an array that had
  just been narrowed to `int32_t` -- a 2x overrun the whole suite passed;
- `arr[i] >>= sh` with `sh` up to 34, undefined on a 32-bit type;
- `negative << n` in the FFT's block scaling, which gcc resolved one way
  on x86 `-O2` and differently on ARM `-Os` -- it presented as a lockup
  under QEMU and cost six wrong diagnoses before ASan/UBSan on the host
  found it in seconds.

So: **after any change to a type width, grep for hardcoded
`sizeof(int64_t)`, then run this.** `sizeof(*ptr)` is the fix for the
first class. Also run it after touching buffer sizes, ring indexing,
arena offsets, or anything in `dsp.c`/`fft.c`/`rx_detect.c` that shifts.

## Reading a failure

- `runtime error:` lines are UBSan: the file:line is exact and the
  message names the operation (`left shift of negative value`, `shift
  exponent 34 is too large`). Fix the arithmetic; do not add a cast that
  silences it.
- `ERROR: AddressSanitizer:` is a memory fault with a stack trace. The
  first frame inside `src/` is the bug; frames in `tests/` are the caller.
  `-fno-sanitize-recover=all` stops at the first one, so fix and re-run.
- The fuzz reporting *anything* other than a frame count and resync count
  means the parser can be driven out of bounds by a hostile or broken
  host; it must never happen.

## What this does NOT cover

- ARM-specific behaviour: run `test-arm` as well after fixing anything
  UBSan reported, because the original bug only *showed* on Cortex-M7.
- The Python model: `test-python`.
- Performance: sanitized builds are ~3x slower; never benchmark them.

## Adding a suite

`SAN_SUITES` and `SAN_ALL` in `cport/Makefile`. A suite that links only a
subset (like `test_usb`) gets its own source list in the target's
`if` branch -- do not hide a link failure by adding `src/*.c` wholesale.
