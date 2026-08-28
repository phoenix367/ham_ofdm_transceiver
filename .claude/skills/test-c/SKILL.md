---
name: test-c
description: Run the C port's golden-vector test suites on the host, optionally under AddressSanitizer and UndefinedBehaviorSanitizer. Use after any change under cport/src/, and always after changing a type width, a buffer size, or anything the fixed-point model also implements.
---

# Testing the C port on the host

```bash
cd cport && make test
```

Seven suites run; each prints `N passed, 0 failed`:

| suite | covers |
|---|---|
| `test_primitives` | FFT/IFFT, BFP, Hilbert, NCO, CORDIC |
| `test_bits` | CRC, scrambler, interleaver, Viterbi, LDPC |
| `test_tx` | frame and burst waveforms, streaming transmitter |
| `test_rx` | frame-at-once detect + decode, HARQ, calibration |
| `test_stream` | streaming receiver, all modes, noisy, burst |
| `test_link` | rate ladder, LC word, station/ARQ |
| `test_broadcast` | non-ARQ groups, SYNC descriptor, EOS |

Comparisons are `memcmp` against vectors generated from the Python
model — no tolerances. A failure is a behavioural change.

## Run the sanitizers too

The suites passing is **not** sufficient evidence after a width or
buffer change. Three defects in this codebase's history were invisible
to the suites and caught only here:

```bash
gcc -std=c99 -g -O1 -fsanitize=address,undefined -Isrc -Itests \
    -o /tmp/ub_test_rx tests/test_rx.c src/*.c -lm && /tmp/ub_test_rx
```

Repeat per suite (`test_rx` and `test_stream` exercise the most). Expect
zero `runtime error` and zero `ERROR:` lines.

What they have caught here: a `memcpy` with a hardcoded `sizeof(int64_t)`
into a narrowed array (a 2x buffer overrun the whole suite passed), a
`>>= sh` with `sh` up to 34 that is undefined on a 32-bit type, and
`negative << n` in the FFT's block scaling.

**Rule of thumb: after any change to a type width, grep for hardcoded
`sizeof(int64_t)` and run the sanitizers.** `sizeof(*ptr)` is the fix.

## Configuration knobs worth testing

```bash
make test                                   # default: EXT frames enabled
gcc ... -DMAX_LLRS=1024 ...                 # build without EXT frames
```

## Do not

Regenerate golden vectors (`../venv/bin/python gen_vectors.py`) to make
a failing test pass, unless the wire format genuinely changed and you
can say why. `cport/tests/test_vectors.h` is the reference.
