---
name: measure-target
description: Measure RAM, flash and cycle counts on the STM32H743 - `make armmeas` for the linked image's memory, `bench/armbench.c` for DWT cycle counts of primitives and whole receive stages on silicon. Use whenever a memory or CPU figure is claimed for the port; QEMU is functional, not cycle-accurate.
---

# Measuring on the target

## RAM and flash: `make armmeas`

```bash
cd cport && make armmeas                                   # as built
make armmeas ARMMEAS_DEFS=-DMAX_LLRS=1024                  # no EXT frames
make armmeas ARMMEAS_SRC='$(TXSRC)' ARMMEAS_DEFS='-DARMMEAS_TX_ONLY -DOFDM_ARENA_BYTES=27000'
```

`bench/armmeas.c` is the reference main. It references exactly what a
station does; every static buffer is a worst case and `--gc-sections`
drops what is unreachable, so **a figure from an ad-hoc main is not a
figure anyone can check**. Output is `size` plus the `.bss` objects over
8 kB. Record results in `cport/FEASIBILITY.md`.

## Cycles: `bench/armbench.c`

Built with the RAM linker script, run via `target-jtag`, results in the
beacon at 0x20000000 as (name, min cycles over reps, units). Add a case
with the `MEASURE(reps, units, "label", body)` macro. Minimum over
repetitions is the honest estimator: interrupts only ever add cycles.

Whole-frame stages pipe `txs_pull` into `rxs_push` on the part; the
transmitter's arena state is saved and restored around each push
(`txs_state_blob`/`txs_state_restore`, a test hook) because the
half-duplex arena guard otherwise -- correctly -- refuses.

## Traps that produced wrong numbers here

- **A measured 0 cycles is gcc deleting the call.** A pure function whose
  result is unread is elided; assigning to a `volatile` OUTSIDE the timed
  region does not help (the call is hoisted past the CYCCNT reads). Put a
  `volatile` store INSIDE the measured function.
- **AXI-SRAM measured 2.3x slower than DTCM and it was not the part.**
  The resident firmware left an MPU region over AXI-SRAM marked
  *shareable*; an M7 has no coherency unit, so that means uncached.
  Disable the inherited MPU (`MPU_CTRL = 0`) before measuring. The bench
  does; the lesson generalises to any figure taken without a core reset.
- **Cache on/off "made no difference" because CCR persisted** from the
  previous image. Toggle the cache inside ONE image and record CCR.
- **`.rodata` in ITCM did not matter** (1.00x) -- checked so nobody pays
  to learn it twice.
- **D2 SRAM is clock-gated**: set `RCC_AHB2ENR` bits 29-31 before
  placing anything there, or every access faults.

## Figures to compare against

147 MMAC/s at 480 MHz for the correlation inner loop, from any of
DTCM/AXI/D2 once the MPU is right. Whole EXTREME frame 1978 Mcycles
(9.5 % of a 480 MHz core), acquisition 41.8 %. All in `FEASIBILITY.md`
with the dates they were taken.
