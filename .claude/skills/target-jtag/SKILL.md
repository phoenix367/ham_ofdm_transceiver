---
name: target-jtag
description: Run a RAM-resident image on the real STM32H743 over the ESP32 JTAG probe - bring the bridge up, reset, load, run, read the result beacon, dump buffers. Use for anything that must be measured on silicon rather than under QEMU (test-arm). Do NOT flash: the board's own firmware stays in place and a power-cycle restores it.
---

# Running on the real STM32H743 over JTAG

Everything runs from RAM. OpenOCD loads the ELF at its link addresses,
sets SP/PC, resumes; the board's flash is never written. Images that
support this: `build/armbench.elf` (`make` in bench), `build/usb_bringup.elf`
(`make usbfw`), `build/analog_loop.elf` (`make analogfw`) -- all linked
with `target/stm32h743_usb.ld` and reporting through a **beacon struct at
0x20000000** (DTCM) read back with `mdw`.

## 1. Bridge up

```bash
PORT=$(ls /dev/ttyUSB* | head -1)          # it RENUMBERS -- never hardcode
cd tools/esp32-probe
./rbb_bridge.py --port $PORT --baud 921600 --listen 3335 > /tmp/bridge.log 2>&1 &
echo $! > /tmp/bridge.pid
until grep -q waiting /tmp/bridge.log; do sleep 1; done
```

Confirm the chain before anything else:

```bash
cd cport && openocd -f ../tools/esp32-probe/stm32h7-rbb.cfg -c init -c "mdw 0x20000000 2" -c exit 2>&1 | grep -E 'cpu tap/device found|^0x2000'
```

`0x6ba00477` is the ARM DP; if it is missing, nothing else will work.

## 2. Reset, load, run, read

```bash
# clean reset: SYSRESETREQ. `reset halt` does NOT work (NRST timing
# over this probe), and restarting in place after poking peripheral
# registers wedges cores -- always start from this.
openocd -f ../tools/esp32-probe/stm32h7-rbb.cfg -c init -c halt -c "mww 0xE000ED0C 0x05FA0004" -c exit
sleep 4
openocd -f ../tools/esp32-probe/stm32h7-rbb.cfg -c init -c halt \
  -c "load_image build/<image>.elf" -c "reg sp 0x20020000" -c "reg pc 0x00000000" \
  -c resume -c "wait_halt 60000" -c "mdw 0x20000000 24" -c exit 2>&1 | grep -E 'downloaded|halted due|^0x2000'
```

~0.62 kB/s: a 50 kB image is **82 s**. Images end in `bkpt`, so
`wait_halt` returns when they finish; a beacon can also be read while the
core runs (`mdw` does not halt it).

Dump a buffer the beacon names: `dump_image /tmp/x.bin <addr> <bytes>`.

## Traps, each measured here

- **`mdw` output goes to STDERR.** A script capturing stdout reports "no
  beacon" for a beacon that is plainly there.
- **The ESP32 rebooting resets the target** -- its GPIO25 drives NRST and
  is low during boot. A jumper going in next to it re-enumerated the
  CH340: bridge socket "connection reset", `/dev/ttyUSB0` became
  `/dev/ttyUSB1`, and the RAM image was gone. Restart the bridge on the
  new node, then reload.
- **Faults vanish into the resident firmware.** A RAM image with no
  vector table inherits the old VTOR; a fault then shows as `PC` in
  `0x08......` and a beacon frozen mid-way. Images that install
  `target/vectors.c` record faults in the beacon instead; a stray
  SysTick is survived, a real fault stops with ICSR/CFSR/HFSR.
- **An unbounded wait looks like a crash.** A `while(!ready)` on a
  hardware flag the firmware cannot satisfy (no VBUS, a clock-gated
  SRAM) stalls with `loops = 0`. Bound it and record the stage.
- **D2/D3 SRAM and every peripheral are clock-gated at reset.** Reading
  them fails as "memory absent" until the RCC enable is set.
- **The core is never reset between runs unless you do it** -- MPU
  regions, cache state and peripheral config all persist from whatever
  ran before, including the resident firmware. See `measure-target`.

## Restoring the board

Power-cycle, or `-c "reset run"`. Flash was never touched. Stop the
bridge with `kill $(cat /tmp/bridge.pid)`.
