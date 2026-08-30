# C port of the fixed-point model -- host build + golden-vector tests.
# See ../docs/c-port-plan.md.

CC      = gcc
CFLAGS  = -std=c99 -Wall -Wextra -Werror -O2
BUILD   = build

SRC     = src/arena.c src/fft.c src/dsp.c src/bits.c src/conv.c src/packets.c src/tx.c \
          src/broadcast.c \
          src/rx_demod.c src/rx_detect.c src/rx_stream.c src/ldpc.c \
          src/link.c src/station.c
GEN     = src/rom_tables.h src/rom_modes.h tests/test_vectors.h

all: test

$(GEN): gen_vectors.py
	../venv/bin/python gen_vectors.py

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/test_primitives: tests/test_primitives.c $(SRC) $(GEN) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ tests/test_primitives.c src/fft.c src/dsp.c

$(BUILD)/test_bits: tests/test_bits.c $(SRC) $(GEN) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ tests/test_bits.c src/bits.c src/conv.c src/ldpc.c

$(BUILD)/test_tx: tests/test_tx.c $(SRC) $(GEN) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ tests/test_tx.c src/arena.c src/tx.c src/packets.c \
	    src/bits.c src/conv.c src/ldpc.c src/fft.c

$(BUILD)/test_link: tests/test_link.c $(SRC) $(GEN) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ tests/test_link.c src/arena.c src/link.c src/station.c \
	    src/rx_demod.c src/rx_detect.c src/ldpc.c src/tx.c src/packets.c \
	    src/bits.c src/conv.c src/fft.c src/dsp.c -lm

$(BUILD)/test_stream: tests/test_stream.c $(SRC) $(GEN) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ tests/test_stream.c src/arena.c src/rx_stream.c src/rx_demod.c \
	    src/rx_detect.c src/ldpc.c src/tx.c src/packets.c src/bits.c \
	    src/conv.c src/fft.c src/dsp.c

$(BUILD)/test_broadcast: tests/test_broadcast.c $(SRC) $(GEN) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ tests/test_broadcast.c src/arena.c src/broadcast.c \
	    src/rx_demod.c src/rx_detect.c src/ldpc.c src/tx.c src/packets.c \
	    src/bits.c src/conv.c src/fft.c src/dsp.c src/link.c src/station.c -lm

# USB host-link protocol. Deliberately NOT in $(SRC): the device-side
# USB layer is optional, and keeping it out means the RAM figures from
# `make armmeas` stay comparable with every earlier measurement. Its own
# cost is ~5 kB (UM_TXQ plus one parser).
$(BUILD)/test_usb: tests/test_usb.c src/usb_proto.c | $(BUILD)
	$(CC) $(CFLAGS) -o $@ tests/test_usb.c src/usb_proto.c

USB_EMU_SRC = bench/usb_modem_emu.c src/usb_proto.c src/usb_modem.c \
              src/arena.c src/station.c src/link.c src/packets.c \
              src/bits.c src/conv.c src/ldpc.c src/tx.c src/fft.c \
              src/dsp.c src/rx_demod.c src/rx_detect.c

$(BUILD)/usb_modem_emu: $(USB_EMU_SRC) $(GEN) | $(BUILD)
	$(CC) $(CFLAGS) -Isrc -o $@ $(USB_EMU_SRC) -lm

usbemu: $(BUILD)/usb_modem_emu

$(BUILD)/test_rx: tests/test_rx.c $(SRC) $(GEN) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ tests/test_rx.c src/arena.c src/rx_demod.c src/rx_detect.c src/rx_stream.c src/ldpc.c \
          src/link.c src/station.c \
	    src/tx.c src/packets.c src/bits.c src/conv.c src/fft.c src/dsp.c

test: $(BUILD)/test_primitives $(BUILD)/test_bits $(BUILD)/test_tx $(BUILD)/test_rx $(BUILD)/test_stream $(BUILD)/test_link $(BUILD)/test_broadcast $(BUILD)/test_usb
	./$(BUILD)/test_primitives
	./$(BUILD)/test_bits
	./$(BUILD)/test_tx
	./$(BUILD)/test_rx
	./$(BUILD)/test_stream
	./$(BUILD)/test_link
	./$(BUILD)/test_broadcast
	./$(BUILD)/test_usb

clean:
	rm -rf $(BUILD)

distclean: clean
	rm -f $(GEN)

.PHONY: all test clean distclean usbemu

# --- measurement harness (plan section 6) -------------------------------
$(BUILD)/replay: bench/replay.c $(SRC) $(GEN) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ bench/replay.c $(SRC)

bench: $(BUILD)/replay
	./$(BUILD)/replay

# Cortex-M7 code/flash size (arm-none-eabi from PATH); host-sized .bss is
# NOT the MCU RAM budget -- see FEASIBILITY.md for per-mode RAM.
ARMCC = arm-none-eabi-gcc
ARMFLAGS = -std=c99 -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
           -O2 -Wall -ffunction-sections

armsize: $(GEN) | $(BUILD)
	@for f in $(SRC); do \
	    $(ARMCC) $(ARMFLAGS) -c $$f -o $(BUILD)/arm_$$(basename $$f .c).o; \
	done
	@arm-none-eabi-size $(BUILD)/arm_*.o

# Run one golden-vector suite on Cortex-M7 under QEMU (qemu/README.md).
#   make qemu SUITE=test_stream
# Needs an M-profile toolchain: a vendor arm-none-eabi (Xilinx, ST) is
# Cortex-A oriented, compiles cortex-m7 fine and then fails at link with
# no v7e-m multilib -- so prefer the packaged one when it is present.
QEMUCC   ?= $(if $(wildcard /usr/bin/arm-none-eabi-gcc),/usr/bin/arm-none-eabi-gcc,arm-none-eabi-gcc)
QEMUSIZE ?= $(QEMUCC:gcc=size)
QEMUMACH ?= mps2-an500
QEMUCPU  ?= cortex-m7
SUITE    ?= test_primitives
QEMUFLAGS = -std=c99 -Os -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 \
            -mfloat-abi=hard -ffunction-sections -fdata-sections \
            -Isrc -Itests -nostartfiles -specs=nano.specs -u _printf_float

qemu: $(GEN) | $(BUILD)
	$(QEMUCC) $(QEMUFLAGS) -T qemu/mps2.ld -Wl,--gc-sections \
	    qemu/startup.c $(SRC) tests/$(SUITE).c -lm \
	    -o $(BUILD)/$(SUITE).elf
	@$(QEMUSIZE) $(BUILD)/$(SUITE).elf
	qemu-system-arm -M $(QEMUMACH) -cpu $(QEMUCPU) -nographic \
	    -semihosting-config enable=on,target=native \
	    -kernel $(BUILD)/$(SUITE).elf

# Cortex-M7 RAM/flash of a station image (FEASIBILITY.md "Memory").
# --gc-sections is REQUIRED: without it the linker keeps whole objects
# and the frame-at-once buffers stay resident, overstating RAM by
# megabytes. ARMMEAS_DEFS=-DMAX_LLRS=1024 measures a build without
# extended frames.
# QEMUFLAGS without the suites' -u _printf_float: pulling newlib's float
# formatter in costs ~10 kB of flash that a station does not spend.
# A transmit-only image must also LINK transmit-only: the receiver's
# arena regions assert their fit at compile time, so leaving those files
# in the build defeats a shrunken -DOFDM_ARENA_BYTES (loudly, which is
# the point). ARMMEAS_SRC=$(TXSRC) selects the transmit half.
TXSRC = src/arena.c src/fft.c src/bits.c src/conv.c src/ldpc.c \
        src/packets.c src/tx.c
ARMMEAS_SRC ?= $(SRC)
ARMMEAS_DEFS ?=
ARMMEASFLAGS = -std=c99 -Os -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 \
               -mfloat-abi=hard -ffunction-sections -fdata-sections \
               -Isrc -nostartfiles -specs=nano.specs

armmeas: $(GEN) | $(BUILD)
	$(QEMUCC) $(ARMMEASFLAGS) $(ARMMEAS_DEFS) -T qemu/mps2.ld \
	    -Wl,--gc-sections qemu/startup.c bench/armmeas.c $(ARMMEAS_SRC) -lm \
	    -o $(BUILD)/armmeas.elf
	@$(QEMUSIZE) $(BUILD)/armmeas.elf
	@echo "--- .bss objects >= 8 kB ---"
	@$(QEMUCC:gcc=nm) --size-sort -S --radix=d $(BUILD)/armmeas.elf | \
	    awk '$$3=="b"||$$3=="B"{t+=$$2; if($$2>=8000) printf "%9d  %s\n",$$2,$$4} \
	         END{printf "%9d  TOTAL .bss\n",t}' | sort -rn

# ZC anchor A/B: EXTREME noise sweep, both arms over identical waveforms.
#   make zcab            (60 trials/point)
#   make zcab TRIALS=250 (knee resolution ~0.01 dB)
ZCAB_SRC = bench/zc_anchor_ab.c src/arena.c src/rx_stream.c src/rx_demod.c \
           src/rx_detect.c src/ldpc.c src/tx.c src/packets.c src/bits.c \
           src/conv.c src/fft.c src/dsp.c
TRIALS ?= 60

zcab: $(GEN) | $(BUILD)
	$(CC) $(CFLAGS) -DZC_ANCHOR_LEGACY -Isrc -o $(BUILD)/zcab_legacy \
	    $(ZCAB_SRC) -lm
	$(CC) $(CFLAGS) -Isrc -o $(BUILD)/zcab_anchored $(ZCAB_SRC) -lm
	@./$(BUILD)/zcab_legacy $(TRIALS)
	@echo
	@./$(BUILD)/zcab_anchored $(TRIALS)

# USB device firmware (TinyUSB). Third-party trees are fetched, not
# vendored -- see usb/README.md.
TINYUSB   ?= /tmp/tinyusb
CMSIS_H7  ?= /tmp/cmsis_h7
CMSIS5    ?= /tmp/cmsis5
USB_RHPORT ?= 0

USBFW_SRC = target/start.c usb/usb_main.c usb/usb_descriptors.c \
            usb/bsp_stm32h7.c src/usb_proto.c src/usb_modem.c \
            src/arena.c src/station.c src/link.c src/packets.c \
            src/bits.c src/conv.c src/ldpc.c src/tx.c src/fft.c \
            $(TINYUSB)/src/tusb.c $(TINYUSB)/src/common/tusb_fifo.c \
            $(TINYUSB)/src/device/usbd.c \
            $(TINYUSB)/src/class/vendor/vendor_device.c \
            $(TINYUSB)/src/portable/synopsys/dwc2/dcd_dwc2.c \
            $(TINYUSB)/src/portable/synopsys/dwc2/dwc2_common.c

usbfw: | $(BUILD)
	$(QEMUCC) -std=c99 -O2 -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 \
	    -mfloat-abi=hard -ffunction-sections -fdata-sections \
	    -Isrc -Iusb -I$(TINYUSB)/src -I$(CMSIS_H7)/Include \
	    -I$(CMSIS5)/CMSIS/Core/Include -DSTM32H743xx \
	    -DOFDM_USB_RHPORT=$(USB_RHPORT) \
	    -nostartfiles -specs=nano.specs -T target/stm32h743_usb.ld \
	    -Wl,--gc-sections $(USBFW_SRC) -o $(BUILD)/usb_bringup.elf
	@$(QEMUSIZE) $(BUILD)/usb_bringup.elf

# Analog loopback test stand: DAC1_OUT1 (PA4) -> wire -> ADC1 ch3 (PA6),
# one board, a NORMAL frame through a real analog path. RAM-resident.
ANALOG_SRC = target/start.c target/vectors.c bench/analog_loop.c \
             src/arena.c src/tx.c src/packets.c src/bits.c src/conv.c \
             src/ldpc.c src/fft.c src/dsp.c src/rx_demod.c \
             src/rx_detect.c src/rx_stream.c

analogfw: $(GEN) | $(BUILD)
	$(QEMUCC) -std=c99 -O2 -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 \
	    -mfloat-abi=hard -ffunction-sections -fdata-sections \
	    -Isrc -Itarget -DMAX_LLRS=1024 -nostartfiles -specs=nano.specs \
	    -T target/stm32h743_usb.ld -Wl,--gc-sections $(ANALOG_SRC) \
	    -o $(BUILD)/analog_loop.elf
	@$(QEMUSIZE) -A $(BUILD)/analog_loop.elf | grep -E '^\.(text|bss|d2_bss|dtcm_bss|results)'

# Two boards linked by audio: board A DAC1_OUT1 (PA4) -> wire -> board B
# ADC1 ch3 (PA6), plus a ground wire. One image, role chosen over JTAG.
LINK_SRC = target/start.c target/vectors.c bench/analog_link.c \
           src/arena.c src/tx.c src/packets.c src/bits.c src/conv.c \
           src/ldpc.c src/fft.c src/dsp.c src/rx_demod.c \
           src/rx_detect.c src/rx_stream.c

linkfw: $(GEN) | $(BUILD)
	$(QEMUCC) -std=c99 -O2 -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 \
	    -mfloat-abi=hard -ffunction-sections -fdata-sections \
	    -Isrc -Itarget -DMAX_LLRS=1024 -nostartfiles -specs=nano.specs \
	    -T target/stm32h743_usb.ld -Wl,--gc-sections $(LINK_SRC) \
	    -o $(BUILD)/analog_link.elf
	@$(QEMUSIZE) -A $(BUILD)/analog_link.elf | grep -E '^\.(text|bss|d2_bss|dtcm_bss|results)'

# Run the two-board link. TX is released FIRST so the whole of RX's
# capture window sits inside the transmission -- the two boards share no
# clock, not even to start. Reset first for the same reason run-% does:
# a RAM image inherits the resident firmware's clock setup, and stale
# peripheral state from a previous run wedges it.
LINK_TX ?= stmA
LINK_RX ?= stmB
LINK_ELF = $(BUILD)/analog_link.elf

link-run: linkfw
	@echo "resetting $(LINK_TX) and $(LINK_RX)..."; \
	 $(OCD2) -c init \
	   -c "targets $(LINK_TX).cpu0" -c halt -c "mww 0xE000ED0C 0x05FA0004" \
	   -c "targets $(LINK_RX).cpu0" -c halt -c "mww 0xE000ED0C 0x05FA0004" \
	   -c exit >/dev/null 2>&1; \
	 sleep 3; \
	 echo "loading the image on both boards (~40 s each over bitbang JTAG)..."; \
	 $(OCD2) -c init \
	   -c "targets $(LINK_TX).cpu0" -c halt -c "load_image $(LINK_ELF)" \
	     -c "mww 0x20000004 1" -c "reg sp 0x20020000" -c "reg pc 0x00000000" \
	   -c "targets $(LINK_RX).cpu0" -c halt -c "load_image $(LINK_ELF)" \
	     -c "mww 0x20000004 2" -c "reg sp 0x20020000" -c "reg pc 0x00000000" \
	   -c "echo {releasing TX, then RX}" \
	   -c "targets $(LINK_TX).cpu0" -c resume \
	   -c "targets $(LINK_RX).cpu0" -c resume \
	   -c "wait_halt 180000" \
	   -c "echo {=== RX beacon ===}" -c "mdw 0x20000000 44" \
	   -c "targets $(LINK_TX).cpu0" -c halt \
	   -c "echo {=== TX beacon ===}" -c "mdw 0x20000000 44" \
	   -c exit 2>&1 | python3 bench/analog_link_report.py

# Read both beacons without disturbing a run (or after one stalls).
link-read:
	@$(OCD2) -c init \
	   -c "targets $(LINK_RX).cpu0" -c halt \
	   -c "echo {=== RX beacon ===}" -c "mdw 0x20000000 44" \
	   -c "targets $(LINK_TX).cpu0" -c halt \
	   -c "echo {=== TX beacon ===}" -c "mdw 0x20000000 44" \
	   -c exit 2>&1 | python3 bench/analog_link_report.py

# --- sanitizers -----------------------------------------------------
# Every suite under AddressSanitizer + UndefinedBehaviorSanitizer, and
# the host-link parser under 40 MB of random input. The suites passing
# is NOT sufficient after a width or buffer change: three defects here
# were invisible to them and caught only this way (see
# .claude/skills/test-sanitize). -fno-sanitize-recover=all makes the
# first report fatal, so a failure cannot scroll past.
SAN_CFLAGS = -std=c99 -g -O1 -fsanitize=address,undefined \
             -fno-sanitize-recover=all -Isrc -Itests
SAN_ALL = src/arena.c src/fft.c src/dsp.c src/bits.c src/conv.c \
          src/packets.c src/tx.c src/broadcast.c src/rx_demod.c \
          src/rx_detect.c src/rx_stream.c src/ldpc.c src/link.c \
          src/station.c
SAN_SUITES = test_primitives test_bits test_tx test_rx test_stream \
             test_link test_broadcast test_usb

sanitize: $(GEN) | $(BUILD)
	@for s in $(SAN_SUITES); do \
	    if [ $$s = test_usb ]; then srcs=src/usb_proto.c; \
	    else srcs="$(SAN_ALL)"; fi; \
	    $(CC) $(SAN_CFLAGS) -o $(BUILD)/san_$$s tests/$$s.c $$srcs -lm \
	        || exit 1; \
	    printf '%-16s ' $$s; \
	    ./$(BUILD)/san_$$s 2>&1 | grep -E 'runtime error|ERROR:|passed,' \
	        | tail -1 || exit 1; \
	    ./$(BUILD)/san_$$s >/dev/null 2>&1 || exit 1; \
	done
	@$(CC) $(SAN_CFLAGS) -o $(BUILD)/san_fuzz_usb bench/fuzz_usb.c \
	    src/usb_proto.c && printf '%-16s ' fuzz_usb && ./$(BUILD)/san_fuzz_usb
	@echo "sanitize: clean"

# Flash-resident firmware: boots on its own from 0x08000000 and survives
# reset, unlike the RAM images (which inherit the resident firmware's
# clock via the JTAG load). target/startup_flash.c brings up the 400 MHz
# clock tree, FPU and vector table from cold. Program with:
#   openocd -f ../tools/esp32-probe/stm32h7-rbb.cfg -c init -c halt \
#     -c "program build/<img>.elf verify" -c exit
FLASHCC = /usr/bin/arm-none-eabi-gcc
FLASHFLAGS = -std=c99 -O2 -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 \
             -mfloat-abi=hard -ffunction-sections -fdata-sections \
             -Isrc -Itarget -nostartfiles -T target/stm32h743_flash.ld \
             -Wl,--gc-sections

heartbeat: | $(BUILD)
	$(FLASHCC) $(FLASHFLAGS) target/startup_flash.c bench/heartbeat_flash.c \
	    -o $(BUILD)/heartbeat_flash.elf
	@$(QEMUSIZE) $(BUILD)/heartbeat_flash.elf

# Flash-resident USB modem: enumerates on power-up, station behind it,
# SysTick clock (DWT is debugger-gated). Fetched trees, like usbfw.
USBFLASH_SRC = target/startup_flash.c usb/usb_flash_main.c \
               usb/usb_descriptors.c usb/bsp_stm32h7.c src/usb_proto.c \
               src/usb_modem.c src/arena.c src/station.c src/link.c \
               src/packets.c src/bits.c src/conv.c src/ldpc.c src/tx.c \
               src/fft.c src/dsp.c \
               $(TINYUSB)/src/tusb.c $(TINYUSB)/src/common/tusb_fifo.c \
               $(TINYUSB)/src/device/usbd.c \
               $(TINYUSB)/src/class/vendor/vendor_device.c \
               $(TINYUSB)/src/portable/synopsys/dwc2/dcd_dwc2.c \
               $(TINYUSB)/src/portable/synopsys/dwc2/dwc2_common.c

# The USB modem WITH a radio: the same station, but its PHY is the
# board's own DAC and ADC instead of a stub. Adds the receiver
# (rx_stream/rx_detect/rx_demod) to the USB image.
# Which modes the receiver listens for: bit0 NORMAL, bit1 ROBUST,
# bit2 EXTREME. Each open receiver detects over every sample, so this
# is a CPU budget -- 7 does not fit real time on this part.
RXMODES ?= 7
# ZC resync period inside a streamed burst, in blocks. 0 = open loop.
RESYNC ?= 4

USBRADIO_SRC = $(subst usb/usb_flash_main.c,usb/usb_radio_main.c,$(USBFLASH_SRC)) \
               src/rx_stream.c src/rx_detect.c src/rx_demod.c

radiofw: $(GEN) | $(BUILD)
	$(FLASHCC) $(FLASHFLAGS) -Iusb -I$(TINYUSB)/src -I$(CMSIS_H7)/Include \
	    -I$(CMSIS5)/CMSIS/Core/Include -DSTM32H743xx -DMAX_LLRS=1024 \
	    -DOFDM_RX_MODES=$(RXMODES) -DBURST_STREAM_RESYNC=$(RESYNC) \
	    $(USBRADIO_SRC) -lm -o $(BUILD)/usb_radio.elf
	@$(QEMUSIZE) $(BUILD)/usb_radio.elf

usbflash: $(GEN) | $(BUILD)
	$(FLASHCC) $(FLASHFLAGS) -Iusb -I$(TINYUSB)/src -I$(CMSIS_H7)/Include \
	    -I$(CMSIS5)/CMSIS/Core/Include -DSTM32H743xx \
	    $(USBFLASH_SRC) -lm -o $(BUILD)/usb_flash.elf
	@$(QEMUSIZE) $(BUILD)/usb_flash.elf

# --- programming and running on the board over the ESP32 JTAG probe ---
#
# These assume the bridge is up (`make bridge`). The board's flash holds
# the firmware; RAM images are loaded and run without touching flash.
# Every value the images need at reset is in target/startup_flash.c.
#
#   make bridge            start the JTAG bridge (autodetects the tty)
#   make flash-usb         build + flash the USB modem (survives reset)
#   make flash-heartbeat   build + flash the cold-boot proof
#   make run-armbench      reset, load a RAM image, run it, dump the beacon
#   make reset             SYSRESETREQ (re-boots whatever is flashed)
#   make backup-flash      save sector 0 before overwriting it
#   make restore-flash     write the backup back
#   make bridge-stop

OCD_CFG  ?= ../tools/esp32-probe/stm32h7-rbb.cfg
OCD       = openocd -f $(OCD_CFG)
BRIDGE    = ../tools/esp32-probe/rbb_bridge.py
BACKUP   ?= $(HOME)/Downloads/stm32h743_flash_sector0_backup.bin

# The reset-init event tries to set an adapter speed the bitbang probe
# has none of; that error is harmless and is filtered so a real failure
# still shows.
OCD_QUIET = 2>&1 | grep -avE 'Translation from khz|reset-init|startup.tcl'

bridge:
	@# Do NOT test this with ps/pgrep for the script name: this recipe
	@# expands $(BRIDGE) into its own command line, so any such pattern
	@# matches the recipe's own shell and it reports "already up" forever
	@# without ever starting anything. Ask the question we actually mean
	@# -- is something listening where OpenOCD will connect?
	@if ss -ltn 2>/dev/null | grep -q ':3335 '; then echo "bridge already up"; else \
	  port=$$(ls /dev/ttyUSB* 2>/dev/null | head -1); \
	  [ -n "$$port" ] || { echo "no /dev/ttyUSB* -- is the probe plugged in?"; exit 1; }; \
	  echo "bridge on $$port"; \
	  $(BRIDGE) --port $$port --baud 921600 --listen 3335 >/tmp/ofdm_bridge.log 2>&1 & \
	  for i in $$(seq 1 15); do grep -q waiting /tmp/ofdm_bridge.log && break; sleep 1; done; \
	  tail -1 /tmp/ofdm_bridge.log; \
	fi

bridge-stop:
	@pids=$$(ps -eo pid,cmd | grep '[r]bb_bridge' | awk '{print $$1}'); \
	 if [ -n "$$pids" ]; then for p in $$pids; do kill $$p 2>/dev/null; done; \
	   echo "bridge stopped ($$pids)"; else echo "no bridge running"; fi

reset:
	@$(OCD) -c init -c halt -c "mww 0xE000ED0C 0x05FA0004" -c exit $(OCD_QUIET) | tail -1; \
	  echo "reset issued (SYSRESETREQ)"

# program a flash image and verify. $1 = elf
define do_flash
	@echo "flashing $(1) ..."; \
	  $(OCD) -c init -c halt -c "program $(1) verify" -c exit $(OCD_QUIET) \
	    | grep -aE 'Verified OK|Error' || { echo "FLASH FAILED"; exit 1; }
endef

flash-heartbeat: heartbeat
	$(call do_flash,$(BUILD)/heartbeat_flash.elf)

flash-usb: usbflash
	$(call do_flash,$(BUILD)/usb_flash.elf)
	@echo "reset the board (make reset, or replug USB) to boot it"

# load a RAM image, run to its bkpt, dump the beacon. make run-armbench etc.
# clean reset first: RAM images inherit the resident clock and a stale
# peripheral/MPU state from the previous run wedges them.
run-%:
	@elf=$$(ls $(BUILD)/$*.elf $(BUILD)/$**.elf 2>/dev/null | head -1); \
	  [ -n "$$elf" ] || { echo "no $(BUILD)/$*.elf -- build it first"; exit 1; }; \
	  echo "reset + load $$elf"; \
	  $(OCD) -c init -c halt -c "mww 0xE000ED0C 0x05FA0004" -c exit $(OCD_QUIET) >/dev/null; \
	  sleep 3; \
	  $(OCD) -c init -c halt -c "load_image $$elf" -c "reg sp 0x20020000" \
	    -c "reg pc 0x00000000" -c resume -c "wait_halt 120000" \
	    -c "mdw 0x20000000 8" -c exit $(OCD_QUIET) \
	    | grep -aE 'downloaded|halted due|^0x2000'

backup-flash:
	@echo "saving sector 0 to $(BACKUP) (~3.5 min over the bitbang probe)"; \
	  $(OCD) -c init -c "dump_image $(BACKUP) 0x08000000 0x20000" -c exit $(OCD_QUIET) \
	    | grep -aE 'dumped|Error'

restore-flash:
	@[ -f $(BACKUP) ] || { echo "no backup at $(BACKUP)"; exit 1; }; \
	  echo "restoring $(BACKUP) to 0x08000000"; \
	  $(OCD) -c init -c halt -c "flash write_image erase $(BACKUP) 0x08000000 bin" \
	    -c exit $(OCD_QUIET) | grep -aE 'wrote|Error'

# --- two boards on one probe (JTAG daisy-chain) ------------------------
# Needs BOTH boards wired and powered; see stm32h7-rbb-dual.cfg for the
# wiring and tools/esp32-probe/README.md for why. `make ids` prints each
# chip's UID in USB-serial form, which is how a label maps to a board.
#
#   make ids           which label is which physical board
#   make beacons       USB-modem beacon from both
#   make flash-usb-a / flash-usb-b    flash one, leave the other running

DUAL_CFG ?= ../tools/esp32-probe/stm32h7-rbb-dual.cfg
OCD2      = openocd -f $(DUAL_CFG)

ids:
	@$(OCD2) -c init -c ab_ids -c exit $(OCD_QUIET) | grep -a 'uid/serial'

beacons:
	@$(OCD2) -c init -c ab_beacons -c exit $(OCD_QUIET) | grep -aE 'stmA|stmB'

# ab_flash uses verify_image, whose success is silent -- it throws on a
# mismatch, so reaching "flashed and reset" IS the verification. Do not
# grep for "Verified OK": that is `program`'s wording, it never appears
# here, and an empty grep fails the target on a successful flash.
flash-usb-a: usbflash
	@$(OCD2) -c init -c "ab_flash stmA build/usb_flash.elf" -c exit $(OCD_QUIET) \
	  | grep -aE 'flashed and reset|Error'

flash-usb-b: usbflash
	@$(OCD2) -c init -c "ab_flash stmB build/usb_flash.elf" -c exit $(OCD_QUIET) \
	  | grep -aE 'flashed and reset|Error'

flash-radio-a: radiofw
	@$(OCD2) -c init -c "ab_flash stmA build/usb_radio.elf" -c exit $(OCD_QUIET) \
	  | grep -aE 'flashed and reset|Error'

flash-radio-b: radiofw
	@$(OCD2) -c init -c "ab_flash stmB build/usb_radio.elf" -c exit $(OCD_QUIET) \
	  | grep -aE 'flashed and reset|Error'

# --- the ESP32 probe firmware itself -----------------------------------
# esp32 core 3.x needs arduino-cli; the legacy `arduino` IDE answers
# "Unknown package". Point ARDUINO_CLI at it if it is not on PATH.
ARDUINO_CLI ?= arduino-cli
PROBE_DIR    = ../tools/esp32-probe
PROBE_PORT  ?= $(shell ls /dev/ttyUSB* 2>/dev/null | head -1)

# Host test of the probe's GPIO layer: models the w1ts/w1tc registers and
# asserts the exact bits for every protocol character, in BOTH build
# arms. A missed call site would drive one board and not the other, which
# on a daisy-chain just looks like a dead scan chain -- so assert it here
# rather than discover it on the bench. No hardware needed.
test-probe:
	@for d in 1 0; do \
	   g++ -DDUAL_PROBE=$$d -I$(PROBE_DIR)/stub -I$(PROBE_DIR) -Wall -Wextra \
	       -o /tmp/test_pinmask$$d $(PROBE_DIR)/test_pinmask.cpp || exit 1; \
	   /tmp/test_pinmask$$d || exit 1; \
	 done

probe-fw:
	@$(ARDUINO_CLI) compile --fqbn esp32:esp32:esp32 $(PROBE_DIR)/esp32_rbb \
	  | tail -2

# Flashing the probe needs the serial port the bridge also wants.
flash-probe: test-probe
	@$(MAKE) --no-print-directory bridge-stop >/dev/null; \
	 echo "flashing probe on $(PROBE_PORT)"; \
	 $(ARDUINO_CLI) compile --fqbn esp32:esp32:esp32 -u -p $(PROBE_PORT) \
	   $(PROBE_DIR)/esp32_rbb | tail -3

.PHONY: bench armsize qemu armmeas zcab usbfw analogfw sanitize heartbeat usbflash bridge bridge-stop reset flash-heartbeat flash-usb backup-flash restore-flash ids beacons flash-usb-a flash-usb-b test-probe probe-fw flash-probe linkfw link-run link-read radiofw flash-radio-a flash-radio-b
