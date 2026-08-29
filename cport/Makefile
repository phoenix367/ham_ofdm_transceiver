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

.PHONY: bench armsize qemu armmeas zcab usbfw analogfw sanitize heartbeat
