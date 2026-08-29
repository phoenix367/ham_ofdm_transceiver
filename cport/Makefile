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

$(BUILD)/test_rx: tests/test_rx.c $(SRC) $(GEN) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ tests/test_rx.c src/arena.c src/rx_demod.c src/rx_detect.c src/rx_stream.c src/ldpc.c \
          src/link.c src/station.c \
	    src/tx.c src/packets.c src/bits.c src/conv.c src/fft.c src/dsp.c

test: $(BUILD)/test_primitives $(BUILD)/test_bits $(BUILD)/test_tx $(BUILD)/test_rx $(BUILD)/test_stream $(BUILD)/test_link $(BUILD)/test_broadcast
	./$(BUILD)/test_primitives
	./$(BUILD)/test_bits
	./$(BUILD)/test_tx
	./$(BUILD)/test_rx
	./$(BUILD)/test_stream
	./$(BUILD)/test_link
	./$(BUILD)/test_broadcast

clean:
	rm -rf $(BUILD)

distclean: clean
	rm -f $(GEN)

.PHONY: all test clean distclean

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

.PHONY: bench armsize qemu armmeas zcab
