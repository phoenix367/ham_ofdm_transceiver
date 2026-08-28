# C port of the fixed-point model -- host build + golden-vector tests.
# See ../docs/c-port-plan.md.

CC      = gcc
CFLAGS  = -std=c99 -Wall -Wextra -Werror -O2
BUILD   = build

SRC     = src/fft.c src/dsp.c src/bits.c src/conv.c src/packets.c src/tx.c \
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
	$(CC) $(CFLAGS) -o $@ tests/test_tx.c src/tx.c src/packets.c \
	    src/bits.c src/conv.c src/ldpc.c src/fft.c

$(BUILD)/test_link: tests/test_link.c $(SRC) $(GEN) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ tests/test_link.c src/link.c src/station.c \
	    src/rx_demod.c src/rx_detect.c src/ldpc.c src/tx.c src/packets.c \
	    src/bits.c src/conv.c src/fft.c src/dsp.c -lm

$(BUILD)/test_stream: tests/test_stream.c $(SRC) $(GEN) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ tests/test_stream.c src/rx_stream.c src/rx_demod.c \
	    src/rx_detect.c src/ldpc.c src/tx.c src/packets.c src/bits.c \
	    src/conv.c src/fft.c src/dsp.c

$(BUILD)/test_broadcast: tests/test_broadcast.c $(SRC) $(GEN) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ tests/test_broadcast.c src/broadcast.c \
	    src/rx_demod.c src/rx_detect.c src/ldpc.c src/tx.c src/packets.c \
	    src/bits.c src/conv.c src/fft.c src/dsp.c src/link.c src/station.c -lm

$(BUILD)/test_rx: tests/test_rx.c $(SRC) $(GEN) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ tests/test_rx.c src/rx_demod.c src/rx_detect.c src/rx_stream.c src/ldpc.c \
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

.PHONY: bench armsize qemu
