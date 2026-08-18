# Console demo app over the virtual channel driver.
# The C stack comes from ../cport (generated headers included).

CC      = gcc
# larger station buffers than the MCU-modest cport defaults: file
# transfers up to ~4 KB per message
CFLAGS  = -std=c99 -Wall -Wextra -O2 -DST_MSG_MAX=4096 -DST_ASM_MAX=8192
BUILD   = build
CPORT   = ../cport

CSRC = $(CPORT)/src/fft.c $(CPORT)/src/dsp.c $(CPORT)/src/bits.c \
       $(CPORT)/src/conv.c $(CPORT)/src/ldpc.c $(CPORT)/src/packets.c \
       $(CPORT)/src/tx.c $(CPORT)/src/rx_demod.c $(CPORT)/src/rx_detect.c \
       $(CPORT)/src/rx_stream.c $(CPORT)/src/link.c $(CPORT)/src/station.c

all: $(BUILD)/ofdm_console

$(CPORT)/src/rom_tables.h:
	$(MAKE) -C $(CPORT) src/rom_tables.h

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/ofdm_console: app.c $(CSRC) $(CPORT)/src/rom_tables.h | $(BUILD)
	$(CC) $(CFLAGS) -o $@ app.c $(CSRC) -lm

clean:
	rm -rf $(BUILD)

.PHONY: all clean
