# Console demo app over the virtual channel driver.
# The C stack comes from ../cport (generated headers included).

CC      = gcc
# larger station buffers than the MCU-modest cport defaults: file
# transfers up to ~4 KB per message
# BURST_STREAM_MAX raised from the MCU-modest cport default of 8: it
# sizes two static buffers (~4.7 kB per block), and the measured win is
# real -- a 14 KB transfer goes from 9 transmissions to 6.
# -ffunction-sections/-fdata-sections + --gc-sections: the app builds its
# waveform through the streaming transmitter, so the frame-at-once path
# (and tx.c's 4.3 MB g_sig with it) is unreferenced and the linker drops
# it. Without section GC the linker pulls in whole objects and the buffer
# stays, which is what made it invisible in earlier size measurements.
CFLAGS  = -std=c99 -Wall -Wextra -O2 -ffunction-sections -fdata-sections \
          -DST_MSG_MAX=4096 -DST_ASM_MAX=8192 \
          -DBURST_STREAM_MAX=16
LDFLAGS += -Wl,--gc-sections
BUILD   = build
CPORT   = ../cport

CSRC = $(CPORT)/src/arena.c $(CPORT)/src/fft.c $(CPORT)/src/dsp.c $(CPORT)/src/bits.c \
       $(CPORT)/src/conv.c $(CPORT)/src/ldpc.c $(CPORT)/src/packets.c \
       $(CPORT)/src/tx.c $(CPORT)/src/rx_demod.c $(CPORT)/src/rx_detect.c \
       $(CPORT)/src/rx_stream.c $(CPORT)/src/link.c $(CPORT)/src/station.c

all: $(BUILD)/ofdm_console

$(CPORT)/src/rom_tables.h:
	$(MAKE) -C $(CPORT) src/rom_tables.h

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/ofdm_console: app.c $(CSRC) $(CPORT)/src/rom_tables.h | $(BUILD)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ app.c $(CSRC) -lm -lz

clean:
	rm -rf $(BUILD)

.PHONY: all clean
