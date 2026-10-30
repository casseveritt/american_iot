# Builds ledtest8 and ledtest16 by compiling the smi_leds driver directly
# with the matching LED_NCHANS, so neither depends on how
# ~/src/smi_leds/build was configured.

SMI_LEDS_DIR = $(HOME)/src/smi_leds
SMI_SRC = $(SMI_LEDS_DIR)/smi_leds/rpi_pixleds_lib.c \
          $(SMI_LEDS_DIR)/smi_leds/rpi_dma_utils.c
RPI_OPTS := $(shell cd $(SMI_LEDS_DIR) && python3 ./detect_rpi.py | tail -1)
CFLAGS = -O2 -I $(SMI_LEDS_DIR)/include $(RPI_OPTS)

all: ledtest8 ledtest16

ledtest8: NCHANS = 8
ledtest16: NCHANS = 16

ledtest8 ledtest16: ledtest.cpp $(SMI_SRC) Makefile
	gcc $(CFLAGS) -DLED_NCHANS=$(NCHANS) -c $(SMI_LEDS_DIR)/smi_leds/rpi_pixleds_lib.c -o lib$(NCHANS).o
	gcc $(CFLAGS) -DLED_NCHANS=$(NCHANS) -c $(SMI_LEDS_DIR)/smi_leds/rpi_dma_utils.c -o dma$(NCHANS).o
	g++ -std=c++20 $(CFLAGS) ledtest.cpp lib$(NCHANS).o dma$(NCHANS).o -lm -o $@
	rm -f lib$(NCHANS).o dma$(NCHANS).o

format:
	clang-format -i ledtest.cpp

clean:
	rm -f ledtest8 ledtest16 *.o

.PHONY: all format clean
