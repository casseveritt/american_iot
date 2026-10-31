# Builds idseq8 and idseq16 by compiling the smi_leds driver directly with
# the matching LED_NCHANS (same approach as ../../ledtest/Makefile), plus
# idseq_host, a hardware-free build that only does --print and --dump.

SMI_LEDS_DIR = $(HOME)/src/smi_leds
SMI_SRC = $(SMI_LEDS_DIR)/smi_leds/rpi_pixleds_lib.c \
          $(SMI_LEDS_DIR)/smi_leds/rpi_dma_utils.c
RPI_OPTS := $(shell cd $(SMI_LEDS_DIR) 2>/dev/null && python3 ./detect_rpi.py | tail -1)
CFLAGS = -O2 -I $(SMI_LEDS_DIR)/include $(RPI_OPTS)

all: idseq8 idseq16

idseq8: NCHANS = 8
idseq16: NCHANS = 16

idseq8 idseq16: idseq.cpp $(SMI_SRC) Makefile
	gcc $(CFLAGS) -DLED_NCHANS=$(NCHANS) -c $(SMI_LEDS_DIR)/smi_leds/rpi_pixleds_lib.c -o lib$(NCHANS).o
	gcc $(CFLAGS) -DLED_NCHANS=$(NCHANS) -c $(SMI_LEDS_DIR)/smi_leds/rpi_dma_utils.c -o dma$(NCHANS).o
	g++ -std=c++20 $(CFLAGS) idseq.cpp lib$(NCHANS).o dma$(NCHANS).o -lm -o $@
	rm -f lib$(NCHANS).o dma$(NCHANS).o

host: idseq_host

idseq_host: idseq.cpp Makefile
	g++ -std=c++20 -O2 -Wall -DLEDMAP_NO_HW idseq.cpp -o $@

format:
	clang-format -i idseq.cpp

clean:
	rm -f idseq8 idseq16 idseq_host *.o

.PHONY: all host format clean
