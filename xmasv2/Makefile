HWEEN_OBJS = hween.o perlin.o cmap.o

SMI_LEDS_DIR = ~/src/smi_leds
SMI_LEDS_INCLUDE = -I $(SMI_LEDS_DIR)/include
SMI_LEDS_LIB = -L $(SMI_LEDS_DIR)/build -lsmi_leds

all: hween

hween: $(HWEEN_OBJS) tree.h color.h Makefile
	g++ -std=c++20 $(HWEEN_OBJS) $(SMI_LEDS_LIB) -o hween -O2

%.o: %.cpp
	g++ -std=c++20 $(SMI_LEDS_INCLUDE) -c $< -o $@

format:
	clang-format -i hween.cpp
	clang-format -i cmap.h
	clang-format -i color.h
	clang-format -i tree.h
	clang-format -i perlin.cpp
	clang-format -i perlin.h


clean:
	rm -f hween
	rm -f example
	rm -f $(HWEEN_OBJS)

