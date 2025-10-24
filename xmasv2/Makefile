
all: hween

hween: hween.cpp tree.h color.h Makefile perlin.o
	g++ -std=c++20 hween.cpp -L ~/src/smi_leds/build -lsmi_leds -I ~/src/smi_leds/include perlin.o -o hween -O2

perlin.o:
	g++ -std=c++20 -c perlin.cpp

format:
	clang-format -i hween.cpp

clean:
	rm -f hween
	rm -f example

