
all: hween

hween: hween.cpp tree.h color.h Makefile
	g++ -std=c++20 hween.cpp -L ~/src/smi_leds/build -lsmi_leds -I ~/src/smi_leds/include -o hween -g

format:
	clang-format -i hween.cpp

clean:
	rm -f hween
	rm -f example

