
all: hween


hween: hween.cpp
	g++ hween.cpp -L ~/src/smi_leds/build -lsmi_leds -I ~/src/smi_leds/include -o hween -g

format:
	clang-format -i hween.cpp

clean:
	rm -f hween
	rm -f example

