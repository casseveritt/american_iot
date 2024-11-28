#include <FastLED.h>
#include <math.h>

#include <algorithm>
#include <vector>
#include <deque>

#define LED0_PIN     7
#define NUM_LEDS_PER_STRIP    8
#define NUM_STRIPS   1
#define NUM_LEDS     (NUM_LEDS_PER_STRIP * NUM_STRIPS)
#define PERIOD       250

using namespace std;

using Pair = pair<short, short>;

const CRGB colors[] = 
  { CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::Orange, CRGB::Cyan, CRGB::Magenta, CRGB::Yellow, CRGB::White };

CRGB strip[NUM_LEDS];

uint32_t timestamp[3];
int idx[3];
uint32_t iteration = 0;

CRGB& pix(int index) {
  return strip[index];
}

deque<CRGB> c;

CRGB randColor() {
  CRGB c;
  c.setHSV(rand() & 255, (rand() & 63) + 191, (rand() & 127) + 128);
  return c;
}

void setup() {

  FastLED.addLeds<WS2812, LED0_PIN, GRB>(strip, NUM_LEDS_PER_STRIP);
  FastLED.setBrightness(32);
  for (int i = 0; i < NUM_LEDS; i++) {
    strip[i] = CRGB::Black;
  }
  for (int i = 0; i < 10; i++) {
    c.push_back(randColor());
  }
}

template <typename T>
T clamp(T val, T lo, T hi) {
  return min(max(val, lo), hi);
}

void loop() {
  idx[0] = iteration % 3;
  idx[1] = (iteration + 2) % 3;
  idx[2] = (iteration + 1) % 3;
  uint32_t ms = timestamp[idx[0]] = millis();

  int offset = ms / PERIOD;
  int ifrac = ms % PERIOD;

  if (ifrac < (timestamp[idx[1]]) % PERIOD) {
    c.pop_front();
    c.push_back(randColor());
  }

  for (int i = 0; i < 8; i++) {
    float baseOffset = i + float(ifrac) / PERIOD;
    float frac = baseOffset - floor(baseOffset);
    int cidx = clamp(int(baseOffset), 0, 10);

    CRGB c0 = c[cidx];
    CRGB c1 = c[cidx+1];
    CRGB col = c0.lerp8(c1, fract8(frac*255));
    pix(i) = col;
  }

  FastLED.show();
  iteration++;
}