#include <FastLED.h>
#include <math.h>

#include <vector>
#include <deque>
#include <algorithm>

#define LED0_PIN     7
#define LED1_PIN     8
#define NUM_LEDS_PER_STRIP    598
#define NUM_STRIPS   2
#define NUM_LEDS     (NUM_LEDS_PER_STRIP * NUM_STRIPS)
#define PERIOD       250

using namespace std;

using Pair = pair<short, short>;

// unused now, delete
vector<Pair> deadRange = {
  {0, 11},
  {193, 205},
  {388, 401},
  {583, 628},
  {809, 822},
  {1002, 1015},
};

// Tree has 12 "spokes" each with about 90 lights.
// Each strip has 6 spokes. Even spokes go up from
// the bottom, odd spokes go down from the top.
// Strip 0 spokes increase clockwise.
// Strip 1 spokes increase counter-clockwise.
// The strips start at adjacent spokes, go in opposite
// directions, and meet at adjacent spokes half way around
// the tree.

vector<Pair> segment = {
  {11, 102},
  {193, 101},
  {205, 296},
  {387, 295},
  {401, 492},
  {582, 491},
  {628, 719},
  {808, 718},
  {822, 912},
  {1001, 911},
  {1015, 1105},
  {1195, 1104}
};

const CRGB colors[] = 
  { CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::Orange, CRGB::Cyan, CRGB::Magenta, CRGB::Yellow, CRGB::White };

CRGB strip[NUM_LEDS];

uint32_t timestamp[3];
int idx[3];
uint32_t iteration = 0;

int pixaddr(int index) {
  if (index >= NUM_LEDS_PER_STRIP) {
    // second strip runs in the opposite direction from the first
    return NUM_LEDS - 1 - (index - NUM_LEDS_PER_STRIP);
  } else {
    return index;
  }
}

CRGB& pix(int index) {
  return strip[pixaddr(index)];
}

deque<CRGB> c;

CRGB randColor() {
  CRGB c;
  c.setHSV(rand() & 255, (rand() & 63) + 191, (rand() & 127) + 128);
  return c;
}

void setup() {

  FastLED.addLeds<WS2812, LED0_PIN, GRB>(strip, NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812, LED1_PIN, GRB>(strip + NUM_LEDS_PER_STRIP, NUM_LEDS_PER_STRIP);     
  FastLED.setBrightness(8);
  for (int i = 0; i < NUM_LEDS; i++) {
    strip[i] = CRGB::Black;
  }
  for (int i = 0; i < 102; i++) {
    c.push_back(randColor());
  }
}

template <typename T>
T clamp(T val, T lo, T hi) {
  return max(min(val, hi), lo);
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

  int incr[] = {1, -1};
  for (int i = 0; i < 100; i++) {
    float baseOffset = i / 3.0f + float(ifrac) / PERIOD;
    float frac = baseOffset - floor(baseOffset);

    int cidx = clamp(int(baseOffset), 0, 100);
    CRGB c0 = c[cidx];
    CRGB c1 = c[cidx+1];
    auto col = c0.lerp8(c1, 255 * frac);
    for (int j = 0; j < 12; j++) {
      auto& seg = segment[j];
      short lo = min(seg.first, seg.second);
      short hi = max(seg.first, seg.second);
      int v = clamp(short(seg.first + incr[j&1] * i), lo, hi);
      if (v != seg.second) {
        pix(v) = col;
      }
    }
  }

  FastLED.show();
  iteration++;
}