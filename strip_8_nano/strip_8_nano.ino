#include <FastLED.h>
#include <math.h>

#include <algorithm>
#include <vector>
#include <deque>

#define LED0_PIN 7
#define NUM_LEDS_PER_STRIP 8
#define NUM_STRIPS 1
#define NUM_LEDS (NUM_LEDS_PER_STRIP * NUM_STRIPS)
#define COLOR_PERIOD 730
#define CYLON_PERIOD 500

using namespace std;

CRGB strip[NUM_LEDS];

uint32_t frame = 0;

CRGB& pix(int index) {
  return strip[index];
}


struct RandColorQueue {
  RandColorQueue(int size, uint32_t period_ms)
    : period(period_ms) {
    for (int i = 0; i < size; i++) {
      c.push_back(randColor());
    }
  }

  uint8_t update(uint32_t time_ms) {
    if ((time_ms / period) != (ms / period)) {
      c.pop_front();
      c.push_back(randColor());
    }
    ms = time_ms;
    return ((ms % period) * 255) / period;
  }

  static CRGB randColor() {
    CRGB c;
    c.setHSV(rand() & 255, (rand() & 63) + 191, rand() & 255);
    return c;
  }

  deque<CRGB> c;
  const uint32_t period;
  uint32_t ms;
};

RandColorQueue cq(2, COLOR_PERIOD);

void setup() {

  FastLED.addLeds<WS2812, LED0_PIN, GRB>(strip, NUM_LEDS_PER_STRIP);
  FastLED.setBrightness(64);
  for (int i = 0; i < NUM_LEDS; i++) {
    strip[i] = CRGB::Black;
  }
}

// triangle wave between 0.0f and 1.0f
float triangle(uint32_t time, uint32_t period) {

  return abs(2.0f * ((time % period) / float(period - 1)) - 1.0f);
}

void loop() {
  uint32_t ms = millis();

  uint8_t cfrac = cq.update(ms);
  CRGB col = cq.c[0].lerp8(cq.c[1], cfrac);

  float v = triangle(ms, CYLON_PERIOD);

  for (int i = 0; i < NUM_LEDS; i++) {
    float fpos = float(i) / (NUM_LEDS - 1);
    float fdist = max(0.0f, 3.0f * (1.0f - abs(v - fpos)) - 2.0f);
    float func = pow(fdist, 4.0f);
    pix(i) = col.scale8(255 * func);
  }

  FastLED.show();
}