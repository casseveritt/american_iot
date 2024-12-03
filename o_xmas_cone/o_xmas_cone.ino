#include "linear.h"

#include <FastLED.h>
#include <math.h>

#include <vector>
#include <deque>
#include <algorithm>

#define LED0_PIN 7
#define LED1_PIN 8
#define NUM_LEDS_PER_STRIP 598
#define NUM_STRIPS 2
#define NUM_LEDS (NUM_LEDS_PER_STRIP * NUM_STRIPS)
#define PERIOD 250
#define MODE 2

using namespace std;
using namespace r3;

using Pair = pair<short, short>;

// Tree has 12 "spokes" each with about 90 lights.
// Each strip has 6 spokes. Even spokes go up from
// the bottom, odd spokes go down from the top.
// Strip 0 spokes increase clockwise.
// Strip 1 spokes increase counter-clockwise.
// The strips start at adjacent spokes, go in opposite
// directions, and meet at adjacent spokes half way around
// the tree.

vector<Pair> segment = {
  { 11, 102 },
  { 193, 101 },
  { 205, 296 },
  { 387, 295 },
  { 401, 492 },
  { 582, 491 },
  { 628, 719 },
  { 808, 718 },
  { 822, 912 },
  { 1001, 911 },
  { 1015, 1105 },
  { 1195, 1104 }
};

struct LedInfo {
  uint32_t index;
  Vec3f pos;
};

struct Sphere {
  Vec3f pos;
  float vel;
  float radius;
  int color;
  bool isInside(const Vec3f& p) const {
    return (p - pos).Length() < radius;
  }
};

vector<LedInfo> led;

const CRGB colors[] = { CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::Orange, CRGB::Cyan, CRGB::Magenta, CRGB::Yellow, CRGB::White };

CRGB strip[NUM_LEDS];

uint32_t timestamp[3];
int idx[3];
uint32_t iteration = 0;
int delayTime = 0;

vector<Sphere> spheres;
float yrot = 0.0f;

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
  FastLED.setBrightness(64);
  for (int i = 0; i < NUM_LEDS; i++) {
    strip[i] = CRGB::Black;
  }
  for (int i = 0; i < 102; i++) {
    c.push_back(randColor());
  }
  for (int j = 0; j < 12; j++) {
    float theta = (2.0f * M_PI * j) / 12.0f;
    Rotationf rot(Vec3f(0, 1, 0), -theta);
    float r_top = 0.04f;  // meters
    float r_bot = 0.31f;
    float height = 1.47f;
    //float slope = -1.47f / (r_bot - r_top);
    float metersPerLed = 1.0f / 60.0f;
    Vec3f dir(r_bot - r_top, -1.47f, 0);
    float stripLen = dir.Length();
    float sinStrip = height / stripLen;
    dir.Normalize();
    auto& seg = segment[j];
    float delta = abs(seg.second - seg.first);
    int incr = (j & 1) ? -1 : 1;
    for (int i = seg.first; i != seg.second; i += incr) {
      led.push_back(LedInfo());
      auto& li = led.back();
      li.index = pixaddr(i);
      auto& p = li.pos;
      float step = abs(i - seg.first);
      float frac = step / delta;
      float ledDistAlongStrip = step * metersPerLed;
      p.x = frac * (r_bot - r_top) + r_top;
      p.y = sinStrip * ledDistAlongStrip;
      p.z = 0;
      p = rot.Rotate(p);
    }
  }
}

template<typename T>
T clamp(T val, T lo, T hi) {
  return max(min(val, hi), lo);
}

void loop() {
  if (MODE == 2) {
    Sphere s = Sphere();
    s.pos = Vec3f(0, 0.75, 0);
    s.radius = 0.5;
    for (const auto& li : led) {
      CRGB& pc = strip[li.index];
      pc = s.isInside(li.pos) ? CRGB(32, 0, 0) : CRGB::Black;
    }
  }
#if 0
  if (MODE == 0) { // Spheres

    Sphere s = Sphere();
    s.pos = Vec3f(0, 0.75, 0);
    s.radius = 0.5;
    for (const auto& li : led) {
      CRGB& pc = strip[li.index];
      if ((s.pos-li.pos).Length() <= s.radius) {
        pc = CRGB(1.0f, 0.0f, 0.0f);
      }
    }
  
    uint32_t ms = timestamp[idx[0]] = millis();
    if (ms >= delayTime) {
      Sphere s = Sphere();
      s.pos = Vec3f(float(rand() % 51) / 100.0, -0.5, float(rand() % 51) / 100.0);
      s.vel =  float((rand() % 41) + 10) / 1000;
      s.radius =  float((rand() % 21) + 10) / 100;
      s.color = rand() % 3;
      spheres.push_back(s);
      delayTime = int(ms) + ((rand() % 900) + 100);
    }
    for (int i=0;i<spheres.size();i++){
      Sphere& s = spheres[i];
      s.pos.y += s.vel;
      //if (s.pos.y >= 2.0) { // Remove spheres over the tree
      //  spheres.erase(std::remove(spheres.begin(), spheres.end(), i), spheres.end());
      //}
      for (const auto& li : led) {
        CRGB& pc = strip[li.index];
        if ((s.pos-li.pos).Length() <= s.radius) {
          if (s.color == 0){
            pc = CRGB(1.0f, 0.0f, 0.0f);
          } else if (s.color == 1) {
            pc = CRGB(0.0f, 1.0f, 0.0f);
          } else {
            pc = CRGB(0.0f, 0.0f, 1.0f);
          }
        }
      }
    }
  }
  if (MODE == 1) {
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

    Vec3f center(0, 0.75f, 0);

    for (const auto& li : led) {
      Vec3f p = li.pos - center;
      p.y = 0;
      p.Normalize();
      p = Rotationf(Vec3f(0, 1, 0), yrot).Rotate(p);
      p.x = pow(p.x, 5.0f);
      p.z = pow(p.z, 5.0f);

      p *= 25;
      CRGB& pc = strip[li.index];
      pc = CRGB(max(0.0f, p.x), max(0.0f, p.y), max(0.0f, p.z));
    }

    yrot += 2.0f * M_PI / 60.0f;
#endif
  FastLED.show();
  iteration++;
}