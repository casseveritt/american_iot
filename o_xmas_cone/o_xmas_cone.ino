#include "linear.h"
#include "perlin.h"

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
// for mode dev
// #define FORCE_MODE 2

constexpr float k2pi = 2.0f * M_PI;

int mode;
int32_t countdown = -1;
uint32_t showTime = 0;
uint32_t newProgStartMs = 0;

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
  float radius;
  float radiusSquared;
  float vel;
  CRGB color;

  Sphere() {}
  Sphere(CRGB colorIn) {
    color = colorIn;
  }
  void setPos(float offset = -1.35) {  // Set position of sphere along the XY plane of the cube
    pos = Vec3f(float((rand() % 63) - 31) / 100.0f, float(rand() % 141) / 100.0f, offset);
  }
  void setRad(int upper, int lower) {  // Set radius of sphere in centemeters
    radius = float((rand() % (upper - lower + 1)) + lower) / 100.0f;
    radiusSquared = pow(radius, 2);
  }
  void setVel(int min, int max) {  // Set sphere velocity in thousandths of a meter per second
    vel = float((rand() % (max - min + 1)) + min) / 1000.0f;
  }
  void setCol(CRGB colorIn) {
    color = colorIn;
  }
  void stepForward(int frameTime) {
    pos.z += vel * (frameTime / 25);
  }
  float dist(const Vec3f& p) {
    return (p - pos).Length();
  }
  bool isInside(const Vec3f& p) const {
    return (p - pos).LengthSquared() < radiusSquared;
  }
};

vector<LedInfo> led;

const CRGB colors[] = { CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::Orange, CRGB::Cyan, CRGB::Magenta, CRGB::Yellow, CRGB::White };

CRGB strip[NUM_LEDS];

int delayTime = 0;
int startTime = 0;

struct FrameTime {

  void update() {
    idx[0] = iteration % 3;
    idx[1] = (iteration + 2) % 3;
    idx[2] = (iteration + 1) % 3;
    timestamp[idx[0]] = millis();
    iteration++;
  }

  uint32_t t0() const {
    return timestamp[idx[0]];
  }

  uint32_t t1() const {
    return timestamp[idx[1]];
  }

  uint32_t dt() const {
    return t0() - t1();
  }

  uint32_t timestamp[3];
  int idx[3];
  uint32_t iteration = 0;
};

FrameTime frameTime;

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

static CRGB randColor() {
  CRGB c;
  c.setHSV(rand() & 255, (rand() & 63) + 191, (rand() & 127) + 128);
  return c;
}

struct ColorMap {

  ColorMap() {}

  void clear() {
    cm.clear();
  }

  void addColor(CRGB c, uint8_t scale = 255) {
    cm.push_back(c.scale8(scale));
  }

  CRGB lookup(float t) const {
    t = clamp(t, 0.0f, 0.999f);
    const float ts = t * cm.size();
    const int ti = ts;
    const float tfr = ts - ti;
    return cm[ti].lerp8(cm[(ti + 1) % cm.size()], uint8_t(255 * tfr));
  }

  vector<CRGB> cm;
};

ColorMap cm1;
ColorMap rgMap;

void setup() {

  FastLED.addLeds<WS2812, LED0_PIN, GRB>(strip, NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812, LED1_PIN, GRB>(strip + NUM_LEDS_PER_STRIP, NUM_LEDS_PER_STRIP);
  FastLED.setBrightness(255);

  cm1.addColor(CRGB::Maroon, 32);
  cm1.addColor(CRGB::Maroon, 32);
  cm1.addColor(CRGB::Maroon, 8);
  cm1.addColor(CRGB::Grey, 8);
  cm1.addColor(CRGB::Maroon, 8);
  cm1.addColor(CRGB::Maroon, 32);
  cm1.addColor(CRGB::Grey, 8);
  cm1.addColor(CRGB::Grey);
  cm1.addColor(CRGB::Maroon, 8);

  rgMap.addColor(CRGB::Red, 8);
  rgMap.addColor(CRGB::Red, 8);
  rgMap.addColor(CRGB::Red, 8);
  rgMap.addColor(CRGB::Red, 64);
  rgMap.addColor(CRGB::Red, 8);
  rgMap.addColor(CRGB::Red, 8);
  rgMap.addColor(CRGB::Red, 8);

  rgMap.addColor(CRGB::Green, 8);
  rgMap.addColor(CRGB::Green, 8);
  rgMap.addColor(CRGB::Green, 8);
  rgMap.addColor(CRGB::Green, 64);
  rgMap.addColor(CRGB::Green, 8);
  rgMap.addColor(CRGB::Green, 8);
  rgMap.addColor(CRGB::Green, 8);

  for (int i = 0; i < NUM_LEDS; i++) {
    strip[i] = CRGB::Black;
  }
  for (int j = 0; j < 12; j++) {
    float theta = (k2pi * j) / 12.0f;
    Rotationf rot(Vec3f(0, 1, 0), -theta);
    float r_top = 0.04f;  // meters
    float r_bot = 0.31f;
    float height = 1.47f;
    //float slope = -1.47f / (r_bot - r_top);
    float metersPerLed = 1.0f / 60.0f;
    Vec3f dir(r_top - r_bot, 1.47f, 0);
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
      p.x = (1.0f - frac) * (r_bot - r_top) + r_top;
      p.y = sinStrip * ledDistAlongStrip;
      p.z = 0;
      p = rot.Rotate(p);
    }
  }
  srand(analogRead(A0) + millis());
}

template<typename T>
T clamp(T val, T lo, T hi) {
  return max(min(val, hi), lo);
}

int spheresRotation = 1100;

void setDelayTime(uint32_t ms, int max, int min) {
  delayTime = int(ms) + ((rand() % (max - min + 1)) + min);
}

void loop_sphere() {
  uint32_t ms = frameTime.t0();
  if (ms >= delayTime && spheres.size() < 50) {
    Sphere s = Sphere(randColor());
    s.setPos();
    s.setVel(5, 15);
    s.setRad(3, 10);
    spheres.push_back(s);
    setDelayTime(ms, 333, 200);
  }

  int timeTaken = ms - startTime;
  for (int i = 0; i < spheres.size(); i++) {  // Moves all the spheres
    Sphere& s = spheres[i];
    s.stepForward(timeTaken);
  }
  startTime = ms;

  for (int i = 0; i < spheres.size(); i++) {  // Culls spheres outside the bounds of the tree
    Sphere& s = spheres[i];
    if (s.pos.z > (0.31f + s.radius)) {
      Sphere tempS = spheres[spheres.size() - 1];
      spheres[spheres.size() - 1] = spheres[i];
      spheres[i] = tempS;
      spheres.pop_back();
    }
  }

  Rotationf rotworld(Vec3f(0, 1, 0), ToRadians(float(spheresRotation) / 10));  // Rotates effect about the Y-axis
  spheresRotation = (spheresRotation + 2) % (360 * 10);                        // Rate of rotation

  for (auto li : led) {
    CRGB& pc = strip[li.index];
    bool outsideSpheres = true;
    float minDist = 100000;
    li.pos = rotworld.Rotate(li.pos);
    for (int i = 0; i < spheres.size(); i++) {
      Sphere& s = spheres[i];
      if (s.isInside(li.pos)) {
        float dist = s.dist(li.pos);
        if (dist < minDist) {
          minDist = dist;
          outsideSpheres = false;
          pc = s.color;
          pc.nscale8(uint8_t(255 * pow(1.0 - dist / s.radius, 1.5f)));
        }
      }
    }
    if (outsideSpheres) pc = CRGB::Black;  // Clear non-intersecting LEDs
  }
}

void rot_y() {
  Vec3f center(0, 0.75f, 0);

  for (const auto& li : led) {
    Vec3f p = li.pos - center;
    p.y = 0;
    p.Normalize();
    p = Rotationf(Vec3f(0, 1, 0), yrot).Rotate(p);
    p.x *= p.x * p.x * p.x * p.x;  // pow(p.x, 5.0f);
    p.z *= p.z * p.z * p.z * p.z;  // pow(p.z, 5.0f);

    p *= 25;
    CRGB& pc = strip[li.index];
    pc = CRGB(max(0.0f, p.x), max(0.0f, p.y), max(0.0f, p.z));
  }

  yrot = fmod(yrot + k2pi / 60.0f, k2pi);
}

void twist() {
  auto ms = frameTime.t0();

  float twistRadsPerMeter = k2pi * 2.5f * (sin(fmod((k2pi * ms) / 12000.0f, k2pi) + 1.0));

  for (const auto& li : led) {
    Vec3f p = li.pos;
    p = Rotationf(Vec3f(0, 1, 0), -twistRadsPerMeter * p.y + yrot).Rotate(p);
    CRGB& pc = strip[li.index];
    pc = p.x > 0 ? CRGB::Blue : CRGB::Black;
    pc.nscale8(8);
  }
  yrot = fmod(yrot + k2pi / 60.0f, k2pi);
}

void screw() {
  auto ms = frameTime.t0();
  float twistRadsPerMeter = k2pi * 2.5f;  //(sin((k2pi * ms) / 12000.0f) + 1.0);

  for (const auto& li : led) {
    Vec3f p = li.pos;
    float y = 1.5f * pow(p.y / 1.5f, 2.0f);
    p = Rotationf(Vec3f(0, 1, 0), -twistRadsPerMeter * y + yrot).Rotate(p);
    p.y = 0;
    p.Normalize();
    float x = atan2(p.x, p.z) / k2pi + 0.5f;
    CRGB& pc = strip[li.index];
    pc = rgMap.lookup(x);
  }
  yrot = fmod(yrot + k2pi / 45.0f, k2pi);
}

void clear(CRGB color) {
  for (const auto& li : led) {
    CRGB& pc = strip[li.index];
    pc = color;
    pc.nscale8(8);
  }
}

void perlin() {
  uint32_t dt = (frameTime.t0() - newProgStartMs);
  Quaternionf q(Vec3f(0, 1, 0), ToRadians(21.0f));
  for (const auto& li : led) {
    CRGB& pc = strip[li.index];
    Vec3f p = q.Rotate(li.pos * 4);
    p.y *= 2.0f;
    float noise = abs(ImprovedNoise::noise(p.x - (dt * 0.0003f), p.y, p.z));
    pc = cm1.lookup(noise);
  }
}

void perlin_flashing() {
  uint32_t dt = (frameTime.t0() - newProgStartMs);
  static uint32_t trappedCondition = 0;
  uint32_t approxIteration = trappedCondition > 0 ? trappedCondition : (dt * 255) / 1000;
  int bigDeltas = 0;
  Quaternionf q(Vec3f(0, 1, 0), ToRadians(0.01f));
  for (const auto& li : led) {
    CRGB& pc = strip[li.index];
    Vec3f p = li.pos * 7;
    p = q.Rotate(p);
    float noise = abs(ImprovedNoise::noise(p.x, p.y, p.z - approxIteration * 0.01f));
    float noise2 = abs(ImprovedNoise::noise(p.x, p.y, p.z - (approxIteration - 1) * 0.01f));
    float noise3 = noise - noise2;
    noise3 = abs(noise3) - 0.2f;
    noise3 = clamp(noise3, 0.0f, 1.0f);
    if (noise3 > 0) {
      bigDeltas++;
    }
    pc.setHSV(64, 255, uint8_t((trappedCondition ? noise3 : noise) * 255));
  }
  if (bigDeltas > 15) {
    trappedCondition = approxIteration;
  }
}

void drawFrameTime() {
  int dt = frameTime.dt();  // Time between frames in ms

  constexpr int beginBarPix = 598;
  constexpr int endBarPix = 623;
  auto barpix = [](int i) -> CRGB& {
    return pix(i + beginBarPix);
  };

  auto drawFromBase = [barpix](int base, int dt, int maxPixels, CRGB primary, CRGB secondary) {
    int tenMs = dt / 10;      // Number of ms between frames in multiples of 10 rounded down
    int remainder = dt % 10;  // Remainder
    for (int i = 0; i < min(maxPixels, tenMs); i++) {
      auto& pc = barpix(base + i);
      pc = ((i % 5) != 4) ? primary : secondary;
      pc.nscale8(10);
    }

    if (tenMs < maxPixels) {
      auto& pc = barpix(base + tenMs);
      pc = ((tenMs % 5) != 4) ? primary : secondary;
      pc.nscale8(millis() % 10 <= remainder ? 10 : 0);
    }
  };

  // clear
  for (int i = 0; i < (endBarPix - beginBarPix); i++) {
    auto& pc = barpix(i);
    pc = CRGB::Black;
  }

  drawFromBase(0, showTime, 4, CRGB::Red, CRGB::Yellow);
  drawFromBase(5, dt - showTime, 8, CRGB::Cyan, CRGB::Magenta);
  drawFromBase(14, dt, 10, CRGB::Blue, CRGB::Green);

  /*
  for (int i = 0; i < min(10, remainder); i++) {
    auto& pc = pix(607 + tenMs + 1 - i);
    pc = ((i % 5) != 4) ? CRGB::Yellow : CRGB::Red;
    pc.nscale8(4);
  }
  */
}

void loop() {
  frameTime.update();
  countdown -= frameTime.dt();
  const int modeSwapTime = 300 * 1000;  // Time to change modes in seconds

  if (countdown < 0) {
    countdown = modeSwapTime;
    int current = mode;
    while (mode == current) {
      mode = rand() % 5;
    }
#if defined(FORCE_MODE)
    mode = FORCE_MODE;
#endif
    newProgStartMs = frameTime.t0();
  }
  switch (mode) {
    case 0:
      rot_y();
      break;
    case 1:
      twist();
      break;
    case 2:
      screw();
      break;
    case 3:
      loop_sphere();
      break;
    case 4:
      perlin();
      break;
    case 5:
      clear(CRGB::Red);
      break;
    case 7:
      perlin_flashing();
      break;
    default:
      break;
  }

  drawFrameTime();

  const uint32_t beforeShow = millis();
  FastLED.show();
  showTime = millis() - beforeShow;
}