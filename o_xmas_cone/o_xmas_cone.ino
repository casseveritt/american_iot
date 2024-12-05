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
// #define FORCE_MODE 4

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
  void setPosXY(float offset = -0.5) {  // Set position of sphere along the XY plane of the tree
    pos = Vec3f(float((rand() % 63) - 31) / 100.0f, float(rand() % 148) / 100.0f, offset);
  }
  void setPosYZ(float offset = -0.5) {  // Set position of sphere along the YZ plane of the tree
    pos = Vec3f(offset, float(rand() % 148) / 100.0f, float((rand() % 63) - 31) / 100.0f);
  }
  void setPosXZ(float offset = -0.5) {  // Set position of sphere along the XZ plane of the tree
    pos = Vec3f(float((rand() % 63) - 31) / 100.0f, offset, float((rand() % 63) - 31) / 100.0f);
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
  FastLED.setBrightness(255);
  for (int i = 0; i < NUM_LEDS; i++) {
    strip[i] = CRGB::Black;
  }
  for (int i = 0; i < 102; i++) {
    c.push_back(randColor());
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

void luke_sphere() {
  Sphere sphere = Sphere();
  sphere.pos = Vec3f(0.0, 0.5f * sin((iteration % 100) / 100.0 * k2pi) + 0.75, 0.0);
  sphere.radiusSquared = pow(0.25f, 2);

  const Sphere s = sphere;

  for (const auto& li : led) {
    CRGB& pc = strip[li.index];
    bool inside = s.isInside(li.pos);
    pc = inside ? CRGB::Red : CRGB::Black;
    pc.nscale8(8);
  }
}

int spheresRotation = 1100;
void setDelayTime(uint32_t ms, int max, int min) {
  delayTime = int(ms) + ((rand() % (max - min + 1)) + min);
}

void loop_sphere() {
  uint32_t ms = timestamp[idx[0]] = millis();
  int radiusMax = 10;
  if (ms >= delayTime && spheres.size() < 50) {
    Sphere s = Sphere(randColor());
    s.setPosYZ();
    s.setVel(5, 15);
    s.setRad(3, 10);
    spheres.push_back(s);
    setDelayTime(ms, 333, 100);
  }

  for (int i = 0; i < spheres.size(); i++) {  // Moves all the spheres
    Sphere& s = spheres[i];
    s.pos.x += s.vel;
  }

  for (int i = 0; i < spheres.size(); i++) {  // Culls spheres outside the bounds of the tree
    Sphere& s = spheres[i];
    if (s.pos.x > (0.31f + float(radiusMax) / 10)) {
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
    li.pos = rotworld.Rotate(li.pos);
    bool outsideSpheres = true;
    float minDist = 100000;
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
    if (outsideSpheres) {
      pc = CRGB::Black;
    }
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
  auto ms = timestamp[idx[0]];
  float twistRadsPerMeter = k2pi * 2.5f * (sin(fmod((k2pi * ms) / 12000.0f, k2pi) + 1.0));

  for (const auto& li : led) {
    Vec3f p = li.pos;
    p = Rotationf(Vec3f(0, 1, 0), -twistRadsPerMeter * p.y + yrot).Rotate(p);
    CRGB& pc = strip[li.index];
    pc = p.x > 0 ? CRGB::Blue : CRGB::Black;
    pc.nscale8(8);
  }
  yrot += k2pi / 60.0f;
}

void screw() {
  auto ms = timestamp[idx[0]];
  float twistRadsPerMeter = k2pi * 2.5f;  //(sin((k2pi * ms) / 12000.0f) + 1.0);

  for (const auto& li : led) {
    Vec3f p = li.pos;
    float y = 1.5f * pow(p.y / 1.5f, 2.0f);
    p = Rotationf(Vec3f(0, 1, 0), -twistRadsPerMeter * y + yrot).Rotate(p);
    p.y = 0;
    p.Normalize();
    CRGB& pc = strip[li.index];
    pc = CRGB::Red;
    float x = p.x * 0.5f + 0.5f;
    float ss = x * x * x * (x * (6.0f * x - 15.0f) + 10.0f);
    pc = pc.lerp8(CRGB::Green, uint8_t(max(0.0f, ss * 255.0f)));
    pc.nscale8(8);
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
  uint32_t dt = (timestamp[idx[0]] - newProgStartMs);
  for (const auto& li : led) {
    CRGB& pc = strip[li.index];
    Vec3f p = li.pos * 4;
    float noise = abs(ImprovedNoise::noise(p.x - (dt * 0.0003f), p.y, p.z));
    pc.setHSV(uint8_t(noise * 255 + 96), 255, 32);
  }
}

void drawFrameTime() {
  int dt = (timestamp[idx[0]] - timestamp[idx[1]]) / 2;

  auto seg = segment[0];
  int incr = seg.first < seg.second ? 1 : -1;
  for (int i = 0; i < min(50, dt); i++) {
    auto& pc = pix(seg.first + i * incr);
    pc = ((i % 5) != 0) ? CRGB::BlueViolet : CRGB::Orange;
    pc.nscale8(8);
  }
}

void drawFrameTimeLuke() {
  int dt = (timestamp[idx[0]] - timestamp[idx[1]]);  // Time between frames in ms

  constexpr int beginBarPix = 598;  // Should be +598, but it breaks for some reason
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
  idx[0] = iteration % 3;
  idx[1] = (iteration + 2) % 3;
  idx[2] = (iteration + 1) % 3;
  uint32_t ms = timestamp[idx[0]] = millis();
  int dt = ms - timestamp[idx[1]];
  countdown -= dt;
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
    newProgStartMs = timestamp[idx[0]];
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
    case 6:
      luke_sphere();
      break;
    default:
      break;
  }

  drawFrameTimeLuke();

  const uint32_t beforeShow = millis();
  FastLED.show();
  showTime = millis() - beforeShow;
  iteration++;
}