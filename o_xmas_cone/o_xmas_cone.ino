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
  float radius;
  float radiusSquared;
  float vel;
  CRGB color;

  Sphere() {}
  Sphere(CRGB colorIn) {
    color = colorIn;
  }
  void setPosXY(float offset = -0.5) { // Set position of sphere along the XY plane of the tree
    pos = Vec3f(float((rand() % 63) - 31) / 100.0f, float(rand() % 148) / 100.0f, offset);
  }
  void setPosYZ(float offset = -0.5) { // Set position of sphere along the YZ plane of the tree
    pos = Vec3f(offset, float(rand() % 148) / 100.0f, float((rand() % 63) - 31) / 100.0f);
  }
  void setPosXZ(float offset = -0.5) { // Set position of sphere along the XZ plane of the tree
    pos = Vec3f(float((rand() % 63) - 31) / 100.0f, offset, float((rand() % 63) - 31) / 100.0f);
  }
  void setRad(int upper, int lower) { // Set radius of sphere in centemeters
    radius = float((rand() % (upper - lower + 1)) + lower) / 100.0f;
    radiusSquared = pow(radius, 2);
  }
  void setVel(int min, int max) { // Set sphere velocity in thousandths of a meter per second
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
    float theta = (2.0f * M_PI * j) / 12.0f;
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
}

template<typename T>
T clamp(T val, T lo, T hi) {
  return max(min(val, hi), lo);
}

void luke_sphere() {
  Sphere sphere = Sphere();
  sphere.pos = Vec3f(0.0, 0.5f * sin((iteration % 100) / 100.0 * 2 * M_PI) + 0.75, 0.0);
  sphere.radiusSquared = pow(0.25f, 2);

  const Sphere s = sphere;

  for (const auto& li : led) {
    CRGB& pc = strip[li.index];
    bool inside = s.isInside(li.pos);
    pc = inside ? CRGB::Red : CRGB::Black;
    pc.nscale8(8);
  }
}

void loop_sphere() {
  uint32_t ms = timestamp[idx[0]] = millis();
  if (ms >= delayTime) {
    Sphere s = Sphere(randColor());
    s.setPosYZ();
    s.setVel(5, 15);
    s.setRad(5, 20);
    spheres.push_back(s);
    delayTime = int(ms) + ((rand() % 200) + 100);
  }

  for (int i = 0; i < spheres.size(); i++) {
    Sphere& s = spheres[i];
    s.pos.x += s.vel;
  }

  for (int i = 0; i < spheres.size(); i++) {
    Sphere& s = spheres[i];
    if (s.pos.x > 1.0f) {
      Sphere tempS = spheres[spheres.size() - 1];
      spheres[spheres.size() - 1] = spheres[i];
      spheres[i] = tempS;
      spheres.pop_back();
    }
  }

  Rotationf rotworld(Vec3f(0, 1, 0), ToRadians(70.0f));

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
          pc.nscale8(uint8_t(255 * pow(1.0 - dist / s.radius, 2.0f)));
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
    p.x = pow(p.x, 5.0f);
    p.z = pow(p.z, 5.0f);

    p *= 25;
    CRGB& pc = strip[li.index];
    pc = CRGB(max(0.0f, p.x), max(0.0f, p.y), max(0.0f, p.z));
  }

  yrot += 2.0f * M_PI / 60.0f;
}

void twist() {
  auto ms = timestamp[idx[0]];
  float twistRadsPerMeter = 2.0 * M_PI * 2.5f * (sin((2.0f * M_PI * ms) / 12000.0f) + 1.0);

  for (const auto& li : led) {
    Vec3f p = li.pos;
    p = Rotationf(Vec3f(0, 1, 0), -twistRadsPerMeter * p.y + yrot).Rotate(p);
    CRGB& pc = strip[li.index];
    pc = p.x > 0 ? CRGB::Blue : CRGB::Black;
    pc.nscale8(8);
  }
  yrot += 2.0f * M_PI / 60.0f;
}

void screw() {
  auto ms = timestamp[idx[0]];
  float twistRadsPerMeter = 2.0 * M_PI * 2.5f;  //(sin((2.0f * M_PI * ms) / 12000.0f) + 1.0);

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
  yrot += 2.0f * M_PI / 45.0f;
  while (yrot > (2.0f * M_PI)) {
    yrot -= 2.0f * M_PI;
  }
}

void clear(CRGB color) {
  for (const auto& li : led) {
    CRGB& pc = strip[li.index];
    pc = color;
    pc.nscale8(8);
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

void loop() {
  idx[0] = iteration % 3;
  idx[1] = (iteration + 2) % 3;
  idx[2] = (iteration + 1) % 3;
  uint32_t ms = timestamp[idx[0]] = millis();
  switch (MODE) {
    case 0:
      luke_sphere();
      break;
    case 1:
      rot_y();
      break;
    case 2:
      loop_sphere();
      break;
    case 3:
      twist();
      break;
    case 4:
      screw();
      break;
    case 5:
      clear(CRGB::Red);
      break;
    default:
      break;
  }

  drawFrameTime();

  FastLED.show();
  iteration++;
}