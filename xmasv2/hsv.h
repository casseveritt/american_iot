
#pragma once
#include "linear.h"

inline r3::Vec3f rgb_from_hsv(const r3::Vec3f &hsv) {
  float h = hsv.x;
  float s = hsv.y;
  float v = hsv.z;

  int i = floor(h * 6);
  float f = h * 6 - i;
  float p = v * (1 - s);
  float q = v * (1 - f * s);
  float t = v * (1 - (1 - f) * s);

  switch (i % 6) {
  case 0:
    return Vec3f(v, t, p);
  case 1:
    return Vec3f(q, v, p);
  case 2:
    return Vec3f(p, v, t);
  case 3:
    return Vec3f(p, q, v);
  case 4:
    return Vec3f(t, p, v);
  case 5:
    return Vec3f(v, p, q);
  }
  return Vec3f(0, 0, 0); // should never reach here
}
