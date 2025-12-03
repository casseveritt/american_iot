
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
  r3::Vec3f rgb;
  switch (i % 6) {
    case 0:
      rgb = r3::Vec3f(v, t, p);
      break;
    case 1:
      rgb = r3::Vec3f(q, v, p);
      break;
    case 2:
      rgb = r3::Vec3f(p, v, t);
      break;
    case 3:
      rgb = r3::Vec3f(p, q, v);
      break;
    case 4:
      rgb = r3::Vec3f(t, p, v);
      break;
    case 5:
      rgb = r3::Vec3f(v, p, q);
      break;
    default:
      rgb = r3::Vec3f(0, 0, 0);
      break;
  }
  float gamma = 2.2f;
  return r3::Vec3f(pow(rgb.x, gamma), pow(rgb.y, gamma), pow(rgb.z, gamma));
}
