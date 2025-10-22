#pragma once

#include "linear.h"
#include <cstdint>

inline void set_color(uint8_t *ptr, int idx, const r3::Vec3f &color) {
  uint8_t *p = ptr + 3 * idx;
  p[0] = (uint8_t)(color.x * 255);
  p[1] = (uint8_t)(color.y * 255);
  p[2] = (uint8_t)(color.z * 255);
}
