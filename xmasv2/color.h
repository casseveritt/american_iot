#pragma once

#include <cstdint>

#include "linear.h"

inline void set_color(uint8_t *ptr, int idx, const r3::Vec3f &color) {
  uint8_t *p = ptr + 3 * idx;
  p[0] = (uint8_t)(color.x * 255);
  p[1] = (uint8_t)(color.y * 255);
  p[2] = (uint8_t)(color.z * 255);
}

struct Color : public r3::Vec3f {
  Color() : r3::Vec3f(0, 0, 0) {}

  Color(const r3::Vec3f &v) : r3::Vec3f(v) {}

  Color(float r, float g, float b) : r3::Vec3f(r, g, b) {}

  Color lerp(const Color &other, float t) const {
    return Color(x + (other.x - x) * t, y + (other.y - y) * t,
                 z + (other.z - z) * t);
  }
};

static const Color BLACK(0.0f, 0.0f, 0.0f);
static const Color WHITE(1.0f, 1.0f, 1.0f);
static const Color RED(1.0f, 0.0f, 0.0f);
static const Color GREEN(0.0f, 1.0f, 0.0f);
static const Color BLUE(0.0f, 0.0f, 1.0f);
static const Color YELLOW(1.0f, 1.0f, 0.0f);
static const Color CYAN(0.0f, 1.0f, 1.0f);
static const Color MAGENTA(1.0f, 0.0f, 1.0f);
static const Color ORANGE(1.0f, 0.647f, 0.0f);
static const Color PURPLE(0.5f, 0.0f, 0.5f);
static const Color PINK(1.0f, 0.753f, 0.796f);
static const Color BROWN(0.647f, 0.165f, 0.165f);
static const Color GRAY(0.5f, 0.5f, 0.5f);
static const Color SILVER(0.753f, 0.753f, 0.753f);
static const Color GOLD(1.0f, 0.843f, 0.0f);

static const Color MAROON(0.502f, 0.0f, 0.0f);
static const Color BLUE_VIOLET(0.541f, 0.169f, 0.886f);