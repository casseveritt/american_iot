#pragma once

#include <cstdint>
#include <vector>

#include "linear.h"

#define STRIPS 16
#define PIXELS_PER_STRIP 320

struct PixInfo {
  uint32_t index;
  r3::Vec3f position;
  bool outside;
};

std::vector<PixInfo> get_pix_info();
