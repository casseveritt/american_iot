#pragma once

#include <cstdint>
#include <vector>

#include "linear.h"

#if defined(TREE_V3)
#define STRIPS 4
#define PIXELS_PER_STRIP 300
#else
#define STRIPS 16
#define PIXELS_PER_STRIP 320
#endif

struct PixInfo {
  uint32_t index;
  r3::Vec3f position;
  bool outside;
};

std::vector<PixInfo> get_pix_info();
