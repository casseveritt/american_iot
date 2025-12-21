#pragma once

#include <cstdint>
#include <vector>

#include "linear.h"

// Must be 16 if smi_leds is compiled for 16.
#define STRIPS 16
#if defined(TREE_V3)
#define PIXELS_PER_STRIP 300
#else
#define PIXELS_PER_STRIP 320
#endif

struct PixInfo {
  uint32_t index;
  r3::Vec3f position;
  bool outside;
};

std::vector<PixInfo> get_pix_info();
