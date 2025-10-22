#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "linear.h"

#define STRIPS 16
#define PIXELS_PER_STRIP 320

#define LED_BRIGHTNESS 16

#define BRANCHES_PER_RING 8

#define RINGS 8

#define BRANCH_PITCH (-M_PI / 4.0)
#define PIXEL_SPACING 0.016667

// [0 .. branchLen/2 - 1] are the outside pixels starting at the trunk
// [branch_Len/2 ... branchLen - 1] are the inside pixels starting at the branch
// tip
constexpr uint32_t branchLen[RINGS] = {74, 74, 74, 74, 56, 38, 38, 20};
constexpr uint32_t branchesPerStrip[RINGS] = {4, 4, 4, 4, 4, 8, 8, 8};

struct PixInfo {
  uint32_t index;
  r3::Vec3f position;
  bool outside;
};

inline std::vector<PixInfo> get_pix_info() {
  int pixels = 0;
  for (int i = 0; i < RINGS; i++) {
    pixels += BRANCHES_PER_RING * branchLen[i];
  }
  std::vector<PixInfo> pi;
  pi.reserve(pixels);
  int ring_start_index = 0;
  int indexAtRing[RINGS] = {};

  for (int i = 1; i < RINGS; i++) {
    indexAtRing[i] =
        indexAtRing[i - 1] +
        PIXELS_PER_STRIP * (BRANCHES_PER_RING / branchesPerStrip[i - 1]);
    printf("indexAtRing[%d] = %d\n", i, indexAtRing[i]);
  }

  for (int i = 0; i < RINGS; i++) {
    if (i != 1) {
      continue;
    }
    int half_branch_length = branchLen[i] / 2;
    for (int j = 0; j < BRANCHES_PER_RING; j++) {
      int branch_in_strip = j % branchesPerStrip[i];
      int index_in_strip = branch_in_strip * branchLen[i];
      int strip_in_ring = j / branchesPerStrip[i];
      int index =
          indexAtRing[i] + strip_in_ring * PIXELS_PER_STRIP + index_in_strip;
      float angle = (j * (2.0 * M_PI)) / BRANCHES_PER_RING;
      float branch_height = // in meters
          +0.6096           // height of first branch at trunk
          + i * 0.1778;     // branch spacing

      size_t branch_start = pi.size();
      // down the branch on the outside
      for (int k = 0; k < half_branch_length; k++) {
        float x = k * PIXEL_SPACING * cos(BRANCH_PITCH);
        float y = k * PIXEL_SPACING * sin(BRANCH_PITCH) + branch_height;
        PixInfo pix;
        pix.index = index + k;
        pix.position = Vec3f(cos(angle) * x, y, sin(angle) * x);
        pix.outside = true;
        pi.push_back(pix);
      }
      // back up the branch on the inside
      for (int k = half_branch_length - 1; k >= 0; k--) {
        float x = k * PIXEL_SPACING * cos(BRANCH_PITCH);
        float y = k * PIXEL_SPACING * sin(BRANCH_PITCH) + branch_height;
        PixInfo pix;
        pix.index = index + half_branch_length + (half_branch_length - 1 - k);
        pix.position = Vec3f(cos(angle) * x, y, sin(angle) * x);
        pix.outside = false;
        pi.push_back(pix);
      }
    }
  }

  return pi;
}
