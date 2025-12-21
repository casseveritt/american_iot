#include "tree.h"

#include <cmath>
#include <cstdint>

#if !defined(TREE_V2) && !defined(TREE_V3)
#define TREE_V2
#endif

#if defined (TREE_V2)

#define BRANCHES_PER_RING 8
#define RINGS 8
#define BRANCH_PITCH float(-M_PI / 4.0)
#define BRANCH_THICKNESS 0.008  // 8 mm
#define PIXEL_SPACING 0.016667

// [0 .. branchLen/2 - 1] are the outside pixels starting at the trunk
// [branch_Len/2 ... branchLen - 1] are the inside pixels starting at the branch
// tip
constexpr uint32_t branchLen[RINGS] = {74, 74, 74, 74, 56, 38, 38, 20};
constexpr uint32_t branchesPerStrip[RINGS] = {4, 4, 4, 4, 4, 8, 8, 8};
constexpr float branchHeightAtTrunkInches[RINGS] = {23.5, 30.5, 38,   47,
                                                    54,   61,   67.5, 74.5};

constexpr int phys_strip_index[] = {0, 1, 2,  3,  4,  5,  13, 7,
                                    8, 9, 10, 11, 12, 15, 15, 15};

std::vector<PixInfo> get_pix_info() {
  int pixels = 0;
  for (int i = 0; i < RINGS; i++) {
    pixels += BRANCHES_PER_RING * branchLen[i];
  }
  std::vector<PixInfo> pi;
  pi.reserve(pixels);
  int ring_start_index = 0;
  int logicalStripAtRing[RINGS] = {};

  for (int i = 1; i < RINGS; i++) {
    logicalStripAtRing[i] = logicalStripAtRing[i - 1] +
                            (BRANCHES_PER_RING / branchesPerStrip[i - 1]);
  }

  for (int i = 0; i < RINGS; i++) {
    int half_branch_length = branchLen[i] / 2;
    for (int j = 0; j < BRANCHES_PER_RING; j++) {
      int branch_in_strip = j % branchesPerStrip[i];
      int index_in_strip = branch_in_strip * branchLen[i];
      int strip_in_ring = j / branchesPerStrip[i];
      int logical_strip = logicalStripAtRing[i] + strip_in_ring;
      int phys_strip = phys_strip_index[logical_strip];
      int index = phys_strip * PIXELS_PER_STRIP + index_in_strip;

      float angle = (j * (2.0 * M_PI)) / BRANCHES_PER_RING;

      if (i & 1) {
        angle -= M_PI / BRANCHES_PER_RING;
      }

      constexpr float angle_adj_deg[] = {0,    0,     -10.0f, 0,
                                         0.0f, 10.0f, -5.0f,  9.0f};

      angle += (angle_adj_deg[i] * M_PI) / 180.0;

      const float branch_height =  // in meters
          0.0254 * branchHeightAtTrunkInches[i];

      r3::Posef branchPitch(
          r3::Rotationf(r3::Vec3f(0.0f, 0.0f, 1.0f), BRANCH_PITCH),
          r3::Vec3f(0, 0, 0));
      r3::Posef branchTranslate(r3::Rotationf(),
                                r3::Vec3f(0.0f, branch_height, 0.0f));
      constexpr float kpi = 3.14159f;
      r3::Posef branchLongitude(
          r3::Rotationf(r3::Vec3f(0.0f, 1.0f, 0.0f), -angle),
          r3::Vec3f(0, 0, 0));
      r3::Posef toTreeFromBranch =
          branchLongitude * branchTranslate * branchPitch;

      size_t branch_start = pi.size();
      // down the branch on the outside
      for (int k = 0; k < half_branch_length; k++) {
        r3::Vec3f position(k * PIXEL_SPACING + 0.02, BRANCH_THICKNESS / 2,
                           0.0f);
        PixInfo pix;
        pix.index = index + k;
        pix.position = toTreeFromBranch * position;
        pix.outside = true;
        pi.push_back(pix);
      }
      // back up the branch on the inside
      for (int k = half_branch_length - 1; k >= 0; k--) {
        r3::Vec3f position(k * PIXEL_SPACING + 0.02, -BRANCH_THICKNESS / 2,
                           0.0f);
        PixInfo pix;
        pix.index = index + half_branch_length + (half_branch_length - 1 - k);
        pix.position = toTreeFromBranch * position;
        pix.outside = false;
        pi.push_back(pix);
      }
    }
  }

  return pi;
}

#endif

#if defined(TREE_V3)

#endif
