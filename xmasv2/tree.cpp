#include "tree.h"

#include <cmath>
#include <cstdint>

#if !defined(TREE_V2) && !defined(TREE_V3)
#define TREE_V2
#endif

#if defined(TREE_V2)

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

// TREE_V3 is implemented with four strips of 300 pixels. Each strip is 5
// meters. The strips are arranged in pairs that are adhered back-to-back. The
// tree configuration is a cone. The strips mostly go longitudinally. Starting
// at the bottom, the strip goes 72 pixels up to the tip of the cone, then 72
// pixels down. There are 12 pixels that go horizontal before ascending
// vertically up and then down again. One pair does the front half of the tree,
// the other pair the back half.

#define STRIPS_V3 4
#define STRIP_PAIRS 2
#define PIXELS_PER_STRIP_V3 300
#define METERS_PER_PIXEL (5.0f / 300.0f)  // 300 pixels = 5 meters
#define CONE_BASE_DIAMETER 0.60f          // meters (60 cm)
#define CONE_TOP_DIAMETER 0.05f           // meters (5 cm)

std::vector<PixInfo> get_pix_info() {
  std::vector<PixInfo> pi;
  pi.reserve(STRIPS_V3 * PIXELS_PER_STRIP_V3);

  // Calculate cone dimensions
  const float base_radius = CONE_BASE_DIAMETER / 2.0f;
  const float top_radius = CONE_TOP_DIAMETER / 2.0f;

  // Each vertical section is 72 pixels
  const float vertical_segment_length = 72 * METERS_PER_PIXEL;  // 1.2 meters
  const float cone_height = vertical_segment_length;

  // Process pairs of strips (back-to-back)
  for (int pair = 0; pair < STRIP_PAIRS; pair++) {
    float base_angle = pair * M_PI;  // 0 for front, π for back

    // Each pair has an inside strip and an outside strip with same positions
    for (int strip_in_pair = 0; strip_in_pair < 2; strip_in_pair++) {
      int strip = pair * 2 + strip_in_pair;
      bool is_outside =
          (strip_in_pair == 1);  // Second strip in pair is outside
      int pixel_idx = 0;

      // Section 1: 72 pixels up to the tip
      for (int i = 0; i < 72; i++) {
        float t = (float)i / 71.0f;  // 0 to 1
        float height = t * cone_height;
        float radius = base_radius + (top_radius - base_radius) * t;

        PixInfo pix;
        pix.index = strip * PIXELS_PER_STRIP_V3 + pixel_idx++;
        pix.position = r3::Vec3f(radius * cos(base_angle), height,
                                 radius * sin(base_angle));
        pix.outside = is_outside;
        pi.push_back(pix);
      }

      // Section 2: 72 pixels down from the tip
      for (int i = 0; i < 72; i++) {
        float t = (float)(71 - i) / 71.0f;  // 1 to 0
        float height = t * cone_height;
        float radius = base_radius + (top_radius - base_radius) * t;
        float angle = base_angle + M_PI / 6.0f;  // Offset angle slightly

        PixInfo pix;
        pix.index = strip * PIXELS_PER_STRIP_V3 + pixel_idx++;
        pix.position =
            r3::Vec3f(radius * cos(angle), height, radius * sin(angle));
        pix.outside = is_outside;
        pi.push_back(pix);
      }

      // Section 3: 12 pixels horizontal at the base
      for (int i = 0; i < 12; i++) {
        float angle_offset = ((float)i / 11.0f) * (M_PI / 3.0f);
        float angle = base_angle + M_PI / 6.0f + angle_offset;

        PixInfo pix;
        pix.index = strip * PIXELS_PER_STRIP_V3 + pixel_idx++;
        pix.position = r3::Vec3f(base_radius * cos(angle),
                                 0.05f,  // Slightly above base
                                 base_radius * sin(angle));
        pix.outside = is_outside;
        pi.push_back(pix);
      }

      // Section 4: 72 pixels up to the tip again
      for (int i = 0; i < 72; i++) {
        float t = (float)i / 71.0f;  // 0 to 1
        float height = t * cone_height;
        float radius = base_radius + (top_radius - base_radius) * t;
        float angle = base_angle + M_PI / 2.0f;  // Different longitude

        PixInfo pix;
        pix.index = strip * PIXELS_PER_STRIP_V3 + pixel_idx++;
        pix.position =
            r3::Vec3f(radius * cos(angle), height, radius * sin(angle));
        pix.outside = is_outside;
        pi.push_back(pix);
      }

      // Section 5: 72 pixels down from the tip
      for (int i = 0; i < 72; i++) {
        float t = (float)(71 - i) / 71.0f;  // 1 to 0
        float height = t * cone_height;
        float radius = base_radius + (top_radius - base_radius) * t;
        float angle = base_angle + M_PI / 2.0f + M_PI / 6.0f;

        PixInfo pix;
        pix.index = strip * PIXELS_PER_STRIP_V3 + pixel_idx++;
        pix.position =
            r3::Vec3f(radius * cos(angle), height, radius * sin(angle));
        pix.outside = is_outside;
        pi.push_back(pix);
      }

      // Remaining 12 pixels to complete 300
      for (int i = 0; i < 12; i++) {
        float angle_offset = ((float)i / 11.0f) * (M_PI / 3.0f);
        float angle = base_angle + M_PI / 2.0f + M_PI / 6.0f + angle_offset;

        PixInfo pix;
        pix.index = strip * PIXELS_PER_STRIP_V3 + pixel_idx++;
        pix.position = r3::Vec3f(base_radius * cos(angle), 0.05f,
                                 base_radius * sin(angle));
        pix.outside = is_outside;
        pi.push_back(pix);
      }
    }
  }

  return pi;
}

#endif
