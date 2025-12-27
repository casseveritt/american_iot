#include <stdint.h>
#include <stdio.h>

#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "perlin.h"
#include "stb_image_write.h"

int main() {
  const int width = 512;
  const int height = 512;
  const int channels = 1;  // Greyscale

  std::vector<uint8_t> image(width * height * channels);

  // Fill with Perlin noise
  float scale = 0.05f;  // Scale factor for the noise
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Sample Perlin noise at this position
      float noise = ImprovedNoise::noise(x * scale, y * scale, 0.0f);
      // Normalize from [-1, 1] to [0, 255]
      uint8_t value = static_cast<uint8_t>((noise + 1.0f) * 127.5f);

      int idx = (y * width + x) * channels;
      image[idx] = value;
    }
  }

  // Write PNG file
  const char* filename = "noise.png";
  if (stbi_write_png(filename, width, height, channels, image.data(),
                     width * channels)) {
    printf("Successfully wrote %s\n", filename);
    return 0;
  } else {
    fprintf(stderr, "Failed to write %s\n", filename);
    return 1;
  }
}
