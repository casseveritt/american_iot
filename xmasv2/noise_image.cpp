#include <stdint.h>
#include <stdio.h>

#include <vector>

// Simple PNG writer using libpng
// Compile with: g++ -o noise_image noise_image.cpp -lpng

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

int main() {
  const int width = 512;
  const int height = 512;
  const int channels = 3;  // RGB

  std::vector<uint8_t> image(width * height * channels);

  // Fill with a simple gradient for now
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int idx = (y * width + x) * channels;
      image[idx + 0] = (x * 255) / width;   // R
      image[idx + 1] = (y * 255) / height;  // G
      image[idx + 2] = 128;                 // B
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
