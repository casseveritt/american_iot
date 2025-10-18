#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include <linear.h>

#include "smi_leds.h"

using namespace r3;

#define NUM_LEDS 296
#define NUM_STRIPS 16

int64_t get_time() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000000000 + ts.tv_nsec;
}

Vec3f rgb_from_hsv(const Vec3f &hsv) {
  float h = hsv.x;
  float s = hsv.y;
  float v = hsv.z;

  int i = floor(h * 6);
  float f = h * 6 - i;
  float p = v * (1 - s);
  float q = v * (1 - f * s);
  float t = v * (1 - (1 - f) * s);

  switch (i % 6) {
  case 0:
    return Vec3f(v, t, p);
  case 1:
    return Vec3f(q, v, p);
  case 2:
    return Vec3f(p, v, t);
  case 3:
    return Vec3f(p, q, v);
  case 4:
    return Vec3f(t, p, v);
  case 5:
    return Vec3f(v, p, q);
  }
  return Vec3f(0, 0, 0); // should never reach here
}

void set_color(uint8_t *&ptr, const Vec3f &color) {
  *(ptr++) = (int)(color.x * 255);
  *(ptr++) = (int)(color.y * 255);
  *(ptr++) = (int)(color.z * 255);
}

int main(int argc, char *argv[]) {
  uint8_t buffer[NUM_LEDS * NUM_STRIPS * 3] = {};
  uint8_t *ptr;

  std::vector<int> strip_indexes = {0, 3, 6, 9, 14};

  // initialize the smi_leds module, starting with a 35% brightness
  leds_init(NUM_LEDS, 35);
  printf("compiled for %d strips\n", leds_num_strips());

  int64_t start = get_time();
  int count = 0;
  int64_t freecount = 0;
  for (float t = 0.0;; t += .0001) {
    // Manually and slowly build the color buffer. 3 bytes per pixel (RGB)
    // for NUM_LEDS pixels and NUM_STRIPS strips
    ptr = buffer;
    for (int strip = 0; strip < NUM_STRIPS; strip++) {
      float hue = fmod(t + (strip * .04), 1.0);
      float r, g, b;

      Vec3f rgb = rgb_from_hsv(Vec3f(hue, 1.0, 1.0));
      hsv_to_rgb(hue, 1.0, 1.0, &r, &g, &b);
      for (int led = 0; led < NUM_LEDS; led++) {
        set_color(ptr, rgb);
      }

      uint8_t *pixels = buffer + (strip * NUM_LEDS * 3);
      Vec3f red(50 / 255.0f, 0, 0);
      for (int led = 0; led < strip; led++) {
        set_color(pixels, red);
      }
    }
}


int main(int argc, char *argv[])
{
    uint8_t buffer[NUM_LEDS * NUM_STRIPS * 3] = {};
    uint8_t *ptr;

    std::vector<int> strip_indexes = {0, 3, 6, 9, 14};

    // initialize the smi_leds module, starting with a 35% brightness
    leds_init(NUM_LEDS, 35);
    printf("compiled for %d strips\n", leds_num_strips());

    int64_t start = get_time();
    int count = 0;
    int64_t freecount = 0;
    for(float t = 0.0;; t += .0001)
    {
        // Manually and slowly build the color buffer. 3 bytes per pixel (RGB)
        // for NUM_LEDS pixels and NUM_STRIPS strips
        ptr = buffer;
        for(int strip = 0; strip < NUM_STRIPS; strip++)
        {
            float hue = fmod(t + (strip * .04), 1.0);
            float r, g, b;

            hsv_to_rgb(hue, 1.0, 1.0, &r, &g, &b);
            for(int led = 0; led < NUM_LEDS; led++)
            {
                *(ptr++) = (int)(r * 255);
                *(ptr++) = (int)(g * 255);
                *(ptr++) = (int)(b * 255);
            }

            uint8_t* pixels = buffer + (strip * NUM_LEDS * 3);
            for (int led = 0; led < strip; led++) {
                pixels[3 * led + 0] = uint8_t(50);
                pixels[3 * led + 1] = uint8_t(0);
                pixels[3 * led + 2] = uint8_t(0);
            }

        }
>>>>>>> 14ff478 (tweaks to example)

    int sidx = strip_indexes[(freecount / NUM_LEDS) % strip_indexes.size()];
    int pidx = freecount % NUM_LEDS;
    Vec3f black(0, 0, 0);
    ptr = buffer + (sidx * NUM_LEDS + pidx) * 3;
    set_color(ptr, black);

    // Send the buffer to the SMI buffer
    leds_set(buffer);

    // Actually send them to the LEDs:
    leds_send();

    // Sleep for a while
    // usleep(50000);
    count++;
    freecount++;
    if (count > 1000) {
      int64_t end = get_time();
      double delta = (end - start) * 1e-9;
      printf("fps = %d\n", int(1000 / delta));
      start = end;
      count = 0;
    }
  }
}
