#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>

#include "smi_leds.h"

#include "color.h"
#include "hsv.h"
#include "linear.h"
#include "time.h"
#include "tree.h"

using namespace r3;

int main(int argc, char *argv[]) {
  constexpr size_t sz = PIXELS_PER_STRIP * STRIPS * 3;
  uint8_t buffer[sz] = {};
  uint8_t *ptr;

  auto pixInfo = get_pix_info();

  // initialize the smi_leds module, starting with a 35% brightness
  leds_init(PIXELS_PER_STRIP, LED_BRIGHTNESS);
  printf("compiled for %d strips\n", leds_num_strips());

  leds_clear();

  int64_t start = get_time_nsec();
  int count = 0;
  int64_t freecount = 0;
  for (float t = 0.0;; t += .0001) {
    uint64_t t_ns = get_time_nsec();

    Vec3f red(0.2f, 0, 0);
    for (int strip = 0; strip < STRIPS; strip++) {
      uint8_t *pixels = buffer + (strip * PIXELS_PER_STRIP * 3);
      for (int led = 0; led < strip; led++) {
        set_color(buffer, strip * PIXELS_PER_STRIP + led, red);
      }
    }

    if (freecount > 0) {
      for (auto p : pixInfo) {
        set_color(buffer, p.index, Vec3f(0, 0.25, 0));
      }
    }

    // Send the buffer to the SMI buffer
    leds_set(buffer);

    // Actually send them to the LEDs:
    leds_send();

    count++;
    freecount++;
    if (count > 1000) {
      int64_t end = get_time_nsec();
      double delta = (end - start) * 1e-9;
      printf("fps = %d\n", int(1000 / delta));
      start = end;
      count = 0;
    }
  }
}
