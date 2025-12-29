#pragma once

#ifdef STUB_LEDS

#include <stdint.h>
#include <stdio.h>

// Stub implementations of LED functions for testing without hardware
inline void leds_init(int pixels_per_strip, int brightness) {
  printf("[STUB] leds_init(%d, %d)\n", pixels_per_strip, brightness);
}

inline int leds_num_strips() {
  return 8;  // Return a reasonable default
}

inline void leds_clear() {
  // No-op
}

inline void leds_brightness(uint8_t brightness) {
  // No-op
}

inline void leds_set(uint8_t* buffer) {
  // No-op
}

inline void leds_send() {
  // No-op
}

#else
// Include the real header when not stubbed
#include "smi_leds.h"
#endif
