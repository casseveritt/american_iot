#pragma once
#include <cstdint>
#include <ctime>

inline int64_t get_time_nsec() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000000000 + ts.tv_nsec;
}
