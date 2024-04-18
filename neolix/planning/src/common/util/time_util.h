#pragma once
#include <stdint.h>
#include <time.h>

#include <memory>
namespace neodrive {
namespace common {
static inline double Now() {
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static inline double NowSec() {
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static inline uint64_t NowMilliSec() {
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return ts.tv_sec * 1000 + ts.tv_nsec * 1e-6;
}

static inline double NowMicroSec() {
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return ts.tv_sec * 1000000 + ts.tv_nsec * 1e-3;
}

static inline uint64_t NowNanoSec() {
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return ts.tv_sec * 1000000000 + ts.tv_nsec;
}
}  // namespace common
}  // namespace neodrive