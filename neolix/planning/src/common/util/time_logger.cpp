#include "time_logger.h"

namespace neodrive {
namespace common {
namespace util {

TimeLogger::TimeLogger(const std::string& logger_name)
    : logger_name_(logger_name),
      start_time_(GetCurrentTimeseocnd()),
      total_(0.0),
      log_stream_("") {}

void TimeLogger::ResetStartTime() {
  start_time_ = GetCurrentTimeseocnd();
  total_ = 0.0;
  log_stream_ = "";
}

// in usec unit
void TimeLogger::RegisterTime(const std::string& time_seg_name) {
  if (disable_) {
    return;
  }
  double end = GetCurrentTimeseocnd();
  double interval = (end - start_time_);
  log_stream_.append(time_seg_name + "[" + std::to_string(interval) + "], ");
  total_ += interval;
  start_time_ = end;
}

void TimeLogger::RegisterTimeAndPrint(const std::string& time_seg_name) {
  if (disable_) {
    return;
  }
  RegisterTime(time_seg_name);
  LOG_INFO("{}:total[{}],{}", logger_name_, total_, log_stream_);
  log_stream_ = "";
}

void TimeLogger::Print() {
  if (disable_) {
    return;
  }
  LOG_INFO("{}:total[{}],{}", logger_name_, total_, log_stream_);
  log_stream_ = "";
}

void TimeLogger::SetDisable(bool disable) { disable_ = disable; }

uint64_t TimeLogger::GetCurrentTimestamp() {
  timeval tm;
  gettimeofday(&tm, nullptr);
  return uint64_t(tm.tv_sec * 1000000 + tm.tv_usec);
}

double TimeLogger::GetCurrentTimeseocnd() {
  timeval tm;
  gettimeofday(&tm, nullptr);
  return double(tm.tv_sec + tm.tv_usec * 1.0e-6);
}

uint64_t TimeLogger::GetCurrentTimeMillisecond() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return uint64_t(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);

  timeval tm;
  gettimeofday(&tm, nullptr);
  return uint64_t(tm.tv_sec * 1000 + tm.tv_usec / 1000);
}

}  // namespace util
}  // namespace common
}  // namespace neodrive
