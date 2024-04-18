#include "time_logger.h"

namespace neodrive {
namespace planning_rl {
namespace utils {

TimeLogger::TimeLogger(const std::string& logger_name)
    : logger_name_(logger_name),
      start_time_(cyber::Time::Now().ToSecond()),
      total_(0.0),
      log_stream_("") {}

void TimeLogger::ResetStartTime() {
  start_time_ = cyber::Time::Now().ToSecond();
  total_ = 0.0;
  log_stream_ = "";
}

// in usec unit
void TimeLogger::RegisterTime(const std::string& time_seg_name) {
  if (disable_) {
    return;
  }
  double end = cyber::Time::Now().ToSecond();
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

}  // namespace utils
}  // namespace planning_rl
}  // namespace neodrive
