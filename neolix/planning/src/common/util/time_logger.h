#pragma once

#include <sys/time.h>

#include <chrono>
#include <string>

#include "neolix_log.h"

namespace neodrive {
namespace common {
namespace util {

class TimeLogger {
 public:
  explicit TimeLogger(const std::string& logger_name);

  // reset the start time with current time
  void ResetStartTime();

  // regist current time as end time, and record it with a name, then the start
  // time is current time
  void RegisterTime(const std::string& time_seg_name);

  // regist and output the record info to log
  void RegisterTimeAndPrint(const std::string& time_seg_name);
  // output the record info to log
  void Print();

  void SetDisable(bool disable);

  static uint64_t GetCurrentTimestamp();

  static double GetCurrentTimeseocnd();

  static uint64_t GetCurrentTimeMillisecond();

 private:
  std::string logger_name_;
  double start_time_;
  double total_;
  std::string log_stream_;
  bool disable_ = false;
};

}  // namespace util
}  // namespace common
}  // namespace neodrive
