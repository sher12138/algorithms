#pragma once

#include <time.h>

#include <chrono>
#include <string>
#include "cyber.h"
#include "neolix_log.h"

namespace neodrive {
namespace planning_rl {
namespace utils {

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

 private:
  std::string logger_name_;
  double start_time_;
  double total_;
  std::string log_stream_;
  bool disable_ = false;
};

}  // namespace utils
}  // namespace planning_rl
}  // namespace neodrive
