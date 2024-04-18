#pragma once
#include <cyber/time/time.h>

#include "src/planning/common/planning_logger.h"

namespace neodrive {
namespace planning {

struct TimeStatisticInfo {
  std::string sign{};
  double abs_time{0.};
  double rel_time{0.};
};

class PrintUtil {
 public:
  /// @brief Use this function to track time at the location you want to track
  /// @param s String markers for statistical time
  void TimeStatisticTool(const std::string &s);

  /// @brief Print time statistics and empty container
  /// @param s String markers for statistical time
  void TimeStatisticPrint(const std::string &s);

 public:
  std::vector<TimeStatisticInfo> time_statistic_{};
};

}  // namespace planning
}  // namespace neodrive
