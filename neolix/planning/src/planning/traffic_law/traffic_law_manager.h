#pragma once

#include <mutex>

#include "clear_zone_law.h"
#include "common/util/time_logger.h"
#include "crosswalk_law.h"
#include "reference_line/reference_line.h"
#include "restricted_area_law.h"
#include "src/planning/math/public_math/utility.h"
#include "src/planning/task/task_info.h"
#include "stop_sign_law.h"
#include "traffic_law.h"
#include "traffic_light_law.h"
#include "y_junction_law.h"

namespace neodrive {
namespace planning {

class TrafficLawManager {
 public:
  TrafficLawManager();
  ~TrafficLawManager() = default;
  int ApplyLaw(TaskInfo* const task_info,
               const InsidePlannerData& inside_planner_data,
               const OutsidePlannerData& outside_data);

 private:
  int CalcAdcBoundary(const TaskInfo& task_info,
                      Boundary* const adc_boundary) const;

  Vec2d Rotate(const Vec2d& vec, const double theta) const;
};

}  // namespace planning
}  // namespace neodrive
