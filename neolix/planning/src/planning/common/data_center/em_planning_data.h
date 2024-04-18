#pragma once

#include "planning_data.h"
#include "src/planning/common/path/path_data.h"
#include "src/planning/common/planning_macros.h"
#include "src/planning/common/speed/speed_data.h"

namespace neodrive {
namespace planning {

class EMPlanningData : public PlanningData {
 public:
  EMPlanningData() = default;
  virtual ~EMPlanningData() override = default;

  virtual std::string type() const override;

  bool AggregatePathAndSpeedData(const double time_resolution,
                                 const double time_length);

 public:
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(PathData, path_data)
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(SpeedData, speed_data)

 private:
  PathData path_data_{};
  SpeedData speed_data_{};
};

}  // namespace planning
}  // namespace neodrive
