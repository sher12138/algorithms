#pragma once

#include "src/planning/common/obstacle/decision_data.h"
#include "src/planning/common/trajectory/publishable_trajectory.h"
#include "src/planning/common/trajectory/trajectory_point.h"
#include "src/planning/reference_line/reference_line.h"

namespace neodrive {
namespace planning {

class PlanningData {
 public:
  PlanningData() = default;
  virtual ~PlanningData() = default;

  const DecisionData &decision_data() const;
  DecisionData *mutable_decision_data() const;
  void set_decision_data(std::shared_ptr<DecisionData> &decision_data_shr_ptr);
  void set_init_planning_point(const PathPoint &init_path_point,
                               const SpeedPoint &init_speed_point);
  virtual std::string type() const;

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(std::shared_ptr<DecisionData>,
                                   decision_data_ptr);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(ReferenceLinePtr, reference_line);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(PublishableTrajectory,
                                       computed_trajectory)
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(TrajectoryPoint, init_planning_point)

 protected:
  std::shared_ptr<DecisionData> decision_data_ptr_{
      std::make_shared<DecisionData>()};
  ReferenceLinePtr reference_line_{nullptr};
  PublishableTrajectory computed_trajectory_{};
  TrajectoryPoint init_planning_point_{};
};

}  // namespace planning
}  // namespace neodrive
