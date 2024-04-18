#include "planning_data.h"

namespace neodrive {
namespace planning {

const DecisionData &PlanningData::decision_data() const {
  return *(decision_data_ptr_);
}

DecisionData *PlanningData::mutable_decision_data() const {
  return decision_data_ptr_.get();
}

void PlanningData::set_decision_data(
    std::shared_ptr<DecisionData> &decision_data_shr_ptr) {
  if (decision_data_shr_ptr != nullptr) {
    decision_data_ptr_ = decision_data_shr_ptr;
  } else {
    LOG_WARN("ignore nullptr decision_data_shr_ptr");
  }
}

void PlanningData::set_init_planning_point(const PathPoint &init_path_point,
                                           const SpeedPoint &init_speed_point) {
  init_planning_point_ = TrajectoryPoint(
      init_path_point, init_speed_point.v(), init_speed_point.a(),
      init_speed_point.j(), init_speed_point.t());
}

std::string PlanningData::type() const { return "PlanningData"; }

}  // namespace planning
}  // namespace neodrive
