#include "idm_velocity_controller.h"

#include <math.h>

namespace neodrive {
namespace planning {
namespace sim_planner {

using neodrive::planning::sim_planner::IntelligentDriverModel;

bool IntelligentVelocityControl::calculateDesiredVelocity(
    const IdmParam& param, const double s, const double s_front, const double v,
    const double v_front, const double dt, double* velocity_at_dt) {
  IntelligentDriverModel model(param);
  IntelligentDriverModel::State state;
  state.s = s;
  state.v = std::max(0.0, v);
  state.s_front = s_front;
  state.v_front = v_front;
  model.setState(state);
  model.Step(dt);

  auto desired_state = model.state();

  *velocity_at_dt = std::max(0.0, desired_state.v);

  return true;
}

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive
