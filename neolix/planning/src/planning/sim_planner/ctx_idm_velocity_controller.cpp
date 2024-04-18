#include "ctx_idm_velocity_controller.h"

namespace neodrive {
namespace planning {
namespace sim_planner {

bool ContextIntelligentVelocityControl::calculateDesiredVelocity(
    const ContextIntelligentDriverModel::IdmParam& idm_param,
    const ContextIntelligentDriverModel::CtxParam& ctx_param, const double s,
    const double s_front, const double s_target, const double v,
    const double v_front, const double v_target, const double dt,
    double* velocity_at_dt) {
  ContextIntelligentDriverModel model(idm_param, ctx_param);
  ContextIntelligentDriverModel::CtxIdmState state;
  state.s = s;
  state.s_front = s_front;
  state.s_target = s_target;
  state.v = std::max(0.0, v);
  state.v_front = v_front;
  state.v_target = v_target;

  model.setState(state);
  model.Step(dt);

  auto desired_state = model.state();
  *velocity_at_dt = std::max(0.0, desired_state.v);

  return true;
}

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive
