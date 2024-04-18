#pragma once

#include "ctx_idm_model.h"
#include "intelligent_driver_model.h"

namespace neodrive {
namespace planning {
namespace sim_planner {

class ContextIntelligentVelocityControl {
 public:
  static bool calculateDesiredVelocity(
      const control::IntelligentDriverModel::Param& idm_param,
      const ContextIntelligentDriverModel::CtxParam& ctx_param, const double s,
      const double s_front, const double s_target, const double v,
      const double v_front, const double v_target, const double dt,
      double* velocity_at_dt);
};

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive
