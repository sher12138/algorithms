#pragma once

#include "idm_model.h"

namespace neodrive {
namespace planning {
namespace sim_planner {

/**
 * @brief Intelligent driver model
 * (https://en.wikipedia.org/wiki/Intelligent_driver_model).
 */
class IntelligentVelocityControl {
 public:
  using IdmParam =
      neodrive::planning::sim_planner::IntelligentDriverModel::Param;

  static bool calculateDesiredVelocity(const IdmParam& param, const double s,
                                       const double s_front, const double v,
                                       const double v_front, const double dt,
                                       double* velocity_at_dt);
};

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive
