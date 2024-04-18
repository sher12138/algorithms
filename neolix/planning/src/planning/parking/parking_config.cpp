#include "parking_config.h"

namespace neodrive {
namespace planning {
ParkingCommonConfig ParkingConfig::common_config() { return common_config_; }

ParkingFirstPlanConfig ParkingConfig::first_plan_park_in_config() {
  return first_plan_park_in_config_;
}

ParkingFirstPlanConfig ParkingConfig::first_plan_park_out_config() {
  return first_plan_park_out_config_;
}

ParkingHybridAStarConfig ParkingConfig::hybrid_a_star_config() {
  return hybrid_a_star_config_;
}

ParkingTrajectoryOptimizerConfig
ParkingConfig::park_trajectory_optimizer_config() {
  return park_trajectory_optimizer_config_;
}

ParkingIterativeAnchoringConfig ParkingConfig::iterative_anchor_config() {
  return iterative_anchor_config_;
}

ParkingDualVariableConfig ParkingConfig::dual_variable_config() {
  return dual_variable_config_;
}

DistanceApproachConfig ParkingConfig::distacne_approach_config() {
  return distacne_approach_config_;
}

bool ParkingConfig::FraseFromConfig() { return true; }

}  // namespace planning
}  // namespace neodrive
