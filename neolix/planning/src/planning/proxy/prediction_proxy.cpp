#include "prediction_proxy.h"

#include "src/common/common_macros.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/proxy/perception_obstacle_util.h"

namespace neodrive {
namespace planning {
using neodrive::global::prediction::PredictionObstacles;

void PredictionProxy::SetPrediction(
    const PredictionObstaclesShrPtr &prediction) {
  SET_PTR(prediction_, prediction, "SetPrediction");
  obstacle_ids_.clear();
  obstacles_valid_.clear();
  std::size_t obstacle_size = prediction_->prediction_obstacle_size();
  obstacles_valid_.resize(obstacle_size);
  int valid_obstacle_cnt = 0;
  std::fill(obstacles_valid_.begin(), obstacles_valid_.end(), false);
  for (int i = 0; i < obstacle_size; ++i) {
    auto &obstacle = prediction_->prediction_obstacle(i);
    if (CheckObstacle(obstacle)) {
      obstacle_ids_.push_back(obstacle.id());
      obstacles_valid_[i] = true;
      ++valid_obstacle_cnt;
    }
  }
  LOG_INFO("receive {} valid and {} failed prediction obstacles",
           valid_obstacle_cnt, obstacle_size - valid_obstacle_cnt);
}

bool PredictionProxy::CheckTrajectory(
    const neodrive::global::prediction::Trajectory &trajectory) const {
  for (const auto &point : trajectory.trajectory_point()) {
    if (Double::is_nan(point.path_point().x()) ||
        Double::is_nan(point.path_point().y()))
      return false;
  }
  return true;
}

bool PredictionProxy::CheckObstacle(const PredictionObstacle &obstacle) const {
  check_proto(obstacle, id);
  check_proto(obstacle, timestamp);
  for (auto &trajectory : obstacle.trajectory()) {
    if (!CheckTrajectory(trajectory)) {
      LOG_WARN("Detect an invalid trajectory {} {}", obstacle.id(),
               trajectory.trajectory_point_size());
    }
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive
