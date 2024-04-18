#pragma once

#include <vector>

#include "src/planning/common/planning_code_define.h"
#include "common/math/vec2d.h"
#include "common/math/double.h"
#include "src/planning/math/public_math/utility.h"
#include "src/planning/proxy/proxy_type.h"

namespace neodrive {
namespace planning {

class PredictionProxy {
 public:
  PredictionProxy() = default;
  ~PredictionProxy() = default;

  void SetPrediction(const PredictionObstaclesShrPtr& prediction);
  const PredictionObstacles& Prediction() const { return *prediction_; }
  const std::vector<int32_t>& GetObstacleIds() const { return obstacle_ids_; }
  const std::vector<bool>& GetObstaclesValid() const {
    return obstacles_valid_;
  }
  double Timestamp() const { return prediction_->header().timestamp_sec(); }

 private:
  bool CheckTrajectory(
      const neodrive::global::prediction::Trajectory& trajectory) const;
  bool CheckObstacle(const PredictionObstacle& obstacle) const;

 private:
  PredictionObstaclesShrPtr prediction_{
      std::make_shared<PredictionObstacles>()};
  std::vector<int> obstacle_ids_;
  std::vector<bool> obstacles_valid_;
};

}  // namespace planning
}  // namespace neodrive
