#pragma once

#include <vector>

#include "common/math/vec2d.h"
#include "src/planning/proxy/proxy_type.h"

namespace neodrive {
namespace planning {

class PerceptionProxy {
 public:
  PerceptionProxy() = default;
  ~PerceptionProxy() = default;

  void SetPerception(const PerceptionObstaclesShrPtr& perception);
  const PerceptionObstacles& Perception() const { return *perception_; }
  const std::vector<int>& GetObstacleIds() const { return obstacle_ids_; }
  const std::vector<bool>& GetObstaclesValid() const {
    return obstacles_valid_;
  }
  double Timestamp() const { return perception_->header().timestamp_sec(); }

 private:
  bool CheckObstacle(const PerceptionObstacle& obstacle) const;

 private:
  PerceptionObstaclesShrPtr perception_{
      std::make_shared<PerceptionObstacles>()};
  std::vector<int> obstacle_ids_;
  std::vector<bool> obstacles_valid_;
};

}  // namespace planning
}  // namespace neodrive
