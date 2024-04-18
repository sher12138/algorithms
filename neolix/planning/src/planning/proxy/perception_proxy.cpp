#include "perception_proxy.h"

#include "common/math/double.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/math/public_math/utility.h"
#include "src/planning/proxy/perception_obstacle_util.h"

namespace neodrive {
namespace planning {
using neodrive::global::perception::PerceptionObstacles;

void PerceptionProxy::SetPerception(
    const PerceptionObstaclesShrPtr &perception) {
  SET_PTR(perception_, perception, "SetPerception");
  obstacle_ids_.clear();
  obstacles_valid_.clear();
  std::size_t obstacle_size = perception_->perception_obstacle_size();
  obstacles_valid_.resize(obstacle_size);
  int valid_obstacle_cnt = 0;
  std::fill(obstacles_valid_.begin(), obstacles_valid_.end(), false);
  for (int i = 0; i < obstacle_size; ++i) {
    auto &obstacle = perception_->perception_obstacle(i);
    if (CheckObstacle(obstacle)) {
      obstacle_ids_.push_back(obstacle.id());
      obstacles_valid_[i] = true;
      ++valid_obstacle_cnt;
    }
  }
  LOG_INFO("receive {} valid and {} failed perception obstacles",
           valid_obstacle_cnt, obstacle_size - valid_obstacle_cnt);
}

bool PerceptionProxy::CheckObstacle(const PerceptionObstacle &obstacle) const {
  bool check_res = true;
  if (!CheckPolygon(obstacle)) {
    LOG_WARN("CheckPolygon failed: {}", obstacle.id());
    check_res = false;
  } else if (Double::is_nan(obstacle.position().x()) ||
             Double::is_nan(obstacle.position().y()) ||
             Double::is_nan(obstacle.position().z())) {
    LOG_WARN("perception_obstacle().position().x/y/z failed: {}",
             obstacle.id());
    check_res = false;
  } else if (Double::is_nan(obstacle.velocity().x()) ||
             Double::is_nan(obstacle.velocity().y()) ||
             Double::is_nan(obstacle.velocity().z())) {
    LOG_WARN("perception_obstacle().velocity().x/y/z failed: {}",
             obstacle.id());
    check_res = false;
  } else if (obstacle.width() <= 0.0 || obstacle.length() <= 0.0 ||
             obstacle.height() <= 0.0) {
    LOG_WARN("perception_obstacle().width/length/height failed: {}",
             obstacle.id());
    check_res = false;
  }
  // if (check_res == false) {
  //   LOG_DEBUG("Detect an invalid obstacle, continued...\n{}",
  //             obstacle.DebugString());
  // }
  return check_res;
}

}  // namespace planning
}  // namespace neodrive
