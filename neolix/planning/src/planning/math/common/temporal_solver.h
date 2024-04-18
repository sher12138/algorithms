#pragma once

#include <deque>
#include <functional>
#include <map>
#include <queue>
#include <vector>

#include "common/math/polygon2d.h"
#include "common/math/segment2d.h"
#include "common/math/vec2d.h"
#include "cyber/common/macros.h"
#include "src/planning/math/common/geometry.h"
#include "src/planning/math/common/obsmap.h"
#include "src/planning/math/common/obstacle_frame_container.h"

namespace neodrive {
namespace planning {
namespace obsmap {

struct TrajectoryData {
  int id{-1};
  std::vector<double> timestamp{};
  std::vector<math::AD2> position{};
  std::vector<double> heading{};
  std::vector<math::Polygon> polygon{};
};

struct CollisionPoint {
  double time{-1.0};
  int obs_idx{-1};
  math::AD2 position{0.0, 0.0};
  void Reset() {
    time = -1.0;
    obs_idx = -1;
    position[0] = position[1] = 0.0;
  }
};

class TemporalObsMap : public planning::obsmap::ObsMap {
 public:
};

class Solution {
 public:
  Solution() = default;
  virtual void PreProcess(const std::vector<TrajectoryData>& data) {}
  virtual bool Solve(const std::vector<TrajectoryData>& data) = 0;
  virtual bool Process(const std::vector<TrajectoryData>& data) = 0;
  planning::obsmap::ObsMap& obsmap() { return obsmap_; }

 protected:
  CollisionPoint res_;
  planning::obsmap::ObsMap obsmap_{100, true};
};

class TemporalCollision : public Solution {
 public:
  TemporalCollision() = default;
  void PreProcess(const std::vector<TrajectoryData>& data);

  bool Solve(const std::vector<TrajectoryData>& data);

  bool Process(const std::vector<TrajectoryData>& data);

  std::vector<const math::Node<math::Polygon>*> DetectCollisionAt(
      int frame_id, const math::Polygon& ego);

  std::vector<const math::Node<math::Polygon>*> DetectCollisionAt(
      double frame_timestamp, const math::Polygon& ego);

 private:
  std::vector<std::pair<int, int>> time_idx_;
  bool log_flag_{false};
};

}  // namespace obsmap
}  // namespace planning
}  // namespace neodrive
