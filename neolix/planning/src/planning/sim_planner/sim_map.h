#pragma once

#include "common/macros.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/math/common/geometry.h"
#include "src/planning/math/common/kdtree.h"
#include "src/planning/reference_line/reference_line.h"
#include "src/planning/task/task_info.h"

namespace neodrive {
namespace planning {
namespace sim_planner {

using namespace planning;

struct SimMapPoint {
  double x, y, s, l = 0.0;
};
struct SimMapLane {
  int lane_id;  // ... 2, 1, 0, -1, -2 ...
  bool valid{false};
  std::vector<SimMapPoint> pts;
};
struct SimMapRoad {
  int road_id;
  double s_s, s_e;
  std::vector<SimMapLane> lanes;
};

enum class LateralBehavior {
  kUndefined = 0,
  kLaneKeeping,
  kLaneChangeLeft,
  kLaneChangeRight,
};

enum class LongitudinalBehavior {
  kMaintain = 0,
  kAccelerate,
  kDecelerate,
  kStopping
};

class SimMap {
  DECLARE_SINGLETON(SimMap);

 public:
  ~SimMap() = default;

  bool createSimMap(TaskInfo& task_info);

  const std::vector<SimMapRoad>& road_map() { return road_map_; }

  ReferencePointVec1d getRefPoints();

  bool getNearestPoint(const Vec2d& pt, SimMapPoint* nearest_pt);

  bool getNearestPoint(const SLPoint& sl_pt, SimMapPoint* nearest_pt);

  bool getRoadLaneId(const Vec2d& pt, int* road_id, int* lane_id);

  bool searchPoint(const Vec2d& pt, const double expact_extend_length,
                   const int expact_extend_lane_nums, Vec2d* goal_pt,
                   double* real_extend_length, int* real_extend_lane_nums);

  bool collisionCheck(const math::Polygon& polygon);

  bool searchBoundary(const SLPoint& sl_pt, const double s, Boundary* boundary,
                      bool* bisexual);

  bool searchBoundary(const Vec2d& pt, const double s, Boundary* boundary,
                      bool* bisexual);

  bool searchRoadBound(const Vec2d& pt, double* l_bound, double* r_bound);

 private:
  ReferenceLinePtr ref_ptr_;
  math::KdTree<math::Node<math::Polygon>> polygon2d_kdtree_{};
  std::vector<Boundary> boundaries_{};

  bool sim_map_valid_{false};
  std::vector<SimMapRoad> road_map_{};
};

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive