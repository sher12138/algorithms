#pragma once

#include <string>
#include <vector>

#include "src/planning/common/math/aabox2d.h"
#include "src/planning/common/math/vec2d.h"
#include "src/planning/common/path/sl_point.h"

namespace neodrive {
namespace planning {

namespace PathRegion {
struct Bound {
  enum class BoundType { LANE = 0, ROAD, OBS, VIR };
  SLPoint upper_point{};
  SLPoint lower_point{};
  SLPoint upper_lane_bound_point{};
  SLPoint lower_lane_bound_point{};
  SLPoint upper_road_bound_point{};
  SLPoint lower_road_bound_point{};
  int upper_id{-9999};
  int lower_id{-9999};
  int upper_obs_type{-1};  // corresponding Obstacle::ObstacleType
  int lower_obs_type{-1};  // corresponding Obstacle::ObstacleType
  BoundType upper_type = BoundType::LANE;
  BoundType lower_type = BoundType::LANE;
};

}  // namespace PathRegion

struct PieceBoundary {
  bool left_is_obs{false};
  bool right_is_obs{false};
  double s{0.0};
  double left_bound{0.0};  // max solve space
  double right_bound{0.0};
  double left_lane_bound{0.0};  // navigation lane bound
  double right_lane_bound{0.0};
  double left_road_bound{0.0};  // navigation road bound
  double right_road_bound{0.0};

  void Reset() {
    left_is_obs = false;
    right_is_obs = false;
    s = 0.0;
    left_bound = 0.0;
    right_bound = 0.0;
    left_lane_bound = 0.0;
    right_lane_bound = 0.0;
    left_road_bound = 0.0;
    right_road_bound = 0.0;
  }
};

struct PathBoundary {
  std::vector<PathRegion::Bound> path_boundary{};
  std::vector<Vec2d> left_xy_boundary{};
  std::vector<Vec2d> right_xy_boundary{};

  void Reset() {
    path_boundary.clear();
    left_xy_boundary.clear();
    right_xy_boundary.clear();
  }
};

struct PathGoalPoints {
  std::vector<SLPoint> goal_sl_points{};
  std::vector<Vec2d> goal_xy_points{};

  void Reset() {
    goal_sl_points.clear();
    goal_xy_points.clear();
  }
};

struct PiecewisePathContext {
  bool context_valid{false};
  std::vector<double> history_piecewise_valid_len{};

  void Reset() {
    context_valid = false;
    history_piecewise_valid_len.clear();
  }
};

struct PathContext {
  bool context_valid{false};
  PiecewisePathContext piecewise_path_context{};
  double prefer_valid_length{0.};
  double piecewise_delta_s{0.6};
  PathBoundary original_path_boundary{};
  PathBoundary shrink_path_boundary{};
  PathGoalPoints path_goal_points{};
  double valid_region_end_s{};
  double valid_backup_end_s{};
  std::vector<AABox2d> lateral_static_obstacles{};

  void Reset() {
    context_valid = false;
    piecewise_path_context.Reset();
    prefer_valid_length = 0.0;
    piecewise_delta_s = 0.6;
    original_path_boundary.Reset();
    shrink_path_boundary.Reset();
    path_goal_points.Reset();
    valid_region_end_s = 0.0;
    valid_backup_end_s = 0.0;
    lateral_static_obstacles.clear();
  }
};

}  // namespace planning
}  // namespace neodrive
