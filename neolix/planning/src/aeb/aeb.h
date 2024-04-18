#pragma once

#include <functional>
#include <unordered_map>

#include "common/aeb_context.h"
#include "common/aeb_obstacle.h"
#include "common/aeb_type.h"

namespace neodrive {
namespace aeb {
class Aeb {
 public:
  void Init();
  void RunOnce();

 private:
  bool JudgeAebActive();
  void CheckForbidAeb();
  void CalculateTurnRadius();
  void SelectFrontObstacles();
  void CalculateStopDist();
  void CheckInputTimeout();
  void GenerateLocalTrajectory();
  bool CheckFreespace();
  bool CheckIfInSkipSector(Point2D& freespace_pt);
  void FreespaceTranspose();
  SectorType GetObstacleSector(AebObstaclePtr aeb_obstacle);
  bool CheckIsSkipCondition();
  bool CheckRunningObstacle(AebObstaclePtr aeb_obstacle);
  bool CheckObstacle(AebObstaclePtr aeb_obstacle);
  void GeneratePolygonPoints(Point2D& start, Point2D& end,
                             std::vector<Point2D>& out_pts);

 private:
  static constexpr double kAebStopSpeed = 0.01;
  static constexpr double kAebStopObstacleVelocity = 0.1;
  static constexpr double kMsgTimeoutThreshold = 1.0;
  static constexpr double kMaxCurvatureRadius = 20000.0;
  static constexpr double kAebBrakePercent = 100;
  static constexpr double kMaxTTC = 20.0;

 private:
  AebConfig* aeb_config_ = nullptr;
  neodrive::common::config::CommonConfig* common_config_ = nullptr;
  std::vector<AebObstaclePtr> consider_obstacles_;
  std::vector<AebObstaclePtr> back_obstacles_;
  std::vector<std::pair<double, double>> skip_sectors_;
  double curvature_radius_ = 0.0;
  Point2D curve_center_;
  double ego_spped_ = 0.0;
  double inner_radius_ = 0.0;
  double outer_radius_ = 0.0;
  double stop_distance_ = 0.0;
  double current_heading_ = 0.0;
  double current_speed_ = 0.0;
  double pre_follow_moving_obs_t_ = 0.0;
  bool initialized_ = false;
  bool pre_aeb_active_ = false;
  bool is_chassis_msg_timeout_ = false;
  bool is_freespace_msg_timeout_ = false;
  bool is_perception_obstacle_msg_timeout_ = false;
  int freespace_trigger_count_ = 0;
  int obstacle_trigger_count_ = 0;
};

}  // namespace aeb
}  // namespace neodrive
