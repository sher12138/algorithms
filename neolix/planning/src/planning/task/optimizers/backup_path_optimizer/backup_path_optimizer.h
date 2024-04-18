/// Optimizer to generate backup path points
#pragma once

#include <functional>

#include "backup_path_model.h"
#include "common/visualizer_event/visualizer_event.h"
#include "math/frame_conversion/sl_analytic_transformation.h"
#include "planning/scenario_manager/scenario_common.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/math/curve1d/spline.h"
#include "src/planning/math/hpipm_solver/backup_path_hpipm_solver.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "task/optimizers/backup_path_optimizer/backup_path_planner_factory.h"
#include "util/path_planner_common.h"

namespace neodrive {
namespace planning {

enum class PoseType {
  REASONABLE = 0,
  UNREASONABLE_LOCATION,
  UNREASONABLE_ORIENTATION,
  UNREASONABLE_BOTH
};
enum class CollisionType {
  NO_COLLISION = 0,
  VIS_CURB_COLLISION,
  MAP_ROAD_COLIISION,
  OBS_COLIISION,
  PATH_ERROR
};

constexpr static double kOrientThreshInLane = 30.0;
constexpr static double kOrientThreshOutLane = 25.0;
constexpr static double kOrientThreshInLaneIndoor = 40.0;
constexpr static double kOrientThreshOutLaneIndoor = 35.0;
constexpr static double kLowSpeedThresh = 0.0;

inline double Deg2Red(double deg) { return deg * M_PI / 180; }
inline double Red2Deg(double rad) { return rad * 180 / M_PI; }

/// class to generate path if main planner is failed
class BackupPathOptimizer final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(BackupPathOptimizer);

 public:
  virtual ~BackupPathOptimizer() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  void PlannerPrepare(TaskInfo& task_info);
  void PathDataReset();

 private:
  Vec3d adc_odom_start_backup_{0., 0., 0.};
  PoseType pose_type_{PoseType::REASONABLE};
  CollisionType collision_type_{CollisionType::NO_COLLISION};

  char backup_monitor_str_[256];
  double gain_{1.0};
  std::vector<BackupPathPlanner::WayPoint> way_pts_{};
  std::vector<PathPoint> out_path_pts_{};
  std::vector<FrenetFramePoint> out_sl_pts_{};
  std::vector<ReferencePoint> out_ref_pts_{};
  std::vector<std::pair<std::string, std::vector<std::function<bool()>>>>
      backup_funcs_{};
};

REGISTER_SCENARIO_TASK(BackupPathOptimizer);
}  // namespace planning
}  // namespace neodrive
