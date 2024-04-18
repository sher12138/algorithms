#pragma once

#include "src/planning/common/path/path_point.h"
#include "src/planning/config/auto_planning_research_config.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedPerceptionObstacleCautionDecider final
    : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedPerceptionObstacleCautionDecider);

 public:
  virtual ~MotorwaySpeedPerceptionObstacleCautionDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  bool Init(TaskInfo& task_info);
  bool Process(TaskInfo& task_info);

 private:
  bool BuildPathAdcBoundingBoxes(const InsidePlannerData& inside_data,
                                 const PathData& path_data);
  ErrorCode StaticObsCollisionCheck(
      const InsidePlannerData& inside_data,
      const std::unordered_map<int, Obstacle>& caution_obstacles,
      OutsidePlannerData* const outside_data);

  ErrorCode CollisionCheckObstacleWithoutTrajectory(
      const InsidePlannerData& inside_data, const Obstacle& obstacle,
      OutsidePlannerData* const outside_data);

  bool ComputeSpeedLimit(TaskInfo& task_info);

 private:
  const config::AutoPlanningResearchConfig::
      SpeedPerceptionObstacleCautionDeciderConfig*
          speed_perception_obstacle_caution_config_{nullptr};
  std::vector<Box2d> path_adc_boxes_{};
  std::vector<double> collide_s_vec_{};

  double adc_current_v_{0.0};
  bool update_limited_speed_{false};
  double last_limited_speed_{std::numeric_limits<double>::infinity()};
  double limited_speed_{std::numeric_limits<double>::infinity()};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedPerceptionObstacleCautionDecider);

}  // namespace planning
}  // namespace neodrive
