#pragma once

#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
namespace neodrive {
namespace planning {

class PathCutinObsAvoidDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(PathCutinObsAvoidDecider);

 public:
  virtual ~PathCutinObsAvoidDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};
  struct PathCollisionDynamicObsData {
    int id{0};
    int count{0};
    int lost_cnt{0};
    bool first_time{true};
    double heading_diff{0.0};
    std::deque<double> heading_diffs{};
    double l_dis{std::numeric_limits<double>::infinity()};
    double last_heading{0.0};
    double accum_heading_diff{0.0};

    void Reset() {
      id = 0;
      count = 0;
      lost_cnt = 0;
      first_time = true;
      heading_diff = 0.0;
      heading_diffs.clear();
      l_dis = std::numeric_limits<double>::infinity();
      last_heading = 0.0;
      accum_heading_diff = 0.0;
    }
  };
  std::unordered_map<int, PathCollisionDynamicObsData>
      left_collision_dynamic_obstacles;
  std::unordered_map<int, PathCollisionDynamicObsData>
      right_collision_dynamic_obstacles;

 private:
  double min_left_obs_l_{0.0};
  double max_right_obs_l_{0.0};
  const double check_region_e_s_{10.0};
  const double check_region_s_s_{12.0};
  const double check_region_w_{4.0};
  const double occupied_reigon_l_{0.3};
  double occupied_front_s_{0.5};
  const double occupied_back_s_{0.5};
  const double obs_turning_threshold_{0.2};
  double max_t_{3.0};
  double l_ttc_{1e4};
  double s_ttc_{1e4};
  bool InitCollisionCheckRegion(TaskInfo& task_info);
  bool DynamicObsProcess(TaskInfo& task_info);
  bool TurningObstacleNeedAvoidance(
      const Obstacle* const obs,
      std::unordered_map<int, PathCollisionDynamicObsData>&
          collision_dynamic_obstacles);
  bool ExtendAvoidanceObs(TaskInfo& task_info, const Obstacle* ori_obs);

  void UpdateDynmaicObsInfo(
      const Obstacle* const obs,
      std::unordered_map<int, PathCollisionDynamicObsData>& dynamic_obs,
      const double& heading_diff);
  void clearObsoleteObstacles(
      const std::vector<neodrive::planning::Obstacle*>& dyn_obs,
      std::unordered_map<int, PathCollisionDynamicObsData>& obs_data);
  bool CreateAvoidanceStaticObs(TaskInfo& task_info);
  Boundary right_collision_check_region_{};
  Boundary left_collision_check_region_{};
  Polygon2d adc_occupied_reigon_{};
};

REGISTER_SCENARIO_TASK(PathCutinObsAvoidDecider);
}  // namespace planning
}  // namespace neodrive
