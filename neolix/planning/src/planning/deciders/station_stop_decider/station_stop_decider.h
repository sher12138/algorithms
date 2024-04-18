#pragma once
#include <deque>

#include "routing.pb.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class StationStopDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(StationStopDecider);

 public:
  virtual ~StationStopDecider() override;

  ErrorCode Execute(TaskInfo &task_info) override;
  void SaveTaskResults(TaskInfo &task_info) override;
  void Reset() override;

  bool WhetherRestartFromStationStop();

 private:
  void Clear();

  bool UpdateRoutingRequest(TaskInfo &task_info);

  void UpdateStateInfo(TaskInfo &task_info);

  bool WhetherReachFinalStation(TaskInfo &task_info);

  bool WhetherReachFrontInnerStation(TaskInfo &task_info);

  void WhetherTakeOverOrMove(TaskInfo &task_info);

  void WhetherSafePullOver(TaskInfo &task_info);

  void WhetherFormationPullOver(
      TaskInfo &task_info,
      const std::vector<PathObstacleBoundary> &obstacles_boundary);

  void WhetherPullOverInPlace(TaskInfo &task_info);

  bool GetObstaclesBoundary(
      const ReferenceLinePtr &reference_line,
      const std::shared_ptr<DecisionData> &decision_data,
      std::vector<PathObstacleBoundary> *obstacles_boundary);

  bool CalcLeftAndRightLaneBound(const ReferenceLinePtr &reference_line,
                                 const double origin_start_s,
                                 const double origin_end_s, const double offset,
                                 double &left_bound, double &right_bound);

  void CalFrontAndBackSafeDistance(
      TaskInfo &task_info,
      const std::vector<PathObstacleBoundary> &obstacles_boundary,
      const ReferencePoint &destination_point, double &front_safe_dis,
      double &back_safe_dis, bool &obstacle_in_station);

  bool IsAllowPullOverInTurn(TaskInfo &task_info);

  bool IsDestinationInUnloadingZone(TaskInfo &task_info);

  bool IsDestinationInShoulder(TaskInfo &task_info);

 private:
  struct station_info {
    double need_stop_time = 0.0;
    bool need_pullover = false;
    ReferencePoint station_pos{};
  };
  std::deque<station_info> station_list_;

  station_info current_station_;
  double inner_station_stop_time_;
  bool is_station_stop_{false};
  bool is_station_stop_need_pull_over_{false};
  bool behavior_stop_vehicle_{false};
  bool is_in_inner_station_mode_{false};
  ReferencePoint station_stop_pos_{};
  bool has_inner_station_{false};
  bool is_destination_in_unloading_zone_{false};
  bool is_destination_changed_{false};
  bool is_destination_in_shoulder_{false};
  bool obstacle_in_station_{false};
  bool destination_back_has_obs_{false};
  bool destination_front_has_obs_{false};

  std::vector<PathObstacleBoundary> obstacles_boundary_{};
  std::vector<Obstacle> dynamic_obstacles_{};
  int obs_behind_end_id_{0};
  double moved_distance_to_end_;
  double left_lane_max_bound_{0.};
  double right_lane_max_bound_{0.};
  double front_obs_dis_to_end_{0.};
  double back_obs_dis_to_end_{0.};
  double move_on_dis_{0.};
  double stop_in_advance_distance_{0.};
  double distance_to_new_end_{0.};
  std::vector<int> obs_idxes_unload_{};
  ReferencePoint dest_ref_point_{};

  static constexpr double kSafeFrontLength = 5.0;
  static constexpr double kSafeBackLength = 10.0;
  static constexpr double kSafeLength = 12.0;
};

REGISTER_SCENARIO_TASK(StationStopDecider);

}  // namespace planning
}  // namespace neodrive
