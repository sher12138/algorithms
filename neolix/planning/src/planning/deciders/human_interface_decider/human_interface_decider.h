#pragma once

#include "config/planning_config.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "src/planning/util/history_data.h"

namespace neodrive {
namespace planning {

class HumanInterfaceDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(HumanInterfaceDecider);

 public:
  virtual ~HumanInterfaceDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  // TODO: need to implemented in future
  void SaveTaskResults(TaskInfo& task_info) override {}
  void Reset() override;

  bool UpdateStopReason(
      const MasterInfo& master_info,
      const neodrive::global::status::GlobalState& curr_global_state);
  void AbnormalStopMonitor();
  void ObstacleBlockMonitor();
  void CheckReachStation();
  void UpdateClosestObsS(const double s);

 private:
  void UpdateFrontObsInfo();

 private:
  HistoryData not_sop_;
  HistoryData stop_from_planning_;
  HistoryData vehicle_collision_;
  HistoryData invalid_pos_;
  Vec3d routing_dest_;

  // this is used to record time period to ask for take-over
  double planning_ask_take_over_start_{0.};
  bool planning_ask_take_over_enable_{false};
  bool front_have_static_obs_{false};
  Vec3d pre_utm_;
  double abnormal_stop_start_{-1.0};
  double obstacle_block_start_{-1.0};
  double abnormal_stop_move_count_{0.0};
  double obstacle_block_move_count_{0.0};
  double stop_start_t_{-1.0};
  double closest_front_obs_s_{999.0};
  bool have_reached_station_{false};
  static constexpr double kFrontStaticObstDistThreshold = 10.0;
  static constexpr double kStartCheckReachStationDist = 10.0;
};

REGISTER_SCENARIO_TASK(HumanInterfaceDecider);

}  // namespace planning
}  // namespace neodrive
