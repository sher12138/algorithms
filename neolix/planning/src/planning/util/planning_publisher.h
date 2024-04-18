#pragma once

#include "common/macros.h"
#include "cyber.h"
#include "cyber/common/memory_pool.h"
#include "decision.pb.h"
#include "path_plot_msg.pb.h"
#include "pilot_state_msgs.pb.h"
#include "planning.pb.h"
#include "speed_plot_msg.pb.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/visualizer_event/visualizer_event.h"
#include "src/planning/navigation/common/navigation_types.h"
#include "src/planning/planning_map/planning_map.h"
#include "src/planning/util/trajectory_stitcher.h"
namespace neodrive {
namespace planning {

class PlanningPublisher {
  DECLARE_SINGLETON(PlanningPublisher);

 public:
  bool Init(const std::shared_ptr<neodrive::cyber::Node> &node);
  void Reset();

  // publish channel info
  bool PublishTrafficLightData(const TaskInfo &task_info);
  bool PublishPlanningResult(
      const std::vector<TrajectoryPoint> &stitch_trajectory, Frame *const frame,
      const bool has_closer_obs);
  bool PublishResetTrajectory();
  void PublishFakeTrajectory(const uint32_t sequence_num);
  bool PublishClearTrajectory();
  void PublishMessages();
  bool PublishEstopTrajectory();

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(std::string, coordinate_type_str);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(bool, planning_trajectory_published);

 private:
  std::string BackSpeedMonitorInfo(
      const OutsidePlannerData &outside_planner_data);

  std::string DecisionSpeedMonitorInfo(
      const OutsidePlannerData &outside_planner_data);

  void AddTrafficLight(const uint64_t id, const double s,
                       const std::vector<ReferencePoint> &ref_pts,
                       TrafficLightDetection *const traffic_light);

  std::string TrajectorySpeedMonitorInfo(const ADCTrajectory &trajectory) const;
  bool SendPlanningSpeedTestDataOut(Frame *const frame);
  void AddReinforcementLearningEvent();
  void SetGearState(std::shared_ptr<ADCTrajectory> &output);

 private:
  bool initialized_{false};
  std::shared_ptr<Node> node_{nullptr};
  DataCenter *data_center_{DataCenter::Instance()};
  PlanningMap *planning_map_{PlanningMap::Instance()};
  TrajectoryStitcher *trajectory_stitcher_{TrajectoryStitcher::Instance()};

  std::shared_ptr<neodrive::cyber::Writer<ADCTrajectory>> planning_pub_;
  std::shared_ptr<
      neodrive::cyber::Writer<neodrive::global::planning::DecisionResult>>
      decision_pub_;
  std::shared_ptr<cyber::Writer<neodrive::global::status::GlobalState>>
      global_state_pub_;
  std::shared_ptr<cyber::Writer<neodrive::global::status::EventReport>>
      event_report_pub_;
  std::shared_ptr<cyber::Writer<neodrive::global::planning::ForbidAeb>>
      aeb_switch_pub_;
  std::shared_ptr<
      neodrive::cyber::Writer<neodrive::planning::test::SpeedPlotMsg>>
      pub_speed_test_;
  std::shared_ptr<Writer<MonitorString>> monitor_log_pub_;
  std::shared_ptr<Writer<EventOfInterest>> record_event_pub_;

  std::shared_ptr<neodrive::cyber::Writer<
      neodrive::global::perception::traffic_light::TrafficLightDetection>>
      traffic_light_pub_;

  vis::EventSender *event_sender_;

  neodrive::cyber::common::MemoryPool<TrafficLightDetection>
      traffic_light_msg_pool_;
  neodrive::cyber::common::MemoryPool<ADCTrajectory> plan_traj_msg_pool_;
  neodrive::cyber::common::MemoryPool<ADCTrajectory> ref_line_msg_pool_;
  neodrive::cyber::common::MemoryPool<ADCTrajectory> fake_trajectory_msg_pool_;
  neodrive::cyber::common::MemoryPool<ADCTrajectory> reset_trajectory_msg_pool_;
  neodrive::cyber::common::MemoryPool<
      neodrive::global::planning::DecisionResult>
      decision_result_msg_pool_;
  neodrive::cyber::common::MemoryPool<neodrive::planning::test::SpeedPlotMsg>
      speed_test_msg_pool_;
  neodrive::cyber::common::MemoryPool<MonitorString> monitor_msg_pool_;
  neodrive::cyber::common::MemoryPool<GlobalState> global_state_msg_pool_;
  neodrive::cyber::common::MemoryPool<EventReport> event_report_msg_pool_;
  neodrive::cyber::common::MemoryPool<ForbidAeb> aeb_switch_msg_pool_;

  bool planning_trajectory_published_{false};
  std::string coordinate_type_str_{""};

  double loop_start_time_{0.0}; //unit:second, debug only, for time delay statistics
};  // namespace planning

}  // namespace planning
}  // namespace neodrive
