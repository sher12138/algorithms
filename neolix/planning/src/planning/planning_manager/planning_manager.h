#pragma once

#include <time.h>

#include <memory>
#include <unordered_map>
#include <vector>

#include "aeb_msgs.pb.h"
#include "common_config/config/common_config.h"
#include "cyber.h"
#include "cyber/common/memory_pool.h"
#include "decision.pb.h"
#include "path_plot_msg.pb.h"
#include "pilot_state_msgs.pb.h"
#include "planning.pb.h"
#include "speed_plot_msg.pb.h"
#include "src/planning/common/obstacle/decision_data.h"
#include "src/planning/common/obstacle/object_table.h"
#include "src/planning/common/planning_code_define.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/common/planning_macros.h"
#include "src/planning/deciders/human_interface_decider/human_interface_decider.h"
#include "src/planning/deciders/obstacles_intention_decider/obstacles_intention_decider.h"
#include "src/planning/deciders/pilot_state_decider/pilot_state_decider.h"
#include "src/planning/deciders/station_stop_decider/station_stop_decider.h"
#include "src/planning/deciders/vehicle_state_control_decider/vehicle_state_control_decider.h"
#include "src/planning/math/common/obstacle_map.h"
#include "src/planning/observer/observer.h"
#include "src/planning/proxy/vehicle_state_proxy.h"
#include "src/planning/public/planning_lib_header.h"
#include "src/planning/scenario_manager/scenario_manager.h"
#include "src/planning/util/history_data.h"
#include "src/planning/util/planning_publisher.h"
#include "src/planning/util/trajectory_stitcher.h"
#include "traffic_light_detection.pb.h"

using neodrive::global::perception::camera::CameraLaneLine;
using neodrive::global::perception::camera::LaneLinePositionType;
using neodrive::global::perception::camera::LaneLineType;

namespace neodrive {
namespace planning {

class PlanningManager {
  DECLARE_SINGLETON(PlanningManager);

 public:
  bool Init(const std::shared_ptr<Node> &node);
  ErrorCode RunOnce();

 private:
  ErrorCode DataPreprocess();
  ErrorCode BehaviorPlanning();
  ErrorCode MotionPlanning();
  ErrorCode DataPostprocess();

  void SaveMonitorMessage();
  // preview check
  ErrorCode CheckWhetherSkipPlanning();

  // main function
  double DriveStrategyMaxSpeed();
  bool DriveStrategyEnablePrediction();
  ErrorCode TransformToOdometry();
  ErrorCode StitchTrajectory();
  ErrorCode InitTaskInfoList();
  ErrorCode ExecuteMotionTasks(TaskInfo &task_info);

  void UpdateRemoteReactionContext(const ReferenceLinePtr ref_ptr);

  bool UpdateTaskInfoUtilsInfo(TaskInfo &task_info);
  void ObsToLocalCoordinate(TaskInfo &task_info,
                            DecisionData *const decision_data);
  bool IsInvalidReferenceLine(const ReferenceLinePtr reference_line,
                              const TrajectoryPoint &init_point) const;

  ErrorCode PostProcessPlanningResult(std::list<TaskInfo> &task_info_list);

  void BehaviorProcess(TaskInfo *const task_info);
  void FinishProcess();
  void ObsInfoLogger(const DecisionData &decision_data);

  // compute motion monitor info
  std::string PathMonitorInfo(const InsidePlannerData &inside_data,
                              const OutsidePlannerData &outside_data) const;
  std::string ObstacleMonitorInfo(const OutsidePlannerData &outside_data) const;
  std::string SpeedMonitorInfo(const InsidePlannerData &inside_data,
                               const OutsidePlannerData &outside_data) const;

  void InitAdcLocalPolygonCheckCloserObs();
  void JudgeIfObsCloseToAdc(const Obstacle *const obs,
                            const std::vector<Vec2d> &obs_corners_local,
                            bool &obs_close_to_adc);
  void ExecuteDataPreprocessFail();

 private:
  bool initialized_{false};
  uint32_t sequence_num_{0};
  std::shared_ptr<Node> node_{nullptr};
  obsmap::ObstacleMap *obstacle_map_{nullptr};

  HumanInterfaceDecider *new_human_interface_decider_{nullptr};
  StationStopDecider *station_stop_decider_{nullptr};
  TrajectoryStitcher *trajectory_stitcher_{TrajectoryStitcher::Instance()};
  ObstaclesIntentionDecider *obstacles_intention_decider_{nullptr};
  common::util::TimeLogger logger_{"planning_manager"};
  DataCenter *data_center_{DataCenter::Instance()};
  PilotStateDecider *pilot_state_decider_ptr_;
  Observer *observer_{Observer::Instance()};

  HistoryData rerouting_trigger_;
  bool vehicle_state_update_{false};
  std::vector<TrajectoryPoint> stitching_trajectory_;

  size_t ref_line_points_size_{0};

  std::vector<std::pair<std::string, ErrorCode (PlanningManager::*)()>>
      planning_steps_;

  Polygon2d adc_local_polygon_check_closer_obs_{};
  bool has_closer_obs_{false};
  bool skip_motion_plan_{false};
  bool pre_auto_driving{false};

  ReferenceLinePtr current_odom_ref_{nullptr};
  ReferenceLinePtr current_utm_ref_{nullptr};

  PlanningPublisher *publisher_{PlanningPublisher::Instance()};
};

}  // namespace planning
}  // namespace neodrive
