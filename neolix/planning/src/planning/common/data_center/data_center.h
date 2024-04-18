/**
 * @file data_center.h
 * @author PnC (pnc@neolix.cn)
 * @brief data center for planning
 * @version 0.1
 * @date 2022-02-27
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <deque>
#include <fstream>
#include <list>
#include <memory>
#include <unordered_map>

#include "common/macros.h"
#include "common/message_util.h"
#include "common/planning_types.h"
#include "common_config/config/common_config.h"
#include "cyber/common/memory_pool.h"
#include "cyber/proto/recorder_cmd.pb.h"
#include "environment.h"
#include "frame.h"
#include "master_info.h"
#include "node/node.h"
#include "pilot_state_msgs.pb.h"
#include "planning.pb.h"
#include "planning/task/task_info.h"
#include "src/planning/common/data_center/speed_limit.h"
#include "src/planning/common/obstacle/object_table.h"
#include "src/planning/common/parking_space/parking_space_base.h"
#include "src/planning/common/planning_code_define.h"
#include "src/planning/common/planning_types.h"
#include "src/planning/navigation/common/navigation_swap_context.h"
#include "src/planning/proxy/aeb_switch_proxy.h"
#include "src/planning/proxy/event_report_proxy.h"
#include "src/planning/proxy/global_state_proxy.h"
#include "src/planning/proxy/traffic_light_proxy.h"
#include "src/planning/proxy/vehicle_state_proxy.h"
#include "src/planning/public/planning_lib_header.h"
#include "src/planning/reference_line/reference_line.h"
#include "src/planning/util/history_data.h"

namespace neodrive {
namespace planning {
enum MonitorItemSource : int {
  // car status report.
  DISCODE = 0,
  PILOT_STATE,
  // ---
  EGO_CAR,
  DRIVE_MODE,
  LOCALIZATION,
  SPEED_LIMIT,
  NAVIGATION,
  SCENARIO_STATE,
  MOTION_PLANNING,
  CRUISE_STATE,
  MOTORWAY_CRUISE_STATE,
  MOTORWAY_CHANGE_LANE,
  DETOUR,
  MOTOTWAY_DETOUR,
  PULL_OVER,
  BIAS_DRIVING,
  REFERENCE_LINE,
  OBSTACLE_ID,
  SPEED_PLANNING,
  PATH_PLANNING,
  FAIL_TASK,
  REQUEST_TAKEOVER,
  STATISTICS,
  CONFLICT_ZONE,
  OBSTACLE_INTENTION,
  REMOTE_INTERACTION,
  CURVES_PASSING,
  JUNCTION,
  BACK_SPEED_PLANNING,
  ITER_DEDUCTION_DECISION,
  PATH_BACKUP,
  MAX_ITERM_SIZE
};
class DataCenter {
  DECLARE_SINGLETON(DataCenter);

 public:
  ~DataCenter() = default;
  bool Init(const std::shared_ptr<cyber::Node> node);

  /**
   *@brief initialize frame data, including environment config, start time,
   *object_table
   *@param sequence_num-frame sequence number
   *@return PLANNING_OK if initialize succeed, otherwise PLANNING_ERROR_FAILED
   */
  ErrorCode InitFrame(const uint32_t sequence_num);
  /**
   *@brief create shadow frame from frame_'s sequence_num, start_time, and
   *environment data
   *@return unique_ptr of new frame
   */
  std::unique_ptr<Frame> CreateShadowFrame() const;
  /**
   *@brief save task_info's frame and master infoto data center
   *@param task_info-reference to task info
   */
  void SaveTask(TaskInfo &task_info);
  /**
   *@brief save frame's obstacle decision and history decision data
   */
  void SaveFrame();
  /**
   *@brief clear frame_ and sequence_queue_ data
   */
  void ClearHistoryFrame();
  void SetMonitorString(const std::string &str, MonitorItemSource source);
  void AggregateMonitorString();
  void SaveMonitorMessage();
  bool IsStill(const Obstacle &obstable);
  static PredictionTrajectory GenerateConstVelocityTraj(
      const Obstacle &obstable);

 public:
  /**
   * utils function to add record event.
   */
  void AddRecordEvent(EventOfInterest::EventType event_type,
                      int32_t record_start_offset = -60,
                      int32_t record_end_offset = 60);

 public:
  // get const reference and mutable pointer function
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(double, initialized_time);

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(std::shared_ptr<cyber::Node>, node);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(common::util::MessageUnit<RoutingResult>,
                                   routing_result);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(Environment, environment);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(MonitorString, monitor_message);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(neodrive::global::planning::PilotState,
                                       pilot_state);

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(GlobalStateProxy, global_state_proxy);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(EventReportProxy, event_report_proxy);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(AebSwitchProxy, aeb_switch_proxy);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(std::list<TaskInfo>, task_info_list);
  using AD2 = std::array<double, 2>;
  using AD3 = std::array<double, 3>;
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<AD2>, frenet_upper_bound);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<AD2>, frenet_lower_bound);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<AD2>, cartesian_upper_bound);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<AD2>, cartesian_lower_bound);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(
      std::vector<std::shared_ptr<EventOfInterest>>, record_event_list);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(VehicleStateProxy, vehicle_state_odometry);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(VehicleStateProxy, vehicle_state_utm);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(VehicleStateProxy,
                                       last_vehicle_state_odometry);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(VehicleStateProxy,
                                       last_vehicle_state_utm);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(LocalizationStatus, localization_status);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(MasterInfo, master_info);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, drive_strategy_max_speed);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, enable_prediction);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(double, init_frame_time);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(BehaviorSpeedLimit, behavior_speed_limits);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(std::vector<std::string>, fail_tasks);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, receive_openapi_command_time);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(PerceptionErrorCodeInLidar,
                                      lidar_perception_error_code);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(SeverityLevel,
                                      lidar_non_point_severity_lvel);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, distance_to_zone);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(SpeedLimitShrPtrMap,
                                       reference_line_speed_limits);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(
      SpeedLimitShrPtrMap, junctions_on_reference_line_speed_limits);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(SingleCameraSegmentation, camera_segments);

  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::shared_ptr<ParkingSpace>,
                                       parking_ptr);

  const VehicleStateProxy &vehicle_state_proxy() const;

  Frame *frame(const uint32_t sequence_num) const;
  Frame *current_frame() const;
  const Frame *last_frame() const;

  ObjectTable *mutable_object_table() const;
  ObjectTable *mutable_object_table_utm() const;

  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, odometry_speed_limit);

  const std::unordered_map<int, Obstacle> &caution_speed_utm_obstacles();
  const std::unordered_map<int, Obstacle> &caution_speed_odometry_obstacles();
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_auto_driving);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_on_map);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(int, current_lane_idx);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(int, target_lane_idx);
  NavigationSwapContext &GetNavigationSwapContext() {
    return navigation_swap_context_;
  }
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(ReferenceLinePtr, target_odom_ref);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(NavigationResult, navigation_result);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(int, aeb_active_cnt);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(int, fcw_active_cnt);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(int, aeb_take_over_cnt);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, motorway_back_out);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(
      std::vector<cyberverse::LaneInfoConstPtr>, route_lanes);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(AD3, routing_destination);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, have_task);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(
      neodrive::global::planning::ScenarioState::State, prev_state);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(
      neodrive::global::planning::ScenarioState::State, lastframe_state);

  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(HornLightsCmd::TurnLevel, turn_light);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_renavigation_fail);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(uint64_t, lane_change_target_id);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, planning_clear_task);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_parking);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_waiting_traffic_light);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_odom_sensor_delay);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_in_port_odd);

  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_sim);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, use_center_pose);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, no_abnormal_stop_check);

 private:
  void UpdateProxy();
  void UpdateObstacleDecision(Obstacle &obstacle);

  void ComputeObstacleTables();
  void UpdateCameraSegments();

 public:
  common::util::MessageUnit<AebCmd> aeb_cmd_msg;
  common::util::MessageUnit<Chassis> chassis_msg;
  common::util::MessageUnit<IsOnMap> is_on_map_msg;
  common::util::MessageUnit<RoutingResult> routing_result_msg;
  common::util::MessageUnit<ControlCommand> control_command_msg;
  common::util::MessageUnit<PadNormalCommand> pad_normal_command_msg;
  common::util::MessageUnit<Freespace> lidar_freespace_msg;
  common::util::MessageUnit<CameraSegmentation> camera_freespace_msg;
  common::util::MessageUnit<PatrolStatus> patrol_status_msg;
  common::util::MessageUnit<PlanningInterface> planning_interface_msg;
  common::util::MessageUnit<LocalizationStatus> localization_status_msg;
  common::util::MessageUnit<TwistStamped> twist_base_link_msg;
  common::util::MessageUnit<TwistStamped> kinematics_estimation_base_link_msg;
  common::util::MessageUnit<PoseStamped> pose_base_link_in_utm_msg;
  common::util::MessageUnit<PoseStamped> pose_base_link_in_odometry_msg;
  common::util::MessageUnit<PoseStamped> pose_center_in_utm_msg;
  common::util::MessageUnit<PoseStamped> pose_center_in_odometry_msg;
  common::util::MessageUnit<PerceptionObstacles> perception_obstacles_msg;
  common::util::MessageUnit<PredictionObstacles> prediction_obstacles_msg;
  common::util::MessageUnit<PerceptionObstacles>
      lslidar_perception_obstacles_msg;
  common::util::MessageUnit<TrafficLightDetection> traffic_light_detection_msg;
  common::util::MessageUnit<PerceptionLanes> perception_lanes_msg;
  common::util::MessageUnit<CyberverseTrigger> cyberverse_trigger_msg;

 private:
  static constexpr double kMinEventOfInterestMsgIntervel = 1.;
  bool initialized_{false};
  double initialized_time_{0.};
  std::shared_ptr<cyber::Node> node_{nullptr};
  common::util::MessageUnit<RoutingResult> routing_result_;
  GlobalStateProxy global_state_proxy_{};
  EventReportProxy event_report_proxy_{};
  AebSwitchProxy aeb_switch_proxy_{};
  Environment environment_{};
  std::vector<std::string> monitor_message_vec_{};
  MonitorString monitor_message_{};
  neodrive::global::planning::PilotState pilot_state_{};
  PerceptionErrorCodeInLidar lidar_perception_error_code_{};
  SeverityLevel lidar_non_point_severity_lvel_{};

  std::unique_ptr<Frame> frame_{nullptr};
  std::unordered_map<uint32_t, std::unique_ptr<Frame>> frames_{};
  uint32_t current_frame_seq_{0};
  std::deque<uint32_t> sequence_queue_{};
  double init_frame_time_{0.};

  VehicleStateProxy vehicle_state_odometry_{};  // odometry
  VehicleStateProxy vehicle_state_utm_{};       // utm
  VehicleStateProxy last_vehicle_state_odometry_{};
  VehicleStateProxy last_vehicle_state_utm_{};

  LocalizationStatus localization_status_{};  // utm status

  std::unique_ptr<ObjectTable> object_table_odometry_{
      std::make_unique<ObjectTable>()};  // odometry
  std::unique_ptr<ObjectTable> object_table_utm_{
      std::make_unique<ObjectTable>()};  // utm

  std::list<TaskInfo> task_info_list_{};

  std::vector<std::array<double, 2>> frenet_upper_bound_{};
  std::vector<std::array<double, 2>> frenet_lower_bound_{};
  std::vector<std::array<double, 2>> cartesian_upper_bound_{};
  std::vector<std::array<double, 2>> cartesian_lower_bound_{};

  double drive_strategy_max_speed_{0.};
  bool enable_prediction_{false};

  MasterInfo master_info_{};
  neodrive::cyber::common::MemoryPool<EventOfInterest> event_msg_pool_;
  std::vector<std::shared_ptr<EventOfInterest>> record_event_list_{};
  std::vector<double> event_of_interest_prev_time_;
  BehaviorSpeedLimit behavior_speed_limits_;
  std::vector<std::string> fail_tasks_;
  double receive_openapi_command_time_{0.};
  uint32_t routing_result_msg_sequence_num_{0};
  double distance_to_zone_{0.0};

  double odometry_speed_limit_{0.};

  std::unordered_map<int, Obstacle> caution_speed_utm_obstacles_;
  std::unordered_map<int, Obstacle> caution_speed_odometry_obstacles_;

  std::vector<double> obs_velocity_presum_{};
  int aeb_active_cnt_{0};
  int fcw_active_cnt_{0};
  int aeb_take_over_cnt_{0};
  SpeedLimitShrPtrMap reference_line_speed_limits_;
  SpeedLimitShrPtrMap junctions_on_reference_line_speed_limits_;
  SingleCameraSegmentation camera_segments_;

  std::atomic<bool> is_auto_driving_{false};
  bool is_on_map_{false};
  int current_lane_idx_{0};
  int target_lane_idx_{0};
  NavigationSwapContext navigation_swap_context_;
  ReferenceLinePtr target_odom_ref_{nullptr};
  NavigationResult navigation_result_;
  std::vector<cyberverse::LaneInfoConstPtr> route_lanes_{};
  bool have_task_{false};
  AD3 routing_destination_;
  std::string navigation_type_str_{""};
  //   int turn_light_{0};
  HornLightsCmd::TurnLevel turn_light_{HornLightsCmd::NO_TURN};
  bool is_renavigation_fail_{false};
  uint64_t lane_change_target_id_{0};

  bool motorway_back_out_{false};
  neodrive::global::planning::ScenarioState::State prev_state_;
  neodrive::global::planning::ScenarioState::State lastframe_state_;

  std::shared_ptr<ParkingSpace> parking_ptr_;
  bool is_parking_{false};
  bool is_waiting_traffic_light_{false};
  bool is_in_port_odd_{false};
  std::atomic<bool> is_odom_sensor_delay_{false};
  std::atomic<bool> planning_clear_task_{false};

  bool is_sim_{false};
  bool use_center_pose_{false};
  bool no_abnormal_stop_check_{false};
};
}  // namespace planning
}  // namespace neodrive
