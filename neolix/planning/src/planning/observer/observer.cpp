#include "observer.h"

#include <time.h>

#include "common/data_center/data_center.h"
#include "common/math/mean_filter.h"
#include "common/vehicle_param.h"
#include "common_config/config/common_config.h"
#include "config/planning_config.h"
#include "neolix_common/util/performance_test.h"
#include "planning_map/planning_map.h"
#include "reader_manager/reader_manager.h"
#include "src/planning/planning_map/planning_map.h"
#include "src/planning/scenario_manager/scenario_manager.h"

namespace neodrive {
namespace planning {

PERFORMANCE_TEST_DEFINE(planning_regulator);

Observer::Observer() {}

bool Observer::Init(const std::shared_ptr<Node> &node) {
  if (initialized_) {
    return true;
  }
  LOG_INFO("Init");
  neodrive::common::config::CommonConfig::Instance();
  auto &topics_config = neodrive::common::config::CommonConfig::Instance()
                            ->topics_config()
                            .topics;
  if (node == nullptr) {
    LOG_ERROR("node is nullptr!!!");
    return false;
  }
  node_ = node;
  data_center_ = DataCenter::Instance();
  auto reader_manager = common::ReaderManager::Instance();
  reader_manager->Init(node_);
  neodrive::cyber::ReaderConfig aeb_reader_config;
  aeb_reader_config.channel_name = topics_config.at("aeb_cmd").topic;
  aeb_reader_config.pending_queue_size = 10;
  data_center_->aeb_cmd_msg.reader =
      reader_manager->CreateReader<AebCmd>(aeb_reader_config);
  data_center_->is_on_map_msg.reader = reader_manager->CreateReader<IsOnMap>(
      topics_config.at("is_on_map").topic);
  data_center_->chassis_msg.reader = reader_manager->CreateReader<Chassis>(
      topics_config.at("planning_chassis").topic);
  data_center_->routing_result_msg.reader =
      reader_manager->CreateReader<RoutingResult>(
          topics_config.at("routing_result").topic);
  data_center_->control_command_msg.reader =
      reader_manager->CreateReader<ControlCommand>(
          topics_config.at("pnc_control").topic);
  data_center_->pad_normal_command_msg.reader =
      reader_manager->CreateReader<PadNormalCommand>(
          topics_config.at("hmi_stop_start").topic);
  data_center_->lidar_freespace_msg.reader =
      reader_manager->CreateReader<Freespace>(
          topics_config.at("lidar_perception_freespace").topic);
  data_center_->camera_freespace_msg.reader =
      reader_manager->CreateReader<CameraSegmentation>(
          topics_config.at("camera_perception_freespace").topic);
  data_center_->patrol_status_msg.reader =
      reader_manager->CreateReader<PatrolStatus>(
          topics_config.at("patrol_status").topic);
  data_center_->planning_interface_msg.reader =
      reader_manager->CreateReader<PlanningInterface>(
          topics_config.at("openapi_planning_interface").topic);
  data_center_->localization_status_msg.reader =
      reader_manager->CreateReader<LocalizationStatus>(
          topics_config.at("localization_status").topic);
  data_center_->twist_base_link_msg.reader =
      reader_manager->CreateReader<TwistStamped>(
          topics_config.at("twist_base_link").topic);
  data_center_->kinematics_estimation_base_link_msg.reader =
      reader_manager->CreateReader<TwistStamped>(
          topics_config.at("kinematics_estimation_base_link").topic);
  data_center_->pose_base_link_in_utm_msg.reader =
      reader_manager->CreateReader<PoseStamped>(neodrive::cyber::ReaderConfig(
          topics_config.at("pose_base_link_in_utm").topic, 10));
  data_center_->pose_base_link_in_odometry_msg.reader =
      reader_manager->CreateReader<PoseStamped>(
          topics_config.at("pose_base_link_in_odometry").topic);
  data_center_->pose_center_in_utm_msg.reader =
      reader_manager->CreateReader<PoseStamped>(
          topics_config.at("pose_center_in_utm").topic);
  data_center_->pose_center_in_odometry_msg.reader =
      reader_manager->CreateReader<PoseStamped>(
          topics_config.at("pose_center_in_odometry").topic);
  data_center_->perception_obstacles_msg.reader =
      reader_manager->CreateReader<PerceptionObstacles>(
          topics_config.at("perception_obstacles").topic);
  data_center_->prediction_obstacles_msg.reader =
      reader_manager->CreateReader<PredictionObstacles>(
          topics_config.at("pnc_prediction").topic);
  data_center_->cyberverse_trigger_msg.reader =
      reader_manager->CreateReader<CyberverseTrigger>(
          topics_config.at("cyberverse_trigger").topic);
  if (FLAGS_planning_enable_lms) {
    data_center_->lslidar_perception_obstacles_msg.reader =
        reader_manager->CreateReader<PerceptionObstacles>(
            topics_config.at("lslidar_perception_obstacles").topic);
  }
  if (FLAGS_planning_enable_traffic_light_law) {
    data_center_->traffic_light_detection_msg.reader =
        reader_manager->CreateReader<TrafficLightDetection>(
            topics_config.at("perception_traffic_light").topic);
  }
  if (FLAGS_planning_enable_perception_lanes) {
    data_center_->perception_lanes_msg.reader =
        reader_manager->CreateReader<PerceptionLanes>(
            topics_config.at("perception_lanes").topic);
  }

  initialized_ = true;
  return true;
}

void Observer::Observe() {
  LOG_INFO("Observe start");
  ObserveAebMsg();
  PLANNING_OBSERVE_FUNCTION(data_center_->chassis_msg);
  PLANNING_OBSERVE_FUNCTION(data_center_->routing_result_msg);
  PLANNING_OBSERVE_FUNCTION(data_center_->control_command_msg);
  PLANNING_OBSERVE_FUNCTION(data_center_->pad_normal_command_msg);
  PLANNING_OBSERVE_FUNCTION(data_center_->lidar_freespace_msg);
  PLANNING_OBSERVE_FUNCTION(data_center_->camera_freespace_msg);
  PLANNING_OBSERVE_FUNCTION(data_center_->patrol_status_msg);
  PLANNING_OBSERVE_FUNCTION(data_center_->planning_interface_msg);
  PLANNING_OBSERVE_FUNCTION(data_center_->localization_status_msg);
  PLANNING_OBSERVE_FUNCTION(data_center_->twist_base_link_msg);
  PLANNING_OBSERVE_FUNCTION(data_center_->kinematics_estimation_base_link_msg);
  PLANNING_OBSERVE_FUNCTION(data_center_->pose_base_link_in_utm_msg);
  PLANNING_OBSERVE_FUNCTION(data_center_->pose_base_link_in_odometry_msg);
  PLANNING_OBSERVE_FUNCTION(data_center_->pose_center_in_utm_msg);
  PLANNING_OBSERVE_FUNCTION(data_center_->pose_center_in_odometry_msg);
  PLANNING_OBSERVE_FUNCTION(data_center_->prediction_obstacles_msg);
  OBSERVE_FUNCTION(data_center_->cyberverse_trigger_msg);
  OBSERVE_FUNCTION(data_center_->is_on_map_msg);
  if (FLAGS_planning_enable_lms) {
    PLANNING_OBSERVE_FUNCTION(data_center_->lslidar_perception_obstacles_msg);
  }
  if (FLAGS_planning_enable_traffic_light_law) {
    PLANNING_OBSERVE_FUNCTION(data_center_->traffic_light_detection_msg);
  }
  if (FLAGS_planning_enable_perception_lanes) {
    PLANNING_OBSERVE_FUNCTION(data_center_->perception_lanes_msg);
  }
  LOG_INFO("Observe end");
}

void Observer::ObservePerceptionObstacleMsg() {
  PLANNING_OBSERVE_FUNCTION(data_center_->perception_obstacles_msg);
}

void Observer::ObserveAebMsg() {
  LOG_INFO("ObserveAebMsg start");
  data_center_->aeb_cmd_msg.is_updated = false;
  data_center_->aeb_cmd_msg.reader->Observe();
  auto tmp_aeb_cmd_msgs =
      data_center_->aeb_cmd_msg.reader->GetNewObservedMessages(
          prev_aeb_cmd_msg_);
  for (auto item = tmp_aeb_cmd_msgs.rbegin(); item != tmp_aeb_cmd_msgs.rend();
       ++item) {
    prev_prev_aeb_cmd_msg_ = prev_aeb_cmd_msg_;
    prev_aeb_cmd_msg_ = *item;
    if (prev_aeb_cmd_msg_ != nullptr && prev_prev_aeb_cmd_msg_ != nullptr) {
      if (prev_prev_aeb_cmd_msg_->aeb_state().state() != AebState::ACTIVE &&
          prev_aeb_cmd_msg_->aeb_state().state() == AebState::ACTIVE) {
        data_center_->set_aeb_active_cnt(data_center_->aeb_active_cnt() + 1);
      }
      if (prev_prev_aeb_cmd_msg_->aeb_state().is_fcw_active() == false &&
          prev_aeb_cmd_msg_->aeb_state().is_fcw_active() == true) {
        data_center_->set_fcw_active_cnt(data_center_->fcw_active_cnt() + 1);
      }
      if (prev_aeb_cmd_msg_->aeb_state().state() == AebState::ACTIVE &&
          prev_is_auto_driving_ && !data_center_->is_auto_driving()) {
        data_center_->set_aeb_take_over_cnt(data_center_->aeb_take_over_cnt() +
                                            1);
        prev_is_auto_driving_ = data_center_->is_auto_driving();
      }
    }
    auto &aeb_cmd = *(data_center_->aeb_cmd_msg.ptr);
    if (prev_aeb_cmd_msg_ != nullptr &&
        aeb_cmd.aeb_state().state() == AebState::ACTIVE &&
        prev_aeb_cmd_msg_->aeb_state().state() != AebState::ACTIVE &&
        prev_aeb_cmd_msg_->header().timestamp_sec() -
                aeb_cmd.header().timestamp_sec() <
            kAebActiveDelayShowTime) {
      // ignore current msg to delay show active aeb cmd
      continue;
    }
    data_center_->aeb_cmd_msg.is_updated = true;
    data_center_->aeb_cmd_msg.is_available = true;
    data_center_->aeb_cmd_msg.ptr = prev_aeb_cmd_msg_;
  }
  prev_is_auto_driving_ = data_center_->is_auto_driving();
  LOG_INFO("ObserveAebMsg end");
}

}  // namespace planning
}  // namespace neodrive
