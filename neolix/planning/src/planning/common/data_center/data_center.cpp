#include "data_center.h"

#include <iostream>

#include "common/util/hash_util.h"
#include "common/util/time_logger.h"
#include "cyber/time/time.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/navigation/config/navigation_config.h"
#include "src/planning/navigation/navigation_base.h"
#include "src/planning/planning_map/planning_map.h"
#include "src/planning/task/task_info.h"

namespace neodrive {
namespace planning {
using neodrive::global::control::ControlAction_Name;
using neodrive::global::localization::ErrorType_Name;
using neodrive::global::localization::ImuMsgDelayStatus_Name;
using neodrive::global::localization::ImuMsgMissingStatus_Name;
using neodrive::global::localization::LocalLidarQuality_Name;
using neodrive::global::localization::LocalLidarStatus_Name;
using neodrive::global::localization::MsfSecurityLevel_Name;
using neodrive::global::localization::SensorErrorCode_Name;
using neodrive::global::perception::PerceptionErrorCode;
using neodrive::global::perception::PerceptionErrorCodeInLidar_Name;
using neodrive::global::status::GearPosition_Name;
using neodrive::global::status::StopReason;

DataCenter::DataCenter() {
  monitor_message_vec_.resize(MonitorItemSource::MAX_ITERM_SIZE, "");
  event_of_interest_prev_time_.resize(EventOfInterest::EventType_MAX + 1, 0);
}

bool DataCenter::Init(const std::shared_ptr<cyber::Node> node) {
  if (initialized_) {
    return true;
  }
  node_ = node;
  // monitor_message_.mutable_header()->set_module_name("planning");
  initialized_time_ = cyber::Time::Now().ToSecond();
  auto &navigation_config =
      config::NavigationConfig::Instance()->navigation_config();
  NavigationStrategy type =
      static_cast<NavigationStrategy>(navigation_config.navigation_strategy);
  if (type == NavigationStrategy::BREADTH_FIRST) {
    navigation_type_str_ = "BREADTH_FIRST";
  } else if (type == NavigationStrategy::ASTAR) {
    navigation_type_str_ = "ASTAR";
  } else {
    return false;
  }
  global_state_proxy_.ResetInitState();
  LOG_DEBUG("Data Center init succeed");
  initialized_ = true;
  return true;
}

ErrorCode DataCenter::InitFrame(const uint32_t sequence_num) {
  auto start_time = cyber::Time::Now();
  current_frame_seq_ = sequence_num;
  init_frame_time_ = start_time.ToSecond();
  monitor_message_.mutable_header()->set_timestamp_sec(init_frame_time_);
  global_state_proxy_.mutable_global_state()
      ->mutable_header()
      ->set_sequence_num(sequence_num);
  global_state_proxy_.mutable_global_state()
      ->mutable_header()
      ->set_timestamp_sec(init_frame_time_);
  global_state_proxy_.mutable_global_state()
      ->mutable_remote_interaction_context()
      ->set_is_pnc_ok(true);
  global_state_proxy_.mutable_global_state()
      ->mutable_remote_interaction_context()
      ->set_valid_pose(true);
  global_state_proxy_.mutable_global_state()
      ->mutable_remote_interaction_context()
      ->set_valid_theta(true);
  event_report_proxy_.ClearEventInfos();
  // clear monitor msg.
  std::fill(monitor_message_vec_.begin(), monitor_message_vec_.end(), "");
  // clear record event.
  record_event_list_.clear();
  // reset master info context per frame.
  master_info_.set_lane_borrow_type(0);
  master_info_.FrameReset();
  behavior_speed_limits_.Reset();
  fail_tasks_.clear();
  master_info_.set_behavior_stop_vehicle(false);
  UpdateProxy();
  UpdateCameraSegments();
  if (master_info_.restricted_area()) {
    master_info_.set_restricted_area(false);
    LOG_INFO("reset restricted_area to false.");
  }
  frame_ = std::make_unique<Frame>(sequence_num);
  if (frame_ == nullptr) {
    LOG_ERROR("Failed to init frame unique_ptr.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  is_auto_driving_ =
      (vehicle_state_odometry_.DrivingMode() ==
       neodrive::global::status::DrivingMode::COMPLETE_AUTO_DRIVE);
  vehicle_state_odometry_ = environment_.vehicle_state_odometry_proxy();
  vehicle_state_utm_ = environment_.vehicle_state_proxy();

  ComputeObstacleTables();

  return ErrorCode::PLANNING_OK;
}

void DataCenter::UpdateProxy() {
  routing_result_.is_updated = false;
  if (routing_result_msg.is_updated &&
      routing_result_msg_sequence_num_ !=
          routing_result_msg.ptr->header().sequence_num()) {
    LOG_WARN("new routing_result_msg: {} {} {}", routing_result_msg.is_updated,
             routing_result_msg.is_available,
             RoutingResult::ErrorCode::ErrorID_Name(
                 routing_result_msg.ptr->error_code().error_id()));
    if (routing_result_msg.ptr->error_code().error_id() ==
        RoutingResult::ErrorCode::SUCCESS) {
      routing_result_.is_updated = true;
      routing_result_.is_available = true;
      routing_result_.ptr = routing_result_msg.ptr;
      routing_result_msg_sequence_num_ =
          routing_result_.ptr->header().sequence_num();
      global_state_proxy_.ResetInitState();
      planning_interface_msg.is_available = false;
      pad_normal_command_msg.is_available = false;
    }
  }
  if (planning_interface_msg.is_updated) {
    LOG_WARN("new planning_interface_msg: {} {} {}",
             planning_interface_msg.is_updated,
             planning_interface_msg.is_available,
             planning_interface_msg.ptr->DebugString());
    if (global_state_proxy_.is_init() || global_state_proxy_.is_finish()) {
      LOG_WARN("ignore openapi command");
      planning_interface_msg.is_available = false;
    } else {
      receive_openapi_command_time_ = init_frame_time_;
    }
  }
  if (chassis_msg.is_updated) {
    environment_.set_vehicle_state(chassis_msg.ptr);
  }
  // 1. combine twist and pose -> ptr_msg
  if (twist_base_link_msg.is_updated ||
      pose_base_link_in_odometry_msg.is_updated) {
    auto twist_ptr = twist_base_link_msg.ptr;
    auto pose_odom_ptr = pose_base_link_in_odometry_msg.ptr;
    if (std::abs(twist_ptr->header().timestamp_sec() -
                 pose_odom_ptr->header().timestamp_sec()) < 2) {
      environment_.set_vehicle_state_odometry(pose_odom_ptr, twist_ptr);
    } else {
      LOG_ERROR("twist's timestamp could not match pose's timestamp.");
    }

    use_center_pose_ = config::PlanningConfig::Instance()
                           ->plan_config()
                           .common.use_center_pose;
    if (use_center_pose_) {
      auto pose_center_odom_ptr = pose_center_in_odometry_msg.ptr;
      if (pose_center_in_odometry_msg.is_updated &&
          std::abs(twist_ptr->header().timestamp_sec() -
                   pose_center_odom_ptr->header().timestamp_sec()) < 2) {
        environment_.set_vehicle_state_odometry(pose_center_odom_ptr);
      } else {
        use_center_pose_ = false;
        LOG_ERROR("not get center pose odom.");
      }
    }
  }
  // 1. combine pose and twist -> ptr_msg
  if (twist_base_link_msg.is_updated || pose_base_link_in_utm_msg.is_updated) {
    auto twist_ptr = twist_base_link_msg.ptr;
    auto pose_utm_ptr = pose_base_link_in_utm_msg.ptr;
    if (std::abs(twist_ptr->header().timestamp_sec() -
                 pose_utm_ptr->header().timestamp_sec()) < 2) {
      environment_.set_vehicle_state(pose_utm_ptr, twist_ptr);
    } else {
      LOG_ERROR("twist's timestamp could not match pose's timestamp.");
    }
    if (use_center_pose_) {
      auto pose_center_utm_ptr = pose_center_in_utm_msg.ptr;
      if (pose_center_in_utm_msg.is_updated &&
          std::abs(twist_ptr->header().timestamp_sec() -
                   pose_center_utm_ptr->header().timestamp_sec()) < 2) {
        environment_.set_vehicle_state(pose_center_utm_ptr);
      } else {
        use_center_pose_ = false;
        LOG_ERROR("not get center pose odom.");
      }
    }
  }

  if (kinematics_estimation_base_link_msg.is_updated) {
    environment_.set_vehicle_state(kinematics_estimation_base_link_msg.ptr);
    LOG_INFO("Kinematics_msg is updated");
  }

  if (perception_obstacles_msg.is_updated) {
    lidar_perception_error_code_ =
        perception_obstacles_msg.ptr->error_code_lidar();
    lidar_non_point_severity_lvel_ =
        perception_obstacles_msg.ptr->severity_level();
    LOG_INFO("new perception msg seq:{},error_code:{}",
             perception_obstacles_msg.ptr->header().sequence_num(),
             PerceptionErrorCodeInLidar_Name(lidar_perception_error_code_));
    if (perception_obstacles_msg.ptr->error_code() ==
        PerceptionErrorCode::ERROR_NONE) {
      environment_.set_perception(perception_obstacles_msg.ptr);
    }
  }

  if (prediction_obstacles_msg.is_updated &&
      config::PlanningConfig::Instance()
          ->plan_config()
          .prediction.enable_prediction) {
    environment_.set_prediction(prediction_obstacles_msg.ptr);
  }
  if (traffic_light_detection_msg.is_updated) {
    environment_.set_traffic_light(traffic_light_detection_msg.ptr);
  }
  if (perception_lanes_msg.is_updated) {
    environment_.set_perception_lanes(perception_lanes_msg.ptr);
  }
  if (cyberverse_trigger_msg.is_updated) {
    PlanningMap::Instance()->InitSharedMemory();
  }
  if (is_on_map_msg.is_updated) {
    distance_to_zone_ = is_on_map_msg.ptr->distance_to_zone();
  }
}

void DataCenter::ComputeObstacleTables() {
  caution_speed_odometry_obstacles_.clear();
  caution_speed_utm_obstacles_.clear();

  /// ids
  std::vector<int> frame_ids{};
  auto &per_obstacles = environment_.perception_proxy().Perception();
  auto &is_per_obs_valid = environment_.perception_proxy().GetObstaclesValid();
  for (int i = 0; i < is_per_obs_valid.size(); ++i) {
    if (!is_per_obs_valid[i]) {
      continue;
    }

    auto id = per_obstacles.perception_obstacle(i).id();
    // matched lane id
    uint64_t obs_matched_lane_id{0};
    double obs_matched_lane_heading_deviation{0.0};
    double obs_matched_lane_offset{0.0};
    if (!PlanningMap::Instance()->GetNearestLaneWithHeading(
            per_obstacles.perception_obstacle(i).position().x(),
            per_obstacles.perception_obstacle(i).position().y(), 5.0,
            std::atan2(per_obstacles.perception_obstacle(i).velocity().y(),
                       per_obstacles.perception_obstacle(i).velocity().x()),
            M_PI, obs_matched_lane_id, obs_matched_lane_heading_deviation,
            obs_matched_lane_offset)) {
      LOG_DEBUG("obs: [{}] could not match closet lane.", id);
    }
    LOG_DEBUG("obs #{} ---> best matched lane: {}", id,
              PlanningMap::Instance()->GetHashIdString(obs_matched_lane_id));

    // utm
    if (Vec2d(per_obstacles.perception_obstacle(i).velocity().x(),
              per_obstacles.perception_obstacle(i).velocity().y())
                .length() < FLAGS_planning_still_obstacle_speed_threshold &&
        per_obstacles.perception_obstacle(i).sub_type() ==
            PerceptionObstacle_SubType::PerceptionObstacle_SubType_ST_TREE) {
      Obstacle po;
      po.copy_from(per_obstacles.perception_obstacle(i));
      caution_speed_utm_obstacles_.emplace(id, po);
    } else {
      if (Obstacle * po{nullptr};
          object_table_utm_->get_obstacle(id, &po) && po) {
        po->copy_from(per_obstacles.perception_obstacle(i));
        po->set_matched_lane_id(obs_matched_lane_id);
        po->set_matched_lane_heading_deviation(
            obs_matched_lane_heading_deviation);
        po->set_matched_lane_offset(obs_matched_lane_offset);
        po->set_frame_cnt(po->frame_cnt() + 1);
      } else {
        std::unique_ptr<Obstacle> new_obs = std::make_unique<Obstacle>();
        new_obs->copy_from(per_obstacles.perception_obstacle(i));
        new_obs->set_matched_lane_id(obs_matched_lane_id);
        new_obs->set_matched_lane_heading_deviation(
            obs_matched_lane_heading_deviation);
        new_obs->set_matched_lane_offset(obs_matched_lane_offset);
        object_table_utm_->put_obstacle(new_obs);
      }
    }

    // odometry
    if (Vec2d(per_obstacles.odometry_perception_obstacle(i).velocity().x(),
              per_obstacles.odometry_perception_obstacle(i).velocity().y())
                .length() < FLAGS_planning_still_obstacle_speed_threshold &&
        per_obstacles.odometry_perception_obstacle(i).sub_type() ==
            PerceptionObstacle_SubType::PerceptionObstacle_SubType_ST_TREE) {
      Obstacle po;
      po.copy_from(per_obstacles.odometry_perception_obstacle(i));
      caution_speed_odometry_obstacles_.emplace(id, po);
    } else {
      if (Obstacle * po{nullptr};
          object_table_odometry_->get_obstacle(id, &po) && po) {
        po->copy_from(per_obstacles.odometry_perception_obstacle(i));
        po->set_matched_lane_id(obs_matched_lane_id);
        po->set_matched_lane_heading_deviation(
            obs_matched_lane_heading_deviation);
        po->set_matched_lane_offset(obs_matched_lane_offset);
        po->set_frame_cnt(po->frame_cnt() + 1);
      } else {
        std::unique_ptr<Obstacle> new_obs = std::make_unique<Obstacle>();
        new_obs->copy_from(per_obstacles.odometry_perception_obstacle(i));
        new_obs->set_matched_lane_id(obs_matched_lane_id);
        new_obs->set_matched_lane_heading_deviation(
            obs_matched_lane_heading_deviation);
        new_obs->set_matched_lane_offset(obs_matched_lane_offset);
        object_table_odometry_->put_obstacle(new_obs);
      }
    }

    if (!caution_speed_odometry_obstacles_.count(id)) frame_ids.push_back(id);
  }

  /// trajs
  std::unordered_map<int, std::vector<PredictionTrajectory>> id_trajs_utm{};
  std::unordered_map<int, std::vector<PredictionTrajectory>>
      id_trajs_odometry{};
  if (config::PlanningConfig::Instance()
          ->plan_config()
          .prediction.enable_prediction) {
    auto &pred_obstacles = environment_.prediction_proxy().Prediction();
    auto &is_pred_obs_valid =
        environment_.prediction_proxy().GetObstaclesValid();
    for (int i = 0; i < is_pred_obs_valid.size(); ++i) {
      if (!is_pred_obs_valid[i] ||
          !pred_obstacles.prediction_obstacle(i).trajectory_size()) {
        continue;
      }

      // utm
      auto id = pred_obstacles.prediction_obstacle(i).id();
      for (auto &traj : pred_obstacles.prediction_obstacle(i).trajectory()) {
        PredictionTrajectory pred_traj;
        pred_traj.set_start_timestamp(
            pred_obstacles.prediction_obstacle(i).timestamp());
        pred_traj.set_probability(traj.probability());
        pred_traj.set_predictor_type(traj.predictor_type());
        for (auto &pt : traj.trajectory_point()) {
          TrajectoryPoint tp;
          tp.set_x(pt.path_point().x());
          tp.set_y(pt.path_point().y());
          tp.set_theta(pt.path_point().theta());
          tp.set_velocity(pt.v());
          tp.set_relative_time(pt.relative_time());
          tp.set_confidence(1.0);  // fake confident
          pred_traj.mutable_trajectory_points()->push_back(std::move(tp));
        }
        if (!pred_traj.trajectory_points().empty()) {
          id_trajs_utm[id].push_back(std::move(pred_traj));
        }
      }
      // odometry
      for (auto &traj :
           pred_obstacles.odometry_prediction_obstacle(i).trajectory()) {
        PredictionTrajectory pred_traj;
        pred_traj.set_start_timestamp(
            pred_obstacles.odometry_prediction_obstacle(i).timestamp());
        pred_traj.set_probability(traj.probability());
        pred_traj.set_predictor_type(traj.predictor_type());
        for (auto &pt : traj.trajectory_point()) {
          TrajectoryPoint tp;
          tp.set_x(pt.path_point().x());
          tp.set_y(pt.path_point().y());
          tp.set_theta(pt.path_point().theta());
          tp.set_velocity(pt.v());
          tp.set_relative_time(pt.relative_time());
          tp.set_confidence(1.0);  // fake confident
          pred_traj.mutable_trajectory_points()->push_back(std::move(tp));
        }
        if (!pred_traj.trajectory_points().empty()) {
          id_trajs_odometry[id].push_back(std::move(pred_traj));
        }
      }
    }
  }

  /// combine ids and trajs
  for (auto id : frame_ids) {
    // utm
    Obstacle *obs_utm{nullptr};
    if (!object_table_utm_->get_obstacle(id, &obs_utm) || !obs_utm ||
        IsStill(*obs_utm)) {
      continue;
    }
    LOG_DEBUG("obstacle [{}] is not still", id);
    auto pred_trajs_utm = obs_utm->mutable_prediction_trajectories();
    if (id_trajs_utm.count(id)) {
      std::swap(*pred_trajs_utm, id_trajs_utm[id]);
    } else {
      *pred_trajs_utm = std::vector{GenerateConstVelocityTraj(*obs_utm)};
    }
    auto uniform_trajectory_utm = obs_utm->mutable_uniform_trajectory();
    *uniform_trajectory_utm = GenerateConstVelocityTraj(*obs_utm);

    // odometry
    Obstacle *obs{nullptr};
    if (!object_table_odometry_->get_obstacle(id, &obs) || !obs) {
      continue;
    }
    auto pred_trajs_odometry = obs->mutable_prediction_trajectories();
    if (id_trajs_odometry.count(id)) {
      std::swap(*pred_trajs_odometry, id_trajs_odometry[id]);
    } else {
      *pred_trajs_odometry = std::vector{GenerateConstVelocityTraj(*obs)};
    }
    auto uniform_trajectory_odometry = obs->mutable_uniform_trajectory();
    *uniform_trajectory_odometry = GenerateConstVelocityTraj(*obs);
  }
}

bool DataCenter::IsStill(const Obstacle &obstable) {
  auto &states = obstable.get_history_states();
  auto his_size = states.size();

  obs_velocity_presum_.clear();
  for (auto &s : states) {
    obs_velocity_presum_.push_back(s.velocity);
  }
  for (std::size_t i = 1; i < his_size; ++i) {
    obs_velocity_presum_[i] += obs_velocity_presum_[i - 1];
  }

  // Increase the sensitivity of speed for vehicle in junction
  if (obstable.type() == Obstacle::ObstacleType::VEHICLE &&
      PlanningMap::Instance()->IsPointInJunction(states[0].position.x(),
                                                 states[0].position.y(), 0)) {
    // Use the older half to calculate speed
    auto half_size = his_size / 2;
    double his_speed = (obs_velocity_presum_[his_size - 1] -
                        (half_size ? obs_velocity_presum_[half_size - 1] : 0)) /
                       (his_size - half_size);
    if (his_speed > FLAGS_planning_still_obstacle_speed_threshold) return false;
  }

  auto len = std::min<std::size_t>(his_size, 5);
  double curr_speed = obs_velocity_presum_[len - 1] / len;

  return curr_speed < FLAGS_planning_still_obstacle_speed_threshold;
}

void DataCenter::UpdateObstacleDecision(Obstacle &obstacle) {
  if (obstacle.history_decision().empty()) {
    LOG_DEBUG("Frame[{}] Obstacle[{}] history decision empty",
              current_frame()->sequence_num(), obstacle.id());
    if (obstacle.decision().empty() ||
        obstacle.decision().front().decision_type() ==
            Decision::DecisionType::UNKNOWN) {
      obstacle.set_decision_continuous_frame_cnt(0);
    } else {
      obstacle.set_decision_continuous_frame_cnt(1);
    }
    return;
  }

  if (obstacle.decision().empty()) {
    obstacle.set_decision_continuous_frame_cnt(0);
    return;
  }

  Decision::DecisionType history_decision_type =
      obstacle.history_decision().front().decision_type();
  Decision::DecisionType current_decision_type =
      obstacle.decision().front().decision_type();

  if (current_decision_type == Decision::DecisionType::UNKNOWN) {
    obstacle.set_decision_continuous_frame_cnt(0);
    return;
  }

  if (history_decision_type == Decision::DecisionType::YIELD_DOWN ||
      history_decision_type == Decision::DecisionType::STOP_DOWN ||
      history_decision_type == Decision::DecisionType::FOLLOW_DOWN) {
    if (current_decision_type == Decision::DecisionType::YIELD_DOWN ||
        current_decision_type == Decision::DecisionType::STOP_DOWN ||
        current_decision_type == Decision::DecisionType::FOLLOW_DOWN) {
      obstacle.set_decision_continuous_frame_cnt(
          (obstacle.decision_continuous_frame_cnt() + 1));
    }
  } else {
    if (history_decision_type == current_decision_type) {
      obstacle.set_decision_continuous_frame_cnt(
          (obstacle.decision_continuous_frame_cnt() + 1));
    } else {
      obstacle.set_decision_continuous_frame_cnt(1);
    }
  }
}

std::unique_ptr<Frame> DataCenter::CreateShadowFrame() const {
  std::unique_ptr<Frame> frame =
      std::make_unique<Frame>(frame_->sequence_num());
  return frame;
}

void DataCenter::SaveFrame() {
  lastframe_state_ = master_info_.curr_scenario();
  sequence_queue_.push_back(frame_->sequence_num());

  if (frame_->IsPlanningDataSafe() &&
      frame_->mutable_planning_data() != nullptr &&
      frame_->mutable_planning_data()->mutable_decision_data() != nullptr) {
    auto &obstacles = frame_->mutable_planning_data()
                          ->mutable_decision_data()
                          ->practical_obstacle();
    for (auto &obstacle : obstacles) {
      if (obstacle == nullptr) {
        continue;
      }
      Obstacle *tmp_obstacle = nullptr;
      if (mutable_object_table()->get_obstacle(obstacle->id(), &tmp_obstacle)) {
        tmp_obstacle->mutable_decision()->clear();
        tmp_obstacle->mutable_history_decision()->clear();
      }
    }
    for (auto &obstacle : obstacles) {
      if (obstacle == nullptr) {
        continue;
      }
      UpdateObstacleDecision(*obstacle);
      obstacle->switch_to_history();
    }
    for (auto &obstacle : obstacles) {
      if (obstacle == nullptr) {
        continue;
      }
      Obstacle *tmp_obstacle = nullptr;
      if (mutable_object_table()->get_obstacle(obstacle->id(), &tmp_obstacle)) {
        tmp_obstacle->mutable_decision()->insert(
            tmp_obstacle->mutable_decision()->end(),
            obstacle->decision().begin(), obstacle->decision().end());
        tmp_obstacle->mutable_history_decision()->insert(
            tmp_obstacle->mutable_history_decision()->end(),
            obstacle->history_decision().begin(),
            obstacle->history_decision().end());
        tmp_obstacle->set_decision_continuous_frame_cnt(
            obstacle->decision_continuous_frame_cnt());
      }
    }
  }

  frames_[frame_->sequence_num()] = std::move(frame_);

  if (sequence_queue_.size() >
      static_cast<std::size_t>(FLAGS_planning_max_history_result)) {
    frames_.erase(sequence_queue_.front());
    sequence_queue_.pop_front();
  }
}

void DataCenter::SaveTask(TaskInfo &task_info) {
  frame_ = std::move(task_info.current_frame());
}

void DataCenter::ClearHistoryFrame() {
  frames_.clear();
  sequence_queue_.clear();
}

void DataCenter::SetMonitorString(const std::string &str,
                                  MonitorItemSource source) {
  if (source < MonitorItemSource::MAX_ITERM_SIZE) {
    monitor_message_vec_[source] = str;
  }
}

void DataCenter::AggregateMonitorString() {
  monitor_message_.clear_msg();
  for (size_t i = 0; i < monitor_message_vec_.size(); ++i) {
    auto &str = monitor_message_vec_[i];
    if (str.length() > 0) {
      monitor_message_.add_msg(str);
    }
  }
}

PredictionTrajectory DataCenter::GenerateConstVelocityTraj(
    const Obstacle &obstacle) {
  PredictionTrajectory ans{};
  ans.set_start_timestamp(obstacle.get_time_stamp());
  ans.set_probability(1.);
  ans.set_predictor_type(
      neodrive::global::prediction::Trajectory_PredictorType::
          Trajectory_PredictorType_FreeMove);
  auto pts = ans.mutable_trajectory_points();

  // Current position as first point
  TrajectoryPoint prev{};
  prev.set_x(obstacle.center().x());
  prev.set_y(obstacle.center().y());
  prev.set_theta(obstacle.velocity_heading());
  prev.set_velocity(obstacle.speed());
  prev.set_relative_time(0.);
  prev.set_confidence(1.);
  pts->push_back(std::move(prev));

  while (pts->size() < 50) {
    auto prev = &pts->back();
    TrajectoryPoint pt{};
    pt.set_x(prev->x() + prev->velocity() * std::cos(prev->theta()) * 0.1);
    pt.set_y(prev->y() + prev->velocity() * std::sin(prev->theta()) * 0.1);
    pt.set_theta(prev->theta());
    pt.set_velocity(prev->velocity());
    pt.set_relative_time(prev->relative_time() + 0.1);
    pt.set_confidence(1.);
    pts->push_back(std::move(pt));
  }

  return ans;  // RVO
}

const VehicleStateProxy &DataCenter::vehicle_state_proxy() const {
  return vehicle_state_odometry_;
}

Frame *DataCenter::frame(const uint32_t sequence_num) const {
  auto iter = frames_.find(sequence_num);
  if (iter != frames_.end()) {
    return iter->second.get();
  }
  return nullptr;
}

Frame *DataCenter::current_frame() const {
  if (frame_ == nullptr) {
    LOG_ERROR("frame_ is nullptr, crash error");
    return nullptr;
  }
  return frame_.get();
}

const Frame *DataCenter::last_frame() const {
  if (sequence_queue_.empty() || current_frame_seq_ < 1 ||
      frames_.find((current_frame_seq_ - 1)) == frames_.end()) {
    return nullptr;
  }
  if (frames_.find(current_frame_seq_ - 1)->second->IsPlanningDataSafe()) {
    return frames_.find(current_frame_seq_ - 1)->second.get();
  }

  return nullptr;
}

ObjectTable *DataCenter::mutable_object_table() const {
  if (object_table_odometry_ == nullptr) {
    LOG_ERROR("object_table odometry is nullptr.");
    return nullptr;
  }
  return object_table_odometry_.get();
}

ObjectTable *DataCenter::mutable_object_table_utm() const {
  if (object_table_utm_ == nullptr) {
    LOG_ERROR("object_table utm is nullptr.");
    return nullptr;
  }
  return object_table_utm_.get();
}

const std::unordered_map<int, Obstacle>
    &DataCenter::caution_speed_utm_obstacles() {
  return caution_speed_utm_obstacles_;
}

const std::unordered_map<int, Obstacle>
    &DataCenter::caution_speed_odometry_obstacles() {
  return caution_speed_odometry_obstacles_;
}

void DataCenter::AddRecordEvent(EventOfInterest::EventType event_type,
                                int32_t record_start_offset,
                                int32_t record_end_offset) {
  auto curr_time = common::util::TimeLogger::GetCurrentTimeseocnd();
  if (curr_time - event_of_interest_prev_time_[event_type] >
      kMinEventOfInterestMsgIntervel) {
    auto event_signal = event_msg_pool_.GetSharedPtr();
    event_signal->mutable_header()->set_module_name("planning");
    event_signal->mutable_header()->set_timestamp_sec(curr_time);
    event_signal->set_event_type(event_type);
    event_signal->set_start_time(record_start_offset);
    event_signal->set_end_time(record_end_offset);
    record_event_list_.emplace_back(std::move(event_signal));
    event_of_interest_prev_time_[event_type] = curr_time;
  }
}

void DataCenter::SaveMonitorMessage() {
  LOG_DEBUG("SaveMonitorMessage");
  auto &vehicle_state_proxy = environment_.vehicle_state_proxy();
  auto &control_command = (*control_command_msg.ptr);
  static char str_buffer[256];
  std::string estop_reason_str = "";
  if (control_command.has_contrl_context() &&
      control_command.contrl_context().has_estop_reason()) {
    estop_reason_str = control_command.contrl_context().estop_reason();
  }

  auto get_pitch_from_quaternion =
      [](const neodrive::global::common::Quaternion &quat) {
        double x = quat.qx();
        double y = quat.qy();
        double z = quat.qz();
        double w = quat.qw();

        double vect3x = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        double vect3y = asin(2 * (w * y - z * x));
        double vect3z = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

        double absr1 = fabs(vect3x) + fabs(vect3y) + fabs(vect3z);

        // see if any less rotation
        if (absr1 >= M_PI * 2.0) {
          double vect4x = vect3x + M_PI;
          double vect4y = vect3y + M_PI;
          double vect4z = vect3z + M_PI;
          if (vect4x >= M_PI) {
            vect4x -= M_PI * 2.0;
          }
          if (vect4y >= M_PI) {
            vect4y -= M_PI * 2.0;
          }
          if (vect4z >= M_PI) {
            vect4z -= M_PI * 2.0;
          }
          double absr2 = fabs(vect4x) + fabs(vect4y) + fabs(vect4z);
          if (absr2 < absr1) {
            return -vect4y;
          }
        }
        return -vect3y;
      };
  const double kDegreePerRad = 180.0 / M_PI;
  double max_negative_torque = 0.0;
  if (chassis_msg.ptr != nullptr && chassis_msg.ptr->has_max_negative_torque())
    max_negative_torque = chassis_msg.ptr->max_negative_torque();
  sprintf(str_buffer,
          "[EGO][v: %.1lf(cmd %.1lf)KPH a: %.1lf(cmd %.1lf) s: %.1lf(cmd %.1lf)"
          "][heading: %.1lf][throttle: %.1lf brake: %.1lf max_torque: %.1f "
          "estop: %d %s][pitch: "
          "worldmodel %.1lf map %.1lf traj %.1lf]",
          vehicle_state_proxy.LinearVelocity() * 3.6,
          master_info_.adc_target_speed() * 3.6,
          vehicle_state_proxy.LinearAcceleration(),
          master_info_.adc_target_accel(), vehicle_state_proxy.SteerPercent(),
          control_command.steering_target(), vehicle_state_proxy.Heading(),
          control_command.throttle(), control_command.brake(),
          max_negative_torque, control_command.estop(),
          estop_reason_str.c_str(),
          get_pitch_from_quaternion(
              pose_base_link_in_odometry_msg.ptr->pose().orientation()) *
              kDegreePerRad,
          frame_->outside_planner_data().init_point_ref_point.pitch() *
              kDegreePerRad,
          environment_.vehicle_state_proxy().TrajPitch() * kDegreePerRad);
  SetMonitorString(str_buffer, MonitorItemSource::EGO_CAR);

  auto &localization_config =
      config::PlanningConfig::Instance()->plan_config().localization;
  auto loc_status = localization_status_msg.ptr;
  std::string error_code_str = "";
  if (loc_status->has_error_code()) {
    if (loc_status->error_code().has_imu()) {
      error_code_str += ErrorType_Name(loc_status->error_code().imu());
    }
    if (loc_status->error_code().has_dead_reckoning()) {
      error_code_str +=
          ErrorType_Name(loc_status->error_code().dead_reckoning());
    }
    if (loc_status->error_code().has_ndt()) {
      error_code_str += ErrorType_Name(loc_status->error_code().ndt());
    }
    if (loc_status->error_code().has_gnss()) {
      error_code_str += ErrorType_Name(loc_status->error_code().gnss());
    }
  }
  std::string odom_error_code_str =
      SensorErrorCode_Name(loc_status->odometry_sensor_error_code());
  double odom_veh_delay = (loc_status->odometry_measurement_time() -
                           loc_status->odometry_raw_veh_time()) *
                          1000.;
  double odom_imu_delay = (loc_status->odometry_measurement_time() -
                           loc_status->odometry_raw_imu_time()) *
                          1000.;
  is_odom_sensor_delay_ =
      (odom_imu_delay > localization_config.hdmap_odom_delay_threshold) ||
      (odom_veh_delay > localization_config.hdmap_odom_delay_threshold);

  std::string utm_error_code_str =
      SensorErrorCode_Name(loc_status->msf_status().sensor_error_code());

  double umt_veh_delay =
      (loc_status->utm_measurement_time() - loc_status->utm_raw_veh_time()) *
      1000.;
  double utm_imu_delay =
      (loc_status->utm_measurement_time() - loc_status->utm_raw_imu_time()) *
      1000.;
  sprintf(str_buffer,
          "[LOC][odom:%d delay:%2.0lf %2.0lfms %s][utm delay:%2.0lf %2.0lfms "
          "%s][gnss:%u %u][error:%s] [on_map:%d] [lane:%i->%i]",
          loc_status->odometry_predict(), odom_veh_delay, odom_imu_delay,
          odom_error_code_str.c_str(), umt_veh_delay, utm_imu_delay,
          utm_error_code_str.c_str(),
          loc_status->msf_status().gnsspos_position_type(),
          loc_status->msf_status().heading_position_type(),
          error_code_str.c_str(), is_on_map_, current_lane_idx_,
          target_lane_idx_);
  SetMonitorString(str_buffer, MonitorItemSource::LOCALIZATION);

  std::string turn_light_str =
      neodrive::global::status::HornLightsCmd::TurnLevel_Name(turn_light_);
  sprintf(str_buffer, "[NAVIGATION][%s][turn light:%s]",
          navigation_type_str_.c_str(), turn_light_str.c_str());
  SetMonitorString(str_buffer, MonitorItemSource::NAVIGATION);

  double running_time = init_frame_time_ - initialized_time_;
  int hourse = static_cast<int>(running_time / 3600.);
  int miniutes = static_cast<int>((running_time - hourse * 3600) / 60.);
  int seconds = static_cast<int>(running_time - hourse * 3600 - miniutes * 60.);
  sprintf(str_buffer, "[STATISTICS][%d:%d:%d][aeb:%d fcw:%d aeb_takeover:%d]",
          hourse, miniutes, seconds, aeb_active_cnt_, fcw_active_cnt_,
          aeb_take_over_cnt_);
  SetMonitorString(str_buffer, MonitorItemSource::STATISTICS);
}

void DataCenter::UpdateCameraSegments() {
  camera_segments_.Clear();
  if (camera_freespace_msg.ptr == nullptr) {
    LOG_WARN("camera_freespace_msg ptr is nullptr!");
    return;
  }
  LOG_INFO("camera_freespace_msg time: {}",
           camera_freespace_msg.ptr->header().timestamp_sec());
  if (camera_freespace_msg.ptr->camera_segments().empty()) {
    LOG_ERROR("camera_segments is empty!");
    return;
  }

  for (const auto &camera_segments :
       camera_freespace_msg.ptr->camera_segments()) {
    if (camera_segments.camera_id() ==
        config::PlanningConfig::Instance()->plan_config().back_out.camera_id) {
      LOG_INFO("camera_segments.camera_id = {}", camera_segments.camera_id());
      camera_segments_ = camera_segments;
      return;
    }
  }
  LOG_ERROR(
      "{} is wrong camera_id",
      config::PlanningConfig::Instance()->plan_config().back_out.camera_id);
}

}  // namespace planning
}  // namespace neodrive
