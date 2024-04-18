#pragma once
#include "container_manager.h"
#include <limits>

namespace neodrive {
namespace planning_rl {

ContainerManager::ContainerManager() {}

bool ContainerManager::Init() {
  INIT_NEOLOG_NAME("planning_rl");
  if (initialized_) {
    return true;
  }
  config::PlanningRLConfig::Instance()->ReLoadConfigFromJson();
  auto &topics_config =
      config::PlanningRLConfig::Instance()->topics_config().topics;
  node_ = neodrive::cyber::CreateNode("planning_rl_container");
  if (node_ == nullptr) {
    return false;
  }
  // neodrive::cyber::proto::QosProfile qos_profile;
  // neodrive::cyber::common::GetProtoFromFile(
  //     config.predictor.prediction_localization_reader_conf, &qos_profile);
  // LOG_INFO("prediction_localization_reader_conf: {}",
  //          qos_profile.DebugString());
  cyber::ReaderConfig reader_config;
  reader_config.channel_name = topics_config.at("perception_obstacles").topic;
  reader_config.pending_queue_size = 50;
  std::cout << "channel_name" << std::endl;
  std::cout << reader_config.channel_name << std::endl;
  perception_obstacles_reader_ = node_->CreateReader<PerceptionObstacles>(
      reader_config,
      std::bind(&ContainerManager::PerceptionObstaclesMsgCallback, this,
                std::placeholders::_1));
  if (perception_obstacles_reader_ == nullptr) {
    std::cout << "perception_obstacles_reader_" << std::endl;
    return false;
  }

  reader_config.channel_name = "/localization/100hz/localization_pose";
  reader_config.pending_queue_size = 500;
  localization_pose_obstacles_reader_ =
      node_->CreateReader<LocalizationEstimate>(
          reader_config,
          std::bind(&ContainerManager::LocalizationObstaclesMsgCallback, this,
                    std::placeholders::_1));
  if (localization_pose_obstacles_reader_ == nullptr) {
    return false;
  }
  // {
  //   cyber::ReaderConfig reader_config;
  //   reader_config.channel_name = "/planning/proxy/DuDriveChassis";
  //   reader_config.pending_queue_size = 500;
  //   chassis_reader_ = node_->CreateReader<Chassis>(
  //       reader_config, std::bind(&ContainerManager::ChassisMsgCallback, this,
  //                                std::placeholders::_1));
  //   if (chassis_reader_ == nullptr) {
  //     return false;
  //   }
  // }
  reader_config.channel_name = "/pnc/planning";
  reader_config.pending_queue_size = 50;
  ego_trajectory_reader_ = node_->CreateReader<ADCTrajectory>(
      reader_config, std::bind(&ContainerManager::ADCTrajectoryMsgCallback,
                               this, std::placeholders::_1));
  if (ego_trajectory_reader_ == nullptr) {
    return false;
  }

  reader_config.channel_name = topics_config.at("routing_result").topic;
  std::cout << reader_config.channel_name << std::endl;
  reader_config.pending_queue_size = 50;
  routing_reader_ = node_->CreateReader<RoutingResult>(
      reader_config, std::bind(&ContainerManager::RoutingMsgCallback, this,
                               std::placeholders::_1));
  // std::cout << "routing_reader_1" << std::endl;
  if (routing_reader_ == nullptr) {
    std::cout << "routing_reader_null" << std::endl;
    return false;
  }

  reader_config.channel_name =
      topics_config.at("pose_base_link_in_odometry").topic;
  reader_config.pending_queue_size = 50;
  odometry_pose_reader_ = node_->CreateReader<PoseStamped>(
      reader_config, std::bind(&ContainerManager::OdomPoseStampedMsgCallback,
                               this, std::placeholders::_1));
  if (odometry_pose_reader_ == nullptr) {
    return false;
  }

  reader_config.channel_name = topics_config.at("pose_base_link_in_utm").topic;
  reader_config.pending_queue_size = 50;
  utm_pose_reader_ = node_->CreateReader<PoseStamped>(
      reader_config, std::bind(&ContainerManager::UtmPoseStampedMsgCallback,
                               this, std::placeholders::_1));
  if (utm_pose_reader_ == nullptr) {
    return false;
  }

  reader_config.channel_name = topics_config.at("twist_base_link").topic;
  reader_config.pending_queue_size = 50;
  twist_reader_ = node_->CreateReader<TwistStamped>(
      reader_config, std::bind(&ContainerManager::TwistStampedMsgCallback, this,
                               std::placeholders::_1));
  if (twist_reader_ == nullptr) {
    return false;
  }

  reader_config.channel_name = topics_config.at("pnc_traffic_light").topic;
  reader_config.pending_queue_size = 50;
  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      reader_config, std::bind(&ContainerManager::TrafficLightDetectionMsgCallback, this,
                               std::placeholders::_1));
  if (traffic_light_reader_ == nullptr) {
    return false;
  }

  LOG_INFO("ContainerManager initialized");
  initialized_ = true;
  return true;
}

bool ContainerManager::HasNewMsg() {
  std::lock_guard<std::mutex> lock(msg_mtx_);
  if (perception_obstacles_msgs_.empty()) {
    return false;
  }
  return true;
}

PerceptionObstaclesShrPtr ContainerManager::FetchPerceptionObstacleMsgs(
    const size_t max_process_msgs_per_interval) {
  auto curr_time = cyber::Time::Now().ToSecond();
  size_t remain_msg_size = std::max(1UL, max_process_msgs_per_interval);
  if (perception_obstacles_msgs_.empty()) {
    std::cout << "perception_obstacles_msgs_.empty()" << std::endl;
    return nullptr;
  }
  while (perception_obstacles_msgs_.size() > remain_msg_size) {
    auto msg = perception_obstacles_msgs_.back();
    // auto delay_sec = curr_time - msg->header().timestamp_sec();
    // container has some too old msg, do not need to be processed
    // LOG_WARN(
    //     "ignore too old msg, curr_time: {:.4f} msg time: {:.4f} delay: {:.4f}
    //     " "sequence_num: {} size: {}", curr_time,
    //     msg->header().timestamp_sec(), delay_sec,
    //     msg->header().sequence_num(), perception_obstacles_msgs_.size());
    perception_obstacles_msgs_.pop_back();
  }
  auto msg = perception_obstacles_msgs_.back();
  perception_obstacles_msgs_.pop_back();
  return msg;
}

// ADCTrajectoryShrPtr ContainerManager::FindMatchedEgoTrajectoryMsg(
//     const double timestamp_sec) {
//   ADCTrajectoryShrPtr matched_ego_trajectory_msg = nullptr;
//   if (ego_trajectory_msgs_.empty()) {
//     // LOG_WARN("not received ego trajectory msg");
//     return matched_ego_trajectory_msg;
//   }
//   double min_time_diff = std::numeric_limits<double>::max();
//   for (auto it = ego_trajectory_msgs_.rbegin();
//        it != ego_trajectory_msgs_.rend(); ++it) {
//     auto msg = *it;
//     auto timestamp = msg->header().timestamp_sec();
//     auto time_diff = std::fabs(timestamp - timestamp_sec);
//     if (time_diff < min_time_diff) {
//       min_time_diff = time_diff;
//       matched_ego_trajectory_msg = msg;
//     } else {
//       break;
//     }
//   }
//   if (matched_ego_trajectory_msg == nullptr) {
//     // LOG_DEBUG("failed to find matched_ego_trajectory_msg");
//     return matched_ego_trajectory_msg;
//   }
//   while (!ego_trajectory_msgs_.empty()) {
//     auto msg = ego_trajectory_msgs_.back();
//     if (msg->header().sequence_num() !=
//         matched_ego_trajectory_msg->header().sequence_num()) {
//       ego_trajectory_msgs_.pop_back();
//     } else {
//       break;
//     }
//   }
//   if (matched_ego_trajectory_msg != nullptr) {
//     auto time_diff =
//         timestamp_sec - matched_ego_trajectory_msg->header().timestamp_sec();
//     if (std::fabs(time_diff) > 0.01) {
//       LOG_WARN("time diff between ego trajectory and perception: {:.4f}",
//        time_diff);
//     }
//   }
//   return matched_ego_trajectory_msg;
// }

LocalizationEstimateShrPtr ContainerManager::FindMatchedLocalizationPoseMsg(
    const double timestamp_sec) {
  LocalizationEstimateShrPtr matched_localization_pose_msg = nullptr;
  if (localization_pose_msgs_.empty()) {
    // LOG_WARN("not received ego trajectory msg");
    return matched_localization_pose_msg;
  }
  double min_time_diff = std::numeric_limits<double>::max();
  for (auto it = localization_pose_msgs_.rbegin();
       it != localization_pose_msgs_.rend(); ++it) {
    auto msg = *it;
    auto timestamp = msg->header().timestamp_sec();
    auto time_diff = std::fabs(timestamp - timestamp_sec);
    if (time_diff < min_time_diff) {
      min_time_diff = time_diff;
      matched_localization_pose_msg = msg;
    } else {
      break;
    }
  }
  if (matched_localization_pose_msg == nullptr) {
    LOG_DEBUG("failed to find matched_localization_pose_msg");
    return matched_localization_pose_msg;
  }
  while (!localization_pose_msgs_.empty()) {
    auto msg = localization_pose_msgs_.back();
    if (msg->header().sequence_num() !=
        matched_localization_pose_msg->header().sequence_num()) {
      localization_pose_msgs_.pop_back();
    } else {
      break;
    }
  }
  if (matched_localization_pose_msg != nullptr) {
    auto time_diff =
        timestamp_sec - matched_localization_pose_msg->header().timestamp_sec();
    if (std::fabs(time_diff) > 0.01) {
      LOG_WARN("time diff between ego trajectory and perception: {:.4f}",
               time_diff);
    }
  }
  return matched_localization_pose_msg;
}

ChassisShrPtr ContainerManager::FindMatchedChassisMsg(
    const double timestamp_sec) {
  ChassisShrPtr matched_chassis_msg = nullptr;
  if (chassis_msgs_.empty()) {
    // LOG_WARN("not received ego trajectory msg");
    return matched_chassis_msg;
  }
  double min_time_diff = std::numeric_limits<double>::max();
  for (auto it = chassis_msgs_.rbegin(); it != chassis_msgs_.rend(); ++it) {
    auto msg = *it;
    auto timestamp = msg->header().timestamp_sec();
    auto time_diff = std::fabs(timestamp - timestamp_sec);
    if (time_diff < min_time_diff) {
      min_time_diff = time_diff;
      matched_chassis_msg = msg;
    } else {
      break;
    }
  }
  if (matched_chassis_msg == nullptr) {
    // LOG_DEBUG("failed to find matched_localization_pose_msg");
    return matched_chassis_msg;
  }
  while (!chassis_msgs_.empty()) {
    auto msg = chassis_msgs_.back();
    if (msg->header().sequence_num() !=
        matched_chassis_msg->header().sequence_num()) {
      chassis_msgs_.pop_back();
    } else {
      break;
    }
  }
  if (matched_chassis_msg != nullptr) {
    auto time_diff =
        timestamp_sec - matched_chassis_msg->header().timestamp_sec();
    if (std::fabs(time_diff) > 0.01) {
      // LOG_WARN("time diff between ego trajectory and perception: {:.4f}",
      //  time_diff);
    }
  }
  return matched_chassis_msg;
}

PoseStampedShrPtr ContainerManager::GetOdomPoseMsg() {
  PoseStampedShrPtr odom_pose_msg = nullptr;
  if (odom_pose_msgs_.empty()) {
    // LOG_WARN("not received ego trajectory msg");
    return odom_pose_msg;
  }
  odom_pose_msg = odom_pose_msgs_.front();
  return odom_pose_msg;
}

PoseStampedShrPtr ContainerManager::GetUtmPoseMsg() {
  PoseStampedShrPtr utm_pose_msg = nullptr;
  if (utm_pose_msgs_.empty()) {
    // LOG_WARN("not received ego trajectory msg");
    return utm_pose_msg;
  }
  utm_pose_msg = utm_pose_msgs_.front();
  return utm_pose_msg;
}

TwistStampedShrPtr ContainerManager::GetTwistMsg() {
  TwistStampedShrPtr twist_msg = nullptr;
  if (twist_msgs_.empty()) {
    // LOG_WARN("not received ego trajectory msg");
    return twist_msg;
  }
  twist_msg = twist_msgs_.front();
  return twist_msg;
}

TrafficLightDetectionShrPtr ContainerManager::GetTrafficLightMsg() {
  TrafficLightDetectionShrPtr traffic_light_msg = nullptr;
  if (traffic_light_msgs_.empty()) {
    // LOG_WARN("not received ego trajectory msg");
    return traffic_light_msg;
  }
  traffic_light_msg = traffic_light_msgs_.front();
  return traffic_light_msg;
}

void ContainerManager::ResetContainer(const double timestamp_sec) {
  // read the first message again when cycly play record
  LOG_INFO("read the first message again when cycly play record");
  ego_trajectory_msgs_.clear();
  perception_obstacles_msgs_.clear();
  localization_pose_msgs_.clear();
  odom_pose_msgs_.clear();
  utm_pose_msgs_.clear();
  twist_msgs_.clear();
  prev_perception_obstacles_msg_time_ = timestamp_sec;
  prev_localization_pose_msg_time_ = timestamp_sec;
}

void ContainerManager::ResetMsgs() {
  // reset messages
  LOG_INFO("reset messages");
  ego_trajectory_msgs_.clear();
  perception_obstacles_msgs_.clear();
  localization_pose_msgs_.clear();
  odom_pose_msgs_.clear();
  utm_pose_msgs_.clear();
  twist_msgs_.clear();
  reset_msg_ = true;
}

bool ContainerManager::GetContainerMsg(
    const size_t max_process_msgs_per_interval, ContainerMessageShrPtr &msg) {
  std::lock_guard<std::mutex> lock(msg_mtx_);
  // Get perception_obstacles
  auto perception_obstacles_msg =
      FetchPerceptionObstacleMsgs(max_process_msgs_per_interval);
  if (perception_obstacles_msg == nullptr) {
    LOG_WARN("not received perception obstacles msg");
    return false;
    // add utm check
  }
  auto timestamp_sec = perception_obstacles_msg->header().timestamp_sec();

  // Get localization_pose_msg
  auto matched_localization_pose_msg =
      FindMatchedLocalizationPoseMsg(timestamp_sec);
  if (matched_localization_pose_msg == nullptr) {
    LOG_WARN("not received localization_pose_msg");
    return false;
  }
  // Get chassis_msg
  // auto matched_chassis_msg = FindMatchedChassisMsg(timestamp_sec);
  if (msg == nullptr) {
    msg = std::make_shared<ContainerMessage>();
  }
  auto odom_pose_msg = GetOdomPoseMsg();
  if (odom_pose_msg == nullptr) {
    LOG_WARN("not received world_model_odom_pose_msg");
    return false;
  }
  auto utm_pose_msg = GetUtmPoseMsg();
  if (utm_pose_msg == nullptr) {
    LOG_WARN("not received world_model_utm_pose_msg");
    return false;
  }
  auto twist_msg = GetTwistMsg();
  if (twist_msg == nullptr) {
    LOG_WARN("not received world_model_twist_msg");
    return false;
  }
  auto traffic_light_msg = GetTrafficLightMsg();
  if (traffic_light_msg == nullptr) {
    LOG_WARN("not received traffic light detection msg")
    return false;
  }
  msg->odom_pose = odom_pose_msg;
  msg->utm_pose = utm_pose_msg;
  msg->twist = twist_msg;
  msg->traffic_light_detect = traffic_light_msg;
  msg->traffic_lights_ptr = traffic_lights_ptr;
  bool use_odom = true;
  if (use_odom) {
    GetFrameDataOdom(perception_obstacles_msg, odom_pose_msg, twist_msg,
                     timestamp_sec, msg);
    if (reference_line_ptr == nullptr || reference_line_ptr->size() < 2) {
      LOG_WARN("reference_line_ptr null or reference_line_size < 2");
      return false;
    }
    msg->reference_line_ptr = reference_line_ptr;
    // auto left_lane_bound_point = refline_point100[i].left_bound_point();
    // auto right_lane_bound_point = refline_point100[i].right_bound_point();
    // auto left_bound_point = refline_point100[i].left_road_bound_point();
    // auto right_bound_point = refline_point100[i].right_road_bound_point();
    // auto x = refline_point100[i].x();
    // auto y = refline_point100[i].y();
    // auto s = refline_point100[i].s();
    // auto theta = refline_point100[i].heading();
    // auto curvature = refline_point100[i].kappa();
    // auto is_stop_line = refline_point100[i].is_in_stop_sign();
    // auto is_crossroad = refline_point100[i].is_in_crosswalk();
    // auto is_speed_bump = refline_point100[i].is_in_speed_bump();
    // auto lane_turn = refline_point100[i].signal_type();
    // std::cout << "pts_utm:" << std::endl;
    // auto pts = msg->reference_line_ptr->ref_points();
    // // for (auto &p : pts) {
    // //   std::cout << p.x() << std::endl;
    // //   std::cout << p.y() << std::endl;
    // //   std::cout << p.heading() << std::endl;
    // //   std::cout << p.s() << std::endl;
    // //   std::cout << std::endl;
    // // }
    // for (auto &p : pts) {
    //   std::cout << "{" << std::endl;
    //   std::cout << "left_lane_bound_point: " << p.left_bound_point().x() << "," << std::endl;
    //   std::cout << "left_lane_bound_point: " << p.left_bound_point().y() << "," << std::endl;
    //   std::cout << "right_lane_bound_point: " << p.right_bound_point().x() << "," << std::endl;
    //   std::cout << "right_lane_bound_point: " << p.right_bound_point().y() << "," << std::endl;
    //   std::cout << "left_bound_point: " << p.left_road_bound_point().x() << "," << std::endl;
    //   std::cout << "left_bound_point: " << p.left_road_bound_point().y() << "," << std::endl;
    //   std::cout << "right_bound_point: " << p.right_road_bound_point().x() << "," << std::endl;
    //   std::cout << "right_bound_point: " << p.right_road_bound_point().y() << "," << std::endl;
    //   std::cout << "x: " << p.x() << "," << std::endl;
    //   std::cout << "y: " << p.y() << "," << std::endl;
    //   std::cout << "s: " << p.s() << "," << std::endl;
    //   std::cout << "theta: " << p.heading() << "," << std::endl;
    //   std::cout << "curvature: " << p.kappa() << "," << std::endl;
    //   std::cout << "is_stop_line: " << p.is_in_stop_sign() << "," << std::endl;
    //   std::cout << "is_crossroad: " << p.is_in_crosswalk() << "," << std::endl;
    //   std::cout << "is_speed_bump: " << p.is_in_speed_bump() << "," << std::endl;
    //   std::cout << "lane_turn: " << p.signal_type() << "," << std::endl;
    //   std::cout << "}," << std::endl;
    // }
    TransformReflineToOdometry(msg);
    // std::cout << "pts_odom:" << std::endl;
    // pts = msg->reference_line_ptr->ref_points();
    // // for (auto &p : pts) {
    // //   std::cout << p.x() << std::endl;
    // //   std::cout << p.y() << std::endl;
    // //   std::cout << p.heading() << std::endl;
    // //   std::cout << p.s() << std::endl;
    // //   std::cout << std::endl;
    // // }
    // for (auto &p : pts) {
    //   std::cout << "{" << std::endl;
    //   std::cout << "left_lane_bound_point: " << p.left_bound_point().x() << "," << std::endl;
    //   std::cout << "left_lane_bound_point: " << p.left_bound_point().y() << "," << std::endl;
    //   std::cout << "right_lane_bound_point: " << p.right_bound_point().x() << "," << std::endl;
    //   std::cout << "right_lane_bound_point: " << p.right_bound_point().y() << "," << std::endl;
    //   std::cout << "left_bound_point: " << p.left_road_bound_point().x() << "," << std::endl;
    //   std::cout << "left_bound_point: " << p.left_road_bound_point().y() << "," << std::endl;
    //   std::cout << "right_bound_point: " << p.right_road_bound_point().x() << "," << std::endl;
    //   std::cout << "right_bound_point: " << p.right_road_bound_point().y() << "," << std::endl;
    //   std::cout << "x: " << p.x() << "," << std::endl;
    //   std::cout << "y: " << p.y() << "," << std::endl;
    //   std::cout << "s: " << p.s() << "," << std::endl;
    //   std::cout << "theta: " << p.heading() << "," << std::endl;
    //   std::cout << "curvature: " << p.kappa() << "," << std::endl;
    //   std::cout << "is_stop_line: " << p.is_in_stop_sign() << "," << std::endl;
    //   std::cout << "is_crossroad: " << p.is_in_crosswalk() << "," << std::endl;
    //   std::cout << "is_speed_bump: " << p.is_in_speed_bump() << "," << std::endl;
    //   std::cout << "lane_turn: " << p.signal_type() << "," << std::endl;
    //   std::cout << "}," << std::endl;
    // }
  } else {
    GetFrameData(perception_obstacles_msg, matched_localization_pose_msg,
                 timestamp_sec, msg);
    if (reference_line_ptr == nullptr || reference_line_ptr->size() < 2) {
      LOG_WARN("reference_line_ptr null or reference_line_size < 2");
      return false;
    }
    msg->reference_line_ptr = reference_line_ptr;
  }

  return true;
}
void ContainerManager::GetFrameData(
    PerceptionObstaclesShrPtr &perception_obstacles_msg,
    LocalizationEstimateShrPtr &matched_localization_pose_msg,
    double &timestamp_sec, ContainerMessageShrPtr &msg) {
  LOG_WARN("process frame data");
  int mode = 2;
  // msg->scene_status = 0; // default
  Agents agents(perception_obstacles_msg, mode);
  if (!agents.validate()) {
    LOG_WARN("agents not validate");
    return;
  }
  Ego ego(matched_localization_pose_msg);
  if (!ego.validate()) {
    LOG_WARN("ego not validate");
    return;
  }
  // get step time
  double step_time = 0.1;
  if (msg->frames->size() > 0) {
    step_time = timestamp_sec - msg->timestamp;
  }
  if (step_time > 1) {
    msg->frames->reset();
  }
  // reset msg
  // if (reset_msg_) {
  //   msg->frames->reset();
  //   msg->timestamp = 0.0;
  //   msg->sequence_num = perception_obstacles_msg->header().sequence_num();
  //   msg->perception_obstacles = perception_obstacles_msg;
  //   msg->scene_status = 1; //start
  //   reset_msg_ = false;
  //   LOG_INFO("reset msg, new scene start");
  //   return;
  // }
  LOG_INFO("step_time:", step_time);
  Frame frame(agents, ego, timestamp_sec, step_time);
  msg->frames->append_frame(frame);
  msg->timestamp = timestamp_sec;
  msg->sequence_num = perception_obstacles_msg->header().sequence_num();
  msg->perception_obstacles = perception_obstacles_msg;
  msg->localization_pose = matched_localization_pose_msg;
  // msg->scene_status = 2; //normal
  LOG_INFO("GetFrameData success");
}

void ContainerManager::GetFrameDataOdom(
    PerceptionObstaclesShrPtr &perception_obstacles_msg,
    PoseStampedShrPtr &odom_pose_msg, TwistStampedShrPtr &twist_msg,
    double &timestamp_sec, ContainerMessageShrPtr &msg) {
  LOG_WARN("process frame data");
  int mode = 1;
  msg->scene_status = 0;  // default
  Agents agents(perception_obstacles_msg, mode);
  if (!agents.validate()) {
    LOG_WARN("agents not validate");
    return;
  }
  Ego ego(odom_pose_msg, twist_msg);
  if (!ego.validate()) {
    LOG_WARN("ego not validate");
    return;
  }
  // get step time
  double step_time = 0.1;
  if (msg->frames->size() > 0) {
    step_time = timestamp_sec - msg->timestamp;
  }
  if (step_time > 1) {
    msg->frames->reset();
  }
  // reset msg
  if (reset_msg_) {
    msg->frames->reset();
    msg->timestamp = 0.0;
    msg->sequence_num = perception_obstacles_msg->header().sequence_num();
    msg->perception_obstacles = perception_obstacles_msg;
    msg->scene_status = 1; //start
    reset_msg_ = false;
    prev_new_scene_perception_msg_time_ = timestamp_sec;
    LOG_INFO("reset msg, new scene start");
    return;
  }
  LOG_INFO("step_time:", step_time);
  Frame frame(agents, ego, timestamp_sec, step_time);
  msg->frames->append_frame(frame);
  msg->timestamp = timestamp_sec;
  msg->sequence_num = perception_obstacles_msg->header().sequence_num();
  msg->perception_obstacles = perception_obstacles_msg;
  // scene start time 5s.
  if (timestamp_sec > prev_new_scene_perception_msg_time_ &&
      timestamp_sec < prev_new_scene_perception_msg_time_ + 5.0) {
    LOG_INFO("new scene starts 5s");
    msg->scene_status = 2;  // 5s starts new scene
  } else {
    msg->scene_status = 3;  // normal
    LOG_INFO("scene_status_3");
    // std::cout << timestamp_sec << std::endl;
    // std::cout << prev_new_scene_routing_msg_time_ << std::endl;
  }
  LOG_INFO("GetFrameDataOdom success");
}

void ContainerManager::ADCTrajectoryMsgCallback(
    const ADCTrajectoryShrPtr &message) {
  std::lock_guard<std::mutex> lock(msg_mtx_);
  // PRINT_DELAY_LOG(message, "[ego trajectory delay]", 0.1);
  ego_trajectory_msgs_.push_front(message);
  if (ego_trajectory_msgs_.size() > max_perception_obstacle_queue_size_) {
    ego_trajectory_msgs_.pop_back();
  }
}

void ContainerManager::PerceptionObstaclesMsgCallback(
    const PerceptionObstaclesShrPtr &message) {
  std::lock_guard<std::mutex> lock(msg_mtx_);
  // PRINT_DELAY_LOG(message, "[perception obstacles delay]", 0.1);
  // std::cout<<message->DebugString() << std::endl;
  auto timestamp_sec = message->header().timestamp_sec();
  // std::cout << "timestamp_sec" << std::endl;
  // std::cout << timestamp_sec << std::endl;
  // std::cout << prev_perception_obstacles_msg_time_ -30 << std::endl;
  if (timestamp_sec < prev_perception_obstacles_msg_time_ - 30.0) {
    ResetContainer(timestamp_sec);
    std::cout << "timestamp_sec_not_match" << std::endl;
    return;
  }
  perception_obstacles_msgs_.push_front(message);
  prev_perception_obstacles_msg_time_ = timestamp_sec;
  // std::cout << perception_obstacles_msgs_.size() << std::endl;
  // std::cout << max_perception_obstacle_queue_size_ << std::endl;
  if (perception_obstacles_msgs_.size() > max_perception_obstacle_queue_size_) {
    perception_obstacles_msgs_.pop_back();
    // std::cout << perception_obstacles_msgs_.size() << std::endl;
  }
}
void ContainerManager::LocalizationObstaclesMsgCallback(
    const LocalizationEstimateShrPtr &message) {
  std::lock_guard<std::mutex> lock(msg_mtx_);
  auto timestamp_sec = message->header().timestamp_sec();
  // PRINT_DELAY_LOG(message, "[localization pose delay]", 0.01);
  localization_pose_msgs_.push_front(message);
  prev_localization_pose_msg_time_ = timestamp_sec;
  if (localization_pose_msgs_.size() > max_localization_queue_size_) {
    localization_pose_msgs_.pop_back();
  }
}
// void ContainerManager::ChassisMsgCallback(const ChassisShrPtr &message) {
//   std::lock_guard<std::mutex> lock(msg_mtx_);
//   auto timestamp_sec = message->header().timestamp_sec();
//   // PRINT_DELAY_LOG(message, "[chassis msgs delay]", 0.01);
//   chassis_msgs_.push_front(message);
//   prev_chassis_msg_time_ = timestamp_sec;
//   if (chassis_msgs_.size() > max_localization_queue_size_) {
//     chassis_msgs_.pop_back();
//   }
// }

void ContainerManager::RoutingMsgCallback(const RoutingResultShrPtr &message) {
  if (!message) {
    LOG_INFO("Error, RoutingMsg!");
    return;
  }
  std::lock_guard<std::mutex> lock(msg_mtx_);
  bool reset_flag = false;
  auto timestamp_sec = message->header().timestamp_sec();
  uint32_t seq_num = message->header().sequence_num();
  if (timestamp_sec < prev_routing_msg_time_ - 10.0 ||
      timestamp_sec > prev_routing_msg_time_ + 10.0) {
    if (seq_num != prev_routing_sequence_num_ - 1) {
      std::cout << "routing seqnum not match" << std::endl;
      prev_routing_sequence_num_ = seq_num;
      reset_flag = true;
    }
  }
  // if (timestamp_sec < prev_routing_msg_time_ - 100.0 ||
  //     timestamp_sec > prev_routing_msg_time_ + 30.0) {
  //   std::cout << "routing_timestamp_sec_not_match" << std::endl;
  //   reset_flag = true;
  // }
  prev_routing_msg_time_ = timestamp_sec;
  if (!CheckoutRoutingMsg(*message)) {
    LOG_INFO("skip, CheckoutRoutingMsg!");

  } else {
    routing_msg_ = message;
    // new refline, need check timestamp
    if (reset_flag) {
      ResetMsgs();
      prev_new_scene_routing_msg_time_ = timestamp_sec;
    }
    if (!BuildReferenceLineByRoutingMsg(*routing_msg_)) {
      LOG_INFO("BuildReferenceLine Failed");
    }
    LOG_INFO("BuildReferenceLine Success!");
  }
}

bool ContainerManager::GetLaneSegmentByRouting(
    const RoutingResult &routing,
    std::vector<RoutingResult_LaneSegment> &lanes) {
  for (auto &route : routing.route()) {
    if (route.has_road_info()) {
      for (auto &passage_region : route.road_info().passage_region()) {
        lanes.insert(lanes.end(), passage_region.segment().begin(),
                     passage_region.segment().end());
        // std::cout << "passage_region.segment" << std::endl;
        // std::cout << passage_region.segment()[0].lan << std::endl;
        // std::cout << passage_region.segment()[1] << std::endl;
      }
    } else {
      lanes.insert(lanes.end(),
                   route.junction_info().passage_region().segment().begin(),
                   route.junction_info().passage_region().segment().end());
    }
  }
  if (lanes.empty()) {
    LOG_INFO("lanes.empty!");
    return false;
  }
  routing_holder_1d_.clear();
  auto &holder = routing_holder_1d_;
  for (auto &lane : lanes) {
    if (!holder.empty() && holder.back().lane_id == lane.id() &&
        std::abs(holder.back().end_s - lane.start_s()) < 0.15) {
      holder.back().end_s = lane.end_s();
    } else {
      holder.push_back({lane.id(), lane.start_s(), lane.end_s()});
    }
  }
  if (holder.empty()) {
    LOG_INFO("holder.empty!");
    return false;
  }
  return true;
}

bool ContainerManager::CheckoutRoutingMsg(const RoutingResult &routing) {
  if (!routing.has_routing_request()) {
    LOG_INFO("routing has no routing request!");
    return false;
  }
  auto &routing_request = routing.routing_request();
  if (last_routing_request_.start().id() == "" ||
      last_routing_request_.end().id() == "") {
    last_routing_request_ = routing_request;
    LOG_INFO("Find new Routing!");
  } else {
    // check whether new routing
    if ((last_routing_request_.start().id() == routing_request.start().id() &&
         std::abs(last_routing_request_.start().s() -
                  routing_request.start().s()) < 1.0) &&
        (last_routing_request_.end().id() == routing_request.end().id() &&
         std::abs(last_routing_request_.end().s() - routing_request.end().s()) <
             1.0)) {
      LOG_INFO("Receive Old Routing, Skip!");
      return false;
    } else {
      last_routing_request_ = routing_request;
      LOG_INFO("Receive New Routing, Build it!");
    }
  }
  return true;
}

bool ContainerManager::BuildReferenceLineByRoutingMsg(
    const RoutingResult &routing) {
  std::vector<RoutingResult_LaneSegment> lanes;
  if (!GetLaneSegmentByRouting(routing, lanes)) {
    LOG_INFO("GetLaneSegmentByRouting Error!");
    std::cout << "GetLaneSegmentByRouting Error" << std::endl;
  }
  if (routing_holder_1d_.empty()) {
    LOG_INFO("routing_holder_1d_ Empty!");
    return false;
  }
  if (!BuildReferenceLineByRoutingHolders()) {
    LOG_INFO("BuildReferenceLineByRoutingHolders Failed!");
    std::cout << "BuildReferenceLineByRoutingHolders Failed" << std::endl;
    return false;
  }
  LOG_INFO("BuildReferenceLineByRoutingHolders Success!");
  std::cout << "BuildReferenceLineByRoutingHolders Success!" << std::endl;
  return true;
}

bool ContainerManager::BuildReferenceLineByRoutingHolders() {
  ReferencePointVec2d reference_line_points_3;
  PlanningRLMap::MapPoint3d map_points3d = BuildMapPointsByRoutingHolders();
  if (map_points3d.empty()) {
    LOG_INFO("map_points3d Empty!");
    return false;
  }
  reference_line_ptr = std::make_shared<ReferenceLine>(map_points3d);
  if (reference_line_ptr == nullptr) {
    std::cout << "reference_line_ptr null!" << std::endl;
    LOG_INFO("reference_line_ptr null!");
    return false;
  }
  std::cout << "Get reference_line_ptr!" << std::endl;
  return true;
}

std::vector<PlanningRLMap::MapPoint2d>
ContainerManager::BuildMapPointsByRoutingHolders() {
  std::vector<PlanningRLMap::MapPoint2d> map_points3d;
  int holder_index = 0;
  for (auto &holder : routing_holder_1d_) {
    auto map_points_3 = BuildMapPointsByRoutingHolder(holder);
    if (map_points_3.empty() && holder_index == 0) {
      LOG_INFO("map_points_3 Empty!");
      return map_points3d;
    }
    if (map_points3d.empty()) {
      map_points3d = map_points_3;
    } else {
      // std::cout.precision(10);
      // std::cout << "map_points_3" << std::endl;
      for (size_t i = 0; i < map_points_3.size(); i++) {
        for (size_t j = 0; j < map_points_3[i].size(); j++) {
          // std::cout << i << std::endl;
          // std::cout << j << std::endl;
          // std::cout << map_points_3[i][j].x << std::endl;
          // std::cout << map_points_3[i][j].y << std::endl;
          // std::cout << map_points_3[i][j].s << std::endl;
          map_points3d[i].push_back(map_points_3[i][j]);
        }
      }
    }
    // std::cout << map_points3d[1].size() << std::endl;
    // for (size_t i = 0; i < map_points3d.size(); i++) {
    //   for (size_t j = 0; j < map_points3d[i].size(); j++) {
    //     std::cout << i << std::endl;
    //     std::cout << j << std::endl;
    //     std::cout << map_points3d[i][j].x << std::endl;
    //     std::cout << map_points3d[i][j].y << std::endl;
    //     std::cout << map_points3d[i][j].s << std::endl;
    //   }
    // }
    holder_index++;
  }
  // line_linear_interpolate and smooth again
  std::vector<PlanningRLMap::MapPoint2d> new_map_points3d;
  for (size_t i = 0; i < map_points3d.size(); i++) {
    // std::cout << "ori3d_ref_len:" << map_points3d[i].size() << std::endl;
    // std::cout << map_points3d[i][0].x << std::endl;
    // std::cout << map_points3d[i][0].y << std::endl;
    // std::cout << map_points3d[i][0].s << std::endl;
    // std::cout << map_points3d[i][0].heading << std::endl;
    // std::cout << map_points3d[i].back().x << std::endl;
    // std::cout << map_points3d[i].back().y << std::endl;
    // std::cout << map_points3d[i].back().s << std::endl;
    // std::cout << map_points3d[i].back().heading << std::endl;
    auto map_points2d =
        Smooth::Instance().line_linear_interpolate(map_points3d[i]);
    PlanningRLMap::MapPoint2d smooth_map_points;
    // std::cout << "ori_ref_len:" << map_points2d.size() << std::endl;
    // std::cout << map_points2d[0].x << std::endl;
    // std::cout << map_points2d[0].y << std::endl;
    // std::cout << map_points2d[0].s << std::endl;
    // std::cout << map_points2d[0].heading << std::endl;
    // std::cout << map_points2d.back().x << std::endl;
    // std::cout << map_points2d.back().y << std::endl;
    // std::cout << map_points2d.back().s << std::endl;
    // std::cout << map_points2d.back().heading << std::endl;
    Smooth::Instance().smooth_path(map_points2d, smooth_map_points, 0.5, 0.2,
                                   0.000001);
    // std::cout << "smooth_ref_len:" << smooth_map_points.size() << std::endl;
    // std::cout << smooth_map_points[0].x << std::endl;
    // std::cout << smooth_map_points[0].y << std::endl;
    // std::cout << smooth_map_points[0].s << std::endl;
    // std::cout << smooth_map_points[0].heading << std::endl;
    // std::cout << smooth_map_points.back().x << std::endl;
    // std::cout << smooth_map_points.back().y << std::endl;
    // std::cout << smooth_map_points.back().s << std::endl;
    // std::cout << smooth_map_points.back().heading << std::endl;
    if (i == 1) {
      // center line cal s
      PlanningRLMap::CalculateMapPointsS(smooth_map_points);
    }
    new_map_points3d.push_back(smooth_map_points);
  }
  // std::cout << "new_map_points3d" << std::endl;
  // for (int i = 0; i < new_map_points3d.size(); i++) {
  //   for (int j = 0; j < new_map_points3d[i].size(); j++) {
  //     std::cout << i << std::endl;
  //     std::cout << j << std::endl;
  //     std::cout << new_map_points3d[i][j].x << std::endl;
  //     std::cout << new_map_points3d[i][j].y << std::endl;
  //     std::cout << new_map_points3d[i][j].s << std::endl;
  //   }
  // }

  return new_map_points3d;
}

std::vector<PlanningRLMap::MapPoint2d>
ContainerManager::BuildMapPointsByRoutingHolder(RoutingResultsHolder &holder) {
  std::vector<PlanningRLMap::MapPoint2d> map_points_3;
  Lane::ConstPtr lane_ptr =
      PlanningRLMap::Instance().FindLaneById(holder.lane_id);
  if (lane_ptr == nullptr) {
    LOG_INFO("Findlane error Error!");
    std::cout << "Findlane error Error" << std::endl;
    return map_points_3;
  }
  auto center_points = lane_ptr->center_line().points();
  auto left_points = lane_ptr->left_divider()->polyline().points();
  auto right_points = lane_ptr->right_divider()->polyline().points();
  auto local_center_points = PlanningRLMap::ConvertToMapPoints(center_points);
  auto local_left_points = PlanningRLMap::ConvertToMapPoints(left_points);
  auto local_right_points = PlanningRLMap::ConvertToMapPoints(right_points);

  local_left_points =
      Smooth::Instance().line_linear_interpolate(local_left_points);
  PlanningRLMap::MapPoint2d smooth_local_left_points;
  Smooth::Instance().smooth_path(local_left_points, smooth_local_left_points,
                                 0.5, 0.2, 0.000001);
  PlanningRLMap::MapPoint2d new_local_left_points =
      PlanningRLMap::SelectMapPointsBys(smooth_local_left_points,
                                        holder.start_s, holder.end_s);

  local_center_points =
      Smooth::Instance().line_linear_interpolate(local_center_points);
  // std::cout << "local_center_points" << std::endl;
  // std::cout << local_center_points.size() << std::endl;
  PlanningRLMap::MapPoint2d smooth_local_center_points;
  Smooth::Instance().smooth_path(
      local_center_points, smooth_local_center_points, 0.5, 0.2, 0.000001);
  // std::cout << "smooth_local_center_points" << std::endl;
  // std::cout << smooth_local_center_points.size() << std::endl;
  PlanningRLMap::MapPoint2d new_local_center_points =
      PlanningRLMap::SelectMapPointsBys(smooth_local_center_points,
                                        holder.start_s, holder.end_s);
  // std::cout << "new_local_center_points" << std::endl;
  // std::cout << new_local_center_points.size() << std::endl;
  // write lane id for road bound
  for (auto &lcp: new_local_center_points) {
    lcp.lane_id = holder.lane_id;
  }

  local_right_points =
      Smooth::Instance().line_linear_interpolate(local_right_points);
  PlanningRLMap::MapPoint2d smooth_local_right_points;
  Smooth::Instance().smooth_path(local_right_points, smooth_local_right_points,
                                 0.5, 0.2, 0.000001);
  PlanningRLMap::MapPoint2d new_local_right_points =
      PlanningRLMap::SelectMapPointsBys(smooth_local_right_points,
                                        holder.start_s, holder.end_s);
  // std::cout<<"After:
  // "<<local_center_points.size()<<","<<local_left_points.size()<<","<<local_right_points.size()<<std::endl;
  map_points_3.push_back(new_local_left_points);
  map_points_3.push_back(new_local_center_points);
  map_points_3.push_back(new_local_right_points);
  return map_points_3;
}

void ContainerManager::OdomPoseStampedMsgCallback(
    const PoseStampedShrPtr &message) {
  std::lock_guard<std::mutex> lock(msg_mtx_);
  auto timestamp_sec = message->header().timestamp_sec();
  // PRINT_DELAY_LOG(message, "[localization pose delay]", 0.01);
  odom_pose_msgs_.push_front(message);
  prev_odom_pose_msg_time_ = timestamp_sec;
  if (odom_pose_msgs_.size() > max_odom_pose_queue_size_) {
    odom_pose_msgs_.pop_back();
  }
}

void ContainerManager::UtmPoseStampedMsgCallback(
    const PoseStampedShrPtr &message) {
  std::lock_guard<std::mutex> lock(msg_mtx_);
  auto timestamp_sec = message->header().timestamp_sec();
  // PRINT_DELAY_LOG(message, "[localization pose delay]", 0.01);
  utm_pose_msgs_.push_front(message);
  prev_utm_pose_msg_time_ = timestamp_sec;
  if (utm_pose_msgs_.size() > max_utm_pose_queue_size_) {
    utm_pose_msgs_.pop_back();
  }
}

void ContainerManager::TwistStampedMsgCallback(
    const TwistStampedShrPtr &message) {
  std::lock_guard<std::mutex> lock(msg_mtx_);
  auto timestamp_sec = message->header().timestamp_sec();
  // PRINT_DELAY_LOG(message, "[localization pose delay]", 0.01);
  twist_msgs_.push_front(message);
  prev_twist_msg_time_ = timestamp_sec;
  if (twist_msgs_.size() > max_twist_queue_size_) {
    twist_msgs_.pop_back();
  }
}

void ContainerManager::TrafficLightDetectionMsgCallback(const TrafficLightDetectionShrPtr &message) {
  std::lock_guard<std::mutex> lock(msg_mtx_);
  auto timestamp_sec = message->header().timestamp_sec();
  traffic_light_msgs_.push_front(message);
  prev_traffic_light_msg_time_ = timestamp_sec;
  if (traffic_light_msgs_.size() > max_traffic_light_queue_size_) {
    traffic_light_msgs_.pop_back();
  }
  traffic_lights_ptr = std::make_shared<TrafficLights>(message);
}

void ContainerManager::TransformReflineToOdometry(
    const ContainerMessageShrPtr &container_msg) {
  auto &utm_pose = container_msg->utm_pose->pose();
  auto &odom_pose = container_msg->odom_pose->pose();
  auto ego_world_x = utm_pose.position().x();
  auto ego_world_y = utm_pose.position().y();
  auto ego_world_orientation = utm_pose.orientation();
  auto ego_world_theta =
      planning_rl::GetYawFromQuaternion(ego_world_orientation);
  auto ego_odo_x = odom_pose.position().x();
  auto ego_odo_y = odom_pose.position().y();
  auto ego_odo_orientation = odom_pose.orientation();
  auto ego_odo_theta = planning_rl::GetYawFromQuaternion(ego_odo_orientation);

  // reference line
  double t_x{0.}, t_y{0.}, t_h{0.}, t_x1{0.}, t_y1{0.}, t_h1{0.};
  auto pts = container_msg->reference_line_ptr->ref_points();
  for (auto &p : pts) {
    // std::cout << "road_bound_point" << std::endl;
    // std::cout << p.left_road_bound_point().x() << std::endl;
    // std::cout << p.left_road_bound_point().y() << std::endl;
    // std::cout << p.right_road_bound_point().x() << std::endl;
    // std::cout << p.right_road_bound_point().y() << std::endl;
    // std::cout << p.left_bound_point().x() << std::endl;
    // std::cout << p.left_bound_point().y() << std::endl;
    // std::cout << p.right_bound_point().x() << std::endl;
    // std::cout << p.right_bound_point().y() << std::endl;
    WorldCoordToVehicleCoord(ego_world_x, ego_world_y, ego_world_theta, p.x(),
                             p.y(), p.heading(), t_x, t_y, t_h);
    VehicleCoordToOdometryCoord(ego_odo_x, ego_odo_y, ego_odo_theta, t_x, t_y,
                                normalize_angle(t_h), t_x1, t_y1, t_h1);
    p.set_x(t_x1);
    p.set_y(t_y1);
    p.set_heading(normalize_angle(t_h1));

    WorldCoordToVehicleCoord(ego_world_x, ego_world_y, ego_world_theta, p.left_bound_point().x(),
                             p.left_bound_point().y(), p.heading(), t_x, t_y, t_h);
    VehicleCoordToOdometryCoord(ego_odo_x, ego_odo_y, ego_odo_theta, t_x, t_y,
                                normalize_angle(t_h), t_x1, t_y1, t_h1);
    p.set_left_bound_point({t_x1, t_y1});

    WorldCoordToVehicleCoord(ego_world_x, ego_world_y, ego_world_theta, p.right_bound_point().x(),
                             p.right_bound_point().y(), p.heading(), t_x, t_y, t_h);
    VehicleCoordToOdometryCoord(ego_odo_x, ego_odo_y, ego_odo_theta, t_x, t_y,
                                normalize_angle(t_h), t_x1, t_y1, t_h1);
    p.set_right_bound_point({t_x1, t_y1});

    WorldCoordToVehicleCoord(ego_world_x, ego_world_y, ego_world_theta, p.left_road_bound_point().x(),
                             p.left_road_bound_point().y(), p.heading(), t_x, t_y, t_h);
    VehicleCoordToOdometryCoord(ego_odo_x, ego_odo_y, ego_odo_theta, t_x, t_y,
                                normalize_angle(t_h), t_x1, t_y1, t_h1);
    p.set_left_road_bound_point({t_x1, t_y1});

    WorldCoordToVehicleCoord(ego_world_x, ego_world_y, ego_world_theta, p.right_road_bound_point().x(),
                             p.right_road_bound_point().y(), p.heading(), t_x, t_y, t_h);
    VehicleCoordToOdometryCoord(ego_odo_x, ego_odo_y, ego_odo_theta, t_x, t_y,
                                normalize_angle(t_h), t_x1, t_y1, t_h1);
    p.set_right_road_bound_point({t_x1, t_y1});
  }
}

}  // namespace planning_rl
}  // namespace neodrive
