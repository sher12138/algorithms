/// Define container manager class
#pragma once
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

// #include "common/macro.h"
#include "common/math/smooth.h"
#include "common/planning_map/planning_map.h"
#include "common/reference_line/reference_line.h"
#include "common/reference_line/reference_point.h"
#include "common/utils/cout_sequence.h"
#include "common/utils/obs_utils.h"
#include "common/utils/time_logger.h"
#include "common/utils/cout_sequence.h"
#include "config/planning_rl_config.h"
#include "cyber/cyber.h"
#include "frame/agent.h"
#include "frame/ego.h"
#include "frame/frame.h"
#include "frame/frames.h"
#include "frame/traffic_light.h"
#include "global_adc_status.pb.h"
#include "localization_dead_reckoning.pb.h"
#include "localization_pose.pb.h"
#include "neolix_log.h"
#include "perception_obstacle.pb.h"
#include "planning.pb.h"
#include "routing.pb.h"
#include "traffic_light_detection.pb.h"

namespace neodrive {
namespace planning_rl {

using neodrive::cyber::Reader;
// LocalizationEstimate
using neodrive::global::localization::LocalizationEstimate;
using LocalizationEstimateShrPtr = std::shared_ptr<LocalizationEstimate>;
using neodrive::global::localization_dr::DRResult;
using DRResultShrPtr = std::shared_ptr<DRResult>;
// PerceptionObstacles
using neodrive::global::perception::PerceptionObstacles;
using PerceptionObstaclesShrPtr = std::shared_ptr<PerceptionObstacles>;
// ADCTrajectory
using neodrive::global::planning::ADCTrajectory;
using ADCTrajectoryShrPtr = std::shared_ptr<ADCTrajectory>;
using neodrive::global::planning::ADCTrajectoryPoint;
// routing
using neodrive::global::routing::RoutingResult;
using RoutingResultShrPtr = std::shared_ptr<RoutingResult>;
using neodrive::global::routing::RoutingRequest;
using RoutingRequestShrPtr = std::shared_ptr<RoutingRequest>;
using neodrive::global::routing::RoutingResult_LaneSegment;
using neodrive::planning_rl::Frame;
using neodrive::planning_rl::Frames;
using FramesShrPtr = std::shared_ptr<Frames>;
// world model
using neodrive::global::common::PoseStamped;
using PoseStampedShrPtr = std::shared_ptr<PoseStamped>;
using neodrive::global::common::TwistStamped;
using TwistStampedShrPtr = std::shared_ptr<TwistStamped>;
using neodrive::global::perception::traffic_light::TrafficLight;
using neodrive::global::perception::traffic_light::TrafficLightDetection;
using TrafficLightDetectionShrPtr = std::shared_ptr<TrafficLightDetection>;

struct RoutingResultsHolder {
  std::string lane_id = "";
  double start_s = -10000.0;
  double end_s = -10000.0;
};

using RoutingResultsHolder1d = std::vector<RoutingResultsHolder>;

// chassis
using neodrive::global::status::Chassis;
using ChassisShrPtr = std::shared_ptr<Chassis>;

#define PRINT_DELAY_LOG(message, flag, interval)                         \
  {                                                                      \
    static uint32_t prev_sequence_num = 0;                               \
    auto timestamp_sec = message->header().timestamp_sec();              \
    auto curr_time = cyber::Time::Now().ToSecond();                      \
    auto delay_sec = curr_time - timestamp_sec;                          \
    auto diff_sequence_num =                                             \
        message->header().sequence_num() - prev_sequence_num;            \
    if (delay_sec > interval || diff_sequence_num > 1) {                 \
      LOG_WARN(                                                          \
          "{} curr_time: {:.4f} msg time: {:.4f} delay: {:.4f} "         \
          "sequence_num: {}",                                            \
          flag, curr_time, timestamp_sec, delay_sec,                     \
          message->header().sequence_num());                             \
      if (diff_sequence_num > 1) {                                       \
        LOG_WARN("{} missing {} messages", flag, diff_sequence_num - 1); \
      }                                                                  \
    }                                                                    \
    prev_sequence_num = message->header().sequence_num();                \
  }

struct ContainerMessage {
  double timestamp{0.};
  uint32_t sequence_num{0};
  int scene_status{0};
  // ADCTrajectoryShrPtr ego_trajectory{nullptr};
  LocalizationEstimateShrPtr localization_pose{nullptr};
  PerceptionObstaclesShrPtr perception_obstacles{nullptr};
  FramesShrPtr frames = std::make_shared<Frames>();
  ReferenceLinePtr reference_line_ptr{nullptr};
  // ChassisShrPtr chassis{nullptr};
  PoseStampedShrPtr odom_pose{nullptr};
  PoseStampedShrPtr utm_pose{nullptr};
  TwistStampedShrPtr twist{nullptr};
  TrafficLightDetectionShrPtr traffic_light_detect{nullptr};
  TrafficLightsShrPtr traffic_lights_ptr{nullptr};
};

using ContainerMessageShrPtr = std::shared_ptr<ContainerMessage>;

/// Manage all containers
class ContainerManager {
  DECLARE_SINGLETON(ContainerManager);

 public:
  /// Container manager initialization
  bool Init();
  /// Get container msg
  bool HasNewMsg();
  bool GetContainerMsg(const size_t max_process_msgs_per_interval,
                       ContainerMessageShrPtr& msg);

 private:
  /// Constructor
  void PerceptionObstaclesMsgCallback(const PerceptionObstaclesShrPtr& message);
  void ADCTrajectoryMsgCallback(const ADCTrajectoryShrPtr& message);
  void LocalizationObstaclesMsgCallback(
      const LocalizationEstimateShrPtr& message);
  void RoutingMsgCallback(const RoutingResultShrPtr& message);
  // void ChassisMsgCallback(const ChassisShrPtr& message);
  void OdomPoseStampedMsgCallback(const PoseStampedShrPtr& message);
  void UtmPoseStampedMsgCallback(const PoseStampedShrPtr& message);
  void TwistStampedMsgCallback(const TwistStampedShrPtr& message);
  void TrafficLightDetectionMsgCallback(const TrafficLightDetectionShrPtr& message);
  PerceptionObstaclesShrPtr FetchPerceptionObstacleMsgs(
      const size_t max_process_msgs_per_interval);
  void ResetContainer(const double timestamp_sec);
  void ResetMsgs();
  void FetchNewADCTrajectoryMsgs();
  void FetchNewPerceptionObstaclesMsgs();

  ADCTrajectoryShrPtr FindMatchedEgoTrajectoryMsg(const double timestamp_sec);
  LocalizationEstimateShrPtr FindMatchedLocalizationPoseMsg(
      const double timestamp_sec);
  ChassisShrPtr FindMatchedChassisMsg(const double timestamp_sec);
  PoseStampedShrPtr GetOdomPoseMsg();
  PoseStampedShrPtr GetUtmPoseMsg();
  TwistStampedShrPtr GetTwistMsg();
  TrafficLightDetectionShrPtr GetTrafficLightMsg();
  bool CheckoutRoutingMsg(const RoutingResult& routing);
  bool BuildReferenceLineByRoutingMsg(const RoutingResult& routing);
  bool GetLaneSegmentByRouting(const RoutingResult& routing,
                               std::vector<RoutingResult_LaneSegment>& lanes);
  PlanningRLMap::MapPoint3d BuildMapPointsByRoutingHolder(
      RoutingResultsHolder& holder);
  PlanningRLMap::MapPoint3d BuildMapPointsByRoutingHolders();
  bool BuildReferenceLineByRoutingHolders();
  void GetFrameData(PerceptionObstaclesShrPtr& perception_obstacles_msg,
                    LocalizationEstimateShrPtr& matched_localization_pose_msg,
                    double& timestamp_sec, ContainerMessageShrPtr& msg);
  void GetFrameDataOdom(PerceptionObstaclesShrPtr& perception_obstacles_msg,
                        PoseStampedShrPtr& odom_pose_msg,
                        TwistStampedShrPtr& twist_msg, double& timestamp_sec,
                        ContainerMessageShrPtr& msg);
  void TransformReflineToOdometry(const ContainerMessageShrPtr& container_msg);

 private:
  static constexpr double kInvalidPosition = 0.1;
  bool initialized_{false};
  bool reset_msg_{false};
  uint32_t prev_routing_sequence_num_{0};
  std::mutex msg_mtx_;
  std::shared_ptr<neodrive::cyber::Node> node_{nullptr};

  std::shared_ptr<Reader<PerceptionObstacles>> perception_obstacles_reader_;
  std::shared_ptr<Reader<ADCTrajectory>> ego_trajectory_reader_;
  std::shared_ptr<Reader<LocalizationEstimate>>
      localization_pose_obstacles_reader_;
  std::shared_ptr<Reader<Chassis>> chassis_reader_;
  std::shared_ptr<Reader<RoutingResult>> routing_reader_;
  std::shared_ptr<Reader<PoseStamped>> odometry_pose_reader_;
  std::shared_ptr<Reader<PoseStamped>> utm_pose_reader_;
  std::shared_ptr<Reader<TwistStamped>> twist_reader_;
  std::shared_ptr<Reader<TrafficLightDetection>> traffic_light_reader_;

  std::deque<ADCTrajectoryShrPtr> ego_trajectory_msgs_;
  std::deque<PerceptionObstaclesShrPtr> perception_obstacles_msgs_;
  std::deque<LocalizationEstimateShrPtr> localization_pose_msgs_;
  std::deque<ChassisShrPtr> chassis_msgs_;
  std::deque<PoseStampedShrPtr> odom_pose_msgs_;
  std::deque<PoseStampedShrPtr> utm_pose_msgs_;
  std::deque<TwistStampedShrPtr> twist_msgs_;
  std::deque<TrafficLightDetectionShrPtr> traffic_light_msgs_;

  RoutingResultShrPtr routing_msg_;
  RoutingRequest last_routing_request_;
  RoutingResultsHolder1d routing_holder_1d_;

  double prev_perception_obstacles_msg_time_{0.};
  double prev_localization_pose_msg_time_{0.};
  double prev_routing_msg_time_{0.};
  double prev_new_scene_routing_msg_time_{0.};
  double prev_new_scene_perception_msg_time_{0.};
  double prev_chassis_msg_time_{0.};
  double prev_odom_pose_msg_time_{0.};
  double prev_utm_pose_msg_time_{0.};
  double prev_twist_msg_time_{0.};
  double prev_traffic_light_msg_time_{0.};
  size_t max_localization_queue_size_{1000};
  size_t max_perception_obstacle_queue_size_{100};
  size_t max_odom_pose_queue_size_{1000};
  size_t max_utm_pose_queue_size_{1000};
  size_t max_twist_queue_size_{1000};
  size_t max_traffic_light_queue_size_{1000};
  ReferenceLinePtr reference_line_ptr{nullptr};
  TrafficLightsShrPtr traffic_lights_ptr{nullptr};

  // PlanningRLMap* map_{nullptr};
  utils::TimeLogger time_log_{"planning_rl_container"};
};

}  // namespace planning_rl
}  // namespace neodrive
