#include "planning_manager.h"

#include <time.h>

#include "common/math/util.h"
#include "common/util/time_logger.h"
#include "planning_manager.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/data_center/em_planning_data.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/common/vehicle_param.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/math/common/obstacle_frame_container.h"
#include "src/planning/planning_map/planning_map.h"
#include "src/planning/reference_line/reference_line_util.h"
#include "src/planning/scenario_manager/scenario_manager.h"
#include "src/planning/util/freespace_lane_bound_fitter.h"
#include "src/planning/util/kinematic_predict_check_utm_pose.h"
#include "src/planning/util/speed_planner_common.h"
#include "src/planning/util/util.h"
#include "src/planning/util/visual_lane_check_utm_heading.h"
#include "src/planning/util/visual_lane_combine_ego_lane.h"

namespace neodrive {
namespace planning {
using neodrive::global::localization::ImuMsgDelayStatus_Name;
using neodrive::global::localization::ImuMsgMissingStatus_Name;
using neodrive::global::localization::LocalLidarQuality_Name;
using neodrive::global::localization::LocalLidarStatus_Name;
using neodrive::global::localization::MsfSecurityLevel_Name;
using neodrive::global::perception::traffic_light::TrafficLight;
using neodrive::global::perception::traffic_light::TrafficLightDetection;
using neodrive::global::planning::ADCPathPoint;
using neodrive::global::planning::ADCTrajectory;
using neodrive::global::planning::ADCTrajectoryPoint;
using neodrive::global::planning::DecisionResult;
using neodrive::global::planning::PilotState;
using neodrive::global::routing::RoutingResult_ErrorCode;
using neodrive::global::status::DrivingMode_Name;
using neodrive::global::status::GearPosition_Name;
using neodrive::global::status::State_Name;
using neodrive::global::status::StopReason_Name;

PlanningManager::PlanningManager() {}

bool PlanningManager::Init(const std::shared_ptr<Node> &node) {
  if (initialized_) {
    return true;
  }
  auto drive_strategy = neodrive::common::config::GetDriveStrategy();
  auto drive_strategy_type =
      neodrive::common::config::GetDriveStrategyType(drive_strategy);
  if (drive_strategy_type ==
      neodrive::common::config::DriveStrategyType::ERROR) {
    LOG_ERROR("GetDriveStrategyType::ERROR");
    return false;
  }
  node_ = node;
  if (!data_center_->Init(node)) {
    LOG_ERROR("DataCenter init failed!");
    return false;
  }
  DataCenter::Instance();
  PlanningMap::Instance();
  VehicleParam::Instance();
  if (!(ScenarioManager::Instance()->Init())) {
    LOG_ERROR("ScenarioManager init failed");
    return false;
  }
  if (!observer_->Init(node)) {
    LOG_ERROR("Observer init failed");
    return false;
  }
  obstacle_map_ = obsmap::ObstacleMap::Instance();

  if (!publisher_->Init(node)) {
    LOG_ERROR("Publisher init failed");
    return false;
  }

  if (!rerouting_trigger_.Init(
          0, FLAGS_planning_change_lane_trigger_rerouting_conts)) {
    LOG_ERROR("init rerouting_trigger failed!");
    return false;
  }
  auto scenario_manager = ScenarioManager::Instance();
  if (!scenario_manager->Init()) {
    LOG_ERROR("init scenario_manager failed!");
    return false;
  }

  new_human_interface_decider_ = dynamic_cast<HumanInterfaceDecider *>(
      scenario_manager->GetDecider("HumanInterfaceDecider"));
  station_stop_decider_ = dynamic_cast<StationStopDecider *>(
      scenario_manager->GetDecider("StationStopDecider"));
  // obstacles_intention_decider_ = dynamic_cast<ObstaclesIntentionDecider *>(
  //     scenario_manager->GetDecider("ObstaclesIntentionDecider"));
  pilot_state_decider_ptr_ = dynamic_cast<PilotStateDecider *>(
      scenario_manager->GetDecider("PilotStateDecider"));

  CHECK_NOTNULL(new_human_interface_decider_);
  CHECK_NOTNULL(station_stop_decider_);
  CHECK_NOTNULL(pilot_state_decider_ptr_);
  // CHECK_NOTNULL(obstacles_intention_decider_);

  planning_steps_.push_back(
      std::make_pair<std::string, ErrorCode (PlanningManager::*)()>(
          "data_preprocess", &PlanningManager::DataPreprocess));
  planning_steps_.push_back(
      std::pair<std::string, ErrorCode (PlanningManager::*)()>(
          "behavior_planning", &PlanningManager::BehaviorPlanning));
  planning_steps_.push_back(
      std::pair<std::string, ErrorCode (PlanningManager::*)()>(
          "motion_planning", &PlanningManager::MotionPlanning));
  planning_steps_.push_back(
      std::pair<std::string, ErrorCode (PlanningManager::*)()>(
          "data_postprocess", &PlanningManager::DataPostprocess));

  InitAdcLocalPolygonCheckCloserObs();

  LOG_INFO("PlanningManager is ready.");
  initialized_ = true;
  return true;
}

ErrorCode PlanningManager::RunOnce() {
  observer_->Observe();
  logger_.ResetStartTime();
  publisher_->Reset();
  skip_motion_plan_ = false;
  ErrorCode ret = ErrorCode::PLANNING_OK;
  for (auto &step : planning_steps_) {
    auto key_str = step.first;
    auto func_ptr = step.second;
    LOG_INFO("start exec :{}", key_str);
    ret = (this->*func_ptr)();
    LOG_INFO("end exec :{}", key_str);
    if (ret != ErrorCode::PLANNING_OK) {
      LOG_ERROR("Execute {} failed!", key_str);
      break;
    }
    logger_.RegisterTime(key_str);
  }
  SaveMonitorMessage();
  publisher_->PublishMessages();
  if (ret == ErrorCode::PLANNING_OK ||
      ret == ErrorCode::PLANNING_SKIP_REST_TASKS) {
    data_center_->SaveFrame();
    ++sequence_num_;
  } else {
    LOG_ERROR("RunOnce failed");
  }
  if (!publisher_->planning_trajectory_published()) {
    publisher_->PublishFakeTrajectory(sequence_num_);
  }
  logger_.RegisterTimeAndPrint("planning_finished");
  return ret;
}

void PlanningManager::ExecuteDataPreprocessFail() {
  if (data_center_->planning_interface_msg.is_available) {
    auto &open_api_cmd = *(data_center_->planning_interface_msg.ptr);
    if (open_api_cmd.state() == neodrive::global::status::FINISH) {
      LOG_INFO("set finish according to openapi command");
      data_center_->mutable_global_state_proxy()->SetFinish(true);
      data_center_->mutable_routing_result()->is_available = false;
      data_center_->planning_interface_msg.is_available = false;
      data_center_->set_parking_ptr(nullptr);
      data_center_->GetNavigationSwapContext().parking_space_ptr.Set(nullptr);
    } else if (open_api_cmd.state() == neodrive::global::status::WAIT &&
               !data_center_->global_state_proxy().is_wait()) {
      LOG_INFO("set wait according to openapi command");
      data_center_->mutable_global_state_proxy()->set_wait();
    }
  }
  // stuck in data_preprocess
  if (data_center_->is_auto_driving() && data_center_->have_task()) {
    data_center_->mutable_event_report_proxy()->SetEvent(
        EventType::PREPROCESS_FAIL);
  }
}

ErrorCode PlanningManager::DataPreprocess() {
  ErrorCode ret = ErrorCode::PLANNING_OK;
  do {
    ret = data_center_->InitFrame(sequence_num_);
    if (ret != ErrorCode::PLANNING_OK) {
      LOG_WARN("init_frame failed.");
      break;
    }
    // 1. data_center check
    if (data_center_->current_frame() == nullptr) {
      LOG_ERROR("current_frame is nullptr");
      ret = ErrorCode::PLANNING_ERROR_FAILED;
      break;
    }

    // compute max speed with drive_strategy
    auto max_speed = DriveStrategyMaxSpeed();
    data_center_->set_drive_strategy_max_speed(max_speed);
    data_center_->mutable_behavior_speed_limits()->set_drive_strategy_max_speed(
        max_speed);
    data_center_->set_is_in_port_odd(PlanningMap::Instance()->IsInPortOdd(
        Vec2d{data_center_->vehicle_state_utm().X(),
              data_center_->vehicle_state_utm().Y()}));

    // compute enable prediction with drive stategy
    data_center_->set_enable_prediction(DriveStrategyEnablePrediction());

    // 1. decide pilot state according to discode.
    pilot_state_decider_ptr_->UpdatePilotState();
    // 2. check skip planning
    ret = CheckWhetherSkipPlanning();
    if (ret == ErrorCode::PLANNING_SKIP_REST_TASKS) {
      skip_motion_plan_ = true;
      break;
    }
    // 3. transform utm reference line to odometry
    ret = TransformToOdometry();
    if (ret != ErrorCode::PLANNING_OK) {
      skip_motion_plan_ = true;
      break;
    }
    // 4. stitch trajectory
    ret = StitchTrajectory();
    if (ret != ErrorCode::PLANNING_OK) {
      skip_motion_plan_ = true;
      break;
    }
    // 5. init task info list
    LOG_INFO("start InitTaskInfoList");
    ret = InitTaskInfoList();
    LOG_INFO("end InitTaskInfoList");
    if (ret != ErrorCode::PLANNING_OK) {
      skip_motion_plan_ = true;
      break;
    }
  } while (false);

  if (ret != ErrorCode::PLANNING_OK) {
    ExecuteDataPreprocessFail();
  } else {
    data_center_->mutable_event_report_proxy()->EventReset(
        EventType::PREPROCESS_FAIL);
  }
  return ret;
}

ErrorCode PlanningManager::BehaviorPlanning() {
  // 7. behavior make decisions
  auto &task_info_list = *data_center_->mutable_task_info_list();
  auto &task_info = task_info_list.front();
  auto ret = ScenarioManager::Instance()->ExecuteBehaviorTasks(task_info);
  if (ret == ErrorCode::PLANNING_SKIP_REST_TASKS) {
    skip_motion_plan_ = true;
    auto planning_data_ptr = task_info.current_frame()->planning_data_ptr();
    data_center_->current_frame()->set_planning_data(planning_data_ptr);
    return ret;
  }
  return ErrorCode::PLANNING_OK;
}

ErrorCode PlanningManager::MotionPlanning() {
  auto &task_info_list = *data_center_->mutable_task_info_list();
  // 13. motion planning
  for (auto &task_info : task_info_list) {
    task_info.set_error_code(ExecuteMotionTasks(task_info));
  }
  return ErrorCode::PLANNING_OK;
}

ErrorCode PlanningManager::DataPostprocess() {
  auto &task_info_list = *data_center_->mutable_task_info_list();
  auto ret = PostProcessPlanningResult(task_info_list);
  if (ret != ErrorCode::PLANNING_OK) {
    LOG_ERROR("PostProcessPlanningResult failed");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  // stuck due to speed plan failure
  if (data_center_->is_auto_driving() &&
      data_center_->current_frame()->outside_planner_data().speed_fail_tasks >=
          2) {
    data_center_->mutable_event_report_proxy()->SetEvent(
        EventType::SPEED_PLAN_FAIL);
  } else {
    data_center_->mutable_event_report_proxy()->EventReset(
        EventType::SPEED_PLAN_FAIL);
  }
  // 14. post process
  Frame *frame = data_center_->current_frame();

  bool pub_flag = publisher_->PublishPlanningResult(stitching_trajectory_,
                                                    frame, has_closer_obs_);
  if (pub_flag == false) {
    LOG_ERROR("Failed to publish the result.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

double PlanningManager::DriveStrategyMaxSpeed() {
  const auto &drive_config =
      common::config::CommonConfig::Instance()->drive_strategy_config();
  if (!drive_config.enable_motorway) {
    return drive_config.non_motorway.max_cruise_speed;
  }
  if (data_center_->last_frame() != nullptr) {
    return (data_center_->last_frame()
                    ->outside_planner_data()
                    .init_point_ref_point.lane_type_is_biking()
                ? drive_config.non_motorway.max_cruise_speed
                : drive_config.motor_way.max_cruise_speed);
  }
  return std::min(drive_config.motor_way.max_cruise_speed,
                  drive_config.non_motorway.max_cruise_speed);
}

bool PlanningManager::DriveStrategyEnablePrediction() {
  const auto &drive_config =
      common::config::CommonConfig::Instance()->drive_strategy_config();
  if (!drive_config.enable_motorway) {
    return drive_config.non_motorway.enable_prediction;
  }
  if (data_center_->last_frame() != nullptr) {
    return (data_center_->last_frame()
                    ->outside_planner_data()
                    .init_point_ref_point.lane_type_is_biking()
                ? drive_config.non_motorway.enable_prediction
                : drive_config.motor_way.enable_prediction);
  }
  return false;
}

void VisRefPoints(const std::vector<ReferencePoint> &points,
                  const std::string &name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kUtm);
  event->mutable_color()->set_r(1);
  event->mutable_color()->set_g(0);
  event->mutable_color()->set_b(0);
  event->mutable_color()->set_a(0.8);

  auto set_pts = [](auto event, auto &pts) {
    for (auto &pt : pts) {
      auto sphere = event->mutable_sphere()->Add();
      sphere->mutable_center()->set_x(pt.x());
      sphere->mutable_center()->set_y(pt.y());
      sphere->mutable_center()->set_z(0);
      sphere->set_radius(0.1);
    }
  };

  set_pts(event, points);
}

ErrorCode PlanningManager::TransformToOdometry() {
  auto &utm_pos = data_center_->environment().vehicle_state_proxy();
  auto &odom_pos = data_center_->environment().vehicle_state_odometry_proxy();
  auto &navigation_swap_context = data_center_->GetNavigationSwapContext();
  // save navigation result
  data_center_->set_navigation_result(
      navigation_swap_context.navigation_result.Get());
  const auto &navigation_result = data_center_->navigation_result();
  current_utm_ref_ = navigation_result.current_utm_ref;
  // Generate current odom reference line
  double t_x{0.}, t_y{0.}, t_h{0.}, t_x1{0.}, t_y1{0.}, t_h1{0.};
  if (navigation_result.current_utm_ref == nullptr ||
      navigation_result.current_utm_ref->ref_points().empty()) {
    LOG_INFO("no navigation ref line, skip TransformToOdometry");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  auto pts = current_utm_ref_->ref_points();
  for (auto &p : pts) {
    earth2vehicle(utm_pos.X(), utm_pos.Y(), utm_pos.Heading(), p.x(), p.y(),
                  p.heading(), t_x, t_y, t_h);
    vehicle2earth(odom_pos.X(), odom_pos.Y(), odom_pos.Heading(), t_x, t_y,
                  normalize_angle(t_h), t_x1, t_y1, t_h1);
    p.set_x(t_x1);
    p.set_y(t_y1);
    p.set_heading(normalize_angle(t_h1));
  }
  LOG_ERROR("enter createfrom");
  ReferenceLinePtr ret = std::make_shared<ReferenceLine>();
  ret->CreateFrom(pts,
                  std::array{current_utm_ref_->crosswalk_overlaps(),
                             current_utm_ref_->signal_overlaps(),
                             current_utm_ref_->yield_sign_overlaps(),
                             current_utm_ref_->stop_sign_overlaps(),
                             current_utm_ref_->junction_overlaps(),
                             current_utm_ref_->speed_bump_overlaps(),
                             current_utm_ref_->clearzone_overlaps(),
                             current_utm_ref_->geo_fence_overlaps(),
                             current_utm_ref_->barrier_gate_overlaps(),
                             current_utm_ref_->parking_space_overlaps()},
                  Vec2d{odom_pos.X(), odom_pos.Y()},
                  current_utm_ref_->anchor_s(),
                  current_utm_ref_->routing_sequence_num());
  current_odom_ref_ = ret;
  data_center_->set_have_task(true);
  VisRefPoints(pts, "origin_ref");

  // auto &vehicle_state = data_center_->vehicle_state_proxy();
  // SLPoint sl_pt;
  // if (current_odom_ref_->GetPointInFrenetFrameWithHeading(
  //         {vehicle_state.X(), vehicle_state.Y()}, vehicle_state.Heading(),
  //         &sl_pt)) {
  //   LOG_INFO("print odom reference line info");
  //   for (auto &pt : current_odom_ref_->ref_points()) {
  //     if (pt.s() < sl_pt.s()) continue;
  //     if (pt.s() > sl_pt.s() + 100.0) break;
  //     LOG_INFO(
  //         "s, x, y:{:.4f}, {:.4f}, {:.4f}, lane id:{}, left lane bound:{}, "
  //         "left road bound:{}, left bound:{}, right lane bound:{}, right road
  //         " "bound:{}, right bound:{}", pt.s(), pt.x(), pt.y(),
  //         cyberverse::HDMap::Instance()->GetIdHashString(pt.hd_map_lane_id()),
  //         pt.left_lane_bound(), pt.left_road_bound(), pt.left_bound(),
  //         pt.right_lane_bound(), pt.right_road_bound(), pt.right_bound());
  //   }
  // }
  // Generate target odom reference line
  if (navigation_result.target_utm_ref != nullptr &&
      !navigation_result.target_utm_ref->ref_points().empty()) {
    auto target_utm_ref = navigation_result.target_utm_ref;
    auto pts = target_utm_ref->ref_points();
    for (auto &p : pts) {
      earth2vehicle(utm_pos.X(), utm_pos.Y(), utm_pos.Heading(), p.x(), p.y(),
                    p.heading(), t_x, t_y, t_h);
      vehicle2earth(odom_pos.X(), odom_pos.Y(), odom_pos.Heading(), t_x, t_y,
                    normalize_angle(t_h), t_x1, t_y1, t_h1);
      p.set_x(t_x1);
      p.set_y(t_y1);
      p.set_heading(normalize_angle(t_h1));
    }
    LOG_ERROR("enter createfrom");
    ReferenceLinePtr tar_ret = std::make_shared<ReferenceLine>();
    tar_ret->CreateFrom(pts,
                        std::array{target_utm_ref->crosswalk_overlaps(),
                                   target_utm_ref->signal_overlaps(),
                                   target_utm_ref->yield_sign_overlaps(),
                                   target_utm_ref->stop_sign_overlaps(),
                                   target_utm_ref->junction_overlaps(),
                                   target_utm_ref->speed_bump_overlaps(),
                                   target_utm_ref->clearzone_overlaps(),
                                   target_utm_ref->geo_fence_overlaps(),
                                   target_utm_ref->barrier_gate_overlaps(),
                                   target_utm_ref->parking_space_overlaps()},
                        Vec2d{odom_pos.X(), odom_pos.Y()},
                        target_utm_ref->anchor_s(),
                        target_utm_ref->routing_sequence_num());
    data_center_->set_target_odom_ref(tar_ret);
    VisRefPoints(pts, "lane_change_ref");
  } else {
    LOG_INFO("set target_odom_ref nullptr");
    data_center_->set_target_odom_ref(nullptr);
  }

  return ErrorCode::PLANNING_OK;
}

ErrorCode PlanningManager::InitTaskInfoList() {
  auto &task_info_list = *data_center_->mutable_task_info_list();
  task_info_list.clear();

  // init planning_data
  auto &navigation_swap_context = data_center_->GetNavigationSwapContext();
  task_info_list.emplace_back(data_center_->CreateShadowFrame(),
                              data_center_->last_frame(), current_utm_ref_,
                              current_odom_ref_);
  LOG_INFO("utm addr:{}, odom addr:{}", (void *)current_utm_ref_.get(),
           (void *)current_odom_ref_.get());
  auto master_info = data_center_->mutable_master_info();
  master_info->set_destination_point(navigation_swap_context.dest_point.Get());
  master_info->set_origin_destination_point(
      navigation_swap_context.dest_point.Get());
  master_info->set_distance_to_end(navigation_swap_context.dist_to_end.Get());
  master_info->set_drive_direction(
      navigation_swap_context.driving_direction.Get());
  if (navigation_swap_context.parking_space_ptr.IsUpdated() &&
      data_center_->master_info().curr_stage() != "PARKING_OUT") {
    data_center_->set_parking_ptr(
        navigation_swap_context.parking_space_ptr.Get());
    if (navigation_swap_context.parking_space_ptr.Get() != nullptr) {
      auto hdmap = cyberverse::HDMap::Instance();
      std::string parking_id = hdmap->GetIdHashString(
          navigation_swap_context.parking_space_ptr.Get()->OriginPark()->Id());
      LOG_INFO("data center set parking ptr id = {}", parking_id);
    }
  }

  auto &task_info = task_info_list.back();
  // check reference line valid.
  if (task_info.current_frame() == nullptr ||
      task_info.reference_line() == nullptr ||
      task_info.reference_line()->ref_points().size() == 0) {
    LOG_ERROR("Empty current_frame or reference line.");
    return ErrorCode::PLANNING_ERROR_NAN;
  }

  // init planning data.
  auto &frame = task_info_list.back().current_frame();
  std::unique_ptr<PlanningData> planning_data_ptr =
      std::make_unique<EMPlanningData>();
  frame->set_planning_data(planning_data_ptr);
  auto planning_data = frame->mutable_planning_data();
  planning_data->set_init_planning_point(stitching_trajectory_.back());

  if (task_info_list.empty()) {
    LOG_ERROR("task_info_list is empty");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  /// add remote_interaction_context for AD-Light
  UpdateRemoteReactionContext(task_info_list.front().reference_line());

  const auto &point = task_info_list.front()
                          .current_frame()
                          ->planning_data()
                          .init_planning_point();
  LOG_INFO(
      "init_planning_point_info: relative_time, x, y, theta, kappa, "
      "dkappa, "
      "s, v, a, j: {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, "
      "{:.3f}, {:.3f}, {:.3f}",
      point.relative_time(), point.x(), point.y(), point.theta(), point.kappa(),
      point.dkappa(), point.s(), point.velocity(), point.acceleration(),
      point.jerk());

  // init common util
  if (!UpdateTaskInfoUtilsInfo(task_info_list.front())) {
    LOG_ERROR("UpdateTaskInfoUtilsInfo failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  PlanningMap::Instance()->GetConflictZone(
      task_info_list.front(), Vec2d(data_center_->vehicle_state_utm().X(),
                                    data_center_->vehicle_state_utm().Y()));

  // init decision data
  const DecisionData *last_decision_data = nullptr;
  if (data_center_->last_frame() != nullptr) {
    last_decision_data =
        &(data_center_->last_frame()->planning_data().decision_data());
  }
  for (auto &task : task_info_list) {
    std::shared_ptr<DecisionData> decision_data =
        std::make_shared<DecisionData>(
            *(data_center_->mutable_object_table()),
            data_center_->environment().perception_proxy().GetObstacleIds(),
            task.reference_line(), last_decision_data);
    task.set_decision_data(decision_data);
  }

  // combine perception lane: only ego lane
  // VisualLaneCombineEgoLane{}.Combine(task_info_list.front());
  VisualLaneCombineEgoLane{}.SetCurbLines(task_info_list.front());

  // Freespace lane bound fitting
  FreespaceLaneBoundFitter fitter{task_info_list.front().reference_line(),
                                  *data_center_->lidar_freespace_msg.ptr,
                                  data_center_->camera_segments()};
  data_center_->set_cartesian_upper_bound(fitter.xy_upper_pts());
  data_center_->set_cartesian_lower_bound(fitter.xy_lower_pts());
  data_center_->set_frenet_upper_bound(fitter.sl_upper_pts());
  data_center_->set_frenet_lower_bound(fitter.sl_lower_pts());

  // send traffic light info from hd map for traffic light detection
  publisher_->PublishTrafficLightData(task_info_list.front());
  return ErrorCode::PLANNING_OK;
}

void PlanningManager::UpdateRemoteReactionContext(
    const ReferenceLinePtr ref_ptr) {
  if (data_center_->vehicle_state_proxy().chassis().current_vcu_mode() !=
      neodrive::global::status::VcuControlMode::VCU_PDU) {
    return;
  }

  auto remote_interaction_context = data_center_->mutable_global_state_proxy()
                                        ->mutable_global_state()
                                        ->mutable_remote_interaction_context();
  ReferencePoint ref_pt{};
  bool find_nearest_pt = ref_ptr->GetNearestRefPointWithHeading(
      {data_center_->vehicle_state_odometry().X(),
       data_center_->vehicle_state_odometry().Y()},
      data_center_->vehicle_state_odometry().Heading(), &ref_pt);
  if (!find_nearest_pt) {
    remote_interaction_context->set_is_pnc_ok(false);
    remote_interaction_context->set_valid_pose(false);
    remote_interaction_context->set_valid_theta(false);
    return;
  }

  double x = data_center_->vehicle_state_utm().X();
  double y = data_center_->vehicle_state_utm().Y();
  auto hdmap = cyberverse::HDMap::Instance();
  std::vector<cyberverse::LaneInfoConstPtr> lanes;
  hdmap->GetLanes({x, y}, 5.0, &lanes);
  if (!lanes.empty() && lanes[0]->IsOnLane({x, y})) return;

  bool theta_valid =
      std::abs(normalize_angle(
          ref_pt.heading() -
          data_center_->vehicle_state_odometry().Heading())) < M_PI_2;

  std::size_t start_index{0}, end_index{0};
  if (!ref_ptr->GetStartEndIndexBySLength(
          ref_pt.s() - 1.5 * VehicleParam::Instance()->back_edge_to_center(),
          VehicleParam::Instance()->length() * 1.5, &start_index, &end_index)) {
    LOG_WARN("find start/end index failed.");
    return;
  }
  std::vector<Vec2d> left_pts{}, right_pts{};
  for (std::size_t i = start_index; i <= end_index; ++i) {
    Vec2d pt{};
    ref_ptr->GetPointInCartesianFrame(
        {ref_ptr->ref_points()[i].s(),
         ref_ptr->ref_points()[i].left_road_bound()},
        &pt);
    left_pts.emplace_back(pt);
    ref_ptr->GetPointInCartesianFrame(
        {ref_ptr->ref_points()[i].s(),
         -ref_ptr->ref_points()[i].right_road_bound()},
        &pt);
    right_pts.emplace_back(pt);
  }
  if (left_pts.size() + right_pts.size() < 3) {
    LOG_WARN("pts size < 3");
    return;
  }
  std::reverse(right_pts.begin(), right_pts.end());
  left_pts.insert(left_pts.end(), right_pts.begin(), right_pts.end());

  Polygon2d polygon(left_pts);
  bool pose_valid =
      polygon.is_point_in({data_center_->vehicle_state_odometry().X(),
                           data_center_->vehicle_state_odometry().Y()});
  remote_interaction_context->set_is_pnc_ok(theta_valid && pose_valid);
  remote_interaction_context->set_valid_pose(pose_valid);
  remote_interaction_context->set_valid_theta(theta_valid);
}

ErrorCode PlanningManager::StitchTrajectory() {
  const auto &vehicle_state = data_center_->vehicle_state_proxy();
  double planning_cycle_time =
      1.0 / (std::max(FLAGS_planning_period_frequence, 1));
  LOG_DEBUG("xxx start stitch");
  if (trajectory_stitcher_->stitch(
          vehicle_state, *(data_center_->control_command_msg.ptr),
          data_center_->last_frame(), data_center_->init_frame_time(),
          planning_cycle_time,
          &stitching_trajectory_) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("stitch failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (stitching_trajectory_.empty()) {
    LOG_ERROR("stitching_trajectory is empty");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  data_center_->mutable_master_info()->set_is_use_position_stitch(
      trajectory_stitcher_->IsUsePositionStitch());
  LOG_DEBUG("xxx end stitch");

  // Test
  LOG_INFO("stitching_trajectory size: {}", stitching_trajectory_.size());
  for (const auto &point : stitching_trajectory_) {
    LOG_INFO(
        "stitch_point_info: relative_time:{:.3f}, x:{:.3f}, y:{:.3f}, "
        "theta:{:.3f}, kappa:{:.3f}, dkappa:{:.3f}, s:{:.3f}, v:{:.3f}, "
        "a:{:.3f}, j:{:.3f}",
        point.relative_time(), point.x(), point.y(), point.theta(),
        point.kappa(), point.dkappa(), point.s(), point.velocity(),
        point.acceleration(), point.jerk());
  }

  return ErrorCode::PLANNING_OK;
}

ErrorCode PlanningManager::ExecuteMotionTasks(TaskInfo &task_info) {
  if (task_info.skip_curr_task_planning()) {
    LOG_INFO("skip_curr_task_planning");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  common::util::TimeLogger motion_task_logger("ExecuteMotionTasks");
  ErrorCode ret = ErrorCode::PLANNING_OK;
  LOG_INFO("ExecuteMotionTasks started");

  auto &frame = task_info.current_frame();
  auto planning_data =
      dynamic_cast<EMPlanningData *>(frame->mutable_planning_data());

  auto reference_line = task_info.reference_line();

  if (!data_center_->global_state_proxy().IsInSpecailState()) {
    // cruise
    if (IsInvalidReferenceLine(reference_line,
                               planning_data->init_planning_point())) {
      LOG_ERROR(
          "global_state in not spacial state, and IsInvalidReferenceLine.");
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  }

  // set data to planning data
  auto decision_data = task_info.decision_data();
  planning_data->set_reference_line(reference_line);
  planning_data->set_decision_data(decision_data);
  ObsToLocalCoordinate(task_info, planning_data->mutable_decision_data());

  LOG_INFO("set_decision_data finished.");
  motion_task_logger.RegisterTime("decision_data");

  Vec2d vehicle_point(planning_data->init_planning_point().x(),
                      planning_data->init_planning_point().y());

  SLPoint start_sl_point;
  if (!reference_line->GetPointInFrenetFrameWithHeading(
          vehicle_point, planning_data->init_planning_point().theta(),
          &start_sl_point)) {
    LOG_ERROR("get_point_in_reference_line failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  // compute forward and backward length
  double vel = data_center_->vehicle_state_proxy().LinearVelocity();
  double ref_length = reference_line->GetLength();
  double forward_length = 0.0;
  double backward_length = 0.0;
  double stop_s = std::max(ref_length, 5.0);
  LOG_DEBUG("current start s: {:.4f}, l: {:.4f}", start_sl_point.s(),
            start_sl_point.l());
  Utility::calc_plan_length(
      start_sl_point.s() - reference_line->ref_points().front().s(), stop_s,
      vel, &backward_length, &forward_length);
  LOG_DEBUG(
      "task_forward_length:{:.2f}, back_length:{:.2f}, init s:{:.2f}, "
      "ref_length:{:.2f}",
      forward_length, backward_length, start_sl_point.s(), ref_length);

  motion_task_logger.RegisterTime("change_lane_done");

  LOG_INFO("PlanningManager on process is ready.");
  ret = ScenarioManager::Instance()->ExecuteScenarioStageMotionTasks(task_info);

  if (task_info.current_frame()
          ->outside_planner_data()
          .ask_take_over_immediately) {
    data_center_->mutable_master_info()->set_is_ask_for_takeover(true);
  }

  // combine path && speed
  if (!planning_data->AggregatePathAndSpeedData(
          FLAGS_planning_output_trajectory_time_resolution,
          FLAGS_planning_trajectory_time_length)) {
    LOG_ERROR("Fail to aggregate speed and path data !");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!data_center_->master_info().behavior_stop_vehicle() &&
      !task_info.current_frame()->outside_planner_data().speed_slow_down &&
      data_center_->master_info().curr_scenario() != ScenarioState::PARKING) {
    speed_planner_common::CreepProcess(
        task_info.current_frame()->inside_planner_data(), planning_data);
  }

  motion_task_logger.RegisterTimeAndPrint("task_process done");
  return ret;
}

ErrorCode PlanningManager::PostProcessPlanningResult(
    std::list<TaskInfo> &task_info_list) {
  ErrorCode ret = ErrorCode::PLANNING_OK;

  TaskInfo *choosed_task_info = nullptr;
  LOG_INFO("task size: [{}]", task_info_list.size());
  int task_info_index = 0;

  for (auto &task_info : task_info_list) {
    if (task_info.error_code() == ErrorCode::PLANNING_OK) {
      choosed_task_info = &task_info;
      if (task_info.last_frame() == nullptr) {
        LOG_INFO("no last data, use this result {}", task_info_index);
        break;
      }
      if (data_center_->global_state_proxy().is_finish()) {
        LOG_INFO("current state is finish, skip... {}", task_info_index);
        continue;
      }
      break;
    }
    task_info_index++;
  }

  if (choosed_task_info == nullptr) {
    LOG_ERROR("Failed to plan on all reference lines.");
    BehaviorProcess(&task_info_list.front());
    FinishProcess();
    publisher_->PublishEstopTrajectory();
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  BehaviorProcess(choosed_task_info);

  // this is for multi task_info, use master info of choosed task info
  // replace the main one in datacenter.
  data_center_->SaveTask(*choosed_task_info);

  return ret;
}

void PlanningManager::BehaviorProcess(TaskInfo *const task_info) {
  if (task_info == nullptr || task_info->reference_line() == nullptr) return;
  auto reference_line = task_info->reference_line();
  auto &frame = task_info->current_frame();
  auto planning_data = frame->mutable_planning_data();
  auto master_info = data_center_->mutable_master_info();
  if (planning_data == nullptr ||
      planning_data->mutable_decision_data() == nullptr) {
    return;
  }

  // update inner state
  // output obs to log for debuging
  // ObsInfoLogger(*planning_data->mutable_decision_data());

  //**update stop reason
  // check invalid pos
  auto &curr_global_state = data_center_->global_state_proxy().global_state();
  if (!data_center_->global_state_proxy().IsInSpecailState()) {
    // cruise
    if (IsInvalidReferenceLine(reference_line,
                               planning_data->init_planning_point())) {
      master_info->set_stop_due_to_invalid_pos(true);
    } else {
      master_info->set_stop_due_to_invalid_pos(false);
    }
  }
  // update status
  new_human_interface_decider_->UpdateStopReason(*master_info,
                                                 curr_global_state);
  new_human_interface_decider_->AbnormalStopMonitor();
  new_human_interface_decider_->ObstacleBlockMonitor();
  new_human_interface_decider_->CheckReachStation();
  LOG_INFO("stop_reason: {}",
           data_center_->global_state_proxy().global_state().stop_reason());
}

void PlanningManager::FinishProcess() {
  auto master_info = data_center_->mutable_master_info();
  double distance_to_end = master_info->distance_to_end();
  double speed =
      data_center_->environment().vehicle_state_proxy().LinearVelocity();
  speed = std::abs(speed);

  LOG_INFO("distance_to_end:{}, linear_velocity:{}", distance_to_end, speed);
  auto &global_state = data_center_->global_state_proxy().global_state();
  double finish_distance_threshold =
      FLAGS_planning_arrive_to_destination_distance_threshold;

  if (global_state.state() == neodrive::global::status::State::CRUISE) {
    if (distance_to_end <= finish_distance_threshold &&
        speed <= FLAGS_planning_adc_stop_velocity_threshold) {
      publisher_->PublishClearTrajectory();
    }
  }
}

bool PlanningManager::IsInvalidReferenceLine(
    const ReferenceLinePtr reference_line,
    const TrajectoryPoint &init_point) const {
  SLPoint sl_point;
  if (!reference_line->GetPointInFrenetFrameWithHeading(
          {init_point.x(), init_point.y()}, init_point.theta(), &sl_point)) {
    LOG_ERROR("Fail to project on reference line, xy=({:.4f}, {:.4f})",
              init_point.x(), init_point.y());
    if (reference_line->ref_points().empty()) {
      LOG_INFO("refer line size: 0");
    } else {
      LOG_INFO("refer line size: {}, start_s: {:.4f}, end_s: {:.4f}",
               reference_line->ref_points().size(),
               reference_line->ref_points().front().s(),
               reference_line->ref_points().back().s());
    }
    return true;
  }
  double target_pt_s =
      sl_point.s() + VehicleParam::Instance()->front_edge_to_center();
  ReferencePoint ahead_ref_point;
  if (!reference_line->GetNearestRefPoint(target_pt_s, &ahead_ref_point)) {
    LOG_WARN("failed to GetNearestRefPoint, s: {:.4f}", target_pt_s);
  }
  if (fabs(ahead_ref_point.s() - sl_point.s()) > 5.0) {
    LOG_INFO("use s to get reference point failed, use xy instead");
    Vec2d xy_pt;
    xy_pt.set_x(init_point.x());
    xy_pt.set_y(init_point.y());

    if (!reference_line->GetNearestRefPointWithHeading(
            xy_pt, init_point.theta(), &ahead_ref_point)) {
      LOG_WARN("failed to GetNearestRefPoint, x: {:.4f}, y: {:.4f}", xy_pt.x(),
               xy_pt.y());
    }
  }

  double ref_seg_heading = ahead_ref_point.heading();
  double curr_pos_heading = normalize_angle(init_point.theta());
  ref_seg_heading = normalize_angle(ref_seg_heading);
  double heading_diff = normalize_angle(ref_seg_heading - curr_pos_heading);

  if (data_center_->master_info().curr_scenario() != ScenarioState::PARKING &&
      std::fabs(heading_diff) >
          FLAGS_invalid_reference_line_heading_diff_threshold) {
    LOG_ERROR(
        "invalid reference line, heading_diff:{:.2f} too large."
        "init_point.x:{}, y:{},"
        "init_point.heading:{},"
        "ahead_ref_point.x:{}, y:{},"
        "ref_seg_heading:{},"
        "heading_diff_threshold: {},"
        "reference_line_length:{},",
        std::fabs(heading_diff), init_point.x(), init_point.y(),
        curr_pos_heading, ahead_ref_point.x(), ahead_ref_point.y(),
        ref_seg_heading, FLAGS_invalid_reference_line_heading_diff_threshold,
        reference_line->GetLength());
    return true;
  }

  return false;
}

void PlanningManager::ObsToLocalCoordinate(TaskInfo &task_info,
                                           DecisionData *const decision_data) {
  auto &vehicle_state_proxy = data_center_->vehicle_state_proxy();
  double vel_x = vehicle_state_proxy.X();
  double vel_y = vehicle_state_proxy.Y();
  double vel_yaw = vehicle_state_proxy.Heading();
  bool obs_close_to_adc{false};

  for (auto &obstacle : decision_data->all_obstacle()) {
    auto &corners = obstacle->polygon_corners();
    std::vector<Vec2d> corners_local;
    double x_target_local = 0.0;
    double y_target_local = 0.0;

    for (auto &tmp_corner : corners) {
      x_target_local = (tmp_corner.x() - vel_x) * cos(vel_yaw) +
                       (tmp_corner.y() - vel_y) * sin(vel_yaw);
      y_target_local = (vel_x - tmp_corner.x()) * sin(vel_yaw) +
                       (tmp_corner.y() - vel_y) * cos(vel_yaw);
      corners_local.emplace_back(Vec2d(x_target_local, y_target_local));
    }
    obstacle->set_local_polygon(corners_local);
    JudgeIfObsCloseToAdc(obstacle, corners_local, obs_close_to_adc);
  }
  has_closer_obs_ = obs_close_to_adc;
  return;
}

void PlanningManager::ObsInfoLogger(const DecisionData &decision_data) {
  // log obs info
  for (auto &obstacle : decision_data.all_obstacle()) {
    if (obstacle == nullptr) continue;
    if (obstacle->is_virtual()) continue;
    std::stringstream str;
    auto &predic_traj = obstacle->prediction_trajectories();
    if (predic_traj.size() > 0) {
      size_t i = predic_traj.size();
      for (size_t j = 0; j < i; ++j)
        str << predic_traj[j].num_of_points() << ",";
    }
    str << std::fixed;
    str << std::setprecision(4);
    str << "global_coor: ";
    auto &poly_corner = obstacle->polygon_corners();
    for (auto &tmp_corner : poly_corner) {
      str << tmp_corner.x() << " , " << tmp_corner.y() << " , ";
    }

    auto &poly_corner_local = obstacle->local_polygon();
    str << "local_coor: ";
    for (auto &tmp_corner : poly_corner_local) {
      str << tmp_corner.x() << " , " << tmp_corner.y() << " , ";
    }
    std::string str1 = str.str();

    LOG_DEBUG(
        "obs id: [{}], vel: {:.2f}, vel_heading: {:.2f}, static: {}, "
        "virtual: {}, traj_size: {},  {}, {}",
        obstacle->id(), obstacle->speed(), obstacle->velocity_heading(),
        obstacle->is_static(), obstacle->is_virtual(), predic_traj.size(), str1,
        VirtualObstacle::VirtualType_Name(obstacle->virtual_type()));
  }

  return;
}

bool PlanningManager::UpdateTaskInfoUtilsInfo(TaskInfo &task_info) {
  // set adc point.
  auto &vehicle_state = data_center_->vehicle_state_proxy();
  task_info.mutable_adc_point()->set_x(vehicle_state.X());
  task_info.mutable_adc_point()->set_y(vehicle_state.Y());
  task_info.mutable_adc_point()->set_theta(vehicle_state.Heading());
  task_info.mutable_adc_point()->set_velocity(vehicle_state.LinearVelocity());

  // set adc point in frenet frame.
  LOG_INFO(
      "ego x:{:.4f}, y:{:.4f}, ref start pt x:{:.4f}, y:{:.4f}, ref end pt "
      "x:{:.4f}, y:{:.4f}",
      vehicle_state.X(), vehicle_state.Y(),
      task_info.reference_line()->ref_points().front().x(),
      task_info.reference_line()->ref_points().front().y(),
      task_info.reference_line()->ref_points().back().x(),
      task_info.reference_line()->ref_points().back().y());
  if (!task_info.reference_line()->GetPointInFrenetFrameWithHeading(
          {vehicle_state.X(), vehicle_state.Y()}, vehicle_state.Heading(),
          task_info.mutable_curr_sl())) {
    LOG_ERROR("GetPointInFrenetFrame failed.");
    return false;
  }
  // set closest referline point.
  const bool is_parking{data_center_->master_info().curr_scenario() ==
                            ScenarioState::PARKING ||
                        data_center_->parking_ptr() != nullptr};
  double heading_tol = is_parking ? M_PI : M_PI_2;
  if (!task_info.reference_line()->GetNearestRefPointWithHeading(
          {vehicle_state.X(), vehicle_state.Y()}, vehicle_state.Heading(),
          task_info.mutable_curr_referline_pt(), heading_tol)) {
    return false;
  }
  // set closest referline point index.
  *task_info.mutable_referline_curr_index() =
      ref_line_util::BinarySearchIndex(task_info.reference_line()->ref_points(),
                                       task_info.curr_referline_pt().s());

  // set adc boundary.
  if (!is_parking &&
      !ref_line_util::GetAdcBoundingBoxBoundary(
          task_info.reference_line(), {vehicle_state.X(), vehicle_state.Y()},
          vehicle_state.Heading(), task_info.mutable_adc_boundary_origin(), 0.0,
          0.0, 0.0)) {
    LOG_ERROR("GetAdcBoundingBoxBoundary failed.");
    return false;
  }
  double buffer = 0.2;
  if (!is_parking &&
      !ref_line_util::GetAdcBoundingBoxBoundary(
          task_info.reference_line(), {vehicle_state.X(), vehicle_state.Y()},
          vehicle_state.Heading(), task_info.mutable_adc_boundary(), buffer,
          buffer, buffer)) {
    LOG_ERROR("GetAdcBoundingBoxBoundary failed.");
    return false;
  }

  return true;
}

ErrorCode PlanningManager::CheckWhetherSkipPlanning() {
  auto master_info = data_center_->mutable_master_info();
  auto &vehicle_state_proxy = data_center_->vehicle_state_proxy();
  LOG_INFO("state: {} routing: {}, ad: {}, drivingmode: {}",
           data_center_->global_state_proxy().StateName(),
           data_center_->routing_result().is_available,
           data_center_->is_auto_driving(),
           DrivingMode_Name(vehicle_state_proxy.DrivingMode()));

  if (data_center_->is_auto_driving() && data_center_->have_task()) {
    if (data_center_->global_state_proxy().is_init() ||
        data_center_->global_state_proxy().is_finish()) {
      data_center_->mutable_global_state_proxy()->set_cruise();
    }
  }
  LOG_INFO("have_task:{}, is_finish:{}", data_center_->have_task(),
           data_center_->global_state_proxy().is_finish());

  if (data_center_->global_state_proxy().is_finish()) {
    LOG_INFO("skip plan");
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  return ErrorCode::PLANNING_OK;
}

void PlanningManager::InitAdcLocalPolygonCheckCloserObs() {
  double closer_obs_distance_threshold =
      config::PlanningConfig::Instance()
          ->plan_config()
          .common.closer_obs_distance_threshold;
  double check_edge_x_local_positive =
      VehicleParam::Instance()->front_edge_to_center() +
      closer_obs_distance_threshold;
  double check_edge_x_local_negtive =
      -VehicleParam::Instance()->back_edge_to_center() -
      closer_obs_distance_threshold;
  double check_edge_y_local_positive =
      VehicleParam::Instance()->left_edge_to_center() +
      closer_obs_distance_threshold;
  double check_edge_y_local_negtive =
      -VehicleParam::Instance()->right_edge_to_center() -
      closer_obs_distance_threshold;
  std::vector<Vec2d> points;
  points.emplace_back(
      Vec2d(check_edge_x_local_negtive, check_edge_y_local_negtive));
  points.emplace_back(
      Vec2d(check_edge_x_local_positive, check_edge_y_local_negtive));
  points.emplace_back(
      Vec2d(check_edge_x_local_positive, check_edge_y_local_positive));
  points.emplace_back(
      Vec2d(check_edge_x_local_negtive, check_edge_y_local_positive));
  adc_local_polygon_check_closer_obs_ = Polygon2d(points);
}

void PlanningManager::JudgeIfObsCloseToAdc(
    const Obstacle *const obs, const std::vector<Vec2d> &obs_corners_local,
    bool &obs_close_to_adc) {
  if (obs_close_to_adc) {
    return;
  }
  if ((obs->type() == Obstacle::ObstacleType::UNKNOWN) ||
      (obs->type() == Obstacle::ObstacleType::UNKNOWN_UNMOVABLE)) {
    return;
  }
  Polygon2d obs_local_polygon{obs_corners_local};
  if (adc_local_polygon_check_closer_obs_.has_overlap(obs_local_polygon)) {
    obs_close_to_adc = true;
  }
}

void PlanningManager::SaveMonitorMessage() {
  LOG_DEBUG("SaveMonitorMessage");
  data_center_->SaveMonitorMessage();
  auto frame = data_center_->current_frame();
  if (frame == nullptr) {
    frame = const_cast<Frame *>(data_center_->last_frame());
    if (frame == nullptr) {
      LOG_ERROR("SaveMonitorMessage frame == nullptr");
      return;
    }
  }
  auto &master_info = data_center_->master_info();

  static char str_buffer[256];
  using neodrive::global::control::ControlAction_Name;
  auto driving_mode = data_center_->vehicle_state_proxy().DrivingMode();
  auto open_api_state = data_center_->planning_interface_msg.ptr->state();
  auto stop_reason = data_center_->planning_interface_msg.ptr->stop_reason();
  auto dis_to_end = master_info.distance_to_end();
  sprintf(
      str_buffer,
      "[DRIVE][%s][ad: %d][openapi: %d %s][pad: %d "
      "%s][routing: %s %d][dis2end: %.1lf][station: %d %d %d %d]",
      DrivingMode_Name(driving_mode).c_str(), data_center_->is_auto_driving(),
      data_center_->planning_interface_msg.is_available,
      State_Name(open_api_state).c_str(),
      data_center_->pad_normal_command_msg.is_available,
      ControlAction_Name(data_center_->pad_normal_command_msg.ptr->action())
          .c_str(),
      RoutingResult::ErrorCode::ErrorID_Name(
          data_center_->routing_result_msg.ptr->error_code().error_id())
          .c_str(),
      data_center_->have_task(), dis_to_end,
      master_info.behavior_stop_vehicle(), master_info.is_station_stop(),
      master_info.is_in_inner_station_mode(), master_info.is_trigger_stop());
  data_center_->SetMonitorString(str_buffer, MonitorItemSource::DRIVE_MODE);

  sprintf(str_buffer, "[REMOTE_INTERACTION][ok: %d][pose: %d][theta: %d]",
          data_center_->global_state_proxy()
              .global_state()
              .remote_interaction_context()
              .is_pnc_ok(),
          data_center_->global_state_proxy()
              .global_state()
              .remote_interaction_context()
              .valid_pose(),
          data_center_->global_state_proxy()
              .global_state()
              .remote_interaction_context()
              .valid_theta());
  data_center_->SetMonitorString(str_buffer,
                                 MonitorItemSource::REMOTE_INTERACTION);

  auto mutable_planning_data = frame->mutable_planning_data();
  auto &outside_planner_data = frame->outside_planner_data();
  size_t num_of_points = 0;
  if (mutable_planning_data != nullptr) {
    auto &computed_trajectory = mutable_planning_data->computed_trajectory();
    num_of_points = computed_trajectory.num_of_points();
  }
  sprintf(str_buffer,
          "[REF][ref size: %lu %s][trajectory: %lu]"
          "[frenet_init_point: %.1lf %.1lf %.1lf %.1lf]",
          ref_line_points_size_, publisher_->coordinate_type_str().c_str(),
          num_of_points, outside_planner_data.frenet_init_point.s(),
          outside_planner_data.frenet_init_point.l(),
          outside_planner_data.frenet_init_point.dl(),
          outside_planner_data.frenet_init_point.ddl());

  data_center_->SetMonitorString(str_buffer, MonitorItemSource::REFERENCE_LINE);

  double valid_width = outside_planner_data.multi_mode_data.valid_width > 1000.0
                           ? 0
                           : outside_planner_data.multi_mode_data.valid_width;
  sprintf(str_buffer,
          "[MOTION][skip: %d][path: %d %d][speed: %d %d][multi_mode: "
          "%d][valid_width: %.1lf][slowdown: %d %d %d %d %d]%s",
          skip_motion_plan_, outside_planner_data.path_succeed_tasks,
          outside_planner_data.path_fail_tasks,
          outside_planner_data.speed_succeed_tasks,
          outside_planner_data.speed_fail_tasks,
          frame->inside_planner_data().curr_multi_level, valid_width,
          outside_planner_data.speed_slow_down,
          outside_planner_data.path_slow_down_check_failed,
          outside_planner_data.lms_sensor_check_failed,
          outside_planner_data.road_bound_safe_check_failed,
          outside_planner_data.moving_obs_slow_down,
          SpeedMonitorInfo(frame->inside_planner_data(), outside_planner_data)
              .c_str());
  data_center_->SetMonitorString(ObstacleMonitorInfo(outside_planner_data),
                                 MonitorItemSource::OBSTACLE_ID);
  data_center_->SetMonitorString(
      data_center_->behavior_speed_limits().AggregateSpeedLimitStr(),
      MonitorItemSource::SPEED_LIMIT);
  data_center_->SetMonitorString(str_buffer,
                                 MonitorItemSource::MOTION_PLANNING);

  data_center_->SetMonitorString(
      PathMonitorInfo(frame->inside_planner_data(), outside_planner_data),
      MonitorItemSource::PATH_PLANNING);
  std::string fail_task_str = "[FAIL_TASK]";
  for (const auto &str : data_center_->fail_tasks()) {
    fail_task_str += "[" + str + "]";
  }
  data_center_->SetMonitorString(fail_task_str, MonitorItemSource::FAIL_TASK);

  std::string takeover_str = "[REQUEST_TAKEOVER]:";
  auto &event_report = data_center_->event_report_proxy().event_report();
  for (int i = 0; i < event_report.event_infos_size(); i++) {
    takeover_str += "[" + event_report.event_infos(i).name() + "]";
  }
  data_center_->SetMonitorString(takeover_str,
                                 MonitorItemSource::REQUEST_TAKEOVER);

  std::string conflictZone_str = "[ConflictZone]";
  auto &zoneContext =
      frame->outside_planner_data().traffic_conflict_zone_context;
  for (std::string s : zoneContext.state) {
    conflictZone_str += "[" + s + "]";
  }
  data_center_->SetMonitorString(conflictZone_str,
                                 MonitorItemSource::CONFLICT_ZONE);
  std::string obs_intention_str = "[OBSINTENTION]";
  auto &obs_intenion_context =
      frame->outside_planner_data().obs_intention_context_str;
  for (auto intention_s : obs_intenion_context) {
    obs_intention_str += intention_s.second;
  }
  data_center_->SetMonitorString(obs_intention_str,
                                 MonitorItemSource::OBSTACLE_INTENTION);
}

std::string PlanningManager::PathMonitorInfo(
    const InsidePlannerData &inside_data,
    const OutsidePlannerData &outside_data) const {
  double init_s = outside_data.frenet_init_point.s();
  std::string str;
  double min_delta_s = 5.;
  double accumulated_s = -1.;
  double min_bound_width = 1000.;
  str = "[PATH]bounds:";
  for (std::size_t i = 0; i < outside_data.path_opt_boundaries.size(); ++i) {
    auto &bound = outside_data.path_opt_boundaries[i];
    auto bound_width =
        std::fabs(bound.left_bound) + std::fabs(bound.right_bound);
    if (bound_width <= min_bound_width - 0.1 ||
        bound.s - accumulated_s > min_delta_s) {
      str += "[" + DoubleFormat(bound.s - init_s, 1) + "," +
             DoubleFormat(bound.left_bound, 1) + "," +
             DoubleFormat(bound.right_bound, 1) + "," +
             DoubleFormat(bound_width, 1) + "]";
      accumulated_s = bound.s;
      if (bound_width < min_bound_width) {
        min_bound_width = bound_width;
      }
    }
  }
  return str;
}

std::string PlanningManager::ObstacleMonitorInfo(
    const OutsidePlannerData &outside_data) const {
  std::string str;
  str += "[OBS]:[ignore obs: ";
  std::string ignore_obs_str;
  const auto &dp_st_map_ignore_dynamic_obs_id =
      outside_data.speed_obstacle_context.dp_st_map_ignore_dynamic_obs_id;
  const auto &dp_st_map_ignore_static_obs_id =
      outside_data.speed_obstacle_context.dp_st_map_ignore_static_obs_id;
  const auto &ignore_dynamic_obs_id =
      outside_data.motorway_speed_obstacle_context.ignore_dynamic_obs_id;
  const auto &ignore_static_obs_id =
      outside_data.motorway_speed_obstacle_context.ignore_static_obs_id;
  std::size_t i = 0;
  for (const auto &obs_id : dp_st_map_ignore_dynamic_obs_id) {
    ignore_obs_str += std::to_string(obs_id);
    if (i < dp_st_map_ignore_dynamic_obs_id.size()) {
      ignore_obs_str += ", ";
    }
    i++;
  }
  i = 0;
  for (const auto &obs_id : dp_st_map_ignore_static_obs_id) {
    ignore_obs_str += std::to_string(obs_id);
    if (i < dp_st_map_ignore_static_obs_id.size()) {
      ignore_obs_str += ", ";
    }
    i++;
  }
  i = 0;
  for (const auto &obs_id : ignore_dynamic_obs_id) {
    ignore_obs_str += std::to_string(obs_id);
    if (i < ignore_dynamic_obs_id.size()) {
      ignore_obs_str += ", ";
    }
    i++;
  }
  i = 0;
  for (const auto &obs_id : ignore_static_obs_id) {
    ignore_obs_str += std::to_string(obs_id);
    if (i < ignore_static_obs_id.size()) {
      ignore_obs_str += ", ";
    }
    i++;
  }

  ignore_obs_str += "]";
  str += ignore_obs_str;

  return str;
}

std::string PlanningManager::SpeedMonitorInfo(
    const InsidePlannerData &inside_data,
    const OutsidePlannerData &outside_data) const {
  double init_s = outside_data.frenet_init_point.s();
  std::string str;
  str += "[static_pre:";
  std::string static_pre_str{};
  double static_pre_s = std::numeric_limits<double>::max();
  for (const auto &obs_collide_info :
       outside_data.speed_obstacle_context.static_pre_obstacles_decision) {
    if (static_pre_s > obs_collide_info.lower_points.front().first.s()) {
      static_pre_s = obs_collide_info.lower_points.front().first.s();
      static_pre_str =
          std::to_string(obs_collide_info.obstacle.id()) + "," +
          DoubleFormat(obs_collide_info.lower_points.front().first.s(), 1) +
          "; ";
    }
  }
  str += static_pre_str;
  str += "][static:";
  std::string static_str{};
  double static_s = static_pre_s;
  for (const auto &obs_collide_info :
       outside_data.speed_obstacle_context.static_obstacles_decision) {
    if (static_s > obs_collide_info.lower_points.front().first.s()) {
      static_s = obs_collide_info.lower_points.front().first.s();
      new_human_interface_decider_->UpdateClosestObsS(static_s);
      static_str =
          std::to_string(obs_collide_info.obstacle.id()) + "," +
          DoubleFormat(obs_collide_info.lower_points.front().first.s(), 1) +
          "; ";
    }
  }
  for (const auto &obs_collide_info :
       outside_data.motorway_speed_obstacle_context
           .multi_cipv_static_obstacles_decision) {
    if (static_s > obs_collide_info.lower_points.front().first.s()) {
      static_s = obs_collide_info.lower_points.front().first.s();
      new_human_interface_decider_->UpdateClosestObsS(static_s);
      static_str =
          std::to_string(obs_collide_info.obstacle.id()) + "," +
          DoubleFormat(obs_collide_info.lower_points.front().first.s(), 1) +
          "; ";
    }
  }
  str += static_str;
  str += "][virtual:";
  std::string virtual_str{};
  double virtual_s = std::numeric_limits<double>::max();
  for (const auto &obs_collide_info :
       outside_data.speed_obstacle_context.virtual_obstacle_decision) {
    if (virtual_s > obs_collide_info.lower_points.front().first.s()) {
      virtual_s = obs_collide_info.lower_points.front().first.s();
      virtual_str =
          std::to_string(obs_collide_info.obstacle.id()) + "," +
          DoubleFormat(obs_collide_info.lower_points.front().first.s(), 1) +
          "," +
          neodrive::global::planning::VirtualObstacle_VirtualType_Name(
              obs_collide_info.obstacle.virtual_type());
    }
  }
  for (const auto &obs_collide_info :
       outside_data.motorway_speed_obstacle_context
           .multi_cipv_virtual_obstacle_decision) {
    if (virtual_s > obs_collide_info.lower_points.front().first.s()) {
      virtual_s = obs_collide_info.lower_points.front().first.s();
      virtual_str =
          std::to_string(obs_collide_info.obstacle.id()) + "," +
          DoubleFormat(obs_collide_info.lower_points.front().first.s(), 1) +
          "," +
          neodrive::global::planning::VirtualObstacle_VirtualType_Name(
              obs_collide_info.obstacle.virtual_type());
    }
  }
  str += virtual_str;
  str += "][dynamic:";
  std::string dynamic_str{};
  double dynamic_s = std::numeric_limits<double>::max();
  for (const auto &obs_collide_info :
       outside_data.speed_obstacle_context.dynamic_obstacles_decision) {
    if (!obs_collide_info.lower_points.empty() &&
        dynamic_s > obs_collide_info.lower_points.front().first.s()) {
      dynamic_s = obs_collide_info.lower_points.front().first.s();
      dynamic_str =
          std::to_string(obs_collide_info.obstacle.id()) + "," +
          DoubleFormat(obs_collide_info.lower_points.front().first.s(), 1) +
          "," +
          DoubleFormat(obs_collide_info.lower_points.front().first.t(), 1) +
          "; ";
    }
  }
  for (const auto &obs_collide_info :
       outside_data.motorway_speed_obstacle_context
           .multi_cipv_dynamic_obstacles_decision) {
    if (!obs_collide_info.lower_points.empty() &&
        dynamic_s > obs_collide_info.lower_points.front().first.s()) {
      dynamic_s = obs_collide_info.lower_points.front().first.s();
      dynamic_str =
          std::to_string(obs_collide_info.obstacle.id()) + "," +
          DoubleFormat(obs_collide_info.lower_points.front().first.s(), 1) +
          "," +
          DoubleFormat(obs_collide_info.lower_points.front().first.t(), 1) +
          "; ";
    }
  }
  str += dynamic_str + "][stop line: ";
  std::string stopline_str{};
  if (outside_data.traffic_light_stop_line < 1000.0) {
    str += DoubleFormat(outside_data.traffic_light_stop_line, 1);
  }
  str += "]";
  return str;
}

}  // namespace planning
}  // namespace neodrive
