#include "station_stop_decider.h"

#include "common/util/time_logger.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/vehicle_param.h"
#include "src/planning/planning_map/planning_map.h"

namespace neodrive {
namespace planning {

StationStopDecider::StationStopDecider() : inner_station_stop_time_(0.0) {
  name_ = "StationStopDecider";
}

StationStopDecider::~StationStopDecider() { Clear(); }

void StationStopDecider::Reset() {}

void StationStopDecider::Clear() {
  is_station_stop_ = false;
  is_station_stop_need_pull_over_ = false;
  inner_station_stop_time_ = 0.0;

  station_list_.clear();
  station_info tmp_info;
  current_station_ = tmp_info;
}

ErrorCode StationStopDecider::Execute(TaskInfo &task_info) {
  behavior_stop_vehicle_ = false;
  is_destination_changed_ = false;
  if (!task_info.reference_line()->ref_points().empty()) {
    station_stop_pos_ = task_info.reference_line()->ref_points().back();
  }
  data_center_->mutable_master_info()->set_pull_over_distance_to_goal(
      data_center_->master_info().distance_to_end());
  UpdateRoutingRequest(task_info);
  UpdateStateInfo(task_info);
  // pull over config
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  const auto &reference_line = task_info.reference_line();
  const auto &decision_data = task_info.decision_data();
  data_center_->mutable_global_state_proxy()
      ->mutable_global_state()
      ->mutable_context()
      ->set_pull_over_not_in_place(false);
  bool is_motorway_ =
      (common::config::CommonConfig::Instance()
           ->drive_strategy_config()
           .enable_motorway &&
       task_info.curr_referline_pt().lane_type_is_pure_city_driving());
  // check whether pullover in place
  if (FLAGS_planning_enable_destination_pull_over && !is_motorway_) {
    if (data_center_->master_info().distance_to_end() <
        plan_config.pull_over.early_check_queue_distance) {
      GetObstaclesBoundary(reference_line, decision_data, &obstacles_boundary_);
      if (is_destination_in_unloading_zone_ || is_destination_in_shoulder_) {
        WhetherFormationPullOver(task_info, obstacles_boundary_);
      }
    }
    if (data_center_->master_info().distance_to_end() <
        plan_config.pull_over.check_obstacle_distance) {
      WhetherTakeOverOrMove(task_info);
    }
    if (data_center_->master_info().pull_over_distance_to_goal() <
            plan_config.pull_over.check_obstacle_distance ||
        data_center_->master_info().distance_to_end() <
            plan_config.pull_over.check_obstacle_distance) {
      WhetherSafePullOver(task_info);
      WhetherPullOverInPlace(task_info);
    }
  }
  // mode 1: final station mode
  WhetherReachFinalStation(task_info);
  if (has_inner_station_) {
    WhetherReachFrontInnerStation(task_info);
  }
  // mode 2: inner station mode
  if (has_inner_station_ && is_in_inner_station_mode_ &&
      data_center_->global_state_proxy().is_wait()) {
    WhetherRestartFromStationStop();
  }
  if (data_center_->is_auto_driving()) {
    if (data_center_->global_state_proxy()
            .global_state()
            .context()
            .pull_over_not_in_place()) {
      LOG_INFO("set pull over fail state.");
      DataCenter::Instance()->mutable_event_report_proxy()->SetEvent(
          EventType::PULL_OVER_FAIL);
    } else {
      LOG_INFO("reset pull over fail state!");
      data_center_->mutable_global_state_proxy()
          ->mutable_global_state()
          ->mutable_context()
          ->set_pull_over_not_in_place(false);
      DataCenter::Instance()->mutable_event_report_proxy()->EventReset(
          EventType::PULL_OVER_FAIL);
    }
  }
  return ErrorCode::PLANNING_OK;
}

void StationStopDecider::SaveTaskResults(TaskInfo &task_info) {
  bool in_the_park =
      task_info.last_frame() != nullptr &&
      task_info.last_frame()->inside_planner_data().is_in_the_park;
  if (data_center_->global_state_proxy()
          .global_state()
          .context()
          .pull_over_not_in_place() ||
      in_the_park) {
    is_station_stop_need_pull_over_ = false;
  }
  LOG_INFO(
      "has_inner_station:{}, is_station_stop:{}, "
      "is_station_stop_need_pull_over: {}, "
      "is_destination_in_unloading_zone: "
      "{}, is_destination_in_shoulder:{}, is_in_inner_station_mode: {}, "
      "behavior_stop_vehicle_:{}, station_list_.size():{}",
      has_inner_station_, is_station_stop_, is_station_stop_need_pull_over_,
      is_destination_in_unloading_zone_, is_destination_in_shoulder_,
      is_in_inner_station_mode_, behavior_stop_vehicle_, station_list_.size());
  data_center_->mutable_master_info()->set_is_station_stop(is_station_stop_);
  data_center_->mutable_master_info()->set_is_station_stop_need_pull_over(
      is_station_stop_need_pull_over_);
  data_center_->mutable_master_info()->set_is_destination_in_unlaoding_zone(
      is_destination_in_unloading_zone_);
  if (behavior_stop_vehicle_) {
    LOG_WARN("set_behavior_stop_vehicle");
    data_center_->mutable_master_info()->set_behavior_stop_vehicle(true);
  }
  data_center_->mutable_master_info()->set_is_in_inner_station_mode(
      is_in_inner_station_mode_);
  data_center_->mutable_master_info()->set_reference_line_destination_point(
      station_stop_pos_);
  // save monitor message.
  static char str_buffer[256];
  sprintf(str_buffer,
          "[PULLOVER][pull_over: %d][unloading_goal: "
          "%d][shoulder_goal: %d][station_occupy: %d][front_obs_dis_to_end: "
          "%.1lf][back_obs_dis_to_end: %.1lf]",
          is_station_stop_need_pull_over_, is_destination_in_unloading_zone_,
          is_destination_in_shoulder_, obstacle_in_station_,
          front_obs_dis_to_end_, back_obs_dis_to_end_);
  data_center_->SetMonitorString(str_buffer, MonitorItemSource::PULL_OVER);
}

bool StationStopDecider::UpdateRoutingRequest(TaskInfo &task_info) {
  if (!data_center_->routing_result().is_updated) {
    return false;
  }
  auto &routing_result = *(data_center_->routing_result().ptr);
  if (!(routing_result.has_header() && routing_result.has_routing_request())) {
    LOG_INFO("invalid routing request, skip");
    return false;
  }

  has_inner_station_ = false;
  Clear();
  if (routing_result.routing_request().waypoint_size() !=
      routing_result.routing_request().stop_at_each_waypoint_size()) {
    LOG_WARN("waypoint_size {} != stop_at_each_waypoint_size {}",
             routing_result.routing_request().waypoint_size(),
             routing_result.routing_request().stop_at_each_waypoint_size());
    return true;
  }
  for (int i = 0; i < routing_result.routing_request().waypoint_size(); ++i) {
    station_info tmp_station_info;
    if (routing_result.routing_request().stop_at_each_waypoint(i) == false) {
      continue;
    }
    tmp_station_info.station_pos.set_x(
        routing_result.routing_request().waypoint(i).pose().x());
    tmp_station_info.station_pos.set_y(
        routing_result.routing_request().waypoint(i).pose().y());
    tmp_station_info.need_stop_time =
        routing_result.routing_request().stop_time_at_each_waypoint(i);
    tmp_station_info.need_pullover =
        routing_result.routing_request().pull_over_at_each_waypoint(i);
    station_list_.push_back(tmp_station_info);
  }

  LOG_INFO("waypoint size: {}, station size: {}",
           routing_result.routing_request().waypoint_size(),
           station_list_.size());
  if (station_list_.size() >= 1) {
    current_station_ = station_list_.front();
    station_list_.pop_front();
  }
  has_inner_station_ = true;
  return true;
}

void StationStopDecider::UpdateStateInfo(TaskInfo &task_info) {
  if (IsDestinationInUnloadingZone(task_info)) {
    is_destination_in_unloading_zone_ = true;
  } else {
    is_destination_in_unloading_zone_ = false;
  }
  if (IsDestinationInShoulder(task_info)) {
    is_destination_in_shoulder_ = true;
  } else {
    is_destination_in_shoulder_ = false;
  }
}

bool StationStopDecider::WhetherReachFinalStation(TaskInfo &task_info) {
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  double vel = std::abs(vehicle_state_.LinearVelocity());
  LOG_INFO(
      "is_cruise {}, distance_to_end {:.2f}, "
      "station_look_forward_dis {}, "
      "arrive_to_destination_distance_threshold {}, vel {}, "
      "adc_stop_velocity_threshold {}",
      data_center_->global_state_proxy().is_cruise(),
      data_center_->master_info().distance_to_end(),
      FLAGS_planning_station_look_forward_dis,
      FLAGS_planning_arrive_to_destination_distance_threshold, vel,
      FLAGS_planning_adc_stop_velocity_threshold);
  if (data_center_->mutable_master_info()->distance_to_end() <
      FLAGS_planning_arrive_to_destination_distance_threshold) {
    behavior_stop_vehicle_ = true;
    LOG_INFO("distance to end less than {:.1f} m, stop from behavior",
             FLAGS_planning_arrive_to_destination_distance_threshold);
  }

  double forward_dis = vel * vel / FLAGS_planning_parking_stop_max_deceleration;
  forward_dis = std::max(forward_dis, FLAGS_planning_station_look_forward_dis);
  double distance_to_end =
      data_center_->mutable_master_info()->distance_to_end();

  bool need_pull_over = false;
  if (data_center_->routing_result().ptr->has_routing_request() &&
      data_center_->routing_result()
          .ptr->routing_request()
          .has_pull_over_at_destination()) {
    need_pull_over = data_center_->routing_result()
                         .ptr->routing_request()
                         .pull_over_at_destination();
  } else {
    need_pull_over = FLAGS_planning_enable_destination_pull_over;
  }
  if (distance_to_end > forward_dis) {
    is_station_stop_ = false;
  } else {
    is_station_stop_ = true;
  }
  if (distance_to_end < FLAGS_planning_station_look_forward_dis &&
      vel < plan_config.pull_over.allow_pull_over_max_speed) {
    if (IsAllowPullOverInTurn(task_info)) {
      is_station_stop_need_pull_over_ = need_pull_over;
    } else {
      is_station_stop_need_pull_over_ = false;
    }
  } else {
    is_station_stop_need_pull_over_ = false;
  }
  if (is_station_stop_ &&
      distance_to_end <=
          5 * FLAGS_planning_arrive_to_destination_distance_threshold &&
      vel <= 2 * FLAGS_planning_adc_stop_velocity_threshold) {
    data_center_->mutable_global_state_proxy()->SetFinish(true);
    data_center_->mutable_routing_result()->is_available = false;
    data_center_->mutable_master_info()
        ->mutable_cruise_context()
        ->is_finish_detour_and_near_station = false;
    LOG_INFO("arrive to final destination, finish state:{}",
             data_center_->global_state_proxy().is_finish());
  }
  return true;
}

bool StationStopDecider::WhetherReachFrontInnerStation(TaskInfo &task_info) {
  if (!data_center_->global_state_proxy().is_cruise()) {
    return true;
  }
  if (std::abs(current_station_.station_pos.x()) < 1.0 &&
      std::abs(current_station_.station_pos.y()) < 1.0) {
    return true;
  }

  auto &refer_line = task_info.reference_line();
  ReferencePoint curr_refer_pt;
  ReferencePoint station_refer_pt;

  if (!refer_line->GetNearestRefPointWithHeading(
          Vec2d{vehicle_state_.X(), vehicle_state_.Y()},
          vehicle_state_.Heading(), &curr_refer_pt)) {
    LOG_DEBUG("failed to GetNearestRefPoint, curr pos");
    return false;
  }
  if (!refer_line->GetNearestRefPoint(Vec2d{current_station_.station_pos.x(),
                                            current_station_.station_pos.y()},
                                      &station_refer_pt)) {
    LOG_DEBUG("failed to GetNearestRefPoint, inner station");
    return true;
  }

  double vel = std::abs(vehicle_state_.LinearVelocity());
  double forward_dis = vel * vel / FLAGS_planning_parking_stop_max_deceleration;
  forward_dis = std::max(forward_dis, FLAGS_planning_station_look_forward_dis);
  double dis_to_station = station_refer_pt.s() - curr_refer_pt.s();
  LOG_DEBUG(
      "distance to inner station: {:.4f}, curr s: {:.4f}, station s: {:.4f}",
      dis_to_station, curr_refer_pt.s(), station_refer_pt.s());

  if (!(dis_to_station < forward_dis &&
        dis_to_station > -VehicleParam::Instance()->length())) {
    return true;
  }
  is_station_stop_ = true;
  is_station_stop_need_pull_over_ = current_station_.need_pullover;
  // set destination pos to motion plan
  station_stop_pos_ = current_station_.station_pos;
  is_in_inner_station_mode_ = true;
  LOG_INFO("inner station mode: {}, need pull over: {}", is_station_stop_,
           is_station_stop_need_pull_over_);
  // reached station?
  double finish_distance_threshold =
      FLAGS_planning_arrive_to_destination_distance_threshold;
  if (dis_to_station < finish_distance_threshold &&
      dis_to_station > -VehicleParam::Instance()->front_edge_to_center()) {
    behavior_stop_vehicle_ = true;
  }

  finish_distance_threshold += VehicleParam::Instance()->front_edge_to_center();
  if (data_center_->global_state_proxy().is_cruise()) {
    if (dis_to_station <= finish_distance_threshold &&
        vel <= FLAGS_planning_adc_stop_velocity_threshold) {
      LOG_INFO("arrive to inner destination, set wait");
      data_center_->mutable_global_state_proxy()->set_wait();
      inner_station_stop_time_ =
          common::util::TimeLogger::GetCurrentTimeseocnd();
    }
  }

  return true;
}

bool StationStopDecider::WhetherRestartFromStationStop() {
  if (station_list_.empty()) {
    return true;
  }

  if (inner_station_stop_time_ < 1.0) {
    return true;
  }

  double delta_time = common::util::TimeLogger::GetCurrentTimeseocnd() -
                      inner_station_stop_time_;
  LOG_INFO("set stop time: {:.4f}, curr_stopped time: {:.4f}",
           current_station_.need_stop_time, delta_time);
  if (delta_time < current_station_.need_stop_time) {
    is_station_stop_ = true;
    is_station_stop_need_pull_over_ = current_station_.need_pullover;
    return true;
  }
  // clear current station info and find next station
  LOG_INFO("restart from station stop, set global state state to cruise");
  current_station_ = station_list_.front();
  station_list_.pop_front();
  data_center_->mutable_global_state_proxy()->set_cruise();
  is_station_stop_ = false;
  is_station_stop_need_pull_over_ = false;
  inner_station_stop_time_ = 0.0;
  is_in_inner_station_mode_ = false;
  return true;
}

void StationStopDecider::WhetherTakeOverOrMove(TaskInfo &task_info) {
  const auto &reference_line = task_info.reference_line();
  const auto &init_point =
      task_info.current_frame()->planning_data().init_planning_point();
  const auto &destination_point =
      data_center_->master_info().destination_point();
  front_obs_dis_to_end_ = kSafeLength;
  back_obs_dis_to_end_ = kSafeLength;
  bool is_in_park = data_center_->is_in_port_odd();
  LOG_INFO("station stop check is in park:{}", is_in_park);

  if (!CalcLeftAndRightLaneBound(reference_line,
                                 task_info.adc_boundary().start_s(),
                                 task_info.adc_boundary().end_s() + 5.0, 1.0,
                                 left_lane_max_bound_, right_lane_max_bound_)) {
    LOG_INFO("CalcLaneBound err");
  }
  if (is_destination_in_unloading_zone_ || is_destination_changed_ ||
      is_destination_in_shoulder_)
    return;
  CalFrontAndBackSafeDistance(task_info, obstacles_boundary_, destination_point,
                              front_obs_dis_to_end_, back_obs_dis_to_end_,
                              obstacle_in_station_);
  if (obstacle_in_station_ ||
      front_obs_dis_to_end_ + back_obs_dis_to_end_ < kSafeLength) {
    data_center_->mutable_global_state_proxy()
        ->mutable_global_state()
        ->mutable_context()
        ->set_pull_over_not_in_place(true);
    LOG_INFO("station has no enough space to pull over, need take over!");
    return;
  }
  if (is_in_park) {
    return;
  }
  if (data_center_->master_info()
          .cruise_context()
          .is_finish_detour_and_near_station) {
    move_on_dis_ = std::clamp(front_obs_dis_to_end_ - 3.0, 0.0, kSafeLength);
    data_center_->mutable_master_info()->set_pull_over_distance_to_goal(
        data_center_->master_info().distance_to_end() + move_on_dis_);
    data_center_->mutable_master_info()->set_distance_to_end(
        data_center_->master_info().distance_to_end() + move_on_dis_);
    LOG_INFO("detour finished, should move on, dis:{:.2f}", move_on_dis_);
  } else {
    destination_back_has_obs_ = back_obs_dis_to_end_ < kSafeLength;
    destination_front_has_obs_ = front_obs_dis_to_end_ < kSafeLength;
    if (destination_front_has_obs_ && destination_back_has_obs_ &&
        front_obs_dis_to_end_ + back_obs_dis_to_end_ > kSafeLength) {
      // destination_s=destination_s+(kSafeBackLength-back_obs_dis_to_end_)-(kSafeFrontLength-front_obs_dis_to_end_)
      // destination_s=destination_s+5-back_obs_dis_to_end_+front_obs_dis_to_end_
      move_on_dis_ =
          std::min(kSafeBackLength - back_obs_dis_to_end_,
                   back_obs_dis_to_end_ + front_obs_dis_to_end_ - 2.0);
      data_center_->mutable_master_info()->set_pull_over_distance_to_goal(
          data_center_->master_info().distance_to_end() + move_on_dis_);
      data_center_->mutable_master_info()->set_distance_to_end(
          data_center_->master_info().distance_to_end() + move_on_dis_);
      LOG_INFO(
          "exist obstacles both ahead and behind target, but have enough "
          "space, should move on, dis:{:.2f}",
          move_on_dis_);
    } else if (back_obs_dis_to_end_ < kSafeBackLength || move_on_dis_ != 0) {
      move_on_dis_ =
          std::max(move_on_dis_, kSafeBackLength - back_obs_dis_to_end_);
      data_center_->mutable_master_info()->set_pull_over_distance_to_goal(
          data_center_->master_info().distance_to_end() + move_on_dis_);
      data_center_->mutable_master_info()->set_distance_to_end(
          data_center_->master_info().distance_to_end() + move_on_dis_);
      LOG_INFO("exist obs behind target, should move on, dis:{:.2f}",
               move_on_dis_);
    } else if (front_obs_dis_to_end_ < kSafeFrontLength ||
               stop_in_advance_distance_ != 0) {
      stop_in_advance_distance_ = std::max(
          stop_in_advance_distance_, kSafeFrontLength - front_obs_dis_to_end_);
      data_center_->mutable_master_info()->set_pull_over_distance_to_goal(
          data_center_->master_info().distance_to_end() -
          stop_in_advance_distance_);
      data_center_->mutable_master_info()->set_distance_to_end(
          data_center_->master_info().distance_to_end() -
          stop_in_advance_distance_);
      LOG_INFO(
          "exist obs ahead target, should pull over in advance, "
          "end_in_advance_distance:{:.2f}",
          stop_in_advance_distance_);
    } else {
      destination_back_has_obs_ = false;
      destination_front_has_obs_ = false;
    }
  }
}

void StationStopDecider::WhetherSafePullOver(TaskInfo &task_info) {
  auto &dynamic_obstacles = task_info.decision_data()->dynamic_obstacle();
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  dynamic_obstacles_.clear();
  for (const auto &obs : dynamic_obstacles) {
    if (obs->PolygonBoundary().start_l() > task_info.adc_boundary().start_l()) {
      continue;
    }
    if (obs->PolygonBoundary().end_s() + 10.0 <
        task_info.adc_boundary().start_s()) {
      continue;
    }
    if (obs->PolygonBoundary().start_s() - 5.0 >
        task_info.adc_boundary().end_s()) {
      continue;
    }
    dynamic_obstacles_.push_back(*obs);
  }
  double observe_t = 2.0;
  std::vector<Vec2d> aabox2d_points{
      {task_info.adc_boundary().start_s(), task_info.adc_boundary().start_l()},
      {task_info.adc_boundary().end_s() +
           observe_t * vehicle_state_.LinearVelocity(),
       task_info.adc_boundary().start_l()},
      {task_info.adc_boundary().end_s() +
           observe_t * vehicle_state_.LinearVelocity(),
       -right_lane_max_bound_},
      {task_info.adc_boundary().start_s(), -right_lane_max_bound_}};
  AABox2d attention_region = AABox2d(aabox2d_points);
  LOG_INFO(
      "pull over region(max s,min s,max l,min l):{:.2f},{:.2f},{:.2f},{:.2f}",
      attention_region.max_x(), attention_region.min_x(),
      attention_region.max_y(), attention_region.min_y());
  for (auto &obs : dynamic_obstacles_) {
    if (obs.speed() < 1.0) continue;
    std::vector<Vec2d> aabox2d_points{
        {obs.PolygonBoundary().start_s(), obs.PolygonBoundary().start_l()},
        {obs.PolygonBoundary().end_s() + observe_t * obs.speed(),
         obs.PolygonBoundary().start_l()},
        {obs.PolygonBoundary().end_s() + observe_t * obs.speed(),
         obs.PolygonBoundary().end_l()},
        {obs.PolygonBoundary().start_s(), obs.PolygonBoundary().end_l()}};
    AABox2d box(aabox2d_points);
    box.set_id(obs.id());
    if (box.has_overlap(attention_region)) {
      LOG_INFO(
          "pull over collision risk. obs(max s,min s,max l,min "
          "l):{:.2f},{:.2f},{:.2f},{:.2f}",
          box.max_x(), box.min_x(), box.max_y(), box.min_y());
      if ((vehicle_state_.LinearVelocity() < obs.speed() &&
           task_info.adc_boundary().end_s() >
               obs.PolygonBoundary().start_s()) ||
          data_center_->master_info().distance_to_end() < 15.0) {
        neodrive::global::planning::SpeedLimit internal_speed_limit{};
        internal_speed_limit.set_source_type(SpeedLimitType::STATION_STOP);
        internal_speed_limit.add_upper_bounds(
            plan_config.pull_over.station_stop_deceleration_ratio *
            vehicle_state_.LinearVelocity());
        internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
        internal_speed_limit.set_acceleration(0.0);
        LOG_INFO(
            "STATION_STOP {} limit speed: speed = "
            "{:.2f}, acc = {:.2f}",
            SpeedLimit_ConstraintType_Name(
                internal_speed_limit.constraint_type()),
            0.7 * vehicle_state_.LinearVelocity(), 0.0);

        data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
            internal_speed_limit);
        break;
      }
    }
  }
}

void StationStopDecider::WhetherFormationPullOver(
    TaskInfo &task_info,
    const std::vector<PathObstacleBoundary> &obstacles_boundary) {
  obs_idxes_unload_.clear();
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  const auto &destination_point =
      data_center_->master_info().destination_point();
  if (obstacles_boundary.empty()) return;
  for (int i = 0, n = obstacles_boundary.size(); i < n; ++i) {
    auto &boundary = obstacles_boundary[i].boundary;
    LOG_INFO(
        "collin dest s:{:.2f}, start s:{:.2f}, obs s:({:.2f}, {:.2f}), "
        "l:({:.2f}, {:.2f})",
        destination_point.s(), task_info.adc_boundary().start_s(),
        boundary.start_s(), boundary.end_s(), boundary.start_l(),
        boundary.end_l());
    if (boundary.start_s() > destination_point.s()) continue;
    if (boundary.end_s() < task_info.adc_boundary().start_s()) continue;
    if (boundary.end_s() - boundary.start_s() > 10) continue;
    if (boundary.end_s() - boundary.start_s() < 1.0) continue;
    if (is_destination_in_unloading_zone_) {
      if (boundary.end_l() < 0) continue;
    } else if (is_destination_in_shoulder_) {
      if (boundary.start_l() > 0) continue;
    }
    obs_idxes_unload_.push_back(i);
  }
  std::sort(obs_idxes_unload_.begin(), obs_idxes_unload_.end(),
            [&obstacles_boundary](auto &a, auto &b) {
              return obstacles_boundary[a].boundary.end_s() >
                     obstacles_boundary[b].boundary.end_s();
            });
  double goal = data_center_->master_info().destination_point().s();
  LOG_INFO("original goal:{:.2f}", goal);
  for (auto i : obs_idxes_unload_) {
    auto &unload_obs = obstacles_boundary[i];
    LOG_INFO("unloading area obs start:{:.2f}, end:{:.2f}, id:{}",
             unload_obs.boundary.start_s(), unload_obs.boundary.end_s(),
             unload_obs.obstacle.id());
    if (goal - unload_obs.boundary.end_s() < 15.0) {
      goal = unload_obs.boundary.start_s() -
             plan_config.pull_over.wujiang_stop_queue_vehicle_distance;
      is_destination_changed_ = true;
    }
  }
  LOG_INFO("queue end goal:{:.2f}, adc end:{:.2f}", goal,
           task_info.adc_boundary().end_s());
  data_center_->mutable_master_info()->set_distance_to_end(
      goal - task_info.curr_sl().s());
  data_center_->mutable_master_info()->set_pull_over_distance_to_goal(
      goal - task_info.curr_sl().s());
  LOG_INFO("dis to goal:{:.2f}",
           data_center_->mutable_master_info()->distance_to_end());
}

void StationStopDecider::WhetherPullOverInPlace(TaskInfo &task_info) {
  if (is_destination_in_unloading_zone_ || is_destination_in_shoulder_) return;
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  const auto &reference_line = task_info.reference_line();

  // check whether need take over to pull over
  if (data_center_->master_info().distance_to_end() < 2.0 &&
      data_center_->vehicle_state_proxy().LinearVelocity() <
          FLAGS_planning_adc_stop_velocity_threshold) {
    ReferencePoint ref_pt;
    if (!reference_line->GetNearestRefPoint(task_info.curr_sl().s(), &ref_pt)) {
      LOG_ERROR("GetNearestRefPoint fail !");
    }
    LOG_INFO("adc right l:{:.2f}, right road bound:{:.2f}",
             task_info.adc_boundary().start_l(), -ref_pt.right_road_bound());
    if (FLAGS_planning_default_left_right_side) {
      if (task_info.adc_boundary().start_l() - (-ref_pt.right_road_bound()) >
          plan_config.pull_over.check_stop_distance_to_boundary) {
        LOG_INFO("right pull over failed, need take over!");
        data_center_->mutable_global_state_proxy()
            ->mutable_global_state()
            ->mutable_context()
            ->set_pull_over_not_in_place(true);
      } else {
        data_center_->mutable_global_state_proxy()
            ->mutable_global_state()
            ->mutable_context()
            ->set_pull_over_not_in_place(false);
      }
    } else if (ref_pt.left_road_bound() - task_info.adc_boundary().end_l() >
               plan_config.pull_over.check_stop_distance_to_boundary) {
      LOG_INFO("left pull over failed, need take over!");
      data_center_->mutable_global_state_proxy()
          ->mutable_global_state()
          ->mutable_context()
          ->set_pull_over_not_in_place(true);
    } else {
      data_center_->mutable_global_state_proxy()
          ->mutable_global_state()
          ->mutable_context()
          ->set_pull_over_not_in_place(false);
    }
  }
}

bool StationStopDecider::GetObstaclesBoundary(
    const ReferenceLinePtr &reference_line,
    const std::shared_ptr<DecisionData> &decision_data,
    std::vector<PathObstacleBoundary> *obstacles_boundary) {
  if (obstacles_boundary == nullptr) {
    return false;
  }
  obstacles_boundary->clear();
  for (std::size_t i = 0; i < decision_data->static_obstacle().size(); ++i) {
    auto obstacle = decision_data->static_obstacle()[i];
    if (obstacle == nullptr || obstacle->is_virtual()) continue;
    LOG_INFO(
        "collin GetObstaclesBoundary s:({:.2f}, {:.2f}), "
        "l:({:.2f}, {:.2f}), id:{}",
        obstacle->min_s(), obstacle->max_s(), obstacle->min_l(),
        obstacle->max_l(), obstacle->id());
    if (obstacle->max_l() < -right_lane_max_bound_ + 0.25) continue;
    if (obstacle->min_l() > left_lane_max_bound_ - 0.25) continue;

    PathObstacleBoundary obstacle_boundary{
        .boundary = obstacle->PolygonBoundary(),
        .obstacle = *obstacle,
        .obstacle_ptr = obstacle};
    obstacles_boundary->emplace_back(obstacle_boundary);
  }
  if (obstacles_boundary->empty()) {
    LOG_INFO("There is no obstacle near the destination.");
    return false;
  }
  sort(obstacles_boundary->begin(), obstacles_boundary->end(),
       [](const PathObstacleBoundary &obstacle_boundary,
          const PathObstacleBoundary &other_obstacle_boundary) {
         return obstacle_boundary.boundary.end_s() <
                other_obstacle_boundary.boundary.end_s();
       });
  return true;
}

bool StationStopDecider::CalcLeftAndRightLaneBound(
    const ReferenceLinePtr &reference_line, const double origin_start_s,
    const double origin_end_s, const double offset, double &left_bound,
    double &right_bound) {
  // add length protection for reference line
  double start_s = origin_start_s;
  double end_s = origin_end_s;
  if (start_s < reference_line->ref_points().front().s()) {
    start_s = reference_line->ref_points().front().s();
    LOG_INFO(
        "CalcMaxLaneBound start_s {} < reference_line->line_start_s() {}, "
        "revise into {}",
        origin_start_s, reference_line->ref_points().front().s(), start_s);
  }
  if (auto es = reference_line->ref_points().back().s(); end_s > es) {
    end_s = es;
    LOG_INFO(
        "CalcMaxLaneBound end_s {} < reference_line->line_end_s() {}, "
        "revise into {}",
        origin_end_s, es, end_s);
  }
  ReferencePoint ref_pt;
  if (!reference_line->GetNearestRefPoint(start_s, &ref_pt)) {
    LOG_ERROR("GetNearestRefPoint fail !");
    return false;
  }
  left_bound = ref_pt.left_bound();
  right_bound = ref_pt.right_bound();

  while (start_s < end_s) {
    if (!reference_line->GetNearestRefPoint(start_s, &ref_pt)) {
      LOG_ERROR("GetNearestRefPoint fail !");
    }
    if (ref_pt.left_bound() > left_bound) {
      left_bound = ref_pt.left_bound();
    }
    if (ref_pt.right_bound() > right_bound) {
      right_bound = ref_pt.right_bound();
    }
    start_s += offset;
  }
  LOG_INFO("min left bound:{:2f}, max right bound:{:2f}", left_bound,
           right_bound);
  return true;
}

void StationStopDecider::CalFrontAndBackSafeDistance(
    TaskInfo &task_info,
    const std::vector<PathObstacleBoundary> &obstacles_boundary,
    const ReferencePoint &destination_point, double &front_dis_to_end,
    double &back_dis_to_end, bool &obstacle_in_station) {
  if (obstacles_boundary.empty()) {
    LOG_INFO("obstacles_boundary is enpty");
    return;
  }
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  obstacle_in_station = false;
  if (!destination_back_has_obs_ && !destination_front_has_obs_) {
    distance_to_new_end_ = data_center_->master_info().distance_to_end();
  }
  for (int i = 0, n = obstacles_boundary.size(); i < n; ++i) {
    auto &obstacle_boundary = obstacles_boundary[i];
    if (obstacle_boundary.boundary.start_l() >
        -right_lane_max_bound_ + VehicleParam::Instance()->width() + 1.0) {
      LOG_INFO("skip obs on the left :{} ",
               obstacle_boundary.obstacle_ptr->id());
      continue;
    }
    if (obstacle_boundary.boundary.start_s() < destination_point.s() &&
        obstacle_boundary.boundary.end_s() > destination_point.s()) {
      obstacle_in_station = true;
      LOG_INFO("destination is occupied by obs:{} ",
               obstacle_boundary.obstacle_ptr->id());
      continue;
    }
    if (obstacle_boundary.boundary.end_s() <
            task_info.adc_boundary().start_s() - kMathEpsilon &&
        (destination_back_has_obs_ || destination_front_has_obs_) &&
        distance_to_new_end_ < 6.0) {
      LOG_INFO("obs:{} is behind adc, but adc is already near the end, skip",
               obstacle_boundary.obstacle_ptr->id());
      continue;
    }
    if (obstacle_boundary.boundary.start_s() > destination_point.s() &&
        obstacle_boundary.boundary.start_s() - destination_point.s() <=
            kSafeLength) {
      front_dis_to_end =
          std::min(front_dis_to_end, obstacle_boundary.boundary.start_s() -
                                         destination_point.s());
      LOG_INFO("obs:{} is in front of destination, front_dis_to_end:{:3f}",
               obstacle_boundary.obstacle_ptr->id(), front_dis_to_end);
      continue;
    }
    if (obstacle_boundary.boundary.end_s() <= destination_point.s() &&
        destination_point.s() - obstacle_boundary.boundary.end_s() <=
            kSafeLength) {
      back_dis_to_end =
          std::min(back_dis_to_end,
                   destination_point.s() - obstacle_boundary.boundary.end_s());
      obs_behind_end_id_ = obstacle_boundary.obstacle_ptr->id();
      LOG_INFO("obs:{} is behind destination, back_dis_to_end:{:3f}",
               obstacle_boundary.obstacle_ptr->id(), back_dis_to_end);
      continue;
    }
  }
}

bool StationStopDecider::IsDestinationInUnloadingZone(TaskInfo &task_info) {
  auto reference_line = task_info.reference_line();
  const auto &destination_point =
      data_center_->master_info().destination_point();
  reference_line->GetNearestRefPoint(destination_point.s(), &dest_ref_point_);
  if (dest_ref_point_.lane_type_is_parking()) {
    LOG_INFO("destination is in unloading zone.");
    return true;
  }
  return false;
}

bool StationStopDecider::IsDestinationInShoulder(TaskInfo &task_info) {
  auto reference_line = task_info.reference_line();
  const auto &destination_point =
      data_center_->master_info().destination_point();
  reference_line->GetNearestRefPoint(destination_point.s(), &dest_ref_point_);
  if (dest_ref_point_.lane_type_is_shoulder()) {
    LOG_INFO("destination is in shoulder.");
    return true;
  }
  return false;
}

bool StationStopDecider::IsAllowPullOverInTurn(TaskInfo &task_info) {
  auto reference_line = task_info.reference_line();
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  double observe_dis = std::fmin(FLAGS_planning_station_look_forward_dis,
                                 data_center_->master_info().distance_to_end());
  // check curvature, right(-), left(+)
  for (const auto &pt : reference_line->ref_points()) {
    if (pt.s() > task_info.curr_sl().s() + observe_dis) break;
    if (pt.s() <= task_info.curr_sl().s()) continue;
    // yizhuang shunfeng requirement, just fit for the only right turning
    if (is_destination_in_unloading_zone_) {
      if (pt.kappa() > plan_config.intention.turn_intention_kappa_threshold) {
        LOG_INFO("front lane has left turn.");
        // return false;
      } else if (pt.kappa() <
                 -plan_config.intention.turn_intention_kappa_threshold) {
        LOG_INFO("front lane has right turn.");
        return true;
      }
    } else {
      if (FLAGS_planning_default_left_right_side) {
        if (pt.kappa() > plan_config.intention.turn_intention_kappa_threshold) {
          LOG_INFO("front lane has left turn.");
          return true;
        } else if (pt.kappa() <
                   -plan_config.intention.turn_intention_kappa_threshold) {
          LOG_INFO("front lane has right turn.");
          return false;
        }
      } else {
        if (pt.kappa() > plan_config.intention.turn_intention_kappa_threshold) {
          LOG_INFO("front lane has left turn.");
          return false;
        } else if (pt.kappa() <
                   -plan_config.intention.turn_intention_kappa_threshold) {
          LOG_INFO("front lane has right turn.");
          return true;
        }
      }
    }
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive
