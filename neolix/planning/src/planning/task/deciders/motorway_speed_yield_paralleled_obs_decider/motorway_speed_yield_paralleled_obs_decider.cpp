#include "motorway_speed_yield_paralleled_obs_decider.h"

#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/util/speed_planner_common.h"

using JunctionType = autobot::cyberverse::Junction::JunctionType;

namespace neodrive {
namespace planning {

namespace {
const auto& obs_parallel_risk_config_{config::PlanningConfig::Instance()
                                          ->planning_research_config()
                                          .obs_parallel_risk};

bool IsSmallObs(const Obstacle* const obs_ptr) {
  if (nullptr == obs_ptr) {
    LOG_ERROR("obs is nullptr.");
    return false;
  }
  bool is_little_size = std::pow(std::max(obs_ptr->length(), obs_ptr->width()),
                                 2.0) < config::PlanningConfig::Instance()
                                            ->plan_config()
                                            .common.little_obs_area_threshold;
  return ((obs_ptr->type() == Obstacle::ObstacleType::BICYCLE) ||
          (obs_ptr->type() == Obstacle::ObstacleType::PEDESTRIAN) ||
          is_little_size);
}

}  // namespace

MotorwaySpeedYieldParalleledObsDecider::
    MotorwaySpeedYieldParalleledObsDecider() {
  name_ = "MotorwaySpeedYieldParalleledObsDecider";
}

MotorwaySpeedYieldParalleledObsDecider::
    ~MotorwaySpeedYieldParalleledObsDecider() {
  Reset();
}

ErrorCode MotorwaySpeedYieldParalleledObsDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  if (!DataCheck(task_info)) {
    LOG_ERROR("DataCheck failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  if (!Process(task_info)) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

void MotorwaySpeedYieldParalleledObsDecider::SaveTaskResults(
    TaskInfo& task_info) {
  if (update_limited_speed_) {
    UpdatedLimitedSpeed();
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::COLLISION_RISK);
    internal_speed_limit.add_upper_bounds(limited_speed_);
    internal_speed_limit.set_constraint_type(SpeedLimitType::HARD);
    internal_speed_limit.set_acceleration(limited_deceleration_);
    LOG_INFO(
        "COLLISION_RISK {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        limited_speed_, limited_deceleration_);
    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);

    last_limited_speed_ = limited_speed_;
  }
}

bool MotorwaySpeedYieldParalleledObsDecider::Process(TaskInfo& task_info) {
  if (!Init(task_info)) {
    LOG_ERROR("Init failed.");
    return false;
  }

  if (adc_current_v_ < obs_parallel_risk_config_.yield_min_peed) {
    LOG_INFO("adc is slower, not deal in parallel risk.");
    return true;
  }

  if (DynamicObstacleParallelCheck(task_info) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("DynamicObstacleParallelCheck failed.");
    return false;
  }

  return true;
}

bool MotorwaySpeedYieldParalleledObsDecider::Init(TaskInfo& task_info) {
  adc_current_l_ = task_info.curr_sl().l();
  adc_current_s_ = task_info.curr_sl().s();
  adc_front_edge_s_ =
      adc_current_s_ + VehicleParam::Instance()->front_edge_to_center();
  adc_back_edge_s_ =
      adc_current_s_ - VehicleParam::Instance()->back_edge_to_center();
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
  min_parallel_dis_ = (3.5 - VehicleParam::Instance()->width()) / 2.0;
  if (!update_limited_speed_) {
    last_limited_speed_ = adc_current_v_;
  }
  update_limited_speed_ = false;
  limited_speed_ = std::numeric_limits<double>::infinity();
  limited_deceleration_ = 0.0;
  target_speed_ = 1e5;

  auto& inside_data = task_info.current_frame()->inside_planner_data();
  adc_polygon_ = VehicleParam::Instance()->get_adc_polygon(
      {inside_data.vel_x, inside_data.vel_y}, inside_data.vel_heading, 0.0, 0.0,
      0.0);
  ClearObsHistoryInfo();
  if (!InitCollisionCheckArea(task_info.adc_boundary_origin())) {
    LOG_ERROR("Fail to init collision check area.");
    return false;
  }

  // TODO1(lgf):use path to judge direction
  if (!speed_planner_common::JudgeDirectionCarWillGo(
          task_info.reference_line(),
          task_info.current_frame()->inside_planner_data(),
          car_go_direction_)) {
    LOG_ERROR("Fail to judge the direction car will go.");
    return false;
  }

  GetCollideDecisionObs(task_info);
  return true;
}

bool MotorwaySpeedYieldParalleledObsDecider::InitCollisionCheckArea(
    const Boundary& adc_boundary) {
  adc_left_collision_check_area_ = std::move(
      Boundary{adc_boundary.start_s(), adc_boundary.end_s() + 1.5,
               adc_boundary.end_l(), adc_boundary.end_l() + kSideAreaWidth});

  adc_right_collision_check_area_ = std::move(Boundary{
      adc_boundary.start_s(), adc_boundary.end_s() + 1.5,
      adc_boundary.start_l() - kSideAreaWidth, adc_boundary.start_l()});

  LOG_INFO(
      "adc boundary, s_s, e_s, s_l, e_l: {:.2f}, {:.2f},{:.2f}, {:.2f}; "
      "adc_current_v {:.3f}",
      adc_boundary.start_s(), adc_boundary.end_s(), adc_boundary.start_l(),
      adc_boundary.end_l(), adc_current_v_);

  return true;
}

void MotorwaySpeedYieldParalleledObsDecider::ClearObsHistoryInfo() {
  // clear paralleled_obs_info_
  for (auto& iter : paralleled_obs_info_) {
    iter.second.lost_cnt++;
  }
  auto obs_parallel_info_tmp = paralleled_obs_info_;
  for (auto& iter : paralleled_obs_info_) {
    if (iter.second.lost_cnt > 4) {
      obs_parallel_info_tmp.erase(iter.first);
    }
  }
  std::swap(paralleled_obs_info_, obs_parallel_info_tmp);
}

bool MotorwaySpeedYieldParalleledObsDecider::DataCheck(TaskInfo& task_info) {
  if (task_info.current_frame() == nullptr) {
    LOG_ERROR("current_frame is nullptr.");
    return false;
  }

  if (task_info.last_frame() == nullptr) {
    LOG_ERROR("last_frame is nullptr.");
    return false;
  }
  if (task_info.current_frame()->mutable_outside_planner_data() == nullptr) {
    LOG_ERROR("mutable_outside_planner_data() is nullptr.");
    return false;
  }

  const auto& multi_cipv_dynamic_obstacles_decision =
      task_info.current_frame()
          ->outside_planner_data()
          .motorway_speed_obstacle_context
          .multi_cipv_dynamic_obstacles_decision;

  for (const auto& obs_decision : multi_cipv_dynamic_obstacles_decision) {
    if (obs_decision.lower_points.empty() ||
        obs_decision.upper_points.empty() ||
        obs_decision.lower_points_heading_diff.empty() ||
        obs_decision.upper_points_heading_diff.empty()) {
      LOG_ERROR(
          "obs_decision.lower_points or obs_decision.upper_points is "
          "empty.");
      return false;
    }
  }

  return true;
}

void MotorwaySpeedYieldParalleledObsDecider::GetCollideDecisionObs(
    TaskInfo& task_info) {
  has_collision_dynamic_obs_.clear();
  const auto& dynamic_obs_collision_info =
      task_info.current_frame()
          ->outside_planner_data()
          .motorway_speed_obstacle_context
          .multi_cipv_dynamic_obstacles_decision;
  std::string str_log = "dynamic_obstacles_decision has obs: ";
  for (const auto& info : dynamic_obs_collision_info) {
    if (!info.reverse && info.lower_points.size() && info.upper_points.size()) {
      has_collision_dynamic_obs_[info.obstacle.id()] =
          CollisionInfo{info.lower_points_heading_diff.front(),
                        info.lower_points.front().first.s(),
                        info.lower_points.front().first.t(),
                        info.lower_points.front().second, info.risk_obs};
      str_log += std::to_string(info.obstacle.id()) + " ,";
    }
  }
  str_log += "num: " + std::to_string(has_collision_dynamic_obs_.size());
  LOG_INFO("{}", str_log);
}

void MotorwaySpeedYieldParalleledObsDecider::UpdataParalleledObsInfo(
    TaskInfo& task_info, const Obstacle* const obs) {
  auto& paralleled_obs_info = paralleled_obs_info_[obs->id()];
  paralleled_obs_info.lat_dis.push_back(
      speed_planner_common::GetObs2AdcLateralDis(*obs, adc_current_l_));
  paralleled_obs_info.lon_dis.push_back(obs->PolygonBoundary().end_s() -
                                        adc_back_edge_s_);
  paralleled_obs_info.obs_speed.push_back(obs->speed());
  paralleled_obs_info.lost_cnt = 0;
  if (++paralleled_obs_info.parallel_cnt > 1e5) {
    paralleled_obs_info.parallel_cnt = 1e5;
  }
  if (paralleled_obs_info.lat_dis.size() >
      obs_parallel_risk_config_.history_info_size) {
    paralleled_obs_info.lat_dis.pop_front();
    paralleled_obs_info.lon_dis.pop_front();
    paralleled_obs_info.obs_speed.pop_front();
  }
}

ErrorCode MotorwaySpeedYieldParalleledObsDecider::DynamicObstacleParallelCheck(
    TaskInfo& task_info) {
  if (task_info.last_frame() == nullptr) {
    return ErrorCode::PLANNING_OK;
  }
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  const auto& dynamic_obs_vec = task_info.current_frame()
                                    ->planning_data()
                                    .decision_data()
                                    .dynamic_obstacle();
  auto outside_data = task_info.current_frame()->mutable_outside_planner_data();

  for (std::size_t i = 0; i < dynamic_obs_vec.size(); ++i) {
    if (dynamic_obs_vec[i] == nullptr) {
      continue;
    }

    double heading_diff = normalize_angle(
        dynamic_obs_vec[i]->velocity_heading() - inside_data.vel_heading);
    if (std::abs(heading_diff) > M_PI_2) {
      continue;
    }

    const auto& obs_boundary = dynamic_obs_vec[i]->PolygonBoundary();
    if (!adc_left_collision_check_area_.has_overlap(obs_boundary) &&
        !adc_right_collision_check_area_.has_overlap(obs_boundary)) {
      LOG_INFO("lgf obs [{}] not overlap with check area.",
               dynamic_obs_vec[i]->id());
      continue;
    }

    UpdataParalleledObsInfo(task_info, dynamic_obs_vec[i]);

    if ((has_collision_dynamic_obs_.find(dynamic_obs_vec[i]->id()) !=
         has_collision_dynamic_obs_.end()) &&
        (has_collision_dynamic_obs_[dynamic_obs_vec[i]->id()].lower_first_t <
         2.0)) {
      LOG_INFO(
          "obs [{}] lower_first_t {:.3f}, dealt in main planner: ",
          dynamic_obs_vec[i]->id(),
          has_collision_dynamic_obs_[dynamic_obs_vec[i]->id()].lower_first_t);
      continue;
    }

    if (adc_left_collision_check_area_.has_overlap(obs_boundary)) {
      DealSideRiskObs(dynamic_obs_vec[i]);
      continue;
    }

    if (adc_right_collision_check_area_.has_overlap(obs_boundary)) {
      DealSideRiskObs(dynamic_obs_vec[i]);
      continue;
    }
  }

  return ErrorCode::PLANNING_OK;
}

void MotorwaySpeedYieldParalleledObsDecider::GetObsParallelResult(
    const Obstacle* const obs, bool& is_parallel_obs,
    double obs_correct_min_parallel_dis) {
  is_parallel_obs = false;
  auto& paralleled_obs_info = paralleled_obs_info_[obs->id()];
  // parallel case 1:lat move closer
  if ((paralleled_obs_info.obs_speed.size() <
       obs_parallel_risk_config_.history_info_size / 2)) {
    LOG_INFO("wait [{}] cycles", paralleled_obs_info.parallel_cnt);
    is_parallel_obs = false;
    return;
  }

  double lat_dis_per_sec =
      paralleled_obs_info.lat_dis.front() - paralleled_obs_info.lat_dis.back();
  if ((lat_dis_per_sec > 0.05) ||
      (paralleled_obs_info.lat_close_cnt >=
       obs_parallel_risk_config_.lat_close_min_cycle)) {
    if (++paralleled_obs_info.lat_close_cnt >=
        obs_parallel_risk_config_.lat_close_min_cycle) {
      paralleled_obs_info.lat_close_cnt =
          obs_parallel_risk_config_.lat_close_min_cycle + 1;
    }
    LOG_INFO("obs [{}] move close: lat_dis_per_sec {:.3f}; lat_close_cnt {}.",
             obs->id(), lat_dis_per_sec, paralleled_obs_info.lat_close_cnt);
  } else {
    paralleled_obs_info.lat_close_cnt = 0;
  }

  // parallel case 2: obs and adc speed near,judge time or lateral distance.
  LOG_INFO(
      "check obs and adc speed : obs [{}], speed {:.2f}, "
      "adc_current_v_ {:.2f}",
      obs->id(), obs->speed(), adc_current_v_);
  double safe_speed_delta = std::max(2.0, adc_current_v_ * 0.2);
  if ((obs->speed() > adc_current_v_ + safe_speed_delta) ||
      (obs->speed() < adc_current_v_ - safe_speed_delta)) {
    is_parallel_obs = false;
    return;
  }

  if (paralleled_obs_info.lat_close_cnt >=
      obs_parallel_risk_config_.lat_close_min_cycle) {
    LOG_INFO("obs [{}] lat_close_cnt {} move closer.", obs->id(),
             paralleled_obs_info.lat_close_cnt);
    is_parallel_obs = true;
    return;
  }

  // double min_parallel_dis_by_obs = ;
  LOG_INFO(
      "obs [{}] width {:.2f}; parallel_cnt {}; lat_dis {:.2f} "
      "obs_correct_min_parallel_dis {:.2f}",
      obs->id(), obs->width(), paralleled_obs_info.parallel_cnt,
      paralleled_obs_info.lat_dis.back(), obs_correct_min_parallel_dis);
  if (paralleled_obs_info.parallel_cnt >
      obs_parallel_risk_config_.parallel_min_cycle) {
    LOG_INFO("obs [{}] not faster, but parallel long time, yeild.", obs->id());
    is_parallel_obs = true;
  } else if (paralleled_obs_info.lat_dis.back() <
             obs_correct_min_parallel_dis) {
    LOG_INFO("obs [{}] has overtake intertion, yeild.", obs->id());
    is_parallel_obs = true;
  }
}

void MotorwaySpeedYieldParalleledObsDecider::DealSideRiskObs(
    const Obstacle* const obs) {
  double obs_correct_min_parallel_dis = std::max(
      0.6, min_parallel_dis_ - std::max(obs->width(), 2.0) / 2.0 * 0.2);

  bool is_parallel_obs{false};
  GetObsParallelResult(obs, is_parallel_obs, obs_correct_min_parallel_dis);
  LOG_INFO("obs [{}] is_parallel_obs {}.", obs->id(), is_parallel_obs);

  if (is_parallel_obs &&
      (adc_current_v_ > obs_parallel_risk_config_.yield_min_peed) &&
      (GoDirectionType::GO_STRAIGHT == car_go_direction_)) {
    update_limited_speed_ = true;
    target_speed_ =
        std::min(target_speed_,
                 static_cast<double>(obs_parallel_risk_config_.yield_min_peed));
    if (adc_current_v_ < target_speed_ + 1.0) {
      // in case control overshoot causing adc speed lower than target speed.
      limited_deceleration_ = 0.0;
    } else {
      limited_deceleration_ =
          std::min(limited_deceleration_,
                   paralleled_obs_info_[obs->id()].lat_dis.back() >
                           obs_correct_min_parallel_dis
                       ? obs_parallel_risk_config_.default_deceleration
                       : 2.0 * obs_parallel_risk_config_.default_deceleration);
    }

    LOG_INFO(
        "adc in risk area and observe obs [{}], adc_current_v_ {:.3f}, set "
        "target_speed_ {:.3f}, limited_deceleration_ {:.3f}",
        obs->id(), adc_current_v_, target_speed_, limited_deceleration_);
  }
}

void MotorwaySpeedYieldParalleledObsDecider::UpdatedLimitedSpeed() {
  if ((adc_current_v_ < target_speed_) ||
      (std::abs(limited_deceleration_) < 1e-2)) {
    limited_speed_ = target_speed_;
    limited_deceleration_ = 0.0;
    return;
  }

  limited_speed_ =
      std::min(limited_speed_, last_limited_speed_ +
                                   kPlanningCycleTime * limited_deceleration_);
  limited_speed_ = std::max(limited_speed_, target_speed_);

  LOG_INFO(
      "target_speed_ {:.2f}, set limited_speed_ {:.2f}, limited_deceleration_ "
      "{:.2f}",
      target_speed_, limited_speed_, limited_deceleration_);
}

}  // namespace planning
}  // namespace neodrive
