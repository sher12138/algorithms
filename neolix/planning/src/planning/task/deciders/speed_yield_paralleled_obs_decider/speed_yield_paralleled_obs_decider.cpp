#include "speed_yield_paralleled_obs_decider.h"

#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/util/speed_planner_common.h"

using JunctionType = autobot::cyberverse::Junction::JunctionType;

namespace neodrive {
namespace planning {

namespace {
const auto& collision_risk_check_config_{config::PlanningConfig::Instance()
                                             ->planning_research_config()
                                             .collision_risk_check};

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

SpeedYieldParalleledObsDecider::SpeedYieldParalleledObsDecider() {
  name_ = "SpeedYieldParalleledObsDecider";
}

SpeedYieldParalleledObsDecider::~SpeedYieldParalleledObsDecider() { Reset(); }

ErrorCode SpeedYieldParalleledObsDecider::Execute(TaskInfo& task_info) {
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

void SpeedYieldParalleledObsDecider::SaveTaskResults(TaskInfo& task_info) {
  if (update_limited_speed_) {
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

bool SpeedYieldParalleledObsDecider::Process(TaskInfo& task_info) {
  if (!Init(task_info)) {
    LOG_ERROR("Init failed.");
    return false;
  }

  if (DynamicObstacleParallelCheck(task_info) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("DynamicObstacleParallelCheck failed.");
    return false;
  }

  return true;
}

bool SpeedYieldParalleledObsDecider::Init(TaskInfo& task_info) {
  adc_current_l_ = task_info.curr_sl().l();
  adc_current_s_ = task_info.curr_sl().s();
  adc_front_edge_s_ =
      adc_current_s_ + VehicleParam::Instance()->front_edge_to_center();
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
  if (!update_limited_speed_) {
    last_limited_speed_ = adc_current_v_;
  }
  update_limited_speed_ = false;
  limited_speed_ = std::numeric_limits<double>::infinity();
  limited_deceleration_ = 0.0;

  auto& inside_data = task_info.current_frame()->inside_planner_data();
  adc_polygon_ = VehicleParam::Instance()->get_adc_polygon(
      {inside_data.vel_x, inside_data.vel_y}, inside_data.vel_heading, 0.0, 0.0,
      0.0);
  if (!InitCollisionCheckArea(task_info.adc_boundary_origin())) {
    LOG_ERROR("Fail to init collision check area.");
    return false;
  }

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

bool SpeedYieldParalleledObsDecider::InitCollisionCheckArea(
    const Boundary& adc_boundary) {
  adc_left_collision_check_area_ = std::move(
      Boundary{adc_boundary.start_s(), adc_boundary.end_s() + 2.0,
               adc_boundary.end_l(), adc_boundary.end_l() + kSideAreaWidth});

  adc_right_collision_check_area_ = std::move(Boundary{
      adc_boundary.start_s(), adc_boundary.end_s(),
      adc_boundary.start_l() - kSideAreaWidth, adc_boundary.start_l()});

  LOG_INFO(
      "adc boundary, s_s, e_s, s_l, e_l: {:.2f}, {:.2f},{:.2f}, {:.2f}; "
      "adc_current_v {:.3f}",
      adc_boundary.start_s(), adc_boundary.end_s(), adc_boundary.start_l(),
      adc_boundary.end_l(), adc_current_v_);

  return true;
}

bool SpeedYieldParalleledObsDecider::DataCheck(TaskInfo& task_info) {
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

  const auto& dynamic_obstacles_decision =
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.dynamic_obstacles_decision;

  for (const auto& obs_decision : dynamic_obstacles_decision) {
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

void SpeedYieldParalleledObsDecider::GetCollideDecisionObs(
    TaskInfo& task_info) {
  has_collision_dynamic_obs_.clear();
  const auto& dynamic_obs_collision_info =
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.dynamic_obstacles_decision;
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

ErrorCode SpeedYieldParalleledObsDecider::DynamicObstacleParallelCheck(
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
  // CheckIfDynamicToStatic(task_info);

  LOG_INFO("___dynamic_obstacle_parallel_check_infos___:");
  for (std::size_t i = 0; i < dynamic_obs_vec.size(); ++i) {
    if (dynamic_obs_vec[i] == nullptr) {
      continue;
    }

    // UpdataObsLongInfo(task_info, dynamic_obs_vec[i]);
    // UpdataObsCutinInfo(task_info, dynamic_obs_vec[i]);
    double heading_diff = normalize_angle(
        dynamic_obs_vec[i]->velocity_heading() - inside_data.vel_heading);
    if (std::abs(heading_diff) > M_PI_2) {
      continue;
    }
    // UpdataObsHeadingInfo(task_info, dynamic_obs_vec[i]);

    LOG_INFO("deal paralleled obs [{}]: ", dynamic_obs_vec[i]->id());
    if (AdcLeftParallelCheck(inside_data, dynamic_obs_vec[i], outside_data)) {
      continue;
    }
    if (AdcRightParallelCheck(inside_data, dynamic_obs_vec[i], outside_data)) {
      continue;
    }
  }

  return ErrorCode::PLANNING_OK;
}

bool SpeedYieldParalleledObsDecider::AdcLeftParallelCheck(
    const InsidePlannerData& inside_data, const Obstacle* const obs,
    OutsidePlannerData* const outside_data) {
  auto& left_dynamic_obstacles_data =
      outside_data->speed_obstacle_context.dynamic_obstacles_collision_risk_data
          .left_dynamic_obstacles_data;

  // 1.deal every dynamic obstacle and update left_dynamic_obstacles_data.
  // if (!LeftSidePrepareData(inside_data, adc_left_collision_check_area_, obs,
  //                          left_dynamic_obstacles_data)) {
  //   return false;
  // }

  if (IsSmallObs(obs)) {
    // DealSideSmallObs(obs, left_dynamic_obstacles_data);
  } else {
    // DealSideBigObs(obs, left_dynamic_obstacles_data);
  }

  // 2.add result of left dynamic obs to all result.
  LOG_INFO("left_dynamic_obstacles_data.size: [{}]",
           left_dynamic_obstacles_data.size());
  for (auto& [id, check_data] : left_dynamic_obstacles_data) {
  }

  return true;
}

bool SpeedYieldParalleledObsDecider::AdcRightParallelCheck(
    const InsidePlannerData& inside_data, const Obstacle* const obs,
    OutsidePlannerData* const outside_data) {
  auto& right_dynamic_obstacles_data =
      outside_data->speed_obstacle_context.dynamic_obstacles_collision_risk_data
          .right_dynamic_obstacles_data;

  // 1.deal every dynamic obstacle and update right_dynamic_obstacles_data.
  // if (!RightSidePrepareData(inside_data, adc_right_collision_check_area_,
  // obs,
  //                           right_dynamic_obstacles_data)) {
  //   return false;
  // }

  if (IsSmallObs(obs)) {
    // DealSideSmallObs(obs, right_dynamic_obstacles_data);
  } else {
    // DealSideBigObs(obs, right_dynamic_obstacles_data);
  }

  // 2.add result of right dynamic obs to all result.
  LOG_INFO("right_dynamic_obstacles_data.size: [{}]",
           right_dynamic_obstacles_data.size());
  for (auto& [id, check_data] : right_dynamic_obstacles_data) {
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive
