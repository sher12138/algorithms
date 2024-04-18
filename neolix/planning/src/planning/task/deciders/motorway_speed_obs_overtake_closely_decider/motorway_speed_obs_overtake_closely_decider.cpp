#include "motorway_speed_obs_overtake_closely_decider.h"

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

MotorwaySpeedObsOvertakeCloselyDecider::
    MotorwaySpeedObsOvertakeCloselyDecider() {
  name_ = "MotorwaySpeedObsOvertakeCloselyDecider";
}

MotorwaySpeedObsOvertakeCloselyDecider::
    ~MotorwaySpeedObsOvertakeCloselyDecider() {
  Reset();
}

ErrorCode MotorwaySpeedObsOvertakeCloselyDecider::Execute(TaskInfo& task_info) {
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

void MotorwaySpeedObsOvertakeCloselyDecider::SaveTaskResults(
    TaskInfo& task_info) {
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

bool MotorwaySpeedObsOvertakeCloselyDecider::Process(TaskInfo& task_info) {
  if (!Init(task_info)) {
    LOG_ERROR("Init failed.");
    return false;
  }

  // Check every dynamic obstcle with every collision check area to see if
  // there are obstacles with collision check result. Ignore these obstacles
  // which are safe but will case hard brake in IDM, and then give target
  // limited speed.
  if (DynamicObstacleCollisionRiskCheck(task_info) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("DynamicObstacleCollisionRiskCheck failed.");
    return false;
  }

  return true;
}

bool MotorwaySpeedObsOvertakeCloselyDecider::Init(TaskInfo& task_info) {
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

  ClearObsHistoryInfo();

  if (!InitCollisionCheckArea(task_info.reference_line(), inside_data,
                              task_info.adc_boundary_origin())) {
    LOG_ERROR("Fail to init collision check area.");
    return false;
  }

  // Judge if ego car go straight or will turn.
  if (!speed_planner_common::JudgeDirectionCarWillGo(
          task_info.reference_line(),
          task_info.current_frame()->inside_planner_data(),
          car_go_direction_)) {
    LOG_ERROR("Fail to judge the direction car will go.");
    return false;
  }

  if (task_info.current_frame()->outside_planner_data().path_data == nullptr) {
    LOG_INFO("path_data is nullptr! ");
    return false;
  }

  if (!InitForntCollisionCheckAreaAlongPath(
          task_info.current_frame()->inside_planner_data(),
          *(task_info.current_frame()->outside_planner_data().path_data))) {
    LOG_ERROR("InitForntCollisionCheckAreaAlongPath err.");
    return false;
  }

  GetCollideDecisionObs(task_info);

  return true;
}

bool MotorwaySpeedObsOvertakeCloselyDecider::DataCheck(TaskInfo& task_info) {
  if (task_info.current_frame() == nullptr) {
    LOG_ERROR("current_frame is nullptr.");
    return false;
  }

  if (task_info.last_frame() == nullptr) {
    LOG_ERROR("last_frame is nullptr.");
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

void MotorwaySpeedObsOvertakeCloselyDecider::ClearObsHistoryInfo() {
  // clear obs_s_info_
  for (auto& iter : obs_s_info_) {
    iter.second.lost_cnt++;
  }
  auto obs_s_info_tmp = obs_s_info_;
  for (auto& iter : obs_s_info_) {
    if (iter.second.lost_cnt > 4) {
      obs_s_info_tmp.erase(iter.first);
    }
  }
  std::swap(obs_s_info_, obs_s_info_tmp);
  LOG_INFO("obs_s_info_ num {}", obs_s_info_.size());

  // clear obs_cutin_info_
  for (auto& iter : obs_cutin_info_) {
    iter.second.lost_cnt++;
  }
  auto obs_cutin_info_tmp = obs_cutin_info_;
  for (auto& iter : obs_cutin_info_) {
    if (iter.second.lost_cnt > 4) {
      obs_cutin_info_tmp.erase(iter.first);
    }
  }
  std::swap(obs_cutin_info_, obs_cutin_info_tmp);
  LOG_INFO("obs_cutin_info_ num {}", obs_cutin_info_.size());

  // clear obs_heading_info_
  for (auto& iter : obs_heading_info_) {
    iter.second.lost_cnt++;
  }
  auto obs_heading_info_tmp = obs_heading_info_;
  for (auto& iter : obs_heading_info_) {
    if (iter.second.lost_cnt > 10) {
      obs_heading_info_tmp.erase(iter.first);
    }
  }
  std::swap(obs_heading_info_, obs_heading_info_tmp);
  LOG_INFO("obs_heading_info_ num {}", obs_heading_info_.size());
}

bool MotorwaySpeedObsOvertakeCloselyDecider::
    InitForntCollisionCheckAreaAlongPath(const InsidePlannerData& inside_data,
                                         const PathData& path_data) {
  if (path_data.path().num_of_points() < 2) {
    LOG_ERROR("path_data.path().num_of_points({}) < 2.",
              path_data.path().num_of_points());
    return false;
  }

  double check_dis = collision_risk_check_config_.front_area_s_length;
  const auto& veh_path = path_data.path().path_points();
  double en_large_buffer{collision_risk_check_config_.front_area_l_buffer};
  double front_en_large_buffer{en_large_buffer};
  double sum_s = 0.;
  double delta_s = 0.0;

  adc_boundaries_front_check_.clear();
  for (std::size_t i = 0; i < veh_path.size(); ++i) {
    if (i >= 1) {
      delta_s = std::sqrt(std::pow(veh_path[i].x() - veh_path[i - 1].x(), 2) +
                          std::pow(veh_path[i].y() - veh_path[i - 1].y(), 2));
      sum_s += delta_s;
      if (sum_s > check_dis) {
        break;
      }
    }
    const auto adc_box = VehicleParam::Instance()->get_adc_bounding_box(
        {veh_path[i].x(), veh_path[i].y()}, veh_path[i].theta(),
        en_large_buffer, front_en_large_buffer, 0.0);
    if (!Utility::check_area(adc_box)) {
      LOG_ERROR("build adc bounding box from path point at s[{}] failed",
                veh_path[i].s());
      return false;
    }
    adc_boundaries_front_check_.emplace_back(adc_box);
  }

  return true;
}

bool MotorwaySpeedObsOvertakeCloselyDecider::InitCollisionCheckArea(
    const ReferenceLinePtr& ref_ptr, const InsidePlannerData& inside_data,
    const Boundary& adc_boundary) {
  adc_back_collision_check_area_ = std::move(Boundary{
      adc_boundary.start_s() - collision_risk_check_config_.back_area_s_length,
      adc_boundary.start_s(),
      adc_boundary.start_l() - collision_risk_check_config_.back_area_l_buffer,
      adc_boundary.end_l() + collision_risk_check_config_.back_area_l_buffer});

  adc_left_collision_check_area_ = std::move(Boundary{
      adc_boundary.start_s(), adc_boundary.end_s(), adc_boundary.end_l(),
      adc_boundary.end_l() + collision_risk_check_config_.side_area_l_width});

  adc_right_collision_check_area_ = std::move(Boundary{
      adc_boundary.start_s(), adc_boundary.end_s(),
      adc_boundary.start_l() - collision_risk_check_config_.side_area_l_width,
      adc_boundary.start_l()});

  LOG_INFO(
      "adc boundary, s_s, e_s, s_l, e_l: {:.2f}, {:.2f},{:.2f}, {:.2f}; "
      "adc_current_v {:.3f}",
      adc_boundary.start_s(), adc_boundary.end_s(), adc_boundary.start_l(),
      adc_boundary.end_l(), adc_current_v_);

  return true;
}

ErrorCode
MotorwaySpeedObsOvertakeCloselyDecider::DynamicObstacleCollisionRiskCheck(
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
  CheckIfDynamicToStatic(task_info);

  LOG_INFO("___dynamic_obstacle_collision_check_infos___:");
  for (std::size_t i = 0; i < dynamic_obs_vec.size(); ++i) {
    if (dynamic_obs_vec[i] == nullptr) {
      continue;
    }

    UpdataObsLongInfo(task_info, dynamic_obs_vec[i]);
    UpdataObsCutinInfo(task_info, dynamic_obs_vec[i]);
    double heading_diff = normalize_angle(
        dynamic_obs_vec[i]->velocity_heading() - inside_data.vel_heading);
    if (std::abs(heading_diff) > M_PI_2) {
      continue;
    }
    UpdataObsHeadingInfo(task_info, dynamic_obs_vec[i]);

    if (!IsEasyHardBrakeObs(dynamic_obs_vec[i])) {
      continue;
    }

    LOG_INFO("obs [{}] deal in collision risk: ", dynamic_obs_vec[i]->id());
    if (AdcBackCollisionCheck(inside_data, dynamic_obs_vec[i], outside_data)) {
      continue;
    }
    if (AdcLeftCollisionCheck(inside_data, dynamic_obs_vec[i], outside_data)) {
      continue;
    }
    if (AdcRightCollisionCheck(inside_data, dynamic_obs_vec[i], outside_data)) {
      continue;
    }
    if (AdcFrontCollisionCheck(inside_data, dynamic_obs_vec[i],
                               outside_data->path_data->path().path_points(),
                               outside_data)) {
      continue;
    }
  }

  return ErrorCode::PLANNING_OK;
}

void MotorwaySpeedObsOvertakeCloselyDecider::CheckIfDynamicToStatic(
    TaskInfo& task_info) {
  const auto& static_obs_vec = task_info.current_frame()
                                   ->planning_data()
                                   .decision_data()
                                   .static_obstacle();
  for (std::size_t i = 0; i < static_obs_vec.size(); ++i) {
    if (static_obs_vec[i] == nullptr) {
      continue;
    }

    if (obs_heading_info_.find(static_obs_vec[i]->id()) !=
        obs_heading_info_.end()) {
      UpdataObsHeadingInfo(task_info, static_obs_vec[i]);
      LOG_INFO("obs [{}] change from dynamic to static, heading {:.4f}",
               static_obs_vec[i]->id(), static_obs_vec[i]->heading());
    }
  }
}

bool MotorwaySpeedObsOvertakeCloselyDecider::AdcRightCollisionCheck(
    const InsidePlannerData& inside_data, const Obstacle* const obs,
    OutsidePlannerData* const outside_data) {
  auto& right_dynamic_obstacles_data =
      outside_data->motorway_speed_obstacle_context
          .dynamic_obstacles_collision_risk_data.right_dynamic_obstacles_data;

  // 1.deal every dynamic obstacle and update right_dynamic_obstacles_data.
  if (!RightSidePrepareData(inside_data, adc_right_collision_check_area_, obs,
                            right_dynamic_obstacles_data)) {
    return false;
  }

  if (IsSmallObs(obs)) {
    DealSideSmallObs(obs, right_dynamic_obstacles_data);
  } else {
    DealSideBigObs(obs, right_dynamic_obstacles_data);
  }

  // 2.add result of right dynamic obs to all result.
  LOG_INFO("right_dynamic_obstacles_data.size: [{}]",
           right_dynamic_obstacles_data.size());
  auto& dp_st_map_ignore_dynamic_obs_id =
      outside_data->motorway_speed_obstacle_context.ignore_dynamic_obs_id;
  auto& speed_backup_ignore_dynamic_obs_id =
      outside_data->motorway_speed_obstacle_context
          .speed_backup_ignore_dynamic_obs_id;
  for (auto& [id, check_data] : right_dynamic_obstacles_data) {
    if (check_data.is_ignore_obs_in_dp_st) {
      dp_st_map_ignore_dynamic_obs_id.insert(id);
    }
    if (check_data.is_ignore_obs_in_backup) {
      speed_backup_ignore_dynamic_obs_id.insert(id);
    }
  }
  return true;
}

bool MotorwaySpeedObsOvertakeCloselyDecider::RightSidePrepareData(
    const InsidePlannerData& inside_data,
    const Boundary& right_collision_check_area, const Obstacle* const obs,
    std::unordered_map<int, DynamicObsCollisionCheckData>&
        right_dynamic_obstacles_data) {
  const Boundary& obs_boundary = obs->PolygonBoundary();
  if (!right_collision_check_area.has_overlap(obs_boundary)) {
    return false;
  }

  // Only deal those obs which have same direction and overlap with
  // left_collision_check_area
  auto& side_dynamic_obs_data = right_dynamic_obstacles_data[obs->id()];
  side_dynamic_obs_data.id = obs->id();

  double obs_l_dis = right_collision_check_area.end_l() - obs_boundary.end_l();
  side_dynamic_obs_data.l_dis =
      side_dynamic_obs_data.first_time
          ? obs_l_dis
          : (side_dynamic_obs_data.l_dis *
                 (1.0 - collision_risk_check_config_.obs_l_dis_filter_ratio) +
             obs_l_dis * collision_risk_check_config_.obs_l_dis_filter_ratio);

  double heading_diff =
      normalize_angle(obs->velocity_heading() - inside_data.vel_heading);
  side_dynamic_obs_data.heading_diff =
      side_dynamic_obs_data.first_time
          ? heading_diff
          : (side_dynamic_obs_data.heading_diff *
                 (1.0 - collision_risk_check_config_.obs_l_dis_filter_ratio) +
             heading_diff *
                 collision_risk_check_config_.obs_l_dis_filter_ratio);

  side_dynamic_obs_data.first_time = false;
  LOG_INFO(
      "obs [{}], s_s, e_s, s_l, e_l: {:.2f}, {:.2f}, {:.2f}, {:.2f}; l_dis, "
      "heading_diff, speed: {:.3f}, {:.3f}, {:.3f}",
      obs->id(), obs_boundary.start_s(), obs_boundary.end_s(),
      obs_boundary.start_l(), obs_boundary.end_l(), side_dynamic_obs_data.l_dis,
      side_dynamic_obs_data.heading_diff, obs->speed());

  return true;
}

bool MotorwaySpeedObsOvertakeCloselyDecider::AdcFrontCollisionCheck(
    const InsidePlannerData& inside_data, const Obstacle* const obs,
    const std::vector<PathPoint>& path_points,
    OutsidePlannerData* const outside_data) {
  if (adc_boundaries_front_check_.size() < 2) {
    LOG_ERROR("adc_boundaries_front_check_ size < 2.");
    return false;
  }

  auto& front_dynamic_obstacles_data =
      outside_data->motorway_speed_obstacle_context
          .dynamic_obstacles_collision_risk_data.front_dynamic_obstacles_data;

  // deal collision risk
  bool find_low{false};
  std::size_t low_index = 0;
  const auto& obs_polygon = obs->polygon();
  FindLowCollisionPointWithPolygon(obs_polygon, find_low, low_index);
  if (find_low) {
    double lower_point_heading_diff = normalize_angle(
        obs->velocity_heading() - path_points[low_index].theta());
    // filter across
    if (std::abs(lower_point_heading_diff) >
        collision_risk_check_config_.front_heading_diff_threshold) {
      return false;
    }

    DealFrontObs(obs, path_points, low_index, outside_data);
  }

  // add result of front dynamic obs to all result.
  LOG_INFO("front_dynamic_obstacles_data, size: [{}]",
           front_dynamic_obstacles_data.size());
  auto& dp_st_map_ignore_dynamic_obs_id =
      outside_data->motorway_speed_obstacle_context.ignore_dynamic_obs_id;
  auto& speed_backup_ignore_dynamic_obs_id =
      outside_data->motorway_speed_obstacle_context
          .speed_backup_ignore_dynamic_obs_id;
  for (auto& [id, check_data] : front_dynamic_obstacles_data) {
    if (check_data.is_ignore_obs_in_dp_st) {
      dp_st_map_ignore_dynamic_obs_id.insert(id);
    }
    if (check_data.is_ignore_obs_in_backup) {
      speed_backup_ignore_dynamic_obs_id.insert(id);
    }
  }

  return true;
}

void MotorwaySpeedObsOvertakeCloselyDecider::DealFrontObs(
    const Obstacle* const obs, const std::vector<PathPoint>& path_points,
    std::size_t low_index, OutsidePlannerData* const outside_data) {
  double lower_point_heading_diff =
      normalize_angle(obs->velocity_heading() - path_points[low_index].theta());
  double obs_lower_project_speed =
      obs->speed() * std::cos(lower_point_heading_diff);
  double obs_project_speed_safe_delta =
      speed_planner_common::CalSafeSpeedDeltaParallelAdc(
          lower_point_heading_diff);
  double lower_point_s = path_points[low_index].s();
  LOG_INFO(
      "front obs id, type:[{}],[{}]; lower_point_heading_diff, "
      "obs_lower_project_speed, obs_project_speed_safe_delta, lower_point_s: "
      "{:.2f}, {:.2f}, {:.2f}, {:.2f}; adc_current_v_: {:.3f}",
      obs->id(), static_cast<int>(obs->type()), lower_point_heading_diff,
      obs_lower_project_speed, obs_project_speed_safe_delta, lower_point_s,
      adc_current_v_);
  auto& front_dynamic_obs_data = outside_data->motorway_speed_obstacle_context
                                     .dynamic_obstacles_collision_risk_data
                                     .front_dynamic_obstacles_data[obs->id()];
  front_dynamic_obs_data.id = obs->id();

  if (obs_lower_project_speed < adc_current_v_) {
    bool is_overtake_obs{false};
    GetObsLongResult(obs, is_overtake_obs);
    double obs_ttc = lower_point_s / (adc_current_v_ - obs_lower_project_speed);
    if ((obs_ttc < collision_risk_check_config_.front_safe_ttc_threshold) ||
        (car_go_direction_ != GoDirectionType::GO_STRAIGHT) ||
        !is_overtake_obs) {
      LOG_INFO("front obs [{}] slower and has risk, use iter cipv.", obs->id());
      return;
    }
  }

  LOG_INFO("ignore front obs [{}] front and safe.", obs->id());
  front_dynamic_obs_data.is_ignore_obs_in_dp_st = true;

  if (obs_lower_project_speed - obs_project_speed_safe_delta > adc_current_v_) {
    return;
  }

  double adc_safe_speed =
      obs_lower_project_speed * collision_risk_check_config_.speed_limit_ratio;
  if (adc_current_v_ > adc_safe_speed) {
    update_limited_speed_ = true;
    limited_speed_ =
        std::min(limited_speed_,
                 last_limited_speed_ -
                     kPlanningCycleTime *
                         collision_risk_check_config_.front_deceleration);
    limited_speed_ = std::max(limited_speed_, adc_safe_speed);
    limited_deceleration_ =
        std::min(limited_deceleration_,
                 -collision_risk_check_config_.front_deceleration * 1.0);
  }

  LOG_INFO(
      "base front obs [{}] get limited_speed_{:.2f}, limited_deceleration_ "
      "{:.2f}",
      obs->id(), limited_speed_, limited_deceleration_);
  return;
}

void MotorwaySpeedObsOvertakeCloselyDecider::FindLowCollisionPointWithPolygon(
    const Polygon2d& obstacle_box, bool& find_low,
    std::size_t& low_index) const {
  find_low = false;
  low_index = 0;
  while (low_index < adc_boundaries_front_check_.size()) {
    if (find_low) {
      break;
    }

    if (!obstacle_box.has_overlap(
            Polygon2d(adc_boundaries_front_check_[low_index]))) {
      (low_index) += 2;
    } else {
      find_low = true;
      if (low_index > 0) {
        if (obstacle_box.has_overlap(
                Polygon2d(adc_boundaries_front_check_[low_index - 1]))) {
          low_index = low_index - 1;
        }
      }
    }
  }
}

bool MotorwaySpeedObsOvertakeCloselyDecider::AdcBackCollisionCheck(
    const InsidePlannerData& inside_data, const Obstacle* const obs,
    OutsidePlannerData* const outside_data) {
  auto& dp_st_map_ignore_dynamic_obs_id =
      outside_data->motorway_speed_obstacle_context.ignore_dynamic_obs_id;

  // Only same direction and obs is totally behind adc,then calculate
  // min_dis and min_ttc.
  if (adc_back_collision_check_area_.has_overlap(obs->PolygonBoundary()) &&
      (obs->PolygonBoundary().end_s() <
       adc_back_collision_check_area_.end_s())) {
    dp_st_map_ignore_dynamic_obs_id.insert(obs->id());
    LOG_INFO("ignore obs [{}] back of adc.", obs->id());
    return true;
  }

  return false;
}

bool MotorwaySpeedObsOvertakeCloselyDecider::AdcLeftCollisionCheck(
    const InsidePlannerData& inside_data, const Obstacle* const obs,
    OutsidePlannerData* const outside_data) {
  auto& left_dynamic_obstacles_data =
      outside_data->motorway_speed_obstacle_context
          .dynamic_obstacles_collision_risk_data.left_dynamic_obstacles_data;

  // 1.deal every dynamic obstacle and update left_dynamic_obstacles_data.
  if (!LeftSidePrepareData(inside_data, adc_left_collision_check_area_, obs,
                           left_dynamic_obstacles_data)) {
    return false;
  }

  if (IsSmallObs(obs)) {
    DealSideSmallObs(obs, left_dynamic_obstacles_data);
  } else {
    DealSideBigObs(obs, left_dynamic_obstacles_data);
  }

  // 2.add result of left dynamic obs to all result.
  LOG_INFO("left_dynamic_obstacles_data.size: [{}]",
           left_dynamic_obstacles_data.size());
  auto& dp_st_map_ignore_dynamic_obs_id =
      outside_data->motorway_speed_obstacle_context.ignore_dynamic_obs_id;
  auto& speed_backup_ignore_dynamic_obs_id =
      outside_data->motorway_speed_obstacle_context
          .speed_backup_ignore_dynamic_obs_id;
  for (auto& [id, check_data] : left_dynamic_obstacles_data) {
    if (check_data.is_ignore_obs_in_dp_st) {
      dp_st_map_ignore_dynamic_obs_id.insert(id);
    }
    if (check_data.is_ignore_obs_in_backup) {
      speed_backup_ignore_dynamic_obs_id.insert(id);
    }
  }

  return true;
}

bool MotorwaySpeedObsOvertakeCloselyDecider::LeftSidePrepareData(
    const InsidePlannerData& inside_data,
    const Boundary& left_collision_check_area, const Obstacle* const obs,
    std::unordered_map<int, DynamicObsCollisionCheckData>&
        left_dynamic_obstacles_data) {
  const Boundary& obs_boundary = obs->PolygonBoundary();
  if (!left_collision_check_area.has_overlap(obs_boundary)) {
    return false;
  }

  // Only deal those obs which have same direction and overlap with
  // left_collision_check_area
  auto& side_dynamic_obs_data = left_dynamic_obstacles_data[obs->id()];
  side_dynamic_obs_data.id = obs->id();

  double obs_l_dis =
      obs_boundary.start_l() - left_collision_check_area.start_l();
  side_dynamic_obs_data.l_dis =
      side_dynamic_obs_data.first_time
          ? obs_l_dis
          : (side_dynamic_obs_data.l_dis *
                 (1.0 - collision_risk_check_config_.obs_l_dis_filter_ratio) +
             obs_l_dis * collision_risk_check_config_.obs_l_dis_filter_ratio);

  double heading_diff =
      normalize_angle(obs->velocity_heading() - inside_data.vel_heading);
  side_dynamic_obs_data.heading_diff =
      side_dynamic_obs_data.first_time
          ? heading_diff
          : (side_dynamic_obs_data.heading_diff *
                 (1.0 - collision_risk_check_config_.obs_l_dis_filter_ratio) +
             heading_diff *
                 collision_risk_check_config_.obs_l_dis_filter_ratio);

  side_dynamic_obs_data.first_time = false;
  LOG_INFO(
      "side obs [{}], s_s, e_s, s_l, e_l: {:.2f}, {:.2f}, {:.2f}, {:.2f}; "
      "l_dis, heading_diff, speed: {:.3f}, {:.3f}, {:.3f}",
      obs->id(), obs_boundary.start_s(), obs_boundary.end_s(),
      obs_boundary.start_l(), obs_boundary.end_l(), side_dynamic_obs_data.l_dis,
      side_dynamic_obs_data.heading_diff, obs->speed());

  return true;
}

void MotorwaySpeedObsOvertakeCloselyDecider::DealSideBigObs(
    const Obstacle* const obs,
    std::unordered_map<int, DynamicObsCollisionCheckData>&
        side_dynamic_obstacles_data) {
  auto& side_dynamic_obs_data = side_dynamic_obstacles_data[obs->id()];
  const Boundary& obs_boundary = obs->PolygonBoundary();

  bool is_overtake_obs{false};
  GetObsLongResult(obs, is_overtake_obs);
  bool is_turning_obs{false};
  GetObsTurningInfo(obs, is_turning_obs);

  // case1: obs turn
  if (is_turning_obs ||
      (side_dynamic_obs_data.heading_diff >
       collision_risk_check_config_.side_heading_diff_threshold) ||
      (side_dynamic_obs_data.l_dis <
       collision_risk_check_config_.side_vehicle_safe_l_dis)) {
    LOG_INFO(
        "side obs [{}] is turning or has big heading diff or lateral close, "
        "deal in iter cipv.",
        obs->id());
    return;
  }

  if (is_overtake_obs && (GoDirectionType::GO_STRAIGHT == car_go_direction_) &&
      (has_collision_dynamic_obs_[obs->id()].lower_first_project_v >
       adc_current_v_)) {
    LOG_INFO("ignore side obs [{}] faster.", obs->id());
    side_dynamic_obs_data.is_ignore_obs_in_dp_st = true;
    double adc_safe_speed =
        has_collision_dynamic_obs_[obs->id()].lower_first_project_v *
        collision_risk_check_config_.speed_limit_ratio;
    if (adc_current_v_ > adc_safe_speed) {
      update_limited_speed_ = true;
      limited_speed_ =
          std::min(limited_speed_,
                   last_limited_speed_ -
                       kPlanningCycleTime *
                           collision_risk_check_config_.side_deceleration);
      limited_speed_ = std::max(limited_speed_, adc_safe_speed);
      limited_deceleration_ =
          std::min(limited_deceleration_,
                   -collision_risk_check_config_.side_deceleration * 1.0);
      LOG_INFO(
          "side obs [{}] not safe enough, adc_safe_speed {:.3f}, "
          "limited_speed_ {:.3f}, limited_deceleration_ {:.3f}",
          obs->id(), adc_safe_speed, limited_speed_, limited_deceleration_);
    }

    LOG_INFO("ignore side obs [{}], limit speed by l_dis.", obs->id());
    double target_v = side_dynamic_obs_data.l_dis *
                      collision_risk_check_config_.side_l_speed_limit_ratio;
    if (adc_current_v_ > target_v) {
      update_limited_speed_ = true;
      limited_speed_ =
          std::min(limited_speed_,
                   last_limited_speed_ -
                       kPlanningCycleTime *
                           collision_risk_check_config_.side_deceleration);
      limited_speed_ = std::max(limited_speed_, target_v);
      limited_deceleration_ =
          std::min(limited_deceleration_,
                   -collision_risk_check_config_.side_deceleration * 1.0);
      LOG_INFO(
          "side obs [{}] l_dis [{:.2f}] , target_v {:.3f}; limited_speed_ "
          "{:.3f}, limited_deceleration_ {:.3f}.",
          obs->id(), side_dynamic_obs_data.l_dis, target_v, limited_speed_,
          limited_deceleration_);
    }
  }
}

void MotorwaySpeedObsOvertakeCloselyDecider::DealSideSmallObs(
    const Obstacle* const obs,
    std::unordered_map<int, DynamicObsCollisionCheckData>&
        side_dynamic_obstacles_data) {
  auto& side_dynamic_obs_data = side_dynamic_obstacles_data[obs->id()];
  const Boundary& obs_boundary = obs->PolygonBoundary();

  LatResult lat_res{LatResult::OVERTAKE};
  GetObsLatResult(obs, lat_res);

  if (LatResult::ACROSS == lat_res) {
    LOG_INFO("side obs [{}] almost across, deal in main and backup planner.",
             obs->id());
    return;
  } else if (LatResult::CUTIN == lat_res) {
    LOG_INFO("side obs [{}] cutin, only deal in main planner.", obs->id());
    side_dynamic_obs_data.is_ignore_obs_in_backup = true;
    return;
  }

  ObsOvertakeAdc(obs, side_dynamic_obs_data);

  if (!side_dynamic_obs_data.is_ignore_obs_in_dp_st) {
    AdcOvertakeObs(obs, side_dynamic_obs_data);
  }
}

void MotorwaySpeedObsOvertakeCloselyDecider::ObsOvertakeAdc(
    const Obstacle* const obs,
    DynamicObsCollisionCheckData& side_dynamic_obs_data) {
  bool is_overtake_obs{false};
  GetObsLongResult(obs, is_overtake_obs);

  if (has_collision_dynamic_obs_[obs->id()].lower_first_project_v >
      adc_current_v_) {
    LOG_INFO("ignore side obs [{}] faster.", obs->id());
    side_dynamic_obs_data.is_ignore_obs_in_dp_st = true;
  } else if ((GoDirectionType::GO_STRAIGHT == car_go_direction_) &&
             (is_overtake_obs ||
              obs->speed() >
                  adc_current_v_ * collision_risk_check_config_
                                       .side_obs_speed_filter_ratio)) {
    LOG_INFO("ignore side obs [{}] straight and overtake.", obs->id());
    side_dynamic_obs_data.is_ignore_obs_in_dp_st = true;
  }

  if (side_dynamic_obs_data.is_ignore_obs_in_dp_st) {
    double adc_safe_speed =
        has_collision_dynamic_obs_[obs->id()].lower_first_project_v *
        collision_risk_check_config_.speed_limit_ratio;
    if (adc_current_v_ > adc_safe_speed) {
      update_limited_speed_ = true;
      limited_speed_ =
          std::min(limited_speed_,
                   last_limited_speed_ -
                       kPlanningCycleTime *
                           collision_risk_check_config_.side_deceleration);
      limited_speed_ = std::max(limited_speed_, adc_safe_speed);
      limited_deceleration_ =
          std::min(limited_deceleration_,
                   -collision_risk_check_config_.side_deceleration * 1.0);
      LOG_INFO(
          "side obs [{}] not safe enough, adc_safe_speed {:.3f}; "
          "limited_speed_ {:.3f}, limited_deceleration_ {:.3f}",
          obs->id(), adc_safe_speed, limited_speed_, limited_deceleration_);
    }
  }
}

void MotorwaySpeedObsOvertakeCloselyDecider::AdcOvertakeObs(
    const Obstacle* const obs,
    DynamicObsCollisionCheckData& side_dynamic_obs_data) {
  if ((GoDirectionType::GO_STRAIGHT == car_go_direction_) &&
      (obs->PolygonBoundary().end_s() < adc_front_edge_s_)) {
    LOG_INFO("ignore side obs [{}] totally behind straight adc front edge.",
             obs->id());
    side_dynamic_obs_data.is_ignore_obs_in_dp_st = true;
    return;
  }

  if (side_dynamic_obs_data.l_dis <
      collision_risk_check_config_.side_vehicle_safe_l_dis) {
    LOG_INFO("side obs [{}] slow and l_dis close, use iter cipv.", obs->id());
    return;
  }

  LOG_INFO("ignore side obs [{}], limit speed by l_dis.", obs->id());
  side_dynamic_obs_data.is_ignore_obs_in_dp_st = true;
  double target_v = side_dynamic_obs_data.l_dis *
                    collision_risk_check_config_.side_l_speed_limit_ratio;
  if (adc_current_v_ > target_v) {
    update_limited_speed_ = true;
    limited_speed_ = std::min(
        limited_speed_, last_limited_speed_ -
                            kPlanningCycleTime *
                                collision_risk_check_config_.side_deceleration);
    limited_speed_ = std::max(limited_speed_, target_v);
    limited_deceleration_ =
        std::min(limited_deceleration_,
                 -collision_risk_check_config_.side_deceleration * 1.0);
    LOG_INFO(
        "side obs [{}] l_dis [{:.2f}] , target_v {:.3f}; limited_speed_ "
        "{:.3f}, limited_deceleration_ {:.3f}.",
        obs->id(), side_dynamic_obs_data.l_dis, target_v, limited_speed_,
        limited_deceleration_);
  }
}

void MotorwaySpeedObsOvertakeCloselyDecider::GetCollideDecisionObs(
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

bool MotorwaySpeedObsOvertakeCloselyDecider::IsEasyHardBrakeObs(
    const Obstacle* const obs) {
  if (has_collision_dynamic_obs_.find(obs->id()) ==
      has_collision_dynamic_obs_.end()) {
    return false;
  }
  if (has_collision_dynamic_obs_[obs->id()].risk_obs) {
    LOG_INFO("obs [{}] is risk obs, not check if ignore", obs->id());
    return false;
  }
  if ((has_collision_dynamic_obs_[obs->id()].lower_first_s >
       collision_risk_check_config_.front_area_s_length) ||
      (has_collision_dynamic_obs_[obs->id()].lower_first_t >
       collision_risk_check_config_.front_area_t_length)) {
    return false;
  }

  return true;
}

void MotorwaySpeedObsOvertakeCloselyDecider::UpdataObsLongInfo(
    TaskInfo& task_info, const Obstacle* const obs) {
  auto& obs_s_info = obs_s_info_[obs->id()];
  obs_s_info.relative_s.push_back(obs->PolygonBoundary().start_s() -
                                  adc_current_s_);
  obs_s_info.lost_cnt = 0;
  if (obs_s_info.relative_s.size() >
      collision_risk_check_config_.info_size_relative_s) {
    obs_s_info.relative_s.pop_front();
  }
}

void MotorwaySpeedObsOvertakeCloselyDecider::UpdataObsCutinInfo(
    TaskInfo& task_info, const Obstacle* const obs) {
  auto& obs_cutin_info = obs_cutin_info_[obs->id()];
  obs_cutin_info.lat_dis.push_back(
      speed_planner_common::GetObs2AdcLateralDis(*obs, adc_current_l_));
  obs_cutin_info.long_pos.push_back(obs->center_sl().s());
  obs_cutin_info.lost_cnt = 0;
  if (obs_cutin_info.lat_dis.size() >
      collision_risk_check_config_.info_size_cutin) {
    obs_cutin_info.lat_dis.pop_front();
    obs_cutin_info.long_pos.pop_front();
  }
}

void MotorwaySpeedObsOvertakeCloselyDecider::UpdataObsHeadingInfo(
    TaskInfo& task_info, const Obstacle* const obs) {
  double obs_adc_cartisian_dis = adc_polygon_.distance_to_2(obs->polygon());
  if (IsAdcNearJunction(task_info) && obs_adc_cartisian_dis < 30) {
    if (obs_heading_info_.find(obs->id()) == obs_heading_info_.end()) {
      auto& obs_heading_info = obs_heading_info_[obs->id()];
      obs_heading_info.last_heading = normalize_angle(obs->heading());
      obs_heading_info.lost_cnt = 0;
    } else {
      auto& obs_heading_info = obs_heading_info_[obs->id()];
      obs_heading_info.heading_diff.push_back(
          normalize_angle(obs->heading() - obs_heading_info.last_heading));
      obs_heading_info.accum_heading_diff +=
          obs_heading_info.heading_diff.back();
      if (obs_heading_info.heading_diff.size() >
          collision_risk_check_config_.info_size_heading) {
        obs_heading_info.accum_heading_diff -=
            obs_heading_info.heading_diff.front();
        obs_heading_info.heading_diff.pop_front();
      }
      obs_heading_info.last_heading = normalize_angle(obs->heading());
      obs_heading_info.lost_cnt = 0;
    }
  }
}

bool MotorwaySpeedObsOvertakeCloselyDecider::IsAdcNearJunction(
    TaskInfo& task_info) {
  auto ref_line = task_info.reference_line();
  const auto& junction_list = ref_line->junctions();
  for (const auto& [junction_ptr, overlap] : junction_list) {
    if (!junction_ptr) {
      continue;
    }
    if (overlap.start_s - 10 <= adc_current_s_ &&
        overlap.end_s >= adc_current_s_ &&
        (junction_ptr->Type() !=
         static_cast<uint32_t>(JunctionType::IN_ROAD))) {
      return true;
    }
  }
  return false;
}

void MotorwaySpeedObsOvertakeCloselyDecider::GetObsLongResult(
    const Obstacle* const obs, bool& is_overtake_obs) {
  const auto& obs_s_info = obs_s_info_[obs->id()];
  is_overtake_obs =
      obs_s_info.relative_s.back() - obs_s_info.relative_s.front() >
      collision_risk_check_config_.obs_overtake_dis_threshold;
  LOG_INFO("obs [{}], curr relative_s {:.3f}, first history relative_s {:.3f}",
           obs->id(), obs_s_info.relative_s.back(),
           obs_s_info.relative_s.front());
}

void MotorwaySpeedObsOvertakeCloselyDecider::GetObsLatResult(
    const Obstacle* const obs, LatResult& lat_res) {
  const auto& obs_cutin_info = obs_cutin_info_[obs->id()];
  if (obs_cutin_info.lat_dis.size() < 2) {
    return;
  }
  double tan_lat_and_long = std::abs(
      (obs_cutin_info.lat_dis.front() - obs_cutin_info.lat_dis.back()) /
      (obs_cutin_info.long_pos.back() - obs_cutin_info.long_pos.front()));
  LOG_INFO(
      "obs [{}], history info size {}; curr and history lat_dis: {:.3f}, "
      "{:.3f}; curr and history long_pos: {:.3f}, {:.3f}; tan_lat_and_long: "
      "{:.3f}",
      obs->id(), obs_cutin_info.lat_dis.size(), obs_cutin_info.lat_dis.back(),
      obs_cutin_info.lat_dis.front(), obs_cutin_info.long_pos.back(),
      obs_cutin_info.long_pos.front(), tan_lat_and_long);
  if (tan_lat_and_long < 0.1763) {
    // 10 degree
    lat_res = LatResult::OVERTAKE;
    return;
  }
  if (tan_lat_and_long < 0.4663) {
    // 25 degree
    lat_res = LatResult::CUTIN;
    return;
  }
  lat_res = LatResult::ACROSS;
}

void MotorwaySpeedObsOvertakeCloselyDecider::GetObsTurningInfo(
    const Obstacle* const obs, bool& is_turning_obs) {
  if (obs_heading_info_.find(obs->id()) == obs_heading_info_.end()) {
    return;
  }

  auto& obs_heading_info = obs_heading_info_[obs->id()];
  is_turning_obs = std::abs(obs_heading_info.accum_heading_diff) >
                   collision_risk_check_config_.obs_turning_threshold;
  LOG_INFO("obs [{}], accum_heading_diff {:.3f}", obs->id(),
           obs_heading_info.accum_heading_diff);
}

}  // namespace planning
}  // namespace neodrive
