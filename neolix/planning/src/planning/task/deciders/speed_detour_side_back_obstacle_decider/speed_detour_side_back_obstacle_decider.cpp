#include "speed_detour_side_back_obstacle_decider.h"

#include "src/planning/reference_line/reference_line_util.h"
namespace neodrive {
namespace planning {

SpeedDetourSideBackObstacleDecider::SpeedDetourSideBackObstacleDecider() {
  name_ = "SpeedDetourSideBackObstacleDecider";
}

SpeedDetourSideBackObstacleDecider::~SpeedDetourSideBackObstacleDecider() {
  Reset();
}

ErrorCode SpeedDetourSideBackObstacleDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  if (!Init(task_info)) {
    LOG_ERROR("Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (!Process(task_info)) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

bool SpeedDetourSideBackObstacleDecider::Init(TaskInfo& task_info) {
  const auto& inside_planner_data =
      task_info.current_frame()->inside_planner_data();
  if (task_info.reference_line() == nullptr ||
      task_info.reference_line()->ref_points().size() < 2) {
    LOG_ERROR("reference_line is invalid.");
    return false;
  }
  if (task_info.current_frame()->mutable_planning_data() == nullptr) {
    LOG_ERROR("mutable_planning_data == nullptr.");
    return false;
  }
  if (task_info.current_frame()
          ->mutable_planning_data()
          ->mutable_decision_data() == nullptr) {
    LOG_ERROR("mutable_decision_data == nullptr.");
    return false;
  }
  init_sl_point_ = inside_planner_data.init_sl_point;
  adc_front_edge_s_ =
      init_sl_point_.s() + VehicleParam::Instance()->front_edge_to_center();
  auto heading_diff_adc_ref = normalize_angle(
      task_info.current_frame()->inside_planner_data().vel_heading -
      task_info.curr_referline_pt().heading());
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v *
                   std::cos(heading_diff_adc_ref);
  LOG_INFO("origin velocity: {:.3f}, filter velocity: {:.3f}",
           task_info.current_frame()->inside_planner_data().vel_v,
           adc_current_v_);
  ref_v_ = std::numeric_limits<double>::infinity();
  auto left_lane_bound = task_info.curr_referline_pt().left_lane_bound();
  auto right_lane_bound = task_info.curr_referline_pt().right_lane_bound();
  back_attention_boundary_.set_end_l(
      init_sl_point_.l() +
      std::min(0.8 * VehicleParam::Instance()->width(), left_lane_bound));
  back_attention_boundary_.set_start_l(
      init_sl_point_.l() -
      std::min(0.8 * VehicleParam::Instance()->width(), right_lane_bound));
  back_attention_boundary_.set_start_s(init_sl_point_.s() - 15.0);
  back_attention_boundary_.set_end_s(adc_front_edge_s_);
  LOG_INFO("back_attention_boundary: {:.3f}, {:.3f}, {:.3f}, {:.3f}",
           back_attention_boundary_.start_s(), back_attention_boundary_.end_s(),
           back_attention_boundary_.start_l(),
           back_attention_boundary_.end_l());
  stop_flag_ = false;
  dangerous_obs_.clear();

  return true;
}

bool SpeedDetourSideBackObstacleDecider::Process(TaskInfo& task_info) {
  if (task_info.current_frame()
          ->planning_data()
          .decision_data()
          .dynamic_obstacle()
          .empty()) {
    LOG_INFO("dynamic_obstacles is empty, skip.");
    return true;
  }
  if (!IsLaneDetourArea(task_info)) {
    LOG_INFO("is not static detour, skip.");
    return true;
  }
  if (!GetConflictAreaInfo(task_info)) {
    LOG_INFO("never find conflict area, skip.");
    return true;
  }
  if (!DynamicObsCheck(task_info, task_info.current_frame()
                                      ->planning_data()
                                      .decision_data()
                                      .dynamic_obstacle())) {
    return true;
  }
  STDecision(task_info);
  UpdateSpeedLimit(task_info);
  return true;
}

bool SpeedDetourSideBackObstacleDecider::IsLaneDetourArea(TaskInfo& task_info) {
  const auto& path_points = task_info.current_frame()
                                ->mutable_outside_planner_data()
                                ->path_data->path()
                                .path_points();
  const auto& shrink_bounds_info =
      task_info.last_frame()->outside_planner_data().shrink_half_ego_boundaries;
  const auto& ref_ptr_ = task_info.reference_line();
  SLPoint adc_current_sl = task_info.curr_sl();

  if (has_collide_risk_) {
    has_collide_risk_ = false;
    return true;
  }
  double max_abs_l = adc_current_sl.l();
  for (const auto& pt : path_points) {
    if (pt.s() > 20.0) {
      break;
    }
    const auto& coordinate = pt.coordinate();
    SLPoint sl_pt{};
    if (!ref_ptr_->GetPointInFrenetFrame(pt, &sl_pt)) {
      LOG_ERROR("lane borrow coordinate trans failed!");
      return false;
    }
    max_abs_l = std::max(max_abs_l, sl_pt.l());
  }
  LOG_INFO("current l: {:.4f}, max l: {:.4f}", adc_current_sl.l(), max_abs_l);
  if (max_abs_l >= 0.0 && max_abs_l - adc_current_sl.l() >=
                              0.5 * VehicleParam::Instance()->width()) {
    LOG_INFO("Enter Lane Borrow Area!");
    return true;
  }
  return false;
}

bool SpeedDetourSideBackObstacleDecider::GetConflictAreaInfo(
    TaskInfo& task_info) {
  const auto& bounds_info =
      task_info.current_frame()
          ->outside_planner_data()
          .path_context.original_path_boundary.path_boundary;
  if (bounds_info.empty()) {
    LOG_INFO("bound info empty.");
    return false;
  }
  for (const auto& pt : bounds_info) {
    if (pt.lower_id != -1 && pt.lower_id != 3) {
      if (task_info.curr_sl().l() > pt.lower_point.l()) continue;
      conflict_area_start_s_ = pt.lower_point.s();
      conflict_area_stop_s_ = std::min(
          conflict_area_start_s_,
          GetStopSAtFrenetPath(task_info, std::max(0.0, pt.lower_point.l())));
      LOG_INFO(
          "find conflict area start point at {:.4f}, base obs id: {}, stop at "
          "{:.4f}",
          pt.lower_point.s(), pt.lower_id, conflict_area_stop_s_);
      return true;
    }
  }
  return false;
}

bool SpeedDetourSideBackObstacleDecider::DynamicObsCheck(
    TaskInfo& task_info, const std::vector<Obstacle*>& dynamic_obs_vec) {
  dangerous_obs_.clear();
  const auto& adc_boundaries = task_info.current_frame()
                                   ->outside_planner_data()
                                   .speed_obstacle_context.adc_boundaries;
  const auto& adc_boundary = task_info.current_frame()
                                 ->outside_planner_data()
                                 .path_obstacle_context.adc_boundary;
  const auto& path_accumulated_s =
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.path_accumulated_s;
  if (adc_boundaries.empty() || path_accumulated_s.empty()) {
    LOG_INFO("check array exception.");
    return false;
  }
  for (const auto* obs : dynamic_obs_vec) {
    bool reverse{false};
    auto heading_diff = normalize_angle(
        obs->velocity_heading() -
        task_info.current_frame()->inside_planner_data().vel_heading);
    reverse = (std::abs(heading_diff) > M_PI_2);
    if (reverse) {
      LOG_INFO("obs [{}] is reverse, ignore.", obs->id());
      continue;
    }
    const auto& boundary = obs->PolygonBoundary();
    if (boundary.start_s() < init_sl_point_.s()) {
      bool find_near_obs = false;
      if (IsDynamicObsNeedIgnord(task_info, obs, find_near_obs)) continue;
      if (!BorrowRegionCollideCheck(task_info, obs)) continue;

      double min_relative_time{0.};
      bool collide_with_path = false;
      double s_dis = init_sl_point_.s() - boundary.end_s();
      if (find_near_obs) {
        min_relative_time = s_dis < 0 ? 0.0 : s_dis / obs->speed();
        collide_with_path = true;
      } else {
        min_relative_time = s_dis / (obs->speed() - adc_current_v_);
        double limited_time =
            (conflict_area_start_s_ - boundary.start_s()) / obs->speed() + 1.0;
        if (min_relative_time > limited_time || min_relative_time < 0.0) {
          LOG_INFO("relative time: {:.4f}, conf time: {:.4f}",
                   min_relative_time, limited_time);
          continue;
        }
        collide_with_path = true;
      }
      if (collide_with_path) {
        double collide_s =
            obs->center_sl().s() + obs->speed() * min_relative_time;
        std::size_t min_index{adc_boundaries.size() + 10};
        for (size_t i = 0; i < adc_boundaries.size(); i++) {
          SLPoint center{};
          task_info.reference_line()->GetPointInFrenetFrame(
              adc_boundaries[i].center(), &center);
          if (center.s() > collide_s) {
            min_index = i > 0 ? i - 1 : i;
            break;
          }
        }
        double stop_s = std::max(conflict_area_stop_s_, collide_s - 1.0);
        double obs_pass_time =
            (stop_s - obs->PolygonBoundary().start_s()) / obs->speed();
        double ego_pass_v = std::max(
            0.0, std::min((stop_s - adc_boundary.end_s()) / obs_pass_time,
                          adc_current_v_));
        std::size_t min_s_index_clamp = std::max(
            static_cast<std::size_t>(0),
            std::min(static_cast<std::size_t>(adc_boundaries.size() - 1),
                     std::min(static_cast<std::size_t>(
                                  path_accumulated_s.size() - 1),
                              min_index)));
        dangerous_obs_.push_back({.id = obs->id(),
                                  .obs_boundary = obs->PolygonBoundary(),
                                  .reverse = reverse,
                                  .s = path_accumulated_s[min_s_index_clamp],
                                  .t = min_relative_time,
                                  .ideal_v = ego_pass_v,
                                  .skip = false});
      }
    }
  }
  return true;
}

bool SpeedDetourSideBackObstacleDecider::STDecision(TaskInfo& task_info) {
  if (dangerous_obs_.empty()) {
    LOG_INFO("dangerous obstacles are empty, skip.");
    return true;
  }

  const auto& adc_boundaries = task_info.current_frame()
                                   ->outside_planner_data()
                                   .speed_obstacle_context.adc_boundaries;
  const auto& adc_boundary = task_info.current_frame()
                                 ->outside_planner_data()
                                 .path_obstacle_context.adc_boundary;
  for (auto& info : dangerous_obs_) {
    double adc_will_run_s = info.t * adc_current_v_;
    bool lateral_skip{false};
    std::size_t path_collide_idx = std::max(
        static_cast<std::size_t>(0),
        std::min(static_cast<std::size_t>(PathIndex(task_info, adc_will_run_s)),
                 adc_boundaries.size() - 1));
    Boundary path_collide_boundary{};
    if (!ref_line_util::ComputePolygonBoundary(
            task_info.reference_line(),
            Polygon2d(adc_boundaries[path_collide_idx]),
            &path_collide_boundary)) {
      LOG_ERROR("compute collide boundary failed, use default boundary.");
      continue;
    }
    double first_lateral_error{0.}, second_lateral_error{0.};

    first_lateral_error =
        std::abs(adc_boundary.end_l() - info.obs_boundary.start_l());
    second_lateral_error =
        std::abs(path_collide_boundary.end_l() - info.obs_boundary.start_l());
    if ((first_lateral_error > 1.0) && (second_lateral_error > 1.0)) {
      lateral_skip = true;
      LOG_INFO(
          "Left Borrow: ego will move to left and away from obstacle, "
          "lateral skip.");
    }
    LOG_INFO(
        "first_lateral_error, second_lateral_error, lateral_skip: {:.3f}, "
        "{:.3f}, [{}]",
        first_lateral_error, second_lateral_error, lateral_skip);

    // combine
    info.skip = lateral_skip;
  }

  LOG_INFO("dangerous obstacles size: {}", dangerous_obs_.size());
  for (const auto& info : dangerous_obs_) {
    if (info.skip) {
      continue;
    }
    stop_flag_ = true;
    if (info.ideal_v < ref_v_) {
      ref_v_ = info.ideal_v;
      final_obs_id_ = info.id;
    }
    LOG_INFO(
        "id[{}], reverse[{}], path s {:.3f}, t {:.3f}, v {:.3f}, "
        "skip[{}]",
        info.id, info.reverse, info.s, info.t, info.ideal_v, info.skip);
  }
  return true;
}

bool SpeedDetourSideBackObstacleDecider::UpdateSpeedLimit(TaskInfo& task_info) {
  if (!stop_flag_) return true;
  has_collide_risk_ = true;

  neodrive::global::planning::SpeedLimit internal_speed_limit{};
  internal_speed_limit.set_source_type(SpeedLimitType::LANE_BORROW_AVOID);
  internal_speed_limit.add_upper_bounds(ref_v_);
  internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
  internal_speed_limit.set_acceleration(0.0);
  LOG_INFO(
      "Inroad detour {} limit speed: speed = {:.2f}, acc = {:.2f}, based obs "
      "[{}].",
      SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
      ref_v_, 0.0, final_obs_id_);
  data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
      internal_speed_limit);
  return true;
}

bool SpeedDetourSideBackObstacleDecider::IsDynamicObsNeedIgnord(
    TaskInfo& task_info, const Obstacle* obs, bool& is_near_obs) {
  if (IgnoreObsByLateralDistance(task_info, obs)) {
    return true;
  }
  if (IgnoreObsByBackBoundary(task_info, obs)) {
    return true;
  }
  if (IgnoreObsBySpeed(obs, is_near_obs)) {
    return true;
  }
  return false;
}

bool SpeedDetourSideBackObstacleDecider::BorrowRegionCollideCheck(
    TaskInfo& task_info, const Obstacle* const obs) {
  const auto& pre_traj = obs->prediction_trajectories().front();
  for (std::size_t i = 0; i < pre_traj.trajectory_points().size(); i += 5) {
    const auto& pt = pre_traj.trajectory_points()[i];
    auto obs_polygon = Utility::get_trajectory_point_polygon(
        obs->center(), {pt.x(), pt.y()}, obs->velocity_heading(), pt.theta(),
        obs->polygon());
    double min_l = 1e5, max_l = -1e5;
    for (auto& pt : obs_polygon.points()) {
      SLPoint sl_pt{};
      if (!task_info.reference_line()->GetPointInFrenetFrame(pt, &sl_pt)) {
        continue;
      }
      ReferencePoint ref_pt{};
      task_info.reference_line()->GetNearestRefPoint(sl_pt.s(), &ref_pt);
      auto left_bound = (data_center_->master_info().curr_scenario() ==
                             ScenarioState::DETOUR ||
                         data_center_->master_info().curr_scenario() ==
                             ScenarioState::MOTORWAY_DETOUR)
                            ? ref_pt.left_bound()
                            : ref_pt.left_lane_bound();
      auto right_bound = (data_center_->master_info().curr_scenario() ==
                              ScenarioState::DETOUR ||
                          data_center_->master_info().curr_scenario() ==
                              ScenarioState::MOTORWAY_DETOUR)
                             ? ref_pt.right_bound()
                             : ref_pt.right_lane_bound();
      if (sl_pt.l() < left_bound && sl_pt.l() > -right_bound) {
        return true;
      }
    }
  }
  return false;
}

bool SpeedDetourSideBackObstacleDecider::IgnoreObsByBackBoundary(
    TaskInfo& task_info, const Obstacle* obs) {
  const auto& boundary = obs->PolygonBoundary();
  if (boundary.has_overlap(back_attention_boundary_)) {
    if (boundary.end_l() < back_attention_boundary_.end_l()) {
      LOG_INFO("obs end_l < back_attention_boundary end_l, ignore.");
      return true;
    }
    auto overlap_ratio = (boundary.end_l() - back_attention_boundary_.end_l()) /
                         (boundary.end_l() - boundary.start_l());
    LOG_INFO("BorrowSide::Left overlap ratio {:.3f}", overlap_ratio);
    if (overlap_ratio < 0.8) {
      LOG_INFO("overlap ratio < 0.8, ignore.");
      return true;
    }
  }
  return false;
}

bool SpeedDetourSideBackObstacleDecider::IgnoreObsByLateralDistance(
    TaskInfo& task_info, const Obstacle* obs) {
  auto& boundary = obs->PolygonBoundary();
  double half_width = VehicleParam::Instance()->width() * 0.5;
  ReferencePoint ref_pt;
  task_info.reference_line()->GetNearestRefPoint(task_info.curr_sl().s(),
                                                 &ref_pt);
  auto left_bound =
      (data_center_->master_info().curr_scenario() == ScenarioState::DETOUR ||
       data_center_->master_info().curr_scenario() ==
           ScenarioState::MOTORWAY_DETOUR)
          ? ref_pt.left_bound()
          : ref_pt.left_lane_bound();
  auto right_bound =
      (data_center_->master_info().curr_scenario() == ScenarioState::DETOUR ||
       data_center_->master_info().curr_scenario() ==
           ScenarioState::MOTORWAY_DETOUR)
          ? ref_pt.right_bound()
          : ref_pt.right_lane_bound();
  if (boundary.start_l() > left_bound + half_width ||
      boundary.end_l() < -right_bound - half_width) {
    LOG_INFO("obs [{}] away from ego, ignore.", obs->id());
    return true;
  }
  return false;
}

bool SpeedDetourSideBackObstacleDecider::IgnoreObsBySpeed(const Obstacle* obs,
                                                          bool& is_near_obs) {
  if (init_sl_point_.s() - obs->PolygonBoundary().end_s() > kDistanceBuff) {
    if (obs->speed() < adc_current_v_) {
      LOG_INFO("Ignore far obs [{}] by low speed.", obs->id());
      return true;
    }
  } else {
    if (obs->speed() + kSpeedBuffer < adc_current_v_) {
      LOG_INFO("Ignore near obs [{}] by low speed.", obs->id());
      return true;
    }
    is_near_obs = true;
  }
  return false;
}

std::size_t SpeedDetourSideBackObstacleDecider::PathIndex(TaskInfo& task_info,
                                                          const double s) {
  std::size_t path_collide_index{0};
  double accumulated_s{0.};
  for (std::size_t i = 1; i < task_info.current_frame()
                                  ->outside_planner_data()
                                  .path_data->path()
                                  .path_points()
                                  .size();
       i++) {
    auto curr_point = task_info.current_frame()
                          ->outside_planner_data()
                          .path_data->path()
                          .path_points()[i];
    auto last_point = task_info.current_frame()
                          ->outside_planner_data()
                          .path_data->path()
                          .path_points()[i - 1];
    accumulated_s += std::sqrt(std::pow(curr_point.x() - last_point.x(), 2) +
                               std::pow(curr_point.y() - last_point.y(), 2));
    if (accumulated_s > s) {
      path_collide_index = i;
      break;
    }
  }
  return path_collide_index;
}

double SpeedDetourSideBackObstacleDecider::GetStopSAtFrenetPath(
    TaskInfo& task_info, const double l) {
  const auto& frenet_path_points = task_info.current_frame()
                                       ->outside_planner_data()
                                       .path_data->frenet_path()
                                       .points();
  double target_s = frenet_path_points.back().s();
  for (std::size_t i = 1; i < frenet_path_points.size(); i++) {
    auto curr_sl_point = frenet_path_points[i];
    if (curr_sl_point.l() > l) {
      target_s = frenet_path_points[i - 1].s();
      break;
    }
  }
  return target_s;
}

}  // namespace planning
}  // namespace neodrive