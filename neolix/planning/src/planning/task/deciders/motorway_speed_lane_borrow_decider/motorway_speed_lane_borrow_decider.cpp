#include "motorway_speed_lane_borrow_decider.h"

#include "proto/scenario_manager_msgs.pb.h"
#include "src/planning/common/math/math_utils.h"
#include "src/planning/common/math/polygon2d.h"
#include "src/planning/common/vehicle_param.h"
#include "src/planning/math/frame_conversion/sl_analytic_transformation.h"
#include "src/planning/reference_line/reference_line_util.h"

namespace neodrive {
namespace planning {

MotorwaySpeedLaneBorrowDecider::MotorwaySpeedLaneBorrowDecider() {
  name_ = "MotorwaySpeedLaneBorrowDecider";
}

MotorwaySpeedLaneBorrowDecider::~MotorwaySpeedLaneBorrowDecider() { Reset(); }

void MotorwaySpeedLaneBorrowDecider::SaveTaskResults(TaskInfo& task_info) {
  if (update_limited_speed_) {
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::LANE_BORROW_AVOID);
    internal_speed_limit.add_upper_bounds(limited_speed_);
    internal_speed_limit.set_acceleration(0.0);
    internal_speed_limit.set_constraint_type(SpeedLimitType::HARD);
    LOG_INFO(
        "LANE_BORROW {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        limited_speed_, 0.0);

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);

    last_limited_speed_ = limited_speed_;
  }
}

ErrorCode MotorwaySpeedLaneBorrowDecider::Execute(TaskInfo& task_info) {
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

bool MotorwaySpeedLaneBorrowDecider::Init(TaskInfo& task_info) {
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
  if (!update_limited_speed_) {
    last_limited_speed_ = adc_current_v_;
  }
  update_limited_speed_ = false;
  limited_speed_ = std::numeric_limits<double>::infinity();
  init_sl_point_ = inside_planner_data.init_sl_point;
  adc_front_edge_s_ =
      init_sl_point_.s() + VehicleParam::Instance()->front_edge_to_center();
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;

  borrow_attention_s_vec_.clear();
  borrow_attention_lane_bounds_.clear();
  borrow_attention_bounds_.clear();
  borrow_lane_bound_segments_.clear();
  borrow_regions_.clear();
  walk_s_befor_stop_ = std::numeric_limits<double>::infinity();
  stop_s_in_frenet_ = std::numeric_limits<double>::infinity();
  final_obs_id_ = -1;

  config_ = config::PlanningConfig::Instance()
                ->planning_research_config()
                .lane_borrow_speed_decider_config;
  auto left_lane_bound = task_info.curr_referline_pt().left_lane_bound();
  auto right_lane_bound = task_info.curr_referline_pt().right_lane_bound();
  back_attention_boundary_.set_end_l(
      init_sl_point_.l() +
      std::min(config_.back_attention_ratio * VehicleParam::Instance()->width(),
               left_lane_bound));
  back_attention_boundary_.set_start_l(
      init_sl_point_.l() -
      std::min(config_.back_attention_ratio * VehicleParam::Instance()->width(),
               right_lane_bound));
  back_attention_boundary_.set_start_s(init_sl_point_.s() -
                                       config_.next_back_attention_length);
  back_attention_boundary_.set_end_s(adc_front_edge_s_);
  LOG_INFO("back_attention_boundary: {:.3f}, {:.3f}, {:.3f}, {:.3f}",
           back_attention_boundary_.start_s(), back_attention_boundary_.end_s(),
           back_attention_boundary_.start_l(),
           back_attention_boundary_.end_l());

  auto& borrow_lane_prepare_collision_info =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->motorway_speed_obstacle_context.borrow_lane_prepare_collision_info;
  borrow_lane_prepare_collision_info.clear();

  stop_flag_ = false;

  return true;
}

bool MotorwaySpeedLaneBorrowDecider::Process(TaskInfo& task_info) {
  if (data_center_->master_info().curr_scenario() !=
      ScenarioState::MOTORWAY_DETOUR) {
    LOG_INFO("not in detour scenario, skip.");
    return true;
  }
  if (task_info.current_frame()
          ->planning_data()
          .decision_data()
          .dynamic_obstacle()
          .empty()) {
    LOG_INFO("dynamic_obstacles is empty, skip.");
    return true;
  }
  bool is_prepare =
      (data_center_->master_info().motorway_lane_borrow_context().stage ==
       neodrive::global::planning::MotorwayDetourStageState::PREPARE) ||
      PrepareStatus(task_info);

  if (!is_prepare) {
    LOG_INFO("is not prepare stage, skip.");
    return true;
  }
  /// compute borrow bounds info
  if (!CalcBorrowBoundsInfo(task_info)) {
    LOG_INFO("compute borrow bounds info failed, skip.");
    return true;
  }
  /// check path whether cross lane bound
  if (!PathCrossLaneBound(task_info)) {
    LOG_INFO("path not cross lane bound, skip.");
    return true;
  }
  /// dynamic obstacles attention
  if (!DynamicObsCheck(task_info, task_info.current_frame()
                                      ->planning_data()
                                      .decision_data()
                                      .dynamic_obstacle())) {
    return true;
  }
  /// speed ST decision
  STDecision(task_info);
  /// create virtual obstacle
  CreateVirtualObstacle(task_info);

  return true;
}

bool MotorwaySpeedLaneBorrowDecider::DynamicObsCheck(
    TaskInfo& task_info, const std::vector<Obstacle*>& dynamic_obs_vec) {
  const auto& adc_boxes = task_info.current_frame()
                              ->outside_planner_data()
                              .motorway_speed_obstacle_context.adc_boundaries;
  const auto& path_accumulated_s =
      task_info.current_frame()
          ->outside_planner_data()
          .motorway_speed_obstacle_context.path_accumulated_s;
  auto& collision_info =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->motorway_speed_obstacle_context.borrow_lane_prepare_collision_info;
  collision_info.clear();
  // all dynamic obs will be check
  for (const auto* obs : dynamic_obs_vec) {
    /// Ignore ?
    bool reverse{false};
    double project_vel{0.};
    if (IsDynamicObsNeedIgnord(task_info, obs, reverse, project_vel)) {
      continue;
    }

    // Get collision info from collision check.
    if (GetCollisionInfoFromCollisionCheck(task_info, obs, reverse, project_vel,
                                           collision_info)) {
      continue;
    }

    /// Filter with borrow_regions
    double predict_time = reverse
                              ? config_.dynamic_reverse_obs_prediction_max_time
                              : config_.dynamic_forward_obs_prediction_max_time;
    if (!BorrowRegionCollideCheck(obs, predict_time)) {
      continue;
    }

    /// Compute prepare collision info
    const auto& pre_traj = obs->prediction_trajectories().front();
    std::size_t min_s_index{adc_boxes.size() + 10};
    std::size_t closet_obs_index{pre_traj.trajectory_points().size() + 10};
    double min_relative_time{0.};
    bool collide_with_path = false;
    for (std::size_t i = 0; i < pre_traj.trajectory_points().size(); i += 3) {
      const auto& pt = pre_traj.trajectory_points()[i];
      if (pt.relative_time() > predict_time) break;
      auto obs_polygon = Utility::get_trajectory_point_polygon(
          obs->center(), {pt.x(), pt.y()}, obs->velocity_heading(), pt.theta(),
          obs->polygon());
      for (std::size_t j = 0; j < adc_boxes.size(); j += 5) {
        auto ego_boundary = adc_boxes[j];
        if (obs_polygon.has_overlap(Polygon2d(ego_boundary))) {
          collide_with_path = true;
          if (j < min_s_index) {
            closet_obs_index = i;
            min_s_index = j;
            min_relative_time = pt.relative_time();
          }
        }
        if (collide_with_path && !reverse) {
          break;
        }
      }
      if (collide_with_path) break;
    }

    if (collide_with_path) {
      auto obs_polygon = Utility::get_trajectory_point_polygon(
          obs->center(),
          {pre_traj.trajectory_points()[closet_obs_index].x(),
           pre_traj.trajectory_points()[closet_obs_index].y()},
          obs->velocity_heading(),
          pre_traj.trajectory_points()[closet_obs_index].theta(),
          obs->polygon());
      Boundary obs_collide_boundary{};
      if (!ref_line_util::ComputePolygonBoundary(
              task_info.reference_line(), obs_polygon, &obs_collide_boundary)) {
        LOG_ERROR("compute collide boundary failed, use default boundary.");
        continue;
      }
      std::size_t min_s_index_clamp = std::max(
          static_cast<std::size_t>(0),
          std::min(
              static_cast<std::size_t>(adc_boxes.size() - 1),
              std::min(static_cast<std::size_t>(path_accumulated_s.size() - 1),
                       min_s_index)));
      auto adc_collide_polygon = Polygon2d(adc_boxes[min_s_index_clamp]);
      Boundary adc_collide_boundary{};
      if (!ref_line_util::ComputePolygonBoundary(task_info.reference_line(),
                                                 adc_collide_polygon,
                                                 &adc_collide_boundary)) {
        LOG_ERROR("compute adc_collide_boundary failed, use default boundary.");
        continue;
      }
      collision_info.push_back({.id = obs->id(),
                                .obs_polygon = obs->polygon(),
                                .obs_collide_polygon = obs_polygon,
                                .adc_collide_polygon = adc_collide_polygon,
                                .obs_boundary = obs->PolygonBoundary(),
                                .obs_collide_boundary = obs_collide_boundary,
                                .adc_collide_boundary = adc_collide_boundary,
                                .reverse = reverse,
                                .project_vel = project_vel,
                                .s = path_accumulated_s[min_s_index_clamp],
                                .t = min_relative_time,
                                .skip = false});
    }
  }

  return true;
}

bool MotorwaySpeedLaneBorrowDecider::GetCollisionInfoFromCollisionCheck(
    TaskInfo& task_info, const Obstacle* const obs, bool reverse,
    double project_vel,
    std::vector<LaneBorrowPrepareCollisionInfo>& collision_info) {
  /// Has collision info
  bool has_collision_info{false};
  const auto& multi_cipv_dynamic_obstacles_decision =
      task_info.current_frame()
          ->outside_planner_data()
          .motorway_speed_obstacle_context
          .multi_cipv_dynamic_obstacles_decision;
  const auto& adc_boxes = task_info.current_frame()
                              ->outside_planner_data()
                              .motorway_speed_obstacle_context.adc_boundaries;
  for (const auto& decision : multi_cipv_dynamic_obstacles_decision) {
    if (decision.obstacle.id() == obs->id()) {
      has_collision_info = true;
      if (!decision.lower_points.empty()) {
        auto obs_collide_polygon = reverse ? decision.obstacle_polygons.back()
                                           : decision.obstacle_polygons.front();
        Boundary obs_collide_boundary{};
        if (!ref_line_util::ComputePolygonBoundary(task_info.reference_line(),
                                                   obs_collide_polygon,
                                                   &obs_collide_boundary)) {
          LOG_ERROR(
              "compute obs collide boundary failed, use default boundary.");
        }

        LOG_INFO("obs [{}] lower_adc_first_index {}.", obs->id(),
                 decision.lower_adc_first_index);

        std::size_t lower_adc_first_index_clamp =
            std::max(0, std::min(static_cast<int>(adc_boxes.size() - 1),
                                 decision.lower_adc_first_index));
        auto adc_collide_polygon =
            Polygon2d(adc_boxes[lower_adc_first_index_clamp]);
        Boundary adc_collide_boundary{};
        if (!ref_line_util::ComputePolygonBoundary(task_info.reference_line(),
                                                   adc_collide_polygon,
                                                   &adc_collide_boundary)) {
          LOG_ERROR("compute failed, use default boundary.");
        }
        collision_info.push_back({.id = obs->id(),
                                  .obs_polygon = obs->polygon(),
                                  .obs_collide_polygon = obs_collide_polygon,
                                  .adc_collide_polygon = adc_collide_polygon,
                                  .obs_boundary = obs->PolygonBoundary(),
                                  .obs_collide_boundary = obs_collide_boundary,
                                  .adc_collide_boundary = adc_collide_boundary,
                                  .reverse = reverse,
                                  .project_vel = project_vel,
                                  .s = decision.lower_points.front().first.s(),
                                  .t = decision.lower_points.front().first.t(),
                                  .skip = false});
      }
      break;
    }
  }
  if (has_collision_info) {
    return true;
  }
  return false;
}

bool MotorwaySpeedLaneBorrowDecider::IgnoreObsByObsSpeed(TaskInfo& task_info,
                                                         const Obstacle* obs) {
  auto point = ReferencePoint();
  if (!task_info.reference_line()->GetNearestRefPoint(task_info.curr_sl().s(),
                                                      &point)) {
    LOG_ERROR("GetNearestRefPoint failed.");
    point.set_heading(task_info.curr_referline_pt().heading());
  }
  auto heading_diff_adc_ref = normalize_angle(
      task_info.current_frame()->inside_planner_data().vel_heading -
      point.heading());
  auto heading_diff_obs_ref =
      normalize_angle(obs->velocity_heading() - point.heading());
  double adc_project_ref_v = adc_current_v_ * std::cos(heading_diff_adc_ref);
  double obs_project_ref_v = obs->speed() * std::cos(heading_diff_obs_ref);
  // Filter obs in back by project velocity
  LOG_DEBUG("three speed :{},{},{}", adc_project_ref_v, obs_project_ref_v);
  if (adc_project_ref_v - obs_project_ref_v > 0.3) {
    LOG_INFO("obs is back and vel project less adc_v - 0.3, ignore.");
    return true;
  }
  return false;
};

bool MotorwaySpeedLaneBorrowDecider::IgnoreObsByObsHeading(
    TaskInfo& task_info, const Obstacle* obs) {
  // filter back obs by heading
  // Limit speed by  obs in back by speed heading
  // according distance from obs to ego to limit speed for cuting heading
  const auto& boundary = obs->PolygonBoundary();
  const auto& path_tail_point = task_info.current_frame()
                                    ->outside_planner_data()
                                    .path_data->path()
                                    .path_points()
                                    .back();
  const auto& adc_xy = task_info.adc_point();

  const double samplet = 10;
  const double x_obs_end =
      obs->center().x() +
      std::cos(obs->velocity_heading()) * obs->speed() * samplet;
  const double y_obs_end =
      obs->center().y() +
      std::sin(obs->velocity_heading()) * obs->speed() * samplet;
  const double obs_tan = std::tan(obs->velocity_heading());
  Segment2d line_obs_end(obs->center(), Vec2d(x_obs_end, y_obs_end));
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  auto adc_polygon_origin = VehicleParam::Instance()->get_adc_polygon(
      {inside_data.vel_x, inside_data.vel_y}, inside_data.vel_heading, 0.2, 1,
      0.0);
  double expand_s = 0.0;
  CalPathZeroKappaEndS(task_info, expand_s);
  auto adc_polygon_expand = VehicleParam::Instance()->get_adc_polygon(
      {inside_data.vel_x, inside_data.vel_y}, inside_data.vel_heading, 0.2,
      expand_s, 0.0);
  auto cal_cross_point = [](const double x, const double y, const double k,
                            const double x1, double y1, const double k1,
                            bool& flag) -> std::pair<double, double> {
    if (k == k1) {
      flag = false;
      return std::make_pair(0, 0);
    }
    double x0 = (y1 - y - k1 * x1 + k * x) / (k - k1);
    double y0 = k * (x0 - x) + y;
    return std::make_pair(x0, y0);
  };
  if (adc_polygon_origin.has_overlap(line_obs_end)) {
    // back obs vel heading point at ego
    double dis = task_info.adc_point().distance_to(obs->center());
    double obs_heading = obs->velocity_heading();
    LOG_INFO("obs {} is back and cutting ego, ignore.", obs->id(),
             obs->speed());
    return true;
  }
  double min_x{std::numeric_limits<double>::infinity()},
      max_x{-std::numeric_limits<double>::infinity()};
  for (const auto& point : adc_polygon_origin.points()) {
    bool flag = true;
    auto cross_point = cal_cross_point(
        obs->center().x(), obs->center().y(), obs_tan, point.x(), point.y(),
        obs_tan != 0 ? -1 / obs_tan : std::numeric_limits<double>::infinity(),
        flag);
    if (!flag) {
      continue;
    }  // only in obs pre polygon hasn't overlap, will run this code, so this
       // judge will never success principle.
    min_x = std::min(min_x, cross_point.first);
    max_x = std::max(max_x, cross_point.first);
  }
  if (min_x > max_x) {
    LOG_ERROR("ERROR,can't find map range from obs to ego!");
    return false;
  }
  for (int value = min_x; min_x <= max_x; min_x += 0.3) {
    Polygon2d obs_polygon = Utility::get_trajectory_point_polygon(
        obs->center(),
        {value, value - obs->center().x() * std::tan(obs->velocity_heading()) +
                    obs->center().y()},
        obs->velocity_heading(), obs->velocity_heading(), obs->polygon());
    if (adc_polygon_origin.has_overlap(obs_polygon)) {
      LOG_INFO("obs {} is back and cutting  ego, ignore!", obs->id());
      return true;
    }
  }
  if (adc_polygon_expand.has_overlap(line_obs_end)) {
    limited_speed_ =
        std::fmin(limited_speed_, adc_current_v_);  // forbidden accelerate
    LOG_INFO("obs {} is back and cutting ego, ignore and forbidden accelerate.",
             obs->id(), 2 * 3.6);
    return true;
  }

  // filter obs by decision info, if hasn't overlap with ego path ,filter
  if (path_tail_point.x() == adc_xy.x() && path_tail_point.y() == adc_xy.y()) {
    LOG_INFO("adc path only one point or circle! no ignore.");
    return false;
  }
  return false;
};

bool MotorwaySpeedLaneBorrowDecider::IgnoreObsByLateralDistance(
    TaskInfo& task_info, const Obstacle* obs) {
  auto& boundary = obs->PolygonBoundary();
  if (boundary.end_s() < init_sl_point_.s() +
                             VehicleParam::Instance()->front_edge_to_center() &&
      boundary.end_s() > init_sl_point_.s() -
                             VehicleParam::Instance()->back_edge_to_center()) {
    LOG_INFO("obs {} is back and in detour side, limit speed", obs->id());
    if ((data_center_->master_info()
                 .motorway_lane_borrow_context()
                 .borrow_side == MotorwayLaneBorrowContext::BorrowSide::Left &&
         boundary.start_l() > init_sl_point_.l()) ||
        (data_center_->master_info()
                 .motorway_lane_borrow_context()
                 .borrow_side == MotorwayLaneBorrowContext::BorrowSide::Right &&
         boundary.end_l() < init_sl_point_.l())) {
      double delta_l = std::abs((boundary.start_l() + boundary.end_l()) / 2 -
                                init_sl_point_.l()) -
                       VehicleParam::Instance()->width() / 2 -
                       std::abs(boundary.start_l() - boundary.end_l()) / 2;
      update_limited_speed_ = true;
      limited_speed_ = std::min(limited_speed_, std::pow(delta_l, 3));
      LOG_INFO("Obs {} is in detour side, limit speed {}kph and ignore!.",
               obs->id(), limited_speed_ * 3.6);
      return true;
    }
  }
  return false;
};

bool MotorwaySpeedLaneBorrowDecider::IgnoreObsByBackBoundary(
    TaskInfo& task_info, const Obstacle* obs) {
  const auto& boundary = obs->PolygonBoundary();
  if (boundary.has_overlap(back_attention_boundary_)) {
    if (data_center_->master_info()
            .motorway_lane_borrow_context()
            .borrow_side == MotorwayLaneBorrowContext::BorrowSide::Left) {
      if (boundary.end_l() < back_attention_boundary_.end_l()) {
        LOG_INFO("obs end_l < back_attention_boundary end_l, ignore.");
        return true;
      }
      auto overlap_ratio =
          (boundary.end_l() - back_attention_boundary_.end_l()) /
          (boundary.end_l() - boundary.start_l());
      LOG_INFO("BorrowSide::Left overlap ratio {:.3f}", overlap_ratio);
      if (overlap_ratio < 0.8) {
        LOG_INFO("overlap ratio < 0.8, ignore.");
        return true;
      }
    } else if (data_center_->master_info()
                   .motorway_lane_borrow_context()
                   .borrow_side ==
               MotorwayLaneBorrowContext::BorrowSide::Right) {
      if (boundary.start_l() > back_attention_boundary_.start_l()) {
        LOG_INFO("obs start_l > back_attention_boundary start_l, ignore.");
        return true;
      }
      auto overlap_ratio =
          (boundary.start_l() - back_attention_boundary_.start_l()) /
          (boundary.end_l() - boundary.start_l());
      LOG_INFO("BorrowSide::Right overlap ratio {:.3f}", overlap_ratio);
      if (overlap_ratio > -0.8) {
        LOG_INFO("overlap ratio > -0.8, ignore.");
        return true;
      }
    }
  }
  return false;
};

bool MotorwaySpeedLaneBorrowDecider::IsDynamicObsNeedIgnord(TaskInfo& task_info,
                                                            const Obstacle* obs,
                                                            bool& reverse,
                                                            double& project_v) {
  reverse = false;
  project_v = 0.;
  if (obs->prediction_trajectories().empty() ||
      obs->prediction_trajectories().front().trajectory_points().empty()) {
    LOG_INFO("obs[{}] prediction trajectory is empty, ignore.", obs->id());
    return true;
  }
  auto heading_diff = normalize_angle(
      obs->velocity_heading() -
      task_info.current_frame()->inside_planner_data().vel_heading);
  reverse = (std::abs(heading_diff) > M_PI_2);
  project_v = obs->speed() * std::cos(heading_diff);
  double safe_speed_delta =
      speed_planner_common::CalSafeSpeedDeltaParallelAdc(heading_diff);
  LOG_INFO(
      "obs[{}] is reverse[{}], project_v[{:.3f}], safe_speed_delta[{:.3f}]",
      obs->id(), reverse, project_v, safe_speed_delta);

  // overlap ratio filter
  const auto& boundary = obs->PolygonBoundary();
  // back of adc
  if (boundary.start_s() < init_sl_point_.s()) {
    if (IgnoreObsByObsSpeed(task_info, obs)) return true;
    if (IgnoreObsByBackBoundary(task_info, obs)) return true;
    if (IgnoreObsByObsHeading(task_info, obs)) return true;
  }
  if (IgnoreObsByLateralDistance(task_info, obs)) return true;
  if (reverse) {
    // Filter reversed obs by longitudinal distance
    if (boundary.start_s() - init_sl_point_.s() >
            config_.next_front_attention_length ||
        boundary.end_s() < init_sl_point_.s()) {
      LOG_INFO(
          "obs {} is reverse, over front_attention_length or behind init_s, "
          "ignore.",
          obs->id());
      return true;
    }
  } else {
    // Filter same direction obs by longitudinal distance and relative speed
    bool obs_far_away_adc = (init_sl_point_.s() - boundary.end_s() >
                             config_.next_back_attention_length) ||
                            (boundary.start_s() - init_sl_point_.s() >
                             config_.next_front_attention_length);
    bool obs_ahead_and_faster = (boundary.start_s() > adc_front_edge_s_) &&
                                (project_v - adc_current_v_ > safe_speed_delta);

    bool obs_behind_but_safe = false;
    if (boundary.end_s() < init_sl_point_.s()) {
      double s_dis = init_sl_point_.s() - boundary.end_s();
      double thw{0.0};
      double ttc{0.0};
      if (project_v < adc_current_v_) {
        thw = s_dis / project_v;
        obs_behind_but_safe = thw > config_.thw;
      } else {
        thw = s_dis / project_v;
        ttc = s_dis / (project_v - adc_current_v_);
        obs_behind_but_safe = (thw > config_.thw) && (ttc > config_.ttc);
      }
      LOG_INFO(
          "obs [{}], project_v {:.3f}, s_dis {:.3f}, thw {:.3f}, ttc {:.3f}.",
          obs->id(), project_v, s_dis, thw, ttc);
    }

    if (obs_far_away_adc || obs_ahead_and_faster || obs_behind_but_safe) {
      LOG_INFO(
          "Ignore forward obs [{}]. Because: obs_far_away_adc [{}],"
          "obs_ahead_and_faster [{}],obs_behind_but_safe [{}].",
          obs->id(), obs_far_away_adc, obs_ahead_and_faster,
          obs_behind_but_safe);
      return true;
    }
  }
  return false;
}

bool MotorwaySpeedLaneBorrowDecider::CalcBorrowBoundsInfo(TaskInfo& task_info) {
  const auto& reference_line = task_info.reference_line();
  std::vector<Vec2d> ref_points{};
  std::vector<double> ref_headings{};
  for (std::size_t i = 0; i < reference_line->ref_points().size(); i += 10) {
    const auto& point = reference_line->ref_points()[i];
    if (point.s() < init_sl_point_.s() - config_.next_back_attention_length)
      continue;
    if (point.s() > init_sl_point_.s() + FLAGS_planning_lane_borrow_preview_s)
      continue;
    borrow_attention_s_vec_.emplace_back(point.s());
    ref_points.emplace_back(Vec2d(point.x(), point.y()));
    ref_headings.emplace_back(point.heading());
    if (data_center_->master_info()
            .motorway_lane_borrow_context()
            .borrow_side == MotorwayLaneBorrowContext::BorrowSide::Left) {
      borrow_attention_lane_bounds_.emplace_back(point.left_lane_bound());
      borrow_attention_bounds_.emplace_back(point.left_bound());
    } else if (data_center_->master_info()
                   .motorway_lane_borrow_context()
                   .borrow_side ==
               MotorwayLaneBorrowContext::BorrowSide::Right) {
      borrow_attention_lane_bounds_.emplace_back(-point.right_lane_bound());
      borrow_attention_bounds_.emplace_back(-point.right_bound());
    }
  }
  if (borrow_attention_bounds_.size() < 2) {
    LOG_ERROR("borrow bounds info size < 2.");
    return false;
  }

  // segments and polygons
  std::vector<Vec2d> lane_bound_xy_points{};
  std::vector<Vec2d> bound_xy_points{};
  for (std::size_t i = 0; i < borrow_attention_s_vec_.size(); ++i) {
    lane_bound_xy_points.emplace_back(
        SLAnalyticTransformation::calculate_xypoint(
            ref_headings[i], ref_points[i], borrow_attention_lane_bounds_[i]));
    bound_xy_points.emplace_back(SLAnalyticTransformation::calculate_xypoint(
        ref_headings[i], ref_points[i], borrow_attention_bounds_[i]));
  }
  for (std::size_t i = 1; i < lane_bound_xy_points.size(); ++i) {
    if (borrow_attention_s_vec_[i - 1] >= init_sl_point_.s()) {
      borrow_lane_bound_segments_.emplace_back(
          Segment2d(lane_bound_xy_points[i - 1], lane_bound_xy_points[i]));
    }
    std::vector<Vec2d> cornors = {lane_bound_xy_points[i - 1],
                                  lane_bound_xy_points[i], bound_xy_points[i],
                                  bound_xy_points[i - 1]};
    borrow_regions_.emplace_back(Polygon2d(cornors));
  }
  LOG_INFO("segments size: {}, polygon size: {}",
           borrow_lane_bound_segments_.size(), borrow_regions_.size());

  return true;
}

bool MotorwaySpeedLaneBorrowDecider::PathCrossLaneBound(TaskInfo& task_info) {
  bool flag_path_cross_lane_bound{false};
  for (std::size_t i = 0; i < task_info.current_frame()
                                  ->outside_planner_data()
                                  .path_data->frenet_path()
                                  .num_of_points();
       i += 5) {
    const auto& veh_pt = task_info.current_frame()
                             ->outside_planner_data()
                             .path_data->path()
                             .path_points()[i];
    Box2d veh_box = VehicleParam::Instance()->get_adc_bounding_box(
        {veh_pt.x(), veh_pt.y()}, veh_pt.theta(), 0.2, 0.2, 0.2);
    for (const auto& segment :
         borrow_lane_bound_segments_) {  // first position to cross
      if (veh_box.has_overlap(segment)) {
        flag_path_cross_lane_bound = true;
        path_cross_lane_point_.set_x(veh_pt.x());
        path_cross_lane_point_.set_y(veh_pt.y());
        SLPoint lane_segment_end{}, adc_front_pt{};
        task_info.reference_line()->GetPointInFrenetFrame(segment.end(),
                                                          &lane_segment_end);
        std::vector<Vec2d> corners;
        veh_box.get_all_corners(&corners);
        if (data_center_->master_info().lane_borrow_context().borrow_side ==
            LaneBorrowContext::BorrowSide::Left) {
          task_info.reference_line()->GetPointInFrenetFrame(corners[1],
                                                            &adc_front_pt);
        } else if (data_center_->master_info()
                       .lane_borrow_context()
                       .borrow_side == LaneBorrowContext::BorrowSide::Right) {
          task_info.reference_line()->GetPointInFrenetFrame(corners[0],
                                                            &adc_front_pt);
        }
        path_cross_lane_s_ = std::min(lane_segment_end.s(), adc_front_pt.s());
        break;
      }
    }
    if (flag_path_cross_lane_bound) break;
  }

  return flag_path_cross_lane_bound;
}

bool MotorwaySpeedLaneBorrowDecider::BorrowRegionCollideCheck(
    const Obstacle* const obs, const double predict_time) {
  const auto& pre_traj = obs->prediction_trajectories().front();
  for (std::size_t i = 0; i < pre_traj.trajectory_points().size(); i += 5) {
    const auto& pt = pre_traj.trajectory_points()[i];
    if (pt.relative_time() > predict_time) break;
    // TODO:(zhp) must be check this coordinate for pnc and other moudel
    auto obs_polygon = Utility::get_trajectory_point_polygon(
        obs->center(), {pt.x(), pt.y()}, obs->velocity_heading(), pt.theta(),
        obs->polygon());
    for (const auto& region : borrow_regions_) {
      if (region.has_overlap(obs_polygon)) {
        return true;
      }
    }
  }

  return false;
}

bool MotorwaySpeedLaneBorrowDecider::PrepareStatus(TaskInfo& task_info) {
  std::vector<Segment2d> path_segments{};
  double segment_dis_sum{0.};
  for (std::size_t i = 5; i < task_info.current_frame()
                                  ->outside_planner_data()
                                  .path_data->path()
                                  .path_points()
                                  .size();
       i += 5) {
    if (segment_dis_sum > 20.0) break;
    auto curr_pt = task_info.current_frame()
                       ->outside_planner_data()
                       .path_data->path()
                       .path_points()[i - 5];
    auto forward_pt = task_info.current_frame()
                          ->outside_planner_data()
                          .path_data->path()
                          .path_points()[i];
    path_segments.emplace_back(
        Segment2d(curr_pt.coordinate(), forward_pt.coordinate()));
    segment_dis_sum += path_segments.back().length();
  }

  double width_threshold = VehicleParam::Instance()->width() * 0.5;
  bool adc_on_refer_lane =
      ((task_info.curr_sl().l() >
        task_info.curr_referline_pt().left_lane_bound() - width_threshold) ||
       (task_info.curr_sl().l() <
        -task_info.curr_referline_pt().right_lane_bound() + width_threshold))
          ? false
          : true;

  bool cross_flag{false};
  for (const auto& path : path_segments) {
    for (const auto& lane : borrow_lane_bound_segments_) {
      if (path.has_intersect(lane)) {
        cross_flag = true;
        break;
      }
    }
  }

  double path_min_l{1000.}, path_max_l{-1000.};
  for (const auto& sl : task_info.current_frame()
                            ->outside_planner_data()
                            .path_data->frenet_path()
                            .points()) {
    if (sl.s() - task_info.current_frame()
                     ->outside_planner_data()
                     .path_data->frenet_path()
                     .points()
                     .front()
                     .s() >
        2 * VehicleParam::Instance()->front_edge_to_center()) {
      break;
    }
    if (sl.l() < path_min_l) path_min_l = sl.l();
    if (sl.l() > path_max_l) path_max_l = sl.l();
  }
  bool path_direction_flag{false};
  if (data_center_->master_info().motorway_lane_borrow_context().borrow_side ==
      MotorwayLaneBorrowContext::BorrowSide::Left) {
    path_direction_flag =
        (path_max_l >
         task_info.curr_sl().l() - 0.1 * VehicleParam::Instance()->width());
  } else if (data_center_->master_info()
                 .motorway_lane_borrow_context()
                 .borrow_side == MotorwayLaneBorrowContext::BorrowSide::Right) {
    path_direction_flag =
        (path_min_l <
         task_info.curr_sl().l() + 0.1 * VehicleParam::Instance()->width());
  }

  LOG_INFO("adc_on_reference_line[{}], cross_flag[{}], path_direction_flag[{}]",
           adc_on_refer_lane, cross_flag, path_direction_flag);

  return adc_on_refer_lane && cross_flag && path_direction_flag;
}

bool MotorwaySpeedLaneBorrowDecider::STDecision(TaskInfo& task_info) {
  auto& collision_info =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->motorway_speed_obstacle_context.borrow_lane_prepare_collision_info;

  if (collision_info.empty()) {
    LOG_INFO("borrow_lane_prepare_collision_info is empty, skip.");
    return true;
  }

  /// Analysis ST points info
  const auto& adc_boxes = task_info.current_frame()
                              ->outside_planner_data()
                              .motorway_speed_obstacle_context.adc_boundaries;
  const auto& adc_boundary =
      task_info.current_frame()
          ->outside_planner_data()
          .motorway_speed_obstacle_context.adc_sl_boundaries.front();
  const auto& borrow_side =
      data_center_->master_info().motorway_lane_borrow_context().borrow_side;

  for (auto& info : collision_info) {
    // longitudinal
    double adc_will_run_s = info.t * adc_current_v_;
    bool longi_skip = (adc_will_run_s > info.s + info.obs_boundary.end_s() -
                                            info.obs_boundary.start_s()) ||
                      (adc_will_run_s < info.s - config_.longi_safe_dis);
    LOG_INFO(
        "obs [{}], boundary length {:.3f}, adc_will_run_s {:.3f}, info.s "
        "{:.3f}, longi_skip [{}].",
        info.id, info.obs_boundary.end_s() - info.obs_boundary.start_s(),
        adc_will_run_s, info.s, longi_skip);

    // lateral
    bool lateral_skip{false};
    std::size_t path_collide_idx = std::max(
        static_cast<std::size_t>(0),
        std::min(static_cast<std::size_t>(PathIndex(task_info, adc_will_run_s)),
                 adc_boxes.size() - 1));
    Boundary path_collide_boundary{};
    if (!ref_line_util::ComputePolygonBoundary(
            task_info.reference_line(), Polygon2d(adc_boxes[path_collide_idx]),
            &path_collide_boundary)) {
      LOG_ERROR("compute collide boundary failed, use default boundary.");
      continue;
    }
    double first_lateral_error{0.}, second_lateral_error{0.};
    if (borrow_side == MotorwayLaneBorrowContext::BorrowSide::Left) {
      first_lateral_error =
          std::abs(adc_boundary.end_l() - info.obs_boundary.start_l());
      second_lateral_error =
          std::abs(path_collide_boundary.end_l() - info.obs_boundary.start_l());
      if ((first_lateral_error > config_.lat_safe_dis) &&
          (second_lateral_error > config_.lat_safe_dis)) {
        lateral_skip = true;
        LOG_INFO(
            "Left Borrow: ego will move to left and away from obstacle, "
            "lateral skip.");
      }
    } else if (borrow_side == MotorwayLaneBorrowContext::BorrowSide::Right) {
      first_lateral_error =
          std::abs(adc_boundary.start_l() - info.obs_boundary.end_l());
      second_lateral_error =
          std::abs(path_collide_boundary.start_l() - info.obs_boundary.end_l());
      if ((first_lateral_error > config_.lat_safe_dis) &&
          (second_lateral_error > config_.lat_safe_dis)) {
        lateral_skip = true;
        LOG_INFO(
            "Right Borrow: ego will move to right and away from obstacle, "
            "lateral skip.");
      }
    }
    LOG_INFO(
        "first_lateral_error, second_lateral_error, lateral_skip: {:.3f}, "
        "{:.3f}, [{}]",
        first_lateral_error, second_lateral_error, lateral_skip);

    // combine
    info.skip = longi_skip && lateral_skip;
  }

  LOG_INFO("lane_borrow_prepare_collision_info size: {}",
           collision_info.size());
  for (const auto& info : collision_info) {
    if (info.skip || (config_.ignore_reversed_obs && info.reverse)) {
      continue;
    }
    stop_flag_ = true;
    walk_s_befor_stop_ = std::min(walk_s_befor_stop_, info.s);
    if (stop_s_in_frenet_ > info.adc_collide_boundary.end_s()) {
      final_obs_id_ = info.id;
    }
    stop_s_in_frenet_ =
        std::min(stop_s_in_frenet_, info.adc_collide_boundary.end_s());
    LOG_INFO(
        "id[{}], reverse[{}], project_vel {:.3f}, path s {:.3f}, t {:.3f}, "
        "stop_s_in_frenet_ {:.3f}, skip[{}]",
        info.id, info.reverse, info.project_vel, info.s, info.t,
        stop_s_in_frenet_, info.skip);
  }

  return true;
}

std::size_t MotorwaySpeedLaneBorrowDecider::PathIndex(TaskInfo& task_info,
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

bool MotorwaySpeedLaneBorrowDecider::CreateVirtualObstacle(
    TaskInfo& task_info) {
  if (!stop_flag_) return true;

  const double init_s =
      task_info.current_frame()->inside_planner_data().init_sl_point.s();
  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data->path();
  stop_s_in_frenet_ = path_cross_lane_s_ > stop_s_in_frenet_
                          ? path_cross_lane_s_
                          : (stop_s_in_frenet_ - config_.longi_safe_dis < 5.0
                                 ? 5.0
                                 : stop_s_in_frenet_ - config_.longi_safe_dis);
  auto ret = task_info.current_frame()
                 ->mutable_planning_data()
                 ->mutable_decision_data()
                 ->create_virtual_obstacle(path, init_s, stop_s_in_frenet_,
                                           VirtualObstacle::LANE_BORROW);
  if (ret != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to create lane borrow virtual obstacle.");
    return true;
  }
  LOG_INFO(
      "Base obs [{}] created virtual obstacle at path s {:.3f}, for "
      "lane "
      "borrow.",
      final_obs_id_, stop_s_in_frenet_);

  return true;
}

bool MotorwaySpeedLaneBorrowDecider::CalPathZeroKappaEndS(TaskInfo& task_info,
                                                          double& expand_s) {
  const auto& path = task_info.current_frame()
                         ->outside_planner_data()
                         .path_data->path()
                         .path_points();
  // choose kappa from -0.01 - 0.01
  for (auto& pt : path) {
    if (pt.kappa() < 0.01 && pt.kappa() > -0.01) {
      expand_s = pt.s();
    } else {
      break;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive
