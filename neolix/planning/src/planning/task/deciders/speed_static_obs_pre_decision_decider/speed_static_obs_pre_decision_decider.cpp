#include "speed_static_obs_pre_decision_decider.h"

#include "src/planning/common/data_center/data_center.h"
#include "src/planning/math/common/occupy_map.h"
#include "src/planning/reference_line/reference_line_util.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {

SpeedStaticObsPreDecisionDecider::SpeedStaticObsPreDecisionDecider() {
  name_ = "SpeedStaticObsPreDecisionDecider";
}

SpeedStaticObsPreDecisionDecider::~SpeedStaticObsPreDecisionDecider() {
  Reset();
}

ErrorCode SpeedStaticObsPreDecisionDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  if (frame->outside_planner_data().path_data == nullptr) {
    LOG_ERROR("path_data == nullptr.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!Process(task_info)) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

bool SpeedStaticObsPreDecisionDecider::Process(TaskInfo& task_info) {
  // 1. static obs pre decision
  if (StaticObsPreDecision(
          task_info, task_info.reference_line(),
          task_info.current_frame()->inside_planner_data(),
          task_info.current_frame()
              ->planning_data()
              .decision_data()
              .static_obstacle(),
          task_info.current_frame()->mutable_outside_planner_data()) !=
      ErrorCode::PLANNING_OK) {
    LOG_ERROR("StaticObsPreDecision failed.");
    return false;
  }
  // if (LowSpeedDynamicObsPreDecision(
  //         task_info.current_frame()->inside_planner_data(),
  //         task_info.current_frame()
  //             ->planning_data()
  //             .decision_data()
  //             .dynamic_obstacle(),
  //         task_info.current_frame()->mutable_outside_planner_data()) !=
  //     ErrorCode::PLANNING_OK) {
  //   LOG_ERROR("LowSpeedDynamicObsPreDecision failed.");
  //   return false;
  // }
  StaticContextInfo(task_info.current_frame()->mutable_outside_planner_data());

  return true;
}

ErrorCode SpeedStaticObsPreDecisionDecider::StaticObsPreDecision(
    TaskInfo& task_info, const ReferenceLinePtr& reference_line,
    const InsidePlannerData& inside_data,
    const std::vector<Obstacle*>& static_obs_vec,
    OutsidePlannerData* const outside_data) const {
  LOG_INFO("___static_obstacle_pre_decision infos___:");
  const auto& plan_research_config =
      config::PlanningConfig::Instance()->planning_research_config();
  const auto& plan_config = config::PlanningConfig::Instance()->plan_config();
  const auto& adc_sl_point = inside_data.init_sl_point;
  const auto& right_freespace_bound = data_center_->frenet_lower_bound();
  const auto& left_freespace_bound = data_center_->frenet_upper_bound();
  bool freespace_filter_flag =
      !(right_freespace_bound.empty() && left_freespace_bound.empty());
  double min_filter_s{adc_sl_point.s() + 1000.0},
      max_filter_s{adc_sl_point.s() - 1000.0};
  for (const auto& [s, l] : right_freespace_bound) {
    min_filter_s = std::min(min_filter_s, s);
    max_filter_s = std::max(max_filter_s, s);
  }
  for (const auto& [s, l] : left_freespace_bound) {
    min_filter_s = std::min(min_filter_s, s);
    max_filter_s = std::max(max_filter_s, s);
  }

  // get path max boundary
  double path_left_road_bound{-100.0};
  double path_right_road_bound{100.0};
  const auto& sl_path_points = outside_data->path_data->frenet_path().points();
  const auto& commonConfig = neodrive::common::config::CommonConfig::Instance();

  for (std::size_t index = 0; index < sl_path_points.size(); ++index) {
    path_left_road_bound = std::fmax(
        path_left_road_bound,
        sl_path_points[index].l() +
            plan_research_config.speed_static_obs_pre_decision_decider_config
                    .static_obs_width_handle_ratio *
                commonConfig->ego_car_config().width);
    path_right_road_bound = std::fmin(
        path_right_road_bound,
        sl_path_points[index].l() -
            plan_research_config.speed_static_obs_pre_decision_decider_config
                    .static_obs_width_handle_ratio *
                commonConfig->ego_car_config().width);
  }

  LOG_INFO("path left bound: {}, path right bound: {}", path_left_road_bound,
           path_right_road_bound);
  for (std::size_t i = 0; i < static_obs_vec.size(); ++i) {
    if (static_obs_vec[i] == nullptr) {
      continue;
    }
    if (static_obs_vec[i]->is_virtual()) {
      continue;
    }
    LOG_DEBUG("id: [{}]", static_obs_vec[i]->id());
    bool need_ignore = false;
    if (!inside_data.change_lane_task_mode) {
      double left_road_bound = path_left_road_bound;
      double right_road_bound = path_right_road_bound;
      GetRoadBound(reference_line, inside_data,
                   static_obs_vec[i]->PolygonBoundary(), &left_road_bound,
                   &right_road_bound);
      if (left_road_bound <= right_road_bound) {
        LOG_ERROR("left bound <= right bound, ERROR");
        return ErrorCode::PLANNING_ERROR_FAILED;
      }
      IsStaticObsNeedIgnore(adc_sl_point, static_obs_vec[i]->PolygonBoundary(),
                            left_road_bound, right_road_bound, true,
                            need_ignore);
    }
    if (need_ignore) {
      LOG_INFO("need_ignore id: [{}]", static_obs_vec[i]->id());
      continue;
    }

    LOG_INFO("start static obstacle collision check:");
    ErrorCode ret = CollisionCheckObstacleWithoutTrajectory(
        task_info, inside_data, *static_obs_vec[i], outside_data,
        &(outside_data->speed_obstacle_context.static_obstacles_decision));
    if (ret == ErrorCode::PLANNING_ERROR_FAILED) {
      LOG_ERROR("obs [{}] collision check failed.", static_obs_vec[i]->id());
      return ret;
    }
    if (inside_data.curr_scenario_state != ScenarioState::BACK_OUT &&
        plan_config.speed_plan.check_collision_with_prediction_trajectory) {
      LOG_INFO("start pre static obstacle collision check:");
      ret = CollisionPreCheckObstacleWithoutTrajectory(
          inside_data, *static_obs_vec[i], outside_data,
          &(outside_data->speed_obstacle_context
                .static_pre_obstacles_decision));
      if (ret == ErrorCode::PLANNING_ERROR_FAILED) {
        LOG_ERROR("obs [{}] collision check failed.", static_obs_vec[i]->id());
      }
    }
  }
  return ErrorCode::PLANNING_OK;
}

ErrorCode SpeedStaticObsPreDecisionDecider::VirtualObsPreDecision(
    TaskInfo& task_info, const InsidePlannerData& inside_data,
    const std::vector<Obstacle*>& virtual_obs_vec,
    OutsidePlannerData* const outside_data) const {
  LOG_INFO("___virtual_obstacle_pre_decision infos___:");
  for (std::size_t i = 0; i < virtual_obs_vec.size(); ++i) {
    if (virtual_obs_vec[i] == nullptr) {
      continue;
    }
    if (!virtual_obs_vec[i]->is_virtual()) {
      continue;
    }
    LOG_INFO("id : [{}]", virtual_obs_vec[i]->id());
    LOG_INFO("start virtual obstacle collision check:");
    ErrorCode ret = CollisionCheckObstacleWithoutTrajectory(
        task_info, inside_data, *virtual_obs_vec[i], outside_data,
        &(outside_data->speed_obstacle_context.virtual_obstacle_decision));
    if (ret == ErrorCode::PLANNING_ERROR_FAILED) {
      LOG_ERROR("collision check failed.");
      return ret;
    }
  }

  return ErrorCode::PLANNING_OK;
}

// Write collision result of low speed dynamic obs into
// static_obstacles_decision. (So low speed dynamic obs has two collision check
// result in st graph.)
ErrorCode SpeedStaticObsPreDecisionDecider::LowSpeedDynamicObsPreDecision(
    TaskInfo& task_info, const InsidePlannerData& inside_data,
    const std::vector<Obstacle*>& dynamic_obs_vec,
    OutsidePlannerData* const outside_data) const {
  if (outside_data == nullptr) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  LOG_INFO("___low_speed_dynamic_obstacle_pre_decision infos___:");
  for (std::size_t i = 0; i < dynamic_obs_vec.size(); ++i) {
    if (dynamic_obs_vec[i] == nullptr) {
      continue;
    }
    if (dynamic_obs_vec[i]->speed() > 1.0) {
      continue;
    }
    bool ignore{false};
    if (!inside_data.change_lane_task_mode) {
      IsDynamicObsNeedIgnore(inside_data, *dynamic_obs_vec[i], ignore);
    }
    if (ignore) {
      continue;
    }
    LOG_INFO("id: {}, start low speed dynamic obstacle collision check",
             dynamic_obs_vec[i]->id());
    ErrorCode ret = CollisionCheckObstacleWithoutTrajectory(
        task_info, inside_data, *dynamic_obs_vec[i], outside_data,
        &(outside_data->speed_obstacle_context.static_obstacles_decision));
    if (ret == ErrorCode::PLANNING_ERROR_FAILED) {
      LOG_ERROR("collision check failed.");
      return ret;
    }
  }

  return ErrorCode::PLANNING_OK;
}

bool SpeedStaticObsPreDecisionDecider::GetRoadBound(
    const ReferenceLinePtr& reference_line,
    const InsidePlannerData& inside_data, const Boundary& boundary,
    double* left_bound, double* right_bound) const {
  if (left_bound == nullptr || right_bound == nullptr) return false;
  const auto& ref_points = reference_line->ref_points();
  auto comp = [](const auto& ref_point, double s) { return ref_point.s() < s; };
  auto start_it = std::lower_bound(ref_points.begin(), ref_points.end(),
                                   boundary.start_s(), comp);
  auto end_it = std::lower_bound(ref_points.begin(), ref_points.end(),
                                 boundary.end_s(), comp);
  if (end_it != ref_points.end()) {
    ++end_it;
  }
  for (auto it = start_it; it != end_it; ++it) {
    *left_bound = std::fmax(*left_bound, it->left_road_bound());
    *right_bound = std::fmin(*right_bound, -it->right_road_bound());
  }
  LOG_DEBUG("final left bound: {}, final right bound: {}", *left_bound,
            *right_bound);
  return true;
}

/**
 * 1.three types collision check:
 * (1) static obs -> static_obstacles_decision.
 * (2) virtual obs -> virtual_obstacles_decision.
 * (3) low speed dynamic obs(1.0) -> static_obstacles_decision.(Deprecated)
 * 2.obstacles_decision covers all time domain of st graph.
 */
ErrorCode
SpeedStaticObsPreDecisionDecider::CollisionCheckObstacleWithoutTrajectory(
    TaskInfo& task_info, const InsidePlannerData& inside_data,
    const Obstacle& obstacle, OutsidePlannerData* const outside_data,
    std::vector<SpeedObstacleDecision>* obstacles_decision) const {
  if (outside_data == nullptr || obstacles_decision == nullptr) {
    LOG_ERROR("Input is nullptr.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (obstacle.length() < 1e-4 || obstacle.width() < 1e-4) {
    LOG_INFO("Obstacle [{}] length({:.4f}) < 1e-4 || width({:.4f}) < 1e-4",
             obstacle.id(), obstacle.length(), obstacle.width());
    return ErrorCode::PLANNING_OK;
  }
  const auto& adc_bounding_boxes =
      outside_data->speed_obstacle_context.adc_boundaries;
  const auto& path_points = outside_data->path_data->path().path_points();
  if (adc_bounding_boxes.size() < 3) {
    LOG_ERROR("adc bounding boxes size < 3.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (path_points.size() != adc_bounding_boxes.size()) {
    LOG_ERROR("path_points size != adc_bounding_boxes size.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  double collide_s = path_points.back().s() + 100.0;
  for (std::size_t index = 0; index < adc_bounding_boxes.size(); index += 2) {
    if (obstacle.polygon().has_overlap(Polygon2d(adc_bounding_boxes[index]))) {
      collide_s = path_points[index].s();
      if (index > 0) {
        if (obstacle.polygon().has_overlap(
                Polygon2d(adc_bounding_boxes[index - 1]))) {
          collide_s = path_points[index - 1].s();
        }
      }
      break;
    }
  }
  LOG_INFO("collid_s, path_points.back.s: {:.3f}, {:.3f}", collide_s,
           path_points.back().s());
  if (collide_s <= path_points.back().s()) {
    LOG_INFO("virtual, is_lane_borrow, is_prepare_borrow: {}, {}, {}",
             obstacle.is_virtual(), inside_data.is_lane_borrowing,
             inside_data.is_prepare_borrowing);
    LOG_INFO("before collid_s: {:.3f}", collide_s);
    if (!obstacle.is_virtual() && inside_data.is_prepare_borrowing &&
        !inside_data.is_lane_borrowing) {
      collide_s -= 4.0;
    }
    collide_s = std::fmax(collide_s, 0.0);
    LOG_INFO("after collid_s: {:.3f}", collide_s);

    SpeedObstacleDecision decision;
    std::vector<std::pair<STPoint, double>> lower_points{};
    std::vector<std::pair<STPoint, double>> upper_points{};
    lower_points.emplace_back(STPoint(collide_s, 0.0), 0.0);
    lower_points.emplace_back(STPoint(collide_s, 100.0), 0.0);
    upper_points.emplace_back(STPoint(path_points.back().s(), 0.0), 0.0);
    upper_points.emplace_back(STPoint(path_points.back().s(), 100.0), 0.0);
    decision.lower_points = lower_points;
    decision.upper_points = upper_points;
    decision.obstacle = obstacle;
    decision.collide = true;
    decision.reverse = false;
    obstacles_decision->emplace_back(decision);
  }
  return ErrorCode::PLANNING_OK;
}

ErrorCode
SpeedStaticObsPreDecisionDecider::CollisionPreCheckObstacleWithoutTrajectory(
    const InsidePlannerData& inside_data, const Obstacle& obstacle,
    OutsidePlannerData* const outside_data,
    std::vector<SpeedObstacleDecision>* obstacles_decision) const {
  if (outside_data == nullptr || obstacles_decision == nullptr) {
    LOG_ERROR("Input is nullptr.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (obstacle.length() < 1e-4 || obstacle.width() < 1e-4) {
    LOG_INFO("Obstacle [{}] length({:.4f}) < 1e-4 || width({:.4f}) < 1e-4",
             obstacle.id(), obstacle.length(), obstacle.width());
    return ErrorCode::PLANNING_OK;
  }
  const auto& adc_bounding_boxes =
      outside_data->speed_obstacle_context.adc_pre_boundaries;
  const auto& path_points =
      outside_data->speed_obstacle_context.pre_path_points;
  const double max_s = outside_data->path_data->path().path_points().back().s();
  if (path_points.size() != adc_bounding_boxes.size()) {
    LOG_DEBUG("path_points size != adc_bounding_boxes size.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  double collide_s = path_points.back().s() + 100.0;
  for (std::size_t index = 0; index < adc_bounding_boxes.size(); index += 2) {
    if (obstacle.polygon().has_overlap(Polygon2d(adc_bounding_boxes[index]))) {
      LOG_INFO("collide in index {}", index);
      collide_s = path_points[index].s();
      if (index > 0) {
        if (obstacle.polygon().has_overlap(
                Polygon2d(adc_bounding_boxes[index - 1]))) {
          LOG_INFO("collide in index {}", index - 1);
          collide_s = path_points[index].s();
        }
      }
      break;
    }
  }

  LOG_INFO("collid_s, path_points.back.s: {:.3f}, {:.3f}", collide_s,
           path_points.back().s());
  if (collide_s <= path_points.back().s()) {
    SpeedObstacleDecision decision;
    std::vector<std::pair<STPoint, double>> lower_points{};
    std::vector<std::pair<STPoint, double>> upper_points{};
    lower_points.emplace_back(STPoint(collide_s, 0.0), 0.0);
    lower_points.emplace_back(STPoint(collide_s, 100.0), 0.0);
    upper_points.emplace_back(STPoint(max_s, 0.0), 0.0);
    upper_points.emplace_back(STPoint(max_s, 100.0), 0.0);
    decision.lower_points = lower_points;
    decision.upper_points = upper_points;
    decision.obstacle = obstacle;
    decision.collide = true;
    decision.reverse = false;
    obstacles_decision->emplace_back(decision);
  }
  return ErrorCode::PLANNING_OK;
}

bool SpeedStaticObsPreDecisionDecider::IsStaticObsNeedIgnore(
    const SLPoint& vel_sl, const Boundary& obs_boundary,
    const double& left_road_bound, const double& right_road_bound,
    const bool is_forward, bool& is_ignore) const {
  is_ignore = false;
  if (obs_boundary.start_l() > left_road_bound ||
      obs_boundary.end_l() < right_road_bound) {
    is_ignore = true;
    LOG_DEBUG("obs_boundary outside road, ignore.");
    return true;
  }
  double valid_s = vel_sl.s();
  if (is_forward) {
    valid_s -= VehicleParam::Instance()->back_edge_to_center();
  } else {
    valid_s -= VehicleParam::Instance()->front_edge_to_center();
  }
  if (obs_boundary.end_s() < valid_s) {
    LOG_DEBUG("obs_boundary behind ego, ignore.");
    is_ignore = true;
  }
  return true;
}

bool SpeedStaticObsPreDecisionDecider::FreespaceFilter(
    const std::vector<Boundary>& adc_sl_boundaries, const double max_filter_s,
    const double min_filter_s, const SLPoint& vel_sl,
    const Boundary& obs_boundary, const bool is_forward, bool& is_valid) const {
  is_valid = false;
  if (adc_sl_boundaries.empty()) {
    return true;
  }

  auto bs_idx = [&adc_sl_boundaries](auto s) -> size_t {
    if (s < adc_sl_boundaries.front().start_s()) return 0;
    std::size_t m = adc_sl_boundaries.size();
    if (s > adc_sl_boundaries.back().end_s()) return m - 1;

    std::size_t l = 0, r = m - 1;
    while (l < r) {
      int mid = l + (r - l) / 2;
      if (adc_sl_boundaries[mid].start_s() < s) {
        l = mid + 1;
      } else {
        r = mid;
      }
    }
    return l;
  };
  auto dis_value = [](auto vel) -> double {
    if (vel < 1.0) {
      return 0.05;
    } else if (vel < 2.0) {
      return 0.10;
    } else if (vel < 3.5) {
      return 0.15;
    } else if (vel < 5.5) {
      return 0.20;
    } else {
      return 0.25;
    }
    return 0.25;
  };

  const double adc_velocity =
      DataCenter::Instance()->vehicle_state_proxy().LinearVelocity();
  const double dis_th = dis_value(adc_velocity);
  std::size_t adc_start_index = bs_idx(obs_boundary.start_s());
  std::size_t adc_end_index = bs_idx(obs_boundary.end_s());
  for (std::size_t index = adc_start_index; index < adc_end_index; ++index) {
    if (adc_sl_boundaries[index].distance_to(obs_boundary) < dis_th) {
      return true;
    }
  }

  is_valid = true;
  LOG_INFO("obs_boundary ignore by freespace.");

  return true;
}

bool SpeedStaticObsPreDecisionDecider::IsDynamicObsNeedIgnore(
    const InsidePlannerData& inside_data, const Obstacle& obstacle,
    bool& is_valid) const {
  is_valid = false;
  SLPoint adc_sl_point = inside_data.init_sl_point;

  double adc_s_on_reference_line = adc_sl_point.s();
  if (inside_data.is_reverse_driving) {
    adc_s_on_reference_line -= VehicleParam::Instance()->front_edge_to_center();
  } else {
    adc_s_on_reference_line -= VehicleParam::Instance()->back_edge_to_center();
  }
  double front_buffer = std::max(inside_data.vel_v * 8.0, 15.0);
  double end_s = obstacle.PolygonBoundary().end_s();
  double start_s = obstacle.PolygonBoundary().start_s();
  double heading_diff =
      normalize_angle(obstacle.velocity_heading() - inside_data.vel_heading);
  double project_vel = obstacle.speed() * std::cos(heading_diff);
  // TEST
  LOG_DEBUG(
      "dynamic obs id[{}], s_s: {:.4f}, e_s: {:.4f}, adc_s: {:.4f}, head_diff: "
      "{:.4f}, proj_v: {:.4f}",
      obstacle.id(), start_s, end_s, adc_s_on_reference_line, heading_diff,
      project_vel);

  if (std::abs(heading_diff) > M_PI_2) {
    // TEST
    LOG_DEBUG("reverse obs");
    if (end_s <= adc_s_on_reference_line + VehicleParam::Instance()->length()) {
      // back of vehicle, ignore
      is_valid = true;
    }
    if (project_vel < -0.5) {
      double expect_obs_forward_s = std::abs(project_vel) * 4.0;
      if (start_s - expect_obs_forward_s >=
          adc_s_on_reference_line + VehicleParam::Instance()->length() +
              front_buffer) {
        // front of vehicle, ignore
        is_valid = true;
      }
    }
  } else {
    // TEST
    LOG_DEBUG("forward obs");
    if (end_s <=
        adc_s_on_reference_line + VehicleParam::Instance()->length() * 0.5) {
      // back of vehicle, ignore
      is_valid = true;
    } else if (start_s >= adc_s_on_reference_line +
                              VehicleParam::Instance()->length() +
                              front_buffer) {
      // front of vehicle, ignore
      is_valid = true;
    } else {
      // complex situation, if the obs is totaly besides vehicle, ignore
      // convert to local
      std::vector<Vec2d> box_corner = obstacle.polygon_corners();
      Boundary obs_box_local;
      double x_target_local{0.0};
      double y_target_local{0.0};

      for (const auto& tmp_corner : box_corner) {
        x_target_local =
            (tmp_corner.x() - inside_data.vel_x) *
                cos(inside_data.vel_heading) +
            (tmp_corner.y() - inside_data.vel_y) * sin(inside_data.vel_heading);
        y_target_local =
            (inside_data.vel_x - tmp_corner.x()) *
                sin(inside_data.vel_heading) +
            (tmp_corner.y() - inside_data.vel_y) * cos(inside_data.vel_heading);
        obs_box_local.set_start_s(
            std::min(obs_box_local.start_s(), x_target_local));
        obs_box_local.set_end_s(
            std::max(obs_box_local.end_s(), x_target_local));
        obs_box_local.set_start_l(
            std::min(obs_box_local.start_l(), y_target_local));
        obs_box_local.set_end_l(
            std::max(obs_box_local.end_l(), y_target_local));
      }
      // besides vehicle ?
      if (obs_box_local.start_s() >=
              -VehicleParam::Instance()->back_edge_to_center() &&
          obs_box_local.end_s() <=
              VehicleParam::Instance()->front_edge_to_center() * 0.6 &&
          ((obs_box_local.start_l() >=
                VehicleParam::Instance()->width() * 0.5 &&
            obs_box_local.end_l() <=
                VehicleParam::Instance()->width() * 0.5 + 1.0) ||
           (obs_box_local.start_l() >=
                -VehicleParam::Instance()->width() * 0.5 - 1.0 &&
            obs_box_local.end_l() <=
                -VehicleParam::Instance()->width() * 0.5))) {
        if (project_vel <= inside_data.vel_v * 0.8) {
          is_valid = true;
          // TEST
          LOG_DEBUG("obs is besides vehicle, ignore");
          LOG_DEBUG(
              "s_s: {:.4f}, e_s: {:.4f}, s_l: {:.4f}, e_l: {:.4f}, "
              "threshold[{:.4f},{:.4f}, "
              "|{:.4f}|, |{:.4f}|]",
              obs_box_local.start_s(), obs_box_local.end_s(),
              obs_box_local.start_l(), obs_box_local.end_l(),
              -VehicleParam::Instance()->back_edge_to_center(),
              VehicleParam::Instance()->front_edge_to_center() * 0.6,
              VehicleParam::Instance()->width() * 0.5,
              VehicleParam::Instance()->width() * 0.5 + 1.0);
        }
      }
    }
  }
  LOG_INFO("igore this obs?: {}", is_valid);

  return true;
}

void SpeedStaticObsPreDecisionDecider::StaticContextInfo(
    OutsidePlannerData* const outside_data) {
  LOG_INFO("__speed_obstacle_context_static_:");
  for (const auto& context :
       outside_data->speed_obstacle_context.static_obstacles_decision) {
    LOG_INFO("obstacle[{}], collided[{}], reverse[{}]", context.obstacle.id(),
             context.collide, context.reverse);
    LOG_INFO("front upper_s, upper_t, upper_speed: {:.4f}, {:.4f}, {:.4f}",
             context.upper_points.front().first.s(),
             context.upper_points.front().first.t(),
             context.upper_points.front().second);
    LOG_INFO("back upper_s, upper_t, upper_speed: {:.4f}, {:.4f}, {:.4f}",
             context.upper_points.back().first.s(),
             context.upper_points.back().first.t(),
             context.upper_points.back().second);
    LOG_INFO("front lower_s, lower_t, lower_speed: {:.4f}, {:.4f}, {:.4f}",
             context.lower_points.front().first.s(),
             context.lower_points.front().first.t(),
             context.lower_points.front().second);
    LOG_INFO("back lower_s, lower_t, lower_speed: {:.4f}, {:.4f}, {:.4f}",
             context.lower_points.back().first.s(),
             context.lower_points.back().first.t(),
             context.lower_points.back().second);
  }
  LOG_INFO("pre static and add into static decision: ");
  for (const auto& context :
       outside_data->speed_obstacle_context.static_pre_obstacles_decision) {
    LOG_INFO("obstacle[{}], collided[{}], reverse[{}]", context.obstacle.id(),
             context.collide, context.reverse);
    LOG_INFO("front upper_s, upper_t, upper_speed: {:.4f}, {:.4f}, {:.4f}",
             context.upper_points.front().first.s(),
             context.upper_points.front().first.t(),
             context.upper_points.front().second);
    LOG_INFO("back upper_s, upper_t, upper_speed: {:.4f}, {:.4f}, {:.4f}",
             context.upper_points.back().first.s(),
             context.upper_points.back().first.t(),
             context.upper_points.back().second);
    LOG_INFO("front lower_s, lower_t, lower_speed: {:.4f}, {:.4f}, {:.4f}",
             context.lower_points.front().first.s(),
             context.lower_points.front().first.t(),
             context.lower_points.front().second);
    LOG_INFO("back lower_s, lower_t, lower_speed: {:.4f}, {:.4f}, {:.4f}",
             context.lower_points.back().first.s(),
             context.lower_points.back().first.t(),
             context.lower_points.back().second);
    outside_data->speed_obstacle_context.static_obstacles_decision.emplace_back(
        context);
  }
}

}  // namespace planning
}  // namespace neodrive
