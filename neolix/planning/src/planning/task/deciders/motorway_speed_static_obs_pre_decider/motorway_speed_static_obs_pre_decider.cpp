#include "motorway_speed_static_obs_pre_decider.h"

namespace neodrive {
namespace planning {

MotorwaySpeedStaticObsPreDecider::MotorwaySpeedStaticObsPreDecider() {
  name_ = "MotorwaySpeedStaticObsPreDecider";
}

MotorwaySpeedStaticObsPreDecider::~MotorwaySpeedStaticObsPreDecider() {
  Reset();
}

ErrorCode MotorwaySpeedStaticObsPreDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
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

bool MotorwaySpeedStaticObsPreDecider::Process(TaskInfo& task_info) {
  auto outside_data_ptr =
      task_info.current_frame()->mutable_outside_planner_data();

  if (StaticObstaclePreDecision(
          task_info.reference_line(),
          task_info.current_frame()->inside_planner_data(),
          task_info.current_frame()
              ->planning_data()
              .decision_data()
              .static_obstacle(),
          outside_data_ptr) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("StaticObstaclePreDecision failed.");
    return false;
  }
  return true;
}

ErrorCode MotorwaySpeedStaticObsPreDecider::StaticObstaclePreDecision(
    const ReferenceLinePtr reference_line, const InsidePlannerData& inside_data,
    const std::vector<Obstacle*>& static_obs_vec,
    OutsidePlannerData* const outside_data) const {
  if (outside_data == nullptr) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  LOG_INFO("static_obs_vec.size = {} ", static_obs_vec.size());
  const auto& adc_sl_point = inside_data.init_sl_point;
  for (std::size_t i = 0; i < static_obs_vec.size(); ++i) {
    if (static_obs_vec[i] == nullptr || static_obs_vec[i]->is_virtual()) {
      continue;
    }
    bool need_ignore = false;
    if (!inside_data.change_lane_task_mode) {
      need_ignore = need_ignore || (static_obs_vec[i]->sub_type() ==
                                    PerceptionObstacle::ST_TREE);
    }
    if (need_ignore) {
      LOG_INFO("Ignore static obstacle id: [{}]", static_obs_vec[i]->id());
      continue;
    }
    ErrorCode ret = CollisionCheckObstacleWithoutTrajectory(
        reference_line, inside_data, *static_obs_vec[i], outside_data);
    if (ret == ErrorCode::PLANNING_ERROR_FAILED) {
      LOG_ERROR("Obstacle [{}] collision check failed.",
                static_obs_vec[i]->id());
      return ret;
    }
  }
  return ErrorCode::PLANNING_OK;
}

bool MotorwaySpeedStaticObsPreDecider::IsStaticObsNeedIgnore(
    const SLPoint& vel_sl, const Boundary& obs_boundary,
    const double& left_road_bound, const double& right_road_bound,
    const bool is_forward, bool& is_ignore) const {
  is_ignore = false;
  if (obs_boundary.start_l() > left_road_bound ||
      obs_boundary.end_l() < right_road_bound) {
    is_ignore = true;
    LOG_INFO("obs_boundary outside road, ignore.");
    return true;
  }
  double valid_s{vel_sl.s()};
  valid_s -= is_forward ? VehicleParam::Instance()->back_edge_to_center()
                        : VehicleParam::Instance()->front_edge_to_center();
  if (obs_boundary.end_s() < valid_s) {
    LOG_INFO("obs_boundary behind ego, ignore.");
    is_ignore = true;
  }
  return true;
}

bool MotorwaySpeedStaticObsPreDecider::FreespaceFilter(
    const std::vector<Boundary>& adc_sl_boundaries, const SLPoint& vel_sl,
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
    return (vel < 1.0)
               ? 0.05
               : (vel < 2.0) ? 0.10
                             : (vel < 3.5) ? 0.15 : (vel < 5.5) ? 0.20 : 0.25;
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

ErrorCode
MotorwaySpeedStaticObsPreDecider::CollisionCheckObstacleWithoutTrajectory(
    const ReferenceLinePtr reference_line, const InsidePlannerData& inside_data,
    const Obstacle& obstacle, OutsidePlannerData* const outside_data) const {
  if (outside_data == nullptr) {
    LOG_ERROR("Input is nullptr.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (obstacle.length() < 1e-4 || obstacle.width() < 1e-4) {
    LOG_INFO("Obstacle [{}] length({:.4f}) < 1e-4 || width({:.4f}) < 1e-4",
             obstacle.id(), obstacle.length(), obstacle.width());
    return ErrorCode::PLANNING_OK;
  }
  const auto& adc_boxes =
      outside_data->motorway_speed_obstacle_context.adc_boundaries;
  const auto& path_points = outside_data->path_data->path().path_points();
  const auto& mpc_config = config::PlanningConfig::Instance()
                               ->planning_research_config()
                               .mpc_velocity_plan_optimizer_config;
  if (adc_boxes.size() < 3) {
    LOG_ERROR("adc bounding boxes size < 3.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (path_points.size() != adc_boxes.size()) {
    LOG_ERROR("path_points size != adc_boxes size.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  double collide_s = path_points.back().s() + 100.0;
  std::size_t path_collide_index{path_points.size()};
  for (std::size_t i = 0; i < adc_boxes.size(); i += 2) {
    if (obstacle.polygon().has_overlap(Polygon2d(adc_boxes[i]))) {
      collide_s = path_points[i].s();
      path_collide_index = i;
      if (i > 0) {
        if (obstacle.polygon().has_overlap(Polygon2d(adc_boxes[i - 1]))) {
          collide_s = path_points[i - 1].s();
          path_collide_index = i - 1;
        }
      }
      break;
    }
  }
  if (collide_s <= path_points.back().s()) {
    MotorwayMultiCipvSpeedObstacleDecision multi_cipv_decision;
    std::vector<std::pair<STPoint, double>> lower_points{};
    std::vector<std::pair<STPoint, double>> upper_points{};
    lower_points.emplace_back(STPoint(collide_s, 0.0), 0.0);
    lower_points.emplace_back(STPoint(collide_s, 100.0), 0.0);
    upper_points.emplace_back(STPoint(path_points.back().s(), 0.0), 0.0);
    upper_points.emplace_back(STPoint(path_points.back().s(), 100.0), 0.0);
    multi_cipv_decision.lower_points = lower_points;
    multi_cipv_decision.upper_points = upper_points;
    multi_cipv_decision.obstacle = obstacle;
    multi_cipv_decision.collide = true;
    multi_cipv_decision.reverse = false;
    multi_cipv_decision.lower_adc_first_index = path_collide_index;
    outside_data->motorway_speed_obstacle_context
        .multi_cipv_static_obstacles_decision.emplace_back(multi_cipv_decision);

    MotorwaySingleCipvSpeedObstacleDecision single_cipv_decision;
    single_cipv_decision.obstacle = obstacle;
    single_cipv_decision.collide = true;
    single_cipv_decision.lower_adc_first_index = path_collide_index;
    single_cipv_decision.collide_s =
        std::fmax(collide_s - mpc_config.static_obs_collide_s_buffer, 0.0);
    single_cipv_decision.collide_v = 0.0;
    single_cipv_decision.collide_t =
        std::max(0.0, collide_s / std::max(inside_data.vel_v, 1.0) -
                          inside_data.init_point.relative_time());
    single_cipv_decision.reverse = false;
    outside_data->motorway_speed_obstacle_context.single_cipv_ostacles_decision
        .emplace_back(single_cipv_decision);

    LOG_INFO(
        "static obs {} collide is true, low index = {}, collide_s  = {:.2f}, "
        "single_cipv_collide_s = {:.2f}, collide_v = {:.2f}, collide_t = "
        "{:.2f}",
        obstacle.id(), path_collide_index, collide_s,
        single_cipv_decision.collide_s, single_cipv_decision.collide_v,
        single_cipv_decision.collide_t);
  }

  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
