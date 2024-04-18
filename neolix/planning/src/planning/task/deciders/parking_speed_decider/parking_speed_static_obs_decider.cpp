#include "parking_speed_static_obs_decider.h"

namespace neodrive {
namespace planning {

ParkingSpeedStaticObsDecider::ParkingSpeedStaticObsDecider() {
  name_ = "ParkingSpeedStaticObsDecider";
}

void ParkingSpeedStaticObsDecider::Reset() {}

ParkingSpeedStaticObsDecider::~ParkingSpeedStaticObsDecider() { Reset(); }

void ParkingSpeedStaticObsDecider::SaveTaskResults(TaskInfo& task_info) {}

ErrorCode ParkingSpeedStaticObsDecider::Execute(TaskInfo& task_info) {
  auto& frame = task_info.current_frame();
  const auto& inside_data = frame->inside_planner_data();
  LOG_INFO(">>>> start execute {}", name_);
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  if (frame->outside_planner_data().path_data == nullptr) {
    LOG_ERROR("path_data == nullptr.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (!StaticObsPreDecision(task_info)) {
    LOG_ERROR("StaticObsPreDecision failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  return ErrorCode::PLANNING_OK;
}

bool ParkingSpeedStaticObsDecider::StaticObsPreDecision(
    TaskInfo& task_info) const {
  LOG_INFO("___static_obstacle_pre_decision infos___:");
  const auto& plan_research_config =
      config::PlanningConfig::Instance()->planning_research_config();
  const auto& plan_config = config::PlanningConfig::Instance()->plan_config();
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  auto outside_data = task_info.current_frame()->mutable_outside_planner_data();
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
  const auto& static_obs_vec = task_info.current_frame()
                                   ->planning_data()
                                   .decision_data()
                                   .static_obstacle();
  for (auto obs_ptr : static_obs_vec) {
    if (obs_ptr == nullptr || obs_ptr->is_virtual()) {
      continue;
    }
    LOG_DEBUG("id: [{}]", obs_ptr->id());
    bool need_ignore{false};
    IsStaticObsNeedIgnore(adc_sl_point, obs_ptr->PolygonBoundary(), true,
                          need_ignore);
    if (need_ignore) {
      LOG_INFO("Ignore static obs [{}]", obs_ptr->id());
      continue;
    }
    LOG_INFO("start static obstacle collision check:");
    ErrorCode ret = CollisionCheckObstacleWithoutTrajectory(
        task_info, inside_data, *obs_ptr, outside_data,
        &(outside_data->speed_obstacle_context.static_obstacles_decision));
    if (ret == ErrorCode::PLANNING_ERROR_FAILED) {
      LOG_ERROR("obs [{}] collision check failed.", obs_ptr->id());
      return false;
    }
  }
  return true;
}

bool ParkingSpeedStaticObsDecider::IsStaticObsNeedIgnore(
    const SLPoint& vel_sl, const Boundary& obs_boundary, const bool is_forward,
    bool& is_ignore) const {
  is_ignore = false;
  double valid_s{vel_sl.s()};
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

ErrorCode ParkingSpeedStaticObsDecider::CollisionCheckObstacleWithoutTrajectory(
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

  double collide_s{path_points.back().s() + 100.0};
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

}  // namespace planning
}  // namespace neodrive
