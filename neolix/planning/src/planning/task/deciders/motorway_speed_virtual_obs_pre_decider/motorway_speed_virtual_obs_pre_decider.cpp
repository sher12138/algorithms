#include "motorway_speed_virtual_obs_pre_decider.h"

namespace neodrive {
namespace planning {

MotorwaySpeedVirtualObsPreDecider::MotorwaySpeedVirtualObsPreDecider() {
  name_ = "MotorwaySpeedVirtualObsPreDecider";
}

MotorwaySpeedVirtualObsPreDecider::~MotorwaySpeedVirtualObsPreDecider() {
  Reset();
}

ErrorCode MotorwaySpeedVirtualObsPreDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  auto outside_data_ptr = frame->mutable_outside_planner_data();
  if (outside_data_ptr == nullptr) {
    LOG_ERROR("mutable_outside_planner_data == nullptr.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (outside_data_ptr->path_data == nullptr) {
    LOG_ERROR("path_data == nullptr.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!VirtualObsPreDecision(
          task_info.reference_line(), frame->inside_planner_data(),
          frame->planning_data().decision_data().virtual_obstacle(),
          outside_data_ptr)) {
    LOG_ERROR("VirtualObsPreDecision failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  return ErrorCode::PLANNING_OK;
}

bool MotorwaySpeedVirtualObsPreDecider::VirtualObsPreDecision(
    const ReferenceLinePtr reference_line, const InsidePlannerData& inside_data,
    const std::vector<Obstacle*>& virtual_obs_vec,
    OutsidePlannerData* const outside_data) const {
  LOG_INFO("___virtual_obstacle_pre_decision infos___:");
  for (std::size_t i = 0; i < virtual_obs_vec.size(); ++i) {
    if (virtual_obs_vec[i] == nullptr || !virtual_obs_vec[i]->is_virtual())
      continue;
    LOG_DEBUG("id : [{}]", virtual_obs_vec[i]->id());
    LOG_DEBUG("start virtual obstacle collision check:");
    bool ret = CollisionCheckVirtualObstacle(reference_line, inside_data,
                                             *virtual_obs_vec[i], outside_data);
    if (!ret) {
      LOG_ERROR("collision check failed.");
      return false;
    }
  }

  return true;
}

bool MotorwaySpeedVirtualObsPreDecider::CollisionCheckVirtualObstacle(
    const ReferenceLinePtr reference_line, const InsidePlannerData& inside_data,
    const Obstacle& obstacle, OutsidePlannerData* const outside_data) const {
  if (outside_data == nullptr) {
    LOG_ERROR("Input is nullptr.");
    return false;
  }
  if (obstacle.length() < 1e-4 || obstacle.width() < 1e-4) {
    LOG_INFO("Obstacle [{}] length({:.4f}) < 1e-4 || width({:.4f}) < 1e-4",
             obstacle.id(), obstacle.length(), obstacle.width());
    return true;
  }
  const auto& adc_bounding_boxes =
      outside_data->motorway_speed_obstacle_context.adc_boundaries;
  const auto& path_points = outside_data->path_data->path().path_points();
  if (adc_bounding_boxes.size() < 3) {
    LOG_ERROR("adc_bounding_boxes.size() < 3.");
    return false;
  }
  if (path_points.size() != adc_bounding_boxes.size()) {
    LOG_ERROR("path_points size != adc_bounding_boxes size.");
    return false;
  }

  double collide_s = path_points.back().s() + 100.0;
  std::size_t path_collide_index{path_points.size()};
  for (std::size_t i = 0; i < adc_bounding_boxes.size(); i += 2) {
    if (obstacle.polygon().has_overlap(Polygon2d(adc_bounding_boxes[i]))) {
      collide_s = path_points[i].s();
      path_collide_index = i;
      if (i > 0 && obstacle.polygon().has_overlap(
                       Polygon2d(adc_bounding_boxes[i - 1]))) {
        collide_s = path_points[i - 1].s();
        path_collide_index = i;
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
        .multi_cipv_virtual_obstacle_decision.emplace_back(multi_cipv_decision);

    MotorwaySingleCipvSpeedObstacleDecision single_cipv_decision;
    single_cipv_decision.obstacle = obstacle;
    single_cipv_decision.collide = true;
    single_cipv_decision.lower_adc_first_index = path_collide_index;
    single_cipv_decision.collide_s = std::max(collide_s, 0.0);
    single_cipv_decision.collide_v = 0.0;
    single_cipv_decision.collide_t =
        std::max(0.1, collide_s / std::max(inside_data.vel_v, 1.0) -
                          inside_data.init_point.relative_time());
    single_cipv_decision.reverse = false;
    outside_data->motorway_speed_obstacle_context.single_cipv_ostacles_decision
        .emplace_back(single_cipv_decision);

    LOG_INFO("obs {} , obs.source_type() = {}", obstacle.id(),
             VirtualObstacle::VirtualType_Name(obstacle.virtual_type()));

    LOG_INFO(
        "virtual obs {} collide is true, low index = {}, collide_s  = {:.2f}, "
        "single_cipv_collide_s = {:.2f}, collide_v = {:.2f}, collide_t = "
        "{:.2f}",
        obstacle.id(), path_collide_index, collide_s,
        single_cipv_decision.collide_s, single_cipv_decision.collide_v,
        single_cipv_decision.collide_t);
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive
