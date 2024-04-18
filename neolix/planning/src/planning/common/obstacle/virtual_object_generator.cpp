#include "virtual_object_generator.h"

#include "common/data_center/data_center.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/planning_logger.h"

namespace neodrive {
namespace planning {

ErrorCode VirtualObjectGenerator::create_virtual_obstacle(
    const ReferenceLinePtr& reference_line, const Boundary& adc_boundary,
    const double point_s, const VirtualObstacle::VirtualType& type,
    int* const id, Obstacle** const obstacle) {
  if (id == nullptr || obstacle == nullptr) {
    LOG_ERROR("input is nullptr");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  ReferencePoint point;
  if (!reference_line->GetNearestRefPoint(point_s, &point)) {
    LOG_ERROR("GetNearestRefPoint failed, s: {:.4f}", point_s);
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  Vec2d point2d(point.x(), point.y());
  double width = 0.0;

  if (compute_virtual_object_width(point, adc_boundary, type, &width) !=
      ErrorCode::PLANNING_OK) {
    LOG_ERROR("compute point width failed");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  return create_virtual_obstacle(point2d,
                                 FLAGS_planning_virtual_obstacle_length,
                                 FLAGS_planning_virtual_obstacle_height, width,
                                 point.heading(), type, id, obstacle);
}

ErrorCode VirtualObjectGenerator::create_virtual_obstacle(
    const ReferenceLinePtr& reference_line, const double point_s,
    const VirtualObstacle::VirtualType& type, int* const id,
    Obstacle** const obstacle) {
  if (id == nullptr || obstacle == nullptr) {
    LOG_ERROR("input is nullptr");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  ReferencePoint point;
  if (!reference_line->GetNearestRefPoint(point_s, &point)) {
    LOG_ERROR("GetNearestRefPoint failed, s: {:.4f}", point_s);
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  Vec2d point2d(point.x(), point.y());
  double width = 0.0;

  if (compute_virtual_object_width(point, &width) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("compute point width failed");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  return create_virtual_obstacle(point2d,
                                 FLAGS_planning_virtual_obstacle_length,
                                 FLAGS_planning_virtual_obstacle_height, width,
                                 point.heading(), type, id, obstacle);
}

ErrorCode VirtualObjectGenerator::create_virtual_obstacle(
    const ReferencePoint& point, const VirtualObstacle::VirtualType& type,
    int* const id, Obstacle** const obstacle) {
  if (id == nullptr || obstacle == nullptr)
    return ErrorCode::PLANNING_ERROR_FAILED;

  Vec2d point2d(point.x(), point.y());
  double width = 0.0;

  if (compute_virtual_object_width(point, &width) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to compute point width.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  ErrorCode ret =
      create_virtual_obstacle(point2d, FLAGS_planning_virtual_obstacle_length,
                              FLAGS_planning_virtual_obstacle_height, width,
                              point.heading(), type, id, obstacle);

  if (ret != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to create virtual obstacle.");
    return ret;
  }
  *obstacle = obstacles_[*id].get();
  return ErrorCode::PLANNING_OK;
}

ErrorCode VirtualObjectGenerator::get_obstacle_by_id(
    const int id, Obstacle** const obstacle_ptr_ptr) const {
  if (obstacle_ptr_ptr == nullptr) return ErrorCode::PLANNING_ERROR_FAILED;

  auto obstacle = obstacles_.find(id);
  if (obstacle != obstacles_.end()) {
    *obstacle_ptr_ptr = obstacle->second.get();
    return ErrorCode::PLANNING_OK;
  } else {
    LOG_ERROR("virtual obstacle [{}] not found", id);
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
}

ErrorCode VirtualObjectGenerator::get_obstacle_ids_by_type(
    const VirtualObstacle::VirtualType& type,
    std::vector<int>* const ids) const {
  if (ids == nullptr) return ErrorCode::PLANNING_ERROR_FAILED;

  ids->clear();
  const auto& ids_iter = virtual_object_id_map_.find(type);
  if (ids_iter == virtual_object_id_map_.end()) {
    LOG_INFO("type not found in virtual_object_id_map");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  for (const auto& id : ids_iter->second) {
    ids->emplace_back(id);
  }
  return ErrorCode::PLANNING_OK;
}

void VirtualObjectGenerator::get_virtual_obstacles(
    std::vector<Obstacle*>* const obstacles) const {
  if (obstacles == nullptr) return;
  obstacles->clear();
  for (auto it = obstacles_.begin(); it != obstacles_.end(); ++it) {
    obstacles->push_back(it->second.get());
  }
}

ErrorCode VirtualObjectGenerator::create_virtual_obstacle(
    const Vec2d& point, const double length, const double height,
    const double width, const double heading,
    const VirtualObstacle::VirtualType& type, int* const id,
    Obstacle** const obstacle) {
  if (id == nullptr || obstacle == nullptr)
    return ErrorCode::PLANNING_ERROR_FAILED;

  std::unique_ptr<Obstacle> obstacle_ptr = std::make_unique<Obstacle>();
  *id = last_id_ - 1;
  obstacle_ptr->set_center(point);
  obstacle_ptr->set_width(width);
  obstacle_ptr->set_heading(heading);
  obstacle_ptr->set_length(length);
  obstacle_ptr->set_height(height);
  obstacle_ptr->set_speed(0.0);
  obstacle_ptr->set_type(Obstacle::ObstacleType::UNKNOWN);
  obstacle_ptr->set_virtual(true);
  obstacle_ptr->set_id(*id);
  obstacle_ptr->set_virtual_type(type);
  LOG_INFO("virtual obstacle type:{}",
           static_cast<int>(obstacle_ptr->virtual_type()));

  obstacle_ptr->mutable_decision()->emplace_back(
      0.0, Decision::DecisionType::STOP_DOWN);
  *obstacle = obstacle_ptr.get();
  *((*obstacle)->mutable_polygon()) = Polygon2d((*obstacle)->bounding_box());
  obstacles_[*id] = std::move(obstacle_ptr);
  virtual_object_id_map_[type].emplace_back(*id);
  --last_id_;
  LOG_INFO("last id:{}", last_id_);
  return ErrorCode::PLANNING_OK;
}

ErrorCode VirtualObjectGenerator::create_lateral_virtual_obstacle(
    const Vec2d& point, const double length, const double height,
    const double width, const double heading,
    const VirtualObstacle::VirtualType& type, int* const id,
    Obstacle** const obstacle) {
  if (id == nullptr || obstacle == nullptr)
    return ErrorCode::PLANNING_ERROR_FAILED;

  std::unique_ptr<Obstacle> obstacle_ptr = std::make_unique<Obstacle>();
  *id = last_id_ - 1;
  obstacle_ptr->set_center(point);
  obstacle_ptr->set_width(width);
  obstacle_ptr->set_heading(heading);
  obstacle_ptr->set_length(length);
  obstacle_ptr->set_height(height);
  obstacle_ptr->set_speed(0.0);
  obstacle_ptr->set_type(Obstacle::ObstacleType::UNKNOWN);
  obstacle_ptr->set_virtual(true);
  obstacle_ptr->set_id(*id);
  obstacle_ptr->set_virtual_type(type);
  LOG_INFO("lateral virtual obstacle type:{}",
           static_cast<int>(obstacle_ptr->virtual_type()));

  obstacle_ptr->mutable_decision()->emplace_back(
      0.0, Decision::DecisionType::STOP_DOWN);
  *obstacle = obstacle_ptr.get();
  *((*obstacle)->mutable_polygon()) = Polygon2d((*obstacle)->bounding_box());
  obstacles_[*id] = std::move(obstacle_ptr);
  --last_id_;
  LOG_INFO("last id:{}", last_id_);
  return ErrorCode::PLANNING_OK;
}

ErrorCode VirtualObjectGenerator::compute_virtual_object_width(
    const ReferencePoint& point, double* const width) const {
  if (width == nullptr) return ErrorCode::PLANNING_ERROR_FAILED;

  *width = point.left_road_bound() + point.right_road_bound();
  *width = std::max(FLAGS_planning_virtual_obstacle_min_width, *width);
  *width = std::min(FLAGS_planning_virtual_obstacle_max_width, *width);

  return ErrorCode::PLANNING_OK;
}

ErrorCode VirtualObjectGenerator::compute_virtual_object_width(
    const ReferencePoint& point, const Boundary& adc_boundary,
    const VirtualObstacle::VirtualType& type, double* const width) const {
  if (width == nullptr) return ErrorCode::PLANNING_ERROR_FAILED;

  if (type == VirtualObstacle::TRAFFIC_LIGHT) {
    *width = 2 * std::max(std::abs(adc_boundary.end_l()),
                          std::abs(adc_boundary.start_l()));
  }
  *width = std::max(point.left_road_bound() + point.right_road_bound(), *width);
  *width = std::max(FLAGS_planning_virtual_obstacle_min_width, *width);
  *width = std::min(FLAGS_planning_virtual_obstacle_max_width, *width);

  return ErrorCode::PLANNING_OK;
}
}  // namespace planning
}  // namespace neodrive
