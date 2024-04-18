#include "decision_data.h"

#include "common/data_center/data_center.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

DecisionData::DecisionData(ObjectTable& object_table,
                           const std::vector<int>& obstacle_ids,
                           const ReferenceLinePtr& reference_line,
                           const DecisionData* const last_decision_data) {
  const auto& plan_config = config::PlanningConfig::Instance()->plan_config();
  LOG_INFO("Total obs size: {}", obstacle_ids.size());
  obstacles_.reserve(obstacle_ids.size());
  for (std::size_t i = 0; i < obstacle_ids.size(); ++i) {
    Obstacle* obstacle = nullptr;
    int id = obstacle_ids[i];
    if (DataCenter::Instance()->caution_speed_odometry_obstacles().count(id)) {
      LOG_INFO("caution_speed_odometry_obstacles {}", id);
      continue;
    }
    if (object_table.get_obstacle_slient(id, &obstacle)) {
      obstacles_.emplace_back(*obstacle);
      obstacles_.back().init_with_reference_line(reference_line);
      if (!obstacles_.back().is_init()) {
        LOG_WARN("Can not get sl point of obstacle[{}], skip..", id);
        obstacles_.pop_back();
        continue;
      }
      if (last_decision_data != nullptr) {
        Obstacle* last_frame_obstacle = nullptr;
        ErrorCode ret = last_decision_data->get_obstacle_by_id(
            id, false, &last_frame_obstacle);
        if (ret == ErrorCode::PLANNING_OK) {
          obstacles_.back().SmoothBoundaryWithHistory(*last_frame_obstacle);
        }
      }
      if (obstacle->prediction_trajectories().empty()) {
        static_obstacle_.emplace_back(&obstacles_.back());
      } else {
        dynamic_obstacle_.emplace_back(&obstacles_.back());
      }
      practical_obstacle_.emplace_back(&obstacles_.back());
      all_obstacle_.emplace_back(&obstacles_.back());
      obstacle_map_[id] = &obstacles_.back();
    } else {
      LOG_INFO("Failed to get obstacle of {}, continue...", id);
    }
  }
  reference_line_ = reference_line;
  LOG_INFO("Valid obs size: {}", all_obstacle_.size());

  if (reference_line_ == nullptr) LOG_ERROR("reference_line_ is nullptr");
}

ErrorCode DecisionData::get_virtual_obstacle_by_type(
    const VirtualObstacle::VirtualType& type,
    std::vector<Obstacle*>& virtual_obstacles) const {
  if (virtual_obstacle_.empty()) {
    LOG_ERROR("no virtual obstacle");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  virtual_obstacles.clear();
  for (auto obstacle_ptr : virtual_obstacle_) {
    if (obstacle_ptr->virtual_type() == type) {
      virtual_obstacles.emplace_back(obstacle_ptr);
    }
  }
  return ErrorCode::PLANNING_OK;
}

ErrorCode DecisionData::get_obstacle_by_id(const int id, const bool is_virtual,
                                           Obstacle** const obstacle) const {
  if (obstacle == nullptr) {
    LOG_ERROR("input is nullptr");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  *obstacle = nullptr;
  if (!is_virtual) {
    const auto it = obstacle_map_.find(id);
    if (it != obstacle_map_.end()) {
      *obstacle = it->second;
      return ErrorCode::PLANNING_OK;
    }
  } else {
    return virtual_object_generator_.get_obstacle_by_id(id, obstacle);
  }
  return ErrorCode::PLANNING_ERROR_NOT_FOUND;
}

ErrorCode DecisionData::create_virtual_obstacle(
    const ReferencePoint& point, const VirtualObstacle::VirtualType& type,
    int* const id) {
  if (id == nullptr) {
    LOG_ERROR("input is nullptr");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  Obstacle* obstacle = nullptr;
  ErrorCode ret = virtual_object_generator_.create_virtual_obstacle(
      point, type, id, &obstacle);

  if (ret != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to create virtual obstacle.");
    return ret;
  }
  if (reference_line_ == nullptr || reference_line_->ref_points().size() < 2) {
    LOG_ERROR("decision data reference line is invalid");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  obstacle->init_with_reference_line(reference_line_);

  if (!obstacle->is_init()) {
    LOG_ERROR("Failed to get sl of virtual obstacle[{}].", *id);
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  LOG_DEBUG("Create_virtual_object: {}", obstacle->to_json().toStyledString());

  static_obstacle_.push_back(obstacle);
  virtual_obstacle_.push_back(obstacle);
  all_obstacle_.push_back(obstacle);
  return ret;
}

ErrorCode DecisionData::create_virtual_obstacle(
    const Vec2d& point, const double length, const double height,
    const double width, const double heading,
    const VirtualObstacle::VirtualType& type) {
  Obstacle* obstacle = nullptr;
  int id;
  if (auto ans = virtual_object_generator_.create_virtual_obstacle(
          point, length, height, width, heading, type, &id, &obstacle);
      ans != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to create virtual obstacle.");
    return ans;
  }

  obstacle->init_with_reference_line(reference_line_);
  if (!obstacle->is_init()) {
    LOG_ERROR("Failed to get sl of virtual obstacle[{}].", id);
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  LOG_DEBUG("Create_virtual_object: {}", obstacle->to_json().toStyledString());

  static_obstacle_.push_back(obstacle);
  virtual_obstacle_.push_back(obstacle);
  all_obstacle_.push_back(obstacle);
  return ErrorCode::PLANNING_OK;
}

ErrorCode DecisionData::create_lateral_virtual_obstacle(
    const Vec2d& point, const double length, const double height,
    const double width, const double heading,
    const VirtualObstacle::VirtualType& type) {
  Obstacle* obstacle = nullptr;
  int id;
  if (auto ans = virtual_object_generator_.create_virtual_obstacle(
          point, length, height, width, heading, type, &id, &obstacle);
      ans != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to create virtual obstacle.");
    return ans;
  }

  obstacle->init_with_reference_line(reference_line_);
  if (!obstacle->is_init()) {
    LOG_ERROR("Failed to get sl of virtual obstacle[{}].", id);
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  LOG_DEBUG("Create_virtual_object: {}", obstacle->to_json().toStyledString());

  lateral_virtual_obstacle_.push_back(obstacle);
  return ErrorCode::PLANNING_OK;
}

ErrorCode DecisionData::create_lateral_virtual_obstacle(
    const AABox2d& box, const double heading,
    const VirtualObstacle::VirtualType& type) {
  Obstacle* obstacle = nullptr;
  int id;
  double height = FLAGS_planning_virtual_obstacle_height;
  LOG_INFO("start to create lateral virtual obstacle by aabox.");
  Box2d box2d(box);
  std::vector<Vec2d> aabox_corners;
  std::vector<Vec2d> polygon_corners;
  Vec2d xy_center_point{};
  reference_line_->GetPointInCartesianFrame(
      {box.center().x(), box.center().y()}, &xy_center_point);
  if (auto ans = virtual_object_generator_.create_lateral_virtual_obstacle(
          xy_center_point, box.length(), 0, box.width(), heading, type, &id,
          &obstacle);
      ans != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to create virtual obstacle.");
    return ans;
  }

  obstacle->init_with_reference_line(reference_line_);
  if (!obstacle->is_init()) {
    LOG_ERROR("Failed to get sl of virtual obstacle[{}].", id);
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  LOG_DEBUG("Create_virtual_object: {}", obstacle->to_json().toStyledString());

  lateral_virtual_obstacle_.push_back(obstacle);
  return ErrorCode::PLANNING_OK;
}

ErrorCode DecisionData::create_virtual_obstacle(
    const Box2d& box, const VirtualObstacle::VirtualType& type) {
  Obstacle* obstacle = nullptr;
  int id;
  Vec2d center_point{box.center()};
  double length = box.length();
  double width = box.width();
  double height = FLAGS_planning_virtual_obstacle_height;
  double heading = box.heading();
  if (auto ans = virtual_object_generator_.create_virtual_obstacle(
          box.center(), box.length(), FLAGS_planning_virtual_obstacle_height,
          box.width(), box.heading(), type, &id, &obstacle);
      ans != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to create virtual obstacle.");
    return ans;
  }

  obstacle->init_with_reference_line(reference_line_);
  if (!obstacle->is_init()) {
    LOG_ERROR("Failed to get sl of virtual obstacle[{}].", id);
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  LOG_DEBUG("Create_virtual_object: {}", obstacle->to_json().toStyledString());

  static_obstacle_.push_back(obstacle);
  virtual_obstacle_.push_back(obstacle);
  all_obstacle_.push_back(obstacle);
  return ErrorCode::PLANNING_OK;
}

ErrorCode DecisionData::create_virtual_obstacle(
    const DiscretizedPath& path, const double init_s, const double pt_s,
    const VirtualObstacle::VirtualType& type) {
  if (path.path_points().empty() ||
      (pt_s - init_s) > path.path_points().back().s()) {
    LOG_INFO("destination is out of path");
    return ErrorCode::PLANNING_OK;
  }
  LOG_INFO("pts:{:.4f}, init_s:{:.4f}, back_s:{:.4f}", pt_s, init_s,
           path.path_points().back().s());
  PathPoint pt{};
  int index = path.query_closest_point(pt_s - init_s);
  if (!path.path_point_at(index, pt)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  LOG_INFO("pts:{:.4f}, init_s:{:.4f}, back_s:{:.4f}", pt_s, init_s,
           path.path_points().back().s());
  LOG_DEBUG(
      "pt_s, index, s, x, y, theta: {:.3f}, {}, {:.3f}, {:.3f}, {:.3f}, {:.3f}",
      pt_s, index, pt.s(), pt.x(), pt.y(), pt.theta());
  const double width = 2.0;

  Obstacle* obstacle = nullptr;
  int id;
  if (auto ans = virtual_object_generator_.create_virtual_obstacle(
          {pt.x(), pt.y()}, FLAGS_planning_virtual_obstacle_length,
          FLAGS_planning_virtual_obstacle_height, width, pt.theta(), type, &id,
          &obstacle);
      ans != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to create virtual obstacle.");
    return ans;
  }
  obstacle->init_with_reference_line(reference_line_);
  LOG_DEBUG("Create_virtual_object: {}", obstacle->to_json().toStyledString());

  static_obstacle_.push_back(obstacle);
  virtual_obstacle_.push_back(obstacle);
  all_obstacle_.push_back(obstacle);

  return ErrorCode::PLANNING_OK;
}

ErrorCode DecisionData::create_virtual_obstacle(
    const double point_s, const VirtualObstacle::VirtualType& type,
    int* const id) {
  if (id == nullptr) {
    LOG_ERROR("input is nullptr");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  Obstacle* obstacle = nullptr;
  ErrorCode ret = virtual_object_generator_.create_virtual_obstacle(
      reference_line_, point_s, type, id, &obstacle);
  if (ret != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to create virtual obstacle.");
    return ret;
  }
  obstacle->init_with_reference_line(reference_line_);
  if (!obstacle->is_init()) {
    LOG_ERROR("Failed to get sl of virtual obstacle[{}].", *id);
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  LOG_DEBUG("Create_virtual_object: {}", obstacle->to_json().toStyledString());
  static_obstacle_.push_back(obstacle);
  virtual_obstacle_.push_back(obstacle);
  all_obstacle_.push_back(obstacle);
  return ret;
}

ErrorCode DecisionData::create_virtual_obstacle(
    const double point_s, const Boundary& adc_boundary,
    const VirtualObstacle::VirtualType& type, int* const id) {
  if (id == nullptr) {
    LOG_ERROR("input is nullptr");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  auto virtual_obstacle_s = point_s;
  LOG_DEBUG(
      "point_s, adc_start/end_s, virtual_obstacle_s: {:.3f}, {:.3f}, {:.3f}, "
      "{:.3f}",
      point_s, adc_boundary.start_s(), adc_boundary.end_s(),
      virtual_obstacle_s);

  Obstacle* obstacle = nullptr;
  ErrorCode ret = virtual_object_generator_.create_virtual_obstacle(
      reference_line_, adc_boundary, virtual_obstacle_s, type, id, &obstacle);
  if (ret != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to create virtual obstacle.");
    return ret;
  }
  obstacle->init_with_reference_line(reference_line_);
  if (!obstacle->is_init()) {
    LOG_ERROR("Failed to get sl of virtual obstacle[{}].", *id);
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  LOG_DEBUG("Create_virtual_object: {}", obstacle->to_json().toStyledString());
  static_obstacle_.push_back(obstacle);
  virtual_obstacle_.push_back(obstacle);
  all_obstacle_.push_back(obstacle);
  return ret;
}

}  // namespace planning
}  // namespace neodrive
