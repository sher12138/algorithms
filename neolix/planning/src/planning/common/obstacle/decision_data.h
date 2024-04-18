#pragma once

#include <list>

#include "common/macros.h"
#include "object_table.h"
#include "obstacle.h"
#include "src/planning/common/path/discretized_path.h"
#include "src/planning/common/planning_code_define.h"
#include "src/planning/common/speed/speed_point.h"
#include "src/planning/common/speed/st_point.h"
#include "src/planning/common/st_graph_data/st_graph_boundary.h"
#include "virtual_object_generator.h"

namespace neodrive {
namespace planning {

class DecisionData {
 public:
  DecisionData() = default;
  DecisionData(ObjectTable& object_table, const std::vector<int>& obstacle_ids,
               const ReferenceLinePtr& reference_line,
               const DecisionData* const last_decision_data = nullptr);
  ~DecisionData() = default;

  ErrorCode get_obstacle_by_id(const int id, const bool is_virtual,
                               Obstacle** const obstacle) const;

  ErrorCode get_virtual_obstacle_by_type(
      const VirtualObstacle::VirtualType& type,
      std::vector<Obstacle*>& virtual_obstacles) const;

  ErrorCode create_virtual_obstacle(const ReferencePoint& point,
                                    const VirtualObstacle::VirtualType& type,
                                    int* const id);

  ErrorCode create_virtual_obstacle(const Vec2d& point, const double length,
                                    const double height, const double width,
                                    const double heading,
                                    const VirtualObstacle::VirtualType& type);

  ErrorCode create_lateral_virtual_obstacle(
      const Vec2d& point, const double length, const double height,
      const double width, const double heading,
      const VirtualObstacle::VirtualType& type);

  ErrorCode create_lateral_virtual_obstacle(
      const AABox2d& box, const double heading,
      const VirtualObstacle::VirtualType& type);

  ErrorCode create_virtual_obstacle(const DiscretizedPath& path,
                                    const double init_s, const double pt_s,
                                    const VirtualObstacle::VirtualType& type);

  ErrorCode create_virtual_obstacle(const double point_s,
                                    const VirtualObstacle::VirtualType& type,
                                    int* const id);

  ErrorCode create_virtual_obstacle(const double point_s,
                                    const Boundary& adc_boundary,
                                    const VirtualObstacle::VirtualType& type,
                                    int* const id);

  ErrorCode create_virtual_obstacle(const Box2d& box,
                                    const VirtualObstacle::VirtualType& type);

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(std::vector<Obstacle*>, static_obstacle);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(std::vector<Obstacle*>, dynamic_obstacle);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(std::vector<Obstacle*>, virtual_obstacle);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(std::vector<Obstacle*>, practical_obstacle);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(std::vector<Obstacle*>,
                                   lateral_virtual_obstacle);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(std::vector<Obstacle*>, all_obstacle);

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(std::vector<AABox2d>, lateral_virtual_boxes);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(ReferenceLinePtr, reference_line);

 private:
  std::vector<Obstacle*> static_obstacle_;
  std::vector<Obstacle*> dynamic_obstacle_;
  std::vector<Obstacle*> virtual_obstacle_;
  std::vector<Obstacle*> practical_obstacle_;
  std::vector<Obstacle*> lateral_virtual_obstacle_;
  std::vector<Obstacle*> all_obstacle_;

  std::vector<AABox2d> lateral_virtual_boxes_;
  std::vector<Obstacle> obstacles_;
  std::unordered_map<int, Obstacle*> obstacle_map_;
  ReferenceLinePtr reference_line_;
  VirtualObjectGenerator virtual_object_generator_;
};

}  // namespace planning
}  // namespace neodrive
