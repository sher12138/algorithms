#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "decision.pb.h"
#include "obstacle.h"
#include "src/planning/common/planning_code_define.h"
#include "src/planning/reference_line/reference_line.h"
namespace neodrive {
namespace planning {

using neodrive::global::planning::VirtualObstacle;
struct EnumClassHash {
  template <typename T>
  std::size_t operator()(T t) const {
    return static_cast<std::size_t>(t);
  }
};

class VirtualObjectGenerator {
 public:
  ErrorCode create_virtual_obstacle(const ReferenceLinePtr& reference_line,
                                    const Boundary& adc_boundary,
                                    const double point_s,
                                    const VirtualObstacle::VirtualType& type,
                                    int* const id, Obstacle** const obstacle);

  ErrorCode create_virtual_obstacle(const ReferenceLinePtr& reference_line,
                                    const double point_s,
                                    const VirtualObstacle::VirtualType& type,
                                    int* const id, Obstacle** const obstacle);

  ErrorCode create_virtual_obstacle(const ReferencePoint& point,
                                    const VirtualObstacle::VirtualType& type,
                                    int* const id, Obstacle** const obstacle);

  ErrorCode get_obstacle_by_id(const int id,
                               Obstacle** const obstacle_ptr) const;

  ErrorCode get_obstacle_ids_by_type(const VirtualObstacle::VirtualType& type,
                                     std::vector<int>* const ids) const;

  void get_virtual_obstacles(std::vector<Obstacle*>* const obstacles) const;

  ErrorCode create_virtual_obstacle(
      const Vec2d& point, const double length, const double height,
      const double width, const double heading,
      const VirtualObstacle::VirtualType& virtual_object_type, int* const id,
      Obstacle** const obstacle);
  ErrorCode create_lateral_virtual_obstacle(
      const Vec2d& point, const double length, const double height,
      const double width, const double heading,
      const VirtualObstacle::VirtualType& type, int* const id,
      Obstacle** const obstacle);

 private:
  ErrorCode compute_virtual_object_width(const ReferencePoint& point,
                                         double* const width) const;

  ErrorCode compute_virtual_object_width(
      const ReferencePoint& point, const Boundary& adc_boundary,
      const VirtualObstacle::VirtualType& type, double* const width) const;

 private:
  std::unordered_map<VirtualObstacle::VirtualType, std::vector<int>,
                     EnumClassHash>
      virtual_object_id_map_;
  std::unordered_map<int, std::unique_ptr<Obstacle>> obstacles_;
  int last_id_ = 0;
};

}  // namespace planning
}  // namespace neodrive
