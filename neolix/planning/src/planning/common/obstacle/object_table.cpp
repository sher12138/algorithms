#include "object_table.h"

#include "src/planning/common/planning_gflags.h"

namespace neodrive {
namespace planning {

ObjectTable::ObjectTable()
    : obstacle_cache_(FLAGS_planning_object_table_obstacle_capacity) {}

bool ObjectTable::is_exist(const int id) {
  Obstacle* obstacle = nullptr;
  if (obstacle_cache_.get_slient(id, &obstacle) == ErrorCode::PLANNING_OK)
    return true;
  else
    return false;
}

bool ObjectTable::get_obstacle(const int id, Obstacle** const obstacle) {
  if (obstacle_cache_.get(id, obstacle) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to get obstacle by id [{}]", id);
    return false;
  }
  return true;
}

bool ObjectTable::get_obstacle_slient(const int id,
                                      Obstacle** const obstacle) const {
  if (obstacle_cache_.get_slient(id, obstacle) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to get obstacle by id [{}]", id);
    return false;
  }
  return true;
}

void ObjectTable::put_obstacle(std::unique_ptr<Obstacle>& obstacle) {
  if (obstacle_cache_.put(obstacle->id(), obstacle) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to put the obstacle [{}].", obstacle->id());
  }
}

}  // namespace planning
}  // namespace neodrive
