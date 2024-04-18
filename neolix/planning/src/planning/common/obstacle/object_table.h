#pragma once

#include <memory>
#include <string>

#include "lru_cache.h"
#include "obstacle.h"
#include "src/planning/common/planning_code_define.h"

namespace neodrive {
namespace planning {

class ObjectTable {
 public:
  ObjectTable();
  ~ObjectTable() = default;

  bool is_exist(const int id);

  bool get_obstacle(const int id, Obstacle** const obstacle);

  bool get_obstacle_slient(const int id, Obstacle** const obstacle) const;

  void put_obstacle(std::unique_ptr<Obstacle>& obstacle);

 private:
  LRUCache obstacle_cache_;
};

}  // namespace planning
}  // namespace neodrive
