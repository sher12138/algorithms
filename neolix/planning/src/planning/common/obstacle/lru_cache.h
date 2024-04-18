#pragma once

#include <list>
#include <memory>
#include <unordered_map>

#include "obstacle.h"
#include "src/planning/common/planning_code_define.h"
#include "src/planning/common/planning_macros.h"

namespace neodrive {
namespace planning {

class LRUCache {
 public:
  explicit LRUCache(std::size_t capacity);
  ~LRUCache() = default;

  ErrorCode put(const int key, std::unique_ptr<Obstacle>& value);
  ErrorCode get(const int key, Obstacle** const value);
  ErrorCode get_slient(const int key, Obstacle** const value) const;
  std::size_t capacity() const;
  std::size_t size() const;
  void clear();

 private:
  struct Item {
    std::unique_ptr<Obstacle> value;
    std::list<int>::iterator address;
  };
  std::size_t capacity_ = 0;
  std::unordered_map<int, Item> map_{};
  std::list<int> list_{};

 private:
  DISALLOW_COPY_AND_ASSIGN_PLANNING(LRUCache);
};

}  // namespace planning
}  // namespace neodrive
