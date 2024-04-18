#include "lru_cache.h"

namespace neodrive {
namespace planning {

LRUCache::LRUCache(std::size_t capacity) : capacity_(capacity) {}

ErrorCode LRUCache::put(const int key, std::unique_ptr<Obstacle>& value) {
  auto it = map_.find(key);
  if (it == map_.end()) {
    list_.push_front(key);
    Item& item = map_[key];
    item.value = std::move(value);
    item.address = list_.begin();

    if (size() > capacity()) {
      int id = list_.back();
      LRUCache::Item& lost_item_ = map_[id];
      list_.erase(lost_item_.address);
      map_.erase(id);
    }
  } else {
    if (it->second.address != list_.begin()) {
      int id = *(it->second.address);
      list_.erase(it->second.address);
      list_.push_front(id);
      it->second.address = list_.begin();
    }
    it->second.value = std::move(value);
  }
  return ErrorCode::PLANNING_OK;
}

ErrorCode LRUCache::get(const int key, Obstacle** const value) {
  if (value == nullptr) return ErrorCode::PLANNING_ERROR_NOT_FOUND;

  *value = nullptr;
  ErrorCode ret = ErrorCode::PLANNING_OK;
  auto it = map_.find(key);
  if (it != map_.end()) {
    if (it->second.address != list_.begin()) {
      int id = *(it->second.address);
      list_.erase(it->second.address);
      list_.push_front(id);
      it->second.address = list_.begin();
    }
    *value = it->second.value.get();
  } else {
    ret = ErrorCode::PLANNING_ERROR_NOT_FOUND;
  }
  return ret;
}

ErrorCode LRUCache::get_slient(const int key, Obstacle** const value) const {
  if (value == nullptr) return ErrorCode::PLANNING_ERROR_NOT_FOUND;

  *value = nullptr;
  ErrorCode ret = ErrorCode::PLANNING_OK;
  auto it = map_.find(key);
  if (it != map_.end()) {
    *value = it->second.value.get();
  } else {
    ret = ErrorCode::PLANNING_ERROR_NOT_FOUND;
  }
  return ret;
}

std::size_t LRUCache::capacity() const { return capacity_; }

std::size_t LRUCache::size() const { return map_.size(); }

void LRUCache::clear() {
  map_.clear();
  list_.clear();
}

}  // namespace planning
}  // namespace neodrive
