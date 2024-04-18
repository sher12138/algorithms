#pragma once

#include <atomic>
#include <mutex>

namespace neodrive {
namespace common {
namespace util {

template <typename T>
class ThreadSafeObject {
 public:
  ThreadSafeObject() : mutex_(), object_() {}
  ~ThreadSafeObject() = default;
  void Set(const T &object) {
    std::lock_guard<std::mutex> lock(mutex_);
    object_ = object;
    updated_ = true;
  }
  void Swap(T *object) {
    std::lock_guard<std::mutex> lock(mutex_);
    object_.Swap(object);
    updated_ = false;
  }
  void Swap(T &object) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::swap(object_, object);
    updated_ = false;
  }
  void Get(T &object) {
    std::lock_guard<std::mutex> lock(mutex_);
    object = object_;
  }
  T Get() {
    std::lock_guard<std::mutex> lock(mutex_);
    return object_;
  }
  bool IsUpdated() { return updated_; }
  void Reset() { updated_ = false; }

 private:
  std::mutex mutex_;
  T object_;
  std::atomic<bool> updated_{false};
};

}  // namespace util
}  // namespace common
}  // namespace neodrive