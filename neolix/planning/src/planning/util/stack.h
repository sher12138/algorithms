// T  need to support <, >, ==, !=, <=, >=
namespace neodrive {
namespace planning {
namespace util {
// */
// O(1) get  max, min from sliding window
template <typename T>
class ExtremumStack {
 public:
  ExtremumStack() = default;
  explicit ExtremumStack(size_t capacity) : capacity_(capacity) {}

  void Push(const T& value) {
    // single stack
    while (!max_stack_.empty() && value > max_stack_.back()) {
      max_stack_.pop_back();
    }
    max_stack_.push_back(value);

    while (!min_stack_.empty() && value < min_stack_.back()) {
      min_stack_.pop_back();
    }
    min_stack_.push_back(value);
    stack_.push_back(value);

    if (stack_.size() > capacity_) {
      auto& front = stack_.front();
      if (front == max_stack_.front()) {
        max_stack_.pop_front();
      }
      if (front == min_stack_.front()) {
        min_stack_.pop_front();
      }
      stack_.pop_front();
    }
  }

  T Max() const { return max_stack_.front(); }

  T Min() const { return min_stack_.front(); }

  T Top() const { return stack_.back(); }

  void Pop() {
    if (stack_.back() == max_stack_.back()) {
      max_stack_.pop_back();
    }
    if (stack_.back() == min_stack_.back()) {
      min_stack_.pop_back();
    }
    stack_.pop_back();
  }

  bool Empty() const { return stack_.empty(); }

  size_t Size() const { return stack_.size(); }

  void Clear() {
    stack_.clear();
    max_stack_.clear();
    min_stack_.clear();
  }

  void Resize(size_t capacity) { capacity_ = capacity; }

  void PrintExt() {
    LOG_INFO("max: {:.3f}", Max());
    LOG_INFO("min: {:.3f}", Min());
  }

 private:
  size_t capacity_ = 0;
  std::deque<T> stack_;
  std::deque<T> max_stack_;
  std::deque<T> min_stack_;
};

}  // namespace util
}  // namespace planning
}  // namespace neodrive
