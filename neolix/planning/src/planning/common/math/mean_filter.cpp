#include "mean_filter.h"

namespace neodrive {
namespace planning {

MeanFilter::MeanFilter() {}

void MeanFilter::set_window_size(const int& windowsize) {
  window_size_ = windowsize;
  initialized_ = true;
}
double MeanFilter::GetMin() const {
  if (min_candidates_.empty()) {
    return std::numeric_limits<double>::infinity();
  } else {
    return min_candidates_.front().second;
  }
}

double MeanFilter::GetMax() const {
  if (max_candidates_.empty()) {
    return -std::numeric_limits<double>::infinity();
  } else {
    return max_candidates_.front().second;
  }
}

bool MeanFilter::Update(const double measurement, double& update) {
  if (initialized_ == false) return false;

  if (values_.size() > window_size_) return false;
  if (min_candidates_.size() > window_size_) return false;
  if (max_candidates_.size() > window_size_) return false;
  ++time_;
  time_ %= static_cast<std::uint_fast8_t>(2 * window_size_);
  if (values_.size() == window_size_) {
    RemoveEarliest();
  }
  Insert(measurement);
  if (values_.size() > 2) {
    update =
        (sum_ - GetMin() - GetMax()) / static_cast<double>(values_.size() - 2);
    return true;
  } else {
    update = sum_ / static_cast<double>(values_.size());
    return true;
  }
}

bool MeanFilter::ShouldPopOldestCandidate(
    const std::uint_fast8_t old_time) const {
  if (old_time < window_size_) {
    return old_time + window_size_ == time_;
  } else if (time_ < window_size_) {
    return old_time == time_ + window_size_;
  } else {
    return false;
  }
}

void MeanFilter::RemoveEarliest() {
  if (values_.size() != window_size_) return;
  double removed = values_.front();
  values_.pop_front();
  sum_ -= removed;
  if (ShouldPopOldestCandidate(min_candidates_.front().first)) {
    min_candidates_.pop_front();
  }
  if (ShouldPopOldestCandidate(max_candidates_.front().first)) {
    max_candidates_.pop_front();
  }
}

void MeanFilter::Insert(const double value) {
  values_.push_back(value);
  sum_ += value;
  while (min_candidates_.size() > 0 && min_candidates_.back().second > value) {
    min_candidates_.pop_back();
  }
  min_candidates_.push_back(std::make_pair(time_, value));
  while (max_candidates_.size() > 0 && max_candidates_.back().second < value) {
    max_candidates_.pop_back();
  }
  max_candidates_.push_back(std::make_pair(time_, value));
}

}  // namespace planning
}  // namespace neodrive
