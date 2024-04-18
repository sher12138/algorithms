#pragma once

#include <cstdint>
#include <deque>
#include <limits>
#include <utility>
#include <vector>

#include "common/macros.h"

namespace neodrive {
namespace planning {

/*
 * This is achieved by keeping track of the last k measurements
 * (where k is the window size), and returning the average of all but the
 * minimum and maximum measurements, which are likely to be outliers.
 */
class MeanFilter {
 public:
  void set_window_size(const int& windowsize);
  bool Update(const double measurement, double& update);

  DECLARE_SINGLETON(MeanFilter);

 private:
  void RemoveEarliest();
  void Insert(const double value);
  double GetMin() const;
  double GetMax() const;
  bool ShouldPopOldestCandidate(const std::uint_fast8_t old_time) const;

  std::uint_fast8_t window_size_ = 0;
  double sum_ = 0.0;
  std::uint_fast8_t time_ = 0;
  // front = earliest
  std::deque<double> values_;
  // front = min
  std::deque<std::pair<std::uint_fast8_t, double>> min_candidates_;
  // front = max
  std::deque<std::pair<std::uint_fast8_t, double>> max_candidates_;
  bool initialized_ = false;
};

}  // namespace planning
}  // namespace neodrive
