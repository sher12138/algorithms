
#pragma once
#include <math.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include "common/util/time_logger.h"

namespace neodrive {
namespace planning {
namespace util {
namespace internal_data_handle {

// internal combine -->

class VecInterval {
 public:
  VecInterval() = default;
  VecInterval(double start, double end) : start_(start), end_(end) {}
  double start() const { return start_; }
  double end() const { return end_; }
  void clear() {
    start_ = 0.0;
    end_ = 0.0;
  }

 private:
  double start_ = 0.0;
  double end_ = 0.0;
};

//
class MatrixInterval {
 public:
  MatrixInterval() = default;
  MatrixInterval(VecInterval lateral_range, VecInterval longitudinal_range)
      : lateral_(lateral_range), longitudinal_(longitudinal_range) {}

  const VecInterval& Lateral() const { return lateral_; }
  const VecInterval& Longitudinal() const { return longitudinal_; }
  void Clear() {
    lateral_.clear();
    longitudinal_.clear();
  }

 private:
  VecInterval lateral_{};
  VecInterval longitudinal_{};
};

std::vector<MatrixInterval> VecInternalCombine(
    std::vector<MatrixInterval>& intervals);

// Non-general practice, only used for the situation where the values of another
// dimension are concentrated.  The general version is yet to be implemented
std::vector<MatrixInterval> MatrixInternalCombine(
    std::vector<MatrixInterval>& input_data);

}  // namespace internal_data_handle
}  // namespace util
}  // namespace planning
}  // namespace neodrive