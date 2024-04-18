#include "internal_data_handle.h"
namespace neodrive {
namespace planning {
namespace util {
namespace internal_data_handle {

namespace {
constexpr double kLaneDis = 1.5;
constexpr double kOverlapGap = 1.0;
}  // namespace

std::vector<MatrixInterval> VecInternalCombine(
    std::vector<MatrixInterval>& intervals) {
  if (intervals.empty()) return {};
  std::vector<MatrixInterval> mergedIntervals;
  std::sort(intervals.begin(), intervals.end(),
            [](MatrixInterval& a, MatrixInterval& b) {
              return a.Longitudinal().end() < b.Longitudinal().end()
                         ? a.Longitudinal().end() != b.Longitudinal().end()
                         : a.Longitudinal().start() < b.Longitudinal().start();
            });

  for (const auto& interval : intervals) {
    if (interval.Lateral().start() == 0 && interval.Lateral().end() == 0) {
      continue;
    }
    LOG_INFO("processing {:.3f},{:.3f}", interval.Longitudinal().start(),
             interval.Longitudinal().end());
    if (mergedIntervals.empty() ||
        interval.Longitudinal().start() >
            mergedIntervals.back().Longitudinal().end() + kOverlapGap) {
      mergedIntervals.push_back(interval);
    } else {
      // exist overlap
      bool sign1 = std::abs(interval.Lateral().start()) <
                   std::abs(mergedIntervals.back().Lateral().start());
      bool sign2 = std::abs(interval.Lateral().end()) <
                   std::abs(mergedIntervals.back().Lateral().end());
      mergedIntervals.back() = MatrixInterval(
          VecInterval(!sign1 ? mergedIntervals.back().Lateral().start()
                             : interval.Lateral().start(),
                      sign2 ? mergedIntervals.back().Lateral().end()
                            : interval.Lateral().end()),

          VecInterval(mergedIntervals.back().Longitudinal().start(),
                      std::max(mergedIntervals.back().Longitudinal().end(),
                               interval.Longitudinal().end())));
    }
  }
  return mergedIntervals;
}

std::vector<MatrixInterval> MatrixInternalCombine(
    std::vector<MatrixInterval>& input_data) {
  // Assuming that there is a relatively concentrated dimension value, the
  // problem can be transformed into k one-dimensional merged temperatures.
  // 1. extract l value and construct diff
  if (input_data.empty()) return {};
  std::vector<double> l_value{};
  for (const auto& matrix : input_data) {
    l_value.push_back(matrix.Lateral().end());  // only consider right fence
  }
  std::sort(l_value.begin(), l_value.end());
  // divide l range
  // sliding window generate  l
  // cloest fence
  const double cloest_ego_l =
      std::min(std::fabs(*std::min_element(l_value.begin(), l_value.end())),
               std::fabs(*std::max_element(l_value.begin(), l_value.end())));
  // find no obs l,

  for (auto& data : input_data) {
    if (std::abs(std::abs(data.Lateral().start()) - cloest_ego_l) > kLaneDis) {
      data.Clear();
    }
  }
  return VecInternalCombine(input_data);
}

}  // namespace internal_data_handle
}  // namespace util
}  // namespace planning
}  // namespace neodrive