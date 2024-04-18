#pragma once

#include <array>
#include <vector>

#include "src/planning/math/common/occupy_map.h"

namespace neodrive {
namespace planning {

class IterativeAncheringPathSmoother {
 public:
  using AD3 = std::array<double, 3>;
  struct Config {
    double half_bound{1};

    double bound_move_step{0.5};

    int max_iterative_loop{10};
  };

 public:
  /// Smooth the points
  /// @param om OccupyMap
  /// @param ref_pts Reference points
  /// @return Result points
  std::vector<AD3> Smooth(const OccupyMap& om, std::vector<AD3> ref_pts,
                          const bool isend);

 private:
  const Config conf_{};
};

}  // namespace planning
}  // namespace neodrive
