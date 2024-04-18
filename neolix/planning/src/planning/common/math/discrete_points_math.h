#pragma once

#include <cmath>
#include <utility>
#include <vector>

#include "neolix_log.h"

namespace neodrive {
namespace planning {

class DiscretePointsMath {
 public:
  DiscretePointsMath() = delete;

  static bool ComputePathProfile(
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<double>* headings, std::vector<double>* accumulated_s,
      std::vector<double>* kappas, std::vector<double>* dkappas);
  static bool ComputeHeading(
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<double>* headings);
};

}  // namespace planning
}  // namespace neodrive
