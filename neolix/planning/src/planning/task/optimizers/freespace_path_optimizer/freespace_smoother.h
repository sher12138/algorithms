#pragma once

#include <vector>
#include <array>

namespace neodrive {
namespace planning {

class FreespaceSmoother {
 public:
  struct Config {
    float weight_smooth{1e10};
    float weight_length{1.};
    float weight_reference{1.};

    int max_iteration{500};
    float max_time{4000};
    bool is_verbose{false};
    bool is_scaled_termination{true};
    bool is_warm_start{true};
  };
  using AD2 = std::array<double, 2>;
  using AD4 = std::array<double, 4>;

 public:
  /// Must initialized with config
  FreespaceSmoother() = delete;

  /// Constructor with configure
  explicit FreespaceSmoother(Config&& conf);

  /// Smooth the points with bounds
  /// @param ref_points rvalue of reference points
  /// @param bounds rvalue of bounds
  /// @return list of points
  std::vector<AD2> Smooth(
      const std::vector<AD2>& ref_points,
      const std::vector<AD4>& bounds) const;

 private:
  Config conf_{};
};

}  // namespace planning
}  // namespace neodrive
