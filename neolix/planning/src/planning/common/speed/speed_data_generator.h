#pragma once

#include "speed_point.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

class SpeedDataGenerator {
  // s = s0 + v0 * t + a0 * t^2 + j0 * t^3 + d0 * t^4 + f0 * t^5
 public:
  SpeedDataGenerator() = default;
  ~SpeedDataGenerator() = default;

  bool add_point_sequence(const std::vector<STPoint>& st_profile,
                          const double init_speed, const double init_acc,
                          const double init_jerk);
  bool get_speed_point_at_time(const double time,
                               SpeedPoint* const speed_point);
  void solve();

 private:
  std::vector<STPoint> points_;
  STPoint init_point_;
  double init_speed_ = 0.0;
  double init_acc_ = 0.0;   // acc/2
  double init_jerk_ = 0.0;  // jerk/3
  double derk_ = 0.0;       // derk/4
  double ferk_ = 0.0;
  Eigen::MatrixXd left_matrix_;
  Eigen::VectorXd b_;
};

}  // namespace planning
}  // namespace neodrive
