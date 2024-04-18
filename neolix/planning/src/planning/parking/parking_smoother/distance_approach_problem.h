#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "src/planning/public/planning_lib_header.h"

//#include "distance_approach_ipopt_cuda_interface.h"
#include "distance_approach_ipopt_fixed_dual_interface.h"
#include "distance_approach_ipopt_fixed_ts_interface.h"
#include "distance_approach_ipopt_interface.h"
#include "distance_approach_ipopt_relax_end_interface.h"
#include "distance_approach_ipopt_relax_end_slack_interface.h"
#include "src/planning/parking/parking_config.h"

namespace neodrive {
namespace planning {

class DistanceApproachProblem {
 public:
  explicit DistanceApproachProblem(
      const DistanceApproachConfig& distance_approch_config,
      const ParkingCommonConfig common_config);

  virtual ~DistanceApproachProblem() = default;

  bool Solve(const Eigen::MatrixXd& x0, const Eigen::MatrixXd& xF,
             const Eigen::MatrixXd& last_time_u, const size_t horizon,
             const double ts, const Eigen::MatrixXd& ego,
             const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
             const Eigen::MatrixXd& l_warm_up, const Eigen::MatrixXd& n_warm_up,
             const Eigen::MatrixXd& s_warm_up,
             const std::vector<double>& XYbounds, const size_t obstacles_num,
             const Eigen::MatrixXi& obstacles_edges_num,
             const Eigen::MatrixXd& obstacles_A,
             const Eigen::MatrixXd& obstacles_b, Eigen::MatrixXd* state_result,
             Eigen::MatrixXd* control_result, Eigen::MatrixXd* time_result,
             Eigen::MatrixXd* dual_l_result, Eigen::MatrixXd* dual_n_result);

 private:
  DistanceApproachConfig distance_approch_config_;
  ParkingCommonConfig common_config_;
};

}  // namespace planning
}  // namespace neodrive
