#pragma once

#include <algorithm>

#include "dual_variable_warm_start_ipopt_interface.h"
#include "dual_variable_warm_start_ipopt_qp_interface.h"
#include "dual_variable_warm_start_osqp_interface.h"
#include "dual_variable_warm_start_slack_osqp_interface.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/parking/parking_config.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

class DualVariableWarmStartProblem {
 public:
  explicit DualVariableWarmStartProblem(
      const ParkingDualVariableConfig& dualvariable_config);

  virtual ~DualVariableWarmStartProblem() = default;

  bool Solve(const size_t horizon, const double ts, const Eigen::MatrixXd& ego,
             const size_t obstacles_num,
             const Eigen::MatrixXi& obstacles_edges_num,
             const Eigen::MatrixXd& obstacles_A,
             const Eigen::MatrixXd& obstacles_b, const Eigen::MatrixXd& xWS,
             Eigen::MatrixXd* l_warm_up, Eigen::MatrixXd* n_warm_up,
             Eigen::MatrixXd* s_warm_up);

 private:
  ParkingDualVariableConfig dualvariable_config_;
};

}  // namespace planning
}  // namespace neodrive
