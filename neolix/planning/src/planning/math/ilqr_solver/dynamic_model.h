#pragma once

#include <Eigen/Dense>

namespace neodrive {
namespace planning {
namespace ilqr {

// X and U represent the state and action dimension respectively.
// Dynamic model interface for a iLQR problem.
template <int X, int U>
class DynamicModel {
 public:
  using VecX = Eigen::Matrix<double, X, 1>;
  using VecU = Eigen::Matrix<double, U, 1>;
  using MatXX = Eigen::Matrix<double, X, X>;
  using MatXU = Eigen::Matrix<double, X, U>;

  struct Jacobians {
    MatXX dfdx = MatXX::Zero();
    MatXU dfdu = MatXU::Zero();
  };

  DynamicModel() = default;
  virtual ~DynamicModel() = default;

  // Compute the next step state using the current step state and action.
  virtual VecX Evaluate(const VecX& x, const VecU& u) const = 0;

  // Compute Jacobian w.r.t state and action.
  virtual Jacobians EvaluateJacobians(const VecX& x, const VecU& u) const = 0;
};

}  // namespace ilqr
}  // namespace planning
}  // namespace neodrive
