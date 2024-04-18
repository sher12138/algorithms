#pragma once

#include <Eigen/Dense>
#include <string>
#include <unordered_map>

namespace neodrive {
namespace planning {
namespace ilqr {

// X and U represent the state and action dimension respectively.
// Cost function interface for a iLQR problem.
template <int X, int U>
class CostFunction {
 public:
  using VecX = Eigen::Matrix<double, X, 1>;
  using VecU = Eigen::Matrix<double, U, 1>;
  using MatXX = Eigen::Matrix<double, X, X>;
  using MatUU = Eigen::Matrix<double, U, U>;
  using MatUX = Eigen::Matrix<double, U, X>;

  struct Derivatives {
    VecX dfdx = VecX::Zero();
    VecU dfdu = VecU::Zero();
    MatXX d2fdxdx = MatXX::Zero();
    MatUU d2fdudu = MatUU::Zero();
    MatUX d2fdudx = MatUX::Zero();
  };

  CostFunction() = default;
  virtual ~CostFunction() = default;

  // Return the cost value for a state-action pair at a step.
  virtual double EvaluateCost(
      const VecX& x, const VecU& u,
      std::unordered_map<std::string, double>* nullable_cost_map) const = 0;

  // Return the cost derivatives w.r.t x and u.
  virtual void EvaluateDerivatives(const VecX& x, const VecU& u,
                                   Derivatives* derivatives_ptr) const = 0;
};

}  // namespace ilqr
}  // namespace planning
}  // namespace neodrive