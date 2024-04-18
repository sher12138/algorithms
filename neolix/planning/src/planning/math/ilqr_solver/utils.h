#pragma once

#include <Eigen/Dense>
#include <string>
#include <unordered_map>

#include "cost_function.h"
#include "dynamic_model.h"

namespace neodrive {
namespace planning {
namespace ilqr {

template <int X, int U>
class QuadCost : public CostFunction<X, U> {
 public:
  using VecX = Eigen::Matrix<double, X, 1>;
  using VecU = Eigen::Matrix<double, U, 1>;
  using MatXX = Eigen::Matrix<double, X, X>;
  using MatUU = Eigen::Matrix<double, U, U>;
  using MatUX = Eigen::Matrix<double, U, X>;

  QuadCost() = default;
  explicit QuadCost(const VecX& x_weight, const VecU& u_weight,
                    const VecX& x_target)
      : CostFunction<X, U>(),
        x_weight_(MatXX(x_weight.asDiagonal())),
        u_weight_(MatUU(u_weight.asDiagonal())),
        x_target_(x_target) {}

 protected:
  double EvaluateCost(const VecX& x, const VecU& u,
                      std::unordered_map<std::string, double>*
                          nullable_cost_map) const override {
    VecX error = x - x_target_;
    double cost =
        (error.transpose() * x_weight_ * error + u.transpose() * u_weight_ * u)
            .value();
    if (nullable_cost_map != nullptr) {
      if (nullable_cost_map->find(name_) == nullable_cost_map->end()) {
        nullable_cost_map->insert({name_, cost});
      } else {
        nullable_cost_map->at(name_) += cost;
      }
    }
    return cost;
  }

  void EvaluateDerivatives(
      const VecX& x, const VecU& u,
      typename CostFunction<X, U>::Derivatives* derivatives) const override {
    VecX error = x - x_target_;
    derivatives->dfdx = 2.0 * x_weight_ * error;
    derivatives->dfdu = 2.0 * u_weight_ * u;
    derivatives->d2fdxdx = 2.0 * x_weight_;
    derivatives->d2fdudu = 2.0 * u_weight_;
    derivatives->d2fdudx = MatUX::Zero();
  }

 private:
  MatXX x_weight_ = MatXX::Zero();
  MatUU u_weight_ = MatUU::Zero();
  VecX x_target_ = VecX::Zero();
  std::string name_ = "QuadCost";
};

class FirstOrderModel : public DynamicModel<1, 1> {
 public:
  explicit FirstOrderModel(double step_size)
      : DynamicModel(), step_size_(step_size) {}

 protected:
  VecX Evaluate(const VecX& x, const VecU& u) const override {
    return VecX(x(0) + step_size_ * u(0));
  }

  Jacobians EvaluateJacobians(const VecX& x, const VecU& u) const override {
    Jacobians jacobians;
    jacobians.dfdx = MatXX(1.0);
    jacobians.dfdu = MatXU(step_size_);
    return jacobians;
  }

 private:
  double step_size_ = 0.0;
};

class SecondOrderModel : public DynamicModel<2, 1> {
 public:
  explicit SecondOrderModel(double step_size)
      : DynamicModel(), step_size_(step_size) {}

 protected:
  // x = [s, v]^T, u = [a]
  VecX Evaluate(const VecX& x, const VecU& u) const override {
    VecX res;
    const double s =
        x(0) + x(1) * step_size_ + 0.5 * u(0) * step_size_ * step_size_;
    const double v = x(1) + step_size_ * u(0);
    res << s, v;
    return res;
  }

  Jacobians EvaluateJacobians(const VecX& x, const VecU& u) const override {
    Jacobians jacobians;
    jacobians.dfdx << 1.0, step_size_, 0.0, 1.0;
    jacobians.dfdu << 0.5 * step_size_ * step_size_, step_size_;
    return jacobians;
  }

 private:
  double step_size_ = 0.0;
};

}  // namespace ilqr
}  // namespace planning
}  // namespace neodrive
