#include "cubic_polynomial_curve1d.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

namespace neodrive {
namespace planning {

CubicPolynomialCurve1d::CubicPolynomialCurve1d(
    const std::array<double, 3>& start, const double end, const double param)
    : CubicPolynomialCurve1d(start[0], start[1], start[2], end, param) {}

CubicPolynomialCurve1d::CubicPolynomialCurve1d(const double x0,
                                               const double dx0,
                                               const double ddx0,
                                               const double x1,
                                               const double param) {
  ComputeCoefficients(x0, dx0, ddx0, x1, param);
  param_ = param;
  start_condition_[0] = x0;
  start_condition_[1] = dx0;
  start_condition_[2] = ddx0;
  end_condition_ = x1;
}

void CubicPolynomialCurve1d::DerivedFromQuarticCurve(
    const PolynomialCurve1d& other) {
  param_ = other.ParamLength();
  for (size_t i = 1; i < 5; ++i) {
    coef_[i - 1] = other.Coef(i) * static_cast<double>(i);
  }
}

CubicPolynomialCurve1d& CubicPolynomialCurve1d::FitWithEndPointFirstOrder(
    const double x0, const double dx0, const double x1, const double dx1,
    const double p) {
  param_ = p;

  Eigen::VectorXd b(4);
  b << x0, dx0, x1, dx1;
  Eigen::MatrixXd a(4, 4);
  a << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, p, std::pow(p, 2),
      std::pow(p, 3), 0.0, 1.0, 2.0 * p, 3.0 * std::pow(p, 2);
  Eigen::MatrixXd a_inv = a.inverse();
  Eigen::VectorXd c = a_inv * b;

  coef_[0] = c[0];
  coef_[1] = c[1];
  coef_[2] = c[2];
  coef_[3] = c[3];

  return *this;
}

double CubicPolynomialCurve1d::Evaluate(const std::uint32_t order,
                                        const double p) const {
  switch (order) {
    case 0: {
      return ((coef_[3] * p + coef_[2]) * p + coef_[1]) * p + coef_[0];
    }
    case 1: {
      return (3.0 * coef_[3] * p + 2.0 * coef_[2]) * p + coef_[1];
    }
    case 2: {
      return 6.0 * coef_[3] * p + 2.0 * coef_[2];
    }
    case 3: {
      return 6.0 * coef_[3];
    }
    default:
      return 0.0;
  }
}

void CubicPolynomialCurve1d::ComputeCoefficients(const double x0,
                                                 const double dx0,
                                                 const double ddx0,
                                                 const double x1,
                                                 const double param) {
  const double p2 = param * param;
  const double p3 = param * p2;
  coef_[0] = x0;
  coef_[1] = dx0;
  coef_[2] = 0.5 * ddx0;
  coef_[3] = (x1 - x0 - dx0 * param - coef_[2] * p2) / p3;
}

double CubicPolynomialCurve1d::Coef(const size_t order) const {
  return coef_[order];
}

}  // namespace planning
}  // namespace neodrive
