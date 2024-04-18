#pragma once

#include <array>
#include <string>

#include "polynomial_curve1d.h"

namespace neodrive {
namespace planning {

class CubicPolynomialCurve1d : public PolynomialCurve1d {
 public:
  CubicPolynomialCurve1d() = default;
  virtual ~CubicPolynomialCurve1d() = default;

  CubicPolynomialCurve1d(const std::array<double, 3>& start, const double end,
                         const double param);

  /**
   * x0 is the value when f(x = 0);
   * dx0 is the value when f'(x = 0);
   * ddx0 is the value when f''(x = 0);
   * f(x = param) = x1
   */
  CubicPolynomialCurve1d(const double x0, const double dx0, const double ddx0,
                         const double x1, const double param);

  CubicPolynomialCurve1d& FitWithEndPointFirstOrder(const double x0,
                                                    const double dx0,
                                                    const double x1,
                                                    const double dx1,
                                                    const double param);

  void DerivedFromQuarticCurve(const PolynomialCurve1d& other);

  double Evaluate(const std::uint32_t order, const double p) const override;

  double ParamLength() const override { return param_; }

  double Coef(const size_t order) const override;

  size_t Order() const override { return 3; }

 private:
  void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                           const double x1, const double param);
  std::array<double, 4> coef_ = {{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> start_condition_ = {{0.0, 0.0, 0.0}};
  double end_condition_ = 0.0;
};

}  // namespace planning
}  // namespace neodrive
