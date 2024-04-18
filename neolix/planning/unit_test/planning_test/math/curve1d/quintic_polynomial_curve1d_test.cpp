#include "src/planning/math/curve1d/quintic_polynomial_curve1d.h"

#include "gtest/gtest.h"
#include "src/planning/math/curve1d/quartic_polynomial_curve1d.h"

namespace neodrive {
namespace planning {

TEST(QuinticPolynomialCurve1dTest, basic_test) {
  double x0 = 0.0;
  double dx0 = 1.0;
  double ddx0 = 0.8;

  double x1 = 10.0;
  double dx1 = 5.0;
  double ddx1 = 0.0;

  double t = 8.0;

  QuinticPolynomialCurve1d curve(x0, dx0, ddx0, x1, dx1, ddx1, t);
  auto e_x0 = curve.Evaluate(0, 0.0);
  auto e_dx0 = curve.Evaluate(1, 0.0);
  auto e_ddx0 = curve.Evaluate(2, 0.0);

  auto e_x1 = curve.Evaluate(0, t);
  auto e_dx1 = curve.Evaluate(1, t);
  auto e_ddx1 = curve.Evaluate(2, t);

  auto e_t = curve.ParamLength();

  EXPECT_NEAR(x0, e_x0, 1.0e-6);
  EXPECT_NEAR(dx0, e_dx0, 1.0e-6);
  EXPECT_NEAR(ddx0, e_ddx0, 1.0e-6);

  EXPECT_NEAR(x1, e_x1, 1.0e-6);
  EXPECT_NEAR(dx1, e_dx1, 1.0e-6);
  EXPECT_NEAR(ddx1, e_ddx1, 1.0e-6);

  EXPECT_NEAR(t, e_t, 1.0e-6);
}

}  // namespace planning
}  // namespace neodrive
