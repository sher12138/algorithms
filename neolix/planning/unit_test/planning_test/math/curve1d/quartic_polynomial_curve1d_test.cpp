#include "src/planning/math/curve1d/quartic_polynomial_curve1d.h"

#include "gtest/gtest.h"
#include "src/planning/math/curve1d/cubic_polynomial_curve1d.h"
#include "src/planning/math/curve1d/quintic_polynomial_curve1d.h"

namespace neodrive {
namespace planning {

TEST(QuarticPolynomialCurve1dTest, Evaluate) {
  {
    double x0 = 0.0;
    double dx0 = 0.0;
    double ddx0 = 0.0;
    double dx1 = 10.0;
    double ddx1 = 1.0;
    double param = 8.0;

    QuarticPolynomialCurve1d curve(x0, dx0, ddx0, dx1, ddx1, param);
    EXPECT_NEAR(dx1, curve.Evaluate(1, param), 1e-8);
    EXPECT_NEAR(ddx1, curve.Evaluate(2, param), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(0, 0.0), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(1, 0.0), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(2, 0.0), 1e-8);
  }
  {
    double x0 = 0.0;
    double dx0 = 0.0;
    double ddx0 = 0.0;
    double dx1 = 5.0;
    double ddx1 = 1.0;
    double param = 3.0;

    QuarticPolynomialCurve1d curve(x0, dx0, ddx0, dx1, ddx1, param);
    EXPECT_NEAR(dx1, curve.Evaluate(1, param), 1e-8);
    EXPECT_NEAR(ddx1, curve.Evaluate(2, param), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(0, 0.0), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(1, 0.0), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(2, 0.0), 1e-8);
  }

  {
    double x0 = 1.0;
    double dx0 = 2.0;
    double ddx0 = 3.0;
    double dx1 = 5.0;
    double ddx1 = 1.0;
    double param = 3.0;

    QuarticPolynomialCurve1d curve(x0, dx0, ddx0, dx1, ddx1, param);
    EXPECT_NEAR(dx1, curve.Evaluate(1, param), 1e-8);
    EXPECT_NEAR(ddx1, curve.Evaluate(2, param), 1e-8);
    EXPECT_NEAR(x0, curve.Evaluate(0, 0.0), 1e-8);
    EXPECT_NEAR(dx0, curve.Evaluate(1, 0.0), 1e-8);
    EXPECT_NEAR(ddx0, curve.Evaluate(2, 0.0), 1e-8);
  }
}

}  // namespace planning
}  // namespace neodrive
