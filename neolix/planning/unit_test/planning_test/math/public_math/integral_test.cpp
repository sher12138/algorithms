#include "common/math/integral.h"

#include <cmath>

#include "gtest/gtest.h"

namespace neodrive {
namespace planning {

namespace {

double LinearFunc(double x) { return 2.0 * x; }

double SquareFunc(double x) { return x * x; }

double CubicFunc(double x) { return x * x * x; }

double SinFunc(double x) { return std::sin(x); }

}  // namespace

TEST(IntegralTest, Integration) {
  double linear_integral = IntegrateByGaussLegendre<5>(LinearFunc, 0.0, 1.0);
  EXPECT_NEAR(linear_integral, 1.0, 1e-5);
  double square_integral = IntegrateByGaussLegendre<5>(SquareFunc, 0.0, 1.0);
  EXPECT_NEAR(square_integral, 1.0 / 3.0, 1e-5);
  double cubic_integral = IntegrateByGaussLegendre<5>(CubicFunc, 0.0, 1.0);
  EXPECT_NEAR(cubic_integral, 1.0 / 4.0, 1e-5);
  double sin_integral = IntegrateByGaussLegendre<5>(SinFunc, 0.0, 0.5 * M_PI);
  EXPECT_NEAR(sin_integral, 1.0, 1e-5);
}

}  // namespace planning
}  // namespace neodrive
