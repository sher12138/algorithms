#include "common/math/interpolation.h"

#include "Eigen/Dense"
#include "gtest/gtest.h"

namespace neodrive {
namespace planning {

TEST(InterpolationTest, LerpOneDim) {
  double t0 = 0.0;
  double t1 = 1.0;
  double x0 = 2.0;
  double x1 = 4.0;

  EXPECT_NEAR(Interpolation::lerp(x0, t0, x1, t1, 0.4), 2.8, 1e-6);
  EXPECT_NEAR(Interpolation::lerp(x0, t0, x1, t1, 0.9), 3.8, 1e-6);
  EXPECT_NEAR(Interpolation::lerp(x0, t0, x1, t1, 1.5), 5.0, 1e-6);
  EXPECT_NEAR(Interpolation::lerp(x0, t0, x1, t1, -0.3), 1.4, 1e-6);
}

TEST(InterpolationTest, LerpTwoDim) {
  double t0 = 0.0;
  double t1 = 1.0;

  Eigen::Vector2d x0(2.0, 1.0);
  Eigen::Vector2d x1(4.0, 5.0);

  Eigen::Vector2d x = Interpolation::lerp(x0, t0, x1, t1, 0.4);
  EXPECT_NEAR(x.x(), 2.8, 1e-6);
  EXPECT_NEAR(x.y(), 2.6, 1e-6);

  x = Interpolation::lerp(x0, t0, x1, t1, 1.2);
  EXPECT_NEAR(x.x(), 4.4, 1e-6);
  EXPECT_NEAR(x.y(), 5.8, 1e-6);

  x = Interpolation::lerp(x0, t0, x1, t1, -0.5);
  EXPECT_NEAR(x.x(), 1.0, 1e-6);
  EXPECT_NEAR(x.y(), -1.0, 1e-6);
}

TEST(InterpolationTest, SlerpCaseOne) {
  double t0 = 0.0;
  double t1 = 1.0;
  double a0 = -2.0;
  double a1 = 8.5;

  EXPECT_NEAR(Interpolation::slerp(a0, t0, a1, t1, 0.4), -2.827, 1e-3);
}

TEST(InterpolationTest, SlerpCaseTwo) {
  double t0 = 0.0;
  double t1 = 1.0;
  double a0 = 3.00;
  double a1 = -3.00;

  EXPECT_NEAR(Interpolation::slerp(a0, t0, a1, t1, 0.5001), -3.1416, 1e-3);
}

}  // namespace planning
}  // namespace neodrive
