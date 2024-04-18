#include "src/planning/math/common/heading.h"

#include <cmath>

#include "gtest/gtest.h"

namespace neodrive {
namespace planning {

TEST(QuaternionTest, quaternion_to_heading) {
  const double v = sqrt(0.5);  // = cos(pi / 4) = sin(pi / 4)
  EXPECT_DOUBLE_EQ(
      0, quaternion_to_heading(v, 0.0, 0.0, -v));  // Pointing to east.
  EXPECT_DOUBLE_EQ(
      M_PI_2, quaternion_to_heading(1.0, 0.0, 0.0, 0.0));  // Pointing to north.
  EXPECT_DOUBLE_EQ(-M_PI_2, quaternion_to_heading(0.0, 0.0, 0.0,
                                                  1.0));  // Pointing to south.
  EXPECT_DOUBLE_EQ(-M_PI,
                   quaternion_to_heading(v, 0.0, 0.0, v));  // Pointing to west.

  Eigen::Quaternionf q(1.0, 0.0, 0.0, 0.0);
  EXPECT_FLOAT_EQ(M_PI_2, quaternion_to_heading(q));  // Pointing to north.

  const double headings[] = {-3.3, -2.2, -1.1, 1.2, 2.3, 3.4, 4.5, 5.6, 6.7};
  for (double heading : headings) {
    EXPECT_NEAR(normalize_angle(heading),
                quaternion_to_heading(heading_to_quaternion<double>(heading)),
                1e-15);
  }
}

}  // namespace planning
}  // namespace neodrive
