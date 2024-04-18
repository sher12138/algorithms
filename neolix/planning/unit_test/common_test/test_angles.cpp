#include "common/angles/angles.h"
#include "gtest/gtest.h"

TEST(AnglesTest, DegreesToRadians) {
  double degrees = 90.0;
  double radians = neodrive::common::angles::from_degrees(degrees);
  EXPECT_DOUBLE_EQ(radians, M_PI / 2.0);
}

TEST(AnglesTest, RadiansToDegrees) {
  double radians = M_PI / 4.0;
  double degrees = neodrive::common::angles::to_degrees(radians);
  EXPECT_DOUBLE_EQ(degrees, 45.0);
}

TEST(AnglesTest, NormalizeAnglePositive) {
  double angle = 3.5 * M_PI;
  double normalized = neodrive::common::angles::normalize_angle_positive(angle);
  EXPECT_DOUBLE_EQ(normalized, 1.5 * M_PI);
}

TEST(AnglesTest, NormalizeAngle) {
  double angle = -1.5 * M_PI;
  double normalized = neodrive::common::angles::normalize_angle(angle);
  EXPECT_DOUBLE_EQ(normalized, 0.5 * M_PI);
}

TEST(AnglesTest, ShortestAngularDistance) {
  double from = 0.0;
  double to = 1.5 * M_PI;
  double distance =
      neodrive::common::angles::shortest_angular_distance(from, to);
  EXPECT_DOUBLE_EQ(distance, -0.5 * M_PI);
}

TEST(AnglesTest, TwoPiComplement) {
  double angle = -M_PI / 4.0;
  double complement = neodrive::common::angles::two_pi_complement(angle);
  EXPECT_DOUBLE_EQ(complement, 7.0 * M_PI / 4.0);
}

TEST(AnglesTest, FindMinMaxDelta) {
  double from = -M_PI / 4.0;
  double left_limit = -M_PI / 2.0;
  double right_limit = M_PI / 2.0;
  double min_delta, max_delta;
  bool result = neodrive::common::angles::find_min_max_delta(
      from, left_limit, right_limit, min_delta, max_delta);
  EXPECT_TRUE(result);
  EXPECT_DOUBLE_EQ(min_delta, -M_PI / 4.0);
  EXPECT_DOUBLE_EQ(max_delta, 3 * M_PI / 4.0);
}

TEST(AnglesTest, ShortestAngularDistanceWithLimits) {
  double from = -M_PI / 4.0;
  double to = M_PI / 4.0;
  double left_limit = -M_PI / 2.0;
  double right_limit = M_PI / 2.0;
  double shortest_angle;
  bool result = neodrive::common::angles::shortest_angular_distance_with_limits(
      from, to, left_limit, right_limit, shortest_angle);
  EXPECT_TRUE(result);
  EXPECT_DOUBLE_EQ(shortest_angle, M_PI / 2.0);
}
