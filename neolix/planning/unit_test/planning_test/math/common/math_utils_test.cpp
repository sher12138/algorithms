#include "common/math/math_utils.h"

#include "gtest/gtest.h"

namespace neodrive {
namespace planning {

TEST(MathUtilsTest, cross_prod) {
  EXPECT_NEAR(cross_prod(Vec2d{0, 0}, Vec2d{0, 1}, Vec2d{1, 0}), -1.0, 1e-5);
  EXPECT_NEAR(cross_prod(Vec2d{0, 0}, Vec2d{1, 0}, Vec2d{0, 1}), 1.0, 1e-5);
  EXPECT_NEAR(cross_prod(Vec2d{0, 1}, Vec2d{0, 0}, Vec2d{1, 0}), 1.0, 1e-5);
  EXPECT_NEAR(cross_prod(Vec2d{1, 2}, Vec2d{3, 4}, Vec2d{5, 6}), 0.0, 1e-5);
  EXPECT_NEAR(cross_prod(Vec2d{1, 2}, Vec2d{3, 4}, Vec2d{6, 5}), -4.0, 1e-5);
  EXPECT_NEAR(cross_prod(Vec2d{2, 2}, Vec2d{7, 5}, Vec2d{3, 4}), 7.0, 1e-5);
}

TEST(MathUtilsTest, inner_prod) {
  EXPECT_NEAR(inner_prod(Vec2d{0, 0}, Vec2d{0, 1}, Vec2d{1, 0}), 0.0, 1e-5);
  EXPECT_NEAR(inner_prod(Vec2d{0, 0}, Vec2d{1, 0}, Vec2d{0, 1}), 0.0, 1e-5);
  EXPECT_NEAR(inner_prod(Vec2d{0, 1}, Vec2d{0, 0}, Vec2d{1, 0}), 1.0, 1e-5);
  EXPECT_NEAR(inner_prod(Vec2d{1, 2}, Vec2d{3, 4}, Vec2d{5, 6}), 16.0, 1e-5);
  EXPECT_NEAR(inner_prod(Vec2d{1, 2}, Vec2d{3, 4}, Vec2d{6, 5}), 16.0, 1e-5);
  EXPECT_NEAR(inner_prod(Vec2d{2, 2}, Vec2d{7, 5}, Vec2d{3, 4}), 11.0, 1e-5);
  EXPECT_NEAR(inner_prod(Vec2d{2, 2}, Vec2d{0, 0}, Vec2d{3, 4}), -6.0, 1e-5);
}

TEST(MathUtilsTest, wrap_angle) {
  EXPECT_NEAR(wrap_angle(-1.2), -1.2 + M_PI * 2.0, 1e-6);
  EXPECT_NEAR(wrap_angle(3.4), 3.4, 1e-6);
  EXPECT_NEAR(wrap_angle(5.6), 5.6, 1e-6);
  EXPECT_NEAR(wrap_angle(7.8), 7.8 - M_PI * 2.0, 1e-6);
  EXPECT_NEAR(wrap_angle(12.4), std::fmod(12.4, M_PI * 2.0), 1e-6);
  EXPECT_NEAR(wrap_angle(-12.4), std::fmod(-12.4, M_PI * 2.0) + M_PI * 2.0,
              1e-6);
}

TEST(MathUtilsTest, normalize_angle) {
  EXPECT_DOUBLE_EQ(1.5, normalize_angle(1.5));
  EXPECT_DOUBLE_EQ(1.5 - M_PI, normalize_angle(1.5 + M_PI));
  EXPECT_DOUBLE_EQ(1.5, normalize_angle(1.5 + M_PI * 2));
  EXPECT_DOUBLE_EQ(1.5, normalize_angle(1.5 - M_PI * 2));
  EXPECT_DOUBLE_EQ(-1.5, normalize_angle(-1.5));
  EXPECT_DOUBLE_EQ(-9.0 + M_PI * 2, normalize_angle(-9.0));
  EXPECT_DOUBLE_EQ(-M_PI, normalize_angle(-M_PI));
  EXPECT_DOUBLE_EQ(-M_PI, normalize_angle(M_PI));
  EXPECT_DOUBLE_EQ(-M_PI, normalize_angle(-M_PI * 3));
  EXPECT_DOUBLE_EQ(-M_PI, normalize_angle(M_PI * 3));
  EXPECT_DOUBLE_EQ(0.0, normalize_angle(M_PI * 4));
}

TEST(MathUtilsTest, sqr) {
  EXPECT_DOUBLE_EQ(121.0, sqr(11.0));
  EXPECT_FLOAT_EQ(144.0f, sqr(-12.0f));
  EXPECT_EQ(169, sqr(-13));
  EXPECT_EQ(2147395600, sqr(46340));
  EXPECT_EQ(-2147479015, sqr(46341));  // Overflow!
}

}  // namespace planning
}  // namespace neodrive
