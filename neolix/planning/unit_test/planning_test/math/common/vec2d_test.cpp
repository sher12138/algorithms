#include "common/math/vec2d.h"

#include <cmath>

#include "gtest/gtest.h"

namespace neodrive {
namespace planning {

TEST(Vec2dTest, NomralCases) {
  Vec2d pt(2, 3);

  Vec2d pt_copy1(pt);
  EXPECT_NEAR(pt_copy1.x(), pt.x(), 1e-5);
  EXPECT_NEAR(pt_copy1.y(), pt.y(), 1e-5);

  Vec2d pt_copy2;
  pt_copy2 = pt;
  EXPECT_NEAR(pt_copy2.x(), pt.x(), 1e-5);
  EXPECT_NEAR(pt_copy2.y(), pt.y(), 1e-5);

  EXPECT_NEAR(pt.length(), std::sqrt(13.0), 1e-5);
  EXPECT_NEAR(pt.length_sqr(), 13.0, 1e-5);
  EXPECT_NEAR(pt.distance_to({0, 0}), std::sqrt(13.0), 1e-5);
  EXPECT_NEAR(pt.distance_sqr_to({0, 0}), 13.0, 1e-5);
  EXPECT_NEAR(pt.distance_to({0, 2}), std::sqrt(5.0), 1e-5);
  EXPECT_NEAR(pt.distance_sqr_to({0, 2}), 5.0, 1e-5);
  EXPECT_NEAR(pt.angle(), std::atan2(3, 2), 1e-5);
  EXPECT_NEAR(pt.cross_prod({4, 5}), -2, 1e-5);
  EXPECT_NEAR(pt.inner_prod({4, 5}), 23, 1e-5);
  EXPECT_EQ(pt.debug_string(), "vec2d ( x = 2  y = 3 )");
  pt.set_x(4);
  pt.set_y(5);
  EXPECT_NEAR(pt.length(), std::sqrt(41.0), 1e-5);
  EXPECT_NEAR(pt.length_sqr(), 41.0, 1e-5);
  pt.normalize();
  EXPECT_NEAR(pt.x(), 4.0 / std::sqrt(41.0), 1e-5);
  EXPECT_NEAR(pt.y(), 5.0 / std::sqrt(41.0), 1e-5);
  EXPECT_NEAR(pt.length(), 1.0, 1e-5);

  const Vec2d d = Vec2d(0.5, 1.5) + Vec2d(2.5, 3.5);
  EXPECT_NEAR(d.x(), 3.0, 1e-5);
  EXPECT_NEAR(d.y(), 5.0, 1e-5);
  const Vec2d e = Vec2d(0.5, 1.5) - Vec2d(2.5, 3.5);
  EXPECT_NEAR(e.x(), -2.0, 1e-5);
  EXPECT_NEAR(e.y(), -2.0, 1e-5);
  const Vec2d f = d / 2.0;
  EXPECT_NEAR(f.x(), 1.5, 1e-5);
  EXPECT_NEAR(f.y(), 2.5, 1e-5);
  const Vec2d g = e * (-3.0);
  EXPECT_NEAR(g.x(), 6.0, 1e-5);
  EXPECT_NEAR(g.y(), 6.0, 1e-5);

  const Vec2d unit_pt = Vec2d::create_unit_vec(M_PI_4);
  EXPECT_NEAR(unit_pt.x(), std::sqrt(2.0) / 2.0, 1e-5);
  EXPECT_NEAR(unit_pt.y(), std::sqrt(2.0) / 2.0, 1e-5);
  EXPECT_NEAR(unit_pt.angle(), M_PI_4, 1e-5);
}

TEST(Vec2dTest, rotate) {
  Vec2d pt(4, 0);
  auto p1 = pt.rotate(M_PI / 2.0);
  EXPECT_NEAR(p1.x(), 0.0, 1e-5);
  EXPECT_NEAR(p1.y(), 4.0, 1e-5);
  auto p2 = pt.rotate(M_PI);
  EXPECT_NEAR(p2.x(), -4.0, 1e-5);
  EXPECT_NEAR(p2.y(), 0.0, 1e-5);
  auto p3 = pt.rotate(-M_PI / 2.0);
  EXPECT_NEAR(p3.x(), 0.0, 1e-5);
  EXPECT_NEAR(p3.y(), -4.0, 1e-5);
  auto p4 = pt.rotate(-M_PI);
  EXPECT_NEAR(p4.x(), -4.0, 1e-5);
  EXPECT_NEAR(p4.y(), 0.0, 1e-5);
}

TEST(Vec2dTest, self_rotate) {
  Vec2d p1(4, 0);
  p1.self_rotate(M_PI / 2.0);
  EXPECT_NEAR(p1.x(), 0.0, 1e-5);
  EXPECT_NEAR(p1.y(), 4.0, 1e-5);
  Vec2d p2(4, 0);
  p2.self_rotate(M_PI);
  EXPECT_NEAR(p2.x(), -4.0, 1e-5);
  EXPECT_NEAR(p2.y(), 0.0, 1e-5);
  Vec2d p3(4, 0);
  p3.self_rotate(-M_PI / 2.0);
  EXPECT_NEAR(p3.x(), 0.0, 1e-5);
  EXPECT_NEAR(p3.y(), -4.0, 1e-5);
  Vec2d p4(4, 0);
  p4.self_rotate(-M_PI);
  EXPECT_NEAR(p4.x(), -4.0, 1e-5);
  EXPECT_NEAR(p4.y(), 0.0, 1e-5);
}

}  // namespace planning
}  // namespace neodrive
