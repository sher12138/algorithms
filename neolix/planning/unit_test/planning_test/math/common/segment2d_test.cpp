#include "common/math/segment2d.h"

#include <cmath>

#include "gtest/gtest.h"

namespace neodrive {
namespace planning {

TEST(Segment2dTest, Accessors) {
  const Segment2d ls({1, 2}, {5, 4});
  EXPECT_NEAR(ls.length(), std::sqrt(20.0), 1e-5);
  EXPECT_NEAR(ls.length_sqr(), 20.0, 1e-5);
  EXPECT_NEAR(ls.center().x(), 3, 1e-5);
  EXPECT_NEAR(ls.center().y(), 3, 1e-5);
  EXPECT_NEAR(ls.heading(), std::atan2(2, 4), 1e-5);
  EXPECT_NEAR(ls.cos_heading(), 4.0 / std::sqrt(20.0), 1e-5);
  EXPECT_NEAR(ls.sin_heading(), 2.0 / std::sqrt(20.0), 1e-5);
  EXPECT_EQ(ls.debug_string(),
            "segment2d (vec2d ( x = 1  y = 2 ), vec2d ( x = 5  y = 4 ))");
}

TEST(Segment2dTest, distance_to) {
  const Segment2d ls({1, 2}, {5, 4});
  Vec2d nearest_pt;
  EXPECT_NEAR(ls.distance_to({0, 0}, &nearest_pt), std::sqrt(5.0), 1e-5);
  EXPECT_NEAR(nearest_pt.distance_to({0, 0}), std::sqrt(5.0), 1e-5);
  EXPECT_NEAR(ls.distance_to({0, 0}), std::sqrt(5.0), 1e-5);
  EXPECT_NEAR(ls.distance_sqr_to({0, 0}), 5.0, 1e-5);
  EXPECT_NEAR(ls.distance_sqr_to({0, 0}, &nearest_pt), 5.0, 1e-5);
  EXPECT_NEAR(ls.distance_to({10, 10}, &nearest_pt), std::sqrt(61.0), 1e-5);
  EXPECT_NEAR(nearest_pt.distance_to({10, 10}), std::sqrt(61.0), 1e-5);
  EXPECT_NEAR(ls.distance_to({1, 2}, &nearest_pt), 0, 1e-5);
  EXPECT_NEAR(nearest_pt.distance_to({1, 2}), 0, 1e-5);
  EXPECT_NEAR(ls.distance_to({5, 4}, &nearest_pt), 0, 1e-5);
  EXPECT_NEAR(nearest_pt.distance_to({5, 4}), 0, 1e-5);
  EXPECT_NEAR(ls.distance_to({3, 3}, &nearest_pt), 0, 1e-5);
  EXPECT_NEAR(nearest_pt.distance_to({3, 3}), 0, 1e-5);
  EXPECT_NEAR(ls.distance_to({4, 4}, &nearest_pt), 2.0 / std::sqrt(20.0), 1e-5);
  EXPECT_NEAR(nearest_pt.distance_to({4, 4}), 2.0 / std::sqrt(20.0), 1e-5);
}

TEST(Segment2dTest, get_perpendicular_foot) {
  const Segment2d ls({1, 2}, {5, 4});
  Vec2d foot_pt;
  EXPECT_NEAR(ls.get_perpendicular_foot({0, 0}, &foot_pt), 0.6 * std::sqrt(5.0),
              1e-5);
  EXPECT_NEAR(foot_pt.x(), -0.6, 1e-5);
  EXPECT_NEAR(foot_pt.y(), 1.2, 1e-5);
  EXPECT_NEAR(ls.get_perpendicular_foot({3, 3}, &foot_pt), 0.0, 1e-5);
  EXPECT_NEAR(foot_pt.x(), 3.0, 1e-5);
  EXPECT_NEAR(foot_pt.y(), 3.0, 1e-5);
}

TEST(Segment2dTest, project_onto_unit) {
  const Segment2d ls({1, 2}, {5, 4});
  EXPECT_NEAR(ls.project_onto_unit({1, 2}), 0.0, 1e-5);
  EXPECT_NEAR(ls.project_onto_unit({5, 4}), std::sqrt(20.0), 1e-5);
  EXPECT_NEAR(ls.project_onto_unit({-2, -3}), -22.0 / std::sqrt(20.0), 1e-5);
  EXPECT_NEAR(ls.project_onto_unit({6, 7}), 30.0 / std::sqrt(20.0), 1e-5);
}

TEST(Segment2dTest, get_intersect) {
  const Segment2d ls({1, 2}, {5, 4});
  Vec2d point;
  EXPECT_FALSE(ls.get_intersect({{1, 3}, {5, 5}}, &point));
  EXPECT_FALSE(ls.get_intersect({{2, 2}, {6, 4}}, &point));

  EXPECT_TRUE(ls.get_intersect({{1, 2}, {-3, 0}}, &point));
  EXPECT_NEAR(point.x(), 1, 1e-5);
  EXPECT_NEAR(point.y(), 2, 1e-5);
  EXPECT_TRUE(ls.get_intersect({{5, 4}, {9, 6}}, &point));
  EXPECT_NEAR(point.x(), 5, 1e-5);
  EXPECT_NEAR(point.y(), 4, 1e-5);

  EXPECT_TRUE(ls.get_intersect({{3, 0}, {3, 10}}, &point));
  EXPECT_NEAR(point.x(), 3, 1e-5);
  EXPECT_NEAR(point.y(), 3, 1e-5);
  EXPECT_TRUE(ls.get_intersect({{3, 10}, {3, 0}}, &point));
  EXPECT_NEAR(point.x(), 3, 1e-5);
  EXPECT_NEAR(point.y(), 3, 1e-5);
  EXPECT_FALSE(ls.get_intersect({{3, 5}, {3, 10}}, &point));
  EXPECT_FALSE(ls.get_intersect({{3, 2}, {3, 0}}, &point));
  EXPECT_TRUE(ls.get_intersect({{3, 3}, {3, 3}}, &point));
  EXPECT_NEAR(point.x(), 3, 1e-5);
  EXPECT_NEAR(point.y(), 3, 1e-5);
  EXPECT_FALSE(ls.get_intersect({{4, 4}, {4, 4}}, &point));
}

TEST(Segment2dTest, is_point_in) {
  const Segment2d ls({1, 2}, {5, 4});
  EXPECT_TRUE(ls.is_point_in({1, 2}));
  EXPECT_TRUE(ls.is_point_in({5, 4}));
  EXPECT_TRUE(ls.is_point_in({3, 3}));
  EXPECT_FALSE(ls.is_point_in({-1, 1}));
  EXPECT_FALSE(ls.is_point_in({7, 5}));
  EXPECT_FALSE(ls.is_point_in({0, 0}));
  EXPECT_FALSE(ls.is_point_in({6, 6}));
}

}  // namespace planning
}  // namespace neodrive
