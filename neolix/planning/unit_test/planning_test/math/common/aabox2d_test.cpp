#include "common/math/aabox2d.h"

#include "gtest/gtest.h"

namespace neodrive {
namespace planning {

TEST(AABox2dTest, get_all_corners) {
  AABox2d box1({0, 0}, 4, 2);
  std::vector<Vec2d> corners1;
  box1.get_all_corners(&corners1);
  EXPECT_NEAR(corners1[0].x(), 2.0, 1e-5);
  EXPECT_NEAR(corners1[0].y(), -1.0, 1e-5);
  EXPECT_NEAR(corners1[1].x(), 2.0, 1e-5);
  EXPECT_NEAR(corners1[1].y(), 1.0, 1e-5);
  EXPECT_NEAR(corners1[2].x(), -2.0, 1e-5);
  EXPECT_NEAR(corners1[2].y(), 1.0, 1e-5);
  EXPECT_NEAR(corners1[3].x(), -2.0, 1e-5);
  EXPECT_NEAR(corners1[3].y(), -1.0, 1e-5);
  EXPECT_EQ(
      box1.debug_string(),
      "aabox2d ( center = vec2d ( x = 0  y = 0 )  length = 4  width = 2 )");
  std::vector<Vec2d> corners2;

  AABox2d box2({3, 1}, {7, 3});
  box2.get_all_corners(&corners2);
  EXPECT_NEAR(corners2[0].x(), 7.0, 1e-5);
  EXPECT_NEAR(corners2[0].y(), 1.0, 1e-5);
  EXPECT_NEAR(corners2[1].x(), 7.0, 1e-5);
  EXPECT_NEAR(corners2[1].y(), 3.0, 1e-5);
  EXPECT_NEAR(corners2[2].x(), 3.0, 1e-5);
  EXPECT_NEAR(corners2[2].y(), 3.0, 1e-5);
  EXPECT_NEAR(corners2[3].x(), 3.0, 1e-5);
  EXPECT_NEAR(corners2[3].y(), 1.0, 1e-5);
  EXPECT_EQ(
      box2.debug_string(),
      "aabox2d ( center = vec2d ( x = 5  y = 2 )  length = 4  width = 2 )");
}

TEST(AABox2dTest, CenterAndLengths) {
  AABox2d box1({0, 0}, 10, 10);
  EXPECT_NEAR(box1.center_x(), 0.0, 1e-5);
  EXPECT_NEAR(box1.center_y(), 0.0, 1e-5);
  EXPECT_NEAR(box1.length(), 10.0, 1e-5);
  EXPECT_NEAR(box1.width(), 10.0, 1e-5);
  EXPECT_NEAR(box1.half_length(), 5.0, 1e-5);
  EXPECT_NEAR(box1.half_width(), 5.0, 1e-5);

  AABox2d box2({{0, 2}, {0, -6}, {3, 0}, {1, 0}});
  EXPECT_NEAR(box2.center_x(), 1.5, 1e-5);
  EXPECT_NEAR(box2.center_y(), -2.0, 1e-5);
  EXPECT_NEAR(box2.length(), 3.0, 1e-5);
  EXPECT_NEAR(box2.width(), 8.0, 1e-5);
  EXPECT_NEAR(box2.half_length(), 1.5, 1e-5);
  EXPECT_NEAR(box2.half_width(), 4.0, 1e-5);
}

TEST(AABox2dTest, has_overlap) {
  AABox2d box1({0, 0}, 4, 2);
  AABox2d box2({3, 1}, {7, 3});
  AABox2d box3({0, 0}, 10, 10);
  EXPECT_FALSE(box1.has_overlap(box2));
  EXPECT_TRUE(box1.has_overlap(box3));
  EXPECT_TRUE(box2.has_overlap(box3));
}

TEST(AABox2dTest, distance_to) {
  AABox2d box({0, 0}, 4, 2);
  EXPECT_NEAR(box.distance_to({3, 0}), 1.0, 1e-5);
  EXPECT_NEAR(box.distance_to({-3, 0}), 1.0, 1e-5);
  EXPECT_NEAR(box.distance_to({0, 2}), 1.0, 1e-5);
  EXPECT_NEAR(box.distance_to({0, -2}), 1.0, 1e-5);
  EXPECT_NEAR(box.distance_to({0, 0}), 0.0, 1e-5);
  EXPECT_NEAR(box.distance_to({0, 1}), 0.0, 1e-5);
  EXPECT_NEAR(box.distance_to({1, 0}), 0.0, 1e-5);
  EXPECT_NEAR(box.distance_to({0, -1}), 0.0, 1e-5);
  EXPECT_NEAR(box.distance_to({-1, 0}), 0.0, 1e-5);
}

TEST(AABox2dTest, is_point_in) {
  AABox2d box({0, 0}, 4, 2);
  EXPECT_TRUE(box.is_point_in({0, 0}));
  EXPECT_TRUE(box.is_point_in({1, 0.5}));
  EXPECT_TRUE(box.is_point_in({-0.5, -1}));
  EXPECT_TRUE(box.is_point_in({2, 1}));
  EXPECT_FALSE(box.is_point_in({-3, 0}));
  EXPECT_FALSE(box.is_point_in({0, 2}));
  EXPECT_FALSE(box.is_point_in({-4, -2}));
}

TEST(AABox2dTest, is_point_on_boundary) {
  AABox2d box({0, 0}, 4, 2);
  EXPECT_FALSE(box.is_point_on_boundary({0, 0}));
  EXPECT_FALSE(box.is_point_on_boundary({1, 0.5}));
  EXPECT_TRUE(box.is_point_on_boundary({-0.5, -1}));
  EXPECT_TRUE(box.is_point_on_boundary({2, 0.5}));
  EXPECT_TRUE(box.is_point_on_boundary({-2, 1}));
  EXPECT_FALSE(box.is_point_on_boundary({-3, 0}));
  EXPECT_FALSE(box.is_point_on_boundary({0, 2}));
  EXPECT_FALSE(box.is_point_on_boundary({-4, -2}));
}

TEST(AABox2dTest, MinMax) {
  AABox2d box1({0, 0}, 4, 2);
  EXPECT_NEAR(box1.min_x(), -2, 1e-5);
  EXPECT_NEAR(box1.max_x(), 2, 1e-5);
  EXPECT_NEAR(box1.min_y(), -1, 1e-5);
  EXPECT_NEAR(box1.max_y(), 1, 1e-5);

  AABox2d box2({3, 1}, {7, 3});
  EXPECT_NEAR(box2.min_x(), 3, 1e-5);
  EXPECT_NEAR(box2.max_x(), 7, 1e-5);
  EXPECT_NEAR(box2.min_y(), 1, 1e-5);
  EXPECT_NEAR(box2.max_y(), 3, 1e-5);
}

TEST(AABox2dTest, shift) {
  AABox2d box({0, 0}, 4, 2);
  box.shift({30, 40});
  std::vector<Vec2d> corners;
  box.get_all_corners(&corners);
  EXPECT_NEAR(corners[0].x(), 32.0, 1e-5);
  EXPECT_NEAR(corners[0].y(), 39.0, 1e-5);
  EXPECT_NEAR(corners[1].x(), 32.0, 1e-5);
  EXPECT_NEAR(corners[1].y(), 41.0, 1e-5);
  EXPECT_NEAR(corners[2].x(), 28.0, 1e-5);
  EXPECT_NEAR(corners[2].y(), 41.0, 1e-5);
  EXPECT_NEAR(corners[3].x(), 28.0, 1e-5);
  EXPECT_NEAR(corners[3].y(), 39.0, 1e-5);
}

}  // namespace planning
}  // namespace neodrive
