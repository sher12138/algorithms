#include "common/math/aabox3d.h"

#include <cmath>

#include "gtest/gtest.h"

namespace neodrive {
namespace planning {

namespace {

constexpr double err = 1e-5;

}  // namespace

TEST(AABox3dTest, Constructors) {
  AABox3d b{{0, 0, 0}, 2, 2, 2};
  EXPECT_NEAR(b.center().x(), 0, err);
  EXPECT_NEAR(b.center().y(), 0, err);
  EXPECT_NEAR(b.center().z(), 0, err);

  EXPECT_NEAR(b.length(), 2, err);
  EXPECT_NEAR(b.width(), 2, err);
  EXPECT_NEAR(b.height(), 2, err);

  EXPECT_NEAR(b.half_length(), 1, err);
  EXPECT_NEAR(b.half_width(), 1, err);
  EXPECT_NEAR(b.half_height(), 1, err);

  b = AABox3d{{-1, -1, -1}, {1, 1, 1}};
  EXPECT_NEAR(b.center().x(), 0, err);
  EXPECT_NEAR(b.center().y(), 0, err);
  EXPECT_NEAR(b.center().z(), 0, err);

  EXPECT_NEAR(b.length(), 2, err);
  EXPECT_NEAR(b.width(), 2, err);
  EXPECT_NEAR(b.height(), 2, err);

  EXPECT_NEAR(b.half_length(), 1, err);
  EXPECT_NEAR(b.half_width(), 1, err);
  EXPECT_NEAR(b.half_height(), 1, err);

  b = AABox3d{{{-1, 1, 0}, {1, 0, 1}, {0, 1, 0}, {1, -1, 1}, {0, 0, -1}}};
  EXPECT_NEAR(b.center().x(), 0, err);
  EXPECT_NEAR(b.center().y(), 0, err);
  EXPECT_NEAR(b.center().z(), 0, err);

  EXPECT_NEAR(b.length(), 2, err);
  EXPECT_NEAR(b.width(), 2, err);
  EXPECT_NEAR(b.height(), 2, err);

  EXPECT_NEAR(b.half_length(), 1, err);
  EXPECT_NEAR(b.half_width(), 1, err);
  EXPECT_NEAR(b.half_height(), 1, err);
}


TEST(AABox3dTest, Params) {
  AABox3d b{{0, 0, 0}, 4, 4, 4};

  EXPECT_NEAR(b.volume(), 64, err);

  EXPECT_NEAR(b.min_x(), -2, err);
  EXPECT_NEAR(b.min_y(), -2, err);
  EXPECT_NEAR(b.min_z(), -2, err);
  EXPECT_NEAR(b.max_x(), 2, err);
  EXPECT_NEAR(b.max_y(), 2, err);
  EXPECT_NEAR(b.max_z(), 2, err);
}

TEST(AABox3dTest, get_all_corners) {
  AABox3d box1({0, 0, 0}, 2, 2, 2);
  std::vector<Vec3d> corners1;
  box1.get_all_corners(&corners1);

  EXPECT_NEAR(corners1[0].x(), 1.0, err);
  EXPECT_NEAR(corners1[0].y(), -1.0, err);
  EXPECT_NEAR(corners1[0].z(), -1.0, err);

  EXPECT_NEAR(corners1[1].x(), 1.0, err);
  EXPECT_NEAR(corners1[1].y(), 1.0, err);
  EXPECT_NEAR(corners1[1].z(), -1.0, err);

  EXPECT_NEAR(corners1[2].x(), -1.0, err);
  EXPECT_NEAR(corners1[2].y(), 1.0, err);
  EXPECT_NEAR(corners1[2].z(), -1.0, err);

  EXPECT_NEAR(corners1[3].x(), -1.0, err);
  EXPECT_NEAR(corners1[3].y(), -1.0, err);
  EXPECT_NEAR(corners1[3].z(), -1.0, err);

  EXPECT_NEAR(corners1[4].x(), 1.0, err);
  EXPECT_NEAR(corners1[4].y(), -1.0, err);
  EXPECT_NEAR(corners1[4].z(), 1.0, err);

  EXPECT_NEAR(corners1[5].x(), 1.0, err);
  EXPECT_NEAR(corners1[5].y(), 1.0, err);
  EXPECT_NEAR(corners1[5].z(), 1.0, err);

  EXPECT_NEAR(corners1[6].x(), -1.0, err);
  EXPECT_NEAR(corners1[6].y(), 1.0, err);
  EXPECT_NEAR(corners1[6].z(), 1.0, err);

  EXPECT_NEAR(corners1[7].x(), -1.0, err);
  EXPECT_NEAR(corners1[7].y(), -1.0, err);
  EXPECT_NEAR(corners1[7].z(), 1.0, err);
}

TEST(AABox3dTest, is_point_in) {
  AABox3d b{{0, 0, 0}, 2, 2, 2};
  EXPECT_TRUE(b.is_point_in({0, 0.5, 0.7}));
  EXPECT_TRUE(b.is_point_in({0.999, 0.999, 0.999}));
  EXPECT_FALSE(b.is_point_in({0, 1.5, 0.7}));
  EXPECT_FALSE(b.is_point_in({1.0001, 1.0001, 1.0001}));
}

TEST(AABox3dTest, distance_to) {
  AABox3d b{{0, 0, 0}, 2, 2, 2};

  EXPECT_NEAR(b.distance_to({0, 0, 0}), 0, err);
  EXPECT_NEAR(b.distance_to({1, 0, 0}), 0, err);
  EXPECT_NEAR(b.distance_to({2, 0, 0}), 1, err);
  EXPECT_NEAR(b.distance_to({2, 2, 0}), std::sqrt(2), err);
  EXPECT_NEAR(b.distance_to({0, 2, 2}), std::sqrt(2), err);
  EXPECT_NEAR(b.distance_to({2, 0, 2}), std::sqrt(2), err);

  EXPECT_NEAR(b.distance_to({{2, 0, 0}, 2, 2, 2}), 0, err);
  EXPECT_NEAR(b.distance_to({{3, 0, 0}, 2, 2, 2}), 1, err);
  EXPECT_NEAR(b.distance_to({{3, 3, 0}, 2, 2, 2}), std::sqrt(2), err);
  EXPECT_NEAR(b.distance_to({{0, 3, 3}, 2, 2, 2}), std::sqrt(2), err);
  EXPECT_NEAR(b.distance_to({{3, 0, 3}, 2, 2, 2}), std::sqrt(2), err);
}

TEST(AABox3dTest, has_overlap) {
  AABox3d b{{0, 0, 0}, 2, 2, 2};

  EXPECT_TRUE(b.has_overlap({{2, 0, 0}, 2.1, 2.1, 2.1}));
  EXPECT_TRUE(b.has_overlap({{0, 2, 0}, 2.1, 2.1, 2.1}));
  EXPECT_TRUE(b.has_overlap({{0, 0, 2}, 2.1, 2.1, 2.1}));
  EXPECT_FALSE(b.has_overlap({{3, 0, 0}, 2, 2, 2}));
  EXPECT_FALSE(b.has_overlap({{3, 3, 0}, 2, 2, 2}));
  EXPECT_FALSE(b.has_overlap({{0, 3, 3}, 2, 2, 2}));
  EXPECT_FALSE(b.has_overlap({{3, 0, 3}, 2, 2, 2}));
}

TEST(AABox3dTest, shift) {
  AABox3d b{{1, 2, 3}, 2, 2, 2};
  b.shift({-1, -2, -3});

  EXPECT_NEAR(b.center().x(), 0, err);
  EXPECT_NEAR(b.center().y(), 0, err);
  EXPECT_NEAR(b.center().z(), 0, err);

  EXPECT_NEAR(b.length(), 2, err);
  EXPECT_NEAR(b.width(), 2, err);
  EXPECT_NEAR(b.height(), 2, err);

  EXPECT_NEAR(b.half_length(), 1, err);
  EXPECT_NEAR(b.half_width(), 1, err);
  EXPECT_NEAR(b.half_height(), 1, err);
}

TEST(AABox3dTest, merge_from_box) {
  AABox3d b{{0, 0, 0}, 1, 1, 1};

  b.merge_from({{0, 0, 0}, 2, 2, 2});
  EXPECT_NEAR(b.center().x(), 0, err);
  EXPECT_NEAR(b.center().y(), 0, err);
  EXPECT_NEAR(b.center().z(), 0, err);

  EXPECT_NEAR(b.length(), 2, err);
  EXPECT_NEAR(b.width(), 2, err);
  EXPECT_NEAR(b.height(), 2, err);

  EXPECT_NEAR(b.half_length(), 1, err);
  EXPECT_NEAR(b.half_width(), 1, err);
  EXPECT_NEAR(b.half_height(), 1, err);

  b.merge_from({{2, 0, 0}, 2, 2, 2});
  EXPECT_NEAR(b.center().x(), 1, err);
  EXPECT_NEAR(b.center().y(), 0, err);
  EXPECT_NEAR(b.center().z(), 0, err);

  EXPECT_NEAR(b.length(), 4, err);
  EXPECT_NEAR(b.width(), 2, err);
  EXPECT_NEAR(b.height(), 2, err);

  b.merge_from({{0, 0, 2}, 2, 2, 2});
  EXPECT_NEAR(b.center().x(), 1, err);
  EXPECT_NEAR(b.center().y(), 0, err);
  EXPECT_NEAR(b.center().z(), 1, err);

  EXPECT_NEAR(b.length(), 4, err);
  EXPECT_NEAR(b.width(), 2, err);
  EXPECT_NEAR(b.height(), 4, err);
}

TEST(AABox3dTest, merge_from_point) {
  AABox3d b{{0, 0, 0}, 2, 2, 2};
  b.merge_from({0, 1, 0});

  EXPECT_NEAR(b.center().x(), 0, err);
  EXPECT_NEAR(b.center().y(), 0, err);
  EXPECT_NEAR(b.center().z(), 0, err);

  EXPECT_NEAR(b.length(), 2, err);
  EXPECT_NEAR(b.width(), 2, err);
  EXPECT_NEAR(b.height(), 2, err);

  EXPECT_NEAR(b.half_length(), 1, err);
  EXPECT_NEAR(b.half_width(), 1, err);
  EXPECT_NEAR(b.half_height(), 1, err);

  b.merge_from({3, 0, 0});
  EXPECT_NEAR(b.center().x(), 1, err);
  EXPECT_NEAR(b.center().y(), 0, err);
  EXPECT_NEAR(b.center().z(), 0, err);

  EXPECT_NEAR(b.length(), 4, err);
  EXPECT_NEAR(b.width(), 2, err);
  EXPECT_NEAR(b.height(), 2, err);

  b.merge_from({0, 0, 3});
  EXPECT_NEAR(b.center().x(), 1, err);
  EXPECT_NEAR(b.center().y(), 0, err);
  EXPECT_NEAR(b.center().z(), 1, err);

  EXPECT_NEAR(b.length(), 4, err);
  EXPECT_NEAR(b.width(), 2, err);
  EXPECT_NEAR(b.height(), 4, err);
}

}  // namespace planning
}  // namespace neodrive
