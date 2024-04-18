#include "common/math/vec3d.h"

#include <cmath>

#include "gtest/gtest.h"

namespace neodrive {
namespace planning {

constexpr double err = 1e-5;

TEST(Vec3dTest, Getters) {
  Vec3d p{1, 2, 3};

  Vec3d pt_copy1(p);
  EXPECT_NEAR(pt_copy1.x(), p.x(), 1e-5);
  EXPECT_NEAR(pt_copy1.y(), p.y(), 1e-5);
  EXPECT_NEAR(pt_copy1.z(), p.z(), 1e-5);

  Vec3d pt_copy2;
  pt_copy2 = p;
  EXPECT_NEAR(pt_copy2.x(), p.x(), 1e-5);
  EXPECT_NEAR(pt_copy2.y(), p.y(), 1e-5);
  EXPECT_NEAR(pt_copy2.z(), p.z(), 1e-5);

  EXPECT_NEAR(p.x(), 1, err);
  EXPECT_NEAR(p.y(), 2, err);
  EXPECT_NEAR(p.z(), 3, err);
  EXPECT_NEAR(p.length(), std::sqrt(1 * 1 + 2 * 2 + 3 * 3), err);
}

TEST(Vec3dTest, Operators) {
  Vec3d v0{1, 2, 3};
  Vec3d v1{2, 3, 4};

  EXPECT_NEAR((v0 + v1).x(), 3, err);
  EXPECT_NEAR((v0 + v1).y(), 5, err);
  EXPECT_NEAR((v0 + v1).z(), 7, err);

  EXPECT_NEAR((v0 - v1).x(), -1, err);
  EXPECT_NEAR((v0 - v1).y(), -1, err);
  EXPECT_NEAR((v0 - v1).z(), -1, err);

  EXPECT_NEAR((v0 * -1).x(), -1, err);
  EXPECT_NEAR((v0 * -1).y(), -2, err);
  EXPECT_NEAR((v0 * -1).z(), -3, err);

  EXPECT_NEAR((v0 / 10).x(), .1, err);
  EXPECT_NEAR((v0 / 10).y(), .2, err);
  EXPECT_NEAR((v0 / 10).z(), .3, err);

  v0 += {1, 1, 1};
  EXPECT_NEAR(v0.x(), 2, err);
  EXPECT_NEAR(v0.y(), 3, err);
  EXPECT_NEAR(v0.z(), 4, err);

  v0 -= {2, 2, 2};
  EXPECT_NEAR(v0.x(), 0, err);
  EXPECT_NEAR(v0.y(), 1, err);
  EXPECT_NEAR(v0.z(), 2, err);

  v0 *= 2;
  EXPECT_NEAR(v0.x(), 0, err);
  EXPECT_NEAR(v0.y(), 2, err);
  EXPECT_NEAR(v0.z(), 4, err);

  v0 /= 4;
  EXPECT_NEAR(v0.x(), 0, err);
  EXPECT_NEAR(v0.y(), .5, err);
  EXPECT_NEAR(v0.z(), 1, err);
}

}  // namespace planning
}  // namespace neodrive
