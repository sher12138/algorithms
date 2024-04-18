#include "src/planning/reference_line/reference_line.h"

namespace neodrive {
namespace planning {

namespace {

constexpr double kErr = 1e-3;

struct UtRefPoint {
  double x{0.};
  double y{0.};
  double heading{0.};
  double s{0.};
};

std::vector<UtRefPoint> data{
    {0., 0., 0., 0.},    {1., 0., 0., 1.},    {2., 0., 0., 2.},
    {3., 0., 0., 3.},    {3., 1., 1.57, 4.},  {3., 2., 1.57, 5.},
    {3., 3., 1.57, 6.},  {2., 3., 3.14, 7.},  {1., 3., 3.14, 8.},
    {1., 4., 1.57, 9.},  {1., 5., 1.57, 10.}, {1., 4., 1.57, 11.},
    {1., 3., 3.14, 12.},
};

std::vector<ReferencePoint> GetRefPoints() {
  std::vector<ReferencePoint> ans{};
  for (auto& [x, y, heading, s] : data) {
    ReferencePoint p{};
    p.set_x(x);
    p.set_y(y);
    p.set_heading(heading);
    p.set_s(s);
    ans.push_back(std::move(p));
  }
  return ans;
};

std::array<std::vector<ReferenceLine::TrafficOverlap>, 9> overlaps{};

}  // namespace

TEST(ReferenceLine, Constructor) {
  auto ref_line =
      ReferenceLine().CreateFrom(GetRefPoints(), overlaps, {0., 0.}, 0., 1);
}

TEST(ReferenceLine, NearestPoint) {
  auto ref_line =
      ReferenceLine().CreateFrom(GetRefPoints(), overlaps, {0., 0.}, 0., 1);
  ReferencePoint ref_pt{};

  EXPECT_TRUE(ref_line->GetNearestRefPoint({0., 0.}, &ref_pt));
  EXPECT_NEAR(ref_pt.x(), 0., kErr);
  EXPECT_NEAR(ref_pt.y(), 0., kErr);
  EXPECT_NEAR(ref_pt.s(), 0., kErr);

  EXPECT_TRUE(ref_line->GetNearestRefPoint({4., 0.}, &ref_pt));
  EXPECT_NEAR(ref_pt.x(), 3., kErr);
  EXPECT_NEAR(ref_pt.y(), 0., kErr);
  EXPECT_NEAR(ref_pt.s(), 3., kErr);

  EXPECT_TRUE(ref_line->GetNearestRefPoint({1., 1000.}, &ref_pt));
  EXPECT_NEAR(ref_pt.x(), 1., kErr);
  EXPECT_NEAR(ref_pt.y(), 5., kErr);
  EXPECT_NEAR(ref_pt.s(), 10., kErr);
}

TEST(ReferenceLine, NearestPointWithHeading) {
  auto ref_line =
      ReferenceLine().CreateFrom(GetRefPoints(), overlaps, {0., 0.}, 0., 1);
  ReferencePoint ref_pt{};

  EXPECT_TRUE(
      ref_line->GetNearestRefPointWithHeading({2., -1.}, 0.45, &ref_pt));
  EXPECT_NEAR(ref_pt.x(), 2., kErr);
  EXPECT_NEAR(ref_pt.y(), 0., kErr);
  EXPECT_NEAR(ref_pt.s(), 2., kErr);

  EXPECT_TRUE(
      ref_line->GetNearestRefPointWithHeading({-1., 0.}, 0.55, &ref_pt));
}

TEST(ReferenceLine, NearestPointByS) {
  auto ref_line =
      ReferenceLine().CreateFrom(GetRefPoints(), overlaps, {0., 0.}, 0., 1);
  ReferencePoint ref_pt{};

  EXPECT_TRUE(ref_line->GetNearestRefPoint(7.4, &ref_pt));
  EXPECT_NEAR(ref_pt.x(), 2., kErr);
  EXPECT_NEAR(ref_pt.y(), 3., kErr);
  EXPECT_NEAR(ref_pt.s(), 7., kErr);

  EXPECT_TRUE(ref_line->GetNearestRefPoint(7.6, &ref_pt));
  EXPECT_NEAR(ref_pt.x(), 1., kErr);
  EXPECT_NEAR(ref_pt.y(), 3., kErr);
  EXPECT_NEAR(ref_pt.s(), 8., kErr);

  EXPECT_TRUE(ref_line->GetNearestRefPoint(-1., &ref_pt));
}

TEST(ReferenceLine, GetPointInFrenetFrame) {
  auto ref_line =
      ReferenceLine().CreateFrom(GetRefPoints(), overlaps, {0., 0.}, 0., 1);
  SLPoint sl_pt{};

  EXPECT_TRUE(ref_line->GetPointInFrenetFrame({1, 1}, &sl_pt));
  EXPECT_NEAR(sl_pt.s(), 1., kErr);
  EXPECT_NEAR(sl_pt.l(), 1., kErr);

  EXPECT_TRUE(ref_line->GetPointInFrenetFrame({4, 1}, &sl_pt));
  EXPECT_NEAR(sl_pt.s(), 4., kErr);
  EXPECT_NEAR(sl_pt.l(), -1., kErr);

  EXPECT_TRUE(ref_line->GetPointInFrenetFrame({4, 0}, &sl_pt));
  EXPECT_NEAR(sl_pt.s(), 3., kErr);
  EXPECT_NEAR(sl_pt.l(), -1., kErr);
}

TEST(ReferenceLine, GetPointInFrenetFrameWithHeading) {
  auto ref_line =
      ReferenceLine().CreateFrom(GetRefPoints(), overlaps, {0., 0.}, 0., 1);
  SLPoint sl_pt{};

  EXPECT_TRUE(ref_line->GetPointInFrenetFrameWithHeading({1, 1}, 0.49, &sl_pt));
  EXPECT_NEAR(sl_pt.s(), 1., kErr);
  EXPECT_NEAR(sl_pt.l(), 1., kErr);

  EXPECT_TRUE(ref_line->GetPointInFrenetFrameWithHeading({4, 1}, 1., &sl_pt));
}

TEST(ReferenceLine, GetPointInCartesianFrame) {
  auto ref_line =
      ReferenceLine().CreateFrom(GetRefPoints(), overlaps, {0., 0.}, 0., 1);
  Vec2d xy_pt{};

  EXPECT_TRUE(ref_line->GetPointInCartesianFrame({1.2, 1.}, &xy_pt));
  EXPECT_NEAR(xy_pt.x(), 1.2, kErr);
  EXPECT_NEAR(xy_pt.y(), 1., kErr);

  EXPECT_TRUE(ref_line->GetPointInCartesianFrame({4.4, -1.}, &xy_pt));
  EXPECT_NEAR(xy_pt.x(), 4., kErr);
  EXPECT_NEAR(xy_pt.y(), 1.4, kErr);

  EXPECT_TRUE(ref_line->GetPointInCartesianFrame({7., -1.}, &xy_pt));
  EXPECT_NEAR(xy_pt.x(), 2, kErr);
  EXPECT_NEAR(xy_pt.y(), 4., kErr);

  EXPECT_TRUE(ref_line->GetPointInCartesianFrame({1000., 1.}, &xy_pt));
}

TEST(ReferenceLine, GetLength) {
  auto ref_line =
      ReferenceLine().CreateFrom(GetRefPoints(), overlaps, {0., 0.}, 0., 1);
  EXPECT_NEAR(ref_line->GetLength(), data.back().s - data.front().s, kErr);
}

TEST(ReferenceLine, AnchorPoint) {
  {
    auto ref_line =
        ReferenceLine().CreateFrom(GetRefPoints(), overlaps, {0., 0.}, 0., 1);
    EXPECT_NEAR(ref_line->anchor_s(), 0.0, kErr);
  }

  {
    auto ref_line =
        ReferenceLine().CreateFrom(GetRefPoints(), overlaps, {0.4, 0.}, 0., 1);
    EXPECT_NEAR(ref_line->anchor_s(), 0., kErr);
  }

  {
    auto ref_line =
        ReferenceLine().CreateFrom(GetRefPoints(), overlaps, {0.9, 0.}, 0., 1);
    EXPECT_NEAR(ref_line->anchor_s(), 1., kErr);
  }

  {
    auto ref_line =
        ReferenceLine().CreateFrom(GetRefPoints(), overlaps, {1, 3}, 8., 1);
    SLPoint sl_pt{};
    EXPECT_TRUE(ref_line->GetPointInFrenetFrame({0, 4}, &sl_pt));
    EXPECT_NEAR(sl_pt.s(), 9., kErr);
    EXPECT_NEAR(sl_pt.l(), 1., kErr);
  }

  {
    auto ref_line =
        ReferenceLine().CreateFrom(GetRefPoints(), overlaps, {1, 3}, 12., 1);
    SLPoint sl_pt{};
    EXPECT_TRUE(ref_line->GetPointInFrenetFrame({0, 4}, &sl_pt));
    EXPECT_NEAR(sl_pt.s(), 11., kErr);
    EXPECT_NEAR(sl_pt.l(), -1., kErr);
  }
}

}  // namespace planning
}  // namespace neodrive
