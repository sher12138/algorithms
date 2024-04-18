#include "common/math/util.h"

#include <cmath>

#include "gtest/gtest.h"
#include "src/planning/math/public_math/utility.h"

namespace neodrive {
namespace planning {

namespace {

double LinearFunc(double x) { return 2.0 * x; }

double SquareFunc(double x) { return x * x; }

double CubicFunc(double x) { return (x - 1.0) * (x - 2.0) * (x - 3.0); }

double SinFunc(double x) { return std::sin(x); }

}  // namespace

TEST(UtilTest, GoldenSectionSearch) {
  double linear_argmin = GoldenSectionSearch(LinearFunc, 0.0, 1.0, 1e-6);
  EXPECT_NEAR(linear_argmin, 0.0, 1e-5);
  double square_argmin = GoldenSectionSearch(SquareFunc, -1.0, 2.0, 1e-6);
  EXPECT_NEAR(square_argmin, 0.0, 1e-5);
  double cubic_argmin_1 = GoldenSectionSearch(CubicFunc, 0.0, 1.5, 1e-6);
  EXPECT_NEAR(cubic_argmin_1, 0.0, 1e-5);
  double cubic_argmin_2 = GoldenSectionSearch(CubicFunc, 1.0, 1.8, 1e-6);
  EXPECT_NEAR(cubic_argmin_2, 1.0, 1e-5);
  double cubic_argmin_3 = GoldenSectionSearch(CubicFunc, 2.0, 3.0, 1e-6);
  EXPECT_NEAR(cubic_argmin_3, 2.0 + 1.0 / std::sqrt(3.0), 1e-5);
  double sin_argmin = GoldenSectionSearch(SinFunc, 0.0, 2 * M_PI, 1e-6);
  EXPECT_NEAR(sin_argmin, 1.5 * M_PI, 1e-5);
}

TEST(UtilityTest, CalcLeftRightSpace1) {
  const double left_bound = 5.0, right_bound = -5.0;

  Obstacle obs1;

  // test1
  Boundary tmp{10.0, 15.0, 1.0, 3.0};
  std::vector<ObstacleBoundary> obs_valid_boundary;
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp, &obs1});
  std::vector<std::pair<double, double>> prefer{{6.0, 2.0}};
  std::vector<std::pair<double, double>> output;
  bool bflag = Utility::CalcLeftRightSpace(obs_valid_boundary, left_bound,
                                           right_bound, output);
  EXPECT_TRUE(bflag);
  EXPECT_EQ(prefer.size(), output.size());

  for (std::size_t i = 0; i < prefer.size(); ++i) {
    EXPECT_NEAR(prefer[i].first, output[i].first, 0.01);
    EXPECT_NEAR(prefer[i].second, output[i].second, 0.01);
  }
}

TEST(UtilityTest, CalcLeftRightSpace2) {
  const double left_bound = 5.0, right_bound = -5.0;

  Obstacle obs1;

  Boundary tmp{10.0, 15.0, -3.0, -1.0};
  std::vector<ObstacleBoundary> obs_valid_boundary;
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp, &obs1});
  std::vector<std::pair<double, double>> prefer{{2.0, 6.0}};
  std::vector<std::pair<double, double>> output;
  bool bflag = Utility::CalcLeftRightSpace(obs_valid_boundary, left_bound,
                                           right_bound, output);
  EXPECT_TRUE(bflag);
  EXPECT_EQ(prefer.size(), output.size());

  for (std::size_t i = 0; i < prefer.size(); ++i) {
    EXPECT_NEAR(prefer[i].first, output[i].first, 0.01);
    EXPECT_NEAR(prefer[i].second, output[i].second, 0.01);
  }
}

TEST(UtilityTest, CalcLeftRightSpace3) {
  const double left_bound = 5.0, right_bound = -5.0;

  Obstacle obs1;

  Boundary tmp{10.0, 15.0, 3.0, 7.0};
  std::vector<ObstacleBoundary> obs_valid_boundary;
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp, &obs1});
  std::vector<std::pair<double, double>> prefer{{8.0, 0.0}};
  std::vector<std::pair<double, double>> output;
  bool bflag = Utility::CalcLeftRightSpace(obs_valid_boundary, left_bound,
                                           right_bound, output);
  EXPECT_TRUE(bflag);
  EXPECT_EQ(prefer.size(), output.size());

  for (std::size_t i = 0; i < prefer.size(); ++i) {
    EXPECT_NEAR(prefer[i].first, output[i].first, 0.01);
    EXPECT_NEAR(prefer[i].second, output[i].second, 0.01);
  }
}

TEST(UtilityTest, CalcLeftRightSpace4) {
  const double left_bound = 5.0, right_bound = -5.0;

  Obstacle obs1;

  Boundary tmp{10.0, 15.0, -7.0, -3.0};
  std::vector<ObstacleBoundary> obs_valid_boundary;
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp, &obs1});
  std::vector<std::pair<double, double>> prefer{{0.0, 8.0}};
  std::vector<std::pair<double, double>> output;
  bool bflag = Utility::CalcLeftRightSpace(obs_valid_boundary, left_bound,
                                           right_bound, output);
  EXPECT_TRUE(bflag);
  EXPECT_EQ(prefer.size(), output.size());

  for (std::size_t i = 0; i < prefer.size(); ++i) {
    EXPECT_NEAR(prefer[i].first, output[i].first, 0.01);
    EXPECT_NEAR(prefer[i].second, output[i].second, 0.01);
  }
}

TEST(UtilityTest, CalcLeftRightSpace5) {
  const double left_bound = 5.0, right_bound = -5.0;

  Obstacle obs1;

  Boundary tmp{10.0, 15.0, 1.0, 3.0};
  Boundary tmp1{20.0, 25.0, -3.0, -1.0};
  std::vector<ObstacleBoundary> obs_valid_boundary;
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp, &obs1});
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp1, &obs1});
  std::vector<std::pair<double, double>> prefer{{6.0, 2.0}, {2.0, 6.0}};
  std::vector<std::pair<double, double>> output;
  bool bflag = Utility::CalcLeftRightSpace(obs_valid_boundary, left_bound,
                                           right_bound, output);
  EXPECT_TRUE(bflag);
  EXPECT_EQ(prefer.size(), output.size());

  for (std::size_t i = 0; i < prefer.size(); ++i) {
    EXPECT_NEAR(prefer[i].first, output[i].first, 0.01);
    EXPECT_NEAR(prefer[i].second, output[i].second, 0.01);
  }
}

TEST(UtilityTest, CalcLeftRightSpace6) {
  const double left_bound = 5.0, right_bound = -5.0;

  Obstacle obs1;

  Boundary tmp{10.0, 15.0, -3.0, -1.0};
  Boundary tmp1{20.0, 25.0, 1.0, 3.0};
  std::vector<ObstacleBoundary> obs_valid_boundary;
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp, &obs1});
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp1, &obs1});
  std::vector<std::pair<double, double>> prefer{{2.0, 6.0}, {6.0, 2.0}};
  std::vector<std::pair<double, double>> output;
  bool bflag = Utility::CalcLeftRightSpace(obs_valid_boundary, left_bound,
                                           right_bound, output);
  EXPECT_TRUE(bflag);
  EXPECT_EQ(prefer.size(), output.size());

  for (std::size_t i = 0; i < prefer.size(); ++i) {
    EXPECT_NEAR(prefer[i].first, output[i].first, 0.01);
    EXPECT_NEAR(prefer[i].second, output[i].second, 0.01);
  }
}

TEST(UtilityTest, CalcLeftRightSpace7) {
  const double left_bound = 5.0, right_bound = -5.0;

  Obstacle obs1;

  Boundary tmp{10.0, 15.0, 1.0, 3.0};
  Boundary tmp1{16.0, 20.0, -3.0, 0.0};
  std::vector<ObstacleBoundary> obs_valid_boundary;
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp, &obs1});
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp1, &obs1});
  std::vector<std::pair<double, double>> prefer{{2.0, 2.0}, {2.0, 2.0}};
  std::vector<std::pair<double, double>> output;
  bool bflag = Utility::CalcLeftRightSpace(obs_valid_boundary, left_bound,
                                           right_bound, output);
  EXPECT_TRUE(bflag);
  EXPECT_EQ(prefer.size(), output.size());

  for (std::size_t i = 0; i < prefer.size(); ++i) {
    EXPECT_NEAR(prefer[i].first, output[i].first, 0.01);
    EXPECT_NEAR(prefer[i].second, output[i].second, 0.01);
  }
}

TEST(UtilityTest, CalcLeftRightSpace8) {
  const double left_bound = 5.0, right_bound = -5.0;

  Obstacle obs1;

  Boundary tmp{10.0, 15.0, 1.0, 3.0};
  Boundary tmp1{11.0, 14.0, -3.0, 0.0};
  std::vector<ObstacleBoundary> obs_valid_boundary;
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp, &obs1});
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp1, &obs1});
  std::vector<std::pair<double, double>> prefer{{2.0, 2.0}, {2.0, 2.0}};
  std::vector<std::pair<double, double>> output;
  bool bflag = Utility::CalcLeftRightSpace(obs_valid_boundary, left_bound,
                                           right_bound, output);
  EXPECT_TRUE(bflag);
  EXPECT_EQ(prefer.size(), output.size());

  for (std::size_t i = 0; i < prefer.size(); ++i) {
    EXPECT_NEAR(prefer[i].first, output[i].first, 0.01);
    EXPECT_NEAR(prefer[i].second, output[i].second, 0.01);
  }
}

TEST(UtilityTest, CalcLeftRightSpace9) {
  const double left_bound = 5.0, right_bound = -5.0;

  Obstacle obs1;

  Boundary tmp{10.0, 15.0, 2.0, 3.0};
  Boundary tmp1{15.0, 16.0, -2.0, -1.0};
  Boundary tmp2{11.0, 12.0, 0.0, 1.0};
  std::vector<ObstacleBoundary> obs_valid_boundary;
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp, &obs1});
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp1, &obs1});
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp2, &obs1});
  std::vector<std::pair<double, double>> prefer{
      {3.0, 2.0}, {3.0, 2.0}, {3.0, 2.0}};
  std::vector<std::pair<double, double>> output;
  bool bflag = Utility::CalcLeftRightSpace(obs_valid_boundary, left_bound,
                                           right_bound, output);
  EXPECT_TRUE(bflag);
  EXPECT_EQ(prefer.size(), output.size());

  for (std::size_t i = 0; i < prefer.size(); ++i) {
    EXPECT_NEAR(prefer[i].first, output[i].first, 0.01);
    EXPECT_NEAR(prefer[i].second, output[i].second, 0.01);
  }
}

TEST(UtilityTest, CalcLeftRightSpace10) {
  const double left_bound = 8.0, right_bound = -8.0;

  Obstacle obs1;

  Boundary tmp{10.0, 15.0, 3.0, 4.0};
  Boundary tmp1{11.0, 14.0, -1.0, 0.0};
  Boundary tmp2{13.0, 17.0, -6.0, -5.0};
  Boundary tmp3{12.0, 13.0, 1.0, 2.0};
  Boundary tmp4{14.0, 17.0, -4.0, -3.0};
  std::vector<ObstacleBoundary> obs_valid_boundary;
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp, &obs1});
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp1, &obs1});
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp2, &obs1});
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp3, &obs1});
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp4, &obs1});
  std::vector<std::pair<double, double>> prefer{
      {2, 4}, {2, 4}, {2, 4}, {2, 2}, {2, 2}};
  std::vector<std::pair<double, double>> output;
  bool bflag = Utility::CalcLeftRightSpace(obs_valid_boundary, left_bound,
                                           right_bound, output);
  EXPECT_TRUE(bflag);
  EXPECT_EQ(prefer.size(), output.size());

  for (std::size_t i = 0; i < prefer.size(); ++i) {
    EXPECT_NEAR(prefer[i].first, output[i].first, 0.01);
    EXPECT_NEAR(prefer[i].second, output[i].second, 0.01);
  }
}

TEST(UtilityTest, CalcLeftRightSpace11) {
  const double left_bound = 8.0, right_bound = -8.0;

  Obstacle obs1;

  Boundary tmp{10.0, 15.0, 3.0, 4.0};
  Boundary tmp1{11.0, 14.0, -1.0, 0.0};
  Boundary tmp2{13.0, 17.0, -6.0, -5.0};
  Boundary tmp3{12.0, 13.0, 1.0, 2.0};
  Boundary tmp4{14.0, 17.0, -4.0, -3.0};
  Boundary tmp5{25.0, 30.0, -4.0, -3.0};
  std::vector<ObstacleBoundary> obs_valid_boundary;
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp, &obs1});
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp1, &obs1});
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp2, &obs1});
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp3, &obs1});
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp4, &obs1});
  obs_valid_boundary.emplace_back(ObstacleBoundary{tmp5, &obs1});
  std::vector<std::pair<double, double>> prefer{{2, 4}, {2, 4}, {2, 4},
                                                {2, 2}, {2, 2}, {4, 11}};
  std::vector<std::pair<double, double>> output;
  bool bflag = Utility::CalcLeftRightSpace(obs_valid_boundary, left_bound,
                                           right_bound, output);
  EXPECT_TRUE(bflag);
  EXPECT_EQ(prefer.size(), output.size());

  for (std::size_t i = 0; i < prefer.size(); ++i) {
    EXPECT_NEAR(prefer[i].first, output[i].first, 0.01);
    EXPECT_NEAR(prefer[i].second, output[i].second, 0.01);
  }
}

}  // namespace planning
}  // namespace neodrive
