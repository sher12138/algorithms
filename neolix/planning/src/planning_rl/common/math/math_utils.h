//!
//! \file
//! \brief Math-related util functions.
//!
#pragma once

#include <limits>
#include <utility>

#include <cmath>
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "common/math/vec2d.h"
//!
//! \namespace neodrive::common::math
//! \brief autobot::common::math
//!
namespace neodrive {
namespace planning_rl {

//!
//! \brief Normalize the value by specified mean and standard deviation.
//! \param value The value to be normalized.
//! \param mean The mean used for normalization.
//! \param std The standard deviation used for normalization.
//! \return The normalized value.
//!
double Normalize(const double value, const double mean, const double std);

//!
//! \brief RELU function used in neural networks as an activation function.
//! \param value The input.
//! \return The output of RELU function.
//!
double Relu(const double value);

//!
//! \brief Sigmoid function used in neural networks as an activation function.
//! \param value The input.
//! \return The output of Sigmoid function.
//!
double Sigmoid(const double x);

//!
//! \brief Softmax function used in neural networks as an activation
//! function. \param vector The input. \return The output of Softmax
//! function.
//!
std::vector<double> Softmax(const std::vector<double>& value,
                            bool use_exp = true);

//!
//! \brief Cross product between two 2-D vectors from the common start point,
//!       and end at two other points.
//! \param start_point The common start point of two vectors in 2-D.
//! \param end_point_1 The end point of the first vector.
//! \param end_point_2 The end point of the second vector.
//! \return The cross product result.
//!
double CrossProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2);

//!
//! \brief Inner product between two 2-D vectors from the common start point,
//!       and end at two other points.
//! \param start_point The common start point of two vectors in 2-D.
//! \param end_point_1 The end point of the first vector.
//! \param end_point_2 The end point of the second vector.
//! \return The inner product result.
//!
double InnerProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2);

//!
//! \brief Cross product between two vectors.
//!       One vector is formed by 1st and 2nd parameters of the function.
//!       The other vector is formed by 3rd and 4th parameters of the function.
//! \param x0 The x coordinate of the first vector.
//! \param y0 The y coordinate of the first vector.
//! \param x1 The x coordinate of the second vector.
//! \param y1 The y coordinate of the second vector.
//! \return The cross product result.
//!
double CrossProd(const double x0, const double y0, const double x1,
                 const double y1);

//!
//! \brief Inner product between two vectors.
//!       One vector is formed by 1st and 2nd parameters of the function.
//!       The other vector is formed by 3rd and 4th parameters of the function.
//! \param x0 The x coordinate of the first vector.
//! \param y0 The y coordinate of the first vector.
//! \param x1 The x coordinate of the second vector.
//! \param y1 The y coordinate of the second vector.
//! \return The inner product result.
//!
double InnerProd(const double x0, const double y0, const double x1,
                 const double y1);

//!
//! \brief Wrap angle to [0, 2 * PI).
//! \param angle the original value of the angle.
//! \return The wrapped value of the angle.
//!
double WrapAngle(const double angle);

//!
//! \brief Normalize angle to [-PI, PI).
//! \param angle the original value of the angle.
//! \return The normalized value of the angle.
//!
double NormalizeAngle(const double angle);

//!
//! \brief Calculate the difference between angle from and to
//! \param from the start angle
//! \param from the end angle
//! \return The difference between from and to. The range is between [0, PI).
//!
double AngleDiff(const double from, const double to);

//!
//! \brief Solve quadratic equation.
//! \param coefficients The coefficients of quadratic equation.
//! \param roots Two roots of the equation if any.
//! \return An integer indicating the success of solving equation.
//!
int SolveQuadraticEquation(const std::vector<double>& coefficients,
                           std::pair<double, double>* roots);

//!
//! \brief Evaluate quintic polynomial.
//! \param coefficients of the quintic polynomial, lower to higher.
//! \param parameter of the quintic polynomial.
//! \return order of derivative to evaluate.
//!
double EvaluateQuinticPolynomial(const std::array<double, 6>& coeffs,
                                 const double t, const uint32_t order,
                                 const double end_t, const double end_v);

//!
//! \brief Evaluate quartic polynomial.
//! \param coefficients of the quartic polynomial, lower to higher.
//! \param parameter of the quartic polynomial.
//! \return order of derivative to evaluate.
//!
double EvaluateQuarticPolynomial(const std::array<double, 5>& coeffs,
                                 const double t, const uint32_t order,
                                 const double end_t, const double end_v);

//!
//! \brief Evaluate cubic polynomial.
//! \param coefficients of the cubic polynomial, lower to higher.
//! \param parameter of the cubic polynomial.
//! \param end_t ending time for extrapolation.
//! \param end_v ending velocity for extrapolation.
//! \return order of derivative to evaluate.
//!
double EvaluateCubicPolynomial(
    const std::array<double, 4>& coefs, const double t, const uint32_t order,
    const double end_t = std::numeric_limits<double>::infinity(),
    const double end_v = 0.0);

//!
//! \brief Compute squared value.
//! \param value The target value to get its squared value.
//! \return Squared value of the input value.
//!
template <typename T>
inline T Square(const T value) {
  return value * value;
}

template <typename T>
inline T sqr(const T value) {
  return value * value;
}

template <std::size_t N>
std::array<double, 2 * N - 2> ComputePolynomial(
    const std::array<double, N - 1>& start_state,
    const std::array<double, N - 1>& end_state, const double param);

template <>
inline std::array<double, 4> ComputePolynomial<3>(
    const std::array<double, 2>& start_state,
    const std::array<double, 2>& end_state, const double param) {
  std::array<double, 4> coefs;
  coefs[0] = start_state[0];
  coefs[1] = start_state[1];

  auto m0 = end_state[0] - start_state[0] - start_state[1] * param;
  auto m1 = end_state[1] - start_state[1];

  auto param_p3 = param * param * param;
  coefs[3] = (m1 * param - 2.0 * m0) / param_p3;

  coefs[2] = (m1 - 3.0 * coefs[3] * param * param) / param * 0.5;
  return coefs;
}

std::vector<double> gradient(std::vector<double>& input);

bool cal_curvature(std::vector<double>& x_list, std::vector<double>& y_list,
                   std::vector<std::vector<double>>& curvature_and_theta);

}  // namespace planning_rl
}  // namespace neodrive
