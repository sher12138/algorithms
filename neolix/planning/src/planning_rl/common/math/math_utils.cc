#include "common/math/math_utils.h"

namespace neodrive {
namespace planning_rl {

double Normalize(const double value, const double mean, const double std) {
  const double eps = 1e-10;
  return (value - mean) / (std + eps);
}

double Relu(const double value) { return (value > 0.0) ? value : 0.0; }

double Sigmoid(const double x) { return 1.0 / (1.0 + std::exp(-x)); }

std::vector<double> Softmax(const std::vector<double>& value, bool use_exp) {
  std::vector<double> result;
  double sum = 0.0;
  for (std::size_t i = 0; i < value.size(); ++i) {
    double exp_value = std::max(0.001, value[i]);
    if (use_exp) {
      exp_value = std::exp(value[i]);
    }
    sum += exp_value;
    result.push_back(exp_value);
  }
  for (std::size_t i = 0; i < value.size(); ++i) {
    result[i] = result[i] / sum;
  }
  return result;
}

double CrossProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2) {
  return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
}

double InnerProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2) {
  return (end_point_1 - start_point).InnerProd(end_point_2 - start_point);
}

double CrossProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * y1 - x1 * y0;
}

double InnerProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * x1 + y0 * y1;
}

double WrapAngle(const double angle) {
  const double new_angle = std::fmod(angle, M_PI * 2.0);
  return new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle;
}

double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

double AngleDiff(const double from, const double to) {
  return NormalizeAngle(to - from);
}

int SolveQuadraticEquation(const std::vector<double>& coefficients,
                           std::pair<double, double>* roots) {
  if (coefficients.size() != 3) {
    return -1;
  }
  const double a = coefficients[0];
  const double b = coefficients[1];
  const double c = coefficients[2];
  if (std::fabs(a) <= std::numeric_limits<double>::epsilon()) {
    return -1;
  }

  double delta = b * b - 4.0 * a * c;
  if (delta < 0.0) {
    return -1;
  }

  double sqrt_delta = std::sqrt(delta);
  roots->first = (-b + sqrt_delta) * 0.5 / a;
  roots->second = (-b - sqrt_delta) * 0.5 / a;
  return 0;
}

double EvaluateQuinticPolynomial(const std::array<double, 6>& coeffs,
                                 const double t, const uint32_t order,
                                 const double end_t, const double end_v) {
  if (t >= end_t) {
    switch (order) {
      case 0: {
        double end_value =
            ((((coeffs[5] * end_t + coeffs[4]) * end_t + coeffs[3]) * end_t +
              coeffs[2]) *
                 end_t +
             coeffs[1]) *
                end_t +
            coeffs[0];
        return end_value + end_v * (t - end_t);
      }
      case 1: {
        return end_v;
      }
      default: { return 0.0; }
    }
  }
  switch (order) {
    case 0: {
      return ((((coeffs[5] * t + coeffs[4]) * t + coeffs[3]) * t + coeffs[2]) *
                  t +
              coeffs[1]) *
                 t +
             coeffs[0];
    }
    case 1: {
      return (((5.0 * coeffs[5] * t + 4.0 * coeffs[4]) * t + 3.0 * coeffs[3]) *
                  t +
              2.0 * coeffs[2]) *
                 t +
             coeffs[1];
    }
    case 2: {
      return (((20.0 * coeffs[5] * t + 12.0 * coeffs[4]) * t) +
              6.0 * coeffs[3]) *
                 t +
             2.0 * coeffs[2];
    }
    case 3: {
      return (60.0 * coeffs[5] * t + 24.0 * coeffs[4]) * t + 6.0 * coeffs[3];
    }
    case 4: {
      return 120.0 * coeffs[5] * t + 24.0 * coeffs[4];
    }
    case 5: {
      return 120.0 * coeffs[5];
    }
    default:
      return 0.0;
  }
}

double EvaluateQuarticPolynomial(const std::array<double, 5>& coeffs,
                                 const double t, const uint32_t order,
                                 const double end_t, const double end_v) {
  if (t >= end_t) {
    switch (order) {
      case 0: {
        double end_value =
            (((coeffs[4] * end_t + coeffs[3]) * end_t + coeffs[2]) * end_t +
             coeffs[1]) *
                end_t +
            coeffs[0];
        return end_value + (t - end_t) * end_v;
      }
      case 1: {
        return end_v;
      }
      default: { return 0.0; }
    }
  }
  switch (order) {
    case 0: {
      return (((coeffs[4] * t + coeffs[3]) * t + coeffs[2]) * t + coeffs[1]) *
                 t +
             coeffs[0];
    }
    case 1: {
      return ((4.0 * coeffs[4] * t + 3.0 * coeffs[3]) * t + 2.0 * coeffs[2]) *
                 t +
             coeffs[1];
    }
    case 2: {
      return (12.0 * coeffs[4] * t + 6.0 * coeffs[3]) * t + 2.0 * coeffs[2];
    }
    case 3: {
      return 24.0 * coeffs[4] * t + 6.0 * coeffs[3];
    }
    case 4: {
      return 24.0 * coeffs[4];
    }
    default:
      return 0.0;
  }
}

double EvaluateCubicPolynomial(const std::array<double, 4>& coefs,
                               const double t, const uint32_t order,
                               const double end_t, const double end_v) {
  if (t > end_t) {
    switch (order) {
      case 0: {
        double end_value =
            ((coefs[3] * end_t + coefs[2]) * end_t + coefs[1]) * end_t +
            coefs[0];
        return end_value + (t - end_t) * end_v;
      }
      case 1: {
        return end_v;
      }
      default: { return 0.0; }
    }
  }

  switch (order) {
    case 0: {
      return ((coefs[3] * t + coefs[2]) * t + coefs[1]) * t + coefs[0];
    }
    case 1: {
      return (3.0 * coefs[3] * t + 2.0 * coefs[2]) * t + coefs[1];
    }
    case 2: {
      return 6.0 * coefs[3] * t + 2.0 * coefs[2];
    }
    case 3: {
      return 6.0 * coefs[3];
    }
    default:
      return 0.0;
  }
}
std::vector<double> gradient(std::vector<double>& input) {
  if (input.size() <= 1) return input;
  std::vector<double> res;
  for (int j = 0; j < input.size(); j++) {
    int j_left = j - 1;
    int j_right = j + 1;
    if (j_left < 0) {
      j_left = 0;  // use your own boundary handler
      j_right = 1;
    }
    if (j_right >= input.size()) {
      j_right = input.size() - 1;
      j_left = j_right - 1;
    }
    // gradient value at position j
    double dist_grad = (input[j_right] - input[j_left]) / 2.0;
    res.push_back(dist_grad);
  }
  return res;
}
bool cal_curvature(std::vector<double>& x_list, std::vector<double>& y_list,
                   std::vector<std::vector<double>>& curvature_and_theta) {
  std::vector<double> x_gradient = gradient(x_list);
  std::vector<double> y_gradient = gradient(y_list);
  std::vector<double> theta;
  for (size_t i = 0; i < x_gradient.size(); i++) {
    theta.push_back(atan2(y_gradient[i], x_gradient[i]));
  }
  std::vector<double> theta_gradient = gradient(theta);
  std::vector<double> curvature_values;
  for (size_t i = 0; i < x_gradient.size(); i++) {
    curvature_values.push_back(
        theta_gradient[i] /
        (sqrt(pow(x_gradient[i], 2) + pow(y_gradient[i], 2)) + 0.00001));
  }
  curvature_and_theta.push_back(curvature_values);
  curvature_and_theta.push_back(theta);
  return true;
}

}  // namespace planning_rl
}  // namespace neodrive
