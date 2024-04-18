#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

namespace neodrive {
namespace planning {
namespace tk {

class spline {
 public:
  enum spline_type { linear = 10, cspline = 30, cspline_hermite = 31 };
  enum bd_type { first_deriv = 1, second_deriv = 2, not_a_knot = 3 };

 protected:
  std::vector<double> m_x, m_y;       // x,y coordinates of points
  std::vector<double> m_b, m_c, m_d;  // spline coefficients
  double m_c0;                        // for left extrapolation
  spline_type m_type;
  bd_type m_left, m_right;
  double m_left_value, m_right_value;
  bool m_made_monotonic;
  void set_coeffs_from_b();             // calculate c_i, d_i from b_i
  size_t find_closest(double x) const;  // closest idx so that m_x[idx]<=x

 public:
  spline()
      : m_type(cspline),
        m_left(second_deriv),
        m_right(second_deriv),
        m_left_value(0.0),
        m_right_value(0.0),
        m_made_monotonic(false) {}

  spline(const std::vector<double>& X, const std::vector<double>& Y,
         spline_type type = cspline, bool make_monotonic = false,
         bd_type left = second_deriv, double left_value = 0.0,
         bd_type right = second_deriv, double right_value = 0.0)
      : m_type(type),
        m_left(left),
        m_right(right),
        m_left_value(left_value),
        m_right_value(right_value),
        m_made_monotonic(false) {
    this->set_points(X, Y, m_type);
    if (make_monotonic) {
      this->make_monotonic();
    }
  }

  void set_boundary(bd_type left, double left_value, bd_type right,
                    double right_value);

  void set_points(const std::vector<double>& x, const std::vector<double>& y,
                  spline_type type = cspline);

  bool make_monotonic();

  double operator()(double x) const;
  double deriv(int order, double x) const;

  std::vector<double> solve(double y, bool ignore_extrapolation = true) const;

  std::vector<double> get_x() const { return m_x; }
  std::vector<double> get_y() const { return m_y; }
  double get_x_min() const {
    assert(!m_x.empty());
    return m_x.front();
  }
  double get_x_max() const {
    assert(!m_x.empty());
    return m_x.back();
  }

  void get_spline_params(std::vector<double>* a, std::vector<double>* b,
                         std::vector<double>* c, std::vector<double>* d) {
    *a = m_y;
    *b = m_b;
    *c = m_c;
    *d = m_d;
  }
};

}  // namespace tk
}  // namespace planning
}  // namespace neodrive
