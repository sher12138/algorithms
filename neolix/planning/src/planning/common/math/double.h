#pragma once

#include <cmath>
#include <limits>

#include "src/planning/common/planning_logger.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

class Double {
 public:
  Double(const double value);
  Double(const Double& other);
  ~Double() = default;

  double value() const;

  static int compare(const double d1, const double d2, const double epsilon);
  static int compare(const double d1, const double d2);
  static int compare(const Double& d1, const Double& d2, const double epsilon);
  static int compare(const Double& d1, const Double& d2);
  static bool is_nan(const double d);
  static bool is_nan(const Double d);
  static Double sqrt(const Double& d1);

  int compare_to(const double d1, const double epsilon) const;
  int compare_to(const double d1) const;
  int compare_to(const Double& d1, const double epsilon) const;
  int compare_to(const Double& d1) const;

  Double operator+(const Double& other) const;
  Double operator-(const Double& other) const;
  Double operator*(const Double& other) const;
  Double operator/(const Double& other) const;

  Double& operator=(const Double& other);
  Double& operator+=(const Double& other);
  Double& operator-=(const Double& other);
  Double& operator*=(const Double& other);
  Double& operator/=(const Double& other);

  bool operator>(const Double& other) const;
  bool operator>=(const Double& other) const;
  bool operator<(const Double& other) const;
  bool operator<=(const Double& other) const;
  bool operator==(const Double& other) const;

 private:
  double value_ = 0.0;
  static constexpr double s_epsilon_ = std::numeric_limits<double>::epsilon();
  static bool approximately_equal(double a, double b, double epsilon);
  static bool essentially_equal(double a, double b, double epsilon);
  static bool definitely_greater_than(double a, double b, double epsilon);
  static bool definitely_less_than(double a, double b, double epsilon);
};

}  // namespace planning
}  // namespace neodrive
