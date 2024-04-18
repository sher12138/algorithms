#include "double.h"

namespace neodrive {
namespace planning {

Double::Double(const double value) : value_(value) {
  if (std::isnan(value)) {
    LOG_ERROR("value is nan");
  }
}

Double::Double(const Double& other) : value_(other.value()) {
  if (std::isnan(other.value())) {
    LOG_ERROR("value is nan");
  }
}

double Double::value() const { return value_; }

int Double::compare(const double d1, const double d2, const double epsilon) {
  if (std::isnan(d1)) {
    LOG_ERROR("d1 is nan");
    return 0;
  }
  if (std::isnan(d2)) {
    LOG_ERROR("d2 is nan");
    return 0;
  }

  if (definitely_greater_than(d1, d2, epsilon)) {
    return 1;
  } else if (definitely_less_than(d1, d2, epsilon)) {
    return -1;
  } else {
    return 0;
  }
}

bool Double::is_nan(const double d) { return std::isnan(d); }

bool Double::is_nan(const Double d) { return is_nan(d.value()); }

int Double::compare(const double d1, const double d2) {
  return compare(d1, d2, s_epsilon_);
}

int Double::compare(const Double& d1, const Double& d2, const double epsilon) {
  return compare(d1.value(), d2.value(), epsilon);
}

int Double::compare(const Double& d1, const Double& d2) {
  return compare(d1.value(), d2.value());
}

Double Double::sqrt(const Double& d1) { return Double(std::sqrt(d1.value())); }

int Double::compare_to(const double d1, const double epsilon) const {
  if (std::isnan(d1)) {
    LOG_ERROR("d1 is nan");
    return 0;
  }
  if (definitely_greater_than(value_, d1, epsilon)) {
    return 1;
  } else if (definitely_less_than(value_, d1, epsilon)) {
    return -1;
  } else {
    return 0;
  }
}

int Double::compare_to(const double d1) const {
  return compare_to(d1, s_epsilon_);
}

int Double::compare_to(const Double& d1, const double epsilon) const {
  return compare_to(d1.value(), epsilon);
}

int Double::compare_to(const Double& d1) const {
  return compare_to(d1.value(), s_epsilon_);
}

Double& Double::operator=(const Double& other) {
  value_ = other.value();
  return *this;
}

Double Double::operator+(const Double& other) const {
  return Double(value_ + other.value());
}

Double Double::operator-(const Double& other) const {
  return Double(value_ - other.value());
}

Double Double::operator*(const Double& other) const {
  return Double(value_ * other.value());
}

Double Double::operator/(const Double& other) const {
  return Double(value_ / other.value());
}

Double& Double::operator+=(const Double& other) {
  value_ += other.value();
  return *this;
}

Double& Double::operator-=(const Double& other) {
  value_ -= other.value();
  return *this;
}

Double& Double::operator*=(const Double& other) {
  value_ *= other.value();
  return *this;
}

Double& Double::operator/=(const Double& other) {
  value_ /= other.value();
  return *this;
}

bool Double::operator>(const Double& other) const {
  return definitely_greater_than(value_, other.value(), s_epsilon_);
}

bool Double::operator>=(const Double& other) const {
  return !((*this) < other);
}

bool Double::operator<(const Double& other) const {
  return definitely_less_than(value_, other.value(), s_epsilon_);
}

bool Double::operator<=(const Double& other) const {
  return !((*this) > other);
}

bool Double::operator==(const Double& other) const {
  return essentially_equal(value_, other.value(), s_epsilon_);
}

bool Double::approximately_equal(double a, double b, double epsilon) {
  return std::fabs(a - b) <= std::fmax(std::fabs(a), std::fabs(b)) * epsilon;
}

bool Double::essentially_equal(double a, double b, double epsilon) {
  return std::fabs(a - b) <= std::fmin(std::fabs(a), std::fabs(b)) * epsilon;
}

bool Double::definitely_greater_than(double a, double b, double epsilon) {
  return (a - b) > std::fmax(std::fabs(a), std::fabs(b)) * epsilon;
}

bool Double::definitely_less_than(double a, double b, double epsilon) {
  return (b - a) > std::fmax(std::fabs(a), std::fabs(b)) * epsilon;
}

}  // namespace planning
}  // namespace neodrive
