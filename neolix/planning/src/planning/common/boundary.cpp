#include "boundary.h"

namespace neodrive {
namespace planning {

Boundary::Boundary(const double s_s, const double e_s, const double s_l,
                   const double e_l)
    : start_s_(s_s), end_s_(e_s), start_l_(s_l), end_l_(e_l) {}

void Boundary::reset() {
  start_s_ = std::numeric_limits<double>::max();
  end_s_ = std::numeric_limits<double>::lowest();
  start_l_ = std::numeric_limits<double>::max();
  end_l_ = std::numeric_limits<double>::lowest();
}

double Boundary::start_s() const { return start_s_; }

double Boundary::end_s() const { return end_s_; }

double Boundary::start_l() const { return start_l_; }

double Boundary::end_l() const { return end_l_; }

void Boundary::set_start_s(const double start_s) { start_s_ = start_s; }

void Boundary::set_end_s(const double end_s) { end_s_ = end_s; }

void Boundary::set_start_l(const double start_l) { start_l_ = start_l; }

void Boundary::set_end_l(const double end_l) { end_l_ = end_l; }

bool Boundary::has_lateral_overlap(const Boundary& boundary) const {
  double max_start_l = std::max(boundary.start_l(), start_l_);
  double min_end_l = std::min(boundary.end_l(), end_l_);
  return max_start_l <= min_end_l;
}

bool Boundary::has_horizontal_overlap(const Boundary& boundary) const {
  double max_start_s = std::max(boundary.start_s(), start_s_);
  double min_end_s = std::min(boundary.end_s(), end_s_);
  return max_start_s <= min_end_s;
}

bool Boundary::has_overlap(const Boundary& boundary) const {
  return has_lateral_overlap(boundary) && has_horizontal_overlap(boundary);
}

bool Boundary::is_point_in(const SLPoint& pt) const {
  return (pt.s() < end_s_ && pt.s() > start_s_ && pt.l() < end_l_ &&
          pt.l() > start_l_);
}

bool Boundary::is_s_in(const double s) const {
  return (s < end_s_ && s > start_s_);
}

bool Boundary::is_l_in(const double l) const {
  return (l < end_l_ && l > start_l_);
}

double Boundary::lateral_distance(const Boundary& boundary) const {
  if (has_lateral_overlap(boundary)) return -1.0;

  return start_l_ >= boundary.end_l() ? start_l_ - boundary.end_l()
                                      : boundary.start_l() - end_l_;
}

void Boundary::expand(const double expand_dist) {
  start_s_ -= expand_dist;
  end_s_ += expand_dist;
  start_l_ -= expand_dist;
  end_l_ += expand_dist;
}

void Boundary::left_expand(const double expand_dist) {
  end_l_ = end_l_ + expand_dist;
}

void Boundary::right_expand(const double expand_dist) {
  start_l_ = start_l_ - expand_dist;
}

void Boundary::front_expand(const double expand_dist) {
  end_s_ = end_s_ + expand_dist;
}

void Boundary::back_expand(const double expand_dist) {
  start_s_ = start_s_ - expand_dist;
}

void Boundary::merge(const Boundary& boundary) {
  start_s_ = std::min(start_s_, boundary.start_s());
  end_s_ = std::max(end_s_, boundary.end_s());
  start_l_ = std::min(start_l_, boundary.start_l());
  end_l_ = std::max(end_l_, boundary.end_l());
}

double Boundary::distance_to(const Boundary& other) const {
  double other_length = other.end_s() - other.start_s();
  double other_width = other.end_l() - other.start_l();
  double other_center_s = (other.start_s() + other.end_s()) / 2.0;
  double other_center_l = (other.start_l() + other.end_l()) / 2.0;
  double curr_length = end_s_ - start_s_;
  double curr_width = end_l_ - start_l_;
  double curr_center_s = (start_s_ + end_s_) / 2.0;
  double curr_center_l = (start_l_ + end_l_) / 2.0;
  const double ds = std::abs(other_center_s - curr_center_s) -
                    other_length / 2.0 - curr_length / 2.0;
  const double dl = std::abs(other_center_l - curr_center_l) -
                    other_width / 2.0 - curr_width / 2.0;
  if (ds <= 0.0) {
    return std::max(0.0, dl);
  }
  if (dl <= 0.0) {
    return ds;
  }
  return std::hypot(ds, dl);
}

double Boundary::distance_to(const Vec2d& point) const {
  double curr_length = end_s_ - start_s_;
  double curr_width = end_l_ - start_l_;
  double curr_center_s = (start_s_ + end_s_) / 2.0;
  double curr_center_l = (start_l_ + end_l_) / 2.0;
  const double ds = std::abs(point.x() - curr_center_s) - curr_length / 2.0;
  const double dl = std::abs(point.y() - curr_center_l) - curr_width / 2.0;
  if (ds <= 0.0) {
    return std::max(0.0, dl);
  }
  if (dl <= 0.0) {
    return ds;
  }
  return std::hypot(ds, dl);
}

}  // namespace planning
}  // namespace neodrive
