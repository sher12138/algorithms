#include "utility.h"

#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {

bool Utility::check_area(const Box2d& box) {
  std::vector<Vec2d> points;
  box.get_all_corners(&points);
  return check_area(points);
}

double Utility::gaussian(const double u, const double std, const double x) {
  return (1.0 / std::sqrt(2.0 * M_PI * std * std)) *
         std::exp(-(x - u) * (x - u) / (2.0 * std * std));
}

double Utility::sigmoid(const double x) { return 1.0 / (1.0 + std::exp(-x)); }

void Utility::uniform_slice(double start, double end, uint32_t num,
                            std::vector<double>* sliced) {
  if (!sliced || num == 0) {
    return;
  }
  const double delta = (end - start) / num;
  sliced->resize(num + 1);
  double s = start;
  for (uint32_t i = 0; i < num; ++i, s += delta) {
    sliced->at(i) = s;
  }
  sliced->at(num) = end;
}

Polygon2d Utility::get_trajectory_point_polygon(
    const Vec2d& obstacle_center_point, const Vec2d& trajectory_point,
    const double obstacle_heading, const double trajectory_point_heading,
    const Polygon2d& obstacle_polygon) {
  std::vector<Vec2d> polygon_point;
  double heading_diff =
      normalize_angle(trajectory_point_heading - obstacle_heading);
  for (const auto& point : obstacle_polygon.points()) {
    double x =
        (point.x() - obstacle_center_point.x()) * std::cos(heading_diff) -
        (point.y() - obstacle_center_point.y()) * std::sin(heading_diff);
    double y =
        (point.x() - obstacle_center_point.x()) * std::sin(heading_diff) +
        (point.y() - obstacle_center_point.y()) * std::cos(heading_diff);
    polygon_point.push_back(
        {x + trajectory_point.x(), y + trajectory_point.y()});
  }
  return Polygon2d(polygon_point);
}

bool Utility::CalcBoundary(const ReferenceLinePtr& reference_line,
                           const Polygon2d& polygon, const Box2d& bounding_box,
                           Boundary* const boundary) {
  if (reference_line == nullptr || boundary == nullptr) {
    LOG_ERROR("input data is nullptr");
    return false;
  }
  bool has_polygon = false;
  SLPoint sl_point;
  for (const auto& pt : polygon.points()) {
    has_polygon = true;
    if (!reference_line->GetPointInFrenetFrame(pt, &sl_point)) {
      LOG_ERROR("GetPointInFrenetFrame failed");
      return false;
    }
    double s = sl_point.s();
    double l = sl_point.l();
    boundary->set_start_s(std::min(boundary->start_s(), s));
    boundary->set_end_s(std::max(boundary->end_s(), s));
    boundary->set_start_l(std::min(boundary->start_l(), l));
    boundary->set_end_l(std::max(boundary->end_l(), l));
  }
  if (!has_polygon) {
    return CalcBoundary(reference_line, bounding_box, boundary);
  }
  return true;
}

bool Utility::CalcBoundary(const ReferenceLinePtr& reference_line,
                           const Box2d& bounding_box,
                           Boundary* const boundary) {
  if (reference_line == nullptr || boundary == nullptr) {
    LOG_ERROR("input data is nullptr");
    return false;
  }
  std::vector<Vec2d> corners;
  bounding_box.get_all_corners(&corners);
  SLPoint sl_point;
  for (const auto& pt : corners) {
    if (!reference_line->GetPointInFrenetFrame(pt, &sl_point)) {
      LOG_ERROR("GetPointInFrenetFrame failed");
      return false;
    }
    double s = sl_point.s();
    double l = sl_point.l();
    boundary->set_start_s(std::min(boundary->start_s(), s));
    boundary->set_end_s(std::max(boundary->end_s(), s));
    boundary->set_start_l(std::min(boundary->start_l(), l));
    boundary->set_end_l(std::max(boundary->end_l(), l));
  }
  return true;
}

bool Utility::CalcBoundary(const ReferencePoint& reference_point,
                           const Box2d& bounding_box, const SLPoint& center_sl,
                           Boundary* const boundary) {
  if (boundary == nullptr) {
    LOG_ERROR("input data is nullptr");
    return false;
  }
  double heading_diff =
      std::fabs(bounding_box.heading() - reference_point.heading());
  double impact_width =
      bounding_box.half_length() * fabs(std::sin(heading_diff)) +
      bounding_box.half_width() * fabs(std::cos(heading_diff));
  double impact_length =
      bounding_box.half_length() * fabs(std::cos(heading_diff)) +
      bounding_box.half_width() * fabs(std::sin(heading_diff));
  boundary->set_start_s(center_sl.s() - impact_length);
  boundary->set_end_s(center_sl.s() + impact_length);
  boundary->set_start_l(center_sl.l() - impact_width);
  boundary->set_end_l(center_sl.l() + impact_width);
  return true;
}

// relative to plan
double Utility::look_forward_distance(double v) {
  double forward_dis = v * FLAGS_planning_trajectory_time_length;
  if (forward_dis > FLAGS_planning_look_forward_short_distance) {
    return FLAGS_planning_look_forward_long_distance;
  }
  return FLAGS_planning_look_forward_short_distance;
}

void Utility::calc_plan_length(const double init_s, const double ref_length,
                               const double v, double* backward_length,
                               double* forward_length) {
  // *forward_length = Utility::look_forward_distance(v);
  *forward_length = std::max(0.0, ref_length - init_s);
  double backward_start_s =
      std::max(0.0, init_s - FLAGS_planning_look_backward_distance);
  *backward_length = init_s - backward_start_s;
  LOG_INFO(
      "forward_length: {:.2f}, backward_length: {:.2f}, ref line len: {:.2f},"
      " init s: {:.2f}",
      *forward_length, *backward_length, ref_length, init_s);
  return;
}

bool Utility::ExtendPathFrontLastPoint(ReferencePointVec1d& ref_pts,
                                       const std::size_t& num,
                                       const double& distance,
                                       const bool& is_forward) {
  if (ref_pts.empty()) return false;

  ReferencePointVec1d front_pts;
  front_pts.clear();
  int sign = is_forward ? 1 : -1;

  Vec2d start_pos(ref_pts.front().x(), ref_pts.front().y());
  double start_heading = ref_pts.front().heading();
  start_heading = normalize_angle(start_heading);
  const auto start_unit_vec = Vec2d::create_unit_vec(start_heading);
  ReferencePoint ref_point = ref_pts.front();
  double front_s = ref_point.s();
  for (std::size_t i = 0; i < num; ++i) {
    Vec2d step_pos = start_unit_vec * (-sign * distance) + start_pos;
    ref_point.set_x(step_pos.x());
    ref_point.set_y(step_pos.y());
    ref_point.set_heading(start_heading);
    ref_point.set_s(-(i * distance) + front_s);
    front_pts.push_back(ref_point);
    start_pos = step_pos;
  }
  reverse(front_pts.begin(), front_pts.end());
  ref_pts.insert(ref_pts.begin(), front_pts.begin(), front_pts.end());

  Vec2d end_pos(ref_pts.back().x(), ref_pts.back().y());
  const double end_heading = ref_pts.back().heading();
  const auto unit_vec = Vec2d::create_unit_vec(end_heading);
  ref_point = ref_pts.back();
  double back_s = ref_point.s();
  for (std::size_t i = 0; i < num; ++i) {
    Vec2d step_pos = unit_vec * sign * distance + end_pos;
    ref_point.set_x(step_pos.x());
    ref_point.set_y(step_pos.y());
    ref_point.set_heading(end_heading);
    ref_point.set_s(i * distance + back_s);
    ref_pts.push_back(ref_point);
    end_pos = step_pos;
  }

  // update s
  if (ref_pts.front().s() > 0.0) {
    return true;
  }
  front_s = ref_pts.front().s();
  front_s = 1.0 - front_s;
  for (std::size_t i = 0; i < ref_pts.size(); ++i) {
    ref_pts[i].set_s(ref_pts[i].s() + front_s);
  }

  return true;
}

bool Utility::CalcLeftRightSpace(std::vector<ObstacleBoundary>& obs_boundarys,
                                 const double& left_bound,
                                 const double& right_bound,
                                 std::vector<std::pair<double, double>>& spaces,
                                 const double L_THRESHOLD,
                                 const double S_THRESHOLD) {
  std::sort(obs_boundarys.begin(), obs_boundarys.end(), [](auto& a, auto& b) {
    return a.first.start_s() < b.first.start_s();
  });

  auto MergeFun = [=](auto& boundarys, auto& self) {
    for (auto& bound : boundarys) {
      if ((bound.first.end_l() > self.first.end_l() &&
           bound.first.start_l() - self.first.end_l() < L_THRESHOLD) ||
          (bound.first.start_l() < self.first.start_l() &&
           self.first.start_l() - bound.first.end_l() < L_THRESHOLD)) {
        self.first.merge(bound.first);
      }
    }
  };

  auto LessThan = [](auto& a, auto& b) {
    return a.first.end_s() < b.first.end_s();
  };
  std::multiset<ObstacleBoundary, decltype(LessThan)> left_holder(LessThan);
  int size = obs_boundarys.size();
  for (int i = 0; i < size; ++i) {
    while (!left_holder.empty() &&
           left_holder.begin()->first.end_s() <
               obs_boundarys[i].first.start_s() - S_THRESHOLD) {
      left_holder.erase(left_holder.begin());
    }
    MergeFun(left_holder, obs_boundarys[i]);
    left_holder.insert(obs_boundarys[i]);
  }

  auto GreatThan = [](auto& a, auto& b) {
    return a.first.start_s() > b.first.start_s();
  };

  std::multiset<ObstacleBoundary, decltype(GreatThan)> right_holder(GreatThan);
  for (int i = size - 1; i >= 0; --i) {
    while (!right_holder.empty() &&
           right_holder.begin()->first.start_s() >
               obs_boundarys[i].first.end_s() + S_THRESHOLD) {
      right_holder.erase(right_holder.begin());
    }
    MergeFun(right_holder, obs_boundarys[i]);
    right_holder.insert(obs_boundarys[i]);
  }

  // calc distance to boundary
  spaces.clear();
  spaces.resize(size);

  auto get_right_left = [=](auto& boundarys, auto& self, double right,
                            double left) -> std::pair<double, double> {
    for (auto bound : boundarys) {
      if (bound.first.start_l() > self.first.end_l())
        left = std::min(left, bound.first.start_l());
      if (bound.first.end_l() < self.first.start_l())
        right = std::max(right, bound.first.end_l());
    }
    return {right, left};
  };

  std::multiset<ObstacleBoundary, decltype(LessThan)> bleft(LessThan);
  for (int i = 0; i < size; ++i) {
    const auto& bound = obs_boundarys[i];
    while (!bleft.empty() &&
           bleft.begin()->first.end_s() < bound.first.start_s())
      bleft.erase(bleft.begin());

    auto [right, left] = get_right_left(bleft, bound, right_bound, left_bound);
    spaces[i] = {std::max(0., bound.first.start_l() - right),
                 std::max(0., left - bound.first.end_l())};
    bleft.insert(bound);
  }

  std::multiset<ObstacleBoundary, decltype(GreatThan)> bright(GreatThan);
  for (int i = size - 1; i >= 0; --i) {
    const auto& bound = obs_boundarys[i];
    while (!bright.empty() &&
           bright.begin()->first.start_s() > bound.first.end_s())
      bright.erase(bright.begin());

    auto [right, left] = get_right_left(bright, bound, right_bound, left_bound);
    spaces[i].first =
        std::min(spaces[i].first, std::max(0., bound.first.start_l() - right));
    spaces[i].second =
        std::min(spaces[i].second, std::max(0., left - bound.first.end_l()));
    bright.insert(bound);
  }

  if (spaces.size() != obs_boundarys.size()) {
    LOG_ERROR("spaces.size({}) != obs_boundarys.size({})", spaces.size(),
              obs_boundarys.size());
    return false;
  }

  return true;
}

// Simple state transfer
void Utility::NewtonSecLaw(const double acc, const double x0, const double v0,
                           const double delta_t, double* v, double* x) {
  double v_, x_;
  v_ = v0 + acc * delta_t;
  x_ = x0 + v0 * delta_t + 0.5 * acc * std::pow(delta_t, 2);
  if (v_ < 0.0) {
    v_ = 0.0;
    x_ = x0 - 0.5 * std::pow(v0, 2) / (acc + 1e-8);
  }
  *v = v_;
  *x = x_;
}

// Fitting Quadratic Polynomial with Least Squares Method
void Utility::FittingQuartic(const Eigen::VectorXd& xs,
                             const Eigen::VectorXd& ys,
                             Eigen::VectorXd* coeff) {
  int n = 4;
  Eigen::MatrixXd A(xs.size(), n + 1);
  Eigen::VectorXd b = ys;
  for (int i = 0; i < xs.size(); ++i) {
    for (int j = 0; j <= n; ++j) {
      A(i, j) = std::pow(xs(i), j);
    }
  }
  *coeff = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
}

bool Utility::SolveQuadraticEquation(const double a, const double b,
                                     const double c, double* root) {
  double discriminant = b * b - 4 * a * c;
  double root_ = -1;
  if (std::abs(a - 0.0) < 1e-6) {
    root_ = -c / b;
  } else {
    if (discriminant > 0.0) {
      root_ = (-b + std::sqrt(discriminant)) / (2 * a);
    } else if (std::abs(discriminant - 0.0) < 1e-6) {
      root_ = -b / (2 * a);
    }
  }
  *root = root_;
  return (root_ > 0.0);
}

}  // namespace planning
}  // namespace neodrive
