#include "st_graph_boundary.h"

namespace neodrive {
namespace planning {

// TODO(wyc): boundary_type_ in initializer list is the same to
// default initializer, no need
STGraphBoundary::STGraphBoundary(const std::vector<STPoint>& points)
    : boundary_type_(BoundaryType::UNKNOWN) {
  if (points.size() < 4) {
    LOG_ERROR("points.size() < 4");
    return;
  }
  for (const auto& point : points) {
    points_.emplace_back(point.t(), point.s());
  }
  lower_points_.emplace_back(points[0]);
  lower_points_.emplace_back(points[1]);
  upper_points_.emplace_back(points[2]);
  upper_points_.emplace_back(points[3]);
  build_from_points();
}

STGraphBoundary::STGraphBoundary(std::vector<Vec2d>& points)
    : boundary_type_(BoundaryType::UNKNOWN) {
  if (points.size() < 4) {
    LOG_ERROR("points.size() < 4");
    return;
  }
  points_ = points;
  lower_points_.emplace_back(STPoint(points[0].y(), points[0].x()));
  lower_points_.emplace_back(STPoint(points[1].y(), points[1].x()));
  upper_points_.emplace_back(STPoint(points[2].y(), points[2].x()));
  upper_points_.emplace_back(STPoint(points[3].y(), points[3].x()));
  build_from_points();
}

// TODO(wyc):
// data created on stack, no need to release manually
STGraphBoundary::~STGraphBoundary() {
  st_s_boundary_cache_.clear();
  upper_points_.clear();
  lower_points_.clear();
}

bool STGraphBoundary::is_empty() const { return points_.empty(); }
bool STGraphBoundary::is_point_in_boundary(
    const STGraphPoint& st_graph_point) const {
  return is_point_in(st_graph_point.point());
}

bool STGraphBoundary::is_point_in_boundary(const STPoint& st_point) const {
  return is_point_in(st_point);
}

const Vec2d STGraphBoundary::point(const std::size_t& index) const {
  if (points_.empty()) {
    LOG_ERROR("points.empty(), index: {}; return init", index);
    Vec2d tmp_pt;
    return tmp_pt;
  }
  if (index < points_.size()) return points_[index];
  LOG_ERROR("points.size({}), index: {}; return back", points_.size(), index);
  return points_.back();
}

const std::vector<Vec2d>& STGraphBoundary::points() const { return points_; }

int32_t STGraphBoundary::id() const { return id_; }
STGraphBoundary::BoundaryType STGraphBoundary::boundary_type() const {
  return boundary_type_;
}
double STGraphBoundary::characteristic_length() const {
  return characteristic_length_;
}
Decision STGraphBoundary::get_decision() const { return decision_; }
Decision* STGraphBoundary::mutable_decision() { return &decision_; }
bool STGraphBoundary::is_virtual() const { return virtual_; }
bool STGraphBoundary::is_static() const { return static_; }
double STGraphBoundary::get_speed() const { return speed_; }
double STGraphBoundary::probability() const { return probability_; }

void STGraphBoundary::set_id(const int32_t& id) { id_ = id; }
void STGraphBoundary::set_boundary_type(const BoundaryType& boundary_type) {
  boundary_type_ = boundary_type;
}
void STGraphBoundary::set_characteristic_length(
    const double& characteristic_length) {
  characteristic_length_ = characteristic_length;
}
void STGraphBoundary::set_decision(const Decision& decision) {
  decision_ = decision;
}
void STGraphBoundary::set_virtual(const bool& is_virtual) {
  virtual_ = is_virtual;
}
void STGraphBoundary::set_static(const bool& is_static) { static_ = is_static; }
void STGraphBoundary::set_speed(const double& speed) { speed_ = speed; }
void STGraphBoundary::set_probability(const double& probability) {
  probability_ = probability;
}

bool STGraphBoundary::get_boundary_s_range_by_time(const double& curr_time,
                                                   double* s_upper,
                                                   double* s_lower) {
  if (s_upper == nullptr || s_lower == nullptr) return false;
  if (curr_time < min_t() || curr_time > max_t()) {
    return false;
  }
  if (get_boundary_s_range_from_cache(curr_time, s_upper, s_lower)) {
    return true;
  }
  const Segment2d segment = {STPoint(0.0, curr_time),
                             STPoint(s_high_limit_, curr_time)};
  *s_upper = s_high_limit_;
  *s_lower = 0.0;

  STPoint p_s_first = {0.0, 0.0};
  STPoint p_s_second = {0.0, 0.0};
  if (!get_overlap(segment, &p_s_first, &p_s_second)) {
    LOG_INFO("curr_time[{}] is out of the coverage scope of the boundary.",
             curr_time);
    return false;
  }
  *s_upper = std::min(*s_upper, std::max(p_s_first.s(), p_s_second.s()));
  *s_lower = std::max(*s_lower, std::min(p_s_first.s(), p_s_second.s()));
  insert_boundary_s_range_to_cache(curr_time, *s_upper, *s_lower);
  return true;
}

bool STGraphBoundary::get_boundary_time_scope(double* start_t,
                                              double* end_t) const {
  if (start_t == nullptr || end_t == nullptr) return false;
  if (points_.size() < 4) {
    LOG_ERROR("points_.size({}) too few", points_.size());
    return false;
  }
  *start_t = std::min(points_.front().x(), points_.back().x());
  *end_t = std::max(points_.at(1).x(), points_.at(2).x());
  if (*start_t > *end_t) {
    LOG_ERROR("boundary start_t and end_t reverse.");
    return false;
  }
  return true;
}

double STGraphBoundary::min_t() const { return min_x_; }

double STGraphBoundary::max_t() const { return max_x_; }

double STGraphBoundary::min_s() const { return min_y_; }

double STGraphBoundary::max_s() const { return max_y_; }

bool STGraphBoundary::get_boundary_s_range_from_cache(const double& curr_time,
                                                      double* s_upper,
                                                      double* s_lower) {
  if (s_upper == nullptr || s_lower == nullptr) return false;
  int key = get_cache_key(curr_time);
  // TODO(wyc):
  // if (auto p = st_s_boundary_cache_.find(key);
  //     p != st_s_boundary_cache_.end()) {
  //   ...
  // }
  std::unordered_map<int, std::pair<double, double>>::iterator p =
      st_s_boundary_cache_.find(key);
  if (p != st_s_boundary_cache_.end()) {
    *s_lower = p->second.first;
    *s_upper = p->second.second;
    return true;
  }
  return false;
}

void STGraphBoundary::insert_boundary_s_range_to_cache(const double& curr_time,
                                                       double& s_upper,
                                                       double& s_lower) {
  int key = get_cache_key(curr_time);
  st_s_boundary_cache_[key] = std::make_pair(s_lower, s_upper);
}

int STGraphBoundary::get_cache_key(const double& curr_time) const {
  // TODO(wyc):
  // static_cast
  return int(curr_time * 1e4);
}

Json::Value STGraphBoundary::to_json() const {
  Json::Value root;
  root["id"] = id();

  std::string boundary_type = "UNKNOWN";
  switch (boundary_type_) {
    case BoundaryType::STOP: {
      boundary_type = "STOP";
      break;
    }
    case BoundaryType::YIELD: {
      boundary_type = "YIELD";
      break;
    }
    case BoundaryType::FOLLOW: {
      boundary_type = "FOLLOW";
      break;
    }
    case BoundaryType::OVERTAKE: {
      boundary_type = "OVERTAKE";
      break;
    }
    case BoundaryType::SIDEPASSFOLLOW: {
      boundary_type = "SIDEPASSFOLLOW";
      break;
    }
    case BoundaryType::SIDEPASSLEAD: {
      boundary_type = "SIDEPASSLEAD";
      break;
    }
    case BoundaryType::NUDGE: {
      boundary_type = "NUDGE";
      break;
    }
    default:
      break;
  }
  root["boundary_type"] = boundary_type;

  for (std::size_t i = 0; i < points_.size(); ++i) {
    Json::Value point;
    point["x"] = points_[i].x();
    point["y"] = points_[i].y();
    root["points"].append(point);
  }
  root["decision"] = decision_.to_json();
  root["characteristic_length"] = characteristic_length_;
  root["s_high_limit"] = s_high_limit_;
  root["area"] = area();
  root["min_x"] = min_x_;
  root["max_x"] = max_x_;
  root["min_y"] = min_y_;
  root["max_y"] = max_y_;
  root["probability"] = probability_;
  root["is_convex"] = is_convex();
  return root;
}

}  // namespace planning
}  // namespace neodrive
