#pragma once

#include <unordered_map>

#include "common/math/polygon2d.h"
#include "src/planning/common/decision/decision.h"
#include "st_graph_point.h"

namespace neodrive {
namespace planning {

// TODO(wyc): the destructor of base class Polygon2d should be virtual
class STGraphBoundary final : public Polygon2d {
 public:
  enum class BoundaryType {
    STOP = 1,
    FOLLOW,
    YIELD,
    OVERTAKE,
    SIDEPASSFOLLOW,
    SIDEPASSLEAD,
    NUDGE,
    RETROGRADE,
    YELLOWLIGHT,
    KEEPCLEAR,
    IGNORE,
    CRUISE_ROAD,
    UNKNOWN
  };

  STGraphBoundary(const std::vector<STPoint>& points);
  STGraphBoundary(std::vector<Vec2d>& points);

  virtual ~STGraphBoundary();

  bool is_empty() const;
  bool is_point_in_boundary(const STGraphPoint& st_graph_point) const;
  bool is_point_in_boundary(const STPoint& st_point) const;

  const Vec2d point(const std::size_t& index) const;
  const std::vector<Vec2d>& points() const;

  int32_t id() const;
  BoundaryType boundary_type() const;
  double characteristic_length() const;
  Decision get_decision() const;
  Decision* mutable_decision();
  bool is_virtual() const;
  bool is_static() const;
  double get_speed() const;
  double probability() const;

  void set_id(const int32_t& id);
  void set_boundary_type(const BoundaryType& boundary_type);
  void set_characteristic_length(const double& characteristic_length);
  void set_decision(const Decision& decision);
  void set_virtual(const bool& is_virtual);
  void set_static(const bool& is_static);
  void set_speed(const double& speed);
  void set_probability(const double& probability);

  bool get_boundary_s_range_by_time(const double& curr_time, double* s_upper,
                                    double* s_lower);
  bool get_boundary_time_scope(double* start_t, double* end_t) const;

  double min_s() const;
  double max_s() const;
  double min_t() const;
  double max_t() const;

  const std::vector<STPoint>& upper_points() const { return upper_points_; }
  const std::vector<STPoint>& lower_points() const { return lower_points_; }
  const STPoint& bottom_left_point() const {
    if (upper_points_.empty()) {
      LOG_ERROR("StGraphBoundary does not have upper_points.");
    }
    return upper_points_.front();
  }
  const STPoint& bottom_right_point() const {
    if (lower_points_.empty()) {
      LOG_ERROR("StGraphBoundary does not have lower_points.");
    }
    return lower_points_.back();
  }
  const double get_low_speed() const {
    if (lower_points_.size() < 2) return 0.0;
    double t_diff = lower_points_.back().t() - lower_points_.front().t();
    if (std::fabs(t_diff) < 1.0e-2) return 0.0;
    return (lower_points_.back().s() - lower_points_.front().s()) / t_diff;
  }
  const double get_begin_low_speed() const { return get_low_speed(); }
  void set_absolute_speed(const double& v) { absolute_speed_ = v; }
  const double get_absolute_speed() const { return absolute_speed_; }

  virtual Json::Value to_json() const;

 private:
  bool get_boundary_s_range_from_cache(const double& curr_time, double* s_upper,
                                       double* s_lower);
  void insert_boundary_s_range_to_cache(const double& curr_time,
                                        double& s_upper, double& s_lower);
  int get_cache_key(const double& curr_time) const;

 private:
  int id_ = 0;
  BoundaryType boundary_type_ = BoundaryType::UNKNOWN;
  double characteristic_length_ = 5.0;
  Decision decision_;
  bool virtual_ = false;
  bool static_ = false;
  double speed_ = 0.0;
  double probability_ = 1.0;
  double s_high_limit_ = 200.0;

  std::unordered_map<int, std::pair<double, double>> st_s_boundary_cache_;
  std::vector<STPoint> upper_points_{};
  std::vector<STPoint> lower_points_{};
  double absolute_speed_{0.0};
};

}  // namespace planning
}  // namespace neodrive
