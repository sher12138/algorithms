#include "reference_line/reference_point.h"

namespace neodrive {
namespace planning {

ReferencePoint::ReferencePoint() {}
ReferencePoint::ReferencePoint(const Vec2d& point, const double heading,
                               const double kappa, const double dkappa,
                               const double left_bound,
                               const double right_bound) {
  set_x(point.x());
  set_y(point.y());
  set_heading(heading);
  set_kappa(kappa);
  set_dkappa(dkappa);
  set_left_bound(left_bound);
  set_right_bound(right_bound);
  set_left_road_bound(left_bound);
  set_right_road_bound(right_bound);
}

ReferencePoint::ReferencePoint(const Vec2d& point, const double heading) {
  set_x(point.x());
  set_y(point.y());
  set_heading(heading);
}

ReferencePoint::ReferencePoint(const Vec2d& point, const double heading,
                               const double kappa, const double dkappa) {
  set_x(point.x());
  set_y(point.y());
  set_heading(heading);
  set_kappa(kappa);
  set_dkappa(dkappa);
}

ReferencePoint::ReferencePoint(const Vec2d& point, const double heading,
                               const double kappa, const double dkappa,
                               const double s, const double right_bound,
                               const double left_bound) {
  set_x(point.x());
  set_y(point.y());
  set_heading(heading);
  set_kappa(kappa);
  set_dkappa(dkappa);
  set_s(s);
  set_right_bound(right_bound);
  set_left_bound(left_bound);
}

ReferencePoint::~ReferencePoint() {}

void ReferencePoint::set_x(const double x) { x_ = x; }
void ReferencePoint::set_y(const double y) { y_ = y; }
void ReferencePoint::set_z(const double z) { z_ = z; }
void ReferencePoint::set_heading(const double heading) { heading_ = heading; }
void ReferencePoint::set_pitch(const double pitch) { pitch_ = pitch; }
void ReferencePoint::set_kappa(const double kappa) { kappa_ = kappa; }

void ReferencePoint::set_dkappa(const double dkappa) { dkappa_ = dkappa; }
void ReferencePoint::set_s(const double s) { s_ = s; }
void ReferencePoint::set_left_bound(double left_bound) {
  left_bound_ = left_bound;
}

void ReferencePoint::set_right_bound(double right_bound) {
  right_bound_ = right_bound;
}
void ReferencePoint::set_right_lane_bound(double right_lane_bound) {
  right_lane_bound_ = right_lane_bound;
}
void ReferencePoint::set_left_lane_bound(double left_lane_bound) {
  left_lane_bound_ = left_lane_bound;
}
void ReferencePoint::set_left_road_bound(double left_road_bound) {
  left_road_bound_ = left_road_bound;
}
void ReferencePoint::set_left_reverse_road_bound(double left_reverse_bound) {
  left_reverse_road_bound_ = left_reverse_bound;
}

void ReferencePoint::set_right_road_bound(double right_road_bound) {
  right_road_bound_ = right_road_bound;
}
void ReferencePoint::set_left_lane_borrow(bool left_lane_borrow) {
  left_lane_borrow_ = left_lane_borrow;
}
void ReferencePoint::set_right_lane_borrow(bool right_lane_borrow) {
  right_lane_borrow_ = right_lane_borrow;
}
void ReferencePoint::set_is_left_lane(bool is_left_lane) {
  is_left_lane_ = is_left_lane;
}
void ReferencePoint::set_is_right_lane(bool is_right_lane) {
  is_right_lane_ = is_right_lane;
}

void ReferencePoint::set_speed_limit(double speed_limit) {
  speed_limit_ = speed_limit;
}
void ReferencePoint::set_is_in_junction(const bool is_in_junction) {
  is_in_junction_ = is_in_junction;
}
void ReferencePoint::set_is_in_crosswalk(const bool is_in_crosswalk) {
  is_in_crosswalk_ = is_in_crosswalk;
}
void ReferencePoint::set_is_in_signal(const bool is_in_signal) {
  is_in_signal_ = is_in_signal;
}
void ReferencePoint::set_is_in_stop_sign(const bool is_in_stop_sign) {
  is_in_stop_sign_ = is_in_stop_sign;
}
void ReferencePoint::set_is_in_yield_sign(const bool is_in_yield_sign) {
  is_in_yield_sign_ = is_in_yield_sign;
}
void ReferencePoint::set_is_in_clear_area(const bool is_in_clear_area) {
  is_in_clear_area_ = is_in_clear_area;
}
void ReferencePoint::set_is_in_speed_bump(const bool is_in_speed_bump) {
  is_in_speed_bump_ = is_in_speed_bump;
}
void ReferencePoint::set_is_in_barrier_gate(const bool is_in_barrier_gate) {
  is_in_barrier_gate_ = is_in_barrier_gate;
}

void ReferencePoint::set_left_signal() { turn_signal_type_ = TST_LEFT; }

void ReferencePoint::set_right_signal() { turn_signal_type_ = TST_RIGHT; }

void ReferencePoint::set_no_signal() { turn_signal_type_ = TST_STRAIGHT; }
void ReferencePoint::set_uturn_signal() { turn_signal_type_ = TST_UTURN; }
void ReferencePoint::set_seg_idx(const size_t seg_idx) { seg_idx_ = seg_idx; }
void ReferencePoint::set_hd_map_lane_id(const uint64_t hd_map_lane_id) {
  hd_map_lane_id_ = hd_map_lane_id;
}
void ReferencePoint::set_hd_map_lane_s(const double& hd_map_lane_s) {
  hd_map_lane_s_ = hd_map_lane_s;
}
void ReferencePoint::set_hd_map_lane_l(const double& hd_map_lane_l) {
  hd_map_lane_l_ = hd_map_lane_l;
}
void ReferencePoint::copy_turn_signal(const ReferencePoint& other) {
  turn_signal_type_ = other.turn_signal_type_;
}
void ReferencePoint::set_left_boundary_edge_type(const BoundaryEdgeType& type) {
  left_boundary_edge_type_ = type;
}
void ReferencePoint::set_right_boundary_edge_type(
    const BoundaryEdgeType& type) {
  right_boundary_edge_type_ = type;
}
void ReferencePoint::set_left_divider_feature(
    const std::vector<DividerFeature>& feature) {
  left_divider_feature_ = feature;
}
void ReferencePoint::set_right_divider_feature(
    const std::vector<DividerFeature>& feature) {
  right_divider_feature_ = feature;
}

void ReferencePoint::set_lane_meeting_ranges(LaneMeetingRanges::Ptr p) {
  lane_meeting_ranges_ = p;
}

void ReferencePoint::set_multiple_type(const std::vector<uint32_t>& type) {
  lane_multiple_type_ = type;
}

/*get para*/
double ReferencePoint::x() const { return x_; }
double ReferencePoint::y() const { return y_; }
double ReferencePoint::z() const { return z_; }
double ReferencePoint::heading() const { return heading_; }
double ReferencePoint::pitch() const { return pitch_; }
double ReferencePoint::kappa() const { return kappa_; }
double ReferencePoint::dkappa() const { return dkappa_; }
double ReferencePoint::s() const { return s_; }

double ReferencePoint::left_bound() const { return std::max(0.0, left_bound_); }
double ReferencePoint::right_bound() const {
  return std::max(0.0, right_bound_);
}
double ReferencePoint::right_lane_bound() const { return right_lane_bound_; }
double ReferencePoint::left_lane_bound() const { return left_lane_bound_; }
double ReferencePoint::left_road_bound() const {
  return std::max(0.0, left_road_bound_);
}
double ReferencePoint::left_reverse_road_bound() const {
  return std::max(0.0, left_reverse_road_bound_);
}
double ReferencePoint::right_road_bound() const {
  return std::max(0.0, right_road_bound_);
}
bool ReferencePoint::left_lane_borrow() const { return left_lane_borrow_; }
bool ReferencePoint::right_lane_borrow() const { return right_lane_borrow_; }
bool ReferencePoint::is_left_lane() const { return is_left_lane_; }
bool ReferencePoint::is_right_lane() const { return is_right_lane_; }

double ReferencePoint::speed_limit() const { return speed_limit_; }
bool ReferencePoint::is_in_junction() const { return is_in_junction_; }
bool ReferencePoint::is_in_crosswalk() const { return is_in_crosswalk_; }
bool ReferencePoint::is_in_signal() const { return is_in_signal_; }
bool ReferencePoint::is_in_stop_sign() const { return is_in_stop_sign_; }
bool ReferencePoint::is_in_yield_sign() const { return is_in_yield_sign_; }
bool ReferencePoint::is_in_clear_area() const { return is_in_clear_area_; }
bool ReferencePoint::is_in_speed_bump() const { return is_in_speed_bump_; }
bool ReferencePoint::is_in_barrier_gate() const { return is_in_barrier_gate_; }
bool ReferencePoint::is_left_signal() const {
  return turn_signal_type_ == TST_LEFT;
}
bool ReferencePoint::is_right_signal() const {
  return turn_signal_type_ == TST_RIGHT;
}
bool ReferencePoint::is_uturn_signal() const {
  return turn_signal_type_ == TST_UTURN;
}
bool ReferencePoint::is_no_signal() const {
  return turn_signal_type_ == TST_STRAIGHT;
}

size_t ReferencePoint::seg_idx() const { return seg_idx_; }
uint64_t ReferencePoint::hd_map_lane_id() const { return hd_map_lane_id_; }
double ReferencePoint::hd_map_lane_s() const { return hd_map_lane_s_; }
double ReferencePoint::hd_map_lane_l() const { return hd_map_lane_l_; }
TurnSignalType ReferencePoint::signal_type() const { return turn_signal_type_; }
const std::vector<uint32_t>& ReferencePoint::lane_multiple_type() const {
  return lane_multiple_type_;
}

bool ReferencePoint::lane_type_is_city_driving() const {
  if (lane_multiple_type_.empty()) return false;
  for (const auto& type : lane_multiple_type_) {
    if (type == Lane_LaneType::Lane_LaneType_CITY_DRIVING) {
      return true;
    }
  }
  return false;
}
bool ReferencePoint::lane_type_is_parking() const {
  if (lane_multiple_type_.empty()) return false;
  for (const auto& type : lane_multiple_type_) {
    if (type == Lane_LaneType::Lane_LaneType_PARKING) {
      return true;
    }
  }
  return false;
}
bool ReferencePoint::lane_type_is_shoulder() const {
  if (lane_multiple_type_.empty()) return false;
  for (const auto& type : lane_multiple_type_) {
    if (type == Lane_LaneType::Lane_LaneType_SHOULDER) {
      return true;
    }
  }
  return false;
}

bool ReferencePoint::lane_type_is_bus_station() const {
  if (lane_multiple_type_.empty()) return false;
  for (const auto& type : lane_multiple_type_) {
    if (type == Lane_LaneType::Lane_LaneType_BUS_BAY_LANE) {
      return true;
    }
  }
  return false;
}
bool ReferencePoint::lane_type_is_biking() const {
  if (lane_multiple_type_.empty()) return false;
  for (const auto& type : lane_multiple_type_) {
    if (type == Lane_LaneType::Lane_LaneType_BIKING) {
      return true;
    }
  }
  return false;
}
bool ReferencePoint::lane_type_is_city_driving_and_biking() const {
  return lane_type_is_biking() && lane_type_is_city_driving();
}
bool ReferencePoint::lane_type_is_pure_city_driving() const {
  return !lane_type_is_biking() && lane_type_is_city_driving();
}
bool ReferencePoint::lane_type_is_left_turn_wating_zone() const {
  if (lane_multiple_type_.empty()) return false;
  for (const auto& type : lane_multiple_type_) {
    if (type == Lane_LaneType::Lane_LaneType_LEFT_TURN_WAITING_ZONE) {
      return true;
    }
  }
  return false;
}

bool ReferencePoint::lane_type_is_indoor_lane() const {
  if (lane_multiple_type_.empty()) return false;
  for (const auto& type : lane_multiple_type_) {
    if (type == Lane_LaneType::Lane_LaneType_INDOOR_LANE) {
      return true;
    }
  }
  return false;
}

std::string ReferencePoint::get_lane_type_name() const {
  if (lane_multiple_type_.empty()) return "";
  std::string name;
  for (const auto& type : lane_multiple_type_) {
    name = name + neodrive::global::hdmap::Lane_LaneType_Name(type) + " ";
  }
  return name;
}

BoundaryEdgeType ReferencePoint::left_boundary_edge_type() const {
  return left_boundary_edge_type_;
}
BoundaryEdgeType ReferencePoint::right_boundary_edge_type() const {
  return right_boundary_edge_type_;
}
const std::vector<DividerFeature>& ReferencePoint::left_divider_feature()
    const {
  return left_divider_feature_;
}
const std::vector<DividerFeature>& ReferencePoint::right_divider_feature()
    const {
  return right_divider_feature_;
}

LaneMeetingRanges::ConstPtr ReferencePoint::lane_meeting_ranges() const {
  return lane_meeting_ranges_;
}

}  // namespace planning
}  // namespace neodrive
