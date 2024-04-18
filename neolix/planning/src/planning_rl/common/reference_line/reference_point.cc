/**
 * @file reference_point.cpp
 **/

#include "reference_point.h"

namespace neodrive {
namespace planning_rl {

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

ReferencePoint::ReferencePoint(const Vec2d& point, const double heading,
                               const double kappa, const Vec2d left_bound,
                               const Vec2d right_bound) {
  set_x(point.x());
  set_y(point.y());
  set_heading(heading);
  set_kappa(kappa);
  set_left_bound_point(left_bound);
  set_right_bound_point(right_bound);
}

ReferencePoint::ReferencePoint(const Vec2d& point, const double heading,
                               const double kappa, const Vec2d left_bound,
                               const Vec2d right_bound, const double s,
                               const std::string& hd_map_lane_id) {
  set_x(point.x());
  set_y(point.y());
  set_heading(heading);
  set_kappa(kappa);
  set_left_bound_point(left_bound);
  set_right_bound_point(right_bound);
  set_s(s);
  set_hd_map_lane_id(hd_map_lane_id);
}

ReferencePoint::~ReferencePoint() {}

void ReferencePoint::set_x(const double x) { x_ = x; }
void ReferencePoint::set_y(const double y) { y_ = y; }
void ReferencePoint::set_heading(const double heading) { heading_ = heading; }
void ReferencePoint::set_kappa(const double kappa) { kappa_ = kappa; }

void ReferencePoint::set_dkappa(const double dkappa) { dkappa_ = dkappa; }
void ReferencePoint::set_s(const double s) { s_ = s; }
void ReferencePoint::set_left_bound(double left_bound) {
  left_bound_ = left_bound;
}

void ReferencePoint::set_right_bound(double right_bound) {
  right_bound_ = right_bound;
}

void ReferencePoint::set_left_crossable(const bool crossable) {
  left_crossable_ = crossable;
}
void ReferencePoint::set_left_bound_point(Vec2d point) {
  left_bound_point_ = point;
  set_left_bound(ReferencePointDistance2D(point));
}
void ReferencePoint::set_right_bound_point(Vec2d point) {
  right_bound_point_ = point;
  set_right_bound(ReferencePointDistance2D(point));
}

void ReferencePoint::set_right_crossable(const bool crossable) {
  right_crossable_ = crossable;
}
void ReferencePoint::set_left_road_bound(double left_road_bound) {
  left_road_bound_ = left_road_bound;
}

void ReferencePoint::set_right_road_bound(double right_road_bound) {
  right_road_bound_ = right_road_bound;
}

void ReferencePoint::set_left_road_bound_point(Vec2d point) {
  left_road_bound_point_ = point;
  set_left_road_bound(ReferencePointDistance2D(point));
}
void ReferencePoint::set_right_road_bound_point(Vec2d point) {
  right_road_bound_point_ = point;
  set_right_road_bound(ReferencePointDistance2D(point));
}
void ReferencePoint::set_left_lane_borrow(bool left_lane_borrow) {
  left_lane_borrow_ = left_lane_borrow;
}
void ReferencePoint::set_right_lane_borrow(bool right_lane_borrow) {
  right_lane_borrow_ = right_lane_borrow;
}
void ReferencePoint::set_is_driving_forward(const bool forward_flag) {
  is_driving_forward_ = forward_flag;
}

void ReferencePoint::set_offset_by_l(const double l) { offset_l_ = l; }
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
void ReferencePoint::set_is_left_merge(const bool is_left_merge) {
  is_left_merge_ = is_left_merge;
}
void ReferencePoint::set_is_right_merge(const bool is_right_merge) {
  is_right_merge_ = is_right_merge;
}
void ReferencePoint::set_is_u_turn(const bool is_u_turn) {
  is_u_turn_ = is_u_turn;
}
void ReferencePoint::set_is_self_reverse_lane(const bool is_self_reverse_lane) {
  is_self_reverse_lane_ = is_self_reverse_lane;
}

void ReferencePoint::set_left_signal() { turn_signal_type_ = TST_LEFT; }

void ReferencePoint::set_right_signal() { turn_signal_type_ = TST_RIGHT; }

void ReferencePoint::set_no_signal() { turn_signal_type_ = TST_STRAIGHT; }
void ReferencePoint::set_uturn_signal() { turn_signal_type_ = TST_UTURN; }
void ReferencePoint::set_hd_map_lane_id(const std::string& hd_map_lane_id) {
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
void ReferencePoint::set_index_of_whole_line(const std::size_t index) {
  index_of_whole_line_ = index;
}
void ReferencePoint::set_left_boundary_edge_type(const BoundaryEdgeType& type) {
  left_boundary_edge_type_ = type;
}
void ReferencePoint::set_right_boundary_edge_type(
    const BoundaryEdgeType& type) {
  right_boundary_edge_type_ = type;
}
/*get para*/
double ReferencePoint::x() const { return x_; }
double ReferencePoint::y() const { return y_; }
double ReferencePoint::heading() const { return heading_; }
double ReferencePoint::kappa() const { return kappa_; }
double ReferencePoint::dkappa() const { return dkappa_; }
double ReferencePoint::s() const { return s_; }
double ReferencePoint::left_bound() const {
  return std::max(0.0, left_bound_);  // - offset_l_
}
Vec2d ReferencePoint::left_bound_point() const { return left_bound_point_; }
Vec2d ReferencePoint::right_bound_point() const { return right_bound_point_; }

double ReferencePoint::right_bound() const {
  return std::max(0.0, right_bound_);  //+ offset_l_
}
bool ReferencePoint::left_crossable() const { return left_crossable_; }

bool ReferencePoint::right_crossable() const { return right_crossable_; }
double ReferencePoint::left_road_bound() const {
  return std::max(0.0, left_road_bound_);  //- offset_l_
}

double ReferencePoint::right_road_bound() const {
  return std::max(0.0, right_road_bound_);  //+ offset_l_
}

Vec2d ReferencePoint::left_road_bound_point() const {
  return left_road_bound_point_;
}
Vec2d ReferencePoint::right_road_bound_point() const {
  return right_road_bound_point_;
}

bool ReferencePoint::left_lane_borrow() const { return left_lane_borrow_; }
bool ReferencePoint::right_lane_borrow() const { return right_lane_borrow_; }
bool ReferencePoint::is_driving_forward() const { return is_driving_forward_; }

double ReferencePoint::offset_l() const { return offset_l_; }
double ReferencePoint::speed_limit() const { return speed_limit_; }
bool ReferencePoint::is_in_junction() const { return is_in_junction_; }
bool ReferencePoint::is_in_crosswalk() const { return is_in_crosswalk_; }
bool ReferencePoint::is_in_signal() const { return is_in_signal_; }
bool ReferencePoint::is_in_stop_sign() const { return is_in_stop_sign_; }
bool ReferencePoint::is_in_yield_sign() const { return is_in_yield_sign_; }
bool ReferencePoint::is_in_clear_area() const { return is_in_clear_area_; }
bool ReferencePoint::is_in_speed_bump() const { return is_in_speed_bump_; }
bool ReferencePoint::is_left_merge() const { return is_left_merge_; }

bool ReferencePoint::is_right_merge() const { return is_right_merge_; }
bool ReferencePoint::is_u_turn() const { return is_u_turn_; }

bool ReferencePoint::is_self_reverse_lane() const {
  return is_self_reverse_lane_;
}
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
std::string ReferencePoint::hd_map_lane_id() const { return hd_map_lane_id_; }
double ReferencePoint::hd_map_lane_s() const { return hd_map_lane_s_; }
double ReferencePoint::hd_map_lane_l() const { return hd_map_lane_l_; }

double ReferencePoint::ReferencePointDistance2D(const Vec2d point) {
  return sqrt(pow(point.x() - x_, 2) + pow(point.y() - y_, 2));
}

TurnSignalType ReferencePoint::signal_type() const { return turn_signal_type_; }
std::size_t ReferencePoint::index_of_whole_line() const {
  return index_of_whole_line_;
}
BoundaryEdgeType ReferencePoint::left_boundary_edge_type() const {
  return left_boundary_edge_type_;
}
BoundaryEdgeType ReferencePoint::right_boundary_edge_type() const {
  return right_boundary_edge_type_;
}

Json::Value ReferencePoint::to_json() const {
  Json::Value root;
  root["x"] = x();
  root["y"] = y();
  root["heading"] = heading();
  root["kappa"] = kappa();
  root["dkappa"] = dkappa();
  root["left_bound"] = left_bound();
  root["right_bound"] = right_bound();
  root["left_crossable"] = left_crossable();
  root["right_crossable"] = right_crossable();
  root["left_road_bound"] = left_road_bound();
  root["right_road_bound"] = right_road_bound();
  if (is_left_signal()) {
    root["turn_signal"] = "left";
  } else if (is_right_signal()) {
    root["turn_signal"] = "right";
  } else if (is_uturn_signal()) {
    root["turn_signal"] = "uturn";
  } else {
    root["turn_signal"] = "straight";
  }

  root["is_near_junction"] = is_in_junction_ ? "true" : "false";

  return root;
}

}  // namespace planning_rl
}  // namespace neodrive
