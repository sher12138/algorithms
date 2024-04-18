/**
 * @file reference_point.h
 **/

#pragma once

#include <json/json.h>
#include "common/math/vec2d.h"
#include "common/planning_map/planning_map.h"
#include "map_application/map_application.h"

namespace neodrive {
namespace planning_rl {

using namespace ::autobot::cyberverse;

enum TurnSignalType {
  TST_STRAIGHT = 0,
  TST_LEFT = 1,
  TST_RIGHT = 2,
  TST_UTURN = 3
};

class ReferencePoint {
 public:
  ReferencePoint();
  ~ReferencePoint();

  ReferencePoint(const Vec2d& point, const double heading, const double kappa,
                 const double dkappa, const double right_bound,
                 const double left_bound);

  ReferencePoint(const Vec2d& point, const double heading, const double kappa,
                 const Vec2d left_bound, const Vec2d right_bound);
  ReferencePoint(const Vec2d& point, const double heading, const double kappa,
                 const Vec2d left_bound, const Vec2d right_bound,
                 const double s, const std::string& hd_map_lane_id);

  ReferencePoint(const Vec2d& point, const double heading);

  ReferencePoint(const Vec2d& point, const double heading, const double kappa,
                 const double dkappa);

  ReferencePoint(const Vec2d& point, const double heading, const double kappa,
                 const double dkappa, const double s, const double right_bound,
                 const double left_bound);
  /*functions*/
  /*set para*/
  void set_x(const double x);
  void set_y(const double y);
  void set_heading(const double heading);
  void set_kappa(const double kappa);
  void set_dkappa(const double dkappa);
  void set_s(const double s);
  void set_left_bound(double left_bound);
  void set_right_bound(double right_bound);
  void set_left_bound_point(Vec2d point);
  void set_right_bound_point(Vec2d point);

  void set_left_crossable(const bool crossable);
  void set_right_crossable(const bool crossable);
  void set_left_road_bound(double left_road_bound);
  void set_right_road_bound(double right_road_bound);
  void set_left_road_bound_point(Vec2d point);
  void set_right_road_bound_point(Vec2d point);
  void set_left_lane_borrow(bool left_lane_borrow);
  void set_right_lane_borrow(bool right_lane_borrow);
  void set_is_driving_forward(const bool forward_flag);

  void set_offset_by_l(const double l);
  void set_speed_limit(double speed_limit);
  void set_is_in_junction(const bool is_in_junction);
  void set_is_in_crosswalk(const bool is_in_crosswalk);
  void set_is_in_signal(const bool is_in_signal);
  void set_is_in_stop_sign(const bool is_in_stop_sign);
  void set_is_in_yield_sign(const bool is_in_yield_sign);
  void set_is_in_clear_area(const bool is_in_clear_area);
  void set_is_in_speed_bump(const bool is_in_speed_bump);
  void set_is_left_merge(const bool is_left_merge);
  void set_is_right_merge(const bool is_right_merge);
  void set_is_u_turn(const bool is_u_turn);
  void set_is_self_reverse_lane(const bool is_self_reverse_lane);
  void set_left_signal();
  void set_right_signal();
  void set_no_signal();
  void set_uturn_signal();
  void set_hd_map_lane_id(const std::string& hd_map_lane_id);
  void set_hd_map_lane_s(const double& hd_map_lane_s);
  void set_hd_map_lane_l(const double& hd_map_lane_l);
  void copy_turn_signal(const ReferencePoint& other);
  void set_index_of_whole_line(const std::size_t index);
  void set_left_boundary_edge_type(const BoundaryEdgeType& type);
  void set_right_boundary_edge_type(const BoundaryEdgeType& type);

  /*get para*/
  double x() const;
  double y() const;
  double heading() const;
  double kappa() const;
  double dkappa() const;
  double s() const;
  double left_bound() const;
  double right_bound() const;
  Vec2d left_bound_point() const;
  Vec2d right_bound_point() const;
  bool left_crossable() const;
  bool right_crossable() const;
  double left_road_bound() const;
  double right_road_bound() const;
  Vec2d left_road_bound_point() const;
  Vec2d right_road_bound_point() const;
  bool left_lane_borrow() const;
  bool right_lane_borrow() const;
  bool is_driving_forward() const;

  double offset_l() const;
  double speed_limit() const;
  bool is_in_junction() const;
  bool is_in_crosswalk() const;
  bool is_in_signal() const;
  bool is_in_stop_sign() const;
  bool is_in_yield_sign() const;
  bool is_in_clear_area() const;
  bool is_in_speed_bump() const;
  bool is_left_merge() const;
  bool is_right_merge() const;
  bool is_u_turn() const;
  bool is_self_reverse_lane() const;
  bool is_left_signal() const;
  bool is_right_signal() const;
  bool is_uturn_signal() const;
  bool is_no_signal() const;
  std::string hd_map_lane_id() const;
  double hd_map_lane_s() const;
  double hd_map_lane_l() const;

  double ReferencePointDistance2D(const Vec2d point);

  TurnSignalType signal_type() const;
  std::size_t index_of_whole_line() const;

  BoundaryEdgeType left_boundary_edge_type() const;
  BoundaryEdgeType right_boundary_edge_type() const;

  Json::Value to_json() const;

 private:
  double x_ = 0.0;
  double y_ = 0.0;
  double heading_ = 0.0;
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
  double s_ = 0.0;
  // boundary relative to the reference point
  // left up, right low
  double left_bound_ = 1.7;
  double right_bound_ = 1.7;
  Vec2d left_bound_point_{0, 0};
  Vec2d right_bound_point_{0, 0};

  bool left_crossable_ = false;
  bool right_crossable_ = false;
  double left_road_bound_ = 1.7;
  double right_road_bound_ = 1.7;
  Vec2d left_road_bound_point_{0, 0};
  Vec2d right_road_bound_point_{0, 0};
  bool left_lane_borrow_ = false;
  bool right_lane_borrow_ = false;
  // true-forward; false-backward
  bool is_driving_forward_ = true;

  double offset_l_ = 0.0;

  double speed_limit_ = 3.0;
  bool is_in_junction_ = false;
  bool is_in_crosswalk_ = false;
  bool is_in_signal_ = false;
  bool is_in_stop_sign_ = false;
  bool is_in_yield_sign_ = false;
  bool is_in_clear_area_ = false;
  bool is_in_speed_bump_ = false;

  bool is_left_merge_ = false;
  bool is_right_merge_ = false;
  bool is_u_turn_ = false;
  bool is_self_reverse_lane_ = false;
  std::string hd_map_lane_id_ = "";
  double hd_map_lane_s_ = 0.0;
  double hd_map_lane_l_ = 0.0;
  std::size_t index_of_whole_line_ = 0;
  TurnSignalType turn_signal_type_ = TST_STRAIGHT;
  BoundaryEdgeType left_boundary_edge_type_ = BoundaryEdgeType::DEFAULT;
  BoundaryEdgeType right_boundary_edge_type_ = BoundaryEdgeType::DEFAULT;
};

// alias for interface using
using ReferencePointVec1d = std::vector<ReferencePoint>;
using ReferencePointVec2d = std::vector<ReferencePointVec1d>;
using ReferencePointVec3d = std::vector<ReferencePointVec2d>;
// for smoothing
struct EvaluatedPoint {
  ReferencePoint path_point;
  double lateral_bound = 0.0;
  double longitudinal_bound = 0.0;

  // enforce smoother to strictly follow this reference point
  bool enforced = false;
  // TODO: useless explicit
  explicit EvaluatedPoint()
      : lateral_bound(0.0), longitudinal_bound(0.0), enforced(false) {}
  explicit EvaluatedPoint(ReferencePoint point, double lateral_bound,
                          double longi_bound, bool enforced)
      : path_point(point),
        lateral_bound(lateral_bound),
        longitudinal_bound(longi_bound),
        enforced(enforced) {}
};

}  // namespace planning_rl
}  // namespace neodrive
