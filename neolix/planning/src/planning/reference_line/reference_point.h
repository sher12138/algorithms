#pragma once

#include "common/math/vec2d.h"
#include "hdmap/hdmap.h"
#include "map_application/map_application.h"
#include "map_data/map_data.h"
#include "semantic_map/semantic_map.h"
#include "src/planning/common/planning_types.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

using namespace ::autobot::cyberverse;
using neodrive::global::hdmap::Lane_LaneDirection_IsValid;

enum TurnSignalType {
  TST_STRAIGHT = 0,
  TST_LEFT = 1,
  TST_RIGHT = 2,
  TST_UTURN = 3
};

struct LaneMeetingRanges {
  using Ptr = std::shared_ptr<LaneMeetingRanges>;
  using ConstPtr = std::shared_ptr<const LaneMeetingRanges>;
  struct LaneSegment {
    uint64_t id{0};
    double start_s{0};
    double end_s{0};
  };

  struct MeetingRange {
    LaneSegment curr{};
    std::vector<std::vector<LaneSegment>> prevs{};
    // std::vector<LaneSegment> next{};  // not used yet
  };

  std::vector<MeetingRange> meetings{};
};

class ReferencePoint {
 public:
  ReferencePoint();
  ~ReferencePoint();

  ReferencePoint(const ReferencePoint&) = default;

  ReferencePoint(const Vec2d& point, const double heading, const double kappa,
                 const double dkappa, const double right_bound,
                 const double left_bound);

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
  void set_z(const double z);
  void set_heading(const double heading);
  void set_pitch(const double pitch);
  void set_kappa(const double kappa);
  void set_dkappa(const double dkappa);
  void set_s(const double s);
  void set_left_bound(double left_bound);
  void set_right_bound(double right_bound);
  void set_left_lane_bound(double left_lane_bound);
  void set_right_lane_bound(double right_lane_bound);
  void set_left_road_bound(double left_road_bound);
  void set_left_reverse_road_bound(double left_reverse_bound);
  void set_right_road_bound(double right_road_bound);
  void set_left_lane_borrow(bool left_lane_borrow);
  void set_right_lane_borrow(bool right_lane_borrow);
  void set_is_left_lane(bool is_left_lane);
  void set_is_right_lane(bool is_right_lane);

  void set_speed_limit(double speed_limit);
  void set_is_in_junction(const bool is_in_junction);
  void set_is_in_crosswalk(const bool is_in_crosswalk);
  void set_is_in_signal(const bool is_in_signal);
  void set_is_in_stop_sign(const bool is_in_stop_sign);
  void set_is_in_yield_sign(const bool is_in_yield_sign);
  void set_is_in_clear_area(const bool is_in_clear_area);
  void set_is_in_speed_bump(const bool is_in_speed_bump);
  void set_is_in_barrier_gate(const bool is_in_barrier_gate);
  void set_left_signal();
  void set_right_signal();
  void set_no_signal();
  void set_uturn_signal();
  void set_seg_idx(const size_t seg_idx);
  void set_hd_map_lane_id(const uint64_t hd_map_lane_id);
  void set_hd_map_lane_s(const double& hd_map_lane_s);
  void set_hd_map_lane_l(const double& hd_map_lane_l);
  void copy_turn_signal(const ReferencePoint& other);
  void set_left_boundary_edge_type(const BoundaryEdgeType& type);
  void set_right_boundary_edge_type(const BoundaryEdgeType& type);
  void set_left_divider_feature(const std::vector<DividerFeature>& feature);
  void set_right_divider_feature(const std::vector<DividerFeature>& feature);
  void set_multiple_type(const std::vector<uint32_t>& type);

  void set_lane_meeting_ranges(LaneMeetingRanges::Ptr p);

  /*get para*/
  double x() const;
  double y() const;
  double z() const;
  double heading() const;
  double pitch() const;
  double kappa() const;
  double dkappa() const;
  double s() const;
  double left_bound() const;
  double right_bound() const;
  double left_lane_bound() const;
  double right_lane_bound() const;
  double left_road_bound() const;
  double left_reverse_road_bound() const;
  double right_road_bound() const;
  bool left_lane_borrow() const;
  bool right_lane_borrow() const;
  bool is_left_lane() const;
  bool is_right_lane() const;

  double speed_limit() const;
  bool is_in_junction() const;
  bool is_in_crosswalk() const;
  bool is_in_signal() const;
  bool is_in_stop_sign() const;
  bool is_in_yield_sign() const;
  bool is_in_clear_area() const;
  bool is_in_speed_bump() const;
  bool is_in_barrier_gate() const;
  bool is_left_signal() const;
  bool is_right_signal() const;
  bool is_uturn_signal() const;
  bool is_no_signal() const;
  size_t seg_idx() const;
  uint64_t hd_map_lane_id() const;
  double hd_map_lane_s() const;
  double hd_map_lane_l() const;

  TurnSignalType signal_type() const;
  const std::vector<uint32_t>& lane_multiple_type() const;
  double distance_sqr_to(Vec2d pt) const {
    return pt.distance_sqr_to({x_, y_});
  }

  /// lane type
  bool lane_type_is_city_driving() const;
  bool lane_type_is_parking() const;
  bool lane_type_is_shoulder() const;
  bool lane_type_is_bus_station() const;
  bool lane_type_is_biking() const;
  bool lane_type_is_city_driving_and_biking() const;
  bool lane_type_is_pure_city_driving() const;
  bool lane_type_is_left_turn_wating_zone() const;
  bool lane_type_is_indoor_lane() const;
  std::string get_lane_type_name() const;

  /// lane boundary edge type
  BoundaryEdgeType left_boundary_edge_type() const;
  BoundaryEdgeType right_boundary_edge_type() const;
  const std::vector<DividerFeature>& left_divider_feature() const;
  const std::vector<DividerFeature>& right_divider_feature() const;

  LaneMeetingRanges::ConstPtr lane_meeting_ranges() const;

 private:
  double x_ = 0.0;
  double y_ = 0.0;
  double z_ = 0.0;
  double heading_ = 0.0;
  double pitch_ = 0.0;
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
  double s_ = 0.0;
  double speed_limit_ = 3.0;

  bool is_in_junction_ = false;
  bool is_in_crosswalk_ = false;
  bool is_in_signal_ = false;
  bool is_in_stop_sign_ = false;
  bool is_in_yield_sign_ = false;
  bool is_in_clear_area_ = false;
  bool is_in_speed_bump_ = false;
  bool is_in_barrier_gate_ = false;

  bool is_left_lane_ = false;
  bool is_right_lane_ = false;

  size_t seg_idx_ = 0;
  uint64_t hd_map_lane_id_ = 0;
  double hd_map_lane_s_ = 0.0;
  double hd_map_lane_l_ = 0.0;
  TurnSignalType turn_signal_type_ = TST_STRAIGHT;
  BoundaryEdgeType left_boundary_edge_type_ = BoundaryEdgeType::DEFAULT;
  BoundaryEdgeType right_boundary_edge_type_ = BoundaryEdgeType::DEFAULT;
  std::vector<DividerFeature> left_divider_feature_{};
  std::vector<DividerFeature> right_divider_feature_{};
  std::vector<uint32_t> lane_multiple_type_{};

  LaneMeetingRanges::Ptr lane_meeting_ranges_{};

 private:
  double left_bound_ = 1.7;
  double right_bound_ = 1.7;
  double left_lane_bound_ = 1.7;
  double right_lane_bound_ = 1.7;
  double left_road_bound_ = 1.7;
  double left_reverse_road_bound_ = 1.7;
  double right_road_bound_ = 1.7;
  bool left_lane_borrow_ = false;
  bool right_lane_borrow_ = false;
};

// alias for interface using
using ReferencePointVec1d = std::vector<ReferencePoint>;
using ReferencePointVec2d = std::vector<ReferencePointVec1d>;
using ReferencePointVec3d = std::vector<ReferencePointVec2d>;

// for smoothing
struct EvaluatedPoint {
  ReferencePoint path_point{};
  double lateral_bound{0.};
  double longitudinal_bound{0.};

  // enforce smoother to strictly follow this reference point
  bool enforced{false};
};

}  // namespace planning
}  // namespace neodrive
