/**
 * @file reference_line.h
 * data struct define
 **/

#pragma once

#include <cmath>
#include "common/math/math_utils.h"
#include "common/math/sl_point.h"
#include "common/planning_map/planning_map.h"
#include "neolix_log.h"
#include "reference_point.h"

namespace neodrive {
namespace planning_rl {

// using TrafficOverlapVec1d = std::vector<TrafficOverlap>;

class ReferenceLine {
 public:
  ReferenceLine() = default;
  ~ReferenceLine() = default;
  ReferenceLine(const ReferencePointVec1d& reference_points);
  ReferenceLine(const ReferenceLine& line);
  ReferenceLine(const PlanningRLMap::MapPoint3d& map_points);

  PlanningRLMap::MapPoint3d pairs_map_points(
      const PlanningRLMap::MapPoint3d& map_points);
  std::vector<std::vector<double>> cal_map_points_curvature(
      const PlanningRLMap::MapPoint2d& map_points);
  void set_ref_points(const ReferencePointVec1d& ref_points);

  ReferencePointVec1d ComputeReferenceLineS(
      ReferencePointVec1d& refline_points);

  // get_closet_ref_point
  bool get_closest_reference_point(const Vec2d& xy_point,
                                   ReferencePoint& closet_ref);

  // void cartesian_to_frenet(const ReferencePoint& pt, const Vec2d& xy_point,
  //                          SLPoint& sl_pt);
  // bool frenet_to_cartesian(const ReferencePoint& pt, const SLPoint& sl_pt,
  //                          Vec2d& xy_point);

  ReferencePoint front_point() const;
  ReferencePoint back_point() const;
  Json::Value to_json() const;

  const ReferencePointVec1d& ref_points() const;
  ReferencePointVec1d* mutable_ref_points();

  void init();

  const ReferencePointVec1d& get_reference_points() const {
    return ref_points_;
  }

  int size() { return ref_points_.size(); }

  bool add_road_boundary_to_reference_points();

 private:
  double length_{0.0};
  ReferencePointVec1d ref_points_{};
  // CartesianFrenetConverter converter_;
};
// alias for interface using
using ReferenceLine2d = std::vector<ReferenceLine>;
using ReferenceLinePtr = std::shared_ptr<ReferenceLine>;

}  // namespace planning_rl
}  // namespace neodrive
