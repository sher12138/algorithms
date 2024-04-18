/// @file Define the reference line interface
#pragma once

#include <memory>

#include "common/boundary.h"
#include "common/macros.h"
#include "common/math/polygon2d.h"
#include "common/path/sl_point.h"
#include "hdmap/hdmap.h"
#include "reference_line/reference_point.h"

namespace neodrive {
namespace planning {

class ReferencelineKdTree;
/// @calss Reference line class
class ReferenceLine {
 public:
  /// @struct Define the overlap info on reference line
  struct TrafficOverlap {
    uint64_t object_id{};
    double start_s{0.};
    double end_s{0.};
  };
  using JunctionList =
      std::vector<std::pair<cyberverse::JunctionInfoConstPtr, TrafficOverlap>>;

 public:
  /// Must construct with reference points size larger than 1
  /// @param ref_points Reference points of the line
  /// @param overlaps Overlaps of the line in order
  /// @param anchor_xy Cartesian position of vehicle
  /// @param anchor_s Frenet position of vehicle
  /// @return Pointer of the reference line
  void CreateFrom(const std::vector<ReferencePoint>& ref_points,
                  const std::array<std::vector<TrafficOverlap>, 10>& overlaps,
                  const Vec2d& anchor_xy, const double anchor_s,
                  const std::size_t routing_seq_num);

  void operator=(const ReferenceLine& in_ref);

 public:
  /// Get nearest xy point on reference line with given point
  /// @param xy_pt (x, y) in cartesian coordination
  /// @param dis_tol Distance tolerance of the result
  /// @return ref_pt Reference point
  /// @return Success or not
  bool GetNearestRefPoint(const Vec2d& xy_pt, ReferencePoint* const ref_pt);

  bool GetfrontNearestRefPoint(
      const Vec2d& xy_pt, uint64_t curLaneID,
      std::vector<std::pair<std::pair<uint64_t, uint64_t>, bool>>& successor,
      ReferencePoint* const ref_pt) const;

  /// Get nearest xy point on reference line with given point and heading
  /// @param xy_pt (x, y) in cartesian coordination
  /// @param heading Heading of the xy_pt
  /// @param dis_tol Distance tolerance
  /// @param heading_tol Heading tolerance
  /// @return ref_pt Reference point
  /// @return Success or not
  bool GetNearestRefPointWithHeading(const Vec2d& xy_pt, const double heading,
                                     ReferencePoint* const ref_pt,
                                     const double heading_tol = M_PI_2);

  /// Get nearest sl point on reference line with given point
  /// @param sl_pt (s, l) in cartesian coordination
  /// @return ref_pt Reference point
  /// @return Success or not
  bool GetNearestRefPoint(const double s, ReferencePoint* ref_pt);

  /// Translate a cartesian point to frenet point
  /// @param xy_pt (x, y) in cartesian coordination
  /// @return sl_pt (s, l) in frenet
  /// @return success or not
  bool GetPointInFrenetFrame(const Vec2d& xy_pt, SLPoint* const sl_pt);

  /// Translate a cartesian point with last frame s to frenet point
  /// @param xy_pt (x, y) in cartesian coordination
  /// @param last_s last frame s of xy_pt
  /// @return sl_pt (s, l) in frenet
  /// @return success or not
  bool GetPointInFrenetFrameWithLastS(const Vec2d& xy_pt, const double last_s,
                                      SLPoint* const sl_pt);

  /// Translate a cartesian point to frenet point
  /// @param xy_pt (x, y) in cartesian coordination
  /// @param heading Heading of the xy_pt
  /// @param dis_tol Distance tolerance of the point
  /// @param heading_tol Heading tolerance of the point
  /// @return sl_pt (s, l) in frenet
  /// @return success or not
  bool GetPointInFrenetFrameWithHeading(const Vec2d& xy_pt,
                                        const double heading,
                                        SLPoint* const sl_pt);

  /// Translate a cartesian point to frenet point
  /// @param xy_pt (x, y) in cartesian coordination
  /// @param obs_center_pt obs center in cartesian coordination
  /// @return sl_pt (s, l) in frenet
  /// @return success or not
  bool GetObsCornerInFrenetFrame(const Vec2d& xy_pt, const Vec2d& obs_center_pt,
                                 SLPoint* const sl_pt);

  /// Translate a frenet point to cartesian point
  /// @param sl_pt (s, l) in coordination
  /// @return xy_pt (x, y) in frenet
  /// @return success or not
  bool GetPointInCartesianFrame(const SLPoint& sl_pt, Vec2d* const xy_pt);

  bool GetPolygonByAABox(const AABox2d& sl_box, Polygon2d* const polygon);

  /// Get points' [start_index, end_index] within [s, s + len] on reference line
  /// @param s Distance of the lin
  /// @param l Longitudinal distance of the line
  /// @return start_index
  /// @return end_index
  /// @return success or not
  bool GetStartEndIndexBySLength(const double s, const double len,
                                 size_t* const start_index,
                                 size_t* const end_index) const;

  /// Get length of the reference line
  /// @return Length of the reference line
  double GetLength() const;

  /// This is a trick for current(20221110) architecture, to change the
  //  lane bound where detouring. For the reason that kd-tree is built from a
  //  certain group of points, changing the coords of kd-tree's node will
  //  destroy it's balanceness. So it's recommanded to reconstruct
  //  the tree(with constructor) other than change the node.
  void SetIndexLaneBound(const size_t idx, double left_lane_bound,
                         double right_lane_bound);
  void SetIndexRoadBound(const size_t idx, double left_road_bound,
                         double right_road_bound);
  void SetIndexBound(const size_t idx, double left_bound, double right_bound);
  void SetIndexLaneBorrowFlag(const size_t idx, bool left_flag,
                              bool right_flag);

 private:
  void BindRefPts(const std::vector<ReferencePoint>& ref_points,
                  const std::array<std::vector<TrafficOverlap>, 10>& overlaps,
                  const Vec2d& anchor_xy, const double anchor_s,
                  const std::size_t routing_seq_num);

 public:
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<ReferencePoint>,
                                             ref_points);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(double, anchor_s);

  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<TrafficOverlap>,
                                             crosswalk_overlaps);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<TrafficOverlap>,
                                             signal_overlaps);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<TrafficOverlap>,
                                             yield_sign_overlaps);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<TrafficOverlap>,
                                             stop_sign_overlaps);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<TrafficOverlap>,
                                             speed_bump_overlaps);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<TrafficOverlap>,
                                             junction_overlaps);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<TrafficOverlap>,
                                             clearzone_overlaps);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<TrafficOverlap>,
                                             geo_fence_overlaps);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<TrafficOverlap>,
                                             barrier_gate_overlaps);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<TrafficOverlap>,
                                             parking_space_overlaps);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(JunctionList, junctions);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(size_t, routing_sequence_num);

 private:
  /// Node to construct 2d-tree
  /// Get nearest point
  void KdTreeNearest(const size_t root, const Vec2d& tar, const size_t dim,
                     size_t* const nearest, double* const min_err) const;

  /// Get all nodes in a range
  void KdTreeNearestsInRadius(const size_t root, const Vec2d& tar,
                              const size_t dim, const double sqr_radius,
                              std::vector<size_t>* ans) const;

  void RebuildKdtree();

  // /// Calculate min square distance from a point to a node(rectange)
  // double LowerSqrDisToPoint(const KdTreeNode& p1, const Vec2d& p2) const;

  // /// Calculate max square distance from a point to a node(rectange)
  // double UpperSqrDisToPoint(const KdTreeNode& p1, const Vec2d& p2) const;

 private:
  double anchor_s_{0.};
  std::vector<ReferencePoint> ref_points_{};

  std::vector<TrafficOverlap> crosswalk_overlaps_{};
  std::vector<TrafficOverlap> signal_overlaps_{};
  std::vector<TrafficOverlap> yield_sign_overlaps_{};
  std::vector<TrafficOverlap> stop_sign_overlaps_{};
  std::vector<TrafficOverlap> junction_overlaps_{};
  std::vector<TrafficOverlap> speed_bump_overlaps_{};
  std::vector<TrafficOverlap> clearzone_overlaps_{};
  std::vector<TrafficOverlap> geo_fence_overlaps_{};
  std::vector<TrafficOverlap> barrier_gate_overlaps_{};
  std::vector<TrafficOverlap> parking_space_overlaps_{};

  JunctionList junctions_{};
  size_t routing_sequence_num_{0};
  std::shared_ptr<ReferencelineKdTree> ref_kdtree_{nullptr};
};

using ReferenceLinePtr = std::shared_ptr<ReferenceLine>;

}  // namespace planning
}  // namespace neodrive
