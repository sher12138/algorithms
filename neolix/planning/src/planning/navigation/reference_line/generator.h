#pragma once

#include <queue>

#include "common/navigation_context.h"
#include "common/navigation_types.h"
#include "common/trajectory/trajectory_point.h"
#include "src/planning/common/planning_types.h"

namespace neodrive {
namespace planning {

/// @class Generate cruise reference line
class Generator {
 public:
  struct RefLaneSegment {
    uint64_t id{0};
    double start_s{0.};
    double end_s{0.};

    cyberverse::LaneInfoConstPtr lane_ptr{nullptr};

    std::vector<uint32_t> lane_multiple_type{};
    Lane::TurningType turn_type{Lane::TurningType::NO_TURN};
    BoundaryEdgeType left_bound_type{BoundaryEdgeType::DEFAULT};
    BoundaryEdgeType right_bound_type{BoundaryEdgeType::DEFAULT};
    double speed_limit{0.};
    std::vector<std::pair<double, std::vector<DividerFeature>>> left_dividers{};
    std::vector<std::pair<double, std::vector<DividerFeature>>>
        right_dividers{};
    std::vector<std::pair<uint64_t, std::vector<BoundaryEdgeType>>>
        reverse_road_info{};

    std::vector<ReferenceLine::TrafficOverlap> crosswalks{};
    std::vector<ReferenceLine::TrafficOverlap> signals{};
    std::vector<ReferenceLine::TrafficOverlap> yield_signs{};
    std::vector<ReferenceLine::TrafficOverlap> stop_signs{};
    std::vector<ReferenceLine::TrafficOverlap> speed_bumps{};
    std::vector<ReferenceLine::TrafficOverlap> clear_areas{};
    std::vector<ReferenceLine::TrafficOverlap> junctions{};
    std::vector<ReferenceLine::TrafficOverlap> geo_fences{};
    std::vector<ReferenceLine::TrafficOverlap> barrier_gates{};
    std::vector<ReferenceLine::TrafficOverlap> parking_spaces{};
    LaneMeetingRanges::Ptr lane_meeting_ranges{};

    bool is_left_accessible{false};
    double left_forward_width{-1};
    double left_backward_width{-1};
    bool is_right_accessible{false};
    double right_forward_width{-1};
    double right_backward_width{-1};

    bool is_left_lane{false};
    bool is_right_lane{false};
    bool is_motorway{false};

    double accumulate_s{0.};  // lane's global start_s
    bool is_to_lane_change{false};
  };

 public:
  /// Default constructor
  Generator(NavigationContext* ctx) : ctx_(ctx) {}

 public:
  typedef std::vector<cyberverse::LaneInfoConstPtr> LaneSeqType;
  typedef std::vector<RefLaneSegment>::iterator RefSegIterType;
  typedef std::vector<ReferencePoint>::iterator RefPtIterType;
  typedef std::array<double, 2> AD2;
  // using global::hdmap::Lane;
  typedef global::hdmap::Lane::LaneTurn TurnType;

  bool BindLaneSeq(const LaneSeqType& route_lanes);

  bool Generate(const LaneSeqType& lane_seq, const TrajectoryPoint& init_point,
                ReferenceLinePtr* const ref_line);
  const ReferencePoint& GetDestinationPoint() const;

  bool GetCurrentLane(const TrajectoryPoint& ego_pose,
                      const LaneSeqType& route_lanes, bool empty_ref_line);
  bool InitCurrentS(ReferenceLinePtr* const ref_line,
                    const TrajectoryPoint& init_point,
                    const LaneSeqType& lane_seq);
  void GetLaneChangeEndPosition(const std::vector<RefLaneSegment>& tails,
                                const TurnType& turn,
                                const ReferenceLinePtr ref_line);

  /// Get distance from init point to destination point
  double GetDistanceToDestination() const;
  double GetDistanceToRefEnd(const ReferenceLinePtr refline) const;
  double GetCurrentS() const { return curr_init_s_; }
  Vec3d GetLaneChangeEndPoint() const { return lane_change_end_point_; }
  void UpdateSegInfo(const TrajectoryPoint& ego_pose, std::size_t& start_idx,
                     std::size_t& end_idx);
  void UpdateRefPts(size_t& seg_start, size_t& seg_end,
                    RefPtIterType& pt_start);
  void GeneratePtsFromSeg(size_t& seg_start, size_t& seg_end, double start_s,
                          std::vector<ReferencePoint>& pt_ret);
  void InitCurrRefPosition(const TrajectoryPoint& ego_pose);
  void CalcRefLinePoints(const TrajectoryPoint& init_point);

 private:
  std::vector<RefLaneSegment> GetPreviousLaneSegments(
      const RefLaneSegment& curr_seg, const double distance);
  std::vector<RefLaneSegment> GetSuccessorLaneSegments(
      const RefLaneSegment& curr_seg, const double distance);
  bool IsMotorway(uint64_t id);
  int GetLaneChangeDirection(uint64_t prev, uint64_t next, TurnType& turn);
  void ExtendRefLaneSegmentFrontAndTail(
      const double extend_front, const double extend_tail,
      std::vector<RefLaneSegment>* const ref_segs,
      std::vector<RefLaneSegment>* const tail_segs);
  RefLaneSegment BuildRefLaneSegment(const RefLaneSegment& lane_seg);

 private:
  static constexpr double kMaxGenerateLength = 250.0;
  static constexpr double kPointGap = 0.2;
  size_t dest_line_index_{0};
  size_t dest_point_index_{0};
  std::vector<RefLaneSegment> ref_lane_segments_{};
  std::vector<ReferencePoint> ref_line_points_{};
  std::vector<int> each_seg_pt_num_{};
  size_t curr_ref_index_{0};
  double curr_init_s_{0.};
  double last_init_s_{0.};
  double distance_to_destination_{0.0};
  ReferencePoint destination_point_{};
  uint64_t curr_lane_id_{0};

  double dist_to_ref_end_{0.};
  RefPtIterType curr_pt_{nullptr};
  RefPtIterType pt_start_{nullptr};
  int curr_pt_idx_{0};
  size_t seg_start_{0};
  size_t seg_end_{0};
  bool is_dest_on_ref_line_{false};
  Vec3d lane_change_end_point_{};
  bool detect_lane_change_{false};
  std::vector<RefLaneSegment> tail_segs_{};
  TurnType lane_change_turn_{global::hdmap::Lane::NO_TURN};

  NavigationContext* ctx_{nullptr};
};

}  // namespace planning
}  // namespace neodrive
