#pragma once

#include "common/macros.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "hdmap/hdmap.h"
#include "map_application/map_application.h"
#include "map_data/map_data.h"
#include "math/common/geometry.h"
#include "reference_line/reference_point.h"
#include "semantic_map/semantic_map.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/task/task_info.h"

namespace neodrive {
namespace planning {

constexpr double kObjectContainPointDistanceThreshold = 0.5;
using LaneObjectsInfo = std::vector<std::tuple<uint64_t, double, double>>;
class PlanningMap final {
 public:
  ~PlanningMap() = default;
  void InitSharedMemory();

  std::string GetHashIdString(const uint64_t hash);

  // given (x,y,heading), find out lane and get (s,l) relative to the lane
  // without lane_ids
  bool GetIdSInLane(const double x, const double y, const double heading,
                    uint64_t &id, double &s);
  bool GetSInLane(const double x, const double y, const double heading,
                  double &s);
  bool GetLInLane(const double x, const double y, const double heading,
                  double &l);

  // given (x,y,heading), get the paras
  bool GetPointLeftBound(const double x, const double y, const double heading,
                         double &bound);
  bool GetPointRightBound(const double x, const double y, const double heading,
                          double &bound);
  /// Get point of lane left bound, this will first check if the point is onlane
  /// and then get the bound distance to the point
  /// @param x Point x
  /// @param y Point y
  /// @param lane Id of lane
  /// @return bound Distance between the lane and the lane left bound
  /// @return reverse_bound Distance to Neighbr reverse road
  bool GetPointLaneLeftRoadBound(const double x, const double y,
                                 const uint64_t lane, double &bound,
                                 double &reverse_bound);
  /// Get point of lane right bound, this will first check if the point is
  /// onlane and then get the bound distance to the point
  /// @param x Point x
  /// @param y Point y
  /// @param lane Id of lane
  /// @return bound Distance between the lane and the lane right bound
  bool GetPointLaneRightRoadBound(const double x, const double y,
                                  const uint64_t &lane, double &bound);
  bool GetPointLeftRoadBound(const double x, const double y,
                             const double heading, double &bound);
  bool GetPointRightRoadBound(const double x, const double y,
                              const double heading, double &bound);

  // given prefered lanes, get s,l
  bool GetIdSInLane(const double x, const double y, const double heading,
                    const std::vector<uint64_t> &lane_ids, uint64_t &id,
                    double &s);
  bool GetIdSInLane(const std::vector<uint64_t> &lane_ids,
                    ReferencePoint &point);
  bool GetSInLane(const double x, const double y, const double heading,
                  const std::vector<uint64_t> &lane_ids, double &s);
  bool GetLInLane(const double x, const double y, const double heading,
                  const std::vector<uint64_t> &lane_ids, double &l);
  /// Get lane left and right width with distance
  /// @param lane lane id
  /// @param s distance from lane start
  /// @return left and right width at the distance
  std::pair<double, double> GetLaneDistanceWidth(const uint64_t &lane,
                                                 const double s);

  bool GetPointLeftBound(const double x, const double y, const double heading,
                         const std::vector<uint64_t> &lane_ids, double &bound);
  bool GetPointRightBound(const double x, const double y, const double heading,
                          const std::vector<uint64_t> &lane_ids, double &bound);
  bool GetPointLeftRoadBound(const double x, const double y,
                             const double heading,
                             const std::vector<uint64_t> &lane_ids,
                             double &bound);
  bool GetPointRightRoadBound(const double x, const double y,
                              const double heading,
                              const std::vector<uint64_t> &lane_ids,
                              double &bound);

  void BorrowLeftLaneBoundAggressive(ReferencePoint &point_utm,
                                     ReferencePoint &point_odometry);

  void BorrowRightLaneBoundAggressive(ReferencePoint &point_utm,
                                      ReferencePoint &point_odometry);

  // given lane id(string), get info relative to the lane
  bool GetLengthOfLane(const uint64_t id, double &length);

  bool GetWidthOfLane(const uint64_t id, double &width);

  bool GetSpeedLimitOfLane(const uint64_t id, double &speed_limit);

  bool GetLaneType(const uint64_t lane_id,
                   neodrive::global::hdmap::Lane_LaneType *lane_type);
  bool GetLeftLanes(const uint64_t id, std::vector<uint64_t> &lanes);

  bool GetRightLanes(const uint64_t id, std::vector<uint64_t> &lanes);

  bool GetPredecessorLanes(const uint64_t id, std::vector<uint64_t> &lanes);

  bool GetSuccessorLanes(const uint64_t id, std::vector<uint64_t> &lanes);

  // given lane id(string) and s, get point from the lane
  bool GetLanePointWithDistance(const uint64_t id, const double s,
                                ReferencePoint &point);

  // given (x,y, radius, heading, heading_diff), get nearest lane_id(string)
  bool GetNearestLaneWithHeading(const double x, const double y,
                                 const double radius, const double heading,
                                 const double heading_diff,
                                 uint64_t &best_matched_lane,
                                 double &heading_deviation, double &offset);

  bool GetNearestLaneWithHeading(const double x, const double y,
                                 const double radius, const double heading,
                                 const double heading_diff,
                                 cyberverse::LaneInfoConstPtr &lane);

  // given lane (id,x,y), get heading
  bool GetHeadingWithLane(const uint64_t id, double &heading);

  // given lane (id,x,y), get (s,l) relative to the lane
  bool GetSLWithLane(const uint64_t id, const double x, const double y,
                     double &s, double &l);

  bool GetPointSpeedLimit(const double x, const double y, const double heading,
                          double &speed_limit);
  double GetLaneSpeedLimitByType(cyberverse::LaneInfoConstPtr &lane);

  double GetLaneSpeedLimit(const uint64_t lane);
  bool GetLaneTurnType(const uint64_t lane_id,
                       Lane::TurningType *lane_turn_type);

  bool GetLaneTurnType(ReferencePoint &point);
  bool GetLaneMultipleType(const uint64_t lane_id,
                           std::vector<uint32_t> &multiple_lane_type);
  bool IsRightLane(const double x, const double y);
  /// Get Signal info by id
  /// @param id Id of signal
  /// @return Pointer of signal
  cyberverse::SignalInfoConstPtr GetSignalById(const uint64_t id);

  BoundaryEdgeType GetLaneLeftBoundaryType(const uint64_t lane);
  BoundaryEdgeType GetLaneRightBoundaryType(const uint64_t lane);
  bool GetLaneBoundaryType(ReferencePoint &point);
  /// Get all crosswalks on lane
  /// @param lane Lane id
  /// @return All crosswalks, each is in format: [id, start_s, end_s]
  LaneObjectsInfo GetLaneCrosswalks(const uint64_t lane);
  /// Get all speed_bumps on lane
  /// @param lane Lane id
  /// @return All results, each is in format: [id, start_s, end_s]
  LaneObjectsInfo GetLaneSpeedBumps(const uint64_t lane);
  /// Get all yield signs on lane
  /// @param lane Lane id
  /// @return All results, each is in format: [id, start_s, end_s]
  LaneObjectsInfo GetLaneYieldSigns(const uint64_t lane);
  /// Get all stop signs on lane
  /// @param lane Lane id
  /// @return All results, each is in format: [id, start_s, end_s]
  LaneObjectsInfo GetLaneStopSigns(const uint64_t lane);
  /// Get all signals on lane
  /// @param lane Lane id
  /// @return All results, each is in format: [id, start_s, end_s]
  LaneObjectsInfo GetLaneSignals(const uint64_t lane);
  /// Get all junctions on lane
  /// @param lane Lane id
  /// @return All results, each is in format: [id, start_s, end_s]
  LaneObjectsInfo GetLaneJunctions(const uint64_t lane);
  /// Get all clear areas on lane
  /// @param lane Lane id
  /// @return All results, each is in format: [id, start_s, end_s]
  LaneObjectsInfo GetLaneClearAreas(const uint64_t lane);
  /// Get all geo fences on lane
  /// @param lane Lane id
  /// @return All results, each is in format: [id, start_s, end_s]
  LaneObjectsInfo GetLaneGeoFences(const uint64_t lane);
  /// Get all barrier gate on lane
  /// @param lane Lane id
  /// @return All results, each is in format: [id, start_s, end_s]
  LaneObjectsInfo GetBarrierGates(const uint64_t lane);  // Not yet developed
  /// Get all lane overlaps on lane
  /// @param lane Lane id
  /// @return All results, each is in format: [id, start_s, end_s]
  LaneObjectsInfo GetLaneOverlapLanes(const uint64_t lane);
  /// Get all lanes at the left forward(same direction wight given) lanes
  /// @param lane_id Id of the lane
  /// @return A list of left forward lanes
  std::vector<uint64_t> GetLaneLeftForwardLanes(const uint64_t lane_id);
  /// Get all lanes at the left backward(reverse direction wight given) lanes
  /// @param lane_id Id of the lane
  /// @return A list of result lanes
  std::vector<uint64_t> GetLaneLeftBackwardLanes(const uint64_t lane_id);
  /// Get all lanes at the right forward(same direction wight given) lanes
  /// @param lane_id Id of the lane
  /// @return A list of right forward lanes
  std::vector<uint64_t> GetLaneRightForwardLanes(const uint64_t lane_id);
  /// Get all lanes at the right backward(reverse direction wight given) lanes
  /// @param lane_id Id of the lane
  /// @return A list of result lanes
  std::vector<uint64_t> GetLaneRightBackwardLanes(const uint64_t lane_id);
  /// Get road id which the lane is on
  /// @param lane_id Id of the lane
  /// @return Road id
  uint64_t GetLaneRoadId(const uint64_t lane_id);
  /// Check if the left boundary is accessible at the distance under the mode
  /// It's mainly for planning's lane borrowing decision
  /// @param lane Id of the lane
  /// @param s Distance of the lane
  /// @param mode Mode of decision
  /// @return Accessible or not
  bool IsLaneLeftDistanceAccessibleWithMode(const uint64_t lane, const double s,
                                            const int mode);
  /// Check if the right boundary is accessible at the distance under the mode
  /// It's mainly for planning's lane borrowing decision
  /// @param lane Id of the lane
  /// @param s Distance of the lane
  /// @param mode Mode of decision
  /// @return Accessible or not
  bool IsLaneRightDistanceAccessibleWithMode(const uint64_t lane,
                                             const double s, const int mode);

  bool IsPointInJunction(const double x, const double y, const double heading);
  bool IsPointInCrosswalk(const double x, const double y, const double heading);
  bool IsPointInSignal(const double x, const double y, const double heading);
  bool IsPointInStopSign(const double x, const double y, const double heading);
  bool IsPointInYieldSign(const double x, const double y, const double heading);
  bool IsPointInClearArea(const double x, const double y, const double heading);
  bool IsPointInSpeedBump(const double x, const double y, const double heading);

  void SetCurrRoutingLaneIds(const std::vector<uint64_t> &holders);

  // color: 0: white; 1: yellow
  bool GetPointLeftRightLaneColor(const ReferencePoint &point, int &left_color,
                                  int &right_color);

  bool IsLaneBelongToUturn(const uint64_t lane_id);

  bool GetJunctionCorners(const uint64_t lane_id,
                          std::vector<common::math::Vec2d> &junction_corners);

  bool GetJunctionId(const ReferencePoint &point, uint64_t &junction_id);

  bool GetJunction(const ReferencePoint &point,
                   cyberverse::JunctionInfoConstPtr &junction_ptr);

  bool GetJunctionById(const uint64_t junction_id,
                       cyberverse::JunctionInfoConstPtr &junction_ptr);

  bool GetParkingSpace(const Vec2d &point,
                       cyberverse::ParkingSpaceInfoConstPtr &park_space_ptr);

  bool GetParkingSpaceById(
      const uint64_t parking_id,
      cyberverse::ParkingSpaceInfoConstPtr &park_space_ptr);

  bool GetPortOdd(const Vec2d &point,
                  cyberverse::DrivingZoneInfoConstPtr &driving_zone_info_ptr);

  bool IsInPortOdd(const Vec2d &point);

  bool IsNearJunction(const double x, const double y, const double radius);

  bool GetNearestLanes(
      const double x, const double y, double radius,
      std::vector<cyberverse::LaneInfoConstPtr> &nearest_lanes);

  bool IsJunctionWithinRadiusMetersAhead(
      const double x, const double y, const double heading, const double radius,
      double &distance, cyberverse::JunctionInfoConstPtr &junction_ptr);

  bool GetNearestLeftLane(uint64_t lane_id, const common::math::Vec2d &point,
                          uint64_t &res);

  bool GetNearestRightLane(uint64_t lane_id, const common::math::Vec2d &point,
                           uint64_t &res);

  bool GetInterp1PointInLane(uint64_t lane_id, double s,
                             common::math::Vec2d &res);

  bool GetInterp1PointInLane(uint64_t lane_id, std::array<double, 2> s,
                             common::math::Vec2d &res);

  bool GetInterp1PointInLane(uint64_t ego_lid, uint64_t merging_lid,
                             bool is_left, math::AD2 &near_pt,
                             math::AD2 &rear_pt, bool is_near,
                             double near_s = -1.0, double rear_s = -1.0);

  bool GetSamplePointFromConnection(const TrafficConflictZoneContext &czContext,
                                    double s, common::math::Vec2d &res);

  bool GetRSInLane(const MergingStopLineInfo &merging, double &s,
                   const common::math::Vec2d &res);

  bool GetRSInLane(const MeetingLaneContext &meeting, double &s,
                   const common::math::Vec2d &res);

  bool GetResDistanceInLane(const MeetingLaneContext &meeting,
                            const uint64_t &id, double &rd, const Vec2d &res);

  bool GetConflictZone(TaskInfo &task_info, const Vec2d &egoPt);

  bool IsLeftLane(uint64_t lane_id);
  bool IsRightLane(uint64_t lane_id);

  bool GetLaneDividers(const uint64_t &lane_id, const double &s,
                       std::map<double, DividerFeature> &left_divider,
                       std::map<double, DividerFeature> &right_divider) const;

  bool GetNeighborReverseRoadInfo(
      const uint64_t lane_id,
      std::vector<std::pair<uint64_t, std::vector<BoundaryEdgeType>>>
          &reverse_info);

  uint32_t GetDividerType(const uint64_t lane_id, const bool is_left);

 private:
  bool LoadMap(const uint64_t &map_path);

  void SetLocalPoint(const double x, const double y, const double heading);

  cyberverse::LaneInfoConstPtr GetLocalLane();

  void SetLocalPoint(const double x, const double y, const double heading,
                     const std::vector<uint64_t> &lane_ids);

  cyberverse::LaneInfoConstPtr GetLocalLane(
      const std::vector<uint64_t> &lane_ids);

  // mode : 0-dashed; 1: white-solid; 2: yellow- solid; 3: double-yellow-solid
  bool CanCrossLane(const uint32_t &type, const uint32_t &color, const int mode,
                    bool &is_cross);
  bool GetLaneSignal(const uint64_t &lane_id, const double &start_s,
                     const double &end_s,
                     std::multimap<double, cyberverse::SignalInfoConstPtr>
                         &signal_in_range) const;
  bool GetLaneNeighbors(const uint64_t &lane_id,
                        std::vector<uint64_t> &left_forwards_lanes,
                        std::vector<uint64_t> &right_forwards_lanes,
                        std::vector<uint64_t> &left_backwards_lanes,
                        std::vector<uint64_t> &right_backwards_lanes) const;

 private:
  constexpr static double kRadius = 10.0;         // in meter
  constexpr static double kHeadingDiff = M_PI_2;  // in rad
  constexpr static double kMaxJunctionRadius = 1000.0;
  std::array<std::string, 9> typeString{"Culdesac",   "Straight", "Diverging",
                                        "Merging",    "Mixed",    "Unknown",
                                        "NDiverging", "NMerging", "NMixed"};

  struct Point2D {
    Eigen::Vector2d point;
    double heading;
    Point2D() : point(0, 0), heading(0) {}
    Point2D(double _x, double _y, double _heading)
        : point(_x, _y), heading(_heading) {}
  };

  Point2D local_point_;
  cyberverse::LaneInfoConstPtr local_lane_;

  std::array<
      std::pair<uint64_t, std::map<MeetingLaneContext, ConflictLaneContext>>, 2>
      junction_info_{{{0, {}}, {0, {}}}};
  std::array<std::pair<uint64_t, std::set<MergingStopLineInfo>>, 2>
      connection_merging_geoinfo_{{{0, {}}, {0, {}}}};

  // do not use this unless you know it
  std::vector<uint64_t> curr_routing_lane_ids_;
  cyberverse::HDMap *hdmap_;
  DECLARE_SINGLETON(PlanningMap);
};

}  // namespace planning
}  // namespace neodrive
