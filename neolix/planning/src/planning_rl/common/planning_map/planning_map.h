#pragma once

#include <string>
#include <vector>

#include "common/math/vec2d.h"
#include "map_data/map_data.h"
#include "neolix_log.h"
#include "semantic_map/semantic_map.h"

namespace neodrive {
namespace planning_rl {

using autobot::cyberverse::Divider;
using autobot::cyberverse::DividerFeature;
using autobot::cyberverse::Junction;
using autobot::cyberverse::Lane;
using autobot::cyberverse::LaneEndConnection;
using autobot::cyberverse::LaneLeftBackwardNeighbor;
using autobot::cyberverse::LaneLeftForwardNeighbor;
using autobot::cyberverse::LaneRightForwardNeighbor;
using autobot::cyberverse::MapLineSegment;
using autobot::cyberverse::RoadLink;
using autobot::cyberverse::StopSign;

class PlanningRLMap {
 public:
  PlanningRLMap(const PlanningRLMap& rhs) = delete;
  PlanningRLMap& operator=(const PlanningRLMap& rhs) = delete;

  bool LoadMap(const std::string& map_path);

 public:
  struct MapPoint {
    MapPoint() {}
    MapPoint(double x, double y, double z, double heading)
        : x(x), y(y), z(z), heading(heading) {}
    MapPoint(double x, double y, double z, double heading, double s,
             std::string lane_id)
        : x(x), y(y), z(z), heading(heading), s(s), lane_id(lane_id) {}
    MapPoint(double x, double y) : x(x), y(y), z(0.0), heading(0.0) {}
    MapPoint(double x, double y, double heading)
        : x(x), y(y), z(0.0), heading(heading) {}
    double x;
    double y;
    double z;
    double heading;
    double s;
    std::string lane_id;
  };
  using MapPoint2d = std::vector<PlanningRLMap::MapPoint>;
  using MapPoint3d = std::vector<PlanningRLMap::MapPoint2d>;

  static PlanningRLMap& Instance() {
    static PlanningRLMap instance;
    return instance;
  }

  const std::string& MapVersion() const;
  bool DumpText() const;

 public:
  bool IsPointOnLane(const PlanningRLMap::MapPoint& point,
                     const std::string& lane_id);
  Lane::ConstPtr FindLaneById(const std::string& id);
  std::string FindLaneByPoint(const PlanningRLMap::MapPoint& point,
                              const double angle_diff_threshold);
  static PlanningRLMap::MapPoint2d ConvertToMapPoints(
      std::vector<autobot::cyberverse::MapPoint>& origin_map_points);
  static PlanningRLMap::MapPoint2d SelectMapPointsBys(
      PlanningRLMap::MapPoint2d& origin_map_points, double start_s,
      double end_s);
  static bool CalculateMapPointsS(PlanningRLMap::MapPoint2d& map_points);
  bool GetRoadBoundPointsByLaneId(
      const std::string& id, PlanningRLMap::MapPoint2d& left_bound_points,
      PlanningRLMap::MapPoint2d& right_bound_points);
  bool GetSpeedLimitOfLane(const std::string& id, double& speed_limit);
  bool GetLeftLanes(const std::string& id, std::vector<std::string>& lanes);
  bool GetRightLanes(const std::string& id, std::vector<std::string>& lanes);
  bool GetPredecessorLanes(const std::string& id,
                           std::vector<std::string>& lanes);
  bool GetSuccessorLanes(const std::string& id,
                         std::vector<std::string>& lanes);
  size_t find_closest_map_points_index(
      const double x, const double y,
      const PlanningRLMap::MapPoint2d& map_points);
  size_t find_closest_map_points_index(
      const PlanningRLMap::MapPoint center_point,
      const PlanningRLMap::MapPoint2d& map_points);
  static Vec2d ConverMapPointToVec2d(PlanningRLMap::MapPoint map_point);

 private:
  //!
  //! \brief Constructor
  //!
  PlanningRLMap();

 private:
  autobot::cyberverse::MapData::Ptr map_data_;
  autobot::cyberverse::SemanticMap semantic_map_;
  autobot::cyberverse::TopologicalMap topological_map_;
  autobot::cyberverse::SemanticMapIndex semantic_map_index_;
};

}  // namespace planning_rl
}  // namespace neodrive