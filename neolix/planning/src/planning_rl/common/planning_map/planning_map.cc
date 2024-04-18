#include "planning_map.h"

// #include <algorithm>

#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "cyber.h"

namespace neodrive {
namespace planning_rl {

PlanningRLMap::PlanningRLMap() {
  auto& path = "/home/caros/adu_data/map/";
  // auto& path = "/home/caros/adu_data/map-yancheng2/";
  // auto& path = "/data16t/fuyihao/autobot/adu_data/map-yancheng2";
  if (!LoadMap(path)) {
    LOG_ERROR("LoadMap error: {}", path);
    exit(-1);
  };
}

bool PlanningRLMap::LoadMap(const std::string& map_path) {
  autobot::cyberverse::MapData::Ptr map_data;
  autobot::cyberverse::SemanticMap semantic_map;
  autobot::cyberverse::TopologicalMap topological_map;
  autobot::cyberverse::SemanticMapIndex semantic_map_index;

  // when doing unit test, the map has been loaded, if map_data_ is not cleared,
  // new map cannot be loaded.
  std::swap(map_data, map_data_);
  std::swap(semantic_map, semantic_map_);
  std::swap(topological_map, topological_map_);
  std::swap(semantic_map_index, semantic_map_index_);

  map_data_ = autobot::cyberverse::MapData::GetMapData(
      autobot::cyberverse::MapDataType::ADU, map_path);
  if (!map_data_->LoadBin()) {
    // LOG_ERROR("Can not load map from the path: {}", map_path);
    exit(-1);
  }

  map_data_->ParseMap(semantic_map_, topological_map_);
  semantic_map_index_ = semantic_map_.semantic_map_index();

  return true;
}

const std::string& PlanningRLMap::MapVersion() const {
  return semantic_map_.map_version();
}

bool PlanningRLMap::DumpText() const { return map_data_->DumpText(); }

Lane::ConstPtr PlanningRLMap::FindLaneById(const std::string& id) {
  Lane::ConstPtr lane = semantic_map_index_.Get<Lane>(id);
  return lane;
}

bool PlanningRLMap::IsPointOnLane(const PlanningRLMap::MapPoint& point,
                                  const std::string& lane_id) {
  Lane::ConstPtr lane = semantic_map_index_.Get<Lane>(lane_id);
  if (lane == nullptr) {
    return false;
  }

  return lane->IsOnLane({point.x, point.y, 0.0});
}

std::string PlanningRLMap::FindLaneByPoint(const PlanningRLMap::MapPoint& point,
                                           const double angle_diff_threshold) {
  std::vector<Lane::Ptr> lanes;
  const size_t lanes_size = semantic_map_index_.SearchWithHeading<Lane>(
      {point.x, point.y}, 0.1, point.heading, angle_diff_threshold, lanes);

  if (lanes_size == 0) {
    // LOG_WARN("No lanes around the obstacle.");
    return "";
  }

  // Delete neighbor lanes
  std::vector<Lane::Ptr> new_lanes;
  for (std::size_t i = 0; i < lanes_size; ++i) {
    if (IsPointOnLane(point, lanes[i]->id())) {
      new_lanes.push_back(lanes[i]);
    }
  }

  if (new_lanes.empty()) {
    // LOG_WARN("Obstacle locates outside the lanes.");
    return "";
  }

  double min_angle_diff = 2.0 * M_PI;
  int most_likely_lane_idx = 0;
  for (std::size_t i = 0; i < new_lanes.size(); ++i) {
    double nearest_point_heading =
        new_lanes[i]
            ->center_line()
            .Get2DNearestLineSegment({point.x, point.y})
            ->heading();
    double angle_diff =
        std::fabs(AngleDiff(point.heading, nearest_point_heading));
    if (angle_diff < min_angle_diff) {
      min_angle_diff = angle_diff;
      most_likely_lane_idx = i;
    }
  }

  return new_lanes[most_likely_lane_idx]->id();
}

PlanningRLMap::MapPoint2d PlanningRLMap::ConvertToMapPoints(
    std::vector<autobot::cyberverse::MapPoint>& origin_map_points) {
  PlanningRLMap::MapPoint2d map_points_local;
  if (origin_map_points.empty()) {
    return map_points_local;
  }
  for (size_t i = 0; i < origin_map_points.size(); i++) {
    PlanningRLMap::MapPoint tmp_map_point = {origin_map_points[i].x(),
                                             origin_map_points[i].y(),
                                             origin_map_points[i].z(), 0.0};
    map_points_local.push_back(tmp_map_point);
  }

  return map_points_local;
}
PlanningRLMap::MapPoint2d PlanningRLMap::SelectMapPointsBys(
    PlanningRLMap::MapPoint2d& origin_map_points, double start_s,
    double end_s) {
  double s = 0.0;
  PlanningRLMap::MapPoint2d new_origin_map_points;
  for (size_t i = 0; i < origin_map_points.size(); i++) {
    if (s >= start_s && s <= end_s) {
      new_origin_map_points.push_back(origin_map_points[i]);
    }
    if (i != origin_map_points.size() - 1) {
      s = s + sqrt(pow(origin_map_points[i].x - origin_map_points[i + 1].x, 2) +
                   pow(origin_map_points[i].y - origin_map_points[i + 1].y, 2));
    }
  }
  return new_origin_map_points;
}
bool PlanningRLMap::CalculateMapPointsS(PlanningRLMap::MapPoint2d& map_points) {
  double s = 0;
  if (map_points.empty()) {
    return false;
  }
  map_points[0].s = s;
  for (size_t i = 1; i < map_points.size(); i++) {
    s = s + sqrt(pow(map_points[i].x - map_points[i - 1].x, 2) +
                 pow(map_points[i].y - map_points[i - 1].y, 2));
    map_points[i].s = s;
  }
  return true;
}
bool PlanningRLMap::GetRoadBoundPointsByLaneId(
    const std::string& id, PlanningRLMap::MapPoint2d& left_bound_points,
    PlanningRLMap::MapPoint2d& right_bound_points) {
  Lane::ConstPtr lane = semantic_map_index_.Get<Lane>(id);
  auto road_link = semantic_map_.GetRoadLink(lane->container_id());
  if (road_link == nullptr) {
    left_bound_points = PlanningRLMap::MapPoint2d{};
    right_bound_points = PlanningRLMap::MapPoint2d{};
    return false;
  }
  auto lane_group = road_link->GetLaneGroup(lane->lane_group_id());
  if (lane_group == nullptr) {
    left_bound_points = PlanningRLMap::MapPoint2d{};
    right_bound_points = PlanningRLMap::MapPoint2d{};
    return false;
  }
  if (lane_group->left_boundary() == nullptr) {
    LOG_INFO("lane_group->left_boundary() is null");
    left_bound_points = PlanningRLMap::MapPoint2d{};
  } else {
    auto left_points = lane_group->left_boundary()->polyline().points();
    left_bound_points = ConvertToMapPoints(left_points);
  }

  if (lane_group->right_boundary() == nullptr) {
    LOG_INFO("lane_group->right_boundary() is null");
    right_bound_points = PlanningRLMap::MapPoint2d{};
  } else {
    auto right_points = lane_group->right_boundary()->polyline().points();
    right_bound_points = ConvertToMapPoints(right_points);
  }
  return true;
}

bool PlanningRLMap::GetSpeedLimitOfLane(const std::string& id,
                                        double& speed_limit) {
  Lane::ConstPtr lane = semantic_map_index_.Get<Lane>(id);
  if (!lane) {
    return false;
  }
  speed_limit = lane->speed_limit();
  return true;
}

bool PlanningRLMap::GetLeftLanes(const std::string& id,
                                 std::vector<std::string>& lanes) {
  Lane::ConstPtr lane = semantic_map_index_.Get<Lane>(id);
  if (!lane) {
    return false;
  }
  std::vector<LaneLeftForwardNeighbor::ConstPtr> left_neighbors;
  topological_map_.lane_connections().GetLeftForwardNeighborsOf(lane,
                                                                left_neighbors);
  lanes.clear();
  for (auto& neighbor : left_neighbors) {
    if (neighbor == nullptr) {
      continue;
    }
    lanes.push_back(neighbor->to()->id());
  }
  return true;
}

bool PlanningRLMap::GetRightLanes(const std::string& id,
                                  std::vector<std::string>& lanes) {
  Lane::ConstPtr lane = semantic_map_index_.Get<Lane>(id);
  if (!lane) {
    return false;
  }
  std::vector<LaneRightForwardNeighbor::ConstPtr> right_neighbors;
  topological_map_.lane_connections().GetRightForwardNeighborsOf(
      lane, right_neighbors);
  lanes.clear();
  for (auto& neighbor : right_neighbors) {
    if (neighbor == nullptr) {
      continue;
    }
    lanes.push_back(neighbor->to()->id());
  }
  return true;
}

bool PlanningRLMap::GetPredecessorLanes(const std::string& id,
                                        std::vector<std::string>& lanes) {
  Lane::ConstPtr lane = semantic_map_index_.Get<Lane>(id);
  if (!lane) {
    return false;
  }
  std::vector<LaneEndConnection::ConstPtr> lane_to_connections;
  topological_map_.lane_connections().GetLaneEndConnectionsTo(
      lane, lane_to_connections);
  if (lane_to_connections.empty()) {
    return false;
  }
  lanes.clear();
  for (auto& connection : lane_to_connections) {
    if (connection == nullptr) {
      continue;
    }
    lanes.push_back(connection->from()->id());
  }
  return true;
}

bool PlanningRLMap::GetSuccessorLanes(const std::string& id,
                                      std::vector<std::string>& lanes) {
  Lane::ConstPtr lane = semantic_map_index_.Get<Lane>(id);
  if (!lane) {
    return false;
  }
  std::vector<LaneEndConnection::ConstPtr> lane_from_connections;
  topological_map_.lane_connections().GetLaneEndConnectionsFrom(
      lane, lane_from_connections);
  if (lane_from_connections.empty()) {
    return false;
  }
  lanes.clear();
  for (auto& connection : lane_from_connections) {
    if (connection == nullptr) {
      continue;
    }
    lanes.push_back(connection->to()->id());
  }
  return true;
}

size_t PlanningRLMap::find_closest_map_points_index(
    const PlanningRLMap::MapPoint center_point,
    const PlanningRLMap::MapPoint2d& map_points) {
  size_t index = 0;
  double dis = 1000000;
  for (size_t i = 0; i < map_points.size(); i++) {
    double d = sqrt(pow(center_point.x - map_points[i].x, 2) +
                    pow(center_point.y - map_points[i].y, 2));
    if (dis > d) {
      dis = d;
      index = i;
    }
  }
  return index;
}

size_t PlanningRLMap::find_closest_map_points_index(
    const double x, const double y,
    const PlanningRLMap::MapPoint2d& map_points) {
  size_t index = 0;
  double dis = 1000000;
  for (size_t i = 0; i < map_points.size(); i++) {
    double d = sqrt(pow(x - map_points[i].x, 2) + pow(y - map_points[i].y, 2));
    if (dis > d) {
      dis = d;
      index = i;
    }
  }
  return index;
}

Vec2d PlanningRLMap::ConverMapPointToVec2d(PlanningRLMap::MapPoint map_point) {
  return Vec2d(map_point.x, map_point.y);
}

}  // namespace planning_rl
}  // namespace neodrive