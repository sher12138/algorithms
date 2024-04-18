#pragma once
#include "hdmap/hdmap.h"
#include "hdmap/topo_map/lane_topomap.h"
#include "src/planning/common/parking_space/parking_space_base.h"
namespace neodrive {
namespace planning {
static cyberverse::LaneInfoConstPtr GetOnlyPredecessor(
    cyberverse::LaneInfoConstPtr lane) {
  auto hdmap = cyberverse::HDMap::Instance();
  auto &to_from_map = cyberverse::LaneTopo::Instance()->GetToFromMap();
  if (!lane) return nullptr;
  auto iter = to_from_map.find(lane->Id());
  if (iter == to_from_map.end() || iter->second.empty()) return nullptr;
  auto only_predesessor = *iter->second.begin();
  return hdmap->GetLaneById(only_predesessor);
}

static cyberverse::LaneTopo::LaneTopoUnitSet GetPredecessorSet(
    cyberverse::LaneInfoConstPtr lane) {
  auto hdmap = cyberverse::HDMap::Instance();
  auto &to_from_map = cyberverse::LaneTopo::Instance()->GetToFromMap();
  if (!lane) return {};
  auto iter = to_from_map.find(lane->Id());
  if (iter == to_from_map.end() || iter->second.empty()) return {};
  auto predesessor_set = iter->second;
  return predesessor_set;
}

static cyberverse::LaneInfoConstPtr GetOnlySuccessor(
    cyberverse::LaneInfoConstPtr lane) {
  auto hdmap = cyberverse::HDMap::Instance();
  auto &from_to_map = cyberverse::LaneTopo::Instance()->GetFromToMap();
  if (!lane) return nullptr;
  auto iter = from_to_map.find(lane->Id());
  if (iter == from_to_map.end() || iter->second.empty()) return nullptr;
  auto only_successor = *iter->second.begin();
  return hdmap->GetLaneById(only_successor);
}

static void GetLaneRelativePoints(cyberverse::LaneInfoConstPtr lane,
                                  Vec3d &origin_pos,
                                  std::vector<Vec3d> &points_rel) {
  auto &points = lane->Points();
  points_rel.reserve(points.size());
  for (uint32_t i = 0; i < points.size(); ++i) {
    auto &each_pt = points[i];
    Vec3d new_pt{each_pt.x(), each_pt.y(), lane->Heading(each_pt)};
    common::ConvertToRelativeCoordinate(new_pt, origin_pos, new_pt);
    points_rel.emplace_back(std::move(new_pt));
  }
}

static void TransformToWorldCorrdinate(ParkingPath &path, Vec3d &pt_in_world) {
  for (auto &each_pt : path.path_points) {
    common::ConvertToWorldCoordinate(each_pt, pt_in_world, each_pt);
  }
}

static void RemoveDuplicates(ParkingPath &path) {
  if (path.path_points.empty()) return;
  auto points = &path.path_points;
  auto s = &path.point_s;
  int count = 1;
  for (auto i = 1; i < points->size(); ++i) {
    const auto &point = (*points)[i];
    double dist = common::Distance2D(point, (*points)[count - 1]);
    if (dist > ParkingSpace::kPointDuplicateDist) {
      (*points)[count] = point;
      (*s)[count] = (*s)[count - 1] + dist;
      count++;
    }
  }
  points->resize(count);
  s->resize(count);
}
}  // namespace planning
}  // namespace neodrive