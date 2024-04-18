#include "planning_map.h"

#include "common/math/math_utils.h"
#include "common/math/util.h"
#include "common/shm_manager.h"
#include "common/util/time_logger.h"
#include "common/visualizer_event/visualizer_event.h"
#include "common_config/config/common_config.h"
#include "src/common/coordinate/coodrdinate_convertion.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/planning_macros.h"
#include "src/planning/common/planning_types.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/public/planning_lib_header.h"
#include "src/planning/reference_line/reference_line_util.h"

namespace neodrive {
namespace planning {
using cyberverse::ClearAreaInfoConstPtr;
using cyberverse::CrosswalkInfoConstPtr;
using cyberverse::JunctionInfoConstPtr;
using cyberverse::LaneInfoConstPtr;
using cyberverse::RoadInfoConstPtr;
using cyberverse::SignalInfoConstPtr;
using cyberverse::SpeedBumpInfoConstPtr;
using cyberverse::StopSignInfoConstPtr;
using cyberverse::YieldSignInfoConstPtr;

void PlanningMap::InitSharedMemory() {
  if (hdmap_->IsInited()) return;
  uint32_t total_size = 1000 * 1024 * 1024 * 1;
  char *buffer =
      (char *)cyberverse::ShmManager::Instance()->OpenOrCreate(total_size);
  uint32_t read_size = hdmap_->LoadMapInfosFromMemory(buffer);
  LOG_INFO("load map from shared memory success:used:{} ,max:{}", read_size,
           total_size);
}

PlanningMap::PlanningMap() { hdmap_ = cyberverse::HDMap::Instance(); }

std::string PlanningMap::GetHashIdString(const uint64_t hash) {
  return hdmap_->GetIdHashString(hash);
}

void PlanningMap::SetLocalPoint(const double x, const double y,
                                const double heading) {
  local_point_ = Point2D(x, y, heading);
  local_lane_ = GetLocalLane();
}

LaneInfoConstPtr PlanningMap::GetLocalLane() {
  std::vector<LaneInfoConstPtr> lanes;
  std::size_t lane_size = hdmap_->GetLanesWithHeading(
      common::math::Vec2d(local_point_.point.x(), local_point_.point.y()),
      kRadius, local_point_.heading, kHeadingDiff, &lanes);
  if (lane_size == 0) {
    return nullptr;
  }

  for (auto &tmp : lanes) {
    if (tmp == nullptr) {
      continue;
    }
    double s(0.0), l(0.0);
    double length(0.0);
    length = tmp->TotalLength();
    tmp->GetProjection(
        common::math::Vec2d(local_point_.point.x(), local_point_.point.y()), &s,
        &l);
    if (s >= 0.0 && s <= length) {
      return tmp;
    }
  }
  return lanes.front();
}

void PlanningMap::SetLocalPoint(const double x, const double y,
                                const double heading,
                                const std::vector<uint64_t> &lane_ids) {
  local_point_ = Point2D(x, y, heading);
  local_lane_ = GetLocalLane(lane_ids);
}

LaneInfoConstPtr PlanningMap::GetLocalLane(
    const std::vector<uint64_t> &lane_ids) {
  std::vector<cyberverse::LaneInfoConstPtr> lanes;
  hdmap_->GetLanesWithHeading(
      common::math::Vec2d(local_point_.point.x(), local_point_.point.y()),
      kRadius, local_point_.heading, kHeadingDiff, &lanes);
  if (lanes.empty()) {
    return nullptr;
  }

  auto is_valid = [](auto lane, auto &pt) {
    double s, l, len = lane->TotalLength();
    lane->GetProjection(common::math::Vec2d(pt.x(), pt.y()), &s, &l);
    return s > 0 && s < len;
  };
  std::unordered_set<uint64_t> us(lane_ids.begin(), lane_ids.end());
  for (auto lane : lanes)
    if (lane && us.count(lane->Id()) && is_valid(lane, local_point_.point))
      return lane;
  // not on lane_ids
  for (auto lane : lanes)
    if (lane && is_valid(lane, local_point_.point)) return lane;

  return lanes.front();
}

bool PlanningMap::GetIdSInLane(const double x, const double y,
                               const double heading, uint64_t &id, double &s) {
  SetLocalPoint(x, y, heading);
  std::vector<cyberverse::LaneInfoConstPtr> lanes;
  hdmap_->GetLanes(
      common::math::Vec2d(local_point_.point.x(), local_point_.point.y()),
      kRadius, &lanes);
  if (lanes.empty()) {
    return false;
  }
  local_lane_ = lanes.front();
  double l = 0;
  if (local_lane_ == nullptr) {
    return false;
  }
  local_lane_->GetProjection(
      common::math::Vec2d(local_point_.point.x(), local_point_.point.y()), &s,
      &l);
  return true;
}

bool PlanningMap::GetLInLane(const double x, const double y,
                             const double heading, double &l) {
  SetLocalPoint(x, y, heading);
  double s = 0;
  if (local_lane_ == nullptr) {
    return false;
  }
  local_lane_->GetProjection(
      common::math::Vec2d(local_point_.point.x(), local_point_.point.y()), &s,
      &l);
  return true;
}

bool PlanningMap::GetPointLeftBound(const double x, const double y,
                                    const double heading, double &bound) {
  SetLocalPoint(x, y, heading);
  if (!local_lane_) {
    return false;
  }
  bound = local_lane_->left_divider_polygon().Distance2D(
      common::math::Vec2d(local_point_.point.x(), local_point_.point.y()));
  return true;
}

bool PlanningMap::GetPointRightBound(const double x, const double y,
                                     const double heading, double &bound) {
  SetLocalPoint(x, y, heading);
  if (!local_lane_) {
    return false;
  }
  bound = local_lane_->right_divider_polygon().Distance2D(
      common::math::Vec2d(local_point_.point.x(), local_point_.point.y()));
  return true;
}

bool PlanningMap::GetPointLaneLeftRoadBound(const double x, const double y,
                                            const uint64_t lane, double &bound,
                                            double &reverse_bound) {
  const common::math::Vec2d current_pt{x, y};
  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane || !ptr_lane->IsOnLane(current_pt)) {
    return false;
  }

  auto road_ptr = hdmap_->GetRoadById(ptr_lane->RoadId());
  if (!road_ptr) {
    return false;
  }
  auto section_id = ptr_lane->SectionId();

  auto section_ptr = road_ptr->GetSectionInRoadById(section_id);
  if (!section_ptr) {
    return false;
  }
  bound = section_ptr->LeftSectionBoundary().Distance2D(current_pt);
  LOG_DEBUG("origin bound:{}", bound);

  /// forward: only junction should be extend
  std::vector<common::math::Vec2d> junction_corners;
  if (section_ptr->LeftBoundaryType() ==
          static_cast<int>(BoundaryEdgeType::VIRTUAL) &&
      GetJunctionCorners(lane, junction_corners)) {
    double left_direction_heading = ptr_lane->Heading(current_pt) + M_PI / 2.0;
    const common::math::Vec2d dest_pt{
        x + kMaxJunctionRadius * std::cos(left_direction_heading),
        y + kMaxJunctionRadius * std::sin(left_direction_heading)};
    common::math::LineSegment2d source_seg(current_pt, dest_pt);
    common::math::Vec2d intersect_pt;
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < junction_corners.size(); ++i) {
      int next_idx = (i + 1) == junction_corners.size() ? 0 : i + 1;
      common::math::LineSegment2d dest_seg(junction_corners[i],
                                           junction_corners[next_idx]);
      if (source_seg.get_intersect(dest_seg, &intersect_pt)) {
        min_dist = std::min(min_dist, intersect_pt.distance_to(current_pt));
      }
    }
    if (min_dist < kMaxJunctionRadius && min_dist > bound) bound = min_dist;
  }

  /// reverse: reverse road should be extend
  double reverse_road_bound = std::numeric_limits<double>::infinity();
  if (section_ptr->LeftBoundaryType() ==
      static_cast<int>(BoundaryEdgeType::MARKING)) {
    for (std::size_t i = 0; i < section_ptr->NeighborReverseRoadIds().size();
         ++i) {
      auto reverse_road_ptr =
          hdmap_->GetRoadById(section_ptr->NeighborReverseRoadIds()[i]);
      if (!reverse_road_ptr) {
        continue;
      }
      for (const auto &sec : reverse_road_ptr->Sections()) {
        if (!(sec->LeftBoundaryType() ==
              static_cast<int>(BoundaryEdgeType::MARKING))) {
          continue;
        }
        reverse_road_bound =
            std::min(reverse_road_bound,
                     sec->RightSectionBoundary().Distance2D(current_pt));
        LOG_DEBUG("origin reverse_road_bound:{}", reverse_road_bound);
      }
    }
  }
  reverse_bound =
      (reverse_road_bound == std::numeric_limits<double>::infinity())
          ? bound
          : std::max(bound, reverse_road_bound);

  return true;
}

bool PlanningMap::GetPointLaneRightRoadBound(const double x, const double y,
                                             const uint64_t &lane,
                                             double &bound) {
  const common::math::Vec2d current_pt{x, y};
  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane || !ptr_lane->IsOnLane(current_pt)) {
    return false;
  }

  auto road_ptr = hdmap_->GetRoadById(ptr_lane->RoadId());
  if (!road_ptr) {
    return false;
  }
  auto section_id = ptr_lane->SectionId();

  auto section_ptr = road_ptr->GetSectionInRoadById(section_id);
  if (!section_ptr) {
    return false;
  }
  bound = section_ptr->RightSectionBoundary().Distance2D(current_pt);
  std::vector<common::math::Vec2d> junction_corners;
  if (section_ptr->RightBoundaryType() ==
          static_cast<int>(BoundaryEdgeType::VIRTUAL) &&
      GetJunctionCorners(lane, junction_corners)) {
    double right_direction_heading = ptr_lane->Heading(current_pt) - M_PI / 2.0;
    const common::math::Vec2d dest_pt{
        x + kMaxJunctionRadius * std::cos(right_direction_heading),
        y + kMaxJunctionRadius * std::sin(right_direction_heading)};
    common::math::LineSegment2d source_seg(current_pt, dest_pt);
    common::math::Vec2d intersect_pt;
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < junction_corners.size() - 1; ++i) {
      common::math::LineSegment2d dest_seg(junction_corners[i],
                                           junction_corners[i + 1]);
      if (source_seg.get_intersect(dest_seg, &intersect_pt)) {
        min_dist = std::min(min_dist, intersect_pt.distance_to(current_pt));
      }
    }
    if (min_dist < kMaxJunctionRadius && min_dist > bound) bound = min_dist;
  }
  return true;
}

std::pair<double, double> PlanningMap::GetLaneDistanceWidth(
    const uint64_t &lane, const double s) {
  std::pair<double, double> ans{-1, -1};
  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane) return ans;
  // failure of projection will not change the left and right width values
  ptr_lane->GetWidth(s, &ans.first, &ans.second);
  return ans;
}

bool PlanningMap::GetPointLeftRoadBound(const double x, const double y,
                                        const double heading, double &bound) {
  SetLocalPoint(x, y, heading);
  if (!local_lane_) {
    return false;
  }

  auto road_ptr = hdmap_->GetRoadById(local_lane_->RoadId());
  if (!road_ptr) {
    return false;
  }
  auto section_id = local_lane_->SectionId();

  auto section_ptr = road_ptr->GetSectionInRoadById(section_id);
  if (!section_ptr) {
    return false;
  }
  bound =
      section_ptr->LeftSectionBoundary().Distance2D(common::math::Vec2d{x, y});
  return true;
}

bool PlanningMap::GetPointRightRoadBound(const double x, const double y,
                                         const double heading, double &bound) {
  SetLocalPoint(x, y, heading);
  if (!local_lane_) {
    return false;
  }

  auto road_ptr = hdmap_->GetRoadById(local_lane_->RoadId());
  if (!road_ptr) {
    return false;
  }
  auto section_id = local_lane_->SectionId();

  auto section_ptr = road_ptr->GetSectionInRoadById(section_id);
  if (!section_ptr) {
    return false;
  }
  bound =
      section_ptr->RightSectionBoundary().Distance2D(common::math::Vec2d{x, y});
  return true;
}

bool PlanningMap::GetIdSInLane(const double x, const double y,
                               const double heading,
                               const std::vector<uint64_t> &lane_ids,
                               uint64_t &id, double &s) {
  SetLocalPoint(x, y, heading, lane_ids);
  std::vector<cyberverse::LaneInfoConstPtr> lanes;
  hdmap_->GetLanes(
      common::math::Vec2d(local_point_.point.x(), local_point_.point.y()),
      kRadius, &lanes);
  if (lanes.empty()) {
    return false;
  }
  local_lane_ = lanes.front();
  if (!local_lane_) {
    return false;
  }
  double l = 0;
  local_lane_->GetProjection(
      common::math::Vec2d(local_point_.point.x(), local_point_.point.y()), &s,
      &l);
  id = local_lane_->Id();
  return true;
}

bool PlanningMap::GetIdSInLane(const std::vector<uint64_t> &lane_ids,
                               ReferencePoint &point) {
  SetLocalPoint(point.x(), point.y(), point.heading(), lane_ids);
  if (!local_lane_) {
    return false;
  }
  double l = 0.0;
  double s = 0.0;
  local_lane_->GetProjection(
      common::math::Vec2d(local_point_.point.x(), local_point_.point.y()), &s,
      &l);
  point.set_hd_map_lane_s(s);
  point.set_hd_map_lane_l(l);
  point.set_hd_map_lane_id(local_lane_->Id());
  return true;
}

bool PlanningMap::GetSInLane(const double x, const double y,
                             const double heading,
                             const std::vector<uint64_t> &lane_ids, double &s) {
  SetLocalPoint(x, y, heading, lane_ids);
  if (!local_lane_) {
    return false;
  }
  double l = 0;
  local_lane_->GetProjection(
      common::math::Vec2d(local_point_.point.x(), local_point_.point.y()), &s,
      &l);
  return true;
}

bool PlanningMap::GetLInLane(const double x, const double y,
                             const double heading,
                             const std::vector<uint64_t> &lane_ids, double &l) {
  SetLocalPoint(x, y, heading, lane_ids);
  if (!local_lane_) {
    return false;
  }
  double s = 0;
  local_lane_->GetProjection(
      common::math::Vec2d(local_point_.point.x(), local_point_.point.y()), &s,
      &l);
  return true;
}
bool PlanningMap::GetPointLeftBound(const double x, const double y,
                                    const double heading,
                                    const std::vector<uint64_t> &lane_ids,
                                    double &bound) {
  SetLocalPoint(x, y, heading, lane_ids);
  if (!local_lane_) {
    return false;
  }
  bound = local_lane_->left_divider_polygon().Distance2D(
      common::math::Vec2d(local_point_.point.x(), local_point_.point.y()));
  return true;
}

bool PlanningMap::GetPointRightBound(const double x, const double y,
                                     const double heading,
                                     const std::vector<uint64_t> &lane_ids,
                                     double &bound) {
  SetLocalPoint(x, y, heading, lane_ids);
  if (!local_lane_) {
    return false;
  }
  bound = local_lane_->right_divider_polygon().Distance2D(
      common::math::Vec2d(local_point_.point.x(), local_point_.point.y()));
  return true;
}

bool PlanningMap::GetLengthOfLane(const uint64_t id, double &length) {
  auto lane = hdmap_->GetLaneById(id);
  if (!lane) {
    return false;
  }
  length = lane->TotalLength();
  return true;
}

bool PlanningMap::GetWidthOfLane(const uint64_t id, double &width) {
  auto lane = hdmap_->GetLaneById(id);
  if (!lane) {
    return false;
  }
  double left_width = 0;
  double right_width = 0;
  lane->GetWidth(0, &left_width, &right_width);
  width = left_width + right_width;
  return true;
}

bool PlanningMap::GetSpeedLimitOfLane(const uint64_t id, double &speed_limit) {
  auto lane = hdmap_->GetLaneById(id);
  if (!lane) {
    return false;
  }
  speed_limit = lane->SpeedLimit();
  return true;
}

bool PlanningMap::GetLaneType(
    const uint64_t lane_id, neodrive::global::hdmap::Lane_LaneType *lane_type) {
  if (lane_id == 0) {
    LOG_DEBUG("input lane is null");
    return false;
  }
  auto lane = hdmap_->GetLaneById(lane_id);
  if (!lane) {
    LOG_DEBUG("cannot find lane from hdmap, id: {}", lane_id);
    return false;
  }
  LOG_INFO("cannot find lane from hdmap, id: {}",
           hdmap_->GetIdHashString(lane_id));
  *lane_type =
      static_cast<neodrive::global::hdmap::Lane_LaneType>(lane->LaneType());
  return true;
}

bool PlanningMap::GetLeftLanes(const uint64_t id,
                               std::vector<uint64_t> &lanes) {
  auto lane = hdmap_->GetLaneById(id);
  if (!lane) {
    return false;
  }
  std::vector<cyberverse::LaneRelation> left_neighbors;
  size_t lane_size =
      hdmap_->TopologicalLaneConnections().GetLeftForwardNeighborsOf(
          id, left_neighbors);
  lanes.clear();
  for (auto &neighbor : left_neighbors) {
    lanes.push_back(neighbor.to());
  }
  return true;
}

bool PlanningMap::GetRightLanes(const uint64_t id,
                                std::vector<uint64_t> &lanes) {
  auto lane = hdmap_->GetLaneById(id);
  if (!lane) {
    return false;
  }
  std::vector<cyberverse::LaneRelation> right_neighbors;
  hdmap_->TopologicalLaneConnections().GetRightForwardNeighborsOf(
      id, right_neighbors);
  lanes.clear();
  for (auto &neighbor : right_neighbors) {
    lanes.push_back(neighbor.to());
  }
  return true;
}

bool PlanningMap::GetPredecessorLanes(const uint64_t id,
                                      std::vector<uint64_t> &lanes) {
  auto lane = hdmap_->GetLaneById(id);
  if (!lane) {
    return false;
  }
  std::vector<cyberverse::LaneRelation> lane_to_connections;
  hdmap_->TopologicalLaneConnections().GetLaneEndConnectionsTo(
      id, lane_to_connections);
  if (lane_to_connections.empty()) {
    return false;
  }
  lanes.clear();
  for (auto &connection : lane_to_connections) {
    if (id == connection.from()) continue;
    lanes.push_back(connection.from());
    if (lanes.back() == id) return false;
  }
  return true;
}

bool PlanningMap::GetSuccessorLanes(const uint64_t id,
                                    std::vector<uint64_t> &lanes) {
  auto lane = hdmap_->GetLaneById(id);
  if (!lane) {
    return false;
  }
  std::vector<cyberverse::LaneRelation> lane_from_connections;
  hdmap_->TopologicalLaneConnections().GetLaneEndConnectionsFrom(
      id, lane_from_connections);
  if (lane_from_connections.empty()) {
    return false;
  }
  lanes.clear();
  for (auto &connection : lane_from_connections) {
    lanes.push_back(connection.to());
    if (lanes.back() == id) return false;
  }
  return true;
}

bool PlanningMap::GetLanePointWithDistance(const uint64_t id, const double s,
                                           ReferencePoint &point) {
  auto lane = hdmap_->GetLaneById(id);
  if (!lane) {
    return false;
  }
  common::math::Vec2d map_point;
  double z;
  if (!lane->GetSmoothPoint(s, map_point, &z))
    LOG_INFO("{} {} failed", lane->TotalLength(), s);
  std::pair<double, double> seg_z;
  const auto &segment = lane->Get2DNearestLineSegment(map_point, &seg_z);
  point.set_hd_map_lane_id(id);
  point.set_x(map_point.x());
  point.set_y(map_point.y());
  point.set_hd_map_lane_s(s);
  point.set_z(z);
  point.set_heading(segment.heading());
  point.set_pitch(std::atan2(seg_z.second - seg_z.first, segment.length()));

  return true;
}

bool PlanningMap::GetNearestLanes(
    const double x, const double y, double radius,
    std::vector<cyberverse::LaneInfoConstPtr> &nearest_lanes) {
  return hdmap_->GetLanes(common::math::Vec2d(x, y), radius, &nearest_lanes) >
         0;
}

bool PlanningMap::GetNearestLaneWithHeading(
    const double x, const double y, const double radius, const double heading,
    const double heading_diff, cyberverse::LaneInfoConstPtr &lane) {
  common::math::Vec2d point_2D(x, y);
  std::vector<cyberverse::LaneInfoConstPtr> lanes;
  std::size_t lane_size = hdmap_->GetLanesWithHeading(point_2D, radius, heading,
                                                      heading_diff, &lanes);
  if (lane_size == 0 || lanes.front() == nullptr) {
    return false;
  }
  lane = lanes.front();
  return true;
}

bool PlanningMap::GetNearestLaneWithHeading(
    const double x, const double y, const double radius, const double heading,
    const double heading_diff, uint64_t &best_matched_lane,
    double &heading_deviation, double &offset) {
  constexpr double max_vertical_projection = 7.0;
  constexpr double max_horizontal_projection = 3.0;
  auto Cmp = [&](const std::pair<double, double> &a,
                 const std::pair<double, double> &b) -> bool {
    return math::Sign(a.second - b.second) == 0 ? a.first < b.first
                                                : a.second < b.second;
  };
  auto Cost = [&max_vertical_projection](auto dis, auto heading_cost,
                                         double w = 0.5) {
    return w * dis / max_vertical_projection + (1 - w) * heading_cost;
  };

  std::vector<cyberverse::LaneInfoConstPtr> nearest_lanes;
  hdmap_->GetLanes(common::math::Vec2d(x, y), radius, &nearest_lanes);
  math::Point ego_pos{x, y};
  math::AD2 ego_heading{std::cos(heading), std::sin(heading)};
  std::pair<uint64_t, double> res{0, std::numeric_limits<double>::infinity()};
  for (auto lane : nearest_lanes) {
    auto &pts = lane->Points();
    auto &segments = lane->Segments();
    std::map<std::pair<double, double>, int, decltype(Cmp)> seg_distance_to_ego(
        Cmp);

    for (int i = 0; i < segments.size(); i++) {
      math::AD2 seg_dir{segments[i].unit_direction().x(),
                        segments[i].unit_direction().y()};
      math::AD2 seg_to_ego{ego_pos.x() - segments[i].start().x(),
                           ego_pos.y() - segments[i].start().y()};
      double dis_y = std::abs(math::Cross(seg_dir, seg_to_ego));
      double dis_x = math::Dot(seg_dir, seg_to_ego);
      if (dis_x > 1e-3) {
        dis_x = std::max(0.0, dis_x - segments[i].length());
      } else if (dis_x < -1e-3) {
        dis_x = -dis_x;
      }
      double dis = math::Length({dis_x, dis_y});
      if (dis_x > max_horizontal_projection ||
          dis_y > max_vertical_projection) {
        LOG_DEBUG("[{}](drop): dis_x->{}, dis_y->{}", i, dis_x, dis_y);
        continue;
      }
      double heading_cost = 0.5 - 0.5 * math::Dot(ego_heading, seg_dir);
      seg_distance_to_ego.insert(
          std::make_pair(std::make_pair(dis, heading_cost), i));
    }
    if (seg_distance_to_ego.empty()) {
      continue;
    }
    int cnt = 0;
    constexpr int max_matched_line_num = 3;
    std::pair<std::pair<int, double>, std::pair<double, double>>
        best_matched_seg{{seg_distance_to_ego.begin()->second,
                          Cost(seg_distance_to_ego.begin()->first.first,
                               seg_distance_to_ego.begin()->first.second)},
                         {seg_distance_to_ego.begin()->first.first,
                          seg_distance_to_ego.begin()->first.second}};
    for (auto &item : seg_distance_to_ego) {
      double cost = Cost(item.first.first, item.first.second);
      double best_cost = best_matched_seg.first.second;
      if (math::Sign(cost - best_matched_seg.first.second, 1e-5) == 0) {
        cost = Cost(item.first.first, item.first.second, 0.4);
        best_cost = Cost(best_matched_seg.second.first,
                         best_matched_seg.second.second, 0.4);
      }
      if (cost < best_cost) {
        best_matched_seg.first.second =
            Cost(item.first.first, item.first.second);
        best_matched_seg.first.first = item.second;
        best_matched_seg.second.first = item.first.first;
        best_matched_seg.second.second = item.first.second;
      }
      if (++cnt > max_matched_line_num) {
        break;
      }
    }
    if (res.second > best_matched_seg.first.second) {
      res.first = lane->Id();
      res.second = best_matched_seg.first.second;
      heading_deviation =
          math::Sign(math::Cross(
              ego_heading,
              math::AD2{
                  segments[best_matched_seg.first.first].unit_direction().x(),
                  segments[best_matched_seg.first.first]
                      .unit_direction()
                      .y()})) *
          std::acos(1 - best_matched_seg.second.second * 2);
      offset = math::Distance(
          ego_pos, math::LineSegment{
                       {segments[best_matched_seg.first.first].start().x(),
                        segments[best_matched_seg.first.first].start().y()},
                       {segments[best_matched_seg.first.first].end().x(),
                        segments[best_matched_seg.first.first].end().y()}});

      LOG_DEBUG("{}[{}]: heading_dev: {}, offset: {}, cost: {}({} , {})",
                GetHashIdString(lane->Id()), best_matched_seg.first.first,
                heading_deviation, offset, best_matched_seg.first.second,
                best_matched_seg.second.first, best_matched_seg.second.second);
    }
  }
  if (res.first == 0 && res.second == std::numeric_limits<double>::infinity()) {
    return false;
  }
  best_matched_lane = res.first;
  return true;
}

bool PlanningMap::GetHeadingWithLane(const uint64_t id, double &heading) {
  MapPoint map_point;

  auto lane = hdmap_->GetLaneById(id);
  if (!lane) {
    return false;
  }
  heading = lane->Heading(0);
  return true;
}

// need to use id
bool PlanningMap::GetSLWithLane(const uint64_t id, const double x,
                                const double y, double &s, double &l) {
  auto lane = hdmap_->GetLaneById(id);
  if (!lane) {
    return false;
  }
  lane->GetProjection(common::math::Vec2d(x, y), &s, &l);
  return true;
}

bool PlanningMap::GetPointSpeedLimit(const double x, const double y,
                                     const double heading,
                                     double &speed_limit) {
  SetLocalPoint(x, y, heading);
  if (!local_lane_) {
    return false;
  }
  speed_limit = local_lane_->SpeedLimit();
  return true;
}

bool PlanningMap::GetLaneMultipleType(
    const uint64_t lane_id, std::vector<uint32_t> &multiple_lane_type) {
  if (lane_id == 0) {
    LOG_DEBUG("input lane is null");
    return false;
  }
  auto lane = hdmap_->GetLaneById(lane_id);
  if (!lane) {
    LOG_DEBUG("cannot find lane from hdmap, id: {}", lane_id);
    return false;
  }
  for (size_t i = 0; i < lane->LaneMultipleType().size(); i++) {
    multiple_lane_type.emplace_back(lane->LaneMultipleType()[i]);
  }
  return true;
}

bool PlanningMap::GetLaneTurnType(const uint64_t lane_id,
                                  Lane::TurningType *lane_turn_type) {
  if (lane_id == 0) {
    LOG_DEBUG("input lane is null");
    return false;
  }
  auto lane = hdmap_->GetLaneById(lane_id);
  if (!lane) {
    LOG_DEBUG("cannot find lane from hdmap, id: {}", lane_id);
    return false;
  }
  *lane_turn_type = static_cast<Lane::TurningType>(lane->TurnType());
  return true;
}

double PlanningMap::GetLaneSpeedLimitByType(
    cyberverse::LaneInfoConstPtr &lane) {
  auto &lane_types = lane->LaneMultipleType();
  double speed_limit = std::numeric_limits<double>::max();
  auto &speed_limit_config = common::config::CommonConfig::Instance()
                                 ->drive_strategy_config()
                                 .non_motorway;
  for (uint32_t i = 0; i < lane_types.size(); ++i) {
    if (lane_types[i] ==
        static_cast<uint32_t>(global::hdmap::Lane::BUS_BAY_LANE)) {
      speed_limit = (speed_limit_config.dynamic_limit_speed < speed_limit)
                        ? speed_limit_config.dynamic_limit_speed
                        : speed_limit;
    }
  }
  return speed_limit < speed_limit_config.max_cruise_speed ? speed_limit : 0.0;
}

double PlanningMap::GetLaneSpeedLimit(const uint64_t lane) {
  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane) return 0.0;
  double lane_limit = ptr_lane->SpeedLimit();
  double type_limit = GetLaneSpeedLimitByType(ptr_lane);
  if (lane_limit > 0.0 && type_limit > 0)
    return std::min(lane_limit, type_limit);
  else if (lane_limit <= 0.0 && type_limit > 0)
    return type_limit;
  else if (lane_limit > 0.0 && type_limit <= 0)
    return lane_limit;
  else
    return 0.0;
}

bool PlanningMap::GetLaneTurnType(ReferencePoint &point) {
  Lane::TurningType pt_turn_type;
  if (!GetLaneTurnType(point.hd_map_lane_id(), &pt_turn_type)) {
    return false;
  }
  switch (pt_turn_type) {
    case Lane::TurningType::NO_TURN:
      point.set_no_signal();
      break;
    case Lane::TurningType::LEFT_TURN:
      point.set_left_signal();
      break;
    case Lane::TurningType::RIGHT_TURN:
      point.set_right_signal();
      break;
    case Lane::TurningType::U_TURN:
      point.set_uturn_signal();
      break;
    default:
      break;
  }

  return true;
}

bool PlanningMap::IsRightLane(const double x, const double y) {
  std::vector<LaneInfoConstPtr> lanes;
  hdmap_->GetLanes({x, y}, kRadius, &lanes);
  if (lanes.empty()) {
    LOG_INFO("empty lane, hdmap_->GetLanes() fails");
    return false;
  }
  std::sort(lanes.begin(), lanes.end(),
            [x, y](const LaneInfoConstPtr &a, const LaneInfoConstPtr &b) {
              return a->DistanceTo({x, y}) < b->DistanceTo({x, y});
            });
  for (auto &lane : lanes)
    LOG_DEBUG("lane id: {}, dist: {:.4f}", lane->Id(),
              lane->DistanceTo({x, y}));
  auto current_lane = lanes[0];
  auto &road_lanes =
      hdmap_->GetRoadById(current_lane->RoadId())->Sections()[0]->LaneIds();
  if (road_lanes.empty()) {
    LOG_INFO("empty road_lanes, hdmap_->GetRoadById() fails");
    return false;
  }
  for (int i = 0; i < road_lanes.size(); ++i)
    LOG_DEBUG("lane id : {}", road_lanes[i]);
  LOG_INFO("cur lane id: {}, right lane id: {}", current_lane->Id(),
           road_lanes.back());
  if (current_lane->Id() == road_lanes.back()) {
    LOG_INFO("cur lane is right lane, no biking lane");
    return true;
  }
  if (road_lanes.size() >= 2 &&
      current_lane->Id() == road_lanes[road_lanes.size() - 2]) {
    auto multi_lane_type =
        hdmap_->GetLaneById(road_lanes.back())->LaneMultipleType();
    for (int i = 0; i < multi_lane_type.size(); ++i) {
      if (multi_lane_type[i] == Lane_LaneType::Lane_LaneType_BIKING) {
        LOG_INFO("cur lane is next to biking lane");
        return true;
      }
    }
  }
  return false;
}

cyberverse::SignalInfoConstPtr PlanningMap::GetSignalById(const uint64_t id) {
  return hdmap_->GetSignalById(id);
}

BoundaryEdgeType PlanningMap::GetLaneLeftBoundaryType(const uint64_t lane) {
  BoundaryEdgeType ans = BoundaryEdgeType::DEFAULT;
  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane) return ans;
  auto road_ptr = hdmap_->GetRoadById(ptr_lane->RoadId());
  if (!road_ptr) return ans;
  auto section_id = ptr_lane->SectionId();

  auto section_ptr = road_ptr->GetSectionInRoadById(section_id);
  if (!section_ptr) return ans;

  return static_cast<BoundaryEdgeType>(section_ptr->LeftBoundaryType());
}

BoundaryEdgeType PlanningMap::GetLaneRightBoundaryType(const uint64_t lane) {
  BoundaryEdgeType ans = BoundaryEdgeType::DEFAULT;
  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane) return ans;
  auto road_ptr = hdmap_->GetRoadById(ptr_lane->RoadId());
  if (!road_ptr) return ans;
  auto section_id = ptr_lane->SectionId();

  auto section_ptr = road_ptr->GetSectionInRoadById(section_id);
  if (!section_ptr) return ans;

  return static_cast<BoundaryEdgeType>(section_ptr->RightBoundaryType());
}

bool PlanningMap::GetLaneBoundaryType(ReferencePoint &point) {
  SetLocalPoint(point.x(), point.y(), point.heading());
  if (!local_lane_) {
    return false;
  }
  auto road_ptr = hdmap_->GetRoadById(local_lane_->RoadId());
  if (!road_ptr) return false;
  auto section_id = local_lane_->SectionId();

  auto section_ptr = road_ptr->GetSectionInRoadById(section_id);
  if (!section_ptr) return false;
  point.set_left_boundary_edge_type(
      static_cast<BoundaryEdgeType>(section_ptr->LeftBoundaryType()));
  point.set_right_boundary_edge_type(
      static_cast<BoundaryEdgeType>(section_ptr->RightBoundaryType()));
}

LaneObjectsInfo PlanningMap::GetLaneCrosswalks(const uint64_t lane) {
  LaneObjectsInfo ans{};
  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane) return ans;
  const auto &objects = ptr_lane->Crosswalks();
  for (uint32_t i = 0; i < objects.size(); ++i) {
    const auto &each = objects[i];
    ans.emplace_back(each.object_id, each.start_s, each.end_s);
  }
  return ans;
}

LaneObjectsInfo PlanningMap::GetLaneSpeedBumps(const uint64_t lane) {
  LaneObjectsInfo ans{};

  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane) return ans;
  const auto &objects = ptr_lane->SpeedBumps();
  for (uint32_t i = 0; i < objects.size(); ++i) {
    const auto &each = objects[i];
    ans.emplace_back(each.object_id, each.start_s, each.end_s);
  }
  return ans;
}

LaneObjectsInfo PlanningMap::GetLaneYieldSigns(const uint64_t lane) {
  LaneObjectsInfo ans{};

  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane) return ans;
  const auto &objects = ptr_lane->YieldSigns();
  for (uint32_t i = 0; i < objects.size(); ++i) {
    const auto &each = objects[i];
    ans.emplace_back(each.object_id, each.start_s, each.end_s);
  }
  return ans;
}

LaneObjectsInfo PlanningMap::GetLaneStopSigns(const uint64_t lane) {
  LaneObjectsInfo ans{};

  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane) return ans;
  const auto &objects = ptr_lane->StopSigns();
  for (uint32_t i = 0; i < objects.size(); ++i) {
    const auto &each = objects[i];
    ans.emplace_back(each.object_id, each.start_s, each.end_s);
  }
  return ans;
}

LaneObjectsInfo PlanningMap::GetLaneSignals(const uint64_t lane) {
  LaneObjectsInfo ans{};
  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane) return ans;
  const auto &objects = ptr_lane->Signals();
  std::unordered_set<uint64_t> ids;
  for (uint32_t i = 0; i < objects.size(); ++i) {
    const auto &each = objects[i];
    if (!ids.emplace(each.object_id).second) continue;
    auto ptr_sig = hdmap_->GetSignalById(each.object_id);
    if (!ptr_sig) continue;
    const auto &stop_lines = ptr_sig->Segments();
    for (uint32_t j = 0; j < stop_lines.size(); ++j) {
      const auto &sl = stop_lines[j];
      if (double s; ptr_lane->Intersect2DWith(sl, s)) {
        ans.emplace_back(each.object_id, s, s + 0.5);
      }
    }
  }
  return ans;
}

LaneObjectsInfo PlanningMap::GetLaneJunctions(const uint64_t lane) {
  LaneObjectsInfo ans{};

  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane) return ans;
  const auto &objects = ptr_lane->Junctions();
  for (uint32_t i = 0; i < objects.size(); ++i) {
    const auto &each = objects[i];
    ans.emplace_back(each.object_id, each.start_s, each.end_s);
  }
  return ans;
}

LaneObjectsInfo PlanningMap::GetLaneClearAreas(const uint64_t lane) {
  LaneObjectsInfo ans{};
  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane) return ans;
  const auto &objects = ptr_lane->ClearAreas();
  for (uint32_t i = 0; i < objects.size(); ++i) {
    const auto &each = objects[i];
    ans.emplace_back(each.object_id, each.start_s, each.end_s);
  }
  return ans;
}

LaneObjectsInfo PlanningMap::GetLaneGeoFences(const uint64_t lane) {
  LaneObjectsInfo ans{};
  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane) return ans;
  for (const auto &obj : ptr_lane->GeofenceInfos()) {
    ans.emplace_back(obj->Id(), obj->StartS(), obj->EndS());
  }
  return ans;
}

LaneObjectsInfo PlanningMap::GetBarrierGates(const uint64_t lane) {
  LaneObjectsInfo ans{};
  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane) return ans;
  const auto &objects = ptr_lane->SpeedBumps();
  for (uint32_t i = 0; i < objects.size(); ++i) {
    const auto &each = objects[i];
    const auto speed_bump_info = hdmap_->GetSpeedBumpById(each.object_id);
    LOG_INFO("speed_bump_info.type: {}", speed_bump_info->Type());
    if (speed_bump_info->Type() == 1) {
      ans.emplace_back(each.object_id, each.start_s, each.end_s);
    }
  }
  return ans;
}

LaneObjectsInfo PlanningMap::GetLaneOverlapLanes(const uint64_t lane) {
  LaneObjectsInfo ans{};

  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane) return ans;
  const auto &objects = ptr_lane->Crosslanes();
  for (uint32_t i = 0; i < objects.size(); ++i) {
    const auto &each = objects[i];
    ans.emplace_back(each.object_id, each.start_s, each.end_s);
  }
  return ans;
}

std::vector<uint64_t> PlanningMap::GetLaneLeftForwardLanes(
    const uint64_t lane_id) {
  std::vector<uint64_t> ans;
  auto lane = hdmap_->GetLaneById(lane_id);
  if (lane == nullptr) return ans;
  std::vector<cyberverse::LaneRelation> left_neighbors;
  if (!hdmap_->TopologicalLaneConnections().GetLeftForwardNeighborsOf(
          lane->Id(), left_neighbors))
    return ans;
  for (auto &ln : left_neighbors) ans.push_back(ln.to());
  return ans;
}

std::vector<uint64_t> PlanningMap::GetLaneLeftBackwardLanes(
    const uint64_t lane_id) {
  std::vector<uint64_t> ans;
  auto lane = hdmap_->GetLaneById(lane_id);
  if (lane == nullptr) return ans;
  std::vector<cyberverse::LaneRelation> left_neighbors;
  if (!hdmap_->TopologicalLaneConnections().GetLeftBackwardNeighborsOf(
          lane->Id(), left_neighbors))
    return ans;
  for (auto &ln : left_neighbors) ans.push_back(ln.to());
  return ans;
}

std::vector<uint64_t> PlanningMap::GetLaneRightForwardLanes(
    const uint64_t lane_id) {
  std::vector<uint64_t> ans;
  auto lane = hdmap_->GetLaneById(lane_id);
  if (lane == nullptr) return ans;
  std::vector<cyberverse::LaneRelation> right_neighbors;
  if (!hdmap_->TopologicalLaneConnections().GetRightForwardNeighborsOf(
          lane->Id(), right_neighbors))
    return ans;
  for (auto &rn : right_neighbors) ans.push_back(rn.to());
  return ans;
}

std::vector<uint64_t> PlanningMap::GetLaneRightBackwardLanes(
    const uint64_t lane_id) {
  std::vector<uint64_t> ans;
  auto lane = hdmap_->GetLaneById(lane_id);
  if (lane == nullptr) return ans;
  std::vector<cyberverse::LaneRelation> right_neighbors;
  if (!hdmap_->TopologicalLaneConnections().GetRightBackwardNeighborsOf(
          lane->Id(), right_neighbors))
    return ans;
  for (auto &rn : right_neighbors) ans.push_back(rn.to());
  return ans;
}

uint64_t PlanningMap::GetLaneRoadId(const uint64_t lane_id) {
  std::vector<uint64_t> ans;
  auto lane = hdmap_->GetLaneById(lane_id);
  if (lane == nullptr) return 0;
  return lane->RoadId();
}

bool PlanningMap::IsPointInJunction(const double x, const double y,
                                    const double heading) {
  local_point_ = Point2D(x, y, heading);
  std::vector<JunctionInfoConstPtr> junctions;
  hdmap_->Getjunctions(
      common::math::Vec2d(local_point_.point.x(), local_point_.point.y()),
      kRadius, &junctions);
  return std::any_of(
      junctions.begin(), junctions.end(),
      [&](const JunctionInfoConstPtr junction) {
        return junction->Polygon().is_point_in(common::math::Vec2d(x, y));
      });
}
bool PlanningMap::IsJunctionWithinRadiusMetersAhead(
    const double x, const double y, const double heading, const double radius,
    double &distance, cyberverse::JunctionInfoConstPtr &junction_ptr) {
  std::vector<JunctionInfoConstPtr> junctions;
  hdmap_->Getjunctions(common::math::Vec2d(x, y), radius, &junctions);
  if (junctions.empty()) return false;
  auto found_junction = std::find_if(
      junctions.begin(), junctions.end(),
      [&](const JunctionInfoConstPtr junction) {
        return junction->Polygon().is_point_in(common::math::Vec2d(x, y));
      });
  if (found_junction != junctions.end()) {
    distance = 0.0;
    junction_ptr = *found_junction;
    return true;
  }
  for (auto &junction : junctions) {
    Vec3d relative_obs;
    auto junction_center_point = Vec3d(
        (junction->Polygon().min_x() + junction->Polygon().max_x()) / 2,
        (junction->Polygon().min_y() + junction->Polygon().max_y()) / 2, 0.0);
    Vec3d ego_position{x, y, heading};
    common::ConvertToRelativeCoordinate(junction_center_point, ego_position,
                                        relative_obs);
    if (relative_obs.x() < 0.0) continue;
    junction_ptr = junction;
    distance =
        junction->Polygon().distance_to_boundary(common::math::Vec2d(x, y));
  }
  if (junction_ptr == nullptr) {
    LOG_INFO("Not Found Junction Ahead!");
    return false;
  }
  return true;
}

bool PlanningMap::IsPointInCrosswalk(const double x, const double y,
                                     const double heading) {
  local_point_ = Point2D(x, y, heading);
  std::vector<CrosswalkInfoConstPtr> crosswalks;
  hdmap_->GetCrosswalks(common::math::Vec2d(x, y), kRadius, &crosswalks);
  return std::any_of(
      crosswalks.begin(), crosswalks.end(),
      [&](const CrosswalkInfoConstPtr crosswalk) {
        return crosswalk->Polygon().is_point_in(common::math::Vec2d(x, y));
      });
}

bool PlanningMap::IsPointInSignal(const double x, const double y,
                                  const double heading) {
  local_point_ = Point2D(x, y, heading);
  std::vector<SignalInfoConstPtr> signals;
  hdmap_->GetSignals(common::math::Vec2d(x, y), kRadius, &signals);
  for (const auto &signal : signals) {
    const auto &segments = signal->Segments();
    for (uint32_t i = 0; i < segments.size(); ++i) {
      const auto &segment = segments[i];
      if (segment.is_point_in(common::math::Vec2d(x, y)) ||
          segment.distance_to(common::math::Vec2d(x, y)) <=
              kObjectContainPointDistanceThreshold)
        return true;
    }
  }
  return false;
}

bool PlanningMap::IsPointInStopSign(const double x, const double y,
                                    const double heading) {
  local_point_ = Point2D(x, y, heading);
  std::vector<StopSignInfoConstPtr> stop_signs;
  hdmap_->GetStopSigns(common::math::Vec2d(x, y), kRadius, &stop_signs);
  for (const auto &stop_sign : stop_signs) {
    const auto &segments = stop_sign->Segments();
    for (uint32_t i = 0; i < segments.size(); ++i) {
      const auto &segment = segments[i];
      if (segment.is_point_in(common::math::Vec2d(x, y)) ||
          segment.distance_to(common::math::Vec2d(x, y)) <=
              kObjectContainPointDistanceThreshold)
        return true;
    }
  }
  return false;
}

bool PlanningMap::IsPointInYieldSign(const double x, const double y,
                                     const double heading) {
  local_point_ = Point2D(x, y, heading);
  std::vector<YieldSignInfoConstPtr> yieldsigns;
  hdmap_->GetYieldSignals(common::math::Vec2d(x, y), kRadius, &yieldsigns);
  for (const auto &yieldsign : yieldsigns) {
    const auto &segments = yieldsign->Segments();
    for (uint32_t i = 0; i < segments.size(); ++i) {
      const auto &segment = segments[i];
      if (segment.is_point_in(common::math::Vec2d(x, y)) ||
          segment.distance_to(common::math::Vec2d(x, y)) <=
              kObjectContainPointDistanceThreshold)
        return true;
    }
  }
  return false;
}

bool PlanningMap::IsPointInClearArea(const double x, const double y,
                                     const double heading) {
  local_point_ = Point2D(x, y, heading);
  std::vector<ClearAreaInfoConstPtr> clearareas;
  hdmap_->GetClearAreas(common::math::Vec2d(x, y), kRadius, &clearareas);
  return std::any_of(
      clearareas.begin(), clearareas.end(),
      [&](const ClearAreaInfoConstPtr cleararea) {
        return cleararea->Polygon().is_point_in(common::math::Vec2d(x, y));
      });
}

bool PlanningMap::IsPointInSpeedBump(const double x, const double y,
                                     const double heading) {
  local_point_ = Point2D(x, y, heading);
  std::vector<SpeedBumpInfoConstPtr> speedbumps;
  hdmap_->GetSpeedBumps(common::math::Vec2d(x, y), kRadius, &speedbumps);
  for (const auto &speedbump : speedbumps) {
    const auto &segments = speedbump->Segments();
    for (uint32_t i = 0; i < segments.size(); ++i) {
      const auto &each_segment = segments[i];
      if (each_segment.is_point_in(common::math::Vec2d(x, y)) ||
          each_segment.distance_to(common::math::Vec2d(x, y)) <=
              kObjectContainPointDistanceThreshold)
        return true;
    }
  }
  return false;
}

// mode : 0: white-dashed; 1: white-solid; 2: yellow- dashed; 3: yellow- solid;
// 4: double-yellow-solid
bool PlanningMap::CanCrossLane(const uint32_t &type, const uint32_t &color,
                               const int mode, bool &is_cross) {
  is_cross = false;
  DividerFeature::DividerColor color_t =
      static_cast<DividerFeature::DividerColor>(color);
  switch (static_cast<DividerFeature::DividerType>(type)) {
    case DividerFeature::DividerType::LONG_DASHED_LINE:
    case DividerFeature::DividerType::DASHED_LINE_SOLID_LINE:
    case DividerFeature::DividerType::SHORT_DASHED_LINE:
    case DividerFeature::DividerType::CURB_TRAVERSABLE:
    case DividerFeature::DividerType::DOUBLE_DASHED_LINE:
      if (color_t == DividerFeature::DividerColor::WHITE && mode > 0) {
        is_cross = true;
      } else if (color_t == DividerFeature::DividerColor::YELLOW && mode > 2) {
        is_cross = true;
      }
      break;
    case DividerFeature::DividerType::DOUBLE_SOLID_LINE:
      if (color_t == DividerFeature::DividerColor::WHITE && mode > 3) {
        is_cross = true;
      } else if (color_t == DividerFeature::DividerColor::YELLOW && mode > 4) {
        is_cross = true;
      }
      break;
    case DividerFeature::DividerType::SINGLE_SOLID_LINE:
    case DividerFeature::DividerType::SOLID_LINE_DASHED_LINE:
      if (mode > 3) {
        is_cross = true;
      } else if (mode >= 2 && color_t == DividerFeature::DividerColor::WHITE) {
        is_cross = true;
      }
      break;
    case DividerFeature::DividerType::UNKNOWN:
    case DividerFeature::DividerType::NO_MARKING:
    case DividerFeature::DividerType::SHADED_AREA_MARKING:
    case DividerFeature::DividerType::DASHED_BLOCKS:
    case DividerFeature::DividerType::CROSSING_ALERT:
    case DividerFeature::DividerType::CURB:
    case DividerFeature::DividerType::WALL_FLAT:
    case DividerFeature::DividerType::WALL_TUNNEL:
    case DividerFeature::DividerType::BARRIER_JERSEY:
    case DividerFeature::DividerType::BARRIER_SOUND:
    case DividerFeature::DividerType::BARRIER_CABLE:
    case DividerFeature::DividerType::GUARDRAIL:
    case DividerFeature::DividerType::FENCE:
    case DividerFeature::DividerType::END_OF_ROAD:
    case DividerFeature::DividerType::CLIFF:
    case DividerFeature::DividerType::DITCH:
    default:
      break;
  }
  return true;
}

void PlanningMap::BorrowLeftLaneBoundAggressive(
    ReferencePoint &point_utm, ReferencePoint &point_odometry) {
  double delta_s = point_utm.left_road_bound() - point_utm.left_lane_bound();
  if (delta_s < 3.0) {
    return;
  }
  double ratio = FLAGS_planning_lane_borrow_ratio;
  ratio = fmin(ratio, 1.0);
  ratio = fmax(0.0, ratio);
  double borrow_s = delta_s * ratio;
  borrow_s = std::fmax(borrow_s, 2.0);
  point_utm.set_left_bound(point_utm.left_lane_bound() + borrow_s);
  point_utm.set_left_lane_borrow(true);
  point_odometry.set_left_bound(point_odometry.left_lane_bound() + borrow_s);
  point_odometry.set_left_lane_borrow(true);
  return;
}

void PlanningMap::BorrowRightLaneBoundAggressive(
    ReferencePoint &point_utm, ReferencePoint &point_odometry) {
  double delta_s = point_utm.right_road_bound() - point_utm.right_lane_bound();
  if (delta_s < 3.0) {
    return;
  }
  double ratio = FLAGS_planning_lane_borrow_ratio;
  ratio = fmin(ratio, 1.0);
  ratio = fmax(0.0, ratio);
  double borrow_s = delta_s * ratio;
  borrow_s = std::fmax(borrow_s, 2.0);
  point_utm.set_right_bound(point_utm.right_lane_bound() + borrow_s);
  point_utm.set_right_lane_borrow(true);
  point_odometry.set_right_bound(point_odometry.right_lane_bound() + borrow_s);
  point_odometry.set_right_lane_borrow(true);
  return;
}

bool PlanningMap::GetLaneDividers(
    const uint64_t &lane_id, const double &s,
    std::map<double, DividerFeature> &left_divider,
    std::map<double, DividerFeature> &right_divider) const {
  auto lane = hdmap_->GetLaneById(lane_id);
  if (lane == nullptr) {
    return false;
  }
  using SFeatureMap = std::map<double, DividerFeature>;
  SFeatureMap left_features;
  SFeatureMap right_features;
  const auto &left_divider_features = lane->left_divider();
  const auto &right_divider_features = lane->right_divider();
  for (uint32_t i = 0; i < left_divider_features.size(); ++i) {
    const auto &each_feature = left_divider_features[i];
    left_features.emplace(
        each_feature.s,
        DividerFeature(
            each_feature.is_virtual,
            static_cast<DividerFeature::DividerColor>(each_feature.color),
            static_cast<DividerFeature::DividerType>(each_feature.type)));
  }
  for (uint32_t i = 0; i < right_divider_features.size(); ++i) {
    const auto &each_feature = right_divider_features[i];
    right_features.emplace(
        each_feature.s,
        DividerFeature(
            each_feature.is_virtual,
            static_cast<DividerFeature::DividerColor>(each_feature.color),
            static_cast<DividerFeature::DividerType>(each_feature.type)));
  }
  if (!left_features.empty()) {
    auto left_lower_it = left_features.lower_bound(s);
    if (left_lower_it != left_features.begin()) {
      if (left_lower_it == left_features.end() ||
          (left_lower_it != left_features.end() &&
           std::fabs(left_lower_it->first - s) > kMinDistance)) {
        auto left_prev_it = std::prev(left_lower_it);
        left_divider.emplace(left_prev_it->first, left_prev_it->second);
      }
    }
    while (left_lower_it != left_features.end()) {
      left_divider.emplace(left_lower_it->first, left_lower_it->second);
      left_lower_it++;
    }
  }
  if (!right_features.empty()) {
    auto right_lower_it = right_features.lower_bound(s);
    if (right_lower_it != right_features.begin()) {
      if (right_lower_it == right_features.end() ||
          (right_lower_it != right_features.end() &&
           std::fabs(right_lower_it->first - s) > kMinDistance)) {
        auto right_prev_it = std::prev(right_lower_it);
        right_divider.emplace(right_prev_it->first, right_prev_it->second);
      }
    }
    while (right_lower_it != right_features.end()) {
      right_divider.emplace(right_lower_it->first, right_lower_it->second);
      right_lower_it++;
    }
  }

  return true;
}

bool PlanningMap::GetLaneSignal(
    const uint64_t &lane_id, const double &start_s, const double &end_s,
    std::multimap<double, SignalInfoConstPtr> &signal_in_range) const {
  auto lane = hdmap_->GetLaneById(lane_id);
  if (lane == nullptr) {
    return false;
  }
  const auto &signals = lane->Signals();
  for (uint32_t i = 0; i < signals.size(); ++i) {
    const auto &each_signal = signals[i];
    if (each_signal.start_s >= start_s && each_signal.start_s <= end_s) {
      signal_in_range.emplace(each_signal.start_s,
                              hdmap_->GetSignalById(each_signal.object_id));
    }
  }
  return true;
}

void PlanningMap::SetCurrRoutingLaneIds(const std::vector<uint64_t> &holders) {
  curr_routing_lane_ids_ = holders;
}

bool UpperBoundCompare(double s, const cyberverse::DividerFeatureInfo &e) {
  return s < e.s;
}

bool PlanningMap::IsLaneLeftDistanceAccessibleWithMode(const uint64_t lane,
                                                       const double s,
                                                       const int mode) {
  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane) return false;

  const auto &features = ptr_lane->left_divider();
  auto index = features.upper_bound(s, UpperBoundCompare);
  if (index == features.size()) return false;

  auto &feature = features[index];
  using Type = autobot::cyberverse::DividerFeature::DividerType;
  using Color = autobot::cyberverse::DividerFeature::DividerColor;
  Type cmp_type = static_cast<Type>(feature.type);
  Color cmp_color = static_cast<Color>(feature.type);
  return (cmp_type == Type::DOUBLE_DASHED_LINE && cmp_color == Color::WHITE &&
          mode > 0) ||
         (cmp_type == Type::DOUBLE_DASHED_LINE && cmp_color == Color::YELLOW &&
          mode > 2) ||
         (cmp_type == Type::DOUBLE_SOLID_LINE && cmp_color == Color::WHITE &&
          mode > 3) ||
         (cmp_type == Type::DOUBLE_SOLID_LINE && cmp_color == Color::YELLOW &&
          mode > 4) ||
         (cmp_type == Type::SOLID_LINE_DASHED_LINE && mode > 3) ||
         (cmp_type == Type::SOLID_LINE_DASHED_LINE &&
          cmp_color == Color::WHITE && mode > 1);
}

bool PlanningMap::IsLaneRightDistanceAccessibleWithMode(const uint64_t lane,
                                                        const double s,
                                                        const int mode) {
  auto ptr_lane = hdmap_->GetLaneById(lane);
  if (!ptr_lane) return false;

  const auto &features = ptr_lane->right_divider();
  auto index = features.upper_bound(s, UpperBoundCompare);
  if (index == features.size()) return false;

  auto &feature = features[index];
  using Type = autobot::cyberverse::DividerFeature::DividerType;
  using Color = autobot::cyberverse::DividerFeature::DividerColor;
  Type cmp_type = static_cast<Type>(feature.type);
  Color cmp_color = static_cast<Color>(feature.type);
  return (cmp_type == Type::DOUBLE_DASHED_LINE && cmp_color == Color::WHITE &&
          mode > 0) ||
         (cmp_type == Type::DOUBLE_DASHED_LINE && cmp_color == Color::YELLOW &&
          mode > 2) ||
         (cmp_type == Type::DOUBLE_SOLID_LINE && cmp_color == Color::WHITE &&
          mode > 3) ||
         (cmp_type == Type::DOUBLE_SOLID_LINE && cmp_color == Color::YELLOW &&
          mode > 4) ||
         (cmp_type == Type::SOLID_LINE_DASHED_LINE && mode > 3) ||
         (cmp_type == Type::SOLID_LINE_DASHED_LINE &&
          cmp_color == Color::WHITE && mode > 1);
}

// color: 0: white; 1: yellow
bool PlanningMap::GetPointLeftRightLaneColor(const ReferencePoint &point,
                                             int &left_color,
                                             int &right_color) {
  if (point.hd_map_lane_id() == 0) {
    LOG_DEBUG("point.hd_map_lane_id is null");
    return false;
  }
  std::map<double, DividerFeature> left_neighbor;
  std::map<double, DividerFeature> right_neighbor;

  GetLaneDividers(point.hd_map_lane_id(), point.hd_map_lane_s(), left_neighbor,
                  right_neighbor);

  if (!left_neighbor.empty()) {
    auto iter = left_neighbor.begin();
    DividerFeature tmp_divider(iter->second);
    if (tmp_divider.divider_color_ == DividerFeature::DividerColor::YELLOW) {
      left_color = 1;
    } else {
      left_color = 0;
    }
  }

  if (!right_neighbor.empty()) {
    auto iter = right_neighbor.begin();
    DividerFeature tmp_divider(iter->second);
    if (tmp_divider.divider_color_ == DividerFeature::DividerColor::YELLOW) {
      right_color = 1;
    } else {
      right_color = 0;
    }
  }

  return true;
}

bool PlanningMap::IsLaneBelongToUturn(const uint64_t lane_id) {
  if (lane_id == 0) {
    return false;
  }
  auto tmp_lane = hdmap_->GetLaneById(lane_id);
  if (tmp_lane == nullptr) {
    return false;
  }
  if (static_cast<Lane::TurningType>(tmp_lane->TurnType()) ==
      Lane::TurningType::U_TURN) {
    return true;
  }
  return false;
}

bool PlanningMap::GetJunctionCorners(
    const uint64_t lane_id,
    std::vector<common::math::Vec2d> &junction_corners) {
  auto tmp_lane = hdmap_->GetLaneById(lane_id);
  if (tmp_lane == nullptr) {
    return false;
  }
  auto tmp_road = hdmap_->GetRoadById(tmp_lane->RoadId());
  if (tmp_road == nullptr) {
    LOG_DEBUG("road_link is null, lane id: {}", lane_id);
    return false;
  }
  if (tmp_road->JunctionId() == 0) {
    LOG_DEBUG("junction info is empty");
    return false;
  }
  JunctionInfoConstPtr junction_ptr =
      hdmap_->GetJunctionById(tmp_road->JunctionId());
  if (junction_ptr == nullptr || junction_ptr->Polygon().points().empty()) {
    LOG_DEBUG("junction points is empty, skip it");
    return false;
  }
  junction_corners.clear();
  const auto &polygon_points = junction_ptr->Polygon().points();
  junction_corners.reserve(polygon_points.size());
  for (uint32_t i = 0; i < polygon_points.size(); ++i) {
    const auto &pt = polygon_points[i];
    junction_corners.push_back({pt.x(), pt.y()});
  }
  return true;
}

bool PlanningMap::GetJunctionId(const ReferencePoint &point,
                                uint64_t &junction_id) {
  // this function failed. always return false.
  return false;
}

bool PlanningMap::GetJunctionById(const uint64_t junction_id,
                                  JunctionInfoConstPtr &junciton_ptr) {
  junciton_ptr = hdmap_->GetJunctionById(junction_id);
  if (nullptr == junciton_ptr) {
    return false;
  }
  return true;
}

bool PlanningMap::GetJunction(const ReferencePoint &point,
                              JunctionInfoConstPtr &junction_ptr) {
  std::vector<JunctionInfoConstPtr> junctions;
  hdmap_->Getjunctions(common::math::Vec2d{point.x(), point.y()}, 1.0,
                       &junctions);
  if (0 == junctions.size()) {
    return false;
  }
  junction_ptr = junctions.front();
  return true;
}

bool PlanningMap::IsNearJunction(const double x, const double y,
                                 const double radius) {
  std::vector<JunctionInfoConstPtr> junctions;
  hdmap_->Getjunctions(common::math::Vec2d{x, y}, radius, &junctions);
  return !junctions.empty();
}

bool PlanningMap::GetParkingSpace(
    const Vec2d &point, cyberverse::ParkingSpaceInfoConstPtr &park_space_ptr) {
  park_space_ptr =
      hdmap_->GetNearestParkingSpace(common::math::Vec2d{point.x(), point.y()});
  if (nullptr == park_space_ptr) {
    return false;
  }
  return true;
}

bool PlanningMap::GetParkingSpaceById(
    const uint64_t parking_id,
    cyberverse::ParkingSpaceInfoConstPtr &park_space_ptr) {
  park_space_ptr = hdmap_->GetParkingSpaceById(parking_id);
  if (park_space_ptr == nullptr) {
    return false;
  }
  return true;
}

bool PlanningMap::GetPortOdd(
    const Vec2d &point,
    cyberverse::DrivingZoneInfoConstPtr &driving_zone_info_ptr) {
  driving_zone_info_ptr =
      hdmap_->GetPortOdd(common::math::Vec2d{point.x(), point.y()});
  if (nullptr == driving_zone_info_ptr) {
    return false;
  }
  return true;
}

bool PlanningMap::IsInPortOdd(const Vec2d &point) {
  return hdmap_->GetPortOdd(common::math::Vec2d{point.x(), point.y()}) !=
         nullptr;
}

bool PlanningMap::GetLaneNeighbors(
    const uint64_t &lane_id, std::vector<uint64_t> &left_forwards_lanes,
    std::vector<uint64_t> &right_forwards_lanes,
    std::vector<uint64_t> &left_backwards_lanes,
    std::vector<uint64_t> &right_backwards_lanes) const {
  auto lane = hdmap_->GetLaneById(lane_id);
  if (lane == nullptr) {
    return false;
  }
  std::vector<cyberverse::LaneRelation> left_forwards;
  std::vector<cyberverse::LaneRelation> right_forwards;
  std::vector<cyberverse::LaneRelation> left_backwards;
  std::vector<cyberverse::LaneRelation> right_backwards;
  hdmap_->TopologicalLaneConnections().GetLeftForwardNeighborsOf(lane->Id(),
                                                                 left_forwards);
  hdmap_->TopologicalLaneConnections().GetRightForwardNeighborsOf(
      lane->Id(), right_forwards);
  hdmap_->TopologicalLaneConnections().GetLeftBackwardNeighborsOf(
      lane->Id(), left_backwards);
  hdmap_->TopologicalLaneConnections().GetRightBackwardNeighborsOf(
      lane->Id(), right_backwards);

  for (const auto &lf : left_forwards) {
    left_forwards_lanes.push_back(lf.to());
  }
  for (const auto &rf : right_forwards) {
    right_forwards_lanes.push_back(rf.to());
  }
  for (const auto &lb : left_backwards) {
    left_backwards_lanes.push_back(lb.to());
  }
  for (const auto &rb : right_backwards) {
    right_backwards_lanes.push_back(rb.to());
  }
  return true;
}

bool PlanningMap::GetNearestLeftLane(uint64_t lane_id,
                                     const common::math::Vec2d &point,
                                     uint64_t &res) {
  std::vector<uint64_t> left_lanes;
  GetLeftLanes(lane_id, left_lanes);
  std::pair<int, double> min_dis = {-1,
                                    std::numeric_limits<double>::infinity()};
  for (auto i = 0; i < left_lanes.size(); i++) {
    auto l = hdmap_->GetLaneById(left_lanes[i]);
    auto dis = l->DistanceTo(point);
    if (dis < min_dis.second) min_dis = {i, dis};
  }
  if (min_dis.first == -1) {
    res = 0;
    return false;
  }
  res = left_lanes[min_dis.first];
  return true;
}

bool PlanningMap::GetNearestRightLane(uint64_t lane_id,
                                      const common::math::Vec2d &point,
                                      uint64_t &res) {
  std::vector<uint64_t> right_lanes;
  GetRightLanes(lane_id, right_lanes);
  std::pair<int, double> min_dis = {-1,
                                    std::numeric_limits<double>::infinity()};
  for (auto i = 0; i < right_lanes.size(); i++) {
    auto l = hdmap_->GetLaneById(right_lanes[i]);
    auto dis = l->DistanceTo(point);
    if (dis < min_dis.second) min_dis = {i, dis};
  }
  if (min_dis.first == -1) {
    res = 0;
    return false;
  }
  res = right_lanes[min_dis.first];
  return true;
}

bool PlanningMap::GetInterp1PointInLane(uint64_t ego_lid, uint64_t merging_lid,
                                        bool is_left, math::AD2 &near_pt,
                                        math::AD2 &rear_pt, bool is_near,
                                        double near_s, double rear_s) {
  auto l_ptr = hdmap_->GetLaneById(ego_lid);
  auto m_ptr = hdmap_->GetLaneById(merging_lid);
  if (l_ptr == nullptr || m_ptr == nullptr) return false;

  constexpr double HalfWidthThreshold = 0.5;
  double tmp = 0.0;
  if (near_s < -1e-9) {
    l_ptr->GetProjection(common::math::Vec2d(near_pt[0], near_pt[1]), &near_s,
                         &tmp);
  }
  if (rear_s < -1e-9) {
    l_ptr->GetProjection(common::math::Vec2d(rear_pt[0], rear_pt[1]), &rear_s,
                         &tmp);
  }

  auto &boundary =
      is_left ? l_ptr->left_divider_polygon() : l_ptr->right_divider_polygon();
  auto &pts = boundary.Points();
  LOG_INFO("projection: {},{}, size: {}", near_s, rear_s, pts.size());

  auto BinarySearchIndexInPolyline = [&boundary](int l, int r, double t) {
    int mid;
    int res;
    while (l + 1 < r) {
      mid = (l + r) / 2;
      res = math::Sign(
          (mid == boundary.Points().size() - 1 ? boundary.Length()
                                               : boundary.AccumulatedS()[mid]) -
              t,
          1e-9);
      LOG_INFO("{},{},{}", l, r, res);
      switch (res) {
        case -1:
          l = mid;
          break;
        case 0:
          l = r = mid;
          break;
        case 1:
          r = mid - 1;
          break;
      }
    }
    return l;
  };
  auto CalculateResDis = [&m_ptr, &pts, &tmp](double x, double y) {
    double s = 0.0;
    m_ptr->GetProjection(common::math::Vec2d(x, y), &s, &tmp);
    return m_ptr->GetWidth(s) / 2.0 - tmp;
  };
  auto Interp1 = [](double scale, const math::AD2 &sp, const math::AD2 &ep,
                    math::AD2 &ans) {
    double rscale = 1 - scale;
    ans[0] = rscale * sp[0] + scale * ep[0];
    ans[1] = rscale * sp[1] + scale * ep[1];
  };
  auto BinarySearchIndexInSegment =
      [&boundary, &Interp1, &CalculateResDis, &HalfWidthThreshold](
          double l, double r, double t, const math::AD2 &sp,
          const math::AD2 &ep, math::AD2 &ans) {
        double eps = 1e-2;
        int cnt = 0;
        while (r - l > eps) {
          double mid = (l + r) / 2;
          Interp1(mid, sp, ep, ans);
          double resd = CalculateResDis(ans[0], ans[1]);
          int type = math::Sign(resd - HalfWidthThreshold, eps);
          switch (type) {
            case -1:
              l = mid;
              break;
            case 0:
              l = r = mid;
              break;
            case 1:
              r = mid;
              break;
          }
          if (++cnt > 1000) break;
        }
        return cnt;
      };

  double tar = 0.0;
  int near_idx = BinarySearchIndexInPolyline(0, pts.size() - 1, near_s);
  int rear_idx =
      BinarySearchIndexInPolyline(near_idx, pts.size() - 1, rear_s) + 1;
  int step = is_near ? 1 : -1;
  int begin = is_near ? near_idx : rear_idx;
  int end = is_near ? rear_idx : near_idx;
  math::AD2 ans{pts[begin].x(), pts[begin].y()};
  int l_cnt = 0;
  LOG_INFO("LOOP: {},{},{}", begin, end, step);
  while (begin != end + step) {
    begin += step;
    if (begin == end + step) break;  // no search result
    double resd = CalculateResDis(pts[begin].x(), pts[begin].y());
    if (resd > HalfWidthThreshold) {
      int cnt = BinarySearchIndexInSegment(
          1e-9, 1.0 - 1e-9, HalfWidthThreshold,
          {pts[begin - step].x(), pts[begin - step].y()},
          {pts[begin].x(), pts[begin].y()}, ans);
      if (cnt > 1000) {
        ans[0] = pts[begin].x();
        ans[1] = pts[begin].y();
        LOG_INFO("Binary search is failed[{} times].", cnt);
        return false;
      } else {
        if (is_near) {
          near_pt[0] = ans[0];
          near_pt[1] = ans[1];
        } else {
          rear_pt[0] = ans[0];
          rear_pt[1] = ans[1];
        }
        return true;
      }
    }
    if (++l_cnt > pts.size()) {
      LOG_INFO("Binary search is failed[{} times].", l_cnt);
      return false;
    }
  }
  return false;
}

bool PlanningMap::GetInterp1PointInLane(uint64_t lane_id, double s,
                                        common::math::Vec2d &res) {
  auto lane_ptr = hdmap_->GetLaneById(lane_id);
  if (lane_ptr == nullptr) return false;
  if (std::abs(s) > lane_ptr->TotalLength()) s = lane_ptr->TotalLength();
  s = s > 0 ? lane_ptr->TotalLength() - s : -s;
  s = s < 0 ? 0 : s;
  const auto &seg = lane_ptr->Segments();
  double tar = 0.0;
  for (int i = seg.size() - 1; i >= 0; i--) {
    if (math::Sign((tar += seg[i].length()) - s) >= 0) {
      auto nVec = seg[i].unit_direction();
      res.set_x(seg[i].start().x() + (tar - s) * nVec.x());
      res.set_y(seg[i].start().y() + (tar - s) * nVec.y());
      return true;
    }
  }
  return false;
}

bool PlanningMap::GetSamplePointFromConnection(
    const TrafficConflictZoneContext &czContext, double forwardDistance,
    common::math::Vec2d &res) {
  if (czContext.current_lane.lane_ptr == nullptr ||
      czContext.next_lane.lane_ptr == nullptr) {
    return false;
  } else {
    int idx = 0;
    while (forwardDistance > czContext.route_length_to_zone[idx]) idx++;
    if (idx > 1) {
      res.set_x(czContext.next_lane.lane_ptr->Points().back().x());
      res.set_y(czContext.next_lane.lane_ptr->Points().back().y());
      return false;
    }
    forwardDistance = czContext.route_length_to_zone[idx] - forwardDistance;
    return GetInterp1PointInLane(
        idx == 0 ? czContext.current_lane.id : czContext.next_lane.id,
        -forwardDistance, res);
  }
}

bool PlanningMap::GetRSInLane(const MergingStopLineInfo &merging, double &s,
                              const common::math::Vec2d &res) {
  auto veh2odom = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto odom_pose =
        DataCenter::Instance()->pose_base_link_in_odometry_msg.ptr->pose();
    double odom_yaw = common::GetYawFromPose(odom_pose);
    double dx = pt.x();
    double dy = pt.y();
    double sin_ = std::sin(odom_yaw);
    double cos_ = std::cos(odom_yaw);
    return {{-dy * sin_ + dx * cos_ + odom_pose.position().x(),
             dx * sin_ + dy * cos_ + odom_pose.position().y()},
            heading + odom_yaw};
  };
  auto utm2veh = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto utm_pose =
        DataCenter::Instance()->pose_base_link_in_utm_msg.ptr->pose();
    double utm_yaw = common::GetYawFromPose(utm_pose);
    double dx = pt.x() - utm_pose.position().x();
    double dy = pt.y() - utm_pose.position().y();
    double sin_ = std::sin(utm_yaw);
    double cos_ = std::cos(utm_yaw);
    return {{dy * sin_ + dx * cos_, -dx * sin_ + dy * cos_}, heading - utm_yaw};
  };
  auto veh2utm = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto utm_pose =
        DataCenter::Instance()->pose_base_link_in_utm_msg.ptr->pose();
    double utm_yaw = common::GetYawFromPose(utm_pose);
    double dx = pt.x();
    double dy = pt.y();
    double sin_ = std::sin(utm_yaw);
    double cos_ = std::cos(utm_yaw);
    return {{-dy * sin_ + dx * cos_ + utm_pose.position().x(),
             dx * sin_ + dy * cos_ + utm_pose.position().y()},
            heading + utm_yaw};
  };
  auto odom2veh = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto odom_pose =
        DataCenter::Instance()->pose_base_link_in_odometry_msg.ptr->pose();
    double odom_yaw = common::GetYawFromPose(odom_pose);
    double dx = pt.x() - odom_pose.position().x();
    double dy = pt.y() - odom_pose.position().y();
    double sin_ = std::sin(odom_yaw);
    double cos_ = std::cos(odom_yaw);
    return {{dy * sin_ + dx * cos_, -dx * sin_ + dy * cos_},
            heading - odom_yaw};
  };
  auto odom2utm = [&veh2utm, &odom2veh](
                      Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto tmp = odom2veh(pt, heading);
    return veh2utm(tmp.first, tmp.second);
  };
  auto mptr = hdmap_->GetLaneById(merging.lane_id);
  if (mptr == nullptr) return false;
  double dis[2] = {0.0, 0.0};
  auto nres = odom2utm(Vec2d(res.x(), res.y()), atan2(res.y(), res.x())).first;
  mptr->GetProjection(common::math::Vec2d(nres.x(), nres.y()), &dis[0],
                      &dis[1]);
  s = mptr->TotalLength() - dis[0] +
      (merging.root_lane_id == 0 ? 0 : merging.sum_length) - merging.merging_s;
  return true;
}

bool PlanningMap::GetRSInLane(const MeetingLaneContext &meeting, double &s,
                              const common::math::Vec2d &res) {
  auto VehToOdom = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto odom_pose =
        DataCenter::Instance()->pose_base_link_in_odometry_msg.ptr->pose();
    double odom_yaw = common::GetYawFromPose(odom_pose);
    double dx = pt.x();
    double dy = pt.y();
    double sin_ = std::sin(odom_yaw);
    double cos_ = std::cos(odom_yaw);
    return {{-dy * sin_ + dx * cos_ + odom_pose.position().x(),
             dx * sin_ + dy * cos_ + odom_pose.position().y()},
            heading + odom_yaw};
  };
  auto UtmToVeh = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto utm_pose =
        DataCenter::Instance()->pose_base_link_in_utm_msg.ptr->pose();
    double utm_yaw = common::GetYawFromPose(utm_pose);
    double dx = pt.x() - utm_pose.position().x();
    double dy = pt.y() - utm_pose.position().y();
    double sin_ = std::sin(utm_yaw);
    double cos_ = std::cos(utm_yaw);
    return {{dy * sin_ + dx * cos_, -dx * sin_ + dy * cos_}, heading - utm_yaw};
  };
  auto VehToUtm = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto utm_pose =
        DataCenter::Instance()->pose_base_link_in_utm_msg.ptr->pose();
    double utm_yaw = common::GetYawFromPose(utm_pose);
    double dx = pt.x();
    double dy = pt.y();
    double sin_ = std::sin(utm_yaw);
    double cos_ = std::cos(utm_yaw);
    return {{-dy * sin_ + dx * cos_ + utm_pose.position().x(),
             dx * sin_ + dy * cos_ + utm_pose.position().y()},
            heading + utm_yaw};
  };
  auto OdomToVeh = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto odom_pose =
        DataCenter::Instance()->pose_base_link_in_odometry_msg.ptr->pose();
    double odom_yaw = common::GetYawFromPose(odom_pose);
    double dx = pt.x() - odom_pose.position().x();
    double dy = pt.y() - odom_pose.position().y();
    double sin_ = std::sin(odom_yaw);
    double cos_ = std::cos(odom_yaw);
    return {{dy * sin_ + dx * cos_, -dx * sin_ + dy * cos_},
            heading - odom_yaw};
  };
  auto OdomToUtm = [&VehToUtm, &OdomToVeh](
                       Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto tmp = OdomToVeh(pt, heading);
    return VehToUtm(tmp.first, tmp.second);
  };
  auto mptr = hdmap_->GetLaneById(meeting.lane_id);
  if (mptr == nullptr) return false;
  double dis[2] = {0.0, 0.0};
  auto nres = OdomToUtm(Vec2d(res.x(), res.y()), atan2(res.y(), res.x())).first;
  mptr->GetProjection(common::math::Vec2d(nres.x(), nres.y()), &dis[0],
                      &dis[1]);
  s = meeting.root_lane_id == 0
          ? meeting.merging_s[0] - dis[0]
          : meeting.sum_length + mptr->TotalLength() - dis[0];
  return true;
}

bool PlanningMap::GetResDistanceInLane(const MeetingLaneContext &meeting,
                                       const uint64_t &id, double &rd,
                                       const Vec2d &res) {
  auto veh2odom = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto odom_pose =
        DataCenter::Instance()->pose_base_link_in_odometry_msg.ptr->pose();
    double odom_yaw = common::GetYawFromPose(odom_pose);
    double dx = pt.x();
    double dy = pt.y();
    double sin_ = std::sin(odom_yaw);
    double cos_ = std::cos(odom_yaw);
    return {{-dy * sin_ + dx * cos_ + odom_pose.position().x(),
             dx * sin_ + dy * cos_ + odom_pose.position().y()},
            heading + odom_yaw};
  };
  auto utm2veh = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto utm_pose =
        DataCenter::Instance()->pose_base_link_in_utm_msg.ptr->pose();
    double utm_yaw = common::GetYawFromPose(utm_pose);
    double dx = pt.x() - utm_pose.position().x();
    double dy = pt.y() - utm_pose.position().y();
    double sin_ = std::sin(utm_yaw);
    double cos_ = std::cos(utm_yaw);
    return {{dy * sin_ + dx * cos_, -dx * sin_ + dy * cos_}, heading - utm_yaw};
  };
  auto veh2utm = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto utm_pose =
        DataCenter::Instance()->pose_base_link_in_utm_msg.ptr->pose();
    double utm_yaw = common::GetYawFromPose(utm_pose);
    double dx = pt.x();
    double dy = pt.y();
    double sin_ = std::sin(utm_yaw);
    double cos_ = std::cos(utm_yaw);
    return {{-dy * sin_ + dx * cos_ + utm_pose.position().x(),
             dx * sin_ + dy * cos_ + utm_pose.position().y()},
            heading + utm_yaw};
  };
  auto odom2veh = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto odom_pose =
        DataCenter::Instance()->pose_base_link_in_odometry_msg.ptr->pose();
    double odom_yaw = common::GetYawFromPose(odom_pose);
    double dx = pt.x() - odom_pose.position().x();
    double dy = pt.y() - odom_pose.position().y();
    double sin_ = std::sin(odom_yaw);
    double cos_ = std::cos(odom_yaw);
    return {{dy * sin_ + dx * cos_, -dx * sin_ + dy * cos_},
            heading - odom_yaw};
  };
  auto odom2utm = [&veh2utm, &odom2veh](
                      Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto tmp = odom2veh(pt, heading);
    return veh2utm(tmp.first, tmp.second);
  };
  auto mptr = hdmap_->GetLaneById(id);
  if (mptr == nullptr) return false;
  double dis[2] = {0.0, 0.0};
  // auto nres = odom2utm(Vec2d(res.x(), res.y()), atan2(res.y(),
  // res.x())).first;
  mptr->GetProjection(common::math::Vec2d(res.x(), res.y()), &dis[0], &dis[1]);
  rd = meeting.ego_s[0] - dis[0];
  return true;
}

void VisObstacles(const neodrive::planning::ReferenceLinePtr &ref_ptr,
                  neodrive::planning::ReferencePoint &refPoint,
                  std::vector<math::Polygon> &sucPolyVec,
                  std::vector<math::Polygon> &merPolyVec,
                  std::vector<math::Polygon> &curLane,
                  std::array<ConflictLaneContext, 2> laneInfo,
                  std::array<std::pair<uint64_t, std::set<MergingStopLineInfo>>,
                             2> &ConnectionMergingGeoInfo,
                  std::set<ConflictLaneContext> &mergingids,
                  const TrafficConflictZoneContext &conflictZone) {
  auto set_rgba = [](auto &event, auto &rgb) {
    event->set_type(visualizer::Event::k3D);
    event->add_attribute(visualizer::Event::kOdom);
    event->mutable_color()->set_r(rgb[0]);
    event->mutable_color()->set_g(rgb[1]);
    event->mutable_color()->set_b(rgb[2]);
    event->mutable_color()->set_a(rgb[3]);
  };
  auto FillPoint = [](auto t, auto x, auto y, auto z) {
    t->set_x(x), t->set_y(y), t->set_z(z);
  };
  auto odom2veh = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto odom_pose =
        DataCenter::Instance()->pose_base_link_in_odometry_msg.ptr->pose();
    double odom_yaw = common::GetYawFromPose(odom_pose);
    double dx = pt.x() - odom_pose.position().x();
    double dy = pt.y() - odom_pose.position().y();
    double sin_ = std::sin(odom_yaw);
    double cos_ = std::cos(odom_yaw);
    return {{dy * sin_ + dx * cos_, -dx * sin_ + dy * cos_},
            heading - odom_yaw};
  };
  auto veh2odom = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto odom_pose =
        DataCenter::Instance()->pose_base_link_in_odometry_msg.ptr->pose();
    double odom_yaw = common::GetYawFromPose(odom_pose);
    double dx = pt.x();
    double dy = pt.y();
    double sin_ = std::sin(odom_yaw);
    double cos_ = std::cos(odom_yaw);
    return {{-dy * sin_ + dx * cos_ + odom_pose.position().x(),
             dx * sin_ + dy * cos_ + odom_pose.position().y()},
            heading + odom_yaw};
  };
  auto utm2veh = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto utm_pose =
        DataCenter::Instance()->pose_base_link_in_utm_msg.ptr->pose();
    double utm_yaw = common::GetYawFromPose(utm_pose);
    double dx = pt.x() - utm_pose.position().x();
    double dy = pt.y() - utm_pose.position().y();
    double sin_ = std::sin(utm_yaw);
    double cos_ = std::cos(utm_yaw);
    return {{dy * sin_ + dx * cos_, -dx * sin_ + dy * cos_}, heading - utm_yaw};
  };
  auto veh2utm = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto utm_pose =
        DataCenter::Instance()->pose_base_link_in_utm_msg.ptr->pose();
    double utm_yaw = common::GetYawFromPose(utm_pose);
    double dx = pt.x();
    double dy = pt.y();
    double sin_ = std::sin(utm_yaw);
    double cos_ = std::cos(utm_yaw);
    return {{-dy * sin_ + dx * cos_ + utm_pose.position().x(),
             dx * sin_ + dy * cos_ + utm_pose.position().y()},
            heading + utm_yaw};
  };
  auto utm2odom = [&utm2veh, &veh2odom](
                      Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto tmp = utm2veh(pt, heading);
    return veh2odom(tmp.first, tmp.second);
  };

  if (!FLAGS_planning_enable_vis_event) return;
  std::vector<visualizer::Event *> events;
  std::vector<std::array<double, 4>> rgb;
  events.emplace_back(vis::EventSender::Instance()->GetEvent("referenceLine"));
  rgb.push_back({0.4, 0.152, 0.0, 0.5});
  events.emplace_back(
      vis::EventSender::Instance()->GetEvent("ConnectionEndFind"));
  rgb.push_back({0.0, 0.297, 0.4, 0.5});
  events.emplace_back(vis::EventSender::Instance()->GetEvent("Successor"));
  rgb.push_back({0.0, 0.006, 0.4, 0.4});
  events.emplace_back(vis::EventSender::Instance()->GetEvent("Merging"));
  rgb.push_back({1.0, 0.27, 0.27, 0.4});
  events.emplace_back(vis::EventSender::Instance()->GetEvent("CurLane"));
  rgb.push_back({0.4, 0.152, 0.0, 0.4});
  events.emplace_back(vis::EventSender::Instance()->GetEvent("Lanekappa"));
  rgb.push_back({0.0, 0.297, 0.4, 0.8});
  events.emplace_back(vis::EventSender::Instance()->GetEvent("SamplePoints"));
  rgb.push_back({0.5, 1, 0.56, 1.0});
  events.emplace_back(
      vis::EventSender::Instance()->GetEvent("ConnectionEndFind2"));
  rgb.push_back({1, 0, 0.56, 1.0});
  for (auto i = 0; i < events.size(); i++) set_rgba(events[i], rgb[i]);

  for (int i = 0; i < 1; i++) {
    auto polygon = events[i]->add_polyline();
    for (auto &p : ref_ptr->ref_points())
      FillPoint(polygon->add_point(), p.x(), p.y(), 0);
  }

  for (int i = 1; i < 2; i++) {
    auto sphere = events[i]->add_sphere();
    FillPoint(sphere->mutable_center(), refPoint.x(), refPoint.y(), 0);
    sphere->set_radius(1);
  }

  static char cntString[20];
  for (int i = 2; i < 3; i++) {
    for (auto &poly : sucPolyVec) {
      auto polygon = events[i]->add_polygon();
      int cnt = 0;
      for (auto &p : poly.points()) {
        FillPoint(polygon->add_point(), p[0], p[1], 0);
        auto text = events[i]->add_text();
        sprintf(cntString, "%d", cnt++);
        text->set_text(cntString);
        FillPoint(text->mutable_position(), p[0], p[1], 0);
      }
    }
  }

  for (int i = 3; i < 4; i++) {
    for (auto &poly : merPolyVec) {
      auto polygon = events[i]->add_polygon();
      for (auto &p : poly.points())
        FillPoint(polygon->add_point(), p[0], p[1], 0);
    }
  }

  for (int i = 4; i < 5; i++) {
    for (auto &poly : curLane) {
      auto polygon = events[i]->add_polygon();
      for (auto &p : poly.points())
        FillPoint(polygon->add_point(), p[0], p[1], 0);
    }
  }

  for (int i = 5; i < 6; i++) {
    for (int j = 0; j < 2; j++) {
      auto p = laneInfo[j].lane_ptr->Points().back();
      auto np = utm2odom(Vec2d(p.x(), p.y()), p.angle());
      auto text = events[i]->add_text();
      text->set_text(laneInfo[j].is_stright ? "Straight" : "Curve");
      FillPoint(text->mutable_position(), np.first.x(), np.first.y(), 0);
    }
  }

  for (int i = 5; i < 6; i++) {
    for (int j = 0; j < 2; j++) {
      if (ConnectionMergingGeoInfo[j].first == 0) continue;
      for (auto &item : ConnectionMergingGeoInfo[j].second) {
        auto sphere = events[i]->add_sphere();
        auto np = utm2odom(Vec2d(item.intersection[0], item.intersection[1]),
                           atan2(item.intersection[1], item.intersection[0]))
                      .first;
        FillPoint(sphere->mutable_center(), np.x(), np.y(), 0);
        sphere->set_radius(1);

        ConflictLaneContext tmp(0);
        tmp.id = item.lane_id;
        auto it = mergingids.find(tmp);
        if (it == mergingids.end() || it->lane_ptr == nullptr) continue;
        auto &pt = it->lane_ptr->Segments().front();
        np = utm2odom(Vec2d(pt.end().x() + 3 * pt.unit_direction().y(),
                            pt.end().y() - 3 * pt.unit_direction().x()),
                      atan2(pt.end().y() - 3 * pt.unit_direction().x(),
                            pt.end().x() + 3 * pt.unit_direction().y()))
                 .first;
        auto text = events[i]->add_text();
        double res;
        // PlanningMap::Instance()->GetRSInLane(item, res,
        // common::math::Vec2d(pt.start().x(), pt.start().y()));
        sprintf(cntString, "%.2f", item.sum_length);
        text->set_text(cntString);
        FillPoint(text->mutable_position(), np.x(), np.y(), 0);
      }
    }
  }

  for (int i = 6; i < 7; i++) {
    for (int cnt = 1; cnt < 10; cnt++) {
      common::math::Vec2d res;
      auto pm = PlanningMap::Instance();
      if (pm->GetSamplePointFromConnection(conflictZone, cnt * 1.5, res)) {
        auto sphere = events[i]->add_sphere();
        auto np =
            utm2odom(Vec2d(res.x(), res.y()), atan2(res.y(), res.x())).first;
        FillPoint(sphere->mutable_center(), np.x(), np.y(), 0);
        sphere->set_radius(1);
      } else
        break;
    }
  }

  auto CalcBoundaryPts = [&utm2odom, &FillPoint](auto &pt, auto &event,
                                                 auto num) {
    auto sphere = event->add_sphere();
    auto np = utm2odom(Vec2d(pt[0], pt[1]), atan2(pt[1], pt[0])).first;
    FillPoint(sphere->mutable_center(), np.x(), np.y(), 0);
    sphere->set_radius(0.35);
    auto text = event->add_text();
    std::string nums = "0";
    nums[0] += num;
    text->set_text(nums);
    FillPoint(text->mutable_position(),
              np.x() - 0.5 * math::Sign(num % 2 - 0.5),
              np.y() + 0.5 * math::Sign((num > 1) - 0.5), 0);
  };

  for (int i = 7; i < 8; i++) {
    for (int j = 0; j < 2; j++) {
      auto meeting = conflictZone.meeting_geoinfo[j];
      for (auto it = meeting->begin(); it != meeting->end(); it++) {
        for (int k = 0; k < 4; k++) {
          CalcBoundaryPts(it->first.zone_polygon.points()[k], events[i],
                          k == 0 ? 0 : (k % 3 + 1));
        }
      }
    }
  }

  LOG_DEBUG("Visualizer finished");
}

bool PlanningMap::GetConflictZone(TaskInfo &task_info, const Vec2d &egoPt) {
  constexpr double kNtypeDetectThreshold = 55.0;
  double safeDistance =
      std::max(task_info.current_frame()->inside_planner_data().vel_v, 8.5) *
      4;                             // 30.6 km/h * 4s = 34 m;
  double kappaThreshold = 1.0 / 30;  // radius = 30 m
  std::set<ConflictLaneContext> successorLaneInfos, mergingLaneInfos;
  std::vector<std::pair<std::pair<uint64_t, uint64_t>, bool>> con1Successor,
      con2Successor, con1Merging, con2Merging;
  std::array<std::vector<std::pair<std::pair<uint64_t, uint64_t>,
                                   std::pair<bool, double>>>,
             2>
      overlapMerging;
  ReferencePoint nearPoint, rearPoint, finalPoint;
  std::array<cyberverse::LaneInfoConstPtr, 2> lanes{nullptr, nullptr};
  std::array<int, 2> successorLaneNum{0, 0}, mergingLaneNum{0, 0}, state{0, 0};
  std::array<uint64_t, 2> laneId{0, 0}, juncId{0, 0};
  uint64_t thirdlaneId{0};
  std::array<double, 3> restRouteLength;
  std::array<TrafficConflictZoneContext::connectionType, 3> type{
      TrafficConflictZoneContext::connectionType::Unknown,
      TrafficConflictZoneContext::connectionType::Unknown,
      TrafficConflictZoneContext::connectionType::Unknown};
  std::array<std::pair<uint64_t, std::set<MergingStopLineInfo>> *, 2> sLine;
  std::array<
      std::pair<uint64_t, std::map<MeetingLaneContext, ConflictLaneContext>> *,
      2>
      jInfo;
  auto &ref_ptr = task_info.reference_line();
  auto &conflictZone = task_info.current_frame()
                           ->mutable_outside_planner_data()
                           ->traffic_conflict_zone_context;
  auto getReferencePoint = [&ref_ptr](double s,
                                      ReferencePoint *reference_point) {
    if (!ref_ptr->GetNearestRefPoint(s, reference_point)) {
      LOG_INFO("GetNearestRefPoint [Failed]");
      return false;
    }
    return true;
  };
  auto GetSuccessor =
      [sfunc = [](auto &&self, cyberverse::HDMap &hdmap, double safeDistance,
                  uint64_t srcId, uint64_t laneId, double sumS,
                  std::vector<std::pair<std::pair<uint64_t, uint64_t>, bool>>
                      &successor_) -> void {
        auto lane = hdmap.GetLaneById(laneId);
        sumS += lane->TotalLength();
        if (sumS < safeDistance) {
          std::vector<cyberverse::LaneRelation> successorLaneConnection;
          hdmap.TopologicalLaneConnections().GetLaneEndConnectionsFrom(
              laneId, successorLaneConnection);
          for (auto &connection : successorLaneConnection) {
            if (connection.from() == laneId) {
              successor_.push_back({{connection.to(), srcId}, false});
              self(self, hdmap, safeDistance, srcId,
                   successor_.back().first.first, sumS, successor_);
            }
          }
        }
      }](cyberverse::HDMap &hdmap, double safeDistance, uint64_t srcId,
         uint64_t laneId,
         std::vector<std::pair<std::pair<uint64_t, uint64_t>, bool>>
             &successor_) -> void {
    sfunc(sfunc, hdmap, safeDistance, srcId, laneId, 0, successor_);
  };
  auto GetMerging =
      [mfunc = [](auto &&self, cyberverse::HDMap &hdmap, double safeDistance,
                  uint64_t srcId, uint64_t laneId, double sumS,
                  std::vector<std::pair<std::pair<uint64_t, uint64_t>, bool>>
                      &merging_) -> void {
        auto lane = hdmap.GetLaneById(laneId);
        sumS += lane->TotalLength();
        if (sumS < safeDistance) {
          std::vector<cyberverse::LaneRelation> mergingLaneConnection;
          hdmap.TopologicalLaneConnections().GetLaneEndConnectionsTo(
              laneId, mergingLaneConnection);
          for (auto &connection : mergingLaneConnection) {
            if (connection.from() == laneId) {
              merging_.push_back({{connection.to(), srcId}, false});
              self(self, hdmap, safeDistance, srcId,
                   merging_.back().first.first, sumS, merging_);
            }
          }
        }
      }](cyberverse::HDMap &hdmap, double safeDistance, uint64_t srcId,
         uint64_t laneId,
         std::vector<std::pair<std::pair<uint64_t, uint64_t>, bool>> &merging_)
      -> void {
    mfunc(mfunc, hdmap, safeDistance, srcId, laneId,
          -hdmap.GetLaneById(srcId)->TotalLength(), merging_);
  };
  auto GetMeeting =
      [mfunc2 = [](auto &&self, cyberverse::HDMap &hdmap, double safeDistance,
                   uint64_t srcId, uint64_t laneId, double sumS,
                   std::vector<std::pair<std::pair<uint64_t, uint64_t>,
                                         std::pair<bool, double>>> &meeting_)
           -> void {
        auto lane = hdmap.GetLaneById(laneId);
        sumS += lane->TotalLength();
        if (sumS < safeDistance) {
          std::vector<cyberverse::LaneRelation> mergingLaneConnection;
          hdmap.TopologicalLaneConnections().GetLaneEndConnectionsTo(
              laneId, mergingLaneConnection);
          for (auto &connection : mergingLaneConnection) {
            if (connection.from() == laneId) {
              meeting_.push_back({{connection.to(), srcId}, {false, sumS}});
              self(self, hdmap, safeDistance, srcId,
                   meeting_.back().first.first, sumS, meeting_);
            }
          }
        }
      }](cyberverse::HDMap &hdmap, double safeDistance, uint64_t srcId,
         uint64_t laneId, double minus_dis,
         std::vector<std::pair<std::pair<uint64_t, uint64_t>,
                               std::pair<bool, double>>> &meeting_) -> void {
    mfunc2(mfunc2, hdmap, safeDistance, srcId, laneId, minus_dis, meeting_);
  };
  auto GetConnection =
      [&GetSuccessor, &GetMerging, safeDistance](
          uint64_t laneId, cyberverse::HDMap &hdmap,
          std::vector<std::pair<std::pair<uint64_t, uint64_t>, bool>>
              &successor_,
          int &successorCnt,
          std::vector<std::pair<std::pair<uint64_t, uint64_t>, bool>> &merging_,
          int &mergingCnt) -> std::pair<std::string, std::string> {
    std::vector<cyberverse::LaneRelation> successorLaneConnection;
    std::string successors = "", mergings = "";
    hdmap.TopologicalLaneConnections().GetLaneEndConnectionsFrom(
        laneId, successorLaneConnection);
    for (auto &connection : successorLaneConnection) {
      int sucVecstart = successor_.size();
      if (connection.from() == laneId) {
        auto successor = connection.to();
        successor_.push_back({{successor, successor}, true});
        successorCnt++;
        GetSuccessor(hdmap, safeDistance, successor, successor, successor_);
        std::vector<cyberverse::LaneRelation> mergingLaneConnection;
        hdmap.TopologicalLaneConnections().GetLaneEndConnectionsTo(
            successor, mergingLaneConnection);
        for (auto &mergingConnection : mergingLaneConnection) {
          if (mergingConnection.from() == successor) {
            int merVecstart = merging_.size();
            if (mergingConnection.to() != laneId) {
              merging_.push_back(
                  {{mergingConnection.to(), mergingConnection.to()}, true});
              mergingCnt++;
              GetMerging(hdmap, safeDistance, mergingConnection.to(),
                         mergingConnection.to(), merging_);
              while (merVecstart < merging_.size()) {
                mergings +=
                    hdmap.GetIdHashString(merging_[merVecstart].first.first) +
                    "(";
                mergings += merging_[merVecstart++].second ? "1) " : "0) ";
              }
            } else
              continue;
          } else {
            LOG_INFO("Error merging relationship occurs: {} -> {}",
                     hdmap.GetIdHashString(mergingConnection.to()),
                     hdmap.GetIdHashString(mergingConnection.from()));
          }
        }
        while (sucVecstart < successor_.size()) {
          successors +=
              hdmap.GetIdHashString(successor_[sucVecstart].first.first) + "(";
          successors += successor_[sucVecstart++].second ? "1) " : "0) ";
        }
      } else {
        LOG_INFO("Error successor relationship occurs: {} -> {}",
                 connection.from(), connection.to());
      }
    }
    return {successors, mergings};
  };
  auto odom2veh = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto odom_pose =
        DataCenter::Instance()->pose_base_link_in_odometry_msg.ptr->pose();
    double odom_yaw = common::GetYawFromPose(odom_pose);
    double dx = pt.x() - odom_pose.position().x();
    double dy = pt.y() - odom_pose.position().y();
    double sin_ = std::sin(odom_yaw);
    double cos_ = std::cos(odom_yaw);
    return {{dy * sin_ + dx * cos_, -dx * sin_ + dy * cos_},
            heading - odom_yaw};
  };
  auto veh2odom = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto odom_pose =
        DataCenter::Instance()->pose_base_link_in_odometry_msg.ptr->pose();
    double odom_yaw = common::GetYawFromPose(odom_pose);
    double dx = pt.x();
    double dy = pt.y();
    double sin_ = std::sin(odom_yaw);
    double cos_ = std::cos(odom_yaw);
    return {{-dy * sin_ + dx * cos_ + odom_pose.position().x(),
             dx * sin_ + dy * cos_ + odom_pose.position().y()},
            heading + odom_yaw};
  };
  auto utm2veh = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto utm_pose =
        DataCenter::Instance()->pose_base_link_in_utm_msg.ptr->pose();
    double utm_yaw = common::GetYawFromPose(utm_pose);
    double dx = pt.x() - utm_pose.position().x();
    double dy = pt.y() - utm_pose.position().y();
    double sin_ = std::sin(utm_yaw);
    double cos_ = std::cos(utm_yaw);
    return {{dy * sin_ + dx * cos_, -dx * sin_ + dy * cos_}, heading - utm_yaw};
  };
  auto veh2utm = [](Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto utm_pose =
        DataCenter::Instance()->pose_base_link_in_utm_msg.ptr->pose();
    double utm_yaw = common::GetYawFromPose(utm_pose);
    double dx = pt.x();
    double dy = pt.y();
    double sin_ = std::sin(utm_yaw);
    double cos_ = std::cos(utm_yaw);
    return {{-dy * sin_ + dx * cos_ + utm_pose.position().x(),
             dx * sin_ + dy * cos_ + utm_pose.position().y()},
            heading + utm_yaw};
  };
  auto utm2odom = [&utm2veh, &veh2odom](
                      Vec2d pt, double heading) -> std::pair<Vec2d, double> {
    auto tmp = utm2veh(pt, heading);
    return veh2odom(tmp.first, tmp.second);
  };
  // four variable combinatorial logic state
  enum ABCDstate {
    m0 = 0,
    m1,
    m2,
    m3,
    m4,
    m5,
    m6,
    m7,
    m8,
    m9,
    m10,
    m11,
    m12,
    m13,
    m14,
    m15
  };
  auto DetermineState =
      [](int state_) -> TrafficConflictZoneContext::connectionType {
    TrafficConflictZoneContext::connectionType type_;
    switch (ABCDstate(state_)) {
      case ABCDstate::m0:   // 0x00 0000
      case ABCDstate::m4:   // 0x04 0100
      case ABCDstate::m12:  // 0x0C 1010
        type_ = TrafficConflictZoneContext::connectionType::Culdesac;
        break;
      case ABCDstate::m1:  // 0x01 0001
        type_ = TrafficConflictZoneContext::connectionType::Straight;
        break;
      case ABCDstate::m3:  // 0x03 0011
        type_ = TrafficConflictZoneContext::connectionType::Diverging;
        break;
      case ABCDstate::m5:   // 0x05 0101
      case ABCDstate::m13:  // 0x0D 1101
        type_ = TrafficConflictZoneContext::connectionType::Merging;
        break;
      case ABCDstate::m7:   // 0x07 0111
      case ABCDstate::m15:  // 0x0F 1111
        type_ = TrafficConflictZoneContext::connectionType::Mixed;
        break;
      default:  // otherwise
        type_ = TrafficConflictZoneContext::connectionType::Unknown;
        break;
    }
    return type_;
  };
  auto CalculateCurvature = [this, &ref_ptr, &safeDistance, &kappaThreshold,
                             &utm2odom](uint64_t eID, uint64_t rID, int type,
                                        double s) -> ConflictLaneContext {
    bool flag[2] = {type & 0x02, type & 0x01};
    double th = 3e-2, halfeps = 5e-4;
    if (!(flag[0] ^ flag[1]) || type > 3 || type <= 0)
      return {0, nullptr, 0,     nullptr,
              5, 0.0,     false, ConflictLaneContext::mTurnType::Unknown};
    if (hdmap_->GetLaneById(eID) == nullptr ||
        hdmap_->GetLaneById(rID) == nullptr)
      return {0, nullptr, 0,     nullptr,
              5, 0.0,     false, ConflictLaneContext::mTurnType::Unknown};

    auto pLPtr = hdmap_->GetLaneById(flag[0] ? eID : rID);
    auto sLPtr = hdmap_->GetLaneById(flag[0] ? rID : eID);

    common::math::Vec2d res;
    const auto &sLseg = sLPtr->Segments().front().unit_direction();
    GetInterp1PointInLane(pLPtr->Id(), s, res);
    const auto &pLEnd = pLPtr->Points().back();

    math::AD2 vPre{pLEnd.x() - res.x(), pLEnd.y() - res.y()};
    math::AD2 vSuc{sLseg.x(), sLseg.y()};

    double cross = math::Cross(vPre, vSuc) / (math::Length(vPre) + 1e-10);
    double orin_cross = cross;
    cross = orin_cross > 0 ? std::max(0.0, cross - th) + halfeps
                           : std::min(0.0, cross + th) - halfeps;
    int sign = math::Sign(cross);

    if (FLAGS_planning_enable_vis_event) {
      auto fill_pt = [](auto t, auto x, auto y, auto z) {
        t->set_x(x), t->set_y(y), t->set_z(z);
      };

      visualizer::Event *event = vis::EventSender::Instance()->GetEvent(
          flag[0] ? "LaneAngle_S" : "LaneAngle_M");
      std::array<double, 4> rgb = {0.4, 0.152, 0.0, 0.5};

      for (int i = 0; i < 2; i++) {
        event->set_type(visualizer::Event::k3D);
        event->add_attribute(visualizer::Event::kOdom);
        event->mutable_color()->set_r(rgb[2 * i]);
        event->mutable_color()->set_g(rgb[1]);
        event->mutable_color()->set_b(rgb[2 * (1 - i)]);
        event->mutable_color()->set_a(rgb[3]);
      }

      auto polyline = event->add_polyline();
      auto p = utm2odom(Vec2d(res.x(), res.y()), atan2(res.x(), res.y())).first;
      fill_pt(polyline->add_point(), p.x(), p.y(), 0);
      p = utm2odom(Vec2d(pLEnd.x(), pLEnd.y()), atan2(res.x(), res.y())).first;
      fill_pt(polyline->add_point(), p.x(), p.y(), 0);
      p = utm2odom(Vec2d(sLPtr->Segments().front().start().x(),
                         sLPtr->Segments().front().start().y()),
                   sLPtr->Segments().front().start().angle())
              .first;
      fill_pt(polyline->add_point(), p.x(), p.y(), 0);
      p = utm2odom(Vec2d(sLPtr->Segments().front().end().x(),
                         sLPtr->Segments().front().end().y()),
                   sLPtr->Segments().front().start().angle())
              .first;
      fill_pt(polyline->add_point(), p.x(), p.y(), 0);

      auto text = event->add_text();
      text->set_text(sign > 0 ? "(^)" : (sign == 0 ? "(-)" : "(v)"));
      p = utm2odom(Vec2d((pLEnd.x() + res.x()) / 2 +
                             0.5 * vPre[1] / math::Length(vPre),
                         (pLEnd.y() + res.y()) / 2 -
                             0.5 * vPre[0] / math::Length(vPre)),
                   atan2((pLEnd.y() + res.y()) / 2 -
                             0.5 * vPre[0] / math::Length(vPre),
                         (pLEnd.x() + res.x()) / 2 +
                             0.5 * vPre[1] / math::Length(vPre)))
              .first;
      fill_pt(text->mutable_position(), p.x(), p.y(), 0);
    }

    return {eID,
            hdmap_->GetLaneById(eID),
            rID,
            hdmap_->GetLaneById(rID),
            flag[0] ? 3 : 2,
            orin_cross,
            sign == 0,
            ConflictLaneContext::mTurnType(sign + 1)};
  };
  auto CalculatePriority = [&type]() -> std::pair<bool, int> {
    int con2Type_ = static_cast<int>(type[1]);
    switch (type[1]) {
      case TrafficConflictZoneContext::connectionType::Diverging:
      case TrafficConflictZoneContext::connectionType::Merging:
      case TrafficConflictZoneContext::connectionType::Mixed:
        return {true, con2Type_ + 4};
      default:
        return {false, 0};
    }
  };
  auto CalcDistanceFromConnection = [this](auto ptr, double dis) {
    for (auto &item : ptr->second) item.ego_rd = dis - item.ego_s;
  };
  auto DetermineCascadeState = [&](double inteval_len) {
    ConflictLaneContext tmp(0);
    std::array<std::map<uint64_t, std::pair<double, double>>, 2> idxSLine;
    std::array<std::vector<size_t>, 2> prime, extend, extendRoot;
    std::array<std::vector<std::set<uint64_t>>, 2> primeChild;
    std::array<std::unordered_map<uint64_t, std::pair<size_t, size_t>>, 2>
        primeridx;
    std::array<std::unordered_map<uint64_t, double>, 2> extendlaneLength;
    double stopS = 0.0;

    for (auto &item : con1Successor) {
      if (item.second) {
        successorLaneInfos.insert(
            CalculateCurvature(item.first.first, laneId[0], 0x01, -5.0));
      }
    }
    for (auto &item : con1Successor) {
      if (!item.second) {
        tmp.id = item.first.second;
        auto it = successorLaneInfos.find(tmp);
        if (it != successorLaneInfos.end()) {
          tmp = *it;
          tmp.id = item.first.first;
          tmp.lane_ptr = hdmap_->GetLaneById(item.first.first);
          if (tmp.lane_ptr != nullptr) successorLaneInfos.insert(tmp);
        }
      }
    }

    for (auto i = 0; i < con1Merging.size(); i++) {
      if (con1Merging[i].second) {
        prime[0].push_back(i);
        primeridx[0].insert(
            std::make_pair(con1Merging[i].first.first,
                           std::make_pair(i, prime[0].size() - 1)));
      } else
        extend[0].push_back(i);
    }
    primeChild[0].resize(prime[0].size());
    for (auto i = 0; i < extend[0].size(); i++) {
      uint64_t idx = con1Merging[extend[0][i]].first.second;
      auto it = primeridx[0].find(idx);
      extendRoot[0].push_back(it->second.second);
    }
    for (auto i = 0; i < extendRoot[0].size(); i++) {
      size_t ridx = extendRoot[0][i];
      primeChild[0][ridx].insert(con1Merging[extend[0][i]].first.first);
    }

    for (auto item = sLine[0]->second.begin(); item != sLine[0]->second.end();
         item++) {
      idxSLine[0].insert(std::make_pair(
          item->lane_id, std::make_pair(item->merging_s, item->ego_s)));
    }
    for (auto i = 0; i < prime[0].size(); i++) {
      auto &item = con1Merging[prime[0][i]].first;
      auto it = idxSLine[0].find(item.first);
      if (it != idxSLine[0].end()) {
        stopS = it->second.first;
        mergingLaneInfos.insert(CalculateCurvature(it->first, laneId[1], 0x02,
                                                   -std::max(stopS, 5.0)));
      } else
        LOG_INFO("Priem mergings in Con1 failed to match");

      int cnt = 0, quefront = 0;
      std::unordered_set<uint64_t> que_set = {
          con1Merging[prime[0][i]].first.first};
      std::vector<uint64_t> que = {con1Merging[prime[0][i]].first.first};
      while (quefront < que.size()) {
        std::vector<cyberverse::LaneRelation> predecessorLaneConnection;
        hdmap_->TopologicalLaneConnections().GetLaneEndConnectionsTo(
            que[quefront], predecessorLaneConnection);
        for (auto &con : predecessorLaneConnection) {
          if (que[quefront] == con.from()) {
            auto cit = primeChild[0][i].find(con.to());
            if (cit != primeChild[0][i].end()) {
              auto ptr = hdmap_->GetLaneById(con.from());
              auto pit = extendlaneLength[0].find(con.from());
              if (que_set.find(con.to()) == que_set.end()) {
                que_set.insert(con.to());
                que.push_back(con.to());
                extendlaneLength[0].insert(std::make_pair(
                    con.to(),
                    ptr->TotalLength() +
                        (pit == extendlaneLength[0].end() ? 0 : pit->second)));
              }
              cnt++;
            }
          }
        }
        quefront++;
      }
    }
    for (auto i = 0; i < extend[0].size(); i++) {
      auto &item = con1Merging[extend[0][i]].first;
      auto it = idxSLine[0].find(item.second);
      if (it != idxSLine[0].end()) {
        tmp.id = item.second;
        auto mit = mergingLaneInfos.find(tmp);
        if (mit != mergingLaneInfos.end()) {
          tmp = *mit;
          tmp.id = item.first;
          tmp.lane_ptr = hdmap_->GetLaneById(item.first);
          if (tmp.lane_ptr != nullptr) mergingLaneInfos.insert(tmp);
          auto msit = sLine[0]->second.find(
              MergingStopLineInfo(item.second, it->second.second));
          auto pit = extendlaneLength[0].find(item.first);
          sLine[0]->second.insert(MergingStopLineInfo(
              item.first, mit->id, msit->intersection, msit->ego_s,
              msit->merging_s,
              (pit == extendlaneLength[0].end() ? 0 : pit->second), 0.0));
        } else
          LOG_INFO("extend mergings in Con1 failed to match");
      } else
        LOG_INFO("extend mergings in Con1 failed to match");
    }

    auto priority = CalculatePriority();
    double next_lane_len = hdmap_->GetLaneById(laneId[1])->TotalLength();
    if (priority.first && next_lane_len < kNtypeDetectThreshold &&
        (inteval_len - next_lane_len) * 2 < safeDistance) {
      restRouteLength[2] = inteval_len;
      type[2] = TrafficConflictZoneContext::connectionType(priority.second);

      for (auto &item : con2Successor) {
        if (item.second) {
          successorLaneInfos.insert(
              CalculateCurvature(item.first.first, laneId[1], 0x01, -5.0));
        }
      }
      for (auto &item : con2Successor) {
        if (!item.second) {
          tmp.id = item.first.second;
          auto it = successorLaneInfos.find(tmp);
          if (it != successorLaneInfos.end()) {
            tmp = *it;
            tmp.id = item.first.first;
            tmp.lane_ptr = hdmap_->GetLaneById(item.first.first);
            if (tmp.lane_ptr != nullptr) successorLaneInfos.insert(tmp);
          }
        }
      }
      for (auto i = 0; i < con2Merging.size(); i++) {
        if (con2Merging[i].second) {
          prime[1].push_back(i);
          primeridx[1].insert(
              std::make_pair(con2Merging[i].first.first,
                             std::make_pair(i, prime[1].size() - 1)));
        } else
          extend[1].push_back(i);
      }
      primeChild[1].resize(prime[1].size());
      for (auto i = 0; i < extend[1].size(); i++) {
        uint64_t idx = con2Merging[extend[1][i]].first.second;
        auto it = primeridx[1].find(idx);
        extendRoot[1].push_back(it->second.second);
      }
      for (auto i = 0; i < extendRoot[1].size(); i++) {
        size_t ridx = extendRoot[1][i];
        primeChild[1][ridx].insert(con2Merging[extend[1][i]].first.first);
      }

      for (const auto &item : sLine[1]->second) {
        idxSLine[1].insert(std::make_pair(
            item.lane_id, std::make_pair(item.merging_s, item.ego_s)));
      }
      for (auto i = 0; i < prime[1].size(); i++) {
        auto &item = con2Merging[prime[1][i]].first;
        auto it = idxSLine[1].find(item.first);
        if (it != idxSLine[1].end()) {
          stopS = it->second.first;
          mergingLaneInfos.insert(CalculateCurvature(
              it->first, thirdlaneId, 0x02, -std::max(stopS, 5.0)));
        } else
          LOG_INFO("Priem mergings in Con2 failed to match");

        int quefront = 0;
        std::unordered_set<uint64_t> que_set = {
            con2Merging[prime[1][i]].first.first};
        std::vector<uint64_t> que = {con2Merging[prime[1][i]].first.first};
        while (quefront < que.size()) {
          std::vector<cyberverse::LaneRelation> predecessorLaneConnection;
          hdmap_->TopologicalLaneConnections().GetLaneEndConnectionsTo(
              que[quefront], predecessorLaneConnection);
          for (auto &con : predecessorLaneConnection) {
            if (que[quefront] == con.from()) {
              auto cit = primeChild[1][i].find(con.to());
              if (cit != primeChild[1][i].end()) {
                auto ptr = hdmap_->GetLaneById(con.from());
                auto pit = extendlaneLength[1].find(con.from());
                if (que_set.find(con.to()) == que_set.end()) {
                  que_set.insert(con.to());
                  que.push_back(con.to());
                  extendlaneLength[1].insert(std::make_pair(
                      con.to(),
                      ptr->TotalLength() + (pit == extendlaneLength[1].end()
                                                ? 0
                                                : pit->second)));
                }
              }
            }
          }
          quefront++;
        }
      }
      for (auto i = 0; i < extend[1].size(); i++) {
        auto &item = con2Merging[extend[1][i]].first;
        auto it = idxSLine[1].find(item.second);
        if (it != idxSLine[1].end()) {
          tmp.id = item.second;
          auto mit = mergingLaneInfos.find(tmp);
          if (mit != mergingLaneInfos.end()) {
            tmp = *mit;
            tmp.id = item.first;
            tmp.lane_ptr = hdmap_->GetLaneById(item.first);
            if (tmp.lane_ptr != nullptr) mergingLaneInfos.insert(tmp);
            auto msit = sLine[1]->second.find(
                MergingStopLineInfo(item.second, it->second.second));
            auto pit = extendlaneLength[1].find(item.first);
            sLine[1]->second.insert(MergingStopLineInfo(
                item.first, mit->id, msit->intersection, msit->ego_s,
                msit->merging_s,
                (pit == extendlaneLength[1].end() ? 0 : pit->second), 0.0));
          } else
            LOG_INFO("extend mergings in Con2 failed to match");
        } else
          LOG_INFO("extend mergings in Con2 failed to match");
      }
    } else {
      restRouteLength[2] = restRouteLength[0];
      type[2] = type[0];
    }
    for (int i = 0; i < 2; i++)
      CalcDistanceFromConnection(sLine[i], restRouteLength[i]);
  };
  auto GenerateZonePolygon =
      [this, safeDistance, &utm2odom](
          std::set<ConflictLaneContext> &infos) -> std::vector<math::Polygon> {
    std::vector<math::Polygon> lanePolyVec;
    for (auto &info : infos) {
      std::vector<math::AD2> laneBound;
      auto lane = hdmap_->GetLaneById(info.id);
      if (lane == nullptr) return {};
      auto leftBoundary = lane->left_divider_polygon().Points();
      auto rightBoundary = lane->right_divider_polygon().Points();
      for (int i = 0; i < leftBoundary.size() + rightBoundary.size(); i++) {
        if (i < leftBoundary.size()) {
          auto p = utm2odom(Vec2d(leftBoundary[i].x(), leftBoundary[i].y()),
                            leftBoundary[i].angle());
          laneBound.push_back({p.first.x(), p.first.y()});
        } else {
          int j = leftBoundary.size() + rightBoundary.size() - 1 - i;
          auto p = utm2odom(Vec2d(rightBoundary[j].x(), rightBoundary[j].y()),
                            rightBoundary[j].angle());
          laneBound.push_back({p.first.x(), p.first.y()});
        }
      }
      lanePolyVec.emplace_back(laneBound);
    }
    return lanePolyVec;
  };
  auto GenerateLanePolyline = [this, safeDistance,
                               &utm2odom](auto lane) -> math::Polyline {
    std::vector<math::AD2> laneBound;
    if (lane == nullptr) return math::Polyline(laneBound);
    auto &center_pts = lane->Points();
    for (int i = 0; i < center_pts.size(); i++) {
      auto p = utm2odom(Vec2d(center_pts[i].x(), center_pts[i].y()),
                        std::atan2(center_pts[i].y(), center_pts[i].x()));
      laneBound.push_back({p.first.x(), p.first.y()});
    }
    return math::Polyline(laneBound);
  };
  auto GenerateLanePolygon = [this, safeDistance,
                              &utm2odom](auto lane) -> math::Polygon {
    std::vector<math::AD2> laneBound;
    if (lane == nullptr) return math::Polygon(laneBound);
    auto leftBoundary = lane->left_divider_polygon().Points();
    auto rightBoundary = lane->right_divider_polygon().Points();
    auto &leftstart =
        lane->left_divider_polygon().Segments().front().unit_direction();
    auto &leftend =
        lane->left_divider_polygon().Segments().back().unit_direction();
    auto &rightstart =
        lane->right_divider_polygon().Segments().back().unit_direction();
    auto &rightend =
        lane->right_divider_polygon().Segments().front().unit_direction();
    double tinyCoff = 0.001;
    std::array<math::AD2, 5> tinyVec{
        {{0.0, 0.0},
         {tinyCoff * leftstart.y(), -tinyCoff * leftstart.x()},
         {tinyCoff * leftend.y(), -tinyCoff * leftend.x()},
         {-tinyCoff * rightstart.y(), tinyCoff * rightstart.x()},
         {-tinyCoff * rightend.y(), tinyCoff * rightend.x()}}};
    int idxCase[4] = {0, leftBoundary.size() - 1, leftBoundary.size(),
                      leftBoundary.size() + rightBoundary.size() - 1};
    for (int i = 0; i < leftBoundary.size() + rightBoundary.size(); i++) {
      int idx = 3;
      while (idx >= 0 && i != idxCase[idx]) idx--;
      idx += 1;
      if (i < leftBoundary.size()) {
        auto p = utm2odom(Vec2d(leftBoundary[i].x() + tinyVec[idx][0],
                                leftBoundary[i].y() + tinyVec[idx][1]),
                          leftBoundary[i].angle());
        laneBound.push_back({p.first.x(), p.first.y()});
      } else {
        int j = leftBoundary.size() + rightBoundary.size() - 1 - i;
        auto p = utm2odom(Vec2d(rightBoundary[j].x() + tinyVec[idx][0],
                                rightBoundary[j].y() + tinyVec[idx][1]),
                          rightBoundary[j].angle());
        laneBound.push_back({p.first.x(), p.first.y()});
      }
    }
    return math::Polygon(laneBound);
  };
  auto GenerateIntersection = [this](uint64_t egoLID, uint64_t mergingLID) {
    auto egoLine = hdmap_->GetLaneById(egoLID);
    auto mergingLine = hdmap_->GetLaneById(mergingLID);
    if (egoLine == nullptr || mergingLine == nullptr) return math::AD2({0, 0});
    auto &egoSegs = egoLine->Segments();
    auto &mergingPts = mergingLine->Points();
    math::AD2 vEgo{egoSegs.back().unit_direction().x(),
                   egoSegs.back().unit_direction().y()};
    math::AD2 vMerging;
    std::vector<math::AD2> egoLanePts, mergingLanePts;
    double ans = 0.0;
    int idx = mergingPts.size() - 2;
    do {
      vMerging[0] = mergingPts.back().x() - mergingPts[idx].x();
      vMerging[1] = mergingPts.back().y() - mergingPts[idx].y();
      double length_ = math::Length(vMerging);
      if (length_ <= kMathEpsilon) {
        vMerging[0] = 0.0;
        vMerging[1] = 0.0;
        LOG_INFO("Lane Segment is invalid.");
      } else {
        vMerging[0] /= length_;
        vMerging[1] /= length_;
      }
      ans = math::Cross(vEgo, vMerging);
      if (--idx < 0) break;
    } while (std::abs(ans) < 0.0175);

    math::AD2 res{0.0, 0.0};
    auto &src_ego_l = egoLine->left_divider_polygon().Points();
    auto &src_ego_r = egoLine->right_divider_polygon().Points();
    auto &src_merging_l = mergingLine->left_divider_polygon().Points();
    auto &src_merging_r = mergingLine->right_divider_polygon().Points();
    const neodrive::common::math::ShmVector<neodrive::common::math::Vec2d>
        *divider_ptra[2][2] = {{&src_ego_r, &src_merging_l},
                               {&src_ego_l, &src_merging_r}};

    int is_inverse = math::Cross(vEgo, vMerging) <= 0.5;
    for (int k = 0; k < 2; k++) {
      int j = is_inverse ? 1 - k : k;
      egoLanePts.clear();
      mergingLanePts.clear();
      for (auto i = 0; i < divider_ptra[j][0]->size(); i++) {
        egoLanePts.push_back(
            {(*divider_ptra[j][0])[i].x(), (*divider_ptra[j][0])[i].y()});
      }
      for (auto i = 0; i < divider_ptra[j][1]->size(); i++) {
        mergingLanePts.push_back(
            {(*divider_ptra[j][1])[i].x(), (*divider_ptra[j][1])[i].y()});
      }
      res = math::FindIntersection(egoLanePts, mergingLanePts);
      if (math::Sign(res[0]) || math::Sign(res[1])) {
        break;
      }
    }
    return res;
  };
  auto GenerateAllIntersection = [this](uint64_t egoLID, uint64_t mergingLID,
                                        std::array<math::AD2, 4> &res) {
    auto ego_line_ptr = hdmap_->GetLaneById(egoLID);
    auto merging_line_ptr = hdmap_->GetLaneById(mergingLID);
    if (ego_line_ptr == nullptr || merging_line_ptr == nullptr) return;

    const auto &src0 = ego_line_ptr->left_divider_polygon().Points();
    const auto &src1 = ego_line_ptr->right_divider_polygon().Points();
    const auto &src2 = merging_line_ptr->left_divider_polygon().Points();
    const auto &src3 = merging_line_ptr->right_divider_polygon().Points();
    std::array<std::vector<math::AD2>, 4> pts;

    for (int j = 0; j < src0.size(); j++) {
      pts[0].push_back({src0[j].x(), src0[j].y()});
    }
    for (int j = 0; j < src1.size(); j++) {
      pts[1].push_back({src1[j].x(), src1[j].y()});
    }
    for (int j = 0; j < src2.size(); j++) {
      pts[2].push_back({src2[j].x(), src2[j].y()});
    }
    for (int j = 0; j < src3.size(); j++) {
      pts[3].push_back({src3[j].x(), src3[j].y()});
    }

    for (int i = 0; i < 2; i++) {
      for (int j = 2; j < 4; j++) {
        std::array<double, 4> dis{
            math::Distance(pts[i].front(), pts[j].front()),
            math::Distance(pts[i].front(), pts[j].back()),
            math::Distance(pts[i].back(), pts[j].front()),
            math::Distance(pts[i].back(), pts[j].back())};
        std::pair<int, double> idx{-1, 1e9};
        for (int k = 0; k < 4; k++) {
          idx.first = k;
          idx.second = dis[k];
        }
        res[i * 2 + j - 2] = math::FindIntersection(
            pts[i], pts[j], idx.first & 0x02, idx.first & 0x01);
      }
    }
    std::array<int, 5> is_find;
    for (int i = 0; i < 4; i++) {
      is_find[i] = math::Sign(res[i][0]) == 0 && math::Sign(res[i][1]) == 0;
      is_find[4] += is_find[i];
    }
    if (is_find[4] > 0) {
      std::array<double, 4> tmpl, tmps;
      merging_line_ptr->GetProjection(src0.front(), &tmps[0], &tmpl[0]);
      merging_line_ptr->GetProjection(src0.back(), &tmps[1], &tmpl[1]);
      merging_line_ptr->GetProjection(src1.front(), &tmps[2], &tmpl[2]);
      merging_line_ptr->GetProjection(src1.back(), &tmps[3], &tmpl[3]);
      for (int i = 0; i < 4; i++) {
        if (is_find[i]) {
          int chooseIdx = i < 2 ? 0 : 2;
          chooseIdx =
              tmpl[chooseIdx] < tmpl[chooseIdx + 1] ? 0 : pts[i > 1].size() - 1;
          res[i] = pts[i > 1][chooseIdx];
        }
      }
    }
  };
  auto CreateFromConnection =
      [this, &GenerateIntersection](
          uint64_t lID, int idx,
          std::vector<std::pair<std::pair<uint64_t, uint64_t>, bool>>
              &Mergings) {
        connection_merging_geoinfo_[idx].first = lID;
        auto &mergingGeoInfo = connection_merging_geoinfo_[idx].second;
        mergingGeoInfo.clear();
        auto lptr = hdmap_->GetLaneById(lID);
        if (lptr == nullptr) return false;
        for (const auto &mID : Mergings) {
          if (mID.second) {
            auto mptr = hdmap_->GetLaneById(mID.first.first);
            if (mptr == nullptr) return false;
            math::AD2 pts = GenerateIntersection(lID, mID.first.first);
            double dis[3] = {0.0, 0.0, 0.0};
            lptr->GetProjection(common::math::Vec2d(pts[0], pts[1]), &dis[0],
                                &dis[2]);
            mptr->GetProjection(common::math::Vec2d(pts[0], pts[1]), &dis[1],
                                &dis[2]);
            dis[0] = lptr->TotalLength() - dis[0];
            dis[1] = mptr->TotalLength() - dis[1];
            mergingGeoInfo.insert(MergingStopLineInfo(
                mID.first.first, 0, pts, dis[0], dis[1], 0.0, 0.0));
          }
        }
        return true;
      };
  auto SearchFromConnection = [this, &CreateFromConnection,
                               &CalcDistanceFromConnection, &restRouteLength,
                               &con1Merging, &con2Merging, &sLine, &laneId]() {
    int idx[2];
    for (int i = 0; i < 2; i++) {
      idx[i] = -1;
      for (int j = 0; j < 2; j++) {
        if (connection_merging_geoinfo_[j].first == laneId[i]) idx[i] = j;
      }
    }
    if (idx[0] < 0 || idx[1] < 0) {
      if (idx[0] < 0 && idx[1] < 0) {
        for (int i = 0; i < 2; i++) {
          CreateFromConnection(laneId[i], idx[i] = i,
                               i ? con2Merging : con1Merging);
        }
      } else {
        for (int i = 0; i < 2; i++) {
          if (idx[i] < 0) {
            CreateFromConnection(laneId[i], idx[i] = 1 - idx[1 - i],
                                 i ? con2Merging : con1Merging);
            break;
          }
        }
      }
    }
    for (int i = 0; i < 2; i++) {
      sLine[i] = &connection_merging_geoinfo_[idx[i]];
    }
  };
  auto mergingGeoInfoRepr = [this, &sLine]() {
    for (int i = 0; i < 2; i++) {
      for (auto &item : sLine[i]->second) {
        item.Repr(*hdmap_);
      }
    }
  };
  auto overlapMergingGeoInfoRepr = [this, &sLine]() {
    for (int i = 0; i < 2; i++) {
      std::string str = "";
      for (auto &item : junction_info_[i].second) {
        str += hdmap_->GetIdHashString(item.first.lane_id) + "\n";
      }
      LOG_INFO("#{}: {}", i, str);
    }
    for (int i = 0; i < 2; i++) {
      for (auto &item : junction_info_[i].second) {
        LOG_INFO("---------------- #{} ----------------", i);
        item.first.Repr(*hdmap_);
        item.second.Repr(*hdmap_);
      }
    }
  };

  // first connection
  if (!getReferencePoint(task_info.curr_sl().s(), &nearPoint)) return false;
  laneId[0] = nearPoint.hd_map_lane_id();
  lanes[0] = hdmap_->GetLaneById(laneId[0]);
  auto laneEnd =
      Vec2d(lanes[0]->Points().back().x(), lanes[0]->Points().back().y());
  double currentRouteLength = nearPoint.hd_map_lane_s(), nextRouteLength = 0.0;
  restRouteLength[0] = restRouteLength[1] =
      lanes[0]->TotalLength() - currentRouteLength;

  uint64_t leftId = 0, rightId = 0;
  GetNearestLeftLane(laneId[0], lanes[0]->Points().back(), leftId);
  GetNearestRightLane(laneId[0], lanes[0]->Points().back(), rightId);

  LOG_INFO("----------------- Start lane output -------------------");
  LOG_INFO("current lane ID: {}", hdmap_->GetIdHashString(laneId[0]));
  LOG_INFO("rest distance #1: {}", restRouteLength[0]);

  auto connectionIdStr =
      GetConnection(laneId[0], *hdmap_, con1Successor, successorLaneNum[0],
                    con1Merging, mergingLaneNum[0]);

  state[0] = ((mergingLaneNum[0] > 1) << 3) | ((mergingLaneNum[0] > 0) << 2) |
             ((successorLaneNum[0] > 1) << 1) | ((successorLaneNum[0] > 0));
  type[0] = DetermineState(state[0]);

  // second connection
  auto tmpTransformAns =
      utm2veh(Vec2d{laneEnd.x(), laneEnd.y()}, laneEnd.angle());
  tmpTransformAns = veh2odom(tmpTransformAns.first, tmpTransformAns.second);
  Vec2d laneEndOdom = tmpTransformAns.first;

  bool crossConnection = ref_ptr->GetfrontNearestRefPoint(
      laneEndOdom, laneId[0], con1Successor, &rearPoint);
  std::pair<std::string, std::string> nextConnectionIdStr{"", ""};
  if (crossConnection) {
    laneId[1] = rearPoint.hd_map_lane_id();
    lanes[1] = hdmap_->GetLaneById(laneId[1]);
    nextRouteLength = lanes[1]->TotalLength();
    restRouteLength[1] = lanes[1]->TotalLength() + restRouteLength[0];
    nextConnectionIdStr =
        GetConnection(laneId[1], *hdmap_, con2Successor, successorLaneNum[1],
                      con2Merging, mergingLaneNum[1]);

    state[1] = ((mergingLaneNum[1] > 1) << 3) | ((mergingLaneNum[1] > 0) << 2) |
               ((successorLaneNum[1] > 1) << 1) | ((successorLaneNum[1] > 0));
    type[1] = DetermineState(state[1]);

    tmpTransformAns = utm2odom(
        Vec2d{lanes[1]->Points().back().x(), lanes[1]->Points().back().y()},
        laneEnd.angle());
    bool thirdConnection = ref_ptr->GetfrontNearestRefPoint(
        tmpTransformAns.first, laneId[1], con2Successor, &finalPoint);
    thirdlaneId = finalPoint.hd_map_lane_id();

    LOG_INFO("next lane ID: {}", hdmap_->GetIdHashString(laneId[1]));
  } else {
    thirdlaneId = laneId[1] = laneId[0];
    lanes[1] = lanes[0];
    successorLaneNum[1] = 0;
    mergingLaneNum[1] = 0;
    state[1] = 10;  // Hex(1010)
    type[1] = TrafficConflictZoneContext::connectionType::Straight;
    LOG_INFO("next lane ID: [Misaligned]");
  }

  SearchFromConnection();
  DetermineCascadeState(restRouteLength[1]);
  auto sucPolyVec = GenerateZonePolygon(successorLaneInfos);
  auto merPolyVec = GenerateZonePolygon(mergingLaneInfos);

  std::vector<cyberverse::LaneRelation> successorLaneConnection;
  hdmap_->TopologicalLaneConnections().GetLaneEndConnectionsFrom(
      laneId[0], successorLaneConnection);
  ConflictLaneContext tmp(0);
  for (auto &connection : successorLaneConnection) {
    if (connection.from() == laneId[0]) {
      tmp.id = connection.to();
      auto it = successorLaneInfos.find(tmp);
      if (it != successorLaneInfos.end()) {
        it->is_extended = false;
      }
    }
  }

  LOG_INFO("#1 Succesors -> {}", connectionIdStr.first);
  LOG_INFO("#1 Mergings -> {}", connectionIdStr.second);
  LOG_INFO("#2 Succesors -> {}", nextConnectionIdStr.first);
  LOG_INFO("#2 Mergings -> {}", nextConnectionIdStr.second);
  LOG_INFO("Mergings Info {} : {} | {}", mergingLaneInfos.size(),
           sLine[0]->second.size(), sLine[1]->second.size());
  LOG_INFO("Connection type: {}({}) -> {}({}) = {}",
           typeString[static_cast<int>(type[0])], state[0],
           typeString[static_cast<int>(type[1])], state[1],
           typeString[static_cast<int>(type[2])]);

  std::array<ConflictLaneContext, 2> laneInfos{
      CalculateCurvature(laneId[0], laneId[1], 0x02, -5.0),
      {laneId[1], lanes[1], 0, nullptr, 5, 0.0, false,
       ConflictLaneContext::mTurnType::Unknown}};
  std::set<ConflictLaneContext> curLane{laneInfos[0]};

  auto CheckOverlapMerge = [this](auto cptr, auto lptr) {
    if (cptr == nullptr || lptr == nullptr) return false;
    auto &cpbegin = cptr->Points().front();
    auto &cpend = cptr->Points().back();
    auto &lpbegin = lptr->Points().front();
    auto &lpend = lptr->Points().back();
    return !(cpbegin == lpbegin || cpend == lpend || cpbegin == lpend ||
             cpend == lpbegin);
  };
  auto CheckUVturn = [this](cyberverse::LaneInfoConstPtr lptr) {
    if (lptr == nullptr) return true;
    auto &lsegs = lptr->Segments();
    auto lsbegin = lsegs.front().unit_direction();
    auto lsend = lsegs.back().unit_direction();
    double intAngle;

    if (hdmap_->TopologicalLaneConnections().GetLaneEndConnectionNumFrom(
            lptr->Id()) == 1 &&
        hdmap_->TopologicalLaneConnections().GetLaneEndConnectionNumTo(
            lptr->Id()) == 1) {
      std::vector<neodrive::cyberverse::LaneRelation> preVec, sucVec;
      hdmap_->TopologicalLaneConnections().GetLaneEndConnectionsTo(lptr->Id(),
                                                                   preVec);
      hdmap_->TopologicalLaneConnections().GetLaneEndConnectionsFrom(lptr->Id(),
                                                                     sucVec);
      lsbegin = hdmap_->GetLaneById(preVec.front().to())
                    ->Segments()
                    .back()
                    .unit_direction();
      lsend = hdmap_->GetLaneById(sucVec.front().to())
                  ->Segments()
                  .front()
                  .unit_direction();
    }

    intAngle = math::Cross({lsbegin.x(), lsbegin.y()}, {lsend.x(), lsend.y()});
    math::AD2 displacement{
        lptr->Points().back().x() - lptr->Points().front().x(),
        lptr->Points().back().y() - lptr->Points().front().y()};
    double circumRadius = 1 / (intAngle + 1e-8);
    double sind90drange = 0.984, extendDis = 5.0;
    LOG_INFO("{}: {} | {}/{}={}", hdmap_->GetIdHashString(lptr->Id()), intAngle,
             lptr->TotalLength(), math::Length(displacement),
             lptr->TotalLength() / math::Length(displacement));
    if (lptr->TotalLength() / math::Length(displacement) > 2) {
      return true;
    } else if (math::Sign(std::abs(intAngle), sind90drange, 1) == 0) {
      double tmp = std::abs(circumRadius *
                            math::Cross(displacement, {lsend.x(), lsend.y()}));
      math::Point intersection{tmp * lsbegin.x() + lptr->Points().front().x(),
                               tmp * lsbegin.y() + lptr->Points().front().y()};
      std::vector<math::AD2> poly_vec;
      auto &lpts = lptr->Points();
      double ans = -1.0;
      for (int i = 0; i < lpts.size(); i++) {
        poly_vec.push_back({lpts[i].x(), lpts[i].y()});
      }
      if (math::IsOverlaped(intersection, math::Polygon(poly_vec))) {
        return true;
      }
      ans = std::numeric_limits<double>::infinity();
      for (int i = 0; i < lsegs.size(); i++) {
        ans = std::min(
            ans,
            math::Distance(
                intersection,
                math::LineSegment({lsegs[i].start().x(), lsegs[i].start().y()},
                                  {lsegs[i].end().x(), lsegs[i].end().y()})));
      }
      LOG_INFO("{}: ( {} , {} ) -> {}", hdmap_->GetIdHashString(lptr->Id()),
               intersection.x(), intersection.y(), ans);
      if (ans < extendDis) {
        return true;
      }
    }
    return false;
  };
  auto CreateFromJunction = [this, &utm2odom, &GenerateAllIntersection,
                             &CheckOverlapMerge, &CheckUVturn,
                             &CalculateCurvature, &GenerateLanePolygon,
                             &GenerateLanePolyline, &overlapMerging,
                             &GetSuccessor, &GetMeeting,
                             safeDistance](uint64_t lID, int idx) {
    auto l_ptr = hdmap_->GetLaneById(lID);
    auto &overlapGeoInfo = junction_info_[idx].second;
    overlapGeoInfo.clear();
    std::array<std::vector<neodrive::cyberverse::JunctionInfoConstPtr>, 3>
        j_ptr_vec;
    if (l_ptr == nullptr) return false;
    hdmap_->Getjunctions(common::math::Vec2d(l_ptr->Points().back().x(),
                                             l_ptr->Points().back().y()),
                         0.1, &j_ptr_vec[1]);
    hdmap_->Getjunctions(common::math::Vec2d(l_ptr->Points().front().x(),
                                             l_ptr->Points().front().y()),
                         0.1, &j_ptr_vec[0]);
    for (auto &p : j_ptr_vec[0]) {
      LOG_INFO("Junction 0: {}", hdmap_->GetIdHashString(p->Id()));
    }
    for (auto &p : j_ptr_vec[1]) {
      LOG_INFO("Junction 1: {}", hdmap_->GetIdHashString(p->Id()));
    }
    for (int i = 0; i < j_ptr_vec[0].size(); i++) {
      for (int j = 0; j < j_ptr_vec[1].size(); j++) {
        if (j_ptr_vec[0][i] == j_ptr_vec[1][j]) {
          j_ptr_vec[2].push_back(j_ptr_vec[0][i]);
        }
      }
    }
    if (j_ptr_vec[2].empty()) return false;
    for (int i = 0; i < j_ptr_vec[2].size(); i++) {
      LOG_INFO("#{}: Junction:{}", idx,
               hdmap_->GetIdHashString(j_ptr_vec[2][i]->Id()));
    }
    junction_info_[idx].first = lID;
    auto jPtr = j_ptr_vec[2].front();
    auto &laneIds = jPtr->LaneIds();
    std::vector<bool> selected_lane_ids;
    math::Polygon curlanePolygon = GenerateLanePolygon(l_ptr);
    for (auto i = 0; i < laneIds.size(); i++) {
      selected_lane_ids.emplace_back(false);
      if (lID != laneIds[i]) {
        auto tlPtr = hdmap_->GetLaneById(laneIds[i]);
        if (!CheckOverlapMerge(l_ptr, tlPtr) || CheckUVturn(tlPtr)) continue;
        selected_lane_ids.back() = true;
      }
    }

    for (int i = 0; i < laneIds.size(); i++) {
      if (selected_lane_ids[i]) {
        auto tlPtr = hdmap_->GetLaneById(laneIds[i]);
        if (math::IsOverlaped(GenerateLanePolyline(tlPtr), curlanePolygon) &&
            math::IsOverlaped(GenerateLanePolygon(tlPtr), curlanePolygon)) {
          LOG_INFO("{}: overlaped", hdmap_->GetIdHashString(laneIds[i]));
          math::AD2 t_dir{
              tlPtr->Points().back().x() - tlPtr->Points().front().x(),
              tlPtr->Points().back().y() - tlPtr->Points().front().y()};
          math::AD2 l_dir{
              l_ptr->Points().back().x() - l_ptr->Points().front().x(),
              l_ptr->Points().back().y() - l_ptr->Points().front().y()};
          bool isInverse = false;
          /**
           *           B
           *        1  | 3
           *        .  |  .
           *       .   |  .
           *      0   |   2
           *         F
           */
          std::array<math::AD2, 4> pts;
          GenerateAllIntersection(lID, laneIds[i], pts);
          std::array<double, 4> dis{0.0, 0.0, 0.0, 0.0};
          l_ptr->GetProjection(common::math::Vec2d(pts[0][0], pts[0][1]),
                               &dis[0], &dis[2]);
          l_ptr->GetProjection(common::math::Vec2d(pts[1][0], pts[1][1]),
                               &dis[1], &dis[3]);
          if (dis[0] > dis[1]) {
            pts[0].swap(pts[1]);
            pts[2].swap(pts[3]);
            dis[2] = dis[3];
          }
          double dis01 = math::Distance(pts[0], pts[1]);
          double dis23 = math::Distance(pts[2], pts[3]);
          double dis02 = math::Distance(pts[0], pts[2]);
          double dis13 = math::Distance(pts[1], pts[3]);
          double gain01 = dis01 / (dis[2] + 1e-9);
          double gain23 = dis23 / (dis[2] + 1e-9);
          int is_abnormal = math::Sign(gain01 / gain23, 1 / 1.5, 1.5);
          LOG_INFO("{}({}) {}({}) {} : {} -> {}", dis01, gain01, dis23, gain23,
                   dis[2], dis02, dis13);

          if (is_abnormal) {
            if (dis02 > dis13) {
              int near_idx = is_abnormal > 0 ? 0 : 2;
              LOG_INFO("Near end {} is overlong", near_idx);
              GetInterp1PointInLane(lID, laneIds[i], near_idx < 2,
                                    pts[near_idx], pts[near_idx + 1], true);
            } else {
              int rearIdx = is_abnormal > 0 ? 1 : 3;
              LOG_INFO("Rear end {} is overlong", rearIdx);
              GetInterp1PointInLane(lID, laneIds[i], rearIdx < 2,
                                    pts[rearIdx - 1], pts[rearIdx], false);
            }
          }

          std::array<std::array<double, 4>, 2> egoLaneS, mergingLaneS;
          for (int j = 0; j < 4; j++) {
            l_ptr->GetProjection(common::math::Vec2d(pts[j][0], pts[j][1]),
                                 &egoLaneS[0][j], &egoLaneS[1][j]);
            tlPtr->GetProjection(common::math::Vec2d(pts[j][0], pts[j][1]),
                                 &mergingLaneS[0][j], &mergingLaneS[1][j]);
          }
          int in_idx = mergingLaneS[0][0] > mergingLaneS[0][2] ? 2 : 0;
          math::AD2 t_vec{(pts[2 - in_idx][0] + pts[3 - in_idx][0]) -
                              (pts[in_idx][0] + pts[in_idx + 1][0]),
                          (pts[2 - in_idx][1] + pts[3 - in_idx][1]) -
                              (pts[in_idx][1] + pts[in_idx + 1][1])};
          math::AD2 l_vec{(pts[1][0] + pts[3][0]) - (pts[0][0] + pts[2][0]),
                          (pts[1][1] + pts[3][1]) - (pts[0][1] + pts[2][1])};

          if (math::Dot(t_vec, l_vec) < 0) {
            isInverse = true;
            LOG_INFO("{}: overlaped, inverse",
                     hdmap_->GetIdHashString(laneIds[i]));
          } else {
            LOG_INFO("{}: overlaped", hdmap_->GetIdHashString(laneIds[i]));
          }
          std::sort(egoLaneS[0].begin(), egoLaneS[0].end());
          std::sort(mergingLaneS[0].begin(), mergingLaneS[0].end());
          math::AD2 es{egoLaneS[0].front(), egoLaneS[0].back()};
          math::AD2 ms{mergingLaneS[0].front(), mergingLaneS[0].back()};

          overlapGeoInfo.insert(std::make_pair(
              MeetingLaneContext(laneIds[i], 0, pts, es, ms, 0.0, isInverse),
              CalculateCurvature(laneIds[i], laneIds[i], 0x02,
                                 -std::max(dis[1], 5.0))));
          int cnt = overlapMerging[idx].size();
          GetMeeting(*hdmap_, safeDistance, laneIds[i], laneIds[i],
                     std::max(0.0, ms[0]) -
                         hdmap_->GetLaneById(laneIds[i])->TotalLength(),
                     overlapMerging[idx]);

          while (cnt < overlapMerging[idx].size()) {
            math::AD2 extend_meeting_s{
                overlapMerging[idx][cnt].second.second,
                std::max(0.0, ms[1] - ms[0]) +
                    overlapMerging[idx][cnt].second.second};
            LOG_DEBUG("{},{} extend to -> {}, {}", ms[0], ms[1],
                     extend_meeting_s[0], extend_meeting_s[1]);
            overlapGeoInfo.insert(std::make_pair(
                MeetingLaneContext(overlapMerging[idx][cnt].first.first,
                                   laneIds[i], pts, es, extend_meeting_s,
                                   overlapMerging[idx][cnt].second.second,
                                   isInverse),
                CalculateCurvature(overlapMerging[idx][cnt].first.first,
                                   laneIds[i], 0x02, -std::max(dis[1], 5.0))));
            cnt++;
          }
        }
      }
    }
    return true;
  };
  auto SearchFromJunction = [this, &laneId, &juncId, &jInfo,
                             &CreateFromJunction]() {
    int idx[2];
    for (int i = 0; i < 2; i++) {
      idx[i] = -1;
      for (int j = 0; j < 2; j++) {
        if (junction_info_[j].first == laneId[i]) idx[i] = j;
      }
    }
    if (idx[0] < 0 || idx[1] < 0) {
      if (idx[0] < 0 && idx[1] < 0) {
        for (int i = 0; i < 2; i++) {
          CreateFromJunction(laneId[i], idx[i] = i);
        }
      } else {
        for (int i = 0; i < 2; i++) {
          if (idx[i] < 0) {
            CreateFromJunction(laneId[i], idx[i] = 1 - idx[1 - i]);
            break;
          }
        }
      }
    }
    jInfo[0] = &junction_info_[idx[0]];
    jInfo[1] = &junction_info_[idx[1]];
  };
  SearchFromJunction();
  // overlapMergingGeoInfoRepr();

  const auto &ego_lane_info =
      CalculateCurvature(laneInfos[0].id, laneInfos[1].id, 0x01, -5.0);
  laneInfos[0].orientation = ego_lane_info.orientation;
  conflictZone.Update(egoPt, laneEnd, laneInfos, restRouteLength, leftId,
                      rightId, successorLaneInfos, mergingLaneInfos, type,
                      typeString, &sLine[0]->second, &sLine[1]->second,
                      &jInfo[0]->second, &jInfo[1]->second);
  auto curLanePolyVec = GenerateZonePolygon(curLane);
  VisObstacles(ref_ptr, rearPoint, sucPolyVec, merPolyVec, curLanePolyVec,
               laneInfos, connection_merging_geoinfo_, mergingLaneInfos,
               conflictZone);

  return type[2] == TrafficConflictZoneContext::connectionType::Straight ? false
                                                                         : true;
}

bool PlanningMap::IsLeftLane(uint64_t lane_id) {
  auto ptr_lane = hdmap_->GetLaneById(lane_id);
  if (!ptr_lane) {
    LOG_WARN("ptr_lane is nullptr, borrow should be caution.");
    return false;
  }
  auto road_ptr = hdmap_->GetRoadById(ptr_lane->RoadId());
  if (!road_ptr) {
    LOG_WARN("ptr_lane is nullptr, borrow should be caution.");
    return false;
  }
  auto section_id = ptr_lane->SectionId();
  auto section_ptr = road_ptr->GetSectionInRoadById(section_id);
  if (!section_ptr) {
    LOG_WARN("ptr_lane is nullptr, borrow should be caution.");
    return false;
  }
  auto lane_no = std::abs(ptr_lane->LaneNo());

  return (lane_no == 1);
}

bool PlanningMap::IsRightLane(uint64_t lane_id) {
  auto ptr_lane = hdmap_->GetLaneById(lane_id);
  if (!ptr_lane) {
    LOG_WARN("ptr_lane is nullptr, borrow should be caution.");
    return true;
  }
  auto road_ptr = hdmap_->GetRoadById(ptr_lane->RoadId());
  if (!road_ptr) {
    LOG_WARN("ptr_lane is nullptr, borrow should be caution.");
    return false;
  }
  auto section_id = ptr_lane->SectionId();
  auto section_ptr = road_ptr->GetSectionInRoadById(section_id);
  if (!section_ptr) {
    LOG_WARN("ptr_lane is nullptr, borrow should be caution.");
    return false;
  }
  auto lane_no = std::abs(ptr_lane->LaneNo());

  return (section_ptr->LaneIds().size() == lane_no);
}

bool PlanningMap::GetNeighborReverseRoadInfo(
    const uint64_t lane_id,
    std::vector<std::pair<uint64_t, std::vector<BoundaryEdgeType>>>
        &reverse_info) {
  auto ptr_lane = hdmap_->GetLaneById(lane_id);
  if (!ptr_lane) {
    return false;
  }
  auto road_ptr = hdmap_->GetRoadById(ptr_lane->RoadId());
  if (!road_ptr) {
    return false;
  }
  auto section_ptr = road_ptr->GetSectionInRoadById(ptr_lane->SectionId());
  if (!section_ptr) {
    return false;
  }

  for (std::size_t i = 0; i < section_ptr->NeighborReverseRoadIds().size();
       ++i) {
    auto reverse_road_ptr =
        hdmap_->GetRoadById(section_ptr->NeighborReverseRoadIds()[i]);
    if (!reverse_road_ptr) {
      continue;
    }
    std::pair<uint64_t, std::vector<BoundaryEdgeType>> info{};
    info.first = section_ptr->NeighborReverseRoadIds()[i];
    for (const auto &sec : reverse_road_ptr->Sections()) {
      info.second.emplace_back(
          static_cast<BoundaryEdgeType>(sec->LeftBoundaryType()));
    }
    reverse_info.emplace_back(info);
  }
  return true;
}

uint32_t PlanningMap::GetDividerType(const uint64_t lane_id,
                                     const bool is_left) {
  const auto &lane = hdmap_->GetLaneById(lane_id);
  if (lane == nullptr) {
    return static_cast<uint32_t>(global::hdmap::LaneBoundaryType::UNKNOWN);
  }
  auto &divider = is_left ? lane->left_divider() : lane->right_divider();
  return divider.empty()
             ? static_cast<uint32_t>(global::hdmap::LaneBoundaryType::UNKNOWN)
             : divider[0].type;
}

}  // namespace planning
}  // namespace neodrive
