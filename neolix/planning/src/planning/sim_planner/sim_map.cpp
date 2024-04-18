#include "sim_map.h"

namespace neodrive {
namespace planning {
namespace sim_planner {

namespace {
constexpr double kIntervalLane = 2.0;
constexpr double kIntervalPoint = 0.5;
constexpr double kIntervalL = 0.2;
constexpr double kCarHalfWidth = 0.6;

bool getRoadIdByS(const std::vector<SimMapRoad>& road_map, const double s,
                  int* road_id) {
  *road_id = -1;
  for (const auto& road : road_map) {
    if (road.s_s <= s && road.s_e >= s) {
      *road_id = road.road_id;
      break;
    }
  }
  return *road_id >= 0;
}

bool getLaneIdByL(const std::vector<SimMapRoad>& road_map, const double l,
                  const int road_id, int* lane_id) {
  if (road_id < 0 || road_id >= road_map.size() ||
      road_map[road_id].lanes.empty()) {
    return false;
  }

  double min_dis{std::numeric_limits<double>::infinity()};
  *lane_id = 0;
  for (const auto& lane : road_map[road_id].lanes) {
    if (lane.pts.empty()) continue;
    double dis = std::abs(l - lane.pts.front().l);
    if (dis < min_dis) {
      min_dis = dis, *lane_id = lane.lane_id;
    }
  }

  return min_dis <= kIntervalL;
}

bool getRoadLaneIdByPoint(const ReferenceLinePtr ref_ptr,
                          const std::vector<SimMapRoad>& road_map,
                          const Vec2d& pt, SLPoint* sl_pt, int* road_id,
                          int* lane_id) {
  if (!ref_ptr->GetPointInFrenetFrame(pt, sl_pt)) {
    LOG_ERROR("could not get point in frenet frame.");
    return false;
  }
  if (!getRoadIdByS(road_map, sl_pt->s(), road_id)) {
    LOG_ERROR("could not get road id by S.");
    return false;
  }
  if (!getLaneIdByL(road_map, sl_pt->l(), *road_id, lane_id)) {
    LOG_ERROR("could not get lane id by L.");
    return false;
  }

  return true;
}

}  // namespace

SimMap::SimMap() {}

ReferencePointVec1d SimMap::getRefPoints() { return ref_ptr_->ref_points(); }

bool SimMap::createSimMap(TaskInfo& task_info) {
  if (task_info.reference_line()->ref_points().empty()) return false;

  ref_ptr_ = task_info.reference_line();
  sim_map_valid_ = false;
  road_map_.clear();
  const auto& obstacle_decision = task_info.current_frame()
                                      ->outside_planner_data()
                                      .path_obstacle_context.obstacle_decision;

  /// Create kd-tree of static obs polygon2d
  std::vector<math::Node<math::Polygon>> polygon2d_nodes{};
  std::vector<math::AD2> polygon2d_pts{};
  int polygon2d_id{0};
  boundaries_.clear();
  for (const auto& decision : obstacle_decision) {
    if (decision.obstacle_boundary.obstacle.PolygonBoundary().end_s() <
        task_info.current_frame()->inside_planner_data().init_sl_point.s()) {
      continue;
    }
    if (decision.decision_type != Decision::DecisionType::GO_LEFT &&
        decision.decision_type != Decision::DecisionType::GO_RIGHT &&
        decision.decision_type != Decision::DecisionType::YIELD_DOWN) {
      continue;
    }
    if (decision.obstacle_boundary.obstacle.is_virtual()) {
      continue;
    }
    LOG_INFO("sim_map consider obstacle: {}",
             decision.obstacle_boundary.obstacle.id());

    polygon2d_pts.clear();
    polygon2d_pts.resize(
        decision.obstacle_boundary.obstacle.polygon().points().size());
    std::size_t pt_num{0};
    for (const auto& pt :
         decision.obstacle_boundary.obstacle.polygon().points()) {
      polygon2d_pts[pt_num++] = {pt.x(), pt.y()};
    }
    polygon2d_nodes.emplace_back(math::Node<math::Polygon>{
        .id = polygon2d_id++, .shape = math::Polygon(polygon2d_pts)});

    Boundary boundary = decision.obstacle_boundary.obstacle.PolygonBoundary();
    boundary.left_expand(kIntervalL), boundary.right_expand(kIntervalL);
    boundaries_.emplace_back(boundary);
  }
  polygon2d_kdtree_.CreateKDT(polygon2d_nodes);
  std::sort(boundaries_.begin(), boundaries_.end(),
            [](auto& a, auto& b) { return a.start_s() < b.start_s(); });
  LOG_INFO("polygon2d_kdtree Length: {}", polygon2d_kdtree_.Length());
  LOG_INFO("boundaries size {}", boundaries_.size());

  /// Create map
  double start_s = ref_ptr_->ref_points().front().s();
  double end_s = ref_ptr_->ref_points().back().s();
  std::size_t road_size =
      std::max(static_cast<std::size_t>(1),
               static_cast<std::size_t>((end_s - start_s) / kIntervalLane));
  road_map_.resize(road_size);

  auto sample_lane = [](auto& ref_ptr, auto& lane, auto& id, auto& s_s,
                        auto& s_e) {
    std::vector<SimMapPoint> pts{};
    Vec2d pt{};
    for (double s = s_s; s < s_e; s += kIntervalPoint) {
      if (!ref_ptr->GetPointInCartesianFrame({s, id * kIntervalL}, &pt)) {
        continue;
      }
      pts.emplace_back(
          SimMapPoint{.x = pt.x(), .y = pt.y(), .s = s, .l = id * kIntervalL});
    }
    lane = SimMapLane{.lane_id = id, .valid = true, .pts = std::move(pts)};
  };
  ReferencePoint ref_pt{};
  for (std::size_t road_id = 0; road_id < road_size; ++road_id) {
    double s_s = std::min(start_s + kIntervalLane * road_id, end_s);
    double s_e = std::min(start_s + kIntervalLane * (road_id + 1), end_s);
    ReferencePoint s_pt, e_pt;
    if (!ref_ptr_->GetNearestRefPoint(s_s, &s_pt) ||
        !ref_ptr_->GetNearestRefPoint(s_e, &e_pt)) {
      break;
    }
    double r_r_b = std::min(s_pt.right_road_bound(), e_pt.right_road_bound()) -
                   kCarHalfWidth;
    double l_r_b = std::min(s_pt.left_road_bound(), e_pt.left_road_bound()) -
                   kCarHalfWidth;
    int right_lanes = std::max(0, static_cast<int>(r_r_b / kIntervalL));
    int center_lanes = 1;
    int left_lanes = std::max(0, static_cast<int>(l_r_b / kIntervalL));
    std::vector<SimMapLane> lanes(right_lanes + center_lanes + left_lanes);
    int lane_id = 0;
    for (int id = -right_lanes; id < 0; id++) {
      sample_lane(ref_ptr_, lanes[lane_id], id, s_s, s_e);
      lane_id++;
    }
    for (int id = 0; id < center_lanes; id++) {
      sample_lane(ref_ptr_, lanes[lane_id], id, s_s, s_e);
      lane_id++;
    }
    for (int id = center_lanes; id < left_lanes; id++) {
      sample_lane(ref_ptr_, lanes[lane_id], id, s_s, s_e);
      lane_id++;
    }

    road_map_[road_id] = SimMapRoad{
        .road_id = road_id, .s_s = s_s, .s_e = s_e, .lanes = std::move(lanes)};
  }

  sim_map_valid_ = true;

  return true;
}

bool SimMap::getNearestPoint(const Vec2d& pt, SimMapPoint* nearest_pt) {
  if (nearest_pt == nullptr) return false;
  SLPoint sl_pt{};
  if (!ref_ptr_->GetPointInFrenetFrame(pt, &sl_pt)) {
    return false;
  }

  *nearest_pt =
      SimMapPoint{.x = pt.x(), .y = pt.y(), .s = sl_pt.s(), .l = sl_pt.l()};

  return true;
}

bool SimMap::getNearestPoint(const SLPoint& sl_pt, SimMapPoint* nearest_pt) {
  if (nearest_pt == nullptr) return false;
  Vec2d pt;
  if (!ref_ptr_->GetPointInCartesianFrame(sl_pt, &pt)) {
    return false;
  }

  *nearest_pt =
      SimMapPoint{.x = pt.x(), .y = pt.y(), .s = sl_pt.s(), .l = sl_pt.l()};
  return true;
}

bool SimMap::getRoadLaneId(const Vec2d& pt, int* road_id, int* lane_id) {
  if (road_id == nullptr || lane_id == nullptr) return false;

  SLPoint sl_pt{};
  return getRoadLaneIdByPoint(ref_ptr_, road_map_, pt, &sl_pt, road_id,
                              lane_id);
}

bool SimMap::searchPoint(const Vec2d& pt, const double expact_extend_length,
                         const int expact_extend_lane_nums, Vec2d* goal_pt,
                         double* real_extend_length,
                         int* real_extend_lane_nums) {
  if (goal_pt == nullptr || real_extend_lane_nums == nullptr) return false;

  *goal_pt = pt, *real_extend_length = 0., *real_extend_lane_nums = 0;

  /// Match pt's road/lane id
  int road_id{-1}, lane_id{0};
  SLPoint sl_pt{};
  if (!getRoadLaneIdByPoint(ref_ptr_, road_map_, pt, &sl_pt, &road_id,
                            &lane_id)) {
    return false;
  }

  /// Match expact_extend_length's road_id and compute real_extend_length
  int expact_road_id{-1};
  if (!getRoadIdByS(road_map_, sl_pt.s() + expact_extend_length,
                    &expact_road_id)) {
    if (!road_map_.empty() && (road_map_.back().s_e > sl_pt.s())) {
      expact_road_id = road_map_.back().road_id;
    }
  }
  if (expact_road_id < 0) return false;
  // if (road_map_[expact_road_id].lanes.empty() ||
  //     road_map_[expact_road_id].lanes.back().pts.empty()) {
  //   return false;
  // }
  if (road_map_[expact_road_id].lanes.empty()) {
    return false;
  }

  *real_extend_length =
      std::min(expact_extend_length,
               (road_map_.empty() ? 0. : (road_map_.back().s_e - sl_pt.s())));

  /// Match expact_extend_lane_nums
  double expact_l = sl_pt.l() + kIntervalL * expact_extend_lane_nums;
  int expact_lane_id{0};
  if (!getLaneIdByL(road_map_, expact_l, expact_road_id, &expact_lane_id) &&
      road_map_[expact_road_id].lanes.empty()) {
    return false;
  }
  while (road_map_[expact_road_id].lanes.back().pts.empty()) {
    road_map_[expact_road_id].lanes.erase(
        road_map_[expact_road_id].lanes.end());
  }
  while (road_map_[expact_road_id].lanes.front().pts.empty()) {
    road_map_[expact_road_id].lanes.erase(
        road_map_[expact_road_id].lanes.begin());
  }
  expact_l =
      expact_extend_lane_nums >= 0
          ? std::min(expact_l,
                     road_map_[expact_road_id].lanes.back().pts.back().l)
          : std::max(expact_l,
                     road_map_[expact_road_id].lanes.front().pts.front().l);
  *real_extend_lane_nums =
      (expact_l < sl_pt.l())
          ? static_cast<int>((expact_l - sl_pt.l() - 1e-3) / kIntervalL)
          : static_cast<int>((expact_l - sl_pt.l() + 1e-3) / kIntervalL);

  if (*real_extend_lane_nums == 0) {
    *real_extend_lane_nums = (expact_l - sl_pt.l() < 0) ? -1 : 1;
  }

  /// Compute goal pt
  if (!ref_ptr_->GetPointInCartesianFrame(
          {sl_pt.s() + *real_extend_length, expact_l}, goal_pt)) {
    LOG_ERROR("GetPointInCartesianFrame failed.");
    return false;
  }

  return true;
}

bool SimMap::collisionCheck(const math::Polygon& polygon) {
  return (polygon2d_kdtree_.Length() >= 1 &&
          !polygon2d_kdtree_.GetOverlapNodesOf(polygon).empty());
}

bool SimMap::searchBoundary(const SLPoint& sl_pt, const double s,
                            Boundary* boundary, bool* ambiguity) {
  if (boundary == nullptr) return false;
  int road_id{-1}, lane_id{0};
  if (!getRoadIdByS(road_map_, sl_pt.s(), &road_id)) {
    LOG_ERROR("could not get road id by S.");
    return false;
  }
  if (!getLaneIdByL(road_map_, sl_pt.l(), road_id, &lane_id)) {
    LOG_ERROR("could not get lane id by L.");
    return false;
  }
  Segment2d sl_seg({sl_pt.s(), sl_pt.l()}, {sl_pt.s() + s, sl_pt.l()});
  for (const auto& bound : boundaries_) {
    Segment2d s_seg({bound.start_s(), sl_pt.l()}, {bound.end_s(), sl_pt.l()});
    Segment2d l_seg({sl_pt.s(), bound.start_l()}, {sl_pt.s(), bound.end_l()});
    if (sl_seg.has_intersect(s_seg) && sl_seg.has_intersect(l_seg)) {
      *boundary = bound;
      break;
    }
  }
  *ambiguity = true;
  return (boundary->start_s() <= boundary->end_s());
}

bool SimMap::searchBoundary(const Vec2d& pt, const double s, Boundary* boundary,
                            bool* ambiguity) {
  SimMapPoint sim_pt;
  if (!getNearestPoint(pt, &sim_pt)) {
    LOG_ERROR("getNearestPoint failed.");
    return false;
  }

  return searchBoundary(SLPoint(sim_pt.s, sim_pt.l), s, boundary, ambiguity);
}

bool SimMap::searchRoadBound(const Vec2d& pt, double* l_bound,
                             double* r_bound) {
  ReferencePoint ref_pt{};
  if (!ref_ptr_->GetNearestRefPoint(pt, &ref_pt)) return false;
  *l_bound = ref_pt.left_road_bound();
  *r_bound = ref_pt.right_road_bound();
  return true;
}

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive