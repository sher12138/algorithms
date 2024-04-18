#include "generator.h"

#include "common/planning_gflags.h"
#include "common/vehicle_param.h"
#include "hdmap/topo_map/lane_topomap.h"
#include "hdmap/topo_map/road_topomap.h"
#include "math/fempos/fem_pos_smoother.h"
#include "navigation/common/navigation_context.h"
#include "planning_map/planning_map.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/visualizer_event/visualizer_event.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/planning_map/planning_map.h"

namespace neodrive {
namespace planning {

// namespace {
using RefLaneSegment = Generator::RefLaneSegment;
using RouteLaneSegment = neodrive::global::routing::RoutingResult_LaneSegment;
using JunctionType = autobot::cyberverse::Junction::JunctionType;
using cyberverse::LaneInfoConstPtr;
using AD3 = std::array<double, 3>;
// using global::hdmap::Lane;
typedef global::hdmap::Lane::LaneTurn TurnType;

void clear(std::queue<LaneInfoConstPtr>& q) {
  std::queue<LaneInfoConstPtr> empty;
  std::swap(empty, q);
}

bool IsEqual(double a, double b) { return std::fabs(a - b) < 1e-3; }
bool IsEqual(ReferencePoint& a, ReferencePoint& b) {
  return IsEqual(a.x(), b.x()) && IsEqual(a.y(), b.y());
}
bool IsNewRoute(const RoutingResultShrPtr& routing_ptr) {
  static size_t last_sequence_num = 0;
  static double last_timestamp = 0.;
  auto new_route =
      (routing_ptr->header().sequence_num() != last_sequence_num &&
       std::abs(routing_ptr->header().timestamp_sec() - last_timestamp) > 0.1);
  last_sequence_num = routing_ptr->header().sequence_num();
  last_timestamp = routing_ptr->header().timestamp_sec();
  return new_route;
}

void ReloadHdMap(const RoutingResult& routing) { return; }

std::vector<RefLaneSegment> Generator::GetPreviousLaneSegments(
    const RefLaneSegment& curr_seg, const double distance) {
  auto hdmap = ctx_->hdmap;
  std::vector<RefLaneSegment> ans{curr_seg};

  double remain = distance;
  while (remain > 1e-2) {
    auto& id = ans.back().id;
    auto& s = ans.back().start_s;
    std::tie(s, remain) =
        std::pair{std::max(s - remain, 0.), std::max(remain - s, 0.)};
    if (remain > 1e-2) {
      std::vector<uint64_t> ls{};
      if (!PlanningMap::Instance()->GetPredecessorLanes(id, ls) || ls.empty())
        break;
      double lane_len{0.};
      if (!PlanningMap::Instance()->GetLengthOfLane(ls[0], lane_len)) break;
      ans.push_back({ls[0], lane_len, lane_len, hdmap->GetLaneById(ls[0])});
    }
  }

  LOG_INFO("Extend front lenth {:.4f}", distance - remain);
  std::reverse(ans.begin(), ans.end());
  return ans;
};

std::vector<RefLaneSegment> Generator::GetSuccessorLaneSegments(
    const RefLaneSegment& curr_seg, const double distance) {
  std::vector<RefLaneSegment> ans{curr_seg};
  auto hdmap = ctx_->hdmap;

  double remain = distance;
  while (remain > 1e-2) {
    auto& id = ans.back().id;
    auto& e = ans.back().end_s;
    double lane_len{0.};
    if (!PlanningMap::Instance()->GetLengthOfLane(id, lane_len)) break;
    std::tie(e, remain) = std::pair{std::min(e + remain, lane_len),
                                    std::max(remain - (lane_len - e), 0.)};
    if (remain > 1e-2) {
      std::vector<uint64_t> ls{};
      if (!PlanningMap::Instance()->GetSuccessorLanes(id, ls) || ls.empty())
        break;
      uint64_t succ_id = 0;
      for (auto id : ls) {
        auto lane = hdmap->GetLaneById(id);
        if (!lane) break;
        auto road = hdmap->GetRoadById(lane->RoadId());
        if (!road) break;
        auto junction = hdmap->GetJunctionById(road->JunctionId());
        bool is_in_junction_turn =
            junction != nullptr &&
            lane->TurnType() !=
                static_cast<uint32_t>(global::hdmap::Lane::NO_TURN);
        if (!is_in_junction_turn) {
          succ_id = id;
          break;
        }
      }
      if (succ_id == 0) break;
      ans.push_back({succ_id, 0., 0., hdmap->GetLaneById(succ_id)});
    }
  }

  LOG_INFO("Extend end lenth {:.4f}", distance - remain);
  return ans;
};

LaneMeetingRanges::Ptr GetLaneMeetingRanges(const uint64_t id) {
  auto lanes = PlanningMap::Instance()->GetLaneOverlapLanes(id);
  if (lanes.empty()) return LaneMeetingRanges::Ptr{};

  // Extend to len
  using LaneSeg = LaneMeetingRanges::LaneSegment;
  using VecSeg = std::vector<LaneSeg>;
  std::function<void(VecSeg&, double, std::vector<VecSeg>*)> dfs_prev =
      [&dfs_prev](
          std::vector<neodrive::planning::LaneMeetingRanges::LaneSegment>& path,
          double len,
          std::vector<
              std::vector<neodrive::planning::LaneMeetingRanges::LaneSegment>>*
              ans) -> void {
    if (path.empty()) return;
    auto valid = std::min(len, path.back().end_s);
    len -= valid;
    path.back().start_s = path.back().end_s - valid;
    if (len < 1e-3) {
      ans->push_back(path);
      return;
    }

    std::vector<uint64_t> prevs{};
    if (!PlanningMap::Instance()->GetPredecessorLanes(path.back().id, prevs) ||
        prevs.empty()) {
      ans->push_back(path);
      return;
    }

    for (auto& prev : prevs) {
      double lane_len{};
      if (!PlanningMap::Instance()->GetLengthOfLane(prev, lane_len)) continue;
      path.push_back(LaneSeg{.id = prev, .end_s = lane_len});
      dfs_prev(path, len, ans);
      path.pop_back();
    }
  };

  auto ans = std::make_shared<LaneMeetingRanges>();
  for (auto& [meet, s, e] : lanes) {
    auto laps = PlanningMap::Instance()->GetLaneOverlapLanes(meet);
    auto it = std::find_if(laps.begin(), laps.end(),
                           [&id](auto& l) { return std::get<0>(l) == id; });
    if (it == laps.end()) continue;
    std::vector<LaneSeg> path{{.id = meet, .end_s = std::get<2>(*it)}};
    ans->meetings.push_back({.curr{id, s, e}});
    dfs_prev(path, 50., &ans->meetings.back().prevs);
  }

  return ans;
}

bool Generator::IsMotorway(uint64_t id) {
  auto hdmap = ctx_->hdmap;
  auto lane = hdmap->GetLaneById(id);
  if (lane == nullptr) return false;
  for (int i = 0; i < lane->LaneMultipleType().size(); ++i) {
    if (lane->LaneMultipleType()[i] ==
        static_cast<uint32_t>(global::hdmap::Lane::BIKING))
      return false;
  }
  return true;
}

std::vector<ReferenceLine::TrafficOverlap> GetSignalOverlap(
    const RefLaneSegment& lane_seg) {
  auto lane = lane_seg.lane_ptr;
  std::vector<ReferenceLine::TrafficOverlap> ans{};
  std::vector<ReferenceLine::TrafficOverlap> tmp{};
  for (int i = 0; i < lane->Signals().size(); ++i) {
    tmp.push_back({lane->Signals()[i].object_id, lane->Signals()[i].start_s,
                   lane->Signals()[i].end_s});
  }
  std::sort(tmp.begin(), tmp.end(), [](auto& a, auto& b) {
    auto [a_id, a_s, a_e] = a;
    auto [b_id, b_s, b_e] = b;
    return a_s < b_s;
  });
  for (const auto& [id, s, e] : tmp) {
    if (!(s > lane_seg.end_s || e < lane_seg.start_s)) {
      if (ans.empty()) {
        ans.push_back({id, s, e});
      } else {
        if ((std::abs(s - ans.back().start_s) >= 0.01 &&
             std::abs(e - ans.back().end_s) >= 0.01) ||
            id != ans.back().object_id) {
          ans.push_back({id, s, e});
        }
      }
    }
  }
  return ans;
}

RefLaneSegment Generator::BuildRefLaneSegment(const RefLaneSegment& lane_seg) {
  auto hdmap = PlanningMap::Instance();
  auto ToOverlap = [](const auto& segs, auto& seg) {
    std::vector<ReferenceLine::TrafficOverlap> ans{};
    auto copy_segs = segs;
    std::sort(copy_segs.begin(), copy_segs.end(), [](auto& a, auto& b) {
      auto [a_id, a_s, a_e] = a;
      auto [b_id, b_s, b_e] = b;
      return a_s < b_s;
    });
    for (const auto& [id, s, e] : copy_segs) {
      if (!(s > seg.end_s || e < seg.start_s)) {
        if (ans.empty()) {
          ans.push_back({id, s, e});
        } else {
          if ((std::abs(s - ans.back().start_s) >= 0.01 &&
               std::abs(e - ans.back().end_s) >= 0.01) ||
              id != ans.back().object_id) {
            ans.push_back({id, s, e});
          }
        }
      }
    }
    return ans;
  };
  RefLaneSegment ans{
      .id = lane_seg.id,
      .start_s = lane_seg.start_s,
      .end_s = lane_seg.end_s,
      .lane_ptr = lane_seg.lane_ptr,

      .left_bound_type = hdmap->GetLaneLeftBoundaryType(lane_seg.id),
      .right_bound_type = hdmap->GetLaneRightBoundaryType(lane_seg.id),
      .speed_limit = hdmap->GetLaneSpeedLimit(lane_seg.id),

      .crosswalks = ToOverlap(hdmap->GetLaneCrosswalks(lane_seg.id), lane_seg),
      .signals = GetSignalOverlap(lane_seg),
      .yield_signs = ToOverlap(hdmap->GetLaneYieldSigns(lane_seg.id), lane_seg),
      .stop_signs = ToOverlap(hdmap->GetLaneStopSigns(lane_seg.id), lane_seg),
      .speed_bumps = ToOverlap(hdmap->GetLaneSpeedBumps(lane_seg.id), lane_seg),
      .clear_areas = ToOverlap(hdmap->GetLaneClearAreas(lane_seg.id), lane_seg),
      .junctions = ToOverlap(hdmap->GetLaneJunctions(lane_seg.id), lane_seg),
      .geo_fences = ToOverlap(hdmap->GetLaneGeoFences(lane_seg.id), lane_seg),
      .barrier_gates = ToOverlap(hdmap->GetBarrierGates(lane_seg.id), lane_seg),
      .lane_meeting_ranges = GetLaneMeetingRanges(lane_seg.id),

      .is_left_accessible = hdmap->IsLaneLeftDistanceAccessibleWithMode(
          lane_seg.id, 0, FLAGS_planning_lane_borrow_mode),
      .is_right_accessible = hdmap->IsLaneRightDistanceAccessibleWithMode(
          lane_seg.id, 0, FLAGS_planning_lane_borrow_mode),

      .is_left_lane = hdmap->IsLeftLane(lane_seg.id),
      .is_right_lane = hdmap->IsRightLane(lane_seg.id),
      .is_motorway = IsMotorway(lane_seg.id),
  };
  hdmap->GetLaneTurnType(ans.id, &ans.turn_type);
  hdmap->GetLaneMultipleType(ans.id, ans.lane_multiple_type);

  std::map<double, DividerFeature> left_dividers{}, right_dividers{};
  hdmap->GetLaneDividers(lane_seg.id, lane_seg.start_s, left_dividers,
                         right_dividers);
  for (const auto& divider : left_dividers) {
    if (ans.left_dividers.empty()) {
      std::vector<DividerFeature> feature;
      feature.emplace_back(divider.second);
      ans.left_dividers.push_back({divider.first, feature});
      continue;
    }
    if (std::abs(ans.left_dividers.back().first - divider.first) <
        kMathEpsilon) {
      ans.left_dividers.back().second.emplace_back(divider.second);
    } else {
      std::vector<DividerFeature> feature;
      feature.push_back(divider.second);
      ans.left_dividers.push_back({divider.first, feature});
    }
  }
  for (const auto& divider : right_dividers) {
    if (ans.right_dividers.empty()) {
      std::vector<DividerFeature> feature;
      feature.emplace_back(divider.second);
      ans.right_dividers.push_back({divider.first, feature});
      continue;
    }
    if (std::abs(ans.right_dividers.back().first - divider.first) <
        kMathEpsilon) {
      ans.right_dividers.back().second.emplace_back(divider.second);
    } else {
      std::vector<DividerFeature> feature;
      feature.emplace_back(divider.second);
      ans.right_dividers.push_back({divider.first, feature});
    }
  }

  hdmap->GetNeighborReverseRoadInfo(lane_seg.id, ans.reverse_road_info);

  if (auto ls = hdmap->GetLaneLeftForwardLanes(ans.id); !ls.empty()) {
    auto [lw, rw] = hdmap->GetLaneDistanceWidth(ans.id, 0);
    ans.left_forward_width = lw + rw;
  }
  if (auto ls = hdmap->GetLaneLeftBackwardLanes(ans.id); !ls.empty()) {
    auto [lw, rw] = hdmap->GetLaneDistanceWidth(ans.id, 0);
    ans.left_backward_width = lw + rw;
  }

  if (auto ls = hdmap->GetLaneRightForwardLanes(ans.id); !ls.empty()) {
    auto [lw, rw] = hdmap->GetLaneDistanceWidth(ans.id, 0);
    ans.right_forward_width = lw + rw;
  }

  if (auto ls = hdmap->GetLaneRightBackwardLanes(ans.id); !ls.empty()) {
    auto [lw, rw] = hdmap->GetLaneDistanceWidth(ans.id, 0);
    ans.right_backward_width = lw + rw;
  }

  // Extend length of overlaps to avoid skiping by reference point
  using namespace std;
  double len{0};
  hdmap->GetLengthOfLane(ans.id, len);
  for (auto& [id, s, e] : ans.crosswalks) {
    e = min(max(s + 0.5, e), len);
  }
  for (auto& [id, s, e] : ans.signals) {
    e = min(max(s + 0.5, e), len);
  }
  for (auto& [id, s, e] : ans.yield_signs) {
    e = min(max(s + 0.5, e), len);
  }
  for (auto& [id, s, e] : ans.stop_signs) {
    e = min(max(s + 0.5, e), len);
  }
  for (auto& [id, s, e] : ans.speed_bumps) {
    e = min(max(s + 0.5, e), len);
  }
  for (auto& [id, s, e] : ans.clear_areas) {
    e = min(max(s + 0.5, e), len);
  }
  for (auto& [id, s, e] : ans.junctions) {
    e = min(max(s + 0.5, e), len);
  }
  for (auto& [id, s, e] : ans.barrier_gates) {
    e = min(max(s + 0.5, e), len);
  }

  auto lt = [](auto& a, auto& b) { return a.start_s < b.start_s; };
  std::sort(ans.crosswalks.begin(), ans.crosswalks.end(), lt);
  std::sort(ans.signals.begin(), ans.signals.end(), lt);
  std::sort(ans.yield_signs.begin(), ans.yield_signs.end(), lt);
  std::sort(ans.stop_signs.begin(), ans.stop_signs.end(), lt);
  std::sort(ans.speed_bumps.begin(), ans.speed_bumps.end(), lt);
  std::sort(ans.clear_areas.begin(), ans.clear_areas.end(), lt);
  std::sort(ans.junctions.begin(), ans.junctions.end(), lt);
  std::sort(ans.geo_fences.begin(), ans.geo_fences.end(), lt);
  std::sort(ans.barrier_gates.begin(), ans.barrier_gates.end(), lt);

  return ans;  // RVO
}

void MergeBrokenLaneSegments(std::vector<RefLaneSegment>* const lane_segs) {
  std::vector<RefLaneSegment> tmp{};
  for (auto& lane_seg : *lane_segs) {
    if (!tmp.empty() && tmp.back().id == lane_seg.id &&
        std::abs(tmp.back().end_s - lane_seg.end_s) < 0.1) {
      tmp.back().end_s = lane_seg.end_s;
    } else {
      tmp.push_back(lane_seg);
    }
  }
  std::swap(*lane_segs, tmp);
}

void LogLaneSegmentInfo(const std::vector<RefLaneSegment>& ref_lane_segs) {
  for (auto& seg : ref_lane_segs) {
    LOG_INFO("-> lane seg(id, start, end) [{}, {:.4f}, {:.4f}] has properties:",
             PlanningMap::Instance()->GetHashIdString(seg.id), seg.start_s,
             seg.end_s);
    LOG_INFO(
        "---- turn type {}, left_bound_type {}, right_bound_type {}, "
        "speed_limit {:.4f},",
        static_cast<int>(seg.turn_type), static_cast<int>(seg.left_bound_type),
        static_cast<int>(seg.right_bound_type), seg.speed_limit);
    auto reverse_info = [](const auto& info) {
      std::string ans{""};
      for (const auto& f : info) {
        ans += PlanningMap::Instance()->GetHashIdString(f.first) + " ";
        for (const auto& t : f.second)
          ans += std::to_string(static_cast<int>(t)) + " ";
        ans += " | ";
      }
      return ans;
    };
    LOG_INFO("---- reverse_info {}", reverse_info(seg.reverse_road_info));

    for (auto& [id, s, e] : seg.crosswalks)
      LOG_INFO("---- crosswalk [{}, {:.4f}, {:.4f}]",
               PlanningMap::Instance()->GetHashIdString(id), s, e);
    for (auto& [id, s, e] : seg.signals)
      LOG_INFO("---- signal [{}, {:.4f}, {:.4f}]",
               PlanningMap::Instance()->GetHashIdString(id), s, e);
    for (auto& [id, s, e] : seg.yield_signs)
      LOG_INFO("---- yield_sign [{}, {:.4f}, {:.4f}]",
               PlanningMap::Instance()->GetHashIdString(id), s, e);
    for (auto& [id, s, e] : seg.stop_signs)
      LOG_INFO("---- stop_sign [{}, {:.4f}, {:.4f}]",
               PlanningMap::Instance()->GetHashIdString(id), s, e);
    for (auto& [id, s, e] : seg.speed_bumps)
      LOG_INFO("---- speed_bump [{}, {:.4f}, {:.4f}]",
               PlanningMap::Instance()->GetHashIdString(id), s, e);
    for (auto& [id, s, e] : seg.clear_areas)
      LOG_INFO("---- clear_area [{}, {:.4f}, {:.4f}]",
               PlanningMap::Instance()->GetHashIdString(id), s, e);
    for (auto& [id, s, e] : seg.junctions)
      LOG_INFO("---- junction [{}, {:.4f}, {:.4f}]",
               PlanningMap::Instance()->GetHashIdString(id), s, e);
    for (auto& [id, s, e] : seg.geo_fences)
      LOG_INFO("---- geo_fence [{}, {:.4f}, {:.4f}]",
               PlanningMap::Instance()->GetHashIdString(id), s, e);
    for (auto& [id, s, e] : seg.barrier_gates)
      LOG_INFO("---- barrier_gate [{}, {:.4f}, {:.4f}]",
               PlanningMap::Instance()->GetHashIdString(id), s, e);

    if (auto meet = seg.lane_meeting_ranges; meet) {
      for (auto& [curr, prevs] : meet->meetings) {
        LOG_INFO("---- meeting range: [{}, {:.4f}, {:.4f}]",
                 PlanningMap::Instance()->GetHashIdString(curr.id),
                 curr.start_s, curr.end_s);
        for (auto& prev : prevs) {
          LOG_INFO("------ lane sequence(id, start, end): ");
          for (auto& p : prev)
            LOG_INFO("({}, {:.4f}, {:.4f})",
                     PlanningMap::Instance()->GetHashIdString(p.id), p.start_s,
                     p.end_s);
        }
      }
    }

    LOG_INFO(
        "---- is_left_accessible {}, left_forward_width {:.4f}, "
        "left_backward_width {:.4f}, is_right_accessible {}, "
        "right_forward_width {:.4f}, right_backward_widht {:.4f}, "
        "accumulate_s {:.4f}",
        seg.is_left_accessible, seg.left_forward_width, seg.left_backward_width,
        seg.is_right_accessible, seg.right_forward_width,
        seg.right_backward_width, seg.accumulate_s);

    LOG_INFO("---- is_left_lane {}, is_right_lane {}", seg.is_left_lane,
             seg.is_right_lane);

    auto divider_info = [](const auto& feature) {
      std::string info{""};
      for (const auto& f : feature) {
        info += std::to_string(f.first) + " ";
        for (const auto& s : f.second)
          info += std::to_string(s.is_virtual_) + " " +
                  std::to_string(static_cast<int>(s.divider_color_)) + " " +
                  std::to_string(static_cast<int>(s.divider_type_)) + " | ";
        info += " || ";
      }
      return info;
    };
    LOG_INFO("---- left_dividers {}", divider_info(seg.left_dividers));
    LOG_INFO("---- right_dividers {}", divider_info(seg.right_dividers));
  }
}

void Generator::ExtendRefLaneSegmentFrontAndTail(
    const double extend_front, const double extend_tail,
    std::vector<RefLaneSegment>* const ref_segs,
    std::vector<RefLaneSegment>* const tail_segs) {
  if (!ref_segs || ref_segs->empty()) return;

  std::vector<RefLaneSegment> tmp{};

  auto fronts =
      GetPreviousLaneSegments({(*ref_segs)[0].id, (*ref_segs)[0].start_s,
                               (*ref_segs)[0].end_s, (*ref_segs)[0].lane_ptr},
                              extend_front);
  for (auto& seg : fronts) tmp.emplace_back(seg);

  if (ref_segs->size() > 2)
    tmp.insert(tmp.end(), ref_segs->begin() + 1, ref_segs->end() - 1);

  auto tails = GetSuccessorLaneSegments(
      {ref_segs->back().id, ref_segs->back().start_s, ref_segs->back().end_s,
       ref_segs->back().lane_ptr},
      extend_tail);
  if (tails.size() && tails.front().id == tmp.back().id) {
    tmp.back().end_s = std::max(tails.front().end_s, tmp.back().end_s);
  } else {
    tail_segs->push_back(tails[0]);
    tmp.push_back(tails[0]);
  }
  for (int i = 1; i < tails.size(); ++i) {
    tail_segs->push_back(tails[i]);
    tmp.push_back(tails[i]);
  }

  std::swap(*ref_segs, tmp);
}

std::vector<RouteLaneSegment> GetRouteLanes(
    const std::vector<cyberverse::LaneInfoConstPtr>& route_lanes) {
  std::vector<RouteLaneSegment> ans;
  // for (auto& route : routing_ptr->route()) {
  //   if (route.has_road_info()) {
  //     for (auto& passage_region : route.road_info().passage_region()) {
  //       ans.insert(ans.end(), passage_region.segment().begin(),
  //                  passage_region.segment().end());
  //     }
  //   } else {
  //     ans.insert(ans.end(),
  //                route.junction_info().passage_region().segment().begin(),
  //                route.junction_info().passage_region().segment().end());
  //   }
  // }
  return ans;
}

void GereateSpeedLimitsFromRef(const std::vector<RefLaneSegment>& lane_segs,
                               SpeedLimitShrPtrMap& speed_limit_map) {
  SpeedLimitShrPtr last_limit = nullptr;
  for (auto& each_seg : lane_segs) {
    if (each_seg.speed_limit < 1e-3) {
      last_limit = nullptr;
      continue;
    }
    if (last_limit == nullptr) {
      last_limit = std::make_shared<SpeedLimitType>();
      last_limit->set_upper_bound(each_seg.speed_limit);
      last_limit->set_zone_start_distance(each_seg.accumulate_s);
      last_limit->set_zone_end_distance(each_seg.accumulate_s + each_seg.end_s -
                                        each_seg.start_s);
      last_limit->set_zone_type(SpeedLimitType::FINITE_ZONE);
      speed_limit_map.emplace(last_limit->zone_start_distance(), last_limit);
    } else if (std::fabs(each_seg.speed_limit - last_limit->upper_bound()) <
               1e-3) {
      last_limit->set_zone_end_distance(each_seg.accumulate_s + each_seg.end_s -
                                        each_seg.start_s);
    } else {
      last_limit = std::make_shared<SpeedLimitType>();
      last_limit->set_upper_bound(each_seg.speed_limit);
      last_limit->set_zone_start_distance(each_seg.accumulate_s);
      last_limit->set_zone_end_distance(each_seg.accumulate_s + each_seg.end_s -
                                        each_seg.start_s);
      last_limit->set_zone_type(SpeedLimitType::FINITE_ZONE);
      speed_limit_map.emplace(last_limit->zone_start_distance(), last_limit);
    }
  }
}

void GereateJunctionsSpeedLimitsFromRef(
    const std::vector<RefLaneSegment>& lane_segs, const float speed_limit,
    SpeedLimitShrPtrMap& speed_limit_map) {
  SpeedLimitShrPtr last_limit = nullptr;
  uint64_t last_id = 0;
  bool updated = false;
  for (auto& each_seg : lane_segs) {
    updated = false;
    for (auto& each_junction : each_seg.junctions) {
      cyberverse::JunctionInfoConstPtr junction_ptr{nullptr};
      if (!PlanningMap::Instance()->GetJunctionById(each_junction.object_id,
                                                    junction_ptr) ||
          !junction_ptr)
        continue;
      uint32_t junction_type = junction_ptr->Type();
      LOG_INFO(
          "lane:{},junction:{},start:{},end:{},type:{}",
          PlanningMap::Instance()->GetHashIdString(each_seg.id),
          PlanningMap::Instance()->GetHashIdString(each_junction.object_id),
          each_seg.accumulate_s + each_junction.start_s,
          each_seg.accumulate_s + each_junction.end_s, junction_type);
      if (junction_type == static_cast<uint32_t>(JunctionType::UNKNOWN) ||
          junction_type == static_cast<uint32_t>(JunctionType::IN_ROAD))
        continue;
      if (last_limit == nullptr) {
        last_limit = std::make_shared<SpeedLimitType>();
        last_id = each_junction.object_id;
        last_limit->set_upper_bound(speed_limit);
        last_limit->set_zone_start_distance(each_seg.accumulate_s +
                                            each_junction.start_s);
        last_limit->set_zone_end_distance(each_seg.accumulate_s +
                                          each_junction.end_s);
        last_limit->set_zone_type(SpeedLimitType::FINITE_ZONE);
        last_limit->set_source_type(SpeedLimitType::JUNCTION_SPEEDLIMIT);
        speed_limit_map.emplace(last_limit->zone_start_distance(), last_limit);
        updated = true;
      } else if (each_junction.object_id == last_id) {
        last_limit->set_zone_end_distance(each_seg.accumulate_s +
                                          each_junction.end_s);
        updated = true;
      } else {
        last_limit = std::make_shared<SpeedLimitType>();
        last_id = each_junction.object_id;
        last_limit->set_upper_bound(speed_limit);
        last_limit->set_zone_start_distance(each_seg.accumulate_s +
                                            each_junction.start_s);
        last_limit->set_zone_end_distance(each_seg.accumulate_s +
                                          each_junction.end_s);
        last_limit->set_zone_type(SpeedLimitType::FINITE_ZONE);
        last_limit->set_source_type(SpeedLimitType::JUNCTION_SPEEDLIMIT);
        speed_limit_map.emplace(last_limit->zone_start_distance(), last_limit);
        updated = true;
      }
    }
    if (!updated) {
      last_limit = nullptr;
      last_id = 0;
    }
  }
}

void LogRefPoint(const ReferencePoint& pt) {
  auto lane_type_string = [](const std::vector<DividerFeature>& feature) {
    std::string lane_info{""};
    for (const auto& f : feature) {
      lane_info += std::to_string(f.is_virtual_) + " " +
                   std::to_string(static_cast<int>(f.divider_color_)) + " " +
                   std::to_string(static_cast<int>(f.divider_type_));
      lane_info += "; ";
    }
    return lane_info;
  };
  LOG_INFO(
      "pt(s, x, y, l, r) [{:.4f}, {:.4f}, {:.4f}, {}, {}] on [{}, {:.4f}] has "
      "crosswalk {}, "
      "signal {}, yield_sign {}, stop_sign {}, speed_bump {}, clear_area {}, "
      "junction {}, barrier_gate {}, meeting {}, left_bound {:.4f}, "
      "left_lane_bound {:.4f}, left_road_bound {:.4f}, left_reverse_bound "
      "{:.4f}, right_bound {:.4f}, "
      "right_lane_bound {:.4f}, right_road_bound {:.4f}, left_type_info {}, "
      "right_type_info {}, ",
      pt.s(), pt.x(), pt.y(), pt.is_left_lane(), pt.is_right_lane(),
      PlanningMap::Instance()->GetHashIdString(pt.hd_map_lane_id()),
      pt.hd_map_lane_s(), pt.is_in_crosswalk(), pt.is_in_signal(),
      pt.is_in_yield_sign(), pt.is_in_stop_sign(), pt.is_in_speed_bump(),
      pt.is_in_clear_area(), pt.is_in_junction(), pt.is_in_barrier_gate(),
      (bool)pt.lane_meeting_ranges(), pt.left_bound(), pt.left_lane_bound(),
      pt.left_road_bound(), pt.left_reverse_road_bound(), pt.right_bound(),
      pt.right_lane_bound(), pt.right_road_bound(),
      lane_type_string(pt.left_divider_feature()),
      lane_type_string(pt.right_divider_feature()));
}

bool CheckSegIdx(const std::vector<Generator::RefLaneSegment>& segs,
                 ReferencePoint& pt) {
  auto hdmap = cyberverse::HDMap::Instance();
  auto cur_lane = segs[pt.seg_idx()].lane_ptr;
  if (cur_lane->IsOnLane({pt.x(), pt.y()})) return true;

  size_t start_idx = segs.size() < 3 ? 0 : pt.seg_idx() - 2;
  size_t end_idx = std::min(pt.seg_idx() + 2, segs.size() - 1);
  std::vector<LaneInfoConstPtr> lanes;
  hdmap->GetLanes({pt.x(), pt.y()}, 3.0, &lanes);
  if (lanes.empty()) {
    LOG_ERROR("find no lane at pt({}, {}, {})", pt.x(), pt.y(), pt.z());
    return false;
  }
  std::unordered_map<uint64_t, size_t> lane_id_map;
  for (size_t i = start_idx; i <= end_idx; ++i)
    lane_id_map.insert(std::make_pair(segs[i].id, i));

  for (auto lane : lanes) {
    if (lane_id_map.find(lane->Id()) != lane_id_map.end()) {
      LOG_INFO("update pt({}, {}, {}) from {} to {}, idx from {} to {}", pt.x(),
               pt.y(), pt.z(), hdmap->GetIdHashString(cur_lane->Id()),
               hdmap->GetIdHashString(lane->Id()), pt.seg_idx(),
               lane_id_map.find(lane->Id())->second);
      pt.set_seg_idx(lane_id_map.find(lane->Id())->second);
      return true;
    }
  }
  LOG_ERROR("no matching lane at pt({}, {}, {})", pt.x(), pt.y(), pt.z());
  return false;
}

void AddRefPointLaneInfo(const std::vector<Generator::RefLaneSegment>& segs,
                         std::vector<ReferencePoint>& ref_pt, int start_idx,
                         std::vector<ReferencePoint>& main_refe_pt) {
  static config::PlanningConfig* planning_config =
      config::PlanningConfig::Instance();

  // Set point properties according to s
  auto hdmap = PlanningMap::Instance();
  const double default_ref_width = FLAGS_default_reference_line_width / 2;
  struct Valid {
    double x{0};
    bool operator()(const ReferenceLine::TrafficOverlap& v) {
      return v.start_s < x && v.end_s > x;
    }
  };
  struct ValidRange {
    double x{0};
    bool operator()(const LaneMeetingRanges::MeetingRange& v) {
      return v.curr.start_s < x && v.curr.end_s > x;
    }
  };
  /// Binary search right bound (a[l] >= v)
  auto RbsIdx =
      [](const std::vector<std::pair<double, std::vector<DividerFeature>>>& a,
         const double v) {
        int ans{0};
        for (size_t i = 1; i + 1 < a.size(); ++i) {
          if (a[i].first >= v && a[i + 1].first <= v) {
            ans = i;
            break;
          }
        }
        if (v <= a.front().first) ans = 0;
        if (v >= a.back().first) ans = a.size() - 1;
        return ans;
      };
  // main_refe_pt.reserve(main_refe_pt.size() + ref_pt.size() - start_idx);
  main_refe_pt.resize(start_idx);
  if (!main_refe_pt.empty() && !ref_pt.empty()) {
    LOG_INFO("end ref pt s,x,y:{:.4f}, {:.4f}, {:.4f}", main_refe_pt.back().s(),
             main_refe_pt.back().x(), main_refe_pt.back().y());
    LOG_INFO("new ref pt s,x,y:{:.4f}, {:.4f}, {:.4f}", ref_pt[0].s(),
             ref_pt[0].x(), ref_pt[0].y());
  }
  for (auto i = 0; i < ref_pt.size(); ++i) {
    auto& pt = ref_pt[i];
    CheckSegIdx(segs, pt);
    auto& seg = segs[pt.seg_idx()];
    double drive_strategy_speed_limit =
        seg.is_motorway ? common::config::CommonConfig::Instance()
                              ->drive_strategy_config()
                              .motor_way.max_cruise_speed
                        : common::config::CommonConfig::Instance()
                              ->drive_strategy_config()
                              .non_motorway.max_cruise_speed;
    pt.set_hd_map_lane_id(seg.id);
    if (double s, l; hdmap->GetSLWithLane(seg.id, pt.x(), pt.y(), s, l)) {
      pt.set_hd_map_lane_s(s);
      pt.set_hd_map_lane_l(l);
    }
    if (seg.turn_type == Lane::TurningType::NO_TURN) pt.set_no_signal();
    if (seg.turn_type == Lane::TurningType::LEFT_TURN) pt.set_left_signal();
    if (seg.turn_type == Lane::TurningType::RIGHT_TURN) pt.set_right_signal();
    if (seg.turn_type == Lane::TurningType::U_TURN) pt.set_uturn_signal();

    pt.set_left_boundary_edge_type(seg.left_bound_type);
    pt.set_right_boundary_edge_type(seg.right_bound_type);
    pt.set_multiple_type(seg.lane_multiple_type);
    pt.set_speed_limit(!FLAGS_planning_enable_hdmap_speed_limit ||
                               seg.speed_limit < 0.03
                           ? drive_strategy_speed_limit
                           : seg.speed_limit);
    auto cs = pt.hd_map_lane_s();
    pt.set_is_in_crosswalk(
        std::any_of(seg.crosswalks.begin(), seg.crosswalks.end(), Valid{cs}));
    pt.set_is_in_signal(
        std::any_of(seg.signals.begin(), seg.signals.end(), Valid{cs}));
    pt.set_is_in_yield_sign(
        std::any_of(seg.yield_signs.begin(), seg.yield_signs.end(), Valid{cs}));
    pt.set_is_in_stop_sign(
        std::any_of(seg.stop_signs.begin(), seg.stop_signs.end(), Valid{cs}));
    pt.set_is_in_speed_bump(
        std::any_of(seg.speed_bumps.begin(), seg.speed_bumps.end(), Valid{cs}));
    pt.set_is_in_clear_area(
        std::any_of(seg.clear_areas.begin(), seg.clear_areas.end(), Valid{cs}));
    pt.set_is_in_junction(
        std::any_of(seg.junctions.begin(), seg.junctions.end(), Valid{cs}));
    pt.set_is_in_barrier_gate(std::any_of(seg.barrier_gates.begin(),
                                          seg.barrier_gates.end(), Valid{cs}));

    auto [lw, rw] = hdmap->GetLaneDistanceWidth(seg.id, pt.hd_map_lane_s());
    pt.set_left_lane_bound(std::max(lw - pt.hd_map_lane_l(), 0.));
    pt.set_left_bound(pt.left_lane_bound() > 0.01 ? pt.left_lane_bound()
                                                  : default_ref_width);
    pt.set_right_lane_bound(std::max(rw + pt.hd_map_lane_l(), 0.));
    pt.set_right_bound(pt.right_lane_bound() > 0.01 ? pt.right_lane_bound()
                                                    : default_ref_width);
    pt.set_left_road_bound(default_ref_width);
    pt.set_left_reverse_road_bound(default_ref_width);
    if (double b, reverse_b; hdmap->GetPointLaneLeftRoadBound(
            pt.x(), pt.y(), seg.id, b, reverse_b)) {
      pt.set_left_road_bound(b);
      pt.set_left_reverse_road_bound(reverse_b);
    }
    pt.set_right_road_bound(default_ref_width);
    if (double b;
        hdmap->GetPointLaneRightRoadBound(pt.x(), pt.y(), seg.id, b)) {
      pt.set_right_road_bound(b);
    }

    pt.set_is_left_lane(seg.is_left_lane);
    pt.set_is_right_lane(seg.is_right_lane);
    if (!seg.left_dividers.empty()) {
      auto index = RbsIdx(seg.left_dividers, pt.s());
      pt.set_left_divider_feature(seg.left_dividers[index].second);
    }
    if (!seg.right_dividers.empty()) {
      auto index = RbsIdx(seg.right_dividers, pt.s());
      pt.set_right_divider_feature(seg.right_dividers[index].second);
    }

    if (seg.lane_meeting_ranges &&
        std::any_of(seg.lane_meeting_ranges->meetings.begin(),
                    seg.lane_meeting_ranges->meetings.end(), ValidRange{cs})) {
      pt.set_lane_meeting_ranges(seg.lane_meeting_ranges);
    }
    main_refe_pt.push_back(pt);
  }
}

static void VisRefPoints(const std::vector<ReferencePoint>& points,
                         const std::string& name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kUtm);

  auto set_pts = [](auto event, auto& pts) {
    for (auto& pt : pts) {
      auto sphere = event->mutable_sphere()->Add();
      sphere->mutable_center()->set_x(pt.x());
      sphere->mutable_center()->set_y(pt.y());
      sphere->mutable_center()->set_z(0);
      sphere->set_radius(0.1);
    }
  };

  set_pts(event, points);
}

void VisPolyline(const std::vector<AD3>& polyline, const std::string& name,
                 const std::array<double, 4>& col) {
  if (!FLAGS_planning_enable_vis_event) return;
  LOG_INFO("start VisPolyline: {}", name);
  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kUtm);
  event->mutable_color()->set_r(col[0]);
  event->mutable_color()->set_g(col[1]);
  event->mutable_color()->set_b(col[2]);
  event->mutable_color()->set_a(col[3]);
  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p[0]), ans->set_y(p[1]), ans->set_z(0);
  };

  auto pl = event->add_polyline();
  for (auto& p : polyline) {
    set_pt(pl->add_point(), p);
  }
}

void VisPaths(const std::vector<ReferencePoint>& paths,
              const std::string& name) {
  if (!FLAGS_planning_enable_vis_event) return;
  LOG_INFO("start VisPolyline");

  std::vector<AD3> pts{};
  for (auto& path : paths) {
    // LOG_INFO("point size {}", path.xs.size());
    // for (size_t i = 0; i < path.xs.size(); ++i) {
    pts.push_back({path.x(), path.y(), 0});
    // }
  }

  VisPolyline(pts, name, {1, 0, 0, 1});
}

void StitchRefPoints(ReferenceLinePtr* const ref_line,
                     std::vector<ReferencePoint>& cur_ref,
                     const double curr_init_s) {
  LOG_INFO("StitchRefPoints");
  if (cur_ref.empty()) return;
  std::vector<ReferencePoint> ans{};
  std::vector<ReferencePoint> stitch_pts{};
  const auto generator_config =
      config::PlanningConfig::Instance()
          ->planning_research_config()
          .reference_line_config.curise_generator_config;
  for (const auto& pt : cur_ref) {
    if (pt.s() > curr_init_s) stitch_pts.push_back(pt);
  }

  // Smooth
  std::vector<EvaluatedPoint> eval_pts{};
  for (auto& p : stitch_pts) {
    eval_pts.push_back(EvaluatedPoint{.path_point = p,
                                      .lateral_bound = 0.2,
                                      .longitudinal_bound = 0.2,
                                      .enforced = false});
  }
  FemPosSmoother smoother{};
  if (!smoother.Smooth(eval_pts, 0.2, 0.2)) {
    LOG_ERROR("FemPos failed to smooth!");
    return;
  }
  cur_ref = smoother.generate_points();
  if (stitch_pts.size() != cur_ref.size()) {
    LOG_ERROR("FemPosSmoother output size {} != input size {}",
              stitch_pts.size(), cur_ref.size());
    return;
  }
  return;
}

void MergeNearbyOverlaps(
    const double gap, std::vector<ReferenceLine::TrafficOverlap>* const laps) {
  if (laps->empty()) return;

  const auto& overlaps = *laps;
  std::vector<ReferenceLine::TrafficOverlap> ans{laps->front()};
  for (size_t i = 1, len = overlaps.size(); i < len; ++i) {
    if (auto& o = overlaps[i]; o.object_id == ans.back().object_id &&
                               o.start_s - ans.back().end_s < gap) {
      ans.back().end_s = o.end_s;
    } else {
      ans.push_back(o);
    }
  }

  std::swap(*laps, ans);
}

void UpdateOverlapStartS(std::vector<ReferenceLine::TrafficOverlap>& laps,
                         const std::vector<ReferencePoint>& ref_points,
                         const RefLaneSegment& seg) {
  auto lane = seg.lane_ptr;
  for (auto& [id, s, e] : laps) {
    common::math::Vec2d pt;
    double z = 0.0;
    lane->GetSmoothPoint(s, pt, &z);
    LOG_INFO("original s:{}", s);
    auto idx = ref_line_util::BinarySearchIndex(ref_points, seg.accumulate_s);
    bool is_found = false;
    double min_dist = DBL_MAX;
    for (int i = idx; i < ref_points.size(); ++i) {
      double dist =
          std::hypot(ref_points[i].x() - pt.x(), ref_points[i].y() - pt.y());
      if (dist > 1.0) {
        if (is_found)
          break;
        else
          continue;
      }
      if (dist < min_dist) {
        min_dist = dist;
        s = ref_points[i].s() - seg.accumulate_s;
        is_found = true;
      }
    }
    LOG_INFO("new s:{}", s);
  }
  return;
}

std::array<std::vector<ReferenceLine::TrafficOverlap>, 10>
GetRefTrafficOverlaps(std::vector<RefLaneSegment>& ref_segs,
                      const std::vector<ReferencePoint>& ref_points,
                      const double start_s, const double end_s) {
  std::array<std::vector<ReferenceLine::TrafficOverlap>, 10> ans{};

  if (ref_segs.empty() || end_s < ref_segs[0].accumulate_s ||
      start_s > ref_segs.back().accumulate_s)
    return ans;

  auto bs_idx = [&ref_segs](auto t) {
    int l = 0, r = ref_segs.size() - 1;
    while (l < r) {
      int mid = l + (r - l + 1) / 2;
      if (ref_segs[mid].accumulate_s > t) {
        r = mid - 1;
      } else {
        l = mid;
      }
    }
    return l;
  };

  auto get_overlaps = [](auto& laps, auto base, auto ss, auto es, auto* ans) {
    for (auto& [id, s, e] : laps)
      if (!(base + e < ss || base + s > es))
        ans->push_back({id, base + s, base + e});
  };

  auto idx = bs_idx(start_s);
  while (idx < ref_segs.size() && ref_segs[idx].accumulate_s < end_s) {
    auto& seg = ref_segs[idx++];
    double base_s = seg.accumulate_s - (idx > 1 ? 0 : seg.start_s);
    std::vector<ReferenceLine::TrafficOverlap> junction_vec{};
    get_overlaps(seg.junctions, base_s, start_s, end_s, &junction_vec);
    std::sort(junction_vec.begin(), junction_vec.end(),
              [](auto& a, auto& b) { return a.end_s < b.end_s; });
    double search_end_s = junction_vec.empty()
                              ? end_s
                              : std::max(end_s, junction_vec.back().end_s);
    LOG_INFO("idx:{}, base s:{}", idx - 1, base_s);
    for (auto& [id, s, e] : seg.signals)
      LOG_INFO("id:{}, s:{}, start s:{}", id, s, s + base_s);
    UpdateOverlapStartS(seg.signals, ref_points, seg);
    for (auto& [id, s, e] : seg.signals)
      LOG_INFO("id:{}, s:{}, start s:{}", id, s, s + base_s);
    get_overlaps(seg.crosswalks, base_s, start_s, search_end_s, &ans[0]);
    get_overlaps(seg.signals, base_s, start_s, search_end_s, &ans[1]);
    get_overlaps(seg.yield_signs, base_s, start_s, search_end_s, &ans[2]);
    get_overlaps(seg.stop_signs, base_s, start_s, search_end_s, &ans[3]);
    get_overlaps(seg.junctions, base_s, start_s, search_end_s, &ans[4]);
    get_overlaps(seg.speed_bumps, base_s, start_s, search_end_s, &ans[5]);
    get_overlaps(seg.clear_areas, base_s, start_s, search_end_s, &ans[6]);
    get_overlaps(seg.geo_fences, base_s, start_s, search_end_s, &ans[7]);
    get_overlaps(seg.barrier_gates, base_s, start_s, search_end_s, &ans[8]);
    get_overlaps(seg.parking_spaces, base_s, start_s, search_end_s, &ans[9]);
  }

  for (auto [id, s, e] : ans[1]) LOG_INFO("signal {}, {}, {}", id, s, e);

  /// Merge overlaps
  const double gap = 1.;
  for (int i = 0; i < ans.size(); ++i) {
    if (i != 1) MergeNearbyOverlaps(gap, &ans[i]);  // skip signals
  }

  for (auto [id, s, e] : ans[1]) LOG_DEBUG("signal {}, {}, {}", id, s, e);

  return ans;
}

ReferenceLinePtr BuildReferenceLine(
    std::vector<RefLaneSegment>& ref_segs,
    const std::vector<ReferencePoint>& ref_points, const Vec2d& init_pos,
    const double init_heading, const double init_s,
    const std::size_t routing_seq_num) {
  if (ref_segs.empty() || ref_points.empty()) {
    LOG_ERROR("No enough ref segs {} or points {}", ref_segs.size(),
              ref_points.size());
    return ReferenceLinePtr{};
  }

  const auto curise_generator_config =
      config::PlanningConfig::Instance()
          ->planning_research_config()
          .reference_line_config.curise_generator_config;
  const double start_s =
      std::max(init_s + curise_generator_config.back_extend_length,
               ref_points.front().s());
  const double end_s =
      std::min(init_s + curise_generator_config.front_extend_length,
               ref_points.back().s());

  auto begin = ref_points.begin() +
               ref_line_util::BinarySearchIndex(ref_points, start_s);
  if (begin + 1 == ref_points.end()) return ReferenceLinePtr{};
  auto end = ref_points.begin() +
             ref_line_util::BinarySearchIndex(ref_points, end_s) + 1;
  LOG_ERROR("enter createfrom");
  ReferenceLinePtr ret = std::make_shared<ReferenceLine>();
  ret->CreateFrom(std::vector<ReferencePoint>(begin, end),
                  GetRefTrafficOverlaps(ref_segs, ref_points, start_s, end_s),
                  init_pos, init_s, routing_seq_num);
  return ret;
}

int Generator::GetLaneChangeDirection(uint64_t prev, uint64_t next,
                                      TurnType& turn) {
  const auto lane_topo = ctx_->lane_topo;
  const auto hdmap = cyberverse::HDMap::Instance();
  auto lane = hdmap->GetLaneById(prev);
  auto road = hdmap->GetRoadById(lane->RoadId());
  auto lane_ids = road->Sections()[0]->LaneIds();
  int change_count = -1;
  int connect_idx = 0, prev_idx = lane->LaneNo();
  for (uint32_t i = 0; i < lane_ids.size(); ++i) {
    if (lane_topo->IsConnectLane(lane_ids[i], next)) {
      auto connect_lane = hdmap->GetLaneById(lane_ids[i]);
      bool need_skip = false;
      for (int j = 0; j < connect_lane->LaneMultipleType().size(); ++j) {
        LOG_DEBUG("lane id:{}, X type:{}",
                  hdmap->GetIdHashString(connect_lane->Id()),
                  connect_lane->LaneMultipleType()[j]);
        if (connect_lane->LaneMultipleType()[j] ==
                static_cast<uint32_t>(
                    global::hdmap::Lane::NONMOTORWAY_TO_MOTORWAY) ||
            connect_lane->LaneMultipleType()[j] ==
                static_cast<uint32_t>(
                    global::hdmap::Lane::MOTORWAY_TO_NONMOTORWAY)) {
          need_skip = true;
          break;
        }
      }
      if (need_skip) continue;
      connect_idx = connect_lane->LaneNo();
      break;
    }
  }
  if (connect_idx != 0) {
    turn = connect_idx > prev_idx ? global::hdmap::Lane::LEFT_TURN
                                  : global::hdmap::Lane::RIGHT_TURN;
  }
  return change_count;
}

// }  // namespace

bool Generator::BindLaneSeq(const LaneSeqType& route_lanes) {
  const double extend_len = FLAGS_planning_reference_line_extend_length;
  double extend_begin = extend_len, extend_end = extend_len;
  auto data_center = DataCenter::Instance();

  if (route_lanes.empty()) return false;
  const auto road_topo = ctx_->road_topo;
  const auto lane_topo = ctx_->lane_topo;
  /// Get lane segments
  ref_lane_segments_.clear();
  double len = 0.0;
  detect_lane_change_ = false;
  lane_change_turn_ = global::hdmap::Lane::NO_TURN;
  int start_idx = ctx_->is_lane_change_ref ? 0 : ctx_->current_lane_idx;
  for (int i = start_idx; i < route_lanes.size(); ++i) {
    len += route_lanes[i]->TotalLength();
    if (!ref_lane_segments_.empty() &&
        !lane_topo->IsConnectLane(ref_lane_segments_.back().id,
                                  route_lanes[i]->Id())) {
      if (len > curr_init_s_) {
        LOG_INFO("detect lane change");
        extend_end = 200.0;
        detect_lane_change_ = true;
        GetLaneChangeDirection(ref_lane_segments_.back().id,
                               route_lanes[i]->Id(), lane_change_turn_);
        break;
      } else {
        ref_lane_segments_.clear();
      }
    }
    ref_lane_segments_.push_back({route_lanes[i]->Id(), 0.0,
                                  route_lanes[i]->TotalLength(),
                                  route_lanes[i]});
  }
  LOG_INFO("route size:{}, cur lane size:{}", route_lanes.size(),
           ref_lane_segments_.size());

  /// Merge broken segments
  MergeBrokenLaneSegments(&ref_lane_segments_);

  // Extend front and tail
  tail_segs_.clear();
  ExtendRefLaneSegmentFrontAndTail(extend_begin, extend_end,
                                   &ref_lane_segments_, &tail_segs_);

  bool is_lc = ref_lane_segments_[0].is_to_lane_change;
  for (auto& seg : ref_lane_segments_)
    seg.is_to_lane_change = is_lc |= seg.is_to_lane_change;

  // Build lane segment info
  for (auto& seg : ref_lane_segments_) seg = BuildRefLaneSegment(seg);
  for (const auto& seg : ref_lane_segments_)
    LOG_DEBUG("build seg route lane addr:{}", (void*)seg.lane_ptr.get());
  for (size_t i = 1; i < ref_lane_segments_.size(); ++i) {
    ref_lane_segments_[i].accumulate_s =
        ref_lane_segments_[i - 1].accumulate_s +
        ref_lane_segments_[i - 1].end_s - ref_lane_segments_[i - 1].start_s;
  }
  LogLaneSegmentInfo(ref_lane_segments_);
  return true;
}
bool Generator::InitCurrentS(ReferenceLinePtr* const ref_line,
                             const TrajectoryPoint& init_point,
                             const LaneSeqType& lane_seq) {
  auto context = ctx_;
  if (*ref_line == nullptr || (*ref_line)->ref_points().empty() ||
      context->request_msg.is_updated) {
    auto hdmap = context->hdmap;
    int idx = context->is_lane_change_ref ? 0 : context->current_lane_idx;
    auto lane = hdmap->GetLaneById(lane_seq[idx]->Id());
    double l;
    lane->GetProjection({init_point.x(), init_point.y()}, &curr_init_s_, &l);
    LOG_INFO("Current s {}", curr_init_s_);
    return true;
  }
  if (context->first_finish_lane_change) {
    SLPoint curr_sl_pt{};
    if (!(*ref_line)->GetPointInFrenetFrame({init_point.x(), init_point.y()},
                                            &curr_sl_pt)) {
      LOG_ERROR("InitCurrentS fail");
      return false;
    }
    curr_init_s_ = curr_sl_pt.s();
    LOG_INFO("Current s {}", curr_init_s_);
    return true;
  }
  return true;
}

void Generator::GetLaneChangeEndPosition(
    const std::vector<RefLaneSegment>& tails, const TurnType& turn,
    const ReferenceLinePtr ref_line) {
  if (tails.empty()) {
    lane_change_end_point_.set_x(
        ref_lane_segments_.back().lane_ptr->Points().back().x());
    lane_change_end_point_.set_y(
        ref_lane_segments_.back().lane_ptr->Points().back().y());
    lane_change_end_point_.set_z(0.0);
    return;
  }
  lane_change_end_point_.set_x(tails.back().lane_ptr->Points().back().x());
  lane_change_end_point_.set_y(tails.back().lane_ptr->Points().back().y());
  lane_change_end_point_.set_z(0.0);
  auto hdmap = cyberverse::HDMap::Instance();
  auto data_center = DataCenter::Instance();
  for (auto seg : tails) {
    LOG_INFO("tail id:{}", hdmap->GetIdHashString(seg.id));
    if (ctx_->lane_set.count(seg.id) != 0) {
      LOG_INFO("tail seg id:{} is in lane set, skip",
               hdmap->GetIdHashString(seg.id));
      continue;
    }
    // ban lane change before solid line
    auto lane = seg.lane_ptr;
    auto lane_boundary_type = turn == global::hdmap::Lane::LEFT_TURN
                                  ? lane->left_divider()[0].type
                                  : lane->right_divider()[0].type;
    if (lane_boundary_type != global::hdmap::LaneBoundaryType::DOTTED_WHITE &&
        lane_boundary_type != global::hdmap::LaneBoundaryType::DOTTED_YELLOW &&
        lane_boundary_type != global::hdmap::LaneBoundaryType::UNKNOWN) {
      lane_change_end_point_.set_x(lane->Points().front().x());
      lane_change_end_point_.set_y(lane->Points().front().y());
      lane_change_end_point_.set_z(0.0);
      LOG_INFO(
          "SOLID_WHITE lane boundary, find lane change end pt x,y,z:{}, {}, {}",
          lane_change_end_point_.x(), lane_change_end_point_.y(),
          lane_change_end_point_.z());
      break;
    }
    // ban lane change before signal or junction turn
    auto road = hdmap->GetRoadById(lane->RoadId());
    auto junction = hdmap->GetJunctionById(road->JunctionId());
    bool is_in_junction_turn =
        junction != nullptr &&
        lane->TurnType() != static_cast<uint32_t>(global::hdmap::Lane::NO_TURN);
    bool is_in_signal = !lane->Signals().empty();
    if (is_in_junction_turn || is_in_signal) {
      lane_change_end_point_.set_x(lane->Points().front().x());
      lane_change_end_point_.set_y(lane->Points().front().y());
      lane_change_end_point_.set_z(0.0);
      LOG_INFO(
          "signal or junction turn, find lane change end pt x,y,z:{}, {}, "
          "{}, is_in_junction_turn:{}, is_in_signal:{}, id:{}",
          lane_change_end_point_.x(), lane_change_end_point_.y(),
          lane_change_end_point_.z(), is_in_junction_turn, is_in_signal,
          hdmap->GetIdHashString(lane->Id()));
      break;
    }
    // ban lane change before current and target lanes are not adjacent
    auto left_neighbour_id = cyberverse::LaneTopo::Instance()->GetNeighbour(
        lane->Id(), global::hdmap::Lane::LEFT_TURN);
    auto right_neighbour_id = cyberverse::LaneTopo::Instance()->GetNeighbour(
        lane->Id(), global::hdmap::Lane::RIGHT_TURN);
    const auto& lane_set = ctx_->lane_set;
    if (lane_set.count(left_neighbour_id) == 0 &&
        lane_set.count(right_neighbour_id) == 0) {
      lane_change_end_point_.set_x(lane->Points().front().x());
      lane_change_end_point_.set_y(lane->Points().front().y());
      lane_change_end_point_.set_z(0.0);
      LOG_INFO(
          "target lane is not adjacent to current lane, find lane "
          "change end pt x,y,z:{}, {}, {}",
          lane_change_end_point_.x(), lane_change_end_point_.y(),
          lane_change_end_point_.z());
      break;
    }
    // ban lane change when road bound is too narrow
    // ReferencePoint ref_pt{};
    // if (ref_line->GetNearestRefPoint(
    //         {lane->Points().front().x(), lane->Points().front().y()},
    //         &ref_pt)) {
    //   double lane_bound = turn == global::hdmap::Lane::LEFT_TURN
    //                           ? ref_pt.left_lane_bound()
    //                           : ref_pt.right_lane_bound();
    //   double road_bound = turn == global::hdmap::Lane::LEFT_TURN
    //                           ? ref_pt.left_road_bound()
    //                           : ref_pt.right_road_bound();
    //   if (road_bound - lane_bound < 0.1) {
    //     lane_change_end_point_.set_x(lane->Points().front().x());
    //     lane_change_end_point_.set_y(lane->Points().front().y());
    //     lane_change_end_point_.set_z(0.0);
    //     LOG_INFO(
    //         "road bound is too narrow, find lane change end pt x,y,z:{}, {},
    //         "
    //         "{}",
    //         lane_change_end_point_.x(), lane_change_end_point_.y(),
    //         lane_change_end_point_.z());
    //     break;
    //   }
    // }
  }
}

bool Generator::Generate(const LaneSeqType& lane_seq,
                         const TrajectoryPoint& init_point,
                         ReferenceLinePtr* const ref_line) {
  auto start_time = cyber::Time::Now().ToSecond();
  auto end_time = cyber::Time::Now().ToSecond();
  bool empty_ref_line =
      (*ref_line == nullptr || (*ref_line)->ref_points().empty() ||
       ctx_->request_msg.is_updated);
  auto& navigation_swap_context =
      DataCenter::Instance()->GetNavigationSwapContext();
  if (navigation_swap_context.parking_space_ptr.Get() &&
      navigation_swap_context.parking_space_ptr.Get()->OriginPark() &&
      navigation_swap_context.parking_space_ptr.Get()->OriginPark()->Type() ==
          global::hdmap::ParkingSpace_ParkingType_VERTICAL) {
    ;
  } else {
    if (!GetCurrentLane(init_point, lane_seq, empty_ref_line)) return false;
  }

  auto data_center = DataCenter::Instance();
  const double extend_len = FLAGS_planning_reference_line_extend_length;
  const auto front_extend_length =
      config::PlanningConfig::Instance()
          ->planning_research_config()
          .reference_line_config.curise_generator_config.front_extend_length;
  auto dis_from_last_init = empty_ref_line ? 0.0 : curr_init_s_ - last_init_s_;
  bool need_regenerate = ctx_->first_finish_lane_change || empty_ref_line ||
                         dis_from_last_init >= front_extend_length / 2;

  if (!InitCurrentS(ref_line, init_point, lane_seq)) return false;

  if (need_regenerate) {
    LOG_INFO("Generate ref line! dis_from_last_init: {}", dis_from_last_init);
    auto& plan_config = config::PlanningConfig::Instance()->plan_config();

    auto reference_line_speed_limits =
        data_center->mutable_reference_line_speed_limits();
    auto junctions_on_reference_line_speed_limits =
        data_center->mutable_junctions_on_reference_line_speed_limits();
    reference_line_speed_limits->clear();
    junctions_on_reference_line_speed_limits->clear();
    bool need_rest_lane_change_end_pt{false};

    // build ref lane segments
    if (empty_ref_line) {
      BindLaneSeq(lane_seq);
      if (ref_lane_segments_.empty()) return false;
      GereateSpeedLimitsFromRef(ref_lane_segments_,
                                *reference_line_speed_limits);
      GereateJunctionsSpeedLimitsFromRef(
          ref_lane_segments_,
          common::config::CommonConfig::Instance()
              ->drive_strategy_config()
              .non_motorway.dynamic_limit_speed,
          *junctions_on_reference_line_speed_limits);
      need_rest_lane_change_end_pt = true;
      LOG_INFO("ref_lane_segments_ size: {}", ref_lane_segments_.size());
    }

    // build ref line points
    CalcRefLinePoints(init_point);
    if (ref_line_points_.empty()) return false;

    LOG_INFO("Update current s to {}", curr_init_s_);
    last_init_s_ = curr_init_s_;

    // Create reference lines
    LOG_INFO("Update reference lines");
    auto curr_ref_line = BuildReferenceLine(
        ref_lane_segments_, ref_line_points_, {init_point.x(), init_point.y()},
        init_point.theta(), curr_init_s_, cyber::Time::Now().ToNanosecond());
    if (!curr_ref_line) return false;
    *ref_line = curr_ref_line;
    LOG_INFO("add curr_ref_line: {}", curr_ref_line->ref_points().size());

    // calculate lane change end point
    if (need_rest_lane_change_end_pt) {
      if (detect_lane_change_) {
        GetLaneChangeEndPosition(tail_segs_, lane_change_turn_, *ref_line);
      } else {
        lane_change_end_point_.set_x(0.0);
        lane_change_end_point_.set_y(0.0);
        lane_change_end_point_.set_z(0.0);
      }
    }

    for (auto& pt : (*ref_line)->ref_points())
      LOG_INFO(
          "s, x, y:{:.4f}, {:.4f}, {:.4f}, lane id:{}, left lane bound:{}, "
          "left road bound:{}, left bound:{}, right lane bound:{}, right road "
          "bound:{}, right bound:{}",
          pt.s(), pt.x(), pt.y(),
          ctx_->hdmap->GetIdHashString(pt.hd_map_lane_id()),
          pt.left_lane_bound(), pt.left_road_bound(), pt.left_bound(),
          pt.right_lane_bound(), pt.right_road_bound(), pt.right_bound());
  }
  end_time = cyber::Time::Now().ToSecond();
  LOG_INFO("Generator first generate use time: {:.4f}", end_time - start_time);

  /// Update init_s
  LOG_INFO("Update ego pos ({}, {})", init_point.x(), init_point.y());
  SLPoint curr_sl_pt{};
  if (!(*ref_line)->GetPointInFrenetFrameWithLastS(
          {init_point.x(), init_point.y()}, curr_init_s_, &curr_sl_pt)) {
    return false;
  }
  curr_init_s_ = curr_sl_pt.s();
  LOG_INFO("Current s {}", curr_init_s_);

  // update final
  auto dest_pt = data_center->routing_destination();
  if (!(*ref_line)->ref_points().empty() &&
      (*ref_line)->GetNearestRefPoint({dest_pt[0], dest_pt[1]},
                                      &destination_point_)) {
    is_dest_on_ref_line_ = true;
    if (std::hypot(destination_point_.x() - dest_pt[0],
                   destination_point_.y() - dest_pt[1]) > 3.5) {
      auto hdmap = ctx_->hdmap;
      auto ref_dest_lane =
          hdmap->GetLaneById(destination_point_.hd_map_lane_id());
      auto routing_end_lane = lane_seq.back();
      if (ref_dest_lane == nullptr || routing_end_lane == nullptr ||
          ref_dest_lane->Id() != routing_end_lane->Id()) {
        LOG_INFO(
            "detination is out of ref, ref end pt x,y,s: {:.4f}, {:.4f}, "
            "{:.4f}",
            (*ref_line)->ref_points().back().x(),
            (*ref_line)->ref_points().back().y(),
            (*ref_line)->ref_points().back().s());
        destination_point_ = (*ref_line)->ref_points().back();
        is_dest_on_ref_line_ = false;
      }
    }
    LOG_INFO("find ref destination x,y,s: {:.4f}, {:.4f}, {:.4f}",
             destination_point_.x(), destination_point_.y(),
             destination_point_.s());
  } else {
    LOG_ERROR("update destination failed.");
    is_dest_on_ref_line_ = false;
    return false;
  }
  return true;
}

const ReferencePoint& Generator::GetDestinationPoint() const {
  auto end_pt = ref_line_points_.back();
  auto dest_pt = DataCenter::Instance()->routing_destination();

  LOG_INFO("find ref destination x,y,s: {:.4f}, {:.4f}, {:.4f}",
           destination_point_.x(), destination_point_.y(),
           destination_point_.s());
  LOG_INFO("ref end x,y,s: {:.4f}, {:.4f}, {:.4f}", end_pt.x(), end_pt.y(),
           end_pt.s());
  LOG_INFO("routing_destination x,y: {:.4f}, {:.4f}", dest_pt[0], dest_pt[1]);

  return destination_point_;
}

bool Generator::GetCurrentLane(
    const TrajectoryPoint& ego_pose,
    const std::vector<cyberverse::LaneInfoConstPtr>& route_lanes,
    bool empty_ref_line) {
  auto hdmap = ctx_->hdmap;
  const auto& lane_set = ctx_->lane_set;
  std::vector<LaneInfoConstPtr> lanes;
  LOG_INFO("ego pose ({:.4f}, {:.4f})", ego_pose.x(), ego_pose.y());
  if (ctx_->is_lane_change_ref || !empty_ref_line) return true;

  hdmap->GetLanes({ego_pose.x(), ego_pose.y()}, 3.0, &lanes);
  if (lanes.empty()) {
    LOG_INFO("empty lane, hdmap_.GetLanes() fails");
    return false;
  }
  curr_lane_id_ = lanes[0]->Id();
  LOG_INFO("get current lane id:{}", hdmap->GetIdHashString(curr_lane_id_));
  for (auto lane : lanes)
    LOG_INFO("curr lanes:{}, dist:{}", hdmap->GetIdHashString(lane->Id()),
             lane->DistanceTo({ego_pose.x(), ego_pose.y()}));
  LOG_INFO("lane_set size:{}", lane_set.size());
  double min_dist = lanes[0]->DistanceTo({ego_pose.x(), ego_pose.y()});
  if (empty_ref_line) {
    for (auto lane : lanes) {
      LOG_DEBUG("all lane ids:{}, dist:{}", lane->Id(),
                lane->DistanceTo({ego_pose.x(), ego_pose.y()}));
      if (lane_set.find(lane->Id()) != lane_set.end()) {
        LOG_INFO("current lane == route lane");
        curr_lane_id_ = lane->Id();
        return true;
      }
    }
    LOG_ERROR("current lane != first route lane");
    return false;
  }
  return true;
}

double Generator::GetDistanceToDestination() const {
  double destination_s = destination_point_.s();
  if (!is_dest_on_ref_line_) {
    destination_s = std::min(destination_s,
                             ref_line_points_.back().s() -
                                 FLAGS_planning_reference_line_extend_length);
  }
  LOG_INFO(
      "dist2end:{}, is_dest_on_ref_line_:{}, destination_point_ s:{}, ref "
      "back s:{}, destination_s:{}",
      destination_s - curr_init_s_, is_dest_on_ref_line_,
      destination_point_.s(), ref_line_points_.back().s(), destination_s);
  return destination_s - curr_init_s_;
}

double Generator::GetDistanceToRefEnd(const ReferenceLinePtr refline) const {
  if (refline == nullptr || refline->ref_points().empty()) return 0.0;
  LOG_INFO("distance to ref end:{}",
           refline->ref_points().back().s() - curr_init_s_);
  return refline->ref_points().back().s() - curr_init_s_;
}

void Generator::GeneratePtsFromSeg(size_t& seg_start, size_t& seg_end,
                                   double start_s,
                                   std::vector<ReferencePoint>& pt_ret) {
  double s = seg_start == 0 ? 0.0 : kPointGap;
  double total_s = start_s + s;
  for (size_t i = seg_start; i < seg_end; ++i) {
    ReferencePoint p{};
    auto& seg = ref_lane_segments_[i];
    int pt_cnt = 0;
    while (s < seg.end_s &&
           PlanningMap::Instance()->GetLanePointWithDistance(seg.id, s, p)) {
      p.set_s(total_s);
      p.set_seg_idx(i);
      if (pt_ret.empty() || !IsEqual(p, pt_ret.back())) {
        ++pt_cnt;
        pt_ret.push_back(p);
        // LOG_INFO("i:{}, s:{:.4f}, totals:{:.4f}", i, s, total_s);
      }
      s += kPointGap;
      total_s += kPointGap;
      // TODO: Set infos for smoother
      p.set_multiple_type(seg.lane_multiple_type);
    }
    s -= seg.end_s;
    if (i == 0) {
      pt_cnt = std::max(1, pt_cnt);
      each_seg_pt_num_.push_back(pt_cnt - 1);
    } else {
      each_seg_pt_num_.push_back(pt_cnt + each_seg_pt_num_[i - 1]);
    }
  }
}

void Smooth(std::vector<ReferencePoint>& pt_ret) {
  auto conf = config::PlanningConfig::Instance()
                  ->planning_research_config()
                  .reference_line_smooth_config;
  std::vector<EvaluatedPoint> eval_pts{};
  for (auto& p : pt_ret) {
    double lateral_bound = conf.lateral_boundary_bound;
    double longitudinal_bound = conf.longitudinal_boundary_bound;
    if (p.lane_type_is_indoor_lane() && conf.indoor.enable_adjust_bound) {
      lateral_bound = conf.indoor.lateral_boundary_bound;
      longitudinal_bound = conf.indoor.longitudinal_boundary_bound;
    }
    if (p.lane_type_is_pure_city_driving() &&
        conf.motorway.enable_adjust_bound) {
      lateral_bound = conf.motorway.lateral_boundary_bound;
      longitudinal_bound = conf.motorway.longitudinal_boundary_bound;
    }
    eval_pts.push_back(EvaluatedPoint{.path_point = p,
                                      .lateral_bound = lateral_bound,
                                      .longitudinal_bound = longitudinal_bound,
                                      .enforced = false});
  }
  FemPosSmoother smoother{};
  if (!smoother.Smooth(eval_pts, 0.2, 0.2)) {
    LOG_ERROR("FemPos failed to smooth!");
    return;
  }
  auto& smooth_res = smoother.generate_points();
  if (pt_ret.size() != smooth_res.size()) {
    LOG_ERROR("FemPosSmoother output size {} != input size {}", pt_ret.size(),
              smooth_res.size());
    return;
  }
  pt_ret.assign(smooth_res.begin(), smooth_res.end());
}

void Generator::UpdateSegInfo(const TrajectoryPoint& ego_pose,
                              std::size_t& start_idx, std::size_t& end_idx) {
  LOG_INFO("curr_pt_ s,x,y:{:.4f}, {:.4f}, {:.4f}",
           ref_line_points_[curr_pt_idx_].s(),
           ref_line_points_[curr_pt_idx_].x(),
           ref_line_points_[curr_pt_idx_].y());
  auto last_pt_idx = curr_pt_idx_;
  while (curr_pt_idx_ != ref_line_points_.size()) {
    curr_pt_idx_++;
    if (curr_pt_idx_ != ref_line_points_.size() &&
        (std::hypot(ego_pose.x() - ref_line_points_[curr_pt_idx_].x(),
                    ego_pose.y() - ref_line_points_[curr_pt_idx_].y()) <
             std::hypot(ego_pose.x() - ref_line_points_[last_pt_idx].x(),
                        ego_pose.y() - ref_line_points_[last_pt_idx].y()) ||
         curr_init_s_ - ref_line_points_[curr_pt_idx_].s() > 5.0)) {
      last_pt_idx = curr_pt_idx_;
      dist_to_ref_end_ -= std::hypot(ref_line_points_[curr_pt_idx_].x() -
                                         ref_line_points_[last_pt_idx].x(),
                                     ref_line_points_[curr_pt_idx_].y() -
                                         ref_line_points_[last_pt_idx].y());
    } else {
      curr_pt_idx_ = last_pt_idx;
      curr_init_s_ = ref_line_points_[curr_pt_idx_].s();
      break;
    }
  }
  // }
  LOG_INFO("curr_pt_ s,x,y:{:.4f}, {:.4f}, {:.4f}",
           ref_line_points_[curr_pt_idx_].s(),
           ref_line_points_[curr_pt_idx_].x(),
           ref_line_points_[curr_pt_idx_].y());
  LOG_INFO("ego pose s,x,y:{:.4f}, {:.4f}, {:.4f}", curr_init_s_, ego_pose.x(),
           ego_pose.y());
  // extend lane seg
  // auto end_ref_pt = ref_line_points_.back();
  // for (std::size_t i = 0; i < ref_lane_segments_.size(); ++i) {
  //   if (ref_lane_segments_[i].lane_ptr->IsOnLane(
  //           {end_ref_pt.x(), end_ref_pt.y()}))
  //     start_idx = i + 1;
  // }
  double dist_to_end =
      ref_line_points_.back().s() - ref_line_points_[curr_pt_idx_].s();
  LOG_INFO("dist_to_end:{}", dist_to_end);
  start_idx = end_idx;
  if (dist_to_end > 200.0) return;
  LOG_INFO("start_idx:{}, end_idx:{}", start_idx, end_idx);
  for (auto i = start_idx; i < ref_lane_segments_.size(); ++i) {
    dist_to_end += ref_lane_segments_[i].end_s - ref_lane_segments_[i].start_s;
    LOG_INFO("i:{}, dtfe:{:.4f}", i, dist_to_end);
    ++end_idx;
    if (dist_to_end >= 200.0) break;
  }
}

void Generator::UpdateRefPts(size_t& seg_start, size_t& seg_end,
                             RefPtIterType& pt_start) {
  if (seg_start >= ref_lane_segments_.size() || ref_lane_segments_.empty() ||
      seg_start >= seg_end)
    return;
  LOG_INFO("seg_start:{}, seg_end:{}", seg_start, seg_end);
  double start_s = ref_line_points_.empty() ? 0.0 : ref_line_points_.back().s();
  std::vector<ReferencePoint> new_generated_pts;
  new_generated_pts.reserve(kMaxGenerateLength / kPointGap);
  new_generated_pts.insert(new_generated_pts.begin(),
                           ref_line_points_.begin() + curr_pt_idx_,
                           ref_line_points_.end());
  LOG_ERROR("new_generated_pts before:{}, start_s:{:.4f}",
            new_generated_pts.size(), start_s);
  GeneratePtsFromSeg(seg_start, seg_end, start_s, new_generated_pts);
  LOG_ERROR("new_generated_pts after:{}", new_generated_pts.size());
  Smooth(new_generated_pts);
  AddRefPointLaneInfo(ref_lane_segments_, new_generated_pts, curr_pt_idx_,
                      ref_line_points_);
  // update accumulated_s after smooth
  auto hdmap = ctx_->hdmap;
  for (size_t i = seg_start; i < seg_end; ++i) {
    if (i == 0) continue;
    if (each_seg_pt_num_[i - 1] < ref_line_points_.size()) {
      LOG_INFO("i:{}, acc s before:{}, after:{}, num:{}", i,
               ref_lane_segments_[i].accumulate_s,
               ref_line_points_[each_seg_pt_num_[i - 1]].s(),
               each_seg_pt_num_[i - 1]);
      if (each_seg_pt_num_[i - 1] != 0)
        LOG_INFO(
            "cur x:{}, y:{}, id:{}, next:{}",
            ref_line_points_[each_seg_pt_num_[i - 1] - 1].x(),
            ref_line_points_[each_seg_pt_num_[i - 1] - 1].y(),
            hdmap->GetIdHashString(
                ref_line_points_[each_seg_pt_num_[i - 1] - 1].hd_map_lane_id()),
            hdmap->GetIdHashString(
                ref_line_points_[each_seg_pt_num_[i - 1]].hd_map_lane_id()),
            hdmap->GetIdHashString(ref_line_points_[each_seg_pt_num_[i - 1] + 1]
                                       .hd_map_lane_id()));
      ref_lane_segments_[i].accumulate_s =
          ref_line_points_[each_seg_pt_num_[i - 1]].s();
    } else {
      ref_lane_segments_[i].accumulate_s =
          ref_line_points_[ref_line_points_.size() - 1].s() + kPointGap;
      if (each_seg_pt_num_[i - 1] > ref_line_points_.size()) {
        LOG_ERROR(
            "each_seg_pt_num_[{}]({}) is out of ref_line_points_.size({})",
            i - 1, each_seg_pt_num_[i - 1], ref_line_points_.size());
      }
    }
  }
}

void Generator::InitCurrRefPosition(const TrajectoryPoint& ego_pose) {
  dist_to_ref_end_ = 0.0;
  int increase_cnt = 0;
  bool is_first_nearest = true;
  double min_dist = std::numeric_limits<double>::infinity();
  LOG_ERROR("pt size:{}", ref_line_points_.size());
  int cnt = 0;
  for (auto iter = ref_line_points_.begin(); iter != ref_line_points_.end();
       ++iter) {
    if (curr_init_s_ - iter->s() > 5.0) {
      ++cnt;
      continue;
    }
    double dist =
        std::hypot(iter->x() - ego_pose.x(), iter->y() - ego_pose.y());
    if (iter != ref_line_points_.begin()) {
      auto last_iter = iter - 1;
      dist_to_ref_end_ +=
          std::hypot(iter->x() - last_iter->x(), iter->y() - last_iter->y());
    }
    if (dist < min_dist && is_first_nearest) {
      curr_pt_ = iter;
      curr_pt_idx_ = cnt;
      curr_init_s_ = iter->s();
      dist_to_ref_end_ = 0.0;

      min_dist = dist;
      increase_cnt = 0;
    } else if (!std::isinf(min_dist)) {
      increase_cnt++;
    }
    if (increase_cnt > 10) is_first_nearest = false;
    cnt++;
  }
  LOG_INFO("pt size:{}, idx:{}", ref_line_points_.size(), curr_pt_idx_);
  LOG_INFO(
      "ego pose:({:.4f}, {:.4f}), nearest refpt:({:.4f}, "
      "{:.4f}),dist_to_ref_end_:{:.4f}",
      ego_pose.x(), ego_pose.y(), ref_line_points_[curr_pt_idx_].x(),
      ref_line_points_[curr_pt_idx_].y(), dist_to_ref_end_);
}

void Generator::CalcRefLinePoints(const TrajectoryPoint& init_point) {
  std::vector<uint32_t> each_seg_pt_num;
  double kPointGap = 0.2;
  bool is_first_calc = true;
  if (ref_lane_segments_.empty()) return;
  LOG_INFO("Start build ref line points");

  const auto front_extend_length =
      config::PlanningConfig::Instance()
          ->planning_research_config()
          .reference_line_config.curise_generator_config.front_extend_length;

  LOG_INFO("curr_init_s_:{}", curr_init_s_);
  if (ref_line_points_.empty()) {
    double s = ref_lane_segments_[0].start_s;
    double total_s = s;
    for (auto& seg : ref_lane_segments_) {
      ++seg_end_;
      total_s += seg.end_s;
      if (total_s - curr_init_s_ >= front_extend_length) break;
    }
    curr_pt_ = ref_line_points_.begin();
    curr_pt_idx_ = 0;
    LOG_INFO("first calc! seg start:{}, seg end:{}", seg_start_, seg_end_);
  } else {
    is_first_calc = false;
    UpdateSegInfo(init_point, seg_start_, seg_end_);
    LOG_INFO("seg start:{}, seg end:{}", seg_start_, seg_end_);
  }

  UpdateRefPts(seg_start_, seg_end_, curr_pt_);

  if (is_first_calc) InitCurrRefPosition(init_point);

  // for (auto& p : ref_line_points_) LogRefPoint(p);
  return;
}

}  // namespace planning
}  // namespace neodrive
