#include "reference_line/reference_line.h"

#include "common/math/util.h"
#include "planning_map/planning_map.h"
#include "reference_line/reference_line_kdtree.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/common/math/math_utils.h"
#include "src/planning/planning_map/planning_map.h"

namespace neodrive {
namespace planning {

namespace {

constexpr double kSearchTol = 15.0;
constexpr double kStartTol = 3.0;

double SqrDis(const ReferencePoint& p1, const Vec2d& p2) {
  return std::pow(p1.x() - p2.x(), 2) + std::pow(p1.y() - p2.y(), 2);
}

}  // namespace
void ReferenceLine::CreateFrom(
    const std::vector<ReferencePoint>& ref_points,
    const std::array<std::vector<ReferenceLine::TrafficOverlap>, 10>& overlaps,
    const Vec2d& anchor_xy, const double anchor_s,
    const std::size_t routing_seq_num) {
  LOG_INFO("createfrom addr:{}", (void*)this);
  if (ref_points.size() >= 2)
    BindRefPts(ref_points, overlaps, anchor_xy, anchor_s, routing_seq_num);
}

void ReferenceLine::operator=(const ReferenceLine& in_ref) {
  ref_points_ = in_ref.ref_points_;
  crosswalk_overlaps_ = in_ref.crosswalk_overlaps_;
  signal_overlaps_ = in_ref.signal_overlaps_;
  yield_sign_overlaps_ = in_ref.yield_sign_overlaps_;
  stop_sign_overlaps_ = in_ref.stop_sign_overlaps_;
  junction_overlaps_ = in_ref.junction_overlaps_;
  speed_bump_overlaps_ = in_ref.speed_bump_overlaps_;
  clearzone_overlaps_ = in_ref.clearzone_overlaps_;
  geo_fence_overlaps_ = in_ref.geo_fence_overlaps_;
  barrier_gate_overlaps_ = in_ref.barrier_gate_overlaps_;
  routing_sequence_num_ = in_ref.routing_sequence_num_;
  junctions_ = in_ref.junctions_;
  anchor_s_ = in_ref.anchor_s_;
  ref_kdtree_ = std::make_shared<ReferencelineKdTree>(ref_points_);
}

void ReferenceLine::BindRefPts(
    const std::vector<ReferencePoint>& ref_points,
    const std::array<std::vector<ReferenceLine::TrafficOverlap>, 10>& overlaps,
    const Vec2d& anchor_xy, const double anchor_s,
    const std::size_t routing_seq_num) {
  ref_points_ = ref_points;
  crosswalk_overlaps_ = overlaps[0];
  signal_overlaps_ = overlaps[1];
  yield_sign_overlaps_ = overlaps[2];
  stop_sign_overlaps_ = overlaps[3];
  junction_overlaps_ = overlaps[4];
  speed_bump_overlaps_ = overlaps[5];
  clearzone_overlaps_ = overlaps[6];
  geo_fence_overlaps_ = overlaps[7];
  barrier_gate_overlaps_ = overlaps[8];
  parking_space_overlaps_ = overlaps[9];
  routing_sequence_num_ = routing_seq_num;
  CHECK(ref_points.size() > 1);
  ref_kdtree_ = std::make_shared<ReferencelineKdTree>(ref_points_);
  auto ret = ref_kdtree_->get_nearest_object(anchor_xy);
  if (ret) anchor_s_ = ret->s();
  // for (auto& pt : ref_points)
  //   LOG_ERROR("x:{:.4f}, y:{:.4f}, s:{:.4f}", pt.x(), pt.y(), pt.s());
  /// Update junction
  for (auto& j : junction_overlaps_) {
    cyberverse::JunctionInfoConstPtr junc_ptr;
    PlanningMap::Instance()->GetJunctionById(j.object_id, junc_ptr);
    junctions_.emplace_back(junc_ptr, j);
    LOG_DEBUG("junction type is [{}] ", junc_ptr->Type());
  }
}

void ReferenceLine::RebuildKdtree() {
  ref_kdtree_ = std::make_shared<ReferencelineKdTree>(ref_points_);
}

bool ReferenceLine::GetNearestRefPoint(const Vec2d& xy_pt,
                                       ReferencePoint* const ref_pt) {
  // LOG_INFO("GetNearestRefPoint ref addr:{}, ref pt addr:{}", (void *)this,
  //          (void *)&ref_points_);
  if (!ref_pt) return false;
  auto ret = ref_kdtree_->get_nearest_object(xy_pt);
  if (ret) {
    // LOG_ERROR("x:{:.4f}, y:{:.4f}", xy_pt.x(), xy_pt.y());
    // LOG_ERROR("ret x:{:.4f}, y:{:.4f}, addr:{}, time:{}", ret->x(), ret->y(),
    // (void*)ret, routing_sequence_num_);
    // if (ret->x() <= 100.0 || ret->y() <= 100.0) {
    //   for (auto& each_pt : ref_points_)
    //     LOG_ERROR("{} {}", each_pt.x(), each_pt.y());
    //   LOG_ERROR("print finish");
    //   LOG_ERROR("cmp addr:{},{}", (void*)&ref_points_[0],
    //             (void*)ref_kdtree_->see_first_ptr());
    // }

    // LOG_ERROR("ret vec size:{}", ret->lane_multiple_type().size());
    *ref_pt = *ret;
    return true;
  } else {
    return false;
  }
}

size_t IndexRefPt(const std::vector<ReferencePoint>& ref,
                  const ReferencePoint& pt) {
  size_t idx = 0;
  for (; idx < ref.size(); ++idx) {
    if (pt.s() == ref[idx].s()) break;
  }
  return idx;
}

bool ReferenceLine::GetfrontNearestRefPoint(
    const Vec2d& xy_pt, uint64_t curLaneID,
    std::vector<std::pair<std::pair<uint64_t, uint64_t>, bool>>& successor,
    ReferencePoint* const ref_pt) const {
  if (!ref_pt) return false;
  double err{std::numeric_limits<double>::infinity()};
  auto nearest = ref_kdtree_->get_nearest_object(xy_pt);
  if (!nearest) return false;
  auto nearests =
      ref_kdtree_->get_objects({nearest->x(), nearest->y()}, kSearchTol);
  nearest = *std::min_element(
      nearests.begin(), nearests.end(), [this](auto a, auto b) {
        return std::abs(a->s() - anchor_s_) < std::abs(b->s() - anchor_s_);
      });
  *ref_pt = *nearest;
  bool lstop = ref_pt->hd_map_lane_id() != curLaneID;
  bool rstop = false;
  uint64_t nextLaneID = 0;
  for (auto& id : successor) {
    rstop = ref_pt->hd_map_lane_id() == id.first.first;
    if (rstop) {
      nextLaneID = id.first.first;
      break;
    }
  }
  if (lstop && (!rstop))
    return false;
  else if (lstop && rstop)
    return true;

  uint64_t midLaneID = curLaneID;
  auto nearest_idx = IndexRefPt(ref_points_, *nearest);
  int cnt = 0;
  while (!lstop && cnt < ref_points_.size()) {
    cnt++;
    if (nearest_idx++ == ref_points_.size() - 1) return false;
    *ref_pt = ref_points_[nearest_idx];
    midLaneID = ref_pt->hd_map_lane_id();
    if (midLaneID != curLaneID) {
      lstop = true;
      break;
    }
  }
  if (!lstop)
    return false;
  else {
    bool matchSuccessor = false;
    for (auto& id : successor) {
      if (midLaneID == id.first.first) {
        matchSuccessor = true;
        break;
      }
    }
    if (!matchSuccessor) return false;
  }

  *ref_pt = ref_points_[nearest_idx];
  return true;
}

bool ReferenceLine::GetNearestRefPointWithHeading(const Vec2d& xy_pt,
                                                  const double heading,
                                                  ReferencePoint* const ref_pt,
                                                  const double heading_tol) {
  if (!ref_pt) return false;
  auto ret = ref_kdtree_->get_objects(xy_pt, kSearchTol);
  if (ret.empty()) {
    LOG_ERROR("GetNearestRefPointWithHeading empty ret");
    return false;
  }

  std::sort(ret.begin(), ret.end(), [&](auto a, auto b) {
    return a->distance_sqr_to(xy_pt) < b->distance_sqr_to(xy_pt);
  });
  for (auto& each_pt : ret) {
    if (std::abs(normalize_angle(each_pt->heading() - heading)) < heading_tol) {
      *ref_pt = *each_pt;
      return true;
    }
  }

  return false;
}

bool ReferenceLine::GetNearestRefPoint(const double s, ReferencePoint* ref_pt) {
  if (!ref_pt) {
    return false;
  }
  auto start_s = ref_points_.front().s();
  auto end_s = ref_points_.back().s();
  if (s <= start_s) {
    *ref_pt = ref_points_.front();
  } else if (s >= end_s) {
    *ref_pt = ref_points_.back();
  } else {
    *ref_pt = ref_points_[ref_line_util::BinarySearchIndex(ref_points_, s)];
  }
  return true;
}

bool ReferenceLine::GetPointInFrenetFrame(const Vec2d& xy_pt,
                                          SLPoint* const sl_pt) {
  ReferencePoint ref_pt{};
  if (!GetNearestRefPoint(xy_pt, &ref_pt)) return false;
  auto si = ref_line_util::BinarySearchIndex(ref_points_, ref_pt.s());
  if (si == ref_points_.size() - 1) --si;
  Vec2d v1{ref_points_[si + 1].x() - ref_points_[si].x(),
           ref_points_[si + 1].y() - ref_points_[si].y()};
  const double len = v1.length();
  Vec2d v2{xy_pt.x() - ref_points_[si].x(), xy_pt.y() - ref_points_[si].y()};
  sl_pt->set_s(ref_points_[si].s() + v1.inner_prod(v2) / len);
  sl_pt->set_l(v1.cross_prod(v2) / len);

  return true;
}

bool ReferenceLine::GetPointInFrenetFrameWithLastS(const Vec2d& xy_pt,
                                                   const double last_s,
                                                   SLPoint* const sl_pt) {
  auto ret = ref_kdtree_->get_objects(xy_pt, kSearchTol);
  if (ret.empty()) {
    LOG_ERROR("GetNearestRefPointWithHeading empty ret");
    return false;
  }
  std::sort(ret.begin(), ret.end(), [&](auto a, auto b) {
    return a->distance_sqr_to(xy_pt) < b->distance_sqr_to(xy_pt);
  });
  ReferencePoint ref_pt{*ret[0]};
  for (auto& each_pt : ret) {
    if (std::abs(each_pt->s() - last_s) < kStartTol) {
      ref_pt = *each_pt;
      break;
    }
  }

  auto si = ref_line_util::BinarySearchIndex(ref_points_, ref_pt.s());
  if (si == ref_points_.size() - 1) --si;
  Vec2d v1{ref_points_[si + 1].x() - ref_points_[si].x(),
           ref_points_[si + 1].y() - ref_points_[si].y()};
  const double len = v1.length();
  Vec2d v2{xy_pt.x() - ref_points_[si].x(), xy_pt.y() - ref_points_[si].y()};
  sl_pt->set_s(ref_points_[si].s() + v1.inner_prod(v2) / len);
  sl_pt->set_l(v1.cross_prod(v2) / len);

  return true;
}

bool ReferenceLine::GetPointInFrenetFrameWithHeading(const Vec2d& xy_pt,
                                                     const double heading,
                                                     SLPoint* const sl_pt) {
  ReferencePoint ref_pt{};
  double heading_tol{M_PI};
  if (!GetNearestRefPointWithHeading(xy_pt, heading, &ref_pt, heading_tol)) {
    return false;
  }

  auto si = ref_line_util::BinarySearchIndex(ref_points_, ref_pt.s());
  if (si == ref_points_.size() - 1) --si;
  Vec2d v1{ref_points_[si + 1].x() - ref_points_[si].x(),
           ref_points_[si + 1].y() - ref_points_[si].y()};
  const double len = v1.length();
  Vec2d v2{xy_pt.x() - ref_points_[si].x(), xy_pt.y() - ref_points_[si].y()};
  sl_pt->set_s(ref_points_[si].s() + v1.inner_prod(v2) / len);
  sl_pt->set_l(v1.cross_prod(v2) / len);

  return true;
}

bool ReferenceLine::GetObsCornerInFrenetFrame(const Vec2d& xy_pt,
                                              const Vec2d& obs_center_pt,
                                              SLPoint* const sl_pt) {
  double diagonal_length = 2 * std::hypot(xy_pt.x() - obs_center_pt.x(),
                                          xy_pt.y() - obs_center_pt.y());
  ReferencePoint ref_pt{};
  if (!GetNearestRefPoint(xy_pt, &ref_pt)) return false;
  ReferencePoint obs_center_ref_pt{};
  if (!GetNearestRefPoint(obs_center_pt, &obs_center_ref_pt)) return false;
  if (std::abs(ref_pt.s() - obs_center_ref_pt.s()) > diagonal_length) {
    LOG_INFO("ref pt s:{}, obs center s:{}, diagonal:{}", ref_pt.s(),
             obs_center_ref_pt.s(), diagonal_length);
    LOG_INFO(
        "nearest ref pt of obs corner is too far away from obs center, "
        "search for new ref pt");
    double search_dist = std::hypot(ref_pt.x() - obs_center_pt.x(),
                                    ref_pt.y() - obs_center_pt.y()) +
                         diagonal_length;
    auto ret = ref_kdtree_->get_objects(xy_pt, search_dist);
    if (ret.empty()) {
      LOG_ERROR("GetObsCornerInFrenetFrame empty ret");
      return false;
    }
    std::sort(ret.begin(), ret.end(), [&](auto a, auto b) {
      return a->distance_sqr_to(xy_pt) < b->distance_sqr_to(xy_pt);
    });
    for (auto& each_pt : ret) {
      if (std::abs(each_pt->s() - obs_center_ref_pt.s()) <= diagonal_length) {
        ref_pt = *each_pt;
        LOG_INFO("find new ref pt s:{}, obs center s:{}", ref_pt.s(),
                 obs_center_ref_pt.s());
        break;
      }
    }
  }

  auto si = ref_line_util::BinarySearchIndex(ref_points_, ref_pt.s());
  if (si == ref_points_.size() - 1) --si;
  Vec2d v1{ref_points_[si + 1].x() - ref_points_[si].x(),
           ref_points_[si + 1].y() - ref_points_[si].y()};
  const double len = v1.length();
  Vec2d v2{xy_pt.x() - ref_points_[si].x(), xy_pt.y() - ref_points_[si].y()};
  sl_pt->set_s(ref_points_[si].s() + v1.inner_prod(v2) / len);
  sl_pt->set_l(v1.cross_prod(v2) / len);

  return true;
}

bool ReferenceLine::GetPolygonByAABox(const AABox2d& sl_box,
                                      Polygon2d* const polygon) {
  std::vector<Vec2d> xy_points;
  SLPoint sl_pts[] = {SLPoint(sl_box.min_x(), sl_box.min_y()),
                      SLPoint(sl_box.max_x(), sl_box.min_y()),
                      SLPoint(sl_box.min_x(), sl_box.max_y()),
                      SLPoint(sl_box.max_x(), sl_box.max_y())};

  for (const auto& sl_pt : sl_pts) {
    Vec2d tmp_pt{};
    GetPointInCartesianFrame(sl_pt, &tmp_pt);
    xy_points.emplace_back(tmp_pt);
    LOG_INFO("get xy point x:{}, y:{}", tmp_pt.x(), tmp_pt.y());
  }
  *polygon = Polygon2d(xy_points);
  return true;
}

bool ReferenceLine::GetPointInCartesianFrame(const SLPoint& sl_pt,
                                             Vec2d* const xy_pt) {
  ReferencePoint ref_pt{};
  if (!GetNearestRefPoint(sl_pt.s(), &ref_pt)) return false;

  auto si = ref_line_util::BinarySearchIndex(ref_points_, ref_pt.s());
  if (si == ref_points_.size() - 1) --si;
  Vec2d v1{ref_points_[si + 1].x() - ref_points_[si].x(),
           ref_points_[si + 1].y() - ref_points_[si].y()};
  v1.normalize();
  auto v2 = v1.rotate(M_PI_2);

  Vec2d ans{ref_points_[si].x(), ref_points_[si].y()};
  *xy_pt = ans + v1 * (sl_pt.s() - ref_points_[si].s()) + v2 * sl_pt.l();

  return true;
}

bool ReferenceLine::GetStartEndIndexBySLength(const double s, const double len,
                                              size_t* const start_index,
                                              size_t* const end_index) const {
  if (start_index == nullptr || end_index == nullptr) {
    return false;
  }
  *start_index = ref_line_util::BinarySearchIndex(ref_points_, s);
  *end_index = ref_line_util::BinarySearchIndex(ref_points_, s + len);
  return *end_index >= *start_index;
}

double ReferenceLine::GetLength() const {
  return ref_points_.back().s() - ref_points_.front().s();
}

void ReferenceLine::SetIndexLaneBound(const size_t idx, double left_lane_bound,
                                      double right_lane_bound) {
  if (idx >= ref_points_.size()) return;
  ref_points_[idx].set_left_lane_bound(left_lane_bound);
  ref_points_[idx].set_right_lane_bound(right_lane_bound);
}

void ReferenceLine::SetIndexRoadBound(const size_t idx, double left_road_bound,
                                      double right_road_bound) {
  if (idx >= ref_points_.size()) return;
  ref_points_[idx].set_left_road_bound(left_road_bound);
  ref_points_[idx].set_right_road_bound(right_road_bound);
}

void ReferenceLine::SetIndexBound(const size_t idx, double left_bound,
                                  double right_bound) {
  if (idx >= ref_points_.size()) return;
  ref_points_[idx].set_left_bound(left_bound);
  ref_points_[idx].set_right_bound(right_bound);
}

void ReferenceLine::SetIndexLaneBorrowFlag(const size_t idx, bool left_flag,
                                           bool right_flag) {
  if (idx >= ref_points_.size()) return;
  ref_points_[idx].set_left_lane_borrow(left_flag);
  ref_points_[idx].set_right_lane_borrow(right_flag);
}

}  // namespace planning
}  // namespace neodrive
