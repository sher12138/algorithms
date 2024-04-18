#include "reference_line/reference_line_util.h"

namespace neodrive {
namespace planning {

namespace ref_line_util {

bool GetAdcBoundingBoxSL(ReferenceLinePtr ref_line, const Vec2d& adc_pose,
                         const double adc_heading,
                         std::vector<SLPoint>* const corners,
                         const double lateral_buffer, const double front_buffer,
                         const double end_buffer) {
  if (corners == nullptr) return false;

  SLPoint center;
  ReferencePoint closest_pt;
  if (!ref_line->GetPointInFrenetFrameWithHeading(adc_pose, adc_heading,
                                                  &center) ||
      !ref_line->GetNearestRefPointWithHeading(adc_pose, adc_heading,
                                               &closest_pt))
    return false;

  double delta_heading = normalize_angle(adc_heading - closest_pt.heading());
  auto box_2d = VehicleParam::Instance()->get_adc_bounding_box(
      Vec2d{center.s(), center.l()}, delta_heading, lateral_buffer,
      front_buffer, end_buffer);
  std::vector<Vec2d> all_corners;
  box_2d.get_all_corners(&all_corners);
  if (all_corners.size() < 4) return false;
  corners->clear();
  for (const auto& pt : all_corners) {
    corners->emplace_back(SLPoint{pt.x(), pt.y()});
  }
  return true;
}

bool GetAdcBoundingBoxBoundary(ReferenceLinePtr ref_line, const Vec2d& adc_pose,
                               const double adc_heading,
                               Boundary* const boundary,
                               const double lateral_buffer,
                               const double front_buffer,
                               const double end_buffer) {
  if (boundary == nullptr) return false;
  std::vector<SLPoint> corners;
  if (!GetAdcBoundingBoxSL(ref_line, adc_pose, adc_heading, &corners,
                           lateral_buffer, front_buffer, end_buffer) ||
      corners.size() < 3)
    return false;
  boundary->reset();
  for (const auto& sl : corners) {
    boundary->set_start_s(std::min(boundary->start_s(), sl.s()));
    boundary->set_end_s(std::max(boundary->end_s(), sl.s()));
    boundary->set_start_l(std::min(boundary->start_l(), sl.l()));
    boundary->set_end_l(std::max(boundary->end_l(), sl.l()));
  }
  return true;
}

bool IsOnRoad(ReferenceLinePtr ref_line, const SLPoint& sl_point) {
  const double s = sl_point.s(), l = sl_point.l();
  ReferencePoint ref_pt{};
  return ref_line->GetNearestRefPoint(s, &ref_pt) &&
         !(l < -ref_pt.right_bound() || l > ref_pt.left_bound());
}

size_t BinarySearchIndex(const std::vector<ReferencePoint>& pts,
                         const double tar) {
  if (pts.empty() || tar < pts[0].s()) return 0;
  if (tar > pts.back().s()) return pts.size() - 1;

  size_t m = pts.size(), l = 0, r = m - 1;
  while (l < r) {
    int mid = l + (r - l) / 2;
    if (pts[mid].s() < tar + 1e-5) {
      l = mid + 1;
    } else {
      r = mid;
    }
  }
  if (l && pts[l].s() - tar > tar - pts[l - 1].s()) --l;

  return l;
}

bool ComputePolygonBoundary(ReferenceLinePtr ref_line, const Polygon2d& polygon,
                            Boundary* polygon_boundary) {
  if (polygon_boundary == nullptr || polygon.points().size() <= 3) return false;

  Boundary default_boundary{};
  std::vector<SLPoint> polygon_sl_points{};
  const auto& points = polygon.points();
  for (const auto& pt : points) {
    polygon_sl_points.emplace_back();
    if (!ref_line->GetPointInFrenetFrame(pt, &polygon_sl_points.back())) {
      LOG_INFO("polygon cannot project on refer_line, skip the obs, obs");
      *polygon_boundary = default_boundary;
      return false;
    }
    polygon_boundary->set_start_s(
        std::min(polygon_sl_points.back().s(), polygon_boundary->start_s()));
    polygon_boundary->set_end_s(
        std::max(polygon_sl_points.back().s(), polygon_boundary->end_s()));
    polygon_boundary->set_start_l(
        std::min(polygon_sl_points.back().l(), polygon_boundary->start_l()));
    polygon_boundary->set_end_l(
        std::max(polygon_sl_points.back().l(), polygon_boundary->end_l()));
  }

  return true;
}

bool SIsInJunction(ReferenceLinePtr ref_line, const double curr_s) {
  auto& rpts = ref_line->ref_points();
  auto idx = BinarySearchIndex(rpts, curr_s);
  return idx < rpts.size() && rpts[idx].is_in_junction();
}

}  // namespace ref_line_util

}  // namespace planning
}  // namespace neodrive
