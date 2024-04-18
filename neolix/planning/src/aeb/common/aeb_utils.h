#pragma once
#include "common/aeb_type.h"
#include "math/polygon2d.h"
#include "math/segment2d.h"
namespace neodrive {
namespace aeb {
bool GetIntersection(const common::math::Polygon2d &polygon,
                     const common::math::Segment2d &seg,
                     common::math::Vec2d &intersect_pt) {
  common::math::Vec2d temp_intersect_pt;
  auto &segs = polygon.segments();
  for (uint32_t i = 0; i < segs.size(); ++i) {
    auto &target_seg = segs[i];
    if (seg.get_intersect(target_seg, &temp_intersect_pt)) {
      intersect_pt = temp_intersect_pt;
      return true;
    }
  }
  return false;
}
}  // namespace aeb
}  // namespace neodrive