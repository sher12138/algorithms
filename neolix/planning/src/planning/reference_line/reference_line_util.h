#pragma once

#include "reference_line/reference_line.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {

namespace ref_line_util {

bool GetAdcBoundingBoxSL(ReferenceLinePtr ref_line, const Vec2d& adc_pose,
                         const double adc_heading,
                         std::vector<SLPoint>* const corners,
                         const double lateral_buffer = 0.,
                         const double front_buffer = 0.,
                         const double end_buffer = 0.);

bool GetAdcBoundingBoxBoundary(ReferenceLinePtr ref_line, const Vec2d& adc_pose,
                               const double adc_heading,
                               Boundary* const boundary,
                               const double lateral_buffer = 0.,
                               const double front_buffer = 0.,
                               const double end_buffer = 0.);

bool IsOnRoad(ReferenceLinePtr ref_line, const SLPoint& sl_point);

size_t BinarySearchIndex(const std::vector<ReferencePoint>& pts,
                         const double tar);

bool ComputePolygonBoundary(ReferenceLinePtr ref_line, const Polygon2d& polygon,
                            Boundary* polygon_boundary);
bool SIsInJunction(ReferenceLinePtr ref_line, const double curr_s);

}  // namespace ref_line_util
}  // namespace planning
}  // namespace neodrive
