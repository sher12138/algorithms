#pragma once

#include "common/math/util.h"
#include "common_config/config/common_config.h"
#include "math/frame_conversion/sl_analytic_transformation.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/common/trajectory/publishable_trajectory.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/math/curve1d/spline.h"

namespace neodrive {
namespace planning {
namespace path_planner_common {

bool CalculateRoadObsBoundary(const std::size_t planner_type,
                              const double static_max_speed,
                              const InsidePlannerData &inside_data,
                              const DecisionData &decision_data,
                              const std::vector<FrenetFramePoint> &frenet_path,
                              PathObstacleContext &path_obstacle_context,
                              std::vector<PieceBoundary> &lateral_boundaries,
                              OutsidePlannerData *const outside_data);

bool CalcFinalTrajValidLength(const std::size_t planner_type,
                              const PathObstacleContext &path_obstacle_context,
                              const ReferenceLinePtr &reference_line,
                              const std::vector<FrenetFramePoint> &frenet_path,
                              double *valid_length);

bool CutOffFinalTrajValidLength(const DiscretizedPath &path,
                                double *valid_length);

bool UpdateHistoryValidLength(const double valid_length,
                              std::vector<double> &history_valid_len);

bool ExtractObsCorrespondingSRange(
    const std::vector<SLPoint> &curve_trajectory_points,
    const PathObstacleDecision &obs_decision, std::size_t &start_index,
    std::size_t &end_index);

bool OnlyProvideOriginReferenceLine(const ReferenceLinePtr &reference_line,
                                    const InsidePlannerData &inside_data,
                                    const FrenetFramePoint &frenet_init_point,
                                    PathData *const path_data);

void AdjustInitState(const ReferenceLinePtr &reference_line,
                     const InsidePlannerData &inside_data,
                     const OutsidePlannerData *outside_data, double &init_dl,
                     double &init_ddl);

void CalcAdcSampleDataForValidLength(
    const std::vector<FrenetFramePoint> &frenet_path,
    const ReferenceLinePtr &reference_line,
    std::vector<Boundary> &adc_boundarys, std::vector<Box2d> &adc_boxes,
    std::vector<Box2d> &adc_boxes_sl,
    std::vector<std::vector<Vec2d>> &adc_box_xy_pts_vec,
    std::vector<std::vector<Vec2d>> &adc_box_sl_pts_vec,
    std::vector<ReferencePoint> &reference_points);

void CalcBoundary(const ReferencePoint &reference_point,
                  const Box2d &bounding_box, const SLPoint &center_sl,
                  Boundary &boundary);

bool GetWithoutRoadBoundCollisionLength(
    const std::vector<ReferencePoint> &reference_points,
    const std::vector<FrenetFramePoint> &frenet_path, double *valid_length);

bool GetWithoutObsCollisionLength(
    const std::size_t &planner_type,
    const PathObstacleContext &path_obstacle_context,
    const std::vector<Boundary> &adc_boundarys,
    const std::vector<Box2d> &adc_boxes, const std::vector<Box2d> &adc_boxes_sl,
    const std::vector<std::vector<Vec2d>> &adc_box_xy_pts_vec,
    const std::vector<std::vector<Vec2d>> &adc_box_sl_pts_vec,
    const std::vector<FrenetFramePoint> &frenet_path,
    double *const valid_length);

}  // namespace path_planner_common
}  // namespace planning
}  // namespace neodrive
