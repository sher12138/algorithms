#include "st_boundary_mapper.h"

#include <limits>

#include "common/math/double.h"
#include "src/planning/common/vehicle_param.h"
#include "src/planning/math/public_math/utility.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {

StBoundaryMapper::StBoundaryMapper(
    const STBoundaryConfiguration& st_boundary_config)
    : st_boundary_config_(st_boundary_config) {}

ErrorCode StBoundaryMapper::GetSpeedLimits(
    const InsidePlannerData& inside_data, const PathData& path_data,
    const double& planning_distance, const std::size_t& matrix_dimension_s,
    const double& default_speed_limit,
    SpeedLimit* const speed_limit_data) const {
  if (speed_limit_data == nullptr || path_data.path().path_points().empty()) {
    LOG_ERROR("speed_limit_data or path_data invalid");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (planning_distance > path_data.path().path_points().back().s()) {
    LOG_ERROR("path length [{}] cannot be less than planning_distance [{}]",
              path_data.path().path_points().back().s(), planning_distance);
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  double init_speed_limits = default_speed_limit;

  if (inside_data.is_pose_adjust || inside_data.is_inlane_uturn ||
      inside_data.is_reverse_driving || inside_data.is_parking_out_slot ||
      inside_data.is_parking_in_slot || inside_data.is_outlane_uturn) {
    init_speed_limits = std::min(init_speed_limits,
                                 st_boundary_config_.maximal_parking_speed());
    LOG_INFO("pose_adjust/inlane/reverse/parking max speed: {:.4f}",
             init_speed_limits);
  }

  // 10m or 3s to find max kappa
  double max_kappa_length = inside_data.vel_v * 3.0;
  double max_kappa_in_max_kappa_length = 0.0;
  max_kappa_length = std::max(max_kappa_length, 10.0);
  for (std::size_t i = 0; i < path_data.path().path_points().size(); ++i) {
    if (path_data.path().path_points().at(i).s() >= max_kappa_length) break;
    max_kappa_in_max_kappa_length =
        std::max(max_kappa_in_max_kappa_length,
                 std::abs(path_data.path().path_points().at(i).kappa()));
  }
  // set limit from centric_accel
  if (max_kappa_in_max_kappa_length > st_boundary_config_.kappa_threshold()) {
    double tmp_limits =
        std::sqrt(st_boundary_config_.centric_acceleration_limit() /
                  max_kappa_in_max_kappa_length);
    init_speed_limits = std::min(init_speed_limits, tmp_limits);
    LOG_INFO("after curv max speed: {:.4f}", init_speed_limits);
  }

  // vehicle steer protection, central limit
  double vehicle_kappa = 0.0;
  if (std::abs(inside_data.vel_steer_angle) > 5.0) {
    vehicle_kappa = 0.25 * std::abs(inside_data.vel_steer_angle) / 100.0;
  }
  if (vehicle_kappa > 0.02) {
    double tmp_max = std::sqrt(
        st_boundary_config_.centric_acceleration_limit() / vehicle_kappa);
    init_speed_limits = std::min(init_speed_limits, tmp_max);
    LOG_INFO("after vehicle steer curv max speed: {:.4f}", init_speed_limits);
  }

  init_speed_limits =
      std::max(init_speed_limits, st_boundary_config_.lowest_speed());

  double s = 0.0;
  const double unit_s = planning_distance / matrix_dimension_s;
  std::size_t i = 0;
  std::size_t j = 1;

  const auto& path_points = path_data.path().path_points();
  while (i < matrix_dimension_s && j < path_points.size()) {
    const auto& point = path_points[j];
    if (Double::compare(s, point.s()) > 0) {
      ++j;
    } else {
      speed_limit_data->mutable_speed_limits()->push_back(
          SpeedPoint(s, 0.0, init_speed_limits, 0.0, 0.0));
      s += unit_s;
      ++i;
    }
  }
  return ErrorCode::PLANNING_OK;
}

const STBoundaryConfiguration& StBoundaryMapper::StBoundaryConfig() const {
  return st_boundary_config_;
}

bool StBoundaryMapper::CreateYieldBufferBoundaryForReverse(
    const STGraphBoundary& origin_boundary, const double& minimal_time_scope,
    const double& planning_time, STGraphBoundary* const buffer_boundary) const {
  if (buffer_boundary == nullptr) return false;

  double boundary_time_scope =
      std::abs(origin_boundary.max_t() - origin_boundary.min_t());
  double min_s = origin_boundary.min_s();
  bool need_create_boundary_flag =
      (min_s < st_boundary_config_.static_obs_stop_dis())
          ? true
          : ((origin_boundary.max_t() >= planning_time ||
              boundary_time_scope >= minimal_time_scope)
                 ? false
                 : true);

  if (need_create_boundary_flag) {
    double expand_time =
        std::max(1.0, minimal_time_scope - boundary_time_scope);
    double buffer_end_t =
        std::min(origin_boundary.max_t() + expand_time, planning_time);
    std::vector<STPoint> boundary_points;
    double mapper_min_s =
        std::min(origin_boundary.point(0).y(), origin_boundary.point(1).y());
    boundary_points.emplace_back(mapper_min_s, origin_boundary.min_t());
    boundary_points.emplace_back(mapper_min_s, buffer_end_t);
    boundary_points.emplace_back(origin_boundary.max_s(), buffer_end_t);
    boundary_points.emplace_back(origin_boundary.max_s(),
                                 origin_boundary.min_t());

    *buffer_boundary = {boundary_points};
    buffer_boundary->set_virtual(origin_boundary.is_virtual());
    buffer_boundary->set_id(origin_boundary.id());
    buffer_boundary->set_characteristic_length(
        origin_boundary.characteristic_length());
    buffer_boundary->set_probability(origin_boundary.probability());
    buffer_boundary->set_boundary_type(origin_boundary.boundary_type());
    buffer_boundary->set_decision(origin_boundary.get_decision());
  }

  return need_create_boundary_flag;
}

bool StBoundaryMapper::CreateYieldBufferBoundaryForNonReverse(
    const STGraphBoundary& origin_boundary, const double& minimal_time_scope,
    const double& planning_time, STGraphBoundary* const buffer_boundary) const {
  if (buffer_boundary == nullptr) return false;

  double boundary_time_scope =
      std::abs(origin_boundary.max_t() - origin_boundary.min_t());
  double min_s = origin_boundary.min_s();
  bool need_create_boundary_flag = (origin_boundary.max_t() >= planning_time ||
                                    boundary_time_scope >= minimal_time_scope)
                                       ? false
                                       : true;

  if (need_create_boundary_flag) {
    double expand_time =
        std::max(1.0, minimal_time_scope - boundary_time_scope);
    double buffer_end_t =
        std::min(origin_boundary.max_t() + expand_time, planning_time);
    std::vector<STPoint> boundary_points;
    double mapper_min_s =
        std::fmin(origin_boundary.point(0).y(), origin_boundary.point(1).y());
    boundary_points.emplace_back(mapper_min_s, origin_boundary.min_t());
    boundary_points.emplace_back(mapper_min_s, buffer_end_t);
    boundary_points.emplace_back(origin_boundary.max_s(), buffer_end_t);
    boundary_points.emplace_back(origin_boundary.max_s(),
                                 origin_boundary.min_t());

    *buffer_boundary = {boundary_points};
    buffer_boundary->set_virtual(origin_boundary.is_virtual());
    buffer_boundary->set_id(origin_boundary.id());
    buffer_boundary->set_characteristic_length(
        origin_boundary.characteristic_length());
    buffer_boundary->set_probability(origin_boundary.probability());
    buffer_boundary->set_boundary_type(origin_boundary.boundary_type());
    buffer_boundary->set_decision(origin_boundary.get_decision());
  }

  return need_create_boundary_flag;
}

}  // namespace planning
}  // namespace neodrive
