#include "em_planning_data.h"

#include "common/planning_gflags.h"

namespace neodrive {
namespace planning {

std::string EMPlanningData::type() const { return "EMPlanningData"; }

bool EMPlanningData::AggregatePathAndSpeedData(const double time_resolution,
                                               const double time_length) {
  if (time_resolution <= 1e-6) {
    LOG_ERROR("time_resolution [{:.4f}] <= 0.0", time_resolution);
    return false;
  }
  if (computed_trajectory_.num_of_points() != 0) {
    LOG_ERROR("computed_trajectory_.num_of_points({}) != 0",
              computed_trajectory_.num_of_points());
    return false;
  }
  if (time_length < time_resolution) {
    return false;
  }

  const SpeedData &speed_data = speed_data_;
  const PathData &path_data = path_data_;
  for (double cur_rel_time = time_resolution;
       cur_rel_time < speed_data.total_time() && cur_rel_time <= time_length;
       cur_rel_time += time_resolution) {
    SpeedPoint speed_point;
    if (!speed_data.get_speed_point_with_time(cur_rel_time, &speed_point)) {
      LOG_ERROR("Fail to get speed point with relative time {}", cur_rel_time);
      break;
    }
    if (speed_point.s() > path_data.path().param_length()) {
      break;
    }
    PathPoint path_point;
    if (!path_data.get_path_point_with_path_s(speed_point.s(), &path_point)) {
      LOG_ERROR(
          "Fail to get path data with s {} at t {}, path total length {} ",
          speed_point.s(), speed_point.t(), path_data.path().param_length());
      break;
    }
    path_point.set_s(init_planning_point_.s() + path_point.s());

    TrajectoryPoint trajectory_point(
        path_point, speed_point.v(), speed_point.a(), speed_point.j(),
        init_planning_point_.relative_time() + speed_point.t());
    computed_trajectory_.append_trajectory_point(trajectory_point);
  }

  return true;
}

}  // namespace planning
}  // namespace neodrive
