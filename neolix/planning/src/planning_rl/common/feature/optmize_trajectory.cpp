#include "optmize_trajectory.h"

namespace neodrive {
namespace planning_rl {

std::vector<TrajectoryPoint2D> Generate2DTraj(
    const TrajPoint &lon_start_state, const TrajPoint &lon_end_state,
    const TrajPoint &lat_start_state, const TrajPoint &lat_end_state,
    const double &deltaT, const double &resolution,
    const std::vector<CurvePoint> &center_line, const int &mode) {
  double t_current = 0.0;
  double horizon = deltaT;
  double t = deltaT;
  RootTraj root_traj(center_line);
  root_traj.generate_5th_order(lon_start_state.x, lon_start_state.x_der,
                               lon_start_state.x_dder, lon_end_state.x,
                               lon_end_state.x_der, lon_end_state.x_dder,
                               t_current, t, horizon);
  root_traj.discrete_traj_points_.resize(1);
  root_traj.sampleArgumentEquidistant(t_current, resolution);
  root_traj.calculateRootTrajectory();

  PolyTraj lat_poly_traj;
  lat_poly_traj.generate_5th_order(lat_start_state.x, lat_start_state.x_der,
                                   lat_start_state.x_dder, lat_end_state.x,
                                   lat_end_state.x_der, lat_end_state.x_dder,
                                   t_current, t, horizon);
  lat_poly_traj.discrete_traj_points_.resize(1);
  lat_poly_traj.sampleArgumentEquidistant(t_current, resolution);

  PolyTraj2D my_2d_traj(lat_poly_traj, root_traj);
  return my_2d_traj.trajectory2D_;
}

}  // namespace planning_rl
}  // namespace neodrive