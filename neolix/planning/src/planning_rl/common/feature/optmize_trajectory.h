#pragma once
#include <vector>
#include "common/opt_traj/include/horner.h"
#include "common/opt_traj/include/polyTraj.h"
#include "common/opt_traj/include/polyTraj2D.h"
#include "common/opt_traj/include/poly_traj_structs.h"
#include "common/opt_traj/include/rootTraj.h"
#include "feature_data_struct.h"

namespace neodrive {
namespace planning_rl {

std::vector<TrajectoryPoint2D> Generate2DTraj(
    const TrajPoint &lon_start_state, const TrajPoint &lon_end_state,
    const TrajPoint &lat_start_state, const TrajPoint &lat_end_state,
    const double &deltaT, const double &resolution,
    const std::vector<CurvePoint> &center_line, const int &mode);

}  // namespace planning_rl
}  // namespace neodrive