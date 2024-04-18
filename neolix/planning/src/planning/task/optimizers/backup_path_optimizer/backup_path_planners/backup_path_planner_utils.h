#pragma once

#include "task/optimizers/backup_path_optimizer/backup_path_planner.h"
#include "util/path_planner_common.h"
namespace neodrive {
namespace planning {

namespace backup_utils {

void FillWayPointHeading(std::vector<BackupPathPlanner::WayPoint>* pts);
void FillWayPointLateral(std::vector<BackupPathPlanner::WayPoint>* pts);

TrajectoryPoint Trans2Utm(const TrajectoryPoint& odom_veh);

}  // namespace backup_utils

}  // namespace planning
}  // namespace neodrive
