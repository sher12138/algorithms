#pragma once

#include "src/planning/util/path_planner_common.h"
#include "src/planning/util/visual_lane_combine_ego_lane.h"
#include "task/optimizers/backup_path_optimizer/backup_path_planner.h"

namespace neodrive {
namespace planning {

/// Backup path planer using quintic BVP curves
class QuinticBvpBackupPathPlanner final : public BackupPathPlanner {
 public:
  QuinticBvpBackupPathPlanner();

  bool GeneratePath(TaskInfo& task_info,
                    std::vector<BackupPathPlanner::WayPoint>* ans,
                    double& gain) override;
};

}  // namespace planning
}  // namespace neodrive
