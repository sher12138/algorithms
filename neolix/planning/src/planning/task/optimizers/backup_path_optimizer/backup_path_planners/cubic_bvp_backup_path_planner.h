#pragma once

#include "task/optimizers/backup_path_optimizer/backup_path_planner.h"

namespace neodrive {
namespace planning {

/// Backup path planer using cubic BVP curves
class CubicBvpBackupPathPlanner final : public BackupPathPlanner {
 public:
  CubicBvpBackupPathPlanner();

  bool GeneratePath(TaskInfo& task_info, 
                    std::vector<BackupPathPlanner::WayPoint>* ans,
                    double &gain) override;
};

}  // namespace planning
}  // namespace neodrive
