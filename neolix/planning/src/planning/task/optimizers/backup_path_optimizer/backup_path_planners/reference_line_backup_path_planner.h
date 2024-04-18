#pragma once

#include "task/optimizers/backup_path_optimizer/backup_path_planner.h"

namespace neodrive {
namespace planning {

/// Generate curve along with reference line
/// (keeps lateral distance, not towards reference line)
class ReferenceLineBackupPathPlanner final : public BackupPathPlanner {
 public:
  ReferenceLineBackupPathPlanner();

  bool GeneratePath(TaskInfo& task_info, 
                    std::vector<BackupPathPlanner::WayPoint>* ans,
                    double &gain) override;
};

}  // namespace planning
}  // namespace neodrive
