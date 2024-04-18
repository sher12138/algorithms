#pragma once

#include "task/optimizers/backup_path_optimizer/backup_path_planner.h"
#include "task/optimizers/backup_path_optimizer/backup_path_planners/cubic_bvp_backup_path_planner.h"
#include "task/optimizers/backup_path_optimizer/backup_path_planners/quintic_bvp_backup_path_planner.h"
#include "task/optimizers/backup_path_optimizer/backup_path_planners/reference_line_backup_path_planner.h"

namespace neodrive {
namespace planning {

enum class BackupPathPlannerType {
  kCubicBvp,
  kQuinticBvp,
  kRefLine,
};

class BackupPathPlannerFactory final {
 public:
  BackupPathPlannerFactory() = default;
  ~BackupPathPlannerFactory() = default;
  BackupPathPlannerFactory(const BackupPathPlannerFactory&) = delete;
  BackupPathPlannerFactory& operator=(const BackupPathPlanner&) = delete;
  BackupPathPlannerFactory(BackupPathPlanner&&) = delete;
  BackupPathPlannerFactory& operator=(BackupPathPlanner&&) = delete;

 public:
  /// Interface to get planner pointer stored
  /// @param t Type of the planner
  /// @return The pointer of planner; nullptr if not found
  BackupPathPlanner::Ptr operator[](const BackupPathPlannerType t);

 private:
  std::unordered_map<BackupPathPlannerType, BackupPathPlanner::Ptr> planners_{};
};

}  // namespace planning
}  // namespace neodrive
