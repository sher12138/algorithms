#include "task/optimizers/backup_path_optimizer/backup_path_planner_factory.h"

namespace neodrive {
namespace planning {

namespace {

BackupPathPlanner::Ptr CreatePlanner(const BackupPathPlannerType t) {
  switch (t) {
    case BackupPathPlannerType::kCubicBvp:
      return std::make_shared<CubicBvpBackupPathPlanner>();
    case BackupPathPlannerType::kQuinticBvp:
      return std::make_shared<QuinticBvpBackupPathPlanner>();
    case BackupPathPlannerType::kRefLine:
      return std::make_shared<ReferenceLineBackupPathPlanner>();
    default:
      return BackupPathPlanner::Ptr{};
  }
  return BackupPathPlanner::Ptr{};
}

}  // namespace

BackupPathPlanner::Ptr BackupPathPlannerFactory::operator[](
    const BackupPathPlannerType t) {
  return planners_.count(t) ? planners_[t] : planners_[t] = CreatePlanner(t);
}

}  // namespace planning
}  // namespace neodrive
