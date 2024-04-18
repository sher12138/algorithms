#pragma once

#include "common/data_center/inside_planner_data.h"
#include "common/data_center/outside_planner_data.h"
#include "common/obstacle/decision_data.h"
#include "common/trajectory/trajectory_point.h"
#include "reference_line/reference_line.h"
#include "reference_line/reference_point.h"
#include "src/planning/task/task_info.h"

namespace neodrive {
namespace planning {

/// Interface for backup planners
class BackupPathPlanner {
 public:
  using Ptr = std::shared_ptr<BackupPathPlanner>;

 public:
  struct WayPoint {
    double x{0};
    double y{0};
    double z{0};

    double s{0};
    double l{0};
    double dl{0};
    double ddl{0};

    double theta{0};
    double kappa{0};
  };

 public:
  /// Disable default constructor, must construct with a name
  BackupPathPlanner() = delete;
  virtual ~BackupPathPlanner() = default;

  /// Constructor with name
  /// @param name Name
  explicit BackupPathPlanner(const std::string& name) : name_{name} {}

  /// Generate path
  /// @param ref_line Reference line
  /// @param veh Vehicle current state
  /// @param decision_data Obstacle information
  /// @param Path generated
  virtual bool GeneratePath(TaskInfo& task_info, std::vector<WayPoint>* ans,
                            double& gain) = 0;

 public:
  const std::string& name() const { return name_; }

 private:
  std::string name_{};
};

}  // namespace planning
}  // namespace neodrive
