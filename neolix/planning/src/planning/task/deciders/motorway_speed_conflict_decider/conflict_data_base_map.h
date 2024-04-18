#pragma once

#include "conflict_data_interface.h"
#include "src/planning/planning_map/planning_map.h"

namespace neodrive {
namespace planning {

class ConflictDataBaseMap final : public ConflictDataInterface {
 public:
  ConflictDataBaseMap();
  virtual ~ConflictDataBaseMap() = default;

  virtual std::vector<ConnectionConflictInfo> ComputeConflictMergeInData(
      TaskInfo& task_info) override;
  virtual std::vector<ConnectionConflictInfo> ComputeConflictMeetingData(
      TaskInfo& task_info) override;
  virtual std::vector<ConnectionConflictInfo> ComputeConflictCustomData(
      TaskInfo& task_info) override {
    return {};
  };
};

}  // namespace planning
}  // namespace neodrive
