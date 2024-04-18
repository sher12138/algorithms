#pragma once

#include "conflict_data_base_map.h"
#include "conflict_data_base_path.h"
#include "conflict_data_base_prediction.h"
#include "conflict_info_interface.h"

namespace neodrive {
namespace planning {

class ConflictInfoForStraight final : public ConflictInfoInterface {
 public:
  ConflictInfoForStraight();
  virtual ~ConflictInfoForStraight() = default;

  std::vector<ConnectionConflictInfo> ComputeConflictInfo(
      TaskInfo& task_info) override;

 private:
  void CalcMergeInAgentsId(const std::vector<Obstacle*>& dynamic_obs,
                           const InsidePlannerData& inside_data,
                           const double cur_s);
  void CalcMeetingAgentsID(const std::vector<Obstacle*>& dynamic_obs,
                           const InsidePlannerData& inside_data);

 private:
  std::unordered_set<int> meeting_agent_ids_{};
  std::unordered_set<int> merge_in_agent_ids_{};
};

}  // namespace planning
}  // namespace neodrive
