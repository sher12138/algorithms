#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "src/planning/common/data_center/speed_context.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/task/task_info.h"

namespace neodrive {
namespace planning {

class BackDataInterface {
 public:
  explicit BackDataInterface(const std::string& name) : name_(name) {}
  virtual ~BackDataInterface() = default;

  virtual std::vector<ConnectionConflictInfo> ComputeConflictMergeInData(
      TaskInfo& task_info) = 0;
  virtual std::vector<ConnectionConflictInfo> ComputeConflictMeetingData(
      TaskInfo& task_info) = 0;
  virtual std::vector<ConnectionConflictInfo> ComputeConflictCustomData(
      TaskInfo& task_info) = 0;

  virtual const bool HasMergingArea(TaskInfo& task_info) const;

  virtual const bool CalAgentInfo(const Obstacle* obs,
                                  VehicleInfo* agent_info) const;

  virtual const bool SetConflictParam(ConnectionConflictInfo* c) const;

  const void set_interactive_agent_ids(const std::unordered_set<int>& agent_ids,
                                       const BackInteractiveType& type) {
    interactive_agent_ids_ = agent_ids;
    scenario_type_ = type;
  }

  const void set_interactive_agent_ids(const std::unordered_set<int>& agent_ids,
                                       const BackInteractiveType& type,
                                       const RightOfWay& way_right) {
    interactive_agent_ids_ = agent_ids;
    scenario_type_ = type;
    way_right_ = way_right;
  }

  virtual const std::string& name() const { return name_; }

 protected:
  std::unordered_set<int> interactive_agent_ids_{};
  BackInteractiveType scenario_type_ = BackInteractiveType::NONE;
  RightOfWay way_right_ = RightOfWay::UNKNOWN;

  config::AutoPlanningResearchConfig::BackSpeedConfilictDeciderConfig
      back_config_ = config::PlanningConfig::Instance()
                         ->planning_research_config()
                         .back_speed_confilict_decider_config;

 private:
  std::string name_{""};
};

}  // namespace planning
}  // namespace neodrive
