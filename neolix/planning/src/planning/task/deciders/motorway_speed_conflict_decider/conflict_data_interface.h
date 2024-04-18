#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "src/planning/common/data_center/speed_context.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/task/task_info.h"

namespace neodrive {
namespace planning {

class ConflictDataInterface {
 public:
  explicit ConflictDataInterface(const std::string& name) : name_(name) {}
  virtual ~ConflictDataInterface() = default;

  virtual std::vector<ConnectionConflictInfo> ComputeConflictMergeInData(
      TaskInfo& task_info) = 0;
  virtual std::vector<ConnectionConflictInfo> ComputeConflictMeetingData(
      TaskInfo& task_info) = 0;
  virtual std::vector<ConnectionConflictInfo> ComputeConflictCustomData(
      TaskInfo& task_info) = 0;

  virtual const bool HasMergingArea(TaskInfo& task_info) const;

  virtual const bool CalAgentInfo(Obstacle* obs, VehicleInfo* agent_info) const;

  virtual const bool SetConflictParam(ConnectionConflictInfo* c) const;

  const void set_interactive_agent_ids(const std::unordered_set<int>& agent_ids,
                                       const MotorwayInteractiveType& type) {
    interactive_agent_ids_ = agent_ids;
    scenario_type_ = type;
  }

  const void set_interactive_agent_ids(const std::unordered_set<int>& agent_ids,
                                       const MotorwayInteractiveType& type,
                                       const RightOfWay& way_right) {
    interactive_agent_ids_ = agent_ids;
    scenario_type_ = type;
    way_right_ = way_right;
  }

  virtual const std::string& name() const { return name_; }

 protected:
  std::unordered_set<int> interactive_agent_ids_{};
  MotorwayInteractiveType scenario_type_ = MotorwayInteractiveType::NONE;
  RightOfWay way_right_ = RightOfWay::UNKNOWN;

  config::AutoPlanningResearchConfig::MotorwaySpeedConfilictDeciderConfig
      conflict_config_ = config::PlanningConfig::Instance()
                             ->planning_research_config()
                             .motorway_speed_confilict_decider_config;

 private:
  std::string name_{""};
};

}  // namespace planning
}  // namespace neodrive
