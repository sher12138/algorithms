#pragma once

#include "conflict_data_interface.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/obstacle/decision_data.h"
#include "src/planning/config/auto_planning_research_config.h"
#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

class ConflictDataBasePrediction final : public ConflictDataInterface {
 public:
  ConflictDataBasePrediction();
  virtual ~ConflictDataBasePrediction() = default;

  virtual std::vector<ConnectionConflictInfo> ComputeConflictMergeInData(
      TaskInfo& task_info) override;
  virtual std::vector<ConnectionConflictInfo> ComputeConflictMeetingData(
      TaskInfo& task_info) override;
  virtual std::vector<ConnectionConflictInfo> ComputeConflictCustomData(
      TaskInfo& task_info) override;

 private:
  void CalcCircle(const double width, const double length, const size_t n,
                  double& r, double& d);
  void CalcCirclePoint(const Vec2d& center_pt, const double heading,
                       const double r, const double d,
                       PredictionCurves& first_circle,
                       PredictionCurves& second_circle,
                       PredictionCurves& third_circle) const;
  bool CalcAgentCurves(const Obstacle* obs,
                       std::vector<PredictionCurves>& agent_curves,
                       std::vector<Vec2d>& agent_traj);
  bool CalcEgoCurves(TaskInfo& task_info, PredictionCurves& ego_curves,
                     std::vector<Vec2d>& ego_traj);
  void CalcMeeting(const PredictionCurves& ego, const PredictionCurves& agent,
                   std::array<math::AD2, 4>& res);
  void CalcConflictZone(const std::vector<Vec2d>& ego,
                        const std::vector<Vec2d>& agent,
                        std::array<math::AD2, 4>& res, math::AD2& ego_rd,
                        math::AD2& agent_rd);

 private:
  const config::AutoPlanningResearchConfig::
      MotorwaySpeedConfilictDeciderConfig::ConflictData::BasePath&
          conflict_data_base_path_config_ =
              config::PlanningConfig::Instance()
                  ->planning_research_config()
                  .motorway_speed_confilict_decider_config.conflict_data
                  .base_path;
};

}  // namespace planning
}  // namespace neodrive
