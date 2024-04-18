#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class ParkingSpeedObsDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(ParkingSpeedObsDecider);

 public:
  virtual ~ParkingSpeedObsDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  void SteeringDetect(TaskInfo& task_info, bool& collide) const;
  void PathDetect(TaskInfo& task_info, bool& collide) const;
  void HeadingDetect(TaskInfo& task_info, bool& collide) const;
  void SetIgnoreObsList(TaskInfo& task_info);
  void SetIgnoreObsListForParkingOut(TaskInfo& task_info) const;
  bool GetCameraPoints(
      const global::perception::SingleCameraSegmentation& camera_segments,
      std::vector<Vec2d>& camera_points,
      std::vector<std::vector<Vec2d>>& holes) const;

 private:
  std::unordered_set<int> camera_ignore_obs_;
};

REGISTER_SCENARIO_TASK(ParkingSpeedObsDecider);

}  // namespace planning
}  // namespace neodrive