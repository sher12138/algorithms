#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class ParkingPathOptimizer final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(ParkingPathOptimizer);

 public:
  virtual ~ParkingPathOptimizer() override;

  ErrorCode Execute(TaskInfo& task_info) override;

  void SaveTaskResults(TaskInfo& task_info) override;

  void Reset() override;

 private:
  std::size_t GetNearestPointIndex(const ParkingPath& curr_parking_path) const;
  Vec3d GetPathPointFromS(const ParkingPath& curr_parking_path, double s) const;
  void BuildPathPoint(const std::vector<Vec3d>& origin_path,
                      std::vector<PathPoint>& final_path,
                      bool is_reverse_driving = false) const;
  void TransformUtmPathToOdom(std::vector<Vec3d>& path);
  void SetParkingSpeedLimit(TaskInfo& task_info,
                            const ParkingPath& curr_parking_path) const;
  void JudgeIfStuck();
  int RematchPathIndex() const;
  void UpdateDistToEnd();
  double GetDistTargetSpeed(const ParkingPath& current_path, double current_s,
                            double dis2end) const;

 private:
  int path_idx_{0};
  bool stop_and_steer_{false};
  int ego_stop_count_{0};
  int ego_stuck_count_{0};
  double dist_to_end_{0.0};
  double start_stop_t_{0.0};
  static constexpr double kStuckTimeThreshold = 60.0;
  static constexpr double kStopVelThreshold = 0.12;
};

REGISTER_SCENARIO_TASK(ParkingPathOptimizer);

}  // namespace planning
}  // namespace neodrive
