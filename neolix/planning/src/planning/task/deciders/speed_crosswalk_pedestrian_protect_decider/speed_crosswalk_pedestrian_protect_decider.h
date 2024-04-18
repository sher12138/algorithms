#pragma once

#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/math/polygon2d.h"
#include "src/planning/common/math/vec2d.h"
#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class SpeedCrosswalkPedestrainProtectDecider final
    : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedCrosswalkPedestrainProtectDecider);

 public:
  virtual ~SpeedCrosswalkPedestrainProtectDecider() override;
  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;
  CipvDecision ReturnResult() const;

 private:
  bool Init(TaskInfo& task_info);
  bool Process(TaskInfo& task_info);
  bool IfInCrosswalkCheckArea(TaskInfo& task_info);
  void CrosswalkPoly2Boundary(
      const std::vector<common::math::ShmPolygon2d>& crosswalk_polygen_vec,
      TaskInfo& task_info);
  bool ProcessProtect(TaskInfo& task_info);
  void CheckDynamicPedestrianInCheckArea(TaskInfo& task_info);
  void CheckStaticPedestrianInCheckArea(TaskInfo& task_info);
  void SetSpeedLimit(double speed_limit, TaskInfo& task_info, bool if_hard);
  void GetPedestrianCheckArea(TaskInfo& task_info);

 protected:
  const VehicleStateProxy& vehicle_state_{data_center_->vehicle_state_proxy()};

 private:
  bool if_next_to_crosswalk_{false};
  bool if_crosswalk_boundary_{false};
  std::vector<SLPoint> corsswalk_boundary_points_{};
  std::pair<double, double> crosswalk_range{0.0, 0.0};
  std::vector<common::math::ShmPolygon2d> crosswalk_polygen_vec_;
  Boundary crosswalk_boundary_;
  std::vector<uint64_t> crosswalk_id_vec_{};

  double adc_current_s_{0.0};
  double adc_front_edge_s_{0.0};
};

REGISTER_SCENARIO_TASK(SpeedCrosswalkPedestrainProtectDecider);

}  // namespace planning
}  // namespace neodrive
