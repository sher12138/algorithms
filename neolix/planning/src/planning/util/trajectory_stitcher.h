#pragma once

#include "src/planning/common/data_center/frame.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/common/planning_macros.h"
namespace neodrive {
namespace planning {
// TODO(wwl) : using Google naming rule
class TrajectoryStitcher {
  DECLARE_SINGLETON(TrajectoryStitcher);

 public:
  ErrorCode stitch(const VehicleStateProxy& vehicle_state,
                   const ControlCommand& control_command,
                   const Frame* const prev_frame, const double curr_time,
                   const double planning_cycle_time,
                   std::vector<TrajectoryPoint>* const stitching_trajectory);

  bool IsUsePositionStitch() const { return is_use_position_stitch_; }

 private:
  void BuildReplanStitchingTrajectory(
      const VehicleStateProxy& vehicle_state,
      std::vector<TrajectoryPoint>* const stitching_trajectory);

  bool is_use_position_stitch_{false};
};

}  // namespace planning
}  // namespace neodrive
