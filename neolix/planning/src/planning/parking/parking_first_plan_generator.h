#pragma once
#include <memory>

#include "src/planning/common/planning_logger.h"
#include "src/planning/reference_line/reference_line.h"
#include "src/planning/common/trajectory/trajectory_point.h"
#include "common/math/util.h"
#include "src/planning/parking/generator/parking_spot_decider.h"
#include "src/planning/parking/generator/parking_trajectory_optimizer.h"
#include "src/planning/parking/geoparking_in/ParallelParking_in.h"
#include "src/planning/parking/geoparking_in/PerpendicularParking_in.h"
#include "src/planning/parking/geoparking_out/AngularParking_out.h"
#include "src/planning/parking/geoparking_out/ParallelParking_out.h"
#include "src/planning/parking/geoparking_out/PerpendicularParking_out.h"
#include "src/planning/parking/parking_config.h"
#include "src/planning/parking/parking_data_info.h"

namespace neodrive {
namespace planning {

class ParkingFirstPlanGenerator {
 public:
  ParkingFirstPlanGenerator();
  ~ParkingFirstPlanGenerator();

  /*
   * park_in_out: false-in; true-out;
   * park_spot_type: 0:undefined; 1: perpendicular; 2: parallel; 3: inclined
   * head_tail_in: 0: undefined, 1: tail in; 2: head in
   */

  // bool Generate(TrajectoryPoint& start_pos, ReferenceLinePtr&
  // parking_ref_line,
  //               std::array<Vec2d, 4>& spot_corners, bool park_in_out = false,
  //               int park_spot_type = 0, int head_tail_in = 0);

  bool Generate(
      TrajectoryPoint& start_pos, /*ReferenceLinePtr& parking_ref_line,*/
      ParkingDataInfo& park_data_info, bool park_in_out);

  // TODO(wyc): const reference may be better
  std::vector<TrajectoryPoint> get_results();
  void Reset();

  bool TrajectoryPartition(
      std::vector<TrajectoryPoint>& traj,
      std::vector<std::vector<TrajectoryPoint>>& partition_traj);

 private:
  bool GenerateParkIn();

  bool GenerateParkOut();

  bool GeometricParkInWithoutSmooth();

  bool GeometricParkInWithSmooth();

  bool HybridAStarParkIn();

  bool GeometricParkOutWithoutSmooth();

  bool GeometricParkOutWithSmooth();

  bool HybridAStarParkOut();

  // bool park_out_start_final_pos_adjust(TrajectoryPoint& start_pos,
  //                                      ReferenceLinePtr& parking_ref_line,
  //                                      std::array<Vec2d, 4>& spot_corners);

  bool ParkOutStartFinalPosAdjust(TrajectoryPoint& start_pos,
                                  //  ReferenceLinePtr& parking_ref_line,
                                  ParkingDataInfo& park_data_info,
                                  bool park_in_out);

  // bool TrajectoryPartition(
  //     std::vector<TrajectoryPoint>& traj,
  //     std::vector<std::vector<TrajectoryPoint>>& partition_traj);

  // ** debug
  void LogReferLineBound();
  void LogParkingResults(std::vector<TrajectoryPoint>& traj);

 private:
  // input
  TrajectoryPoint start_pos_;
  ReferenceLinePtr parking_ref_line_;
  // false: in; true: out
  bool park_in_out_ = false;
  // 0:undefined; 1: perpendicular; 2: parallel; 3: inclined
  int park_spot_type_ = 0;
  // 0: undefined, 1: tail in; 2: head in
  int head_tail_in_ = 0;

  // inner
  std::vector<TrajectoryPoint> trajectory_points_{};

  ParkingConfig parking_config_;
  ParkingDataInfo park_data_info_;
  // ParkingSpotDecider park_spot_decider_;

  // park in planner
  PP_ParkingIn pp_in_;
  PPP_ParkingIn ppp_in_;

  // park out planner
  AP_ParkingOut ap_out_;
  PP_ParkingOut pp_out_;
  PPP_ParkingOut ppp_out_;

  std::unique_ptr<ParkingTrajectoryOptimizer> park_optimizer_;
  std::unique_ptr<IterativeAnchoringSmoother> iterative_anchoring_smoother_;
};

}  // namespace planning
}  // namespace neodrive
