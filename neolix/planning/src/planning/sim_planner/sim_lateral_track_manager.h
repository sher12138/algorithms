#pragma once
#include "sim_lateral_manager.h"
#include "sim_lateral_track_planner.h"
#include "src/planning/common/data_center/data_center.h"
namespace neodrive {
namespace planning {
namespace sim_planner {
class SimLateralTrackManager : public SimLateralManager {
 public:
  using SimLateralTrackPlanner =
      neodrive::planning::sim_planner::SimLateralTrackPlanner;
  using CostStructureRP = SimLateralTrackPlanner::CostStructureRP;

  struct Snapshot {
    bool valid = false;
    int original_winner_id;
    int processed_winner_id;
    State plan_state;
    std::vector<std::vector<DcpAction>> action_script;
    std::vector<bool> sim_res;
    std::vector<bool> risky_res;
    std::vector<std::string> sim_info;
    std::vector<double> final_cost;
    std::vector<std::vector<CostStructureRP>> progress_cost;
    std::vector<std::vector<Vehicle>> forward_trajs;
    std::vector<std::vector<LateralBehavior>> forward_lat_behaviors;
    std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors;
    std::vector<ReferencePoint> ref_lane;

    double plan_stamp = 0.0;
    double time_cost = 0.0;
  };

  bool run(const double stamp, const Task& task, const State& ego_state,
           const std::vector<SimMapPoint>& track_pts,
           const std::unordered_map<std::string, int>& all_lanes_id,
           Snapshot* last_snapshot);

  void saveSnapshot(const double stamp, Snapshot* snapshot);

 private:
  bool prepare(const double stamp, const Task& task, const State& ego_state);

  bool evaluateReferenceVelocity(const Task& task, double* ref_vel);

  Snapshot last_snapshot_;

  SimLateralTrackPlanner bp_;
  SimMap* sim_map_{SimMap::Instance()};

  State ego_state_;

  int ego_road_id_{-1};
  int ego_lane_id_{0};
};
}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive