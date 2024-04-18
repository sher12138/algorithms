#pragma once
#include "sim_lateral_planner.h"
#include "src/planning/common/data_center/data_center.h"

namespace neodrive {
namespace planning {
namespace sim_planner {
class SimLateralManager {
 public:
  using DcpLatAction = neodrive::planning::sim_planner::DcpTree::DcpLatAction;
  using DcpLonAction = neodrive::planning::sim_planner::DcpTree::DcpLonAction;
  using DcpAction = neodrive::planning::sim_planner::DcpTree::DcpAction;
  using CostStructure =
      neodrive::planning::sim_planner::SimLateralPlanner::CostStructure;

  enum class LaneChangeTriggerType { kStick = 0, kActive };

  struct ReplanningContext {
    bool is_valid = false;
    double seq_start_time;
    std::vector<DcpAction> action_seq;
  };

  struct ActivateLaneChangeRequest {
    double trigger_time;
    double desired_operation_time;
    int ego_lane_id;
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
  };

  struct LaneChangeProposal {
    bool valid = false;
    double trigger_time = 0.0;
    double operation_at_seconds = 0.0;
    int ego_lane_id;
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
  };

  struct LaneChangeContext {
    bool completed = true;
    bool trigger_when_appropriate = false;
    double trigger_time = 0.0;
    double desired_operation_time = 0.0;
    int ego_lane_id = 0;
    int potential_left_id = 0, potential_center_id = 0, potential_right_id = 0;
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
    LaneChangeTriggerType type;
  };

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
    std::vector<std::vector<CostStructure>> progress_cost;
    std::vector<CostStructure> tail_cost;
    std::vector<std::vector<Vehicle>> forward_trajs;
    std::vector<std::vector<LateralBehavior>> forward_lat_behaviors;
    std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors;
    std::vector<ReferencePoint> ref_lane;
    neodrive::planning::Boundary last_leading_boundary;

    double plan_stamp = 0.0;
    double time_cost = 0.0;
  };

  SimLateralManager() = default;
  SimLateralManager(
      const ReplanningContext& context, const Snapshot& last_snapshot,
      const Task& last_task, const LaneChangeContext& lc_context,
      const LaneChangeProposal& last_lc_proposal,
      const std::vector<ActivateLaneChangeRequest>& preliminary_active_requests)
      : context_(context),
        last_snapshot_(last_snapshot),
        last_task_(last_task),
        lc_context_(lc_context),
        last_lc_proposal_(last_lc_proposal),
        preliminary_active_requests_(preliminary_active_requests) {}

  // void init();

  bool run(const double stamp, const Task& task, const State& ego_state,
           ReplanningContext* context, Snapshot* last_snapshot, Task* last_task,
           LaneChangeContext* lc_context, LaneChangeProposal* last_lc_proposal,
           std::vector<ActivateLaneChangeRequest>* preliminary_active_requests);

  void reset();

  SimLateralPlanner& planner();

  int original_winner_id() const { return last_snapshot_.original_winner_id; }
  int processed_winner_id() const { return last_snapshot_.processed_winner_id; }

  std::unordered_map<std::string, int> getAllLaneID();

  //  private:
 public:
  double getNearestFutureDecisionPoint(const double stamp, const double delta);

  bool IsTriggerAppropriate(const LateralBehavior& lat);

  bool prepare(const double stamp, const Task& task, const State& ego_state);

  bool evaluateReferenceVelocity(const Task& task, double* ref_vel);

  bool getReplanDesiredAction(const double current_time,
                              DcpAction* desired_action);

  void saveSnapshot(const double stamp, Snapshot* snapshot);

  bool reselectByContext(const double stamp, const Snapshot& snapshot,
                         int* new_seq_id);

  void updateLaneChangeContextByTask(const double stamp, const Task& task);

  bool generateLaneChangeProposal(const double stamp, const Task& task);

 private:
  SimLateralPlanner bp_;
  SimMap* sim_map_{SimMap::Instance()};

  State ego_state_;
  double work_rate_{20.0};

  int ego_road_id_{-1};
  int ego_lane_id_{0};
  ReplanningContext context_;
  Snapshot last_snapshot_;
  Task last_task_;
  LaneChangeContext lc_context_;
  LaneChangeProposal last_lc_proposal_;
  std::vector<ActivateLaneChangeRequest> preliminary_active_requests_;
};

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive