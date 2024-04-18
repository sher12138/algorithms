#include "sim_lateral_track_manager.h"

#include <ctime>

namespace neodrive {
namespace planning {
namespace sim_planner {

// run
bool SimLateralTrackManager::run(
    const double stamp, const Task& task, const State& ego_state,
    const std::vector<SimMapPoint>& track_pts,
    const std::unordered_map<std::string, int>& all_lanes_id,
    Snapshot* last_snapshot) {
  bp_.init();
  bp_.setVehicleState(ego_state);

  if (prepare(stamp, task, ego_state) != true) {
    LOG_ERROR("prepare failed!");
    return false;
  }

  if (!bp_.runOnce(track_pts, all_lanes_id)) {
    LOG_ERROR("runonce failed!");
    return false;
  }

  Snapshot snapshot;
  saveSnapshot(stamp, &snapshot);
  {
    std::ostringstream line_info;
    line_info << "[SimLateralManager][Output][";
    for (auto& a : snapshot.action_script[snapshot.processed_winner_id]) {
      line_info << DcpTree::retLonActionName(a.lon);
    }
    line_info << "|";
    for (auto& a : snapshot.action_script[snapshot.processed_winner_id]) {
      line_info << DcpTree::retLatActionName(a.lat);
    }
    line_info << "]";
    for (auto& v : snapshot.forward_trajs[snapshot.processed_winner_id]) {
      line_info << std::fixed << std::setprecision(5) << "<"
                << v.state().time_stamp - stamp << "," << v.state().velocity
                << "," << v.state().acceleration << "," << v.state().curvature
                << ">";
    }
    LOG_INFO("{}", line_info.str().c_str());
  }

  snapshot.ref_lane = sim_map_->getRefPoints();
  last_snapshot_ = snapshot;
  generateLaneChangeProposal(stamp, task);
  *last_snapshot = last_snapshot_;
  return true;
}

void SimLateralTrackManager::saveSnapshot(const double stamp,
                                          Snapshot* snapshot) {
  snapshot->valid = true;
  snapshot->plan_state = bp_.plan_state();
  snapshot->original_winner_id = bp_.winner_id();
  snapshot->processed_winner_id = bp_.winner_id();
  snapshot->action_script = bp_.action_script();
  snapshot->sim_res = bp_.sim_res();
  snapshot->risky_res = bp_.risky_res();
  snapshot->sim_info = bp_.sim_info();
  snapshot->progress_cost = bp_.progress_cost();
  snapshot->forward_trajs = bp_.forward_trajs();
  snapshot->forward_lat_behaviors = bp_.forward_lat_behaviors();
  snapshot->forward_lon_behaviors = bp_.forward_lon_behaviors();

  snapshot->plan_stamp = stamp;
}

bool SimLateralTrackManager::prepare(const double stamp, const Task& task,
                                     const State& ego_state) {
  ego_state_ = ego_state;

  DcpAction desired_action;

  desired_action.lat = DcpLatAction::kLaneKeeping;
  desired_action.lon = DcpLonAction::kAccelerate;
  desired_action.t = 1.0;
  if (!sim_map_->getRoadLaneId(ego_state_.vec_position, &ego_road_id_,
                               &ego_lane_id_)) {
    LOG_ERROR("getRoadLaneId failed.");
    return false;
  }

  bp_.updateDcpTree(desired_action);
  double ref_vel;
  evaluateReferenceVelocity(task, &ref_vel);

  LOG_INFO("[SimLateralManager]<task vel, ref_vel>: {:.3f}, {:.3f}",
           task.user_desired_vel, ref_vel);
  bp_.setDesiredVelocity(ref_vel);

  LOG_INFO("[SimLateralManager]desired <lon,lat,t>: {}, {}, {:.3f}",
           DcpTree::retLonActionName(desired_action.lon).c_str(),
           DcpTree::retLatActionName(desired_action.lat).c_str(),
           desired_action.t);

  return true;
}

bool SimLateralTrackManager::evaluateReferenceVelocity(const Task& task,
                                                       double* ref_vel) {
  SimMapPoint current_fs;
  if (!sim_map_->getNearestPoint(last_snapshot_.plan_state.vec_position,
                                 &current_fs)) {
    LOG_INFO("get nearest point failed!");
    return false;
  }

  double c;
  double v_max_by_curvature;
  double v_ref = kInf;

  double a_comfort = bp_.cfg().sim_.ego_.ego_lon_.ego_lon_limit_.soft_brake;
  double t_forward = last_snapshot_.plan_state.velocity / a_comfort;
  double current_fs_s = current_fs.s;
  for (const auto& P : last_snapshot_.ref_lane) {
    if (P.s() < current_fs_s) {
      continue;
    }
    if (P.kappa() != 0.0) {
      c = P.kappa();
      v_max_by_curvature =
          sqrt(bp_.cfg().sim_.ego_.ego_lat_.ego_lat_limit_.acc / fabs(c));
      v_ref = v_max_by_curvature < v_ref ? v_max_by_curvature : v_ref;
    }
  }

  *ref_vel = std::floor(std::min(std::max(v_ref, 0.0), task.user_desired_vel));
  LOG_INFO(
      "[SimLateralManager][Desired]User ref vel: {:.3f}, final ref vel: "
      "{:.3f}",
      task.user_desired_vel, *ref_vel);

  return true;
}
}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive