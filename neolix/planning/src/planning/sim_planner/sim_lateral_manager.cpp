#include "sim_lateral_manager.h"

#include <ctime>

namespace neodrive {
namespace planning {
namespace sim_planner {

// run
bool SimLateralManager::run(
    const double stamp, const Task& task, const State& ego_state,
    ReplanningContext* context, Snapshot* last_snapshot, Task* last_task,
    LaneChangeContext* lc_context, LaneChangeProposal* last_lc_proposal,
    std::vector<ActivateLaneChangeRequest>* preliminary_active_requests) {
  bp_.init();
  bp_.setVehicleState(ego_state);
  bp_.set_last_leading_boundary(last_snapshot_.last_leading_boundary);
  auto start_p = clock();
  if (prepare(stamp, task, ego_state) != true) {
    LOG_ERROR("prepare failed!");
    return false;
  }
  auto end_p = clock();
  LOG_INFO("prepare time is: {}", end_p - start_p);

  auto start_r = clock();
  if (!bp_.runOnce()) {
    LOG_ERROR("runonce failed!");
    return false;
  }
  auto end_r = clock();
  LOG_INFO("runonce time is: {}", end_r - start_r);
  Snapshot snapshot;
  saveSnapshot(stamp, &snapshot);
  auto start_re = clock();
  if (!reselectByContext(stamp, snapshot, &snapshot.processed_winner_id)) {
    LOG_ERROR("reselect by context failed!");
    return false;
  }
  auto end_re = clock();
  LOG_INFO("Re select time is: {}", end_re - start_re);

  {
    std::ostringstream line_info;
    line_info << "[SimLateralManager][Output]Reselected <if_risky:"
              << snapshot.risky_res[snapshot.processed_winner_id] << ">[";
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
  auto start_g = clock();
  generateLaneChangeProposal(stamp, task);
  auto end_g = clock();
  LOG_INFO("generateLaneChangeProposal time is: {}", end_g - start_g);
  context_.is_valid = true;
  context_.seq_start_time = stamp;
  context_.action_seq = snapshot.action_script[snapshot.processed_winner_id];

  lc_context_.potential_center_id = bp_.potential_center_id();
  lc_context_.potential_left_id = bp_.potential_left_id();
  lc_context_.potential_right_id = bp_.potential_right_id();

  *context = context_;
  *last_snapshot = last_snapshot_;
  *last_task = last_task_;
  *lc_context = lc_context_;
  *last_lc_proposal = last_lc_proposal_;
  *preliminary_active_requests = preliminary_active_requests_;

  return true;
}

std::unordered_map<std::string, int> SimLateralManager::getAllLaneID() {
  std::unordered_map<std::string, int> all_lanes_id;
  all_lanes_id.insert(std::make_pair("left", bp_.potential_left_id()));
  all_lanes_id.insert(std::make_pair("center", bp_.potential_center_id()));
  all_lanes_id.insert(std::make_pair("right", bp_.potential_right_id()));
  return all_lanes_id;
}

bool SimLateralManager::prepare(const double stamp, const Task& task,
                                const State& ego_state) {
  ego_state_ = ego_state;

  DcpAction desired_action;
  if (!getReplanDesiredAction(stamp, &desired_action)) {
    desired_action.lat = DcpLatAction::kLaneKeeping;
    desired_action.lon = DcpLonAction::kMaintain;
    double fdp_stamp = getNearestFutureDecisionPoint(stamp, 0.0);
    desired_action.t = fdp_stamp - stamp;
  }
  if (!sim_map_->getRoadLaneId(ego_state_.vec_position, &ego_road_id_,
                               &ego_lane_id_)) {
    LOG_ERROR("getRoadLaneId failed.");
    return false;
  }
  LOG_INFO("ego_road_id {}, ego_lane_id {}", ego_road_id_, ego_lane_id_);
  updateLaneChangeContextByTask(stamp, task);
  if (lc_context_.completed) {
    desired_action.lat = DcpLatAction::kLaneKeeping;
  }

  {
    std::ostringstream line_info;
    line_info << "[SimLateralManager]Replan context <valid, stamp, seq>:<"
              << context_.is_valid << "," << std::fixed << std::setprecision(3)
              << context_.seq_start_time << ",";
    for (auto& a : context_.action_seq) {
      line_info << DcpTree::retLonActionName(a.lon);
    }
    line_info << "|";
    for (auto& a : context_.action_seq) {
      line_info << DcpTree::retLatActionName(a.lat);
    }
    line_info << ">";
    LOG_INFO("{}", line_info.str().c_str());
  }
  {
    std::ostringstream line_info;
    line_info << "[SimLateralManager]LC context <completed, twa, tt, dt, l_id, "
                 "lat, type>:<"
              << lc_context_.completed << ","
              << lc_context_.trigger_when_appropriate << "," << std::fixed
              << std::setprecision(3) << lc_context_.trigger_time << ","
              << lc_context_.desired_operation_time << ","
              << lc_context_.ego_lane_id << ">";
    LOG_INFO("{}", line_info.str().c_str());
  }
  bp_.updateDcpTree(desired_action);
  double ref_vel;
  evaluateReferenceVelocity(task, &ref_vel);

  LOG_INFO("[SimLateralManager]<task vel, ref_vel>: {:.3f}, {:.3f}",
           task.user_desired_vel, ref_vel);
  bp_.setDesiredVelocity(ref_vel);

  auto lc_info = task.lc_info;
  if (!lc_context_.completed) {
    if (stamp >= lc_context_.desired_operation_time) {
      if (lc_context_.lat == LateralBehavior::kLaneChangeLeft) {
        LOG_INFO("[HMI]Recommending left at {:.3f}, with desired time: {:.3f}",
                 stamp, lc_context_.desired_operation_time);
        lc_info.recommend_lc_left = true;
      } else if (lc_context_.lat == LateralBehavior::kLaneChangeRight) {
        lc_info.recommend_lc_right = true;
      }
    }
  }

  bp_.setLaneChangeInfo(lc_info);

  LOG_INFO("[SimLateralManager]desired <lon,lat,t>: {}, {}, {:.3f}",
           DcpTree::retLonActionName(desired_action.lon).c_str(),
           DcpTree::retLatActionName(desired_action.lat).c_str(),
           desired_action.t);

  return true;
}

bool SimLateralManager::getReplanDesiredAction(const double current_time,
                                               DcpAction* desired_action) {
  if (!context_.is_valid) return false;
  double time_since_last_plan =
      current_time -
      context_.seq_start_time;  // Relative time from the start of the
                                // previous planned action sequence
  if (time_since_last_plan < -kEPS) return false;
  double t_aggre = 0.0;
  bool find_match_action = false;
  int action_seq_len = context_.action_seq.size();
  for (int i = 0; i < action_seq_len; ++i) {
    t_aggre += context_.action_seq[i].t;
    if (time_since_last_plan + kEPS < t_aggre) {
      *desired_action = context_.action_seq[i];
      desired_action->t =
          t_aggre -
          time_since_last_plan;  // Record: The time this action has been
                                 // executed and will not be repeated
      find_match_action = true;
      break;
    }
  }
  if (!find_match_action) {
    return false;
  }
  return true;
}

double SimLateralManager::getNearestFutureDecisionPoint(const double stamp,
                                                        const double delta) {
  double past_decision_point =
      std::floor((stamp + delta) / bp_.cfg().sim_.duration_.layer) *
      bp_.cfg().sim_.duration_.layer;
  return past_decision_point + bp_.cfg().sim_.duration_.layer;
}

void SimLateralManager::updateLaneChangeContextByTask(const double stamp,
                                                      const Task& task) {
  if (!last_task_.is_under_ctrl && task.is_under_ctrl) {
    LOG_INFO("[HMI]Autonomous mode activated!");
    lc_context_.completed = true;
    lc_context_.trigger_when_appropriate = false;
    last_lc_proposal_.trigger_time = stamp;
  }

  if (task.is_under_ctrl) {
    if (!lc_context_.completed) {
      if ((lc_context_.ego_lane_id != ego_lane_id_) ||
          ((last_lc_proposal_.lat == LateralBehavior::kLaneChangeLeft) &&
           (ego_lane_id_ == lc_context_.potential_center_id) &&
           (lc_context_.potential_center_id ==
            lc_context_.potential_left_id)) ||
          ((last_lc_proposal_.lat == LateralBehavior::kLaneChangeRight) &&
           (ego_lane_id_ == lc_context_.potential_center_id) &&
           (lc_context_.potential_center_id ==
            lc_context_.potential_right_id))) {  // P:You should not compare
                                                 // the current lane ID with
        // the previous cycle lane ID, you should compare it
        // with the potential lane ID
        LOG_INFO(
            "[HMI]lane change completed due to different lane id {} to {} Cd "
            "alc.",
            lc_context_.ego_lane_id, ego_lane_id_);
        lc_context_.completed = true;
        lc_context_.trigger_when_appropriate = false;
        last_lc_proposal_.trigger_time = stamp;
      } else {
        if (lc_context_.type == LaneChangeTriggerType::kActive) {
          if (bp_.cfg()
                  .function_.active_lc_.enable_auto_cancel_by_outdate_time &&
              stamp > lc_context_.desired_operation_time +
                          bp_.cfg()
                              .function_.active_lc_
                              .auto_cancel_if_late_for_seconds) {
            if (lc_context_.lat == LateralBehavior::kLaneChangeLeft) {
              LOG_INFO(
                  "[HMI]ACTIVE [Left] auto cancel due to outdated for {:.3f} "
                  "s"
                  "Cd alc.",
                  stamp - lc_context_.desired_operation_time);
            } else {
              LOG_INFO(
                  "[HMI]ACTIVE [Right] auto cancel due to outdated for "
                  "{:.3f} "
                  "s. Cd alc.",
                  stamp - lc_context_.desired_operation_time);
            }
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          }
        }
      }
      if (lc_context_.trigger_when_appropriate) {
        if (lc_context_.lat == LateralBehavior::kLaneChangeLeft) {
          if (IsTriggerAppropriate(LateralBehavior::kLaneChangeLeft)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time = getNearestFutureDecisionPoint(
                stamp, bp_.cfg().function_.stick_lane_change_in_seconds);
            lc_context_.ego_lane_id = ego_lane_id_;
            lc_context_.lat = LateralBehavior::kLaneChangeLeft;
            lc_context_.type = LaneChangeTriggerType::kStick;
            last_lc_proposal_.trigger_time = stamp;
            LOG_INFO(
                "[HMI][[cached]] stick [Left] appropriate in {:.3f} s. "
                "Trigger "
                "time {:.3f} and absolute action time {:.3f}",
                bp_.cfg().function_.stick_lane_change_in_seconds,
                lc_context_.trigger_time, lc_context_.desired_operation_time);
          }
        } else if (lc_context_.lat == LateralBehavior::kLaneChangeRight) {
          if (IsTriggerAppropriate(LateralBehavior::kLaneChangeRight)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time = getNearestFutureDecisionPoint(
                stamp, bp_.cfg().function_.stick_lane_change_in_seconds);
            lc_context_.ego_lane_id = ego_lane_id_;
            lc_context_.lat = LateralBehavior::kLaneChangeRight;
            lc_context_.type = LaneChangeTriggerType::kStick;
            last_lc_proposal_.trigger_time = stamp;
            LOG_INFO(
                "[HMI][[cached]] stick [Right] triggered in {:.3f} s. "
                "Trigger "
                "time {:.3f} and absolute action time {:.3f}",
                bp_.cfg().function_.stick_lane_change_in_seconds,
                lc_context_.trigger_time, lc_context_.desired_operation_time);
          }
        }
      } else {
        if (last_lc_proposal_.valid &&
            (last_lc_proposal_.ego_lane_id == ego_lane_id_) &&
            stamp > last_lc_proposal_.trigger_time &&
            last_lc_proposal_.lat !=
                LateralBehavior::kLaneKeeping) {  // P:You should not compare
                                                  // the current lane ID with
                                                  // the previous cycle lane
                                                  // ID, you should compare it
                                                  // with the potential lane
                                                  // ID
          if ((last_lc_proposal_.lat == LateralBehavior::kLaneChangeLeft) ||
              (last_lc_proposal_.lat == LateralBehavior::kLaneChangeRight)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time =
                last_lc_proposal_.trigger_time +
                last_lc_proposal_.operation_at_seconds;
            lc_context_.ego_lane_id = last_lc_proposal_.ego_lane_id;
            lc_context_.lat = last_lc_proposal_.lat;
            lc_context_.type = LaneChangeTriggerType::kActive;
            last_lc_proposal_.trigger_time = stamp;
            if (last_lc_proposal_.lat == LateralBehavior::kLaneChangeLeft) {
              LOG_INFO(
                  "[HMI][[Active]] [Left] triggered in {:.3f} s. Trigger "
                  "time "
                  "{:.3f} and absolute action time {:.3f}",
                  last_lc_proposal_.operation_at_seconds,
                  lc_context_.trigger_time, lc_context_.desired_operation_time);
            } else {
              LOG_INFO(
                  "[HMI][[Active]] [Right] triggered in {:.3f} s. Trigger "
                  "time "
                  "{:.3f} and absolute action time {:.3f}",
                  last_lc_proposal_.operation_at_seconds,
                  lc_context_.trigger_time, lc_context_.desired_operation_time);
            }
          }
        }
      }
    }
  }
  // test
  lc_context_.completed = true;
  last_lc_proposal_.valid = false;
  last_task_ = task;
}

bool SimLateralManager::evaluateReferenceVelocity(const Task& task,
                                                  double* ref_vel) {
  if (last_snapshot_.ref_lane.empty()) {
    LOG_INFO(
        "[SimLateralManager]The first planning (the reference line of the "
        "previous planning cycle was empty)....");
    *ref_vel = task.user_desired_vel;
    return true;
  }
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

void SimLateralManager::saveSnapshot(const double stamp, Snapshot* snapshot) {
  snapshot->valid = true;
  snapshot->plan_state = bp_.plan_state();
  snapshot->original_winner_id = bp_.winner_id();
  snapshot->processed_winner_id = bp_.winner_id();
  snapshot->action_script = bp_.action_script();
  snapshot->sim_res = bp_.sim_res();
  snapshot->risky_res = bp_.risky_res();
  snapshot->sim_info = bp_.sim_info();
  snapshot->final_cost = bp_.final_cost();
  snapshot->progress_cost = bp_.progress_cost();
  // snapshot->tail_cost = bp_.tail_cost();
  snapshot->forward_trajs = bp_.forward_trajs();
  snapshot->forward_lat_behaviors = bp_.forward_lat_behaviors();
  snapshot->forward_lon_behaviors = bp_.forward_lon_behaviors();
  snapshot->last_leading_boundary = bp_.last_leading_boundary();

  snapshot->plan_stamp = stamp;
  snapshot->time_cost = bp_.time_cost();
}

bool SimLateralManager::reselectByContext(const double stamp,
                                          const Snapshot& snapshot,
                                          int* new_seq_id) {
  *new_seq_id = snapshot.original_winner_id;
  // int selected_seq_id;
  // int num_seqs = snapshot.action_script.size();
  // bool find_match = false;
  // double cost = kInf;

  // for (int i = 0; i < num_seqs; i++) {
  //   if (!snapshot.sim_res[i]) {
  //     continue;
  //   }
  //   LateralBehavior lat_behavior;
  //   double operation_at_seconds;
  //   bool is_cancel_behavior;
  //   bp_.classifyActionSeq(snapshot.action_script[i], &operation_at_seconds,
  //                         &lat_behavior, &is_cancel_behavior);
  //   find_match = true;
  //   if (snapshot.final_cost[i] < cost) {
  //     cost = snapshot.final_cost[i];
  //     selected_seq_id = i;
  //   }
  // }

  // if (!find_match) {
  //   LOG_INFO("do not find match!");
  //   return false;
  // }
  // *new_seq_id = selected_seq_id;
  return true;
}

bool SimLateralManager::generateLaneChangeProposal(const double stamp,
                                                   const Task& task) {
  if (!lc_context_.completed) {
    preliminary_active_requests_.clear();
    LOG_INFO(
        "[SimLateralManager][ActiveLc]Clear request due to not completed lc: "
        "{:.3f}",
        stamp);
    return true;
  }
  if (lc_context_.completed) {
    preliminary_active_requests_.clear();
    LOG_INFO(
        "[SimLateralManager][ActiveLc]Clear request due to stick not rest: "
        "{:.3f}",
        stamp);
    return true;
  }
  if (stamp - last_lc_proposal_.trigger_time < 0.0) {
    last_lc_proposal_.valid = false;
    last_lc_proposal_.trigger_time = stamp;
    last_lc_proposal_.ego_lane_id = ego_lane_id_;
    last_lc_proposal_.lat = LateralBehavior::kLaneKeeping;
    preliminary_active_requests_.clear();
    LOG_INFO(
        "[SimLateralManager][ActiveLc]Clear request due to illegal stamp: "
        "{:.3f}",
        stamp);
    return true;
  }
  if (stamp - last_lc_proposal_.trigger_time <
      bp_.cfg().function_.active_lc_.cold_duration) {
    preliminary_active_requests_.clear();
    LOG_INFO(
        "[SimLateralManager][ActiveLc]Clear request due to cold down: {:.3f} "
        "< "
        "{:.3f} + "
        "{:.3f}",
        stamp, last_lc_proposal_.trigger_time,
        bp_.cfg().function_.active_lc_.cold_duration);
    return true;
  }
  //  Lane changing speed requirements
  if (last_snapshot_.plan_state.velocity <
          bp_.cfg().function_.active_lc_.activate_speed_lower_bound ||
      last_snapshot_.plan_state.velocity >
          bp_.cfg().function_.active_lc_.activate_speed_upper_bound) {
    preliminary_active_requests_.clear();
    LOG_INFO(
        "[SimLateralManager][ActiveLc]Clear request due to illegal spd: "
        "{:.3f} "
        "at time "
        "{:.3f}",
        last_snapshot_.plan_state.velocity, stamp);
    return true;
  }

  LateralBehavior lat_behavior;
  double operation_at_seconds;
  bool is_cancel_behavior;
  bp_.classifyActionSeq(
      last_snapshot_.action_script[last_snapshot_.original_winner_id],
      &operation_at_seconds, &lat_behavior, &is_cancel_behavior);
  if (lat_behavior == LateralBehavior::kLaneKeeping || is_cancel_behavior) {
    LOG_INFO(
        "[SimLateralManager][ActiveLc]Clear request due to not ideal "
        "behavior: "
        "{:.3f}",
        stamp);
    preliminary_active_requests_.clear();
    return true;
  }

  ActivateLaneChangeRequest this_request;
  this_request.trigger_time = stamp;
  this_request.desired_operation_time = stamp + operation_at_seconds;
  this_request.ego_lane_id = ego_lane_id_;
  this_request.lat = lat_behavior;
  if (preliminary_active_requests_.empty()) {
    preliminary_active_requests_.emplace_back(this_request);
  } else {
    auto last_request = preliminary_active_requests_.back();
    if (last_request.ego_lane_id != this_request.ego_lane_id) {
      LOG_INFO(
          "[SimLateralManager][ActiveLc]Invalid this request due to lane id "
          "inconsitent.");
      preliminary_active_requests_.clear();
      return true;
    }
    if (last_request.lat != this_request.lat) {
      LOG_INFO(
          "[SimLateralManager][ActiveLc]Invalid this request due to behavior "
          "inconsitent.");
      preliminary_active_requests_.clear();
      return true;
    }
    if (fabs(last_request.desired_operation_time -
             this_request.desired_operation_time) >
        bp_.cfg().function_.active_lc_.consistent_operate_time_min_gap) {
      LOG_INFO(
          "[SimLateralManager][ActiveLc]Invalid this request due to time "
          "inconsitent.");
      preliminary_active_requests_.clear();
      return true;
    }
    preliminary_active_requests_.emplace_back(this_request);
    LOG_INFO(
        "[SimLateralManager][ActiveLc]valid this request. Queue size {} and "
        "operate at {}",
        preliminary_active_requests_.size(), operation_at_seconds);
  }

  if (preliminary_active_requests_.size() >=
      bp_.cfg().function_.active_lc_.consistent_min_num_frame) {
    if (operation_at_seconds <
        bp_.cfg().function_.active_lc_.activate_max_duration_in_seconds +
            kEPS) {
      last_lc_proposal_.valid = true;
      last_lc_proposal_.trigger_time = stamp;
      last_lc_proposal_.operation_at_seconds =
          operation_at_seconds >
                  bp_.cfg().function_.active_lc_.active_min_operation_in_seconds
              ? operation_at_seconds
              : getNearestFutureDecisionPoint(
                    stamp,
                    bp_.cfg()
                        .function_.active_lc_.active_min_operation_in_seconds) -
                    stamp;
      last_lc_proposal_.ego_lane_id = ego_lane_id_;
      last_lc_proposal_.lat = lat_behavior;
      preliminary_active_requests_.clear();
      LOG_INFO(
          "[HMI]Gen proposal with trigger time {:.3f} lane id {} behavior {} "
          "operate at {:.3f}",
          last_lc_proposal_.trigger_time, last_lc_proposal_.ego_lane_id,
          static_cast<int>(last_lc_proposal_.lat),
          last_lc_proposal_.operation_at_seconds);
    } else {
      preliminary_active_requests_.clear();
      LOG_INFO("[HMI]Abandan Queue due to change time not legal.");
    }
  }

  return true;
}

bool SimLateralManager::IsTriggerAppropriate(const LateralBehavior& lat) {
#if 1
  return true;
#endif
  // check whether appropriate to conduct lat behavior right now,
  // if not, recommend a velocity instead.
  if (!last_snapshot_.valid) return false;
  // KXXXX
  // KKXXX
  // KKKLL
  // KKKKL
  const int kMinActionCheckIdx = 1;
  const int kMaxActionCheckIdx = 3;
  const int kMinMatchSeqs = 3;
  int num_action_seqs = last_snapshot_.action_script.size();
  int num_match_seqs = 0;
  for (int i = 0; i < num_action_seqs; i++) {
    if (!last_snapshot_.sim_res[i] || last_snapshot_.risky_res[i]) continue;
    auto action_seq = last_snapshot_.action_script[i];
    int num_actions = action_seq.size();
    for (int j = kMinActionCheckIdx; j <= kMaxActionCheckIdx; j++) {
      if (lat == LateralBehavior::kLaneChangeLeft &&
          action_seq[j].lat == DcpLatAction::kLaneChangeLeft &&
          (action_seq[j].lon == DcpLonAction::kAccelerate ||
           action_seq[j].lon == DcpLonAction::kMaintain)) {
        num_match_seqs++;
        break;
      } else if (lat == LateralBehavior::kLaneChangeRight &&
                 action_seq[j].lat == DcpLatAction::kLaneChangeRight &&
                 (action_seq[j].lon == DcpLonAction::kAccelerate ||
                  action_seq[j].lon == DcpLonAction::kMaintain)) {
        num_match_seqs++;
        break;
      }
    }
  }
  if (num_match_seqs < kMinMatchSeqs) return false;
  return true;
}

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive