#include "sim_conflict_decider.h"

#include "src/planning/math/curve1d/spline.h"

namespace neodrive {
namespace planning {
bool SimConflictDecider::Init(TaskInfo& task_info) {
  double width = VehicleParam::Instance()->width();
  double length = VehicleParam::Instance()->length();
  double adc_back_edge_to_center =
      VehicleParam::Instance()->back_edge_to_center();
  SLPoint adc_current_sl = task_info.curr_sl();
  double adc_front_edge_to_center = length - adc_back_edge_to_center;
  double adc_current_v = task_info.current_frame()->inside_planner_data().vel_v;
  double adc_current_a = task_info.current_frame()->inside_planner_data().vel_a;
  std::vector<double> suspention_ego{adc_front_edge_to_center,
                                     adc_back_edge_to_center, width / 2.0,
                                     width / 2.0};
  std::vector<double> param_ego{2.0, 0.5, 3.0};
  ego_.set_param(suspention_ego, 0.2, param_ego);
  ego_.sl_position = Vec2d(adc_current_sl.s(), adc_current_sl.l());
  ego_.sl_velocity = Vec2d(adc_current_v, 0.0);
  ego_.sl_accel = Vec2d(adc_current_a, 0.0);
  ego_.heading = task_info.current_frame()->inside_planner_data().vel_heading;
  motorway_speed_back_cipv_decider_config_ptr_ =
      &config::PlanningConfig::Instance()
           ->planning_research_config()
           .speed_vehicle_back_cipv_decider_config;
  if (nullptr == motorway_speed_back_cipv_decider_config_ptr_) {
    LOG_ERROR("get config failed.");
    return false;
  }
  if (!CalInputData(task_info)) {
    return false;
  }

  /// compute all conflict infos and combine:vector<Connect...>
  std::vector<ConnectionConflictInfo> all_conflict_infos{};
  if (!CalcConflictArea(task_info, all_conflict_infos)) {
    LOG_INFO("Calc conflict area failed!");
    return false;
  }

  process_data_.clear();
  process_data_ = std::move(CombineConflictInfos(all_conflict_infos));

  SplitFRAreaBound();
  if (sigment_fr_bounds_.empty()) {
    LOG_INFO("Finish Last Merging Area!");
    return false;
  }

  return true;
}

bool SimConflictDecider::CalInputData(TaskInfo& task_info) {
  auto outside_data_ptr =
      task_info.current_frame()->mutable_outside_planner_data();
  MotorwaySequenceDecisionData upstream_decision{
      .source = MotorwaySequenceDecisionType::NONE};
  for (const auto& decision :
       outside_data_ptr->motorway_speed_context.goal_decision) {
    if (decision.source == MotorwaySequenceDecisionType::MULTI_CIPV) {
      upstream_decision = decision;
      upstream_decision.source = MotorwaySequenceDecisionType::MULTI_CIPV;
      break;
    }
  }
  if (upstream_decision.source != MotorwaySequenceDecisionType::MULTI_CIPV) {
    LOG_ERROR("Upstream Limit is Empty!");
    return false;
  }
  std::vector<double> p_seq{}, v_seq{}, a_seq{}, t_seq{};

  if (upstream_decision.goal_s.size() < 4) {
    LOG_ERROR("Cannot perform differential compensation!");
    return false;
  }
  input_iter_data_.Reset();
  for (size_t i = 0; i < upstream_decision.goal_s.size(); i++) {
    p_seq.emplace_back(upstream_decision.goal_s[i].s());
    v_seq.emplace_back(upstream_decision.goal_v[i].s());
    a_seq.emplace_back(upstream_decision.goal_a[i].s());
    t_seq.emplace_back(upstream_decision.goal_s[i].t());
  }
  tk::spline goal_p_spline, goal_v_spline, goal_a_spline;
  goal_p_spline.set_points(t_seq, p_seq, tk::spline::spline_type::linear);
  goal_v_spline.set_points(t_seq, v_seq, tk::spline::spline_type::linear);
  goal_a_spline.set_points(t_seq, a_seq, tk::spline::spline_type::linear);
  for (size_t i = 0; i < 81; ++i) {
    double curr_t = i * time_step_;
    input_iter_data_.deduction_ego_p_sequence.emplace_back(
        goal_p_spline(curr_t));
    input_iter_data_.deduction_ego_v_sequence.emplace_back(
        goal_v_spline(curr_t));
    input_iter_data_.deduction_ego_a_sequence.emplace_back(
        goal_a_spline(curr_t));
    input_iter_data_.deduction_ego_t_sequence.emplace_back(curr_t);
  }
  return true;
}

std::vector<std::vector<ConnectionConflictInfo>>
SimConflictDecider::CombineConflictInfos(
    std::vector<ConnectionConflictInfo>& conflict_infos) {
  std::vector<std::vector<ConnectionConflictInfo>> ans{};

  std::sort(conflict_infos.begin(), conflict_infos.end(),
            [](const auto& a, const auto& b) {
              return a.conflict_area_bound[3] < b.conflict_area_bound[3];
            });
  int start = 0;
  if (conflict_infos.empty()) {
    ConnectionConflictInfo c;
    c.lane_id = 1;
    std::vector<double> cab{1e+8, 1e+8, 1e+8, 1e+8};
    c.conflict_area_bound = cab;
    std::vector<double> suspension{100.0, 100.0, 100.0, 100.0};
    std::vector<double> param_limit{3.0, 1.0, 3.0};
    c.agent = VehicleInfo(0, Vec2d(0.0, 0.0), Vec2d(0.0, 0.0), Vec2d(0.0, 0.0),
                          Vec2d(0.0, 0.0), 0.0, suspension, 1.0, param_limit);
    conflict_infos.emplace_back(c);
  }
  for (int i = 1; i <= conflict_infos.size(); ++i) {
    if (i == conflict_infos.size() ||
        conflict_infos[i].conflict_area_bound[3] -
                conflict_infos[i - 1].conflict_area_bound[3] >
            motorway_speed_back_cipv_decider_config_ptr_
                ->conflict_area_integration_gap) {
      std::vector<ConnectionConflictInfo> sub_set(
          conflict_infos.begin() + start, conflict_infos.begin() + i);
      ans.emplace_back(sub_set);
      start = i;
    }
  }
  LOG_INFO("Cluster Out {} conflict Areas!", ans.size());

  return ans;
}

bool SimConflictDecider::SplitFRAreaBound() {
  sigment_fr_bounds_.clear();
  for (const auto& c : process_data_) {
    ConnectionConflictInfo front_bound, rear_bound;
    front_bound = c.front();
    rear_bound = c.back();
    if (rear_bound.conflict_area_bound[3] <= 0.0) {
      continue;
    }
    std::vector<double> sigment_fr_bound{front_bound.conflict_area_bound[3],
                                         rear_bound.conflict_area_bound[3],
                                         rear_bound.conflict_area_bound[2]};
    sigment_fr_bounds_.emplace_back(sigment_fr_bound);
  }
  if (sigment_fr_bounds_.empty()) {
    LOG_WARN("Bound Sigment is Empty!");
    ConnectionConflictInfo front_bound, rear_bound;
    front_bound = (process_data_.back()).front();
    rear_bound = (process_data_.back()).back();
    if (rear_bound.conflict_area_bound[2] > 0.0) {
      std::vector<double> sigment_fr_bound{front_bound.conflict_area_bound[3],
                                           rear_bound.conflict_area_bound[3],
                                           rear_bound.conflict_area_bound[2]};
      sigment_fr_bounds_.emplace_back(sigment_fr_bound);
    }
  }
  LOG_INFO("FR Area Bound Size is {}", sigment_fr_bounds_.size());
  return true;
}

bool SimConflictDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute sim_conflict_decider");

  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return false;
  }

  task_info.current_frame()
      ->mutable_outside_planner_data()
      ->motorway_speed_context.trigger_backcipv = false;
  if (!CalPostProcessDataForSkip(task_info)) {
    return false;
  }
  if (!Init(task_info)) {
    SubmitSpeedDecision(task_info);
    LOG_WARN("Skip Merging Area Speed Optimizer!");
    return true;
  }
  if (!CalInteractiveAgent(task_info)) {
    SubmitSpeedDecision(task_info);
    LOG_WARN("Skip Merging Area Speed Optimizer!");
    return true;
  }
  expected_state_.Reset();
  bool rush_failed = false, execute_yield = false;

  if (risk_prob_ >= 0.0) {
    // rush
    LOG_INFO("Start Rush Model.....");
    conflict_area_bound_ = conflict_area_rush_bound_;
    if (conflict_area_rush_bound_.empty()) {
      LOG_INFO("Do not should rush, skip!");
      return true;
    }
    LOG_INFO("Conflict Area Rush Bound: {:3f}, {:3f}, {:3f}, {:3f}",
             conflict_area_rush_bound_[0], conflict_area_rush_bound_[1],
             conflict_area_rush_bound_[2], conflict_area_rush_bound_[3]);
    if (!SetRushLimit(task_info)) {
      SubmitSpeedDecision(task_info);
      LOG_ERROR("Set Rush Limit Failed!");
      return true;
    }
    ConflictAreaRushOptModel rush_model(ego_, key_conflict_info_,
                                        conflict_area_bound_);

    if (!rush_model.Init(0.3, 0.0, key_conflict_info_.param.rush_time,
                         &special_t_)) {
      LOG_ERROR("Rush Model Init Failed!");
      rush_failed = true;
    }
    // determine redecision
    if (!IsRedecision(conflict_area_rush_bound_[2], special_t_)) {
      SubmitSpeedDecision(task_info);
      LOG_INFO("Do not need redecision, skip!");
      return true;
    }
    rush_model.set_speed_limit(upstream_speed_limit_);
    rush_model.set_s_limit(upstream_s_limit_);
    rush_model.set_acc_limit(upstream_a_limit_);
    rush_model.set_min_distance(key_conflict_info_.param.rush_margin_distance);

    if (!rush_model.Planner(
            ego_.sl_velocity.x() + motorway_speed_back_cipv_decider_config_ptr_
                                       ->rush_velocity_increment,
            ego_.sl_velocity.x())) {
      LOG_ERROR("Rush Model Unsolvable!");
      rush_failed = true;
    }
    if (!rush_failed) {
      expected_state_.is_rush_decision = true;
      expected_state_.deduction_ego_p_sequence = rush_model.get_expected_s();
      expected_state_.deduction_ego_v_sequence = rush_model.get_expected_v();
      expected_state_.deduction_ego_a_sequence = rush_model.get_expected_a();
      expected_state_.deduction_ego_t_sequence = rush_model.get_expected_t();
    }
  }
  if (risk_prob_ < 0.0 || rush_failed) {
    // yield double check
    conflict_area_bound_ = conflict_area_yield_bound_;
    LOG_INFO("Conflict Area Yield Bound: {:3f}, {:3f}, {:3f}, {:3f}",
             conflict_area_yield_bound_[0], conflict_area_yield_bound_[1],
             conflict_area_yield_bound_[2], conflict_area_yield_bound_[3]);
    if (rush_failed && conflict_area_bound_[3] >= 0.0) {
      LOG_INFO("BP: Start Yield Model.....");
    } else if (conflict_area_bound_[3] >= 0.0) {
      LOG_INFO("Start Yield Model.....");
    } else {
      SubmitSpeedDecision(task_info);
      LOG_ERROR("Yield Failed in Conflict Area!");
      return true;
    }
    if (!SetYieldLimit(task_info)) {
      SubmitSpeedDecision(task_info);
      LOG_ERROR("Set Yield Limit Failed!");
      return true;
    }
    ConflictAreaYieldOptModel yield_model(ego_, key_conflict_info_.agent,
                                          conflict_area_bound_);

    if (!yield_model.Init(0.3, 0.0, key_conflict_info_.param.yield_time,
                          &special_t_)) {
      SubmitSpeedDecision(task_info);
      LOG_ERROR("Yield Model Init Failed, Decision Error!");
      return true;
    }
    yield_model.set_speed_limit(upstream_speed_limit_);
    yield_model.set_s_limit(upstream_s_limit_);
    yield_model.set_acc_limit(upstream_a_limit_);
    double velocity_upper =
        time_step_ * std::floor(ego_.sl_velocity.x() / time_step_);
    if (conflict_area_bound_[3] >
        key_conflict_info_.param.yield_advance_response_distance) {
      velocity_upper = 0.5;
    }
    if (!yield_model.Planner(velocity_upper)) {
      SubmitSpeedDecision(task_info);
      LOG_ERROR("Yield Model Unsolvable!");
      return true;
    }
    expected_state_.is_rush_decision = false;
    execute_yield = true;
    expected_state_.deduction_ego_p_sequence = yield_model.get_expected_s();
    expected_state_.deduction_ego_v_sequence = yield_model.get_expected_v();
    expected_state_.deduction_ego_a_sequence = yield_model.get_expected_a();
    expected_state_.deduction_ego_t_sequence = yield_model.get_expected_t();
  }

  std::vector<double> s_set, v_set, a_set;
  if (!CalPostProcessData(task_info, expected_state_, execute_yield, &s_set,
                          &v_set, &a_set)) {
    SubmitSpeedDecision(task_info);
    LOG_ERROR("Calculate Post Process Data Failed!");
    return true;
  }
  return true;
}

bool SimConflictDecider::CalPostProcessDataForSkip(TaskInfo& task_info) {
  auto outside_data_ptr =
      task_info.current_frame()->mutable_outside_planner_data();
  for (const auto& decision :
       outside_data_ptr->motorway_speed_context.goal_decision) {
    if (decision.source == MotorwaySequenceDecisionType::MULTI_CIPV) {
      goal_decision_ = decision;
      goal_decision_.source = MotorwaySequenceDecisionType::BACK_CIPV;
      break;
    }
  }
  if (goal_decision_.source != MotorwaySequenceDecisionType::BACK_CIPV) {
    LOG_ERROR("Get MULTI_CIPV Data Failed!");
    return false;
  }
  return true;
}

void SimConflictDecider::SubmitSpeedDecision(TaskInfo& task_info) {
  task_info.current_frame()
      ->mutable_outside_planner_data()
      ->motorway_speed_context.goal_decision.emplace_back(
          std::move(goal_decision_));
}

bool SimConflictDecider::CalInteractiveAgent(TaskInfo& task_info) {
  uint64_t sig_fr_size = sigment_fr_bounds_.size();
  uint64_t connect_areas_size = process_data_.size();
  std::vector<ConnectionConflictInfo> pending_interactive_cases =
      process_data_[connect_areas_size - sig_fr_size];
  std::vector<double> pending_sig_bounds = sigment_fr_bounds_.front();
  double rush_prob_diff = 0.0, yield_prob_diff = 1.0;
  bool is_skip = false;
  for (const auto& c : pending_interactive_cases) {
    LOG_INFO("the obstacle id being processed is {}", c.agent.id);
    double prob_tmp = 0.0;
    std::vector<double> conflict_area_bound = c.conflict_area_bound;
    // Skip if there is no interactive agent or if the interactive agent has
    // passed through the conflict area
    LOG_INFO("conflict_area_bound[0]: {}", conflict_area_bound[0]);
    if (conflict_area_bound[0] > 100.0 || conflict_area_bound[0] <= 0.0) {
      is_skip = true;
      continue;
    }
    if (!CalRushProbByGameTheory(c, pending_sig_bounds, &prob_tmp)) {
      LOG_ERROR("Calculate Rush Prob By Game Theory Failed!");
      return false;
    }
    std::vector<double> cab_yield{conflict_area_bound[0],
                                  conflict_area_bound[1], pending_sig_bounds[2],
                                  pending_sig_bounds[0]};
    std::vector<double> cab_rush{conflict_area_bound[0], conflict_area_bound[1],
                                 pending_sig_bounds[2], pending_sig_bounds[1]};
    LOG_INFO(
        "prob_tmp: {}, yield_prob_diff: {}, rush_prob_diff: {}, "
        "pending_sig_bounds[0]: {}, conflict_area_bound[0]: {}",
        prob_tmp, yield_prob_diff, rush_prob_diff, pending_sig_bounds[0],
        conflict_area_bound[0]);
    if (rush_prob_diff > prob_tmp && pending_sig_bounds[0] > 0.0) {
      is_skip = false;
      rush_prob_diff = prob_tmp;
      key_conflict_info_ = c;
      conflict_area_yield_bound_ = cab_yield;
    } else if (prob_tmp >= 0.0 && yield_prob_diff > prob_tmp &&
               conflict_area_bound[0] > 0.0 &&
               conflict_area_bound[0] < 90.0) {  // ego rush
      is_skip = false;
      yield_prob_diff = prob_tmp;
      key_conflict_info_ = c;
      conflict_area_rush_bound_ = cab_rush;
      conflict_area_yield_bound_ = cab_yield;
    } else if (conflict_area_bound[0] <= 0.0 && pending_sig_bounds[0] <= 0.0) {
      is_skip = true;
    }
  }
  if (is_skip || conflict_area_yield_bound_.empty()) {
    LOG_INFO("Do not handle interaction, skip!");
    return false;
  }
  LOG_INFO("Rush Prob Diff: {:3f}, Yield Prob Diff: {:3f}", rush_prob_diff,
           yield_prob_diff);
  risk_prob_ = (rush_prob_diff >= 0.0) ? yield_prob_diff : rush_prob_diff;
  if (pending_sig_bounds[0] <= 0.0) {  // todo:pending_sig_bounds[1] > 0.0
    // must rush
    risk_prob_ = 1.0;
  }
  return true;
}

bool SimConflictDecider::SetRushLimit(TaskInfo& task_info) {
  const auto& tunnel_decision = task_info.current_frame()
                                    ->mutable_outside_planner_data()
                                    ->motorway_speed_context.tunnel_decision;
  uint64_t size_s_bound = tunnel_decision.upper_s_tunnel.size();
  if (size_s_bound == 0) {
    LOG_INFO("Missing Rush Upper Bound!");
    return false;
  }
  upstream_speed_limit_.clear();
  upstream_s_limit_.clear();
  upstream_a_limit_.clear();
  for (int i = 0; i < size_s_bound; ++i) {
    upstream_speed_limit_.emplace_back(tunnel_decision.upper_v_tunnel[i].s());
    upstream_s_limit_.emplace_back(tunnel_decision.upper_s_tunnel[i].s());
    upstream_a_limit_.emplace_back(10.0);
  }
  if (!upstream_speed_limit_.empty() && !upstream_s_limit_.empty() &&
      upstream_speed_limit_.size() == upstream_s_limit_.size()) {
    return true;
  }
  LOG_ERROR("Upstream Limit is Empty!");
  return false;
}

bool SimConflictDecider::IsRedecision(const double rear_bound,
                                      const double constraints_time) {
  const auto& init_s_seq = goal_decision_.goal_s;
  for (int i = 0; i < init_s_seq.size(); ++i) {
    // rear_bound is distal end
    if (init_s_seq[i].s() >= rear_bound) {
      if (init_s_seq[i].t() <= constraints_time) {
        return false;  // do not need redecision
      }
      return true;
    }
  }
  return true;
}

bool SimConflictDecider::SetYieldLimit(TaskInfo& task_info) {
  if (input_iter_data_.deduction_ego_t_sequence.size() != 81) {
    LOG_ERROR("yield limit size do not match 81, is {}",
              input_iter_data_.deduction_ego_t_sequence.size());
    return false;
  }
  upstream_speed_limit_.clear();
  upstream_s_limit_.clear();
  upstream_a_limit_.clear();
  for (int i = 0; i < input_iter_data_.deduction_ego_t_sequence.size(); ++i) {
    upstream_speed_limit_.emplace_back(
        input_iter_data_.deduction_ego_v_sequence[i]);
    upstream_s_limit_.emplace_back(
        input_iter_data_.deduction_ego_p_sequence[i]);
    upstream_a_limit_.emplace_back(
        input_iter_data_.deduction_ego_a_sequence[i]);
  }
  if (!upstream_speed_limit_.empty() && !upstream_s_limit_.empty() &&
      upstream_speed_limit_.size() == upstream_s_limit_.size()) {
    return true;
  }
  LOG_ERROR("Upstream Limit is Empty!");
  return false;
}

bool SimConflictDecider::CalPostProcessData(
    TaskInfo& task_info, const MergeAreaEgoStateSequence& pre_data,
    bool is_yield, std::vector<double>* s_set, std::vector<double>* v_set,
    std::vector<double>* a_set) {
  std::vector<double> s_set_, v_set_, a_set_;
  uint64_t size_pre_data = pre_data.deduction_ego_p_sequence.size(),
           size_post_data = 81;
  MotorwaySequenceDecisionData goal_decision{
      .source = MotorwaySequenceDecisionType::BACK_CIPV};
  if (size_pre_data == 0) {
    LOG_INFO("Pre Data Size is {:3f}", size_pre_data);
    return false;
  }
  // tunnel_decision
  auto& tunnel_decision = task_info.current_frame()
                              ->mutable_outside_planner_data()
                              ->motorway_speed_context.tunnel_decision;
  tunnel_decision.Reset();
  const double max_limit_speed =
      DataCenter::Instance()->drive_strategy_max_speed();
  if (is_yield) {
    const auto& constraints_s_seq = input_iter_data_.deduction_ego_p_sequence;
    for (int i = 0; i < size_post_data; ++i) {
      double s_upper_bound = std::max(conflict_area_yield_bound_[3], 1e-3),
             s_lower_bound = 0.0;
      double ego_s = constraints_s_seq[i];
      s_upper_bound = std::min(s_upper_bound, ego_s);
      tunnel_decision.upper_s_tunnel.emplace_back(
          STPoint(s_upper_bound + .3, i * time_step_));
      tunnel_decision.lower_s_tunnel.emplace_back(
          STPoint(s_lower_bound - .1, i * time_step_));
      tunnel_decision.upper_v_tunnel.emplace_back(
          STPoint(max_limit_speed, i * time_step_));
      tunnel_decision.lower_v_tunnel.emplace_back(STPoint(.0, i * time_step_));
    }
  }
  for (int i = 0; i < size_post_data; ++i) {
    STPoint st_point{}, vt_point{}, at_point{};
    if (i < size_pre_data) {
      st_point.set_s(expected_state_.deduction_ego_p_sequence[i]);
      st_point.set_t(expected_state_.deduction_ego_t_sequence[i]);
      vt_point.set_s(expected_state_.deduction_ego_v_sequence[i]);
      vt_point.set_t(expected_state_.deduction_ego_t_sequence[i]);
      at_point.set_s(expected_state_.deduction_ego_a_sequence[i]);
      at_point.set_t(expected_state_.deduction_ego_t_sequence[i]);
      goal_decision.goal_s.emplace_back(st_point);
      goal_decision.goal_v.emplace_back(vt_point);
      goal_decision.goal_a.emplace_back(at_point);
      s_set_.emplace_back(expected_state_.deduction_ego_p_sequence[i]);
      v_set_.emplace_back(expected_state_.deduction_ego_v_sequence[i]);
      a_set_.emplace_back(expected_state_.deduction_ego_a_sequence[i]);
    } else {
      double a = a_set_.back();
      // double a = 1e-3;
      double v = v_set_.back() + a * time_step_;
      double s = s_set_.back();
      if (v > 0.0) {
        s += 0.5 * (v_set_.back() + v) * time_step_;
      } else {
        v = 0.0;
        s += (-std::pow(v_set_.back(), 2)) / (2.0 * a);
      }
      st_point.set_s(s);
      st_point.set_t(i * time_step_);
      vt_point.set_s(v);
      vt_point.set_t(i * time_step_);
      at_point.set_s(a);
      at_point.set_t(i * time_step_);
      goal_decision.goal_s.emplace_back(st_point);
      goal_decision.goal_v.emplace_back(vt_point);
      goal_decision.goal_a.emplace_back(at_point);
      s_set_.emplace_back(s);
      v_set_.emplace_back(v);
      a_set_.emplace_back(a);
    }
  }

  // tunnel_decision
  if (!is_yield) {
    for (int i = 0; i < size_post_data; ++i) {
      double s_upper_bound = 80.0, s_lower_bound = 0.0;
      double ego_s = s_set_[i];
      s_upper_bound = std::min(s_upper_bound, ego_s);
      tunnel_decision.upper_s_tunnel.emplace_back(
          STPoint(s_upper_bound + .3, i * time_step_));
      tunnel_decision.lower_s_tunnel.emplace_back(
          STPoint(s_lower_bound - .1, i * time_step_));
      tunnel_decision.upper_v_tunnel.emplace_back(
          STPoint(max_limit_speed, i * time_step_));
      tunnel_decision.lower_v_tunnel.emplace_back(STPoint(.0, i * time_step_));
    }
  }
  task_info.current_frame()
      ->mutable_outside_planner_data()
      ->motorway_speed_context.trigger_backcipv = true;
  task_info.current_frame()
      ->mutable_outside_planner_data()
      ->motorway_speed_context.conflict_brake = std::max(-5.0, a_set_.back());
  task_info.current_frame()
      ->mutable_outside_planner_data()
      ->motorway_speed_context.goal_decision.emplace_back(
          std::move(goal_decision));
  *s_set = s_set_;
  *v_set = v_set_;
  *a_set = a_set_;
  return true;
}

bool SimConflictDecider::CalRushProbByGameTheory(
    const ConnectionConflictInfo& c, const std::vector<double>& sig_bounds,
    double* rush_prob_diff) {
  VehicleInfo agent = c.agent;
  std::vector<double> conflict_area_bound = c.conflict_area_bound;
  double l = conflict_area_bound[3], l_ = conflict_area_bound[0],
         l_c_agent = conflict_area_bound[0], l_c_ego = sig_bounds[2];
  l = (l - ego_.front_suspension <= 0.0) ? 1e-5 : (l - ego_.front_suspension);
  l_ = (l_ - agent.front_suspension <= 0.0) ? 1e-5
                                            : (l_ - agent.front_suspension);
  double ttc_ego = std::max(l, 1.0) / std::max(ego_.sl_velocity.x(), 1e-3),
         ttc_agent = l_ / std::max(agent.sl_velocity.x(), 1e-3);
  double t_ego = (l_c_ego + ego_.rear_suspension) /
                 std::max(ego_.sl_velocity.x(), 1e-3),
         t_agent = (l_c_agent + agent.rear_suspension) /
                   std::max(agent.sl_velocity.x(), 1e-3);
  double ego_rush_and_agent_rush_gains =
      -1.0 / ttc_ego - c.param.param_a * c.param.param_c * t_ego;
  double ego_rush_and_agent_yield_gains =
      1.0 / ttc_ego + c.param.param_a * t_ego;
  double ego_yield_and_agent_rush_gains =
      1.0 / ttc_ego - c.param.param_a * t_ego;
  double ego_yield_and_agent_yield_gains =
      1.0 / ttc_ego - c.param.param_a * t_ego;

  double agent_rush_and_ego_rush_gains =
      -1.0 / ttc_agent - c.param.param_a * c.param.param_c * t_agent;
  double agent_rush_and_ego_yield_gains =
      1.0 / ttc_agent + c.param.param_a * t_agent;
  double agent_yield_and_ego_rush_gains =
      1.0 / ttc_agent - c.param.param_a * t_agent;
  double agent_yield_and_ego_yield_gains =
      1.0 / ttc_agent - c.param.param_a * t_agent;

  double agent_rush_prob =
      (ego_rush_and_agent_yield_gains - ego_yield_and_agent_yield_gains) /
      (ego_rush_and_agent_yield_gains + ego_yield_and_agent_rush_gains -
       ego_rush_and_agent_rush_gains - ego_yield_and_agent_yield_gains);
  double ego_rush_prob =
      (agent_rush_and_ego_yield_gains - agent_yield_and_ego_yield_gains) /
      (agent_rush_and_ego_yield_gains + agent_yield_and_ego_rush_gains -
       agent_rush_and_ego_rush_gains - agent_yield_and_ego_yield_gains);
  if (agent_rush_prob < 0.0 || ego_rush_prob < 0.0) {
    LOG_ERROR("Game Theory Failed!");
    return false;
  }
  LOG_INFO("Ego Rush Prob: {:3f}, Agent Rush Prob: {:3f}", ego_rush_prob,
           agent_rush_prob);
  *rush_prob_diff =
      std::max(-1.0, ego_rush_prob - agent_rush_prob -
                         c.param.param_margin);  // del ego's rush probability
  return true;
}

bool SimConflictDecider::CalcConflictArea(
    TaskInfo& task_info, std::vector<ConnectionConflictInfo>& conflict_info) {
  auto CalcConflictAreaBaseMap =
      [](TaskInfo& task_info, const IntegrativeConflictInfo& conflict_agent)
      -> std::vector<ConnectionConflictInfo> {
    ConflictDataBaseMap cdbm;
    std::unordered_set<int> agent_ids = {conflict_agent.id};

    cdbm.set_interactive_agent_ids(
        agent_ids, neodrive::planning::MotorwayInteractiveType::NONE);
    auto ans = cdbm.ComputeConflictMeetingData(task_info);
    if (!ans.empty()) {
      ans[0].param.param_margin =
          ans[0].param.param_margin * conflict_agent.style;
    }

    cdbm.set_interactive_agent_ids(
        agent_ids, neodrive::planning::MotorwayInteractiveType::NONE);
    auto ans_mergein = cdbm.ComputeConflictMergeInData(task_info);
    if (!ans_mergein.empty()) {
      ans_mergein[0].param.param_margin =
          ans_mergein[0].param.param_margin * conflict_agent.style;
    }

    ans.insert(ans.end(), ans_mergein.begin(), ans_mergein.end());
    return ans;
  };
  auto CalcConflictAreaBasePath =
      [](TaskInfo& task_info, const IntegrativeConflictInfo& conflict_agent)
      -> std::vector<ConnectionConflictInfo> {
    ConflictDataBasePath cdbp;
    std::unordered_set<int> agent_ids = {conflict_agent.id};

    cdbp.set_interactive_agent_ids(
        agent_ids, neodrive::planning::MotorwayInteractiveType::NONE);
    auto ans = cdbp.ComputeConflictMeetingData(task_info);
    if (!ans.empty()) {
      ans[0].param.param_margin =
          ans[0].param.param_margin * conflict_agent.style;
    }

    cdbp.set_interactive_agent_ids(
        agent_ids, neodrive::planning::MotorwayInteractiveType::NONE);
    auto ans_mergein = cdbp.ComputeConflictMergeInData(task_info);
    if (!ans_mergein.empty()) {
      ans_mergein[0].param.param_margin =
          ans_mergein[0].param.param_margin * conflict_agent.style;
    }

    ans.insert(ans.end(), ans_mergein.begin(), ans_mergein.end());
    return ans;
  };
  auto CalcConflictAreaBasePrediction =
      [](TaskInfo& task_info, const IntegrativeConflictInfo& conflict_agent)
      -> std::vector<ConnectionConflictInfo> {
    ConflictDataBasePrediction cdbp;
    std::unordered_set<int> agent_ids = {conflict_agent.id};

    cdbp.set_interactive_agent_ids(
        agent_ids, neodrive::planning::MotorwayInteractiveType::NONE);
    auto ans = cdbp.ComputeConflictMeetingData(task_info);
    if (!ans.empty()) {
      ans[0].param.param_margin =
          ans[0].param.param_margin * conflict_agent.style;
    }

    cdbp.set_interactive_agent_ids(
        agent_ids, neodrive::planning::MotorwayInteractiveType::NONE);
    auto ans_mergein = cdbp.ComputeConflictMergeInData(task_info);
    if (!ans_mergein.empty()) {
      ans_mergein[0].param.param_margin =
          ans_mergein[0].param.param_margin * conflict_agent.style;
    }

    ans.insert(ans.end(), ans_mergein.begin(), ans_mergein.end());
    return ans;
  };
  for (const auto& iter : conflict_infos_) {
    if (static_cast<uint32_t>(iter.conflict_area_calc_method) == 1) {
      auto ans = CalcConflictAreaBaseMap(task_info, iter);
      conflict_info.insert(conflict_info.end(), ans.begin(), ans.end());
    } else if (static_cast<uint32_t>(iter.conflict_area_calc_method) == 2) {
      auto ans = CalcConflictAreaBasePath(task_info, iter);
      conflict_info.insert(conflict_info.end(), ans.begin(), ans.end());
    } else if (static_cast<uint32_t>(iter.conflict_area_calc_method) == 3) {
      auto ans = CalcConflictAreaBasePrediction(task_info, iter);
      conflict_info.insert(conflict_info.end(), ans.begin(), ans.end());
    }
  }
  std::sort(
      conflict_info.begin(), conflict_info.end(),
      [](const ConnectionConflictInfo& a, const ConnectionConflictInfo& b) {
        return a.conflict_area_bound[3] < b.conflict_area_bound[3];
      });
  return true;
}
}  // namespace planning
}  // namespace neodrive