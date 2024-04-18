#include "speed_back_cipv_decider.h"

#include "src/planning/math/curve1d/spline.h"

namespace neodrive {
namespace planning {

namespace {
void VisS(const std::vector<double>& s_set,
          const TrafficConflictZoneContext& czContext) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("DecisionS");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto SetPt = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.2);
  };
  for (const auto& s : s_set) {
    common::math::Vec2d res;
    PlanningMap::Instance()->GetSamplePointFromConnection(czContext, s, res);
    Vec2d xy_point(res.x(), res.y());
    SetPt(event, xy_point);
  }
}

void VisClusterResult(const std::vector<Vec2d>& xys) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("cluster_area");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto SetPt = [](auto event, auto& pt, double k) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.2 + 0.2 * k);
  };
  for (size_t i = 0; i < xys.size(); ++i) {
    const auto& xy = xys[i];
    SetPt(event, xy, i / 2);
  }
}

void VisAgent(const double x, const double y) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("KeyAgent");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto SetPt = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(2.0);
  };
  Vec2d xy_point(x, y);
  SetPt(event, xy_point);
}

void VisObsPredictionSegment(const Vec2d& start_pt, const Vec2d& end_pt) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("PrediceitonSegment");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto SetPt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  auto polyline = event->mutable_polyline()->Add();
  SetPt(polyline->add_point(), start_pt);
  SetPt(polyline->add_point(), end_pt);
}

void VisAdcCornerPoints(const std::vector<Vec2d>& adc_pts) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("MergingAdcCornerPts");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto SetPt = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.2);
  };
  for (const auto& pt : adc_pts) {
    SetPt(event, pt);
  }
}

bool CompareByFrontEdge(const ConnectionConflictInfo& a,
                        const ConnectionConflictInfo& b) {
  return a.conflict_area_bound[3] < b.conflict_area_bound[3];
}

}  // namespace

SpeedBackCipvDecider::SpeedBackCipvDecider() { name_ = "SpeedBackCipvDecider"; }

SpeedBackCipvDecider::~SpeedBackCipvDecider() { Reset(); }

bool SpeedBackCipvDecider::Init(TaskInfo& task_info) {
  const auto& adc_corner_pt_coordinate =
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.adc_corner_pt_coordinate;
  if (adc_corner_pt_coordinate.size() <
      static_cast<int>(AdcCollideCornerPoint::NONE)) {
    LOG_ERROR("adc_corner_pt_coordinate size < 4");
    return false;
  }

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

  speed_vehicle_merging_decider_config_ptr_ =
      &config::PlanningConfig::Instance()
           ->planning_research_config()
           .speed_vehicle_merging_decider_config;
  speed_back_cipv_decider_config_ptr_ =
      &config::PlanningConfig::Instance()
           ->planning_research_config()
           .speed_vehicle_back_cipv_decider_config;
  if (nullptr == speed_vehicle_merging_decider_config_ptr_ ||
      nullptr == speed_back_cipv_decider_config_ptr_) {
    LOG_ERROR("get config failed.");
    return false;
  }
  if (!CalInputData(task_info)) {
    return false;
  }

  /// compute all conflict infos and combine:vector<Connect...>
  auto merge_in_info = BackInfoForMergeIn{}.ComputeConflictInfo(task_info);

  std::vector<ConnectionConflictInfo> all_conflict_infos{};
  all_conflict_infos.insert(all_conflict_infos.end(), merge_in_info.begin(),
                            merge_in_info.end());

  std::sort(all_conflict_infos.begin(), all_conflict_infos.end(),
            CompareByFrontEdge);
  if (InitialFilter(all_conflict_infos)) {
    LOG_INFO("initial start filtering: merge in!");
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

ErrorCode SpeedBackCipvDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  if (!Init(task_info)) {
    LOG_WARN("Skip Merging Area Speed Optimizer!");
    return ErrorCode::PLANNING_OK;
  }
  if (!CalInteractiveAgent(task_info)) {
    LOG_WARN("Skip Merging Area Speed Optimizer!");
    return ErrorCode::PLANNING_OK;
  }
  expected_state_.Reset();
  // LOG_INFO("Key Agent ID is {:3f}", key_agent_.id);
  bool rush_failed = false, execute_yield = false;

  if (risk_prob_ >= 0.0) {
    // rush
    LOG_INFO("Start Rush Model.....");
    conflict_area_bound_ = conflict_area_rush_bound_;
    if (conflict_area_rush_bound_.empty()) {
      LOG_INFO("Do not should rush, skip!");
      return ErrorCode::PLANNING_OK;
    }
    LOG_INFO("Conflict Area Rush Bound: {:3f}, {:3f}, {:3f}, {:3f}",
             conflict_area_rush_bound_[0], conflict_area_rush_bound_[1],
             conflict_area_rush_bound_[2], conflict_area_rush_bound_[3]);
    if (!set_rush_limit(task_info)) {
      LOG_ERROR("Set Rush Limit Failed!");
      return ErrorCode::PLANNING_OK;
    }
    ConflictAreaRushOptModel rush_model(ego_, key_conflict_info_,
                                        conflict_area_bound_);

    if (!rush_model.Init(0.3, 0.0, key_conflict_info_.param.rush_time,
                         &special_t_)) {
      LOG_ERROR("Rush Model Init Failed!");
      rush_failed = true;
    }
    LOG_INFO("Rush Special Time is {:3f}", special_t_);
    // determine redecision
    if (!is_redecision(task_info, conflict_area_rush_bound_[2], special_t_)) {
      LOG_INFO("Do not need redecision, skip!");
      return ErrorCode::PLANNING_OK;
    }
    // if (!setExactLimit()) {
    //   LOG_ERROR("Set Exact Limit Failed!");
    //   rush_failed = true;
    // }
    rush_model.set_speed_limit(upstream_speed_limit_);
    rush_model.set_s_limit(upstream_s_limit_);
    rush_model.set_acc_limit(upstream_a_limit_);
    rush_model.set_min_distance(key_conflict_info_.param.rush_margin_distance);

    if (!rush_model.Planner(
            ego_.sl_velocity.x() +
                speed_back_cipv_decider_config_ptr_->rush_velocity_increment,
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
      LOG_ERROR("Yield Failed in Conflict Area!");
      return ErrorCode::PLANNING_OK;
    }
    if (!set_yield_limit(task_info)) {
      LOG_ERROR("Set Yield Limit Failed!");
      return ErrorCode::PLANNING_OK;
    }
    ConflictAreaYieldOptModel yield_model(ego_, key_conflict_info_.agent,
                                          conflict_area_bound_);

    if (!yield_model.Init(0.3, 0.0, key_conflict_info_.param.yield_time,
                          &special_t_)) {
      LOG_ERROR("Yield Model Init Failed, Decision Error!");
      return ErrorCode::PLANNING_OK;
    }
    LOG_INFO("Yield Special Time is {:3f}", special_t_);
    // if (!setExactLimit()) {
    //   LOG_ERROR("Set Exact Limit Failed!");
    //   return ErrorCode::PLANNING_OK;
    // }
    yield_model.set_speed_limit(upstream_speed_limit_);
    yield_model.set_s_limit(upstream_s_limit_);
    yield_model.set_acc_limit(upstream_a_limit_);
    if (!yield_model.Planner(time_step_ *
                             std::floor(ego_.sl_velocity.x() / time_step_))) {
      LOG_ERROR("Yield Model Unsolvable!");
      return ErrorCode::PLANNING_OK;
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
    LOG_ERROR("Calculate Post Process Data Failed!");
    return ErrorCode::PLANNING_OK;
  }

  // visualizer
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  uint64_t cur_lane_id = traffic_conflict_zone_context.current_lane.id;
  uint64_t sig_fr_size = sigment_fr_bounds_.size();
  uint64_t connect_areas_size = process_data_.size();
  std::vector<ConnectionConflictInfo> pending_interactive_cases =
      process_data_[connect_areas_size - sig_fr_size];
  double ego_s = pending_interactive_cases[0].conflict_area_bound[2] - 1.0;
  if (expected_state_.is_rush_decision) {
    LOG_WARN("Result is rush!");
  } else {
    LOG_WARN("Result is yield!");
  }

  for (size_t i = 0; i < s_set.size(); ++i) {
    LOG_INFO("T,P,V,A: {:.3f}, {:.3f}, {:.3f}, {:.3f}", i * time_step_,
             s_set[i], v_set[i], a_set[i]);
  }
  VisS(s_set, traffic_conflict_zone_context);
  VisAgent(key_conflict_info_.agent.xy_position.x(),
           key_conflict_info_.agent.xy_position.y());
  return ErrorCode::PLANNING_OK;
}

bool SpeedBackCipvDecider::CalPostProcessData(
    TaskInfo& task_info, const MergeAreaEgoStateSequence& pre_data,
    bool is_yield, std::vector<double>* s_set, std::vector<double>* v_set,
    std::vector<double>* a_set) {
  // send speed limit type
  neodrive::global::planning::SpeedLimit internal_speed_limit{};
  internal_speed_limit.set_source_type(SpeedLimitType::BACK_CIPV);
  internal_speed_limit.add_upper_bounds(0);
  internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
  internal_speed_limit.set_acceleration(0);
  LOG_INFO("triger this ");
  data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
      internal_speed_limit);

  std::vector<double> s_set_, v_set_, a_set_;
  uint64_t size_pre_data = pre_data.deduction_ego_p_sequence.size(),
           size_post_data = 81;
  if (size_pre_data == 0) {
    LOG_INFO("Pre Data Size is {:3f}", size_pre_data);
    return false;
  }
  // goal_decision
  auto& goal_decision_data = task_info.current_frame()
                                 ->mutable_outside_planner_data()
                                 ->speed_obstacle_context.iter_deduction;
  if (input_iter_data_.deduction_ego_p_sequence.size() != 81) {
    LOG_ERROR("front result not match 81, its size is {}!",
              input_iter_data_.deduction_ego_p_sequence.size());
    return false;
  }
  auto& upper_s_bounds = task_info.current_frame()
                             ->mutable_outside_planner_data()
                             ->speed_context.dp_st_data.upper_iter_bound;
  auto& lower_s_bounds = task_info.current_frame()
                             ->mutable_outside_planner_data()
                             ->speed_context.dp_st_data.lower_iter_bound;
  auto& upper_v_bounds = task_info.current_frame()
                             ->mutable_outside_planner_data()
                             ->speed_context.dp_st_data.upper_iter_v_bound;
  auto& lower_v_bounds = task_info.current_frame()
                             ->mutable_outside_planner_data()
                             ->speed_context.dp_st_data.lower_iter_v_bound;

  upper_s_bounds.clear();
  lower_s_bounds.clear();
  upper_v_bounds.clear();
  lower_v_bounds.clear();
  goal_decision_data.Reset();
  const double max_limit_speed = data_center_->drive_strategy_max_speed();
  if (is_yield) {
    const auto& constraints_s_seq = input_iter_data_.deduction_ego_p_sequence;
    for (size_t i = 0; i < size_post_data; ++i) {
      double s_upper_bound = std::max(conflict_area_yield_bound_[3], 1e-3),
             s_lower_bound = 0.0;
      double ego_s = constraints_s_seq[i];
      s_upper_bound = std::min(s_upper_bound, ego_s);
      upper_s_bounds.emplace_back(
          STBoundInfo(STPoint(s_upper_bound + .3, i * time_step_),
                      STGraphBoundary::BoundaryType::UNKNOWN, 0, 0.0));
      lower_s_bounds.emplace_back(
          STBoundInfo(STPoint(s_lower_bound - .1, i * time_step_),
                      STGraphBoundary::BoundaryType::UNKNOWN, 0, 0.0));
      STGoalVInfo u_v_bound{
          .goal_t_v = STPoint(max_limit_speed, i * time_step_),
          .boundary_type = STGraphBoundary::BoundaryType::UNKNOWN};
      upper_v_bounds.emplace_back(u_v_bound);

      STGoalVInfo l_v_bound{
          .goal_t_v = STPoint(.0, i * time_step_),
          .boundary_type = STGraphBoundary::BoundaryType::UNKNOWN};
      lower_v_bounds.emplace_back(l_v_bound);
    }
  }
  for (size_t i = 0; i < size_post_data; ++i) {
    STPoint st_point{}, vt_point{}, at_point{};
    if (i < size_pre_data) {
      goal_decision_data.deduction_ego_a_sequence.emplace_back(
          expected_state_.deduction_ego_a_sequence[i]);
      goal_decision_data.deduction_ego_v_sequence.emplace_back(
          expected_state_.deduction_ego_v_sequence[i]);
      goal_decision_data.deduction_ego_p_sequence.emplace_back(
          expected_state_.deduction_ego_p_sequence[i]);
      goal_decision_data.deduction_ego_t_sequence.emplace_back(
          expected_state_.deduction_ego_t_sequence[i]);
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
      goal_decision_data.deduction_ego_a_sequence.emplace_back(a);
      goal_decision_data.deduction_ego_v_sequence.emplace_back(v);
      goal_decision_data.deduction_ego_p_sequence.emplace_back(s);
      goal_decision_data.deduction_ego_t_sequence.emplace_back(i * time_step_);
      s_set_.emplace_back(s);
      v_set_.emplace_back(v);
      a_set_.emplace_back(a);
    }
  }

  // goal decision
  if (!is_yield) {
    for (size_t i = 0; i < size_post_data; ++i) {
      double s_upper_bound = 80.0, s_lower_bound = 0.0;
      double ego_s = s_set_[i];
      s_upper_bound = std::min(s_upper_bound, ego_s);
      upper_s_bounds.emplace_back(
          STBoundInfo(STPoint(s_upper_bound + .3, i * time_step_),
                      STGraphBoundary::BoundaryType::UNKNOWN, 0, 0.0));
      lower_s_bounds.emplace_back(
          STBoundInfo(STPoint(s_lower_bound - .1, i * time_step_),
                      STGraphBoundary::BoundaryType::UNKNOWN, 0, 0.0));
      STGoalVInfo u_v_bound{
          .goal_t_v = STPoint(max_limit_speed, i * time_step_),
          .boundary_type = STGraphBoundary::BoundaryType::UNKNOWN};
      upper_v_bounds.emplace_back(u_v_bound);

      STGoalVInfo l_v_bound{
          .goal_t_v = STPoint(.0, i * time_step_),
          .boundary_type = STGraphBoundary::BoundaryType::UNKNOWN};
      lower_v_bounds.emplace_back(l_v_bound);
    }
  }
  *s_set = s_set_;
  *v_set = v_set_;
  *a_set = a_set_;
  return true;
}

bool SpeedBackCipvDecider::CalProcessDataBaseMap(TaskInfo& task_info) {
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  if (!is_merging_area(task_info)) {
    LOG_INFO("No Merging Area!");
    return false;
  }
  std::vector<ConnectionConflictInfo> sub_set;
  std::vector<double> s_last;
  const auto& dynamic_obstacle = task_info.current_frame()
                                     ->planning_data()
                                     .decision_data()
                                     .dynamic_obstacle();
  if (dynamic_obstacle.empty()) {
    LOG_INFO("No Dynamic Obs!");
    return false;
  }
  process_data_.clear();
  for (size_t i = 0; i < 2; ++i) {
    if (traffic_conflict_zone_context.merging_geoinfo[i]) {
      for (const auto& msl :
           *(traffic_conflict_zone_context.merging_geoinfo[i])) {
        s_last.emplace_back(msl.ego_rd);
        if (s_last.size() >= 2) {
          if ((s_last.back() - *(s_last.rbegin() + 1) >
               speed_back_cipv_decider_config_ptr_
                   ->conflict_area_integration_gap) &&
              !sub_set.empty()) {
            process_data_.emplace_back(sub_set);
            sub_set.clear();
          }
        }
        // dynamic obs
        for (const auto& obs : dynamic_obstacle) {
          if (obs->matched_lane_id() == msl.lane_id) {
            double heading_diff = normalize_angle(
                obs->velocity_heading() -
                task_info.current_frame()->inside_planner_data().vel_heading);
            if (std::abs(heading_diff) > M_PI_2) {
              continue;
            }
            ConnectionConflictInfo c;
            c.lane_id = msl.lane_id;
            double agent_rd, ego_rd = msl.ego_rd;
            Vec2d agent_center = obs->center();
            common::math::Vec2d p_agent(agent_center.x(), agent_center.y());
            if (!PlanningMap::Instance()->GetRSInLane(msl, agent_rd, p_agent)) {
              LOG_ERROR("Calculate Agent RD Failed!");
              return false;
            }
            // todo: agent_rd < 0 -> 0; ego_rd < 0 -> 0.
            // if (agent_rd <= 0.0) {
            //   agent_rd = 0.0;
            //   LOG_WARN("The agent is in a conflict zone, agent_rd is {}",
            //            agent_rd);
            // }
            // if (ego_rd <= 0.0) {
            //   ego_rd = 0.0;
            //   LOG_WARN("The ego is in a conflict zone, ego_rd is {}",
            //   ego_rd);
            // }
            double obs_length = obs->length();
            // special case deal
            double merging_s = std::min(msl.merging_s, 10.0),
                   ego_s = std::min(msl.ego_s, 10.0);
            std::vector<double> cab{agent_rd, agent_rd + merging_s + obs_length,
                                    ego_rd + ego_s + 1.0, ego_rd};
            c.conflict_area_bound = cab;
            CalAgentInfo(obs, &c.agent);
            c.agent.id = obs->id();
            sub_set.emplace_back(c);
          }
        }
        if (sub_set.empty()) {
          ConnectionConflictInfo c;
          c.lane_id = 1;
          std::vector<double> cab{1e+8, 1e+8, msl.ego_rd + msl.ego_s + 1.0,
                                  msl.ego_rd};
          c.conflict_area_bound = cab;
          std::vector<double> suspension{100.0, 100.0, 100.0, 100.0};
          std::vector<double> param_limit{3.0, 1.0, 3.0};
          c.agent = VehicleInfo(0.0, Vec2d(0.0, 0.0), Vec2d(0.0, 0.0),
                                Vec2d(0.0, 0.0), Vec2d(0.0, 0.0), 0.0,
                                suspension, 1.0, param_limit);
          sub_set.emplace_back(c);
        }
      }

      if (i == 0 && !traffic_conflict_zone_context.merging_geoinfo[1] &&
          !sub_set.empty()) {
        process_data_.emplace_back(sub_set);
        sub_set.clear();
      }
      if (i == 1 && !sub_set.empty()) {
        process_data_.emplace_back(sub_set);
        sub_set.clear();
      }
    }
  }
  LOG_INFO("Cluster Out {} conflict Areas!", process_data_.size());
  if (process_data_.empty()) {
    return false;
  }
  return true;
}

bool SpeedBackCipvDecider::CalMeetingDataBaseMap(
    TaskInfo& task_info, std::vector<ConnectionConflictInfo>* meeting_data) {
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  std::vector<ConnectionConflictInfo> meeting_data_{};
  const auto& dynamic_obstacle = task_info.current_frame()
                                     ->planning_data()
                                     .decision_data()
                                     .dynamic_obstacle();
  if (dynamic_obstacle.empty()) {
    LOG_INFO("No Dynamic Obs!");
    return false;
  }
  uint64_t ego_lane_id = traffic_conflict_zone_context.current_lane.id;
  Vec2d utm_ego_center = traffic_conflict_zone_context.ego_pos_utm;
  meeting_data_.clear();
  for (size_t i = 0; i < 2; ++i) {
    double extend_lane_length = (i == 0)
                                    ? 0.0
                                    : traffic_conflict_zone_context.current_lane
                                          .lane_ptr->TotalLength();
    if (traffic_conflict_zone_context.meeting_geoinfo[i]) {
      auto mg = traffic_conflict_zone_context.meeting_geoinfo[i];
      for (auto it = mg->begin(); it != mg->end(); it++) {
        auto& mlc = it->first;
        // ego
        double ego_rd{0.0}, ego_s{0.0};
        if (!PlanningMap::Instance()->GetResDistanceInLane(
                mlc, ego_lane_id, ego_rd, utm_ego_center)) {
          LOG_ERROR("Calculate Ego RD Failed!");
          return false;
        }
        ego_rd += extend_lane_length;
        ego_s = ego_rd + mlc.ego_s[1] - mlc.ego_s[0];
        // dynamic obs
        for (const auto& obs : dynamic_obstacle) {
          if (obs->matched_lane_id() == mlc.lane_id) {
            double heading_diff = normalize_angle(
                obs->velocity_heading() -
                task_info.current_frame()->inside_planner_data().vel_heading);
            if (std::abs(heading_diff) > M_PI_2) {
              continue;
            }
            const auto& obs_boundary = obs->PolygonBoundary();
            if (obs_boundary.start_s() >
                task_info.curr_sl().s() + VehicleParam::Instance()->length() -
                    VehicleParam::Instance()->back_edge_to_center()) {
              continue;
            }
            ConnectionConflictInfo c;
            c.lane_id = mlc.lane_id;
            double agent_rd;
            Vec2d agent_center = obs->center();
            common::math::Vec2d p_agent(agent_center.x(), agent_center.y());
            // todo?
            if (!PlanningMap::Instance()->GetRSInLane(mlc, agent_rd, p_agent)) {
              LOG_ERROR("Calculate Agent RD Failed!");
              return false;
            }
            double obs_length = obs->length();
            // special case deal
            double meeting_s =
                std::min(std::abs(mlc.merging_s[1] - mlc.merging_s[0]), 10.0);
            std::vector<double> cab{agent_rd, agent_rd + meeting_s + obs_length,
                                    ego_s + 1.0, ego_rd};
            c.conflict_area_bound = cab;
            CalAgentInfo(obs, &c.agent);
            c.agent.id = obs->id();
            meeting_data_.emplace_back(c);
          }
        }
      }
    }
  }
  *meeting_data = meeting_data_;
  LOG_INFO("Meeting Agents' size is {} !", meeting_data_.size());
  return true;
}

bool SpeedBackCipvDecider::CalProcessDataBasePath(
    TaskInfo& task_info,
    const std::vector<ConnectionConflictInfo>& meeting_data) {
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  const auto& dp_st_ignore_dynamic_obs =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->speed_obstacle_context.dp_st_map_ignore_dynamic_obs_id;
  if ((!is_merging_area(task_info)) && meeting_data.empty()) {
    LOG_INFO("No Merging And Meeting Area!");
    return false;
  }
  const auto& dynamic_obstacle = task_info.current_frame()
                                     ->planning_data()
                                     .decision_data()
                                     .dynamic_obstacle();
  if (dynamic_obstacle.empty()) {
    LOG_INFO("No Dynamic Obs!");
    return false;
  }
  process_data_.clear();
  //
  std::vector<ConnectionConflictInfo> process_data = meeting_data;
  const auto& merging_ids = traffic_conflict_zone_context.merging_ids;
  std::unordered_set<uint64_t> merging_lane_ids;
  for (auto iter = merging_ids.begin(); iter != merging_ids.end(); iter++) {
    merging_lane_ids.insert(iter->id);
  }

  for (const auto& obs : dynamic_obstacle) {
    double heading_diff = normalize_angle(
        obs->velocity_heading() -
        task_info.current_frame()->inside_planner_data().vel_heading);
    if (std::abs(heading_diff) > M_PI_2) {
      continue;
    }
    const auto& obs_boundary = obs->PolygonBoundary();
    if (obs_boundary.start_s() >
        task_info.curr_sl().s() + VehicleParam::Instance()->length() -
            VehicleParam::Instance()->back_edge_to_center()) {
      continue;
    }
    if (merging_lane_ids.find(obs->matched_lane_id()) ==
        merging_lane_ids.end()) {
      continue;
    }
    if (dp_st_ignore_dynamic_obs.find(obs->id()) !=
        dp_st_ignore_dynamic_obs.end()) {
      LOG_INFO("dp st ignore dynamic obs id is {}", obs->id());
      continue;
    }
    ConnectionConflictInfo c;
    c.lane_id = obs->matched_lane_id();
    CalAgentInfo(obs, &c.agent);
    c.agent.id = obs->id();
    if (!CollisionCheckWithObstaclePolyline(
            task_info.current_frame()->inside_planner_data(), *obs,
            task_info.current_frame()->mutable_outside_planner_data(),
            &c.conflict_area_bound)) {
      std::vector<double> cab{1e+8, 1e+8, 1e+8, 1e+8};
      c.conflict_area_bound = cab;
    }

    process_data.emplace_back(c);
  }
  std::sort(process_data.begin(), process_data.end(), CompareByFrontEdge);
  if (InitialFilter(process_data)) {
    LOG_INFO("initial start filtering: merge in!");
  }

  size_t start = 0;
  if (process_data.empty()) {
    ConnectionConflictInfo c;
    c.lane_id = 1;
    std::vector<double> cab{1e+8, 1e+8, 1e+8, 1e+8};
    c.conflict_area_bound = cab;
    std::vector<double> suspension{100.0, 100.0, 100.0, 100.0};
    std::vector<double> param_limit{3.0, 1.0, 3.0};
    c.agent =
        VehicleInfo(0.0, Vec2d(0.0, 0.0), Vec2d(0.0, 0.0), Vec2d(0.0, 0.0),
                    Vec2d(0.0, 0.0), 0.0, suspension, 1.0, param_limit);
    process_data.emplace_back(c);
  }
  for (size_t i = 1; i <= process_data.size(); ++i) {
    if (i == process_data.size() ||
        process_data[i].conflict_area_bound[3] -
                process_data[i - 1].conflict_area_bound[3] >
            speed_back_cipv_decider_config_ptr_
                ->conflict_area_integration_gap) {
      std::vector<ConnectionConflictInfo> sub_set(process_data.begin() + start,
                                                  process_data.begin() + i);
      process_data_.emplace_back(sub_set);
      start = i;
    }
  }
  LOG_INFO("Cluster Out {} conflict Areas!", process_data_.size());
  if (process_data_.empty()) {
    return false;
  }
  const auto& path_points = task_info.current_frame()
                                ->mutable_outside_planner_data()
                                ->path_data->path()
                                .path_points();
  std::vector<Vec2d> xys{};
  for (size_t i = 0; i < process_data_.size(); ++i) {
    const auto& entry = process_data_[i];
    if (!entry.empty()) {
      const auto& front_area = entry.front().conflict_area_bound;
      const auto& rear_area = entry.back().conflict_area_bound;
      double front_value = front_area[3], rear_value = rear_area[2];
      LOG_INFO("cluster front bound: {:3f}, rear bound: {:3f}", front_value,
               rear_value);
      bool f_memory = false, r_memory = false;
      for (const auto& pt : path_points) {
        if (pt.s() >= front_value && !f_memory) {
          xys.emplace_back(pt.coordinate());
          f_memory = true;
        }
        if (pt.s() >= rear_value && !r_memory) {
          xys.emplace_back(pt.coordinate());
          r_memory = true;
        }
        if (f_memory && r_memory) {
          break;
        }
      }
      if (xys.size() % 2 != 0) {
        xys.emplace_back(Vec2d(0.0, 0.0));
      }
    }
  }
  VisClusterResult(xys);
  return true;
}

bool SpeedBackCipvDecider::is_merging_area(TaskInfo& task_info) {
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  bool is_merge_area = false;
  if (!((traffic_conflict_zone_context.type ==
         TrafficConflictZoneContext::connectionType::Merging) ||
        (traffic_conflict_zone_context.type ==
         TrafficConflictZoneContext::connectionType::NMerging))) {
    LOG_WARN("Current Type is Merge Area!");
    is_merge_area = true;
  }
  if (!((traffic_conflict_zone_context.near_type ==
         TrafficConflictZoneContext::connectionType::Merging) ||
        (traffic_conflict_zone_context.near_type ==
         TrafficConflictZoneContext::connectionType::NMerging))) {
    LOG_WARN("Near Type is Merge Area!");
    is_merge_area = true;
  }
  if (!((traffic_conflict_zone_context.rear_type ==
         TrafficConflictZoneContext::connectionType::Merging) ||
        (traffic_conflict_zone_context.rear_type ==
         TrafficConflictZoneContext::connectionType::NMerging))) {
    LOG_WARN("Rear Type is Merge Area!");
    is_merge_area = true;
  }
  return is_merge_area;
}

bool SpeedBackCipvDecider::set_speed_limit(
    const std::vector<double>& speed_limit) {
  if (speed_limit.size() != std::floor(special_t_ / time_step_)) {
    LOG_ERROR("Set Speed Limit Failed!");
    return false;
  }
  speed_limit_ = speed_limit;
  return true;
}

bool SpeedBackCipvDecider::set_exact_limit() {
  if (special_t_ < time_step_) {
    LOG_ERROR("Special Time is Too Small!");
    return false;
  }
  uint64_t steps = std::floor(special_t_ / time_step_);
  speed_limit_.clear();
  s_limit_.clear();
  // The situation where yielding fails to solve (reasonable): there is a
  // blockage in front of the lane where the merging vehicle is located during
  // the merging process
  // The situation (risk) of failure to solve the problem of preemptive
  // execution: the window sent by the upstream for 5 seconds is not long
  // enough, which is less than a special time, resulting in the constraint of s
  // being unable to be overcome For the latter, reluctantly switch to another
  // line to do a back up
  for (size_t i = 0; i < steps; ++i) {
    if (i < upstream_speed_limit_.size()) {
      speed_limit_.emplace_back(upstream_speed_limit_[i]);
      s_limit_.emplace_back(upstream_s_limit_[i]);
    } else {
      speed_limit_.emplace_back(upstream_speed_limit_.back());
      s_limit_.emplace_back(upstream_s_limit_.back());
    }
  }
  return true;
}

bool SpeedBackCipvDecider::set_rush_limit(TaskInfo& task_info) {
  const auto& upper_s_bounds = task_info.current_frame()
                                   ->mutable_outside_planner_data()
                                   ->speed_context.dp_st_data.upper_iter_bound;
  const auto& upper_v_bounds =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->speed_context.dp_st_data.upper_iter_v_bound;

  uint64_t size_s_bound = upper_s_bounds.size();
  if (size_s_bound == 0) {
    LOG_INFO("Missing Rush Upper Bound!");
    return false;
  }
  upstream_speed_limit_.clear();
  upstream_s_limit_.clear();
  upstream_a_limit_.clear();
  for (size_t i = 0; i < size_s_bound; ++i) {
    upstream_speed_limit_.emplace_back(upper_v_bounds[i].goal_t_v.s());
    upstream_s_limit_.emplace_back(upper_s_bounds[i].st_point.s());
    upstream_a_limit_.emplace_back(10.0);
  }
  if (!upstream_speed_limit_.empty() && !upstream_s_limit_.empty() &&
      upstream_speed_limit_.size() == upstream_s_limit_.size()) {
    return true;
  }
  LOG_ERROR("Upstream Limit is Empty!");
  return false;
}

bool SpeedBackCipvDecider::CalInputData(TaskInfo& task_info) {
  const auto& goal_decision_data = task_info.current_frame()
                                       ->mutable_outside_planner_data()
                                       ->speed_obstacle_context.iter_deduction;
  std::vector<double> p_seq{}, v_seq{}, a_seq{}, t_seq{};
  if (goal_decision_data.deduction_ego_t_sequence.size() < 4) {
    LOG_ERROR("Cannot perform differential compensation!");
    return false;
  }
  input_iter_data_.Reset();
  for (size_t i = 0; i < goal_decision_data.deduction_ego_p_sequence.size();
       i++) {
    p_seq.emplace_back(goal_decision_data.deduction_ego_p_sequence[i]);
    v_seq.emplace_back(goal_decision_data.deduction_ego_v_sequence[i]);
    a_seq.emplace_back(goal_decision_data.deduction_ego_a_sequence[i]);
    t_seq.emplace_back(goal_decision_data.deduction_ego_t_sequence[i]);
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

bool SpeedBackCipvDecider::set_yield_limit(TaskInfo& task_info) {
  upstream_speed_limit_.clear();
  upstream_s_limit_.clear();
  upstream_a_limit_.clear();
  for (size_t i = 0; i < input_iter_data_.deduction_ego_p_sequence.size();
       ++i) {
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

bool SpeedBackCipvDecider::SplitFRAreaBound() {
  sigment_fr_bounds_.clear();
  for (const auto& c : process_data_) {
    ConnectionConflictInfo front_bound,
        rear_bound;  // front representing rear
    front_bound = c.front();
    rear_bound = c.back();
    // Strive for the 'front' boundary [1], 'back' boundary [2]
    // Give way to the 'front' boundary [0], the 'back' boundary [2]
    if (rear_bound.conflict_area_bound[3] <= 0.0) {
      continue;
    }
    std::vector<double> sigment_fr_bound{front_bound.conflict_area_bound[3],
                                         rear_bound.conflict_area_bound[3],
                                         rear_bound.conflict_area_bound[2]};
    sigment_fr_bounds_.emplace_back(
        sigment_fr_bound);  // [1] Determined the end of this interaction, [1]
                            // is a negative value interaction end
  }
  if (sigment_fr_bounds_.empty()) {
    LOG_WARN("Bound Sigment Is Empty!");
    ConnectionConflictInfo front_bound,
        rear_bound;  // front representing rear
    front_bound = (process_data_.back()).front();
    rear_bound = (process_data_.back()).back();
    if (rear_bound.conflict_area_bound[2] >
        0.0) {  // Complete end of interaction
      std::vector<double> sigment_fr_bound{front_bound.conflict_area_bound[3],
                                           rear_bound.conflict_area_bound[3],
                                           rear_bound.conflict_area_bound[2]};
      sigment_fr_bounds_.emplace_back(sigment_fr_bound);
    }
  }
  LOG_INFO("FR Area Bound Size is {}", sigment_fr_bounds_.size());
  return true;
}

bool SpeedBackCipvDecider::CalInteractiveAgent(TaskInfo& task_info) {
  uint64_t sig_fr_size = sigment_fr_bounds_.size();
  uint64_t connect_areas_size = process_data_.size();
  std::vector<ConnectionConflictInfo> pending_interactive_cases =
      process_data_[connect_areas_size - sig_fr_size];
  std::vector<double> pending_sig_bounds = sigment_fr_bounds_.front();
  // Simple Game Theory for Screening Interactive Agents
  // Single to single ground interaction, interactive agents choose the agent
  // with the highest probability of seizing
  double rush_prob_diff = 0.0, yield_prob_diff = 1.0;
  bool is_skip = false;
  for (const auto& c : pending_interactive_cases) {
    double prob_tmp = 0.0;
    // VehicleInfo agent = c.agent;
    std::vector<double> conflict_area_bound = c.conflict_area_bound;
    // Skip if there is no interactive agent or if the interactive agent has
    // passed through the conflict area
    if (conflict_area_bound[0] >
            speed_back_cipv_decider_config_ptr_->agent_filter_far_end ||
        conflict_area_bound[0] <=
            speed_back_cipv_decider_config_ptr_->agent_filter_near_end) {
      is_skip = true;
      continue;
    }
    if (!CalRushProbByGameTheory(c, pending_sig_bounds, &prob_tmp)) {
      LOG_ERROR("Calculate Rush Prob By Game Theory Failed!");
      return false;
    }
    // Ego yield (entering the conflict zone is not yield)
    std::vector<double> cab_yield{conflict_area_bound[0],
                                  conflict_area_bound[1], pending_sig_bounds[2],
                                  pending_sig_bounds[0]};
    std::vector<double> cab_rush{conflict_area_bound[0], conflict_area_bound[1],
                                 pending_sig_bounds[2], pending_sig_bounds[1]};
    if (rush_prob_diff > prob_tmp && pending_sig_bounds[0] > 0.0) {
      is_skip = false;
      rush_prob_diff = prob_tmp;
      // key_agent_ = agent;
      key_conflict_info_ = c;
      conflict_area_yield_bound_ = cab_yield;
    } else if (prob_tmp >= 0.0 && yield_prob_diff > prob_tmp &&
               conflict_area_bound[0] > 0.0 &&
               conflict_area_bound[0] <
                   speed_back_cipv_decider_config_ptr_
                       ->agent_filter_rush_far_end) {  // ego rush
      is_skip = false;
      yield_prob_diff = prob_tmp;
      // key_agent_ = agent;
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

bool SpeedBackCipvDecider::CalRushProbByGameTheory(
    const ConnectionConflictInfo& c, const std::vector<double>& sig_bounds,
    double* rush_prob_diff) {
  VehicleInfo agent = c.agent;
  std::vector<double> conflict_area_bound = c.conflict_area_bound;
  // double l = conflict_area_bound[3], l_ = conflict_area_bound[0],
  //        l_c_agent = conflict_area_bound[1], l_c_ego = sig_bounds[2];
  double l = conflict_area_bound[3], l_ = conflict_area_bound[0],
         l_c_agent = conflict_area_bound[0], l_c_ego = sig_bounds[2];
  l = (l - ego_.front_suspension <= 0.0) ? 1e-5 : (l - ego_.front_suspension);
  l_ = (l_ - agent.front_suspension <= 0.0) ? 1e-5
                                            : (l_ - agent.front_suspension);
  double ttc_ego = l / std::max(ego_.sl_velocity.x(), 1e-3),
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

bool SpeedBackCipvDecider::CalOtherAgentBound(
    const uint64_t merging_lane_id, const common::math::Vec2d& point) {
  return true;
}
bool SpeedBackCipvDecider::CalAgentInfo(const Obstacle* obs,
                                        VehicleInfo* agent_info) {
  Vec2d xy_p(obs->center().x(), obs->center().y());
  Vec2d sl_p(obs->center_sl().s(), obs->center_sl().l());
  Vec2d sl_v(obs->speed(), 0.0);
  Vec2d sl_a(0.0, 0.0);
  double width = obs->width();
  double length = obs->length();
  std::vector<double> suspension{0.5 * length, 0.5 * length, 0.5 * width,
                                 0.5 * width};
  std::vector<double> param_limit{3.0, 1.0, 3.0};
  *agent_info =
      VehicleInfo(obs->id(), xy_p, sl_p, sl_v, sl_a, obs->velocity_heading(),
                  suspension, 0.5, param_limit);
  return true;
}

bool SpeedBackCipvDecider::CollisionCheckWithObstaclePolyline(
    const InsidePlannerData& inside_data, const Obstacle& obstacle,
    OutsidePlannerData* const outside_data,
    std::vector<double>* conflict_area_bound) {
  const auto& pred_traj = obstacle.uniform_trajectory();
  if (pred_traj.num_of_points() < 2) {
    LOG_ERROR("trajectory points less 2.");
    return false;
  }
  // 1.obs_near_adc_prediction_segment
  OBS_SIDE obs_side{OBS_SIDE::UNKNOWN};
  Segment2d obs_prediction_segment{};
  Vec2d obs_base_pt{};
  GetObsPredictionSegment(inside_data, obstacle, obs_side,
                          obs_prediction_segment, obs_base_pt);

  // 2.adc_bounding_boxes，
  AdcCollideCornerPoint adc_first_collide_corner_point{
      AdcCollideCornerPoint::NONE};
  AdcCollideCornerPoint ego_collide_corner_point{AdcCollideCornerPoint::NONE};
  AdcCollideCornerPoint agent_collide_corner_point{AdcCollideCornerPoint::NONE};
  const auto& path_points = outside_data->path_data->path().path_points();
  int path_first_collide_i = path_points.size() + 2;
  int path_ego_end_collide_i = path_points.size() + 2;
  int path_agent_end_collide_i = path_points.size() + 2;
  Vec2d path_first_collide_pt{}, path_ego_end_collide_pt{},
      path_agent_end_collide_pt{};
  GetFirstCollideInfo(outside_data, obs_prediction_segment, obs_side,
                      adc_first_collide_corner_point, ego_collide_corner_point,
                      agent_collide_corner_point, path_first_collide_i,
                      path_first_collide_pt, path_ego_end_collide_i,
                      path_ego_end_collide_pt, path_agent_end_collide_i,
                      path_agent_end_collide_pt);

  // 3.lower:s,t,adc_first_collide_corner_point.
  if (!GetLaneMergingInfo(path_points, path_first_collide_i, obstacle.id(),
                          path_first_collide_pt, path_ego_end_collide_i,
                          path_agent_end_collide_i, path_agent_end_collide_pt,
                          obstacle.length(), obs_base_pt,
                          conflict_area_bound)) {
    LOG_WARN("Calculate Lane Merging Info Failed!");
    return false;
  }
  return true;
}

void SpeedBackCipvDecider::GetObsPredictionSegment(
    const InsidePlannerData& inside_data, const Obstacle& obstacle,
    OBS_SIDE& obs_side, Segment2d& obs_prediction_segment, Vec2d& obs_base_pt) {
  double obs_nearest_local_y{std::numeric_limits<double>::infinity()};
  std::vector<Vec2d> obs_bounding_box_corners;
  obstacle.bounding_box().get_all_corners(&obs_bounding_box_corners);
  for (const auto& pt : obs_bounding_box_corners) {
    double obs_local_x{0.0}, obs_local_y{0.0}, obs_local_heading{0.0};
    earth2vehicle(inside_data.vel_x, inside_data.vel_y, inside_data.vel_heading,
                  pt.x(), pt.y(), obstacle.velocity_heading(), obs_local_x,
                  obs_local_y, obs_local_heading);
    if (std::abs(obs_local_y) < obs_nearest_local_y) {
      obs_nearest_local_y = std::abs(obs_local_y);
      obs_base_pt = pt;
      obs_side = obs_local_y > 0.0 ? OBS_SIDE::LEFT : OBS_SIDE::RIGHT;
    }
  }
  double length_ray =
      (obstacle.type() == Obstacle::ObstacleType::PEDESTRIAN)
          ? obstacle.speed() * speed_back_cipv_decider_config_ptr_
                                   ->speed_back_pedestrain_pre_time
          : speed_vehicle_merging_decider_config_ptr_->obs_prediction_dis;
  Vec2d segment_end_pt{
      obs_base_pt.x() + length_ray * std::cos(obstacle.velocity_heading()),
      obs_base_pt.y() + length_ray * std::sin(obstacle.velocity_heading())};
  obs_prediction_segment = std::move(Segment2d(obs_base_pt, segment_end_pt));
  VisObsPredictionSegment(obs_base_pt, segment_end_pt);
}

void SpeedBackCipvDecider::GetFirstCollideInfo(
    const OutsidePlannerData* const outside_data,
    const Segment2d& obs_prediction_segment, const OBS_SIDE& obs_side,
    AdcCollideCornerPoint& adc_first_collide_corner_point,
    AdcCollideCornerPoint& ego_collide_corner_point,
    AdcCollideCornerPoint& agent_collide_corner_point,
    int& path_first_collide_i, Vec2d& path_first_collide_pt,
    int& path_ego_end_collide_i, Vec2d& path_ego_end_collide_pt,
    int& path_agent_end_collide_i, Vec2d& path_agent_end_collide_pt) {
  const auto& adc_corner_pt_coordinate =
      outside_data->speed_obstacle_context.adc_corner_pt_coordinate;
  Segment2d current_adc_back_edge{adc_corner_pt_coordinate[static_cast<int>(
                                      AdcCollideCornerPoint::LEFT_REAR)],
                                  adc_corner_pt_coordinate[static_cast<int>(
                                      AdcCollideCornerPoint::RIGHT_REAR)]};
  Segment2d current_adc_left_edge{adc_corner_pt_coordinate[static_cast<int>(
                                      AdcCollideCornerPoint::LEFT_REAR)],
                                  adc_corner_pt_coordinate[static_cast<int>(
                                      AdcCollideCornerPoint::LEFT_FRONT)]};
  Segment2d current_adc_right_edge{adc_corner_pt_coordinate[static_cast<int>(
                                       AdcCollideCornerPoint::RIGHT_REAR)],
                                   adc_corner_pt_coordinate[static_cast<int>(
                                       AdcCollideCornerPoint::RIGHT_FRONT)]};
  if (obs_prediction_segment.get_intersect(current_adc_back_edge,
                                           &path_first_collide_pt)) {
    path_first_collide_i = 0, path_ego_end_collide_i = 0,
    path_agent_end_collide_i = 0;
    if (OBS_SIDE::RIGHT == obs_side) {
      adc_first_collide_corner_point = AdcCollideCornerPoint::RIGHT_REAR;
      ego_collide_corner_point = AdcCollideCornerPoint::RIGHT_REAR;
      agent_collide_corner_point = AdcCollideCornerPoint::RIGHT_FRONT;
    } else {
      adc_first_collide_corner_point = AdcCollideCornerPoint::LEFT_REAR;
      ego_collide_corner_point = AdcCollideCornerPoint::LEFT_REAR;
      agent_collide_corner_point = AdcCollideCornerPoint::LEFT_FRONT;
    }
    LOG_INFO("obs collide adc at back.");
    return;
  }
  if (OBS_SIDE::LEFT == obs_side &&
      obs_prediction_segment.get_intersect(current_adc_left_edge,
                                           &path_first_collide_pt)) {
    path_first_collide_i = 0, path_ego_end_collide_i = 0,
    path_agent_end_collide_i = 0;
    adc_first_collide_corner_point = AdcCollideCornerPoint::LEFT_REAR;
    ego_collide_corner_point = AdcCollideCornerPoint::LEFT_REAR;
    agent_collide_corner_point = AdcCollideCornerPoint::LEFT_FRONT;
    LOG_INFO("obs collide adc at left.");
    return;
  }
  if (OBS_SIDE::RIGHT == obs_side &&
      obs_prediction_segment.get_intersect(current_adc_right_edge,
                                           &path_first_collide_pt)) {
    path_first_collide_i = 0, path_ego_end_collide_i = 0,
    path_agent_end_collide_i = 0;
    adc_first_collide_corner_point = AdcCollideCornerPoint::RIGHT_REAR;
    ego_collide_corner_point = AdcCollideCornerPoint::RIGHT_REAR;
    agent_collide_corner_point = AdcCollideCornerPoint::RIGHT_FRONT;
    LOG_INFO("obs collide adc at right.");
    return;
  }

  bool find_first_collide_pt{false}, find_ego_end_collide_pt{false},
      find_agent_end_collide_pt{false};
  const auto& path_points = outside_data->path_data->path().path_points();
  for (size_t i = 0; i < path_points.size(); ++i) {
    std::vector<Vec2d> adc_corner_pts;
    GetAdcCornerPointCoordinate(path_points[i].coordinate(),
                                path_points[i].theta(), adc_corner_pts);
    if (adc_corner_pts.size() < static_cast<int>(AdcCollideCornerPoint::NONE)) {
      LOG_WARN("number of adc corner points less than 4.");
      continue;
    }
    VisAdcCornerPoints(adc_corner_pts);

    Segment2d path_adc_back_edge{
        adc_corner_pts[static_cast<int>(AdcCollideCornerPoint::LEFT_REAR)],
        adc_corner_pts[static_cast<int>(AdcCollideCornerPoint::RIGHT_REAR)]};
    Segment2d path_adc_front_edge{
        adc_corner_pts[static_cast<int>(AdcCollideCornerPoint::LEFT_FRONT)],
        adc_corner_pts[static_cast<int>(AdcCollideCornerPoint::RIGHT_FRONT)]};
    if (!find_first_collide_pt) {
      if (obs_prediction_segment.get_intersect(path_adc_back_edge,
                                               &path_first_collide_pt)) {
        find_first_collide_pt = true;
      } else if (OBS_SIDE::RIGHT == obs_side) {
        Segment2d adc_side_edge = Segment2d{
            adc_corner_pts[static_cast<int>(AdcCollideCornerPoint::RIGHT_REAR)],
            adc_corner_pts[static_cast<int>(
                AdcCollideCornerPoint::RIGHT_FRONT)]};
        if (obs_prediction_segment.get_intersect(adc_side_edge,
                                                 &path_first_collide_pt)) {
          find_first_collide_pt = true;
        }
      } else {
        Segment2d adc_side_edge = Segment2d{
            adc_corner_pts[static_cast<int>(AdcCollideCornerPoint::LEFT_REAR)],
            adc_corner_pts[static_cast<int>(
                AdcCollideCornerPoint::LEFT_FRONT)]};
        if (obs_prediction_segment.get_intersect(adc_side_edge,
                                                 &path_first_collide_pt)) {
          find_first_collide_pt = true;
        }
      }

      if (find_first_collide_pt) {
        path_first_collide_i = i;
      }
    }
    if (!find_ego_end_collide_pt) {
      if (obs_prediction_segment.get_intersect(path_adc_back_edge,
                                               &path_ego_end_collide_pt)) {
        find_ego_end_collide_pt = true;
        path_ego_end_collide_i = i;
      }
    }
    if (!find_agent_end_collide_pt) {
      if (obs_prediction_segment.get_intersect(path_adc_back_edge,
                                               &path_agent_end_collide_pt)) {
        find_agent_end_collide_pt = true;
        path_agent_end_collide_i = i;
      }
    }
    if (find_first_collide_pt && find_ego_end_collide_pt &&
        find_agent_end_collide_pt) {
      break;
    }
  }
}

bool SpeedBackCipvDecider::GetLaneMergingInfo(
    const std::vector<PathPoint>& path_points, const int path_first_collide_i,
    const int obs_id, const Vec2d& path_first_collide_pt,
    const int ego_end_collide_i, const int agent_end_collide_i,
    const Vec2d& agent_end_collide_pt, const double agent_length,
    const Vec2d& obs_base_pt, std::vector<double>* conflict_area_bound) {
  bool miss_ego_end_pt{false}, miss_agent_end_pt{false};
  if (path_first_collide_i >= path_points.size() || path_points.empty()) {
    LOG_WARN("Don't Find First Collide Pt!");
    return false;
  }
  if (path_first_collide_i == 0) {
    LOG_WARN("Ego in Conflict Area!");
    return false;
  }
  if (ego_end_collide_i >= path_points.size()) {
    miss_ego_end_pt = true;
    LOG_WARN("Don't Found Ego Right-of-Way Pt!");
  }
  if (agent_end_collide_i >= path_points.size()) {
    miss_agent_end_pt = true;
    LOG_WARN("Don't Found Agent Right-of-Way Pt!");
  }

  double ego_s{0.0}, agent_s{0.0}, ego_rd{0.0}, agent_rd{0.0};
  ego_rd = path_points[path_first_collide_i].s();
  agent_rd =
      std::sqrt(std::pow(path_first_collide_pt.x() - obs_base_pt.x(), 2) +
                std::pow(path_first_collide_pt.y() - obs_base_pt.y(), 2));
  ego_s = miss_ego_end_pt
              ? (ego_rd + 3.0)
              : std::max(ego_rd, path_points[ego_end_collide_i].s());
  agent_s =
      miss_agent_end_pt
          ? (agent_rd + 3.0)
          : (std::sqrt(
                 std::pow(agent_end_collide_pt.x() - obs_base_pt.x(), 2) +
                 std::pow(agent_end_collide_pt.y() - obs_base_pt.y(), 2)) +
             agent_length);
  std::vector<double> area_bound{agent_rd, agent_s, ego_s, ego_rd};
  LOG_INFO("Obs:{}, Agent RD:{:3f}, Agent S:{:3f}, Ego RD:{:3f}, Ego S:{:3f}",
           obs_id, agent_rd, agent_s, ego_rd, ego_s);
  *conflict_area_bound = area_bound;
  return true;
}

void SpeedBackCipvDecider::GetAdcCornerPointCoordinate(
    const Vec2d& adc_coordinate, const double adc_heading,
    std::vector<Vec2d>& adc_corner_pt_coordinate) {
  adc_corner_pt_coordinate.clear();
  adc_corner_pt_coordinate.resize(
      static_cast<int>(AdcCollideCornerPoint::NONE));
  double x_g = 0.0;
  double y_g = 0.0;
  double theta_g = 0.0;
  vehicle2earth(
      adc_coordinate.x(), adc_coordinate.y(), adc_heading,
      -VehicleParam::Instance()->back_edge_to_center(),
      VehicleParam::Instance()->left_edge_to_center() +
          speed_vehicle_merging_decider_config_ptr_->merging_adc_buffer,
      0.0, x_g, y_g, theta_g);
  adc_corner_pt_coordinate[static_cast<int>(AdcCollideCornerPoint::LEFT_REAR)] =
      std::move(Vec2d(x_g, y_g));
  vehicle2earth(
      adc_coordinate.x(), adc_coordinate.y(), adc_heading,
      -VehicleParam::Instance()->back_edge_to_center(),
      -VehicleParam::Instance()->right_edge_to_center() -
          speed_vehicle_merging_decider_config_ptr_->merging_adc_buffer,
      0.0, x_g, y_g, theta_g);
  adc_corner_pt_coordinate[static_cast<int>(
      AdcCollideCornerPoint::RIGHT_REAR)] = std::move(Vec2d(x_g, y_g));
  vehicle2earth(
      adc_coordinate.x(), adc_coordinate.y(), adc_heading,
      VehicleParam::Instance()->front_edge_to_center(),
      -VehicleParam::Instance()->right_edge_to_center() -
          speed_vehicle_merging_decider_config_ptr_->merging_adc_buffer,
      0.0, x_g, y_g, theta_g);
  adc_corner_pt_coordinate[static_cast<int>(
      AdcCollideCornerPoint::RIGHT_FRONT)] = std::move(Vec2d(x_g, y_g));
  vehicle2earth(
      adc_coordinate.x(), adc_coordinate.y(), adc_heading,
      VehicleParam::Instance()->front_edge_to_center(),
      VehicleParam::Instance()->left_edge_to_center() +
          speed_vehicle_merging_decider_config_ptr_->merging_adc_buffer,
      0.0, x_g, y_g, theta_g);
  adc_corner_pt_coordinate[static_cast<int>(
      AdcCollideCornerPoint::LEFT_FRONT)] = std::move(Vec2d(x_g, y_g));
}

bool SpeedBackCipvDecider::is_redecision(TaskInfo& task_info,
                                         const double rear_bound,
                                         const double constraints_time) {
  if (input_iter_data_.deduction_ego_p_sequence.size() != 81) {
    LOG_ERROR("front result not match 81, its size is {}!",
              input_iter_data_.deduction_ego_p_sequence.size());
    return false;
  }
  const auto& init_s_seq = input_iter_data_.deduction_ego_p_sequence;
  const auto& init_t_seq = input_iter_data_.deduction_ego_t_sequence;
  for (size_t i = 0; i < init_s_seq.size(); ++i) {
    // rear_bound is distal end
    if (init_s_seq[i] >= rear_bound) {
      if (init_t_seq[i] <= constraints_time) {
        return false;  // do not need redecision
      }
      return true;
    }
  }
  return true;
}

bool SpeedBackCipvDecider::InitialFilter(
    std::vector<ConnectionConflictInfo>& conflict_info) {
  if (conflict_info.empty()) {
    return false;
  }
  const auto& nearest_info = conflict_info.front().conflict_area_bound;
  double ego_v = ego_.sl_velocity.x();
  if (nearest_info[3] > 10.0 && ego_v <= 1.0) {
    conflict_info.clear();
    return true;
  }
  return false;
}

std::vector<std::vector<ConnectionConflictInfo>>
SpeedBackCipvDecider::CombineConflictInfos(
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
            speed_back_cipv_decider_config_ptr_
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

}  // namespace planning
}  // namespace neodrive