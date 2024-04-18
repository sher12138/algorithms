#include "motorway_speed_optimizer_for_iter.h"

#include "common/visualizer_event/visualizer_event.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/math/curve1d/spline.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {

namespace {

constexpr double k_max_jerk = 10.0;
constexpr double k_min_jerk = -50.0;
constexpr double k_min_acceleration = -5.5;
constexpr double k_max_soft_speed_time = 4.0;
constexpr double k_soft_max_acceleration = 1.0;

void VisBoundST(const std::vector<STPoint>& upper_bound,
                const std::vector<STPoint>& lower_bound) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto e_vt_l = vis::EventSender::Instance()->GetEvent("bound__upper");
  e_vt_l->set_type(visualizer::Event::k2D);
  e_vt_l->mutable_color()->set_r(0.8);
  e_vt_l->mutable_color()->set_g(0.0);
  e_vt_l->mutable_color()->set_b(0.0);
  e_vt_l->mutable_color()->set_a(0.6);
  auto vt_l_pz = e_vt_l->add_polyline();
  auto vt_l_lz = e_vt_l->add_polyline();

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x), ans->set_y(y), ans->set_z(0);
  };

  for (size_t i = 0; i < lower_bound.size(); ++i) {
    set_pt(vt_l_pz->add_point(), lower_bound[i].t(), lower_bound[i].s());
    set_pt(vt_l_lz->add_point(), upper_bound[i].t(), upper_bound[i].s());
  }
}

void VisBoundVT(const std::vector<double>& v_bound) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto e_v_bound = vis::EventSender::Instance()->GetEvent("new_v_bound");
  e_v_bound->set_type(visualizer::Event::k2D);
  e_v_bound->mutable_color()->set_r(0.8);
  e_v_bound->mutable_color()->set_g(0.0);
  e_v_bound->mutable_color()->set_b(0.0);
  e_v_bound->mutable_color()->set_a(0.6);
  auto v_bound__ = e_v_bound->add_polyline();

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x), ans->set_y(y), ans->set_z(0);
  };

  for (size_t i = 0; i < v_bound.size(); ++i) {
    set_pt(v_bound__->add_point(), i * 0.4, v_bound[i]);
  }
}

void VisGoalTV(const std::vector<STPoint>& goal_v,
               const std::vector<STPoint>& goal_s) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto e_vt_l = vis::EventSender::Instance()->GetEvent("goal_upper");
  e_vt_l->set_type(visualizer::Event::k2D);
  e_vt_l->mutable_color()->set_r(0.8);
  e_vt_l->mutable_color()->set_g(0.0);
  e_vt_l->mutable_color()->set_b(0.0);
  e_vt_l->mutable_color()->set_a(0.6);
  auto vt_l_pl = e_vt_l->add_polyline();

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x), ans->set_y(y), ans->set_z(0);
  };

  for (size_t i = 0; i < goal_v.size(); ++i) {
    set_pt(vt_l_pl->add_point(), goal_s[i].t(), goal_v[i].s());
  }
}

void VisObstacles(const DecisionData& decision_data) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event_virtual =
      vis::EventSender::Instance()->GetEvent("virtual_obstacles");
  event_virtual->set_type(visualizer::Event::k3D);
  event_virtual->add_attribute(visualizer::Event::kOdom);
  event_virtual->mutable_color()->set_r(0.8);
  event_virtual->mutable_color()->set_g(0.7);
  event_virtual->mutable_color()->set_b(0.2);
  event_virtual->mutable_color()->set_a(0.6);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x()), ans->set_y(p.y()), ans->set_z(0);
  };

  std::vector<Vec2d> pts{};
  for (auto obs : decision_data.virtual_obstacle()) {
    auto polygon = event_virtual->add_polygon();
    obs->bounding_box().get_all_corners(&pts);
    for (auto& p : pts) set_pt(polygon->add_point(), p);
  }

  auto event_dynamic =
      vis::EventSender::Instance()->GetEvent("dynamic_obstacles");
  event_dynamic->set_type(visualizer::Event::k3D);
  event_dynamic->add_attribute(visualizer::Event::kOdom);
  event_dynamic->mutable_color()->set_r(0.8);
  event_dynamic->mutable_color()->set_g(0.7);
  event_dynamic->mutable_color()->set_b(0.2);
  event_dynamic->mutable_color()->set_a(0.6);

  pts.clear();
  for (auto obs : decision_data.dynamic_obstacle()) {
    auto polygon = event_dynamic->add_polygon();
    obs->bounding_box().get_all_corners(&pts);
    for (auto& p : pts) set_pt(polygon->add_point(), p);
  }

  auto event_static =
      vis::EventSender::Instance()->GetEvent("static_obstacles");
  event_static->set_type(visualizer::Event::k3D);
  event_static->add_attribute(visualizer::Event::kOdom);
  event_static->mutable_color()->set_r(0.8);
  event_static->mutable_color()->set_g(0.7);
  event_static->mutable_color()->set_b(0.2);
  event_static->mutable_color()->set_a(0.6);

  pts.clear();
  for (auto obs : decision_data.static_obstacle()) {
    auto polygon = event_static->add_polygon();
    obs->bounding_box().get_all_corners(&pts);
    for (auto& p : pts) set_pt(polygon->add_point(), p);
  }
}

void VisOptBefore(const std::vector<MotorwaySpeedForIter::OptVariables>& vs) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto e_sv = vis::EventSender::Instance()->GetEvent("3order_speed_sv");
  e_sv->set_type(visualizer::Event::k2D);
  e_sv->mutable_color()->set_r(0.8);
  e_sv->mutable_color()->set_g(0.0);
  e_sv->mutable_color()->set_b(0.0);
  e_sv->mutable_color()->set_a(0.6);
  auto sv_pl = e_sv->add_polyline();

  auto e_sa = vis::EventSender::Instance()->GetEvent("3order_speed_sa");
  e_sa->set_type(visualizer::Event::k2D);
  e_sa->mutable_color()->set_r(0.8);
  e_sa->mutable_color()->set_g(0.0);
  e_sa->mutable_color()->set_b(0.0);
  e_sa->mutable_color()->set_a(0.6);
  auto sa_pl = e_sa->add_polyline();
  auto e_sj = vis::EventSender::Instance()->GetEvent("3order_speed_sj");
  e_sj->set_type(visualizer::Event::k2D);
  e_sj->mutable_color()->set_r(0.8);
  e_sj->mutable_color()->set_g(0.0);
  e_sj->mutable_color()->set_b(0.0);
  e_sj->mutable_color()->set_a(0.6);
  auto sj_pl = e_sj->add_polyline();

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x), ans->set_y(y), ans->set_z(0);
  };
  double time{0.0};
  for (auto& v : vs) {
    // set_pt(sv_pl->add_point(), v.xk.state_s, v.xk.state_v);

    set_pt(sv_pl->add_point(), time * 0.4, v.xk.state_v);
    set_pt(sa_pl->add_point(), time * 0.4, v.xk.state_a);
    set_pt(sj_pl->add_point(), time * 0.4, v.uk.control_jerk);
    time++;
  }
}

// draw before opt
void VisOptAfter(const double delta_t,
                 const std::vector<double>& upper_s_bounds,
                 const std::vector<double>& lower_s_bounds,
                 const std::vector<double>& upper_v_bounds,
                 const std::vector<double>& lower_v_bounds,
                 const std::vector<double>& upper_a_bounds,
                 const std::vector<double>& lower_a_bounds,
                 const std::vector<double>& upper_jerk_bounds,
                 const std::vector<double>& lower_jerk_bounds,
                 const std::vector<STPoint>& goal_s,
                 const std::vector<STPoint>& goal_v,
                 const std::vector<STPoint>& goal_a) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x), ans->set_y(y), ans->set_z(0);
  };

  auto e_vt_l = vis::EventSender::Instance()->GetEvent("check_goal__v");
  e_vt_l->set_type(visualizer::Event::k2D);
  e_vt_l->mutable_color()->set_r(0.8);
  e_vt_l->mutable_color()->set_g(0.0);
  e_vt_l->mutable_color()->set_b(0.0);
  e_vt_l->mutable_color()->set_a(0.6);
  auto vt_l_pz = e_vt_l->add_polyline();
  for (size_t i = 0; i < goal_v.size(); ++i) {
    set_pt(vt_l_pz->add_point(), delta_t * i, goal_v[i].s());
  }

  auto e_bound_v = vis::EventSender::Instance()->GetEvent("check_bound__v");
  e_bound_v->set_type(visualizer::Event::k2D);
  e_bound_v->mutable_color()->set_r(0.8);
  e_bound_v->mutable_color()->set_g(0.0);
  e_bound_v->mutable_color()->set_b(0.0);
  e_bound_v->mutable_color()->set_a(0.6);
  auto vt_l_pv = e_bound_v->add_polyline();
  auto vt_l_lv = e_bound_v->add_polyline();
  for (size_t i = 0; i < lower_v_bounds.size(); ++i) {
    set_pt(vt_l_pv->add_point(), delta_t * (i + 1), lower_v_bounds[i]);
    set_pt(vt_l_lv->add_point(), delta_t * (i + 1), upper_v_bounds[i]);
  }

  auto e_bound_s = vis::EventSender::Instance()->GetEvent("check_bound__s");
  e_bound_s->set_type(visualizer::Event::k2D);
  e_bound_s->mutable_color()->set_r(0.8);
  e_bound_s->mutable_color()->set_g(0.0);
  e_bound_s->mutable_color()->set_b(0.0);
  e_bound_s->mutable_color()->set_a(0.6);
  auto vt_l_pl = e_bound_s->add_polyline();
  auto vt_l_pu = e_bound_s->add_polyline();
  for (size_t i = 0; i < lower_s_bounds.size(); ++i) {
    set_pt(vt_l_pl->add_point(), delta_t * (i + 1), lower_s_bounds[i]);
    set_pt(vt_l_pu->add_point(), delta_t * (i + 1), upper_s_bounds[i]);
  }

  auto e_goal_s = vis::EventSender::Instance()->GetEvent("draw goal s");
  e_goal_s->set_type(visualizer::Event::k2D);
  e_goal_s->mutable_color()->set_r(0.8);
  e_goal_s->mutable_color()->set_g(0.0);
  e_goal_s->mutable_color()->set_b(0.0);
  e_goal_s->mutable_color()->set_a(0.6);
  auto vt_l_ps = e_goal_s->add_polyline();
  for (size_t i = 0; i < goal_s.size(); ++i) {
    set_pt(vt_l_ps->add_point(), delta_t * (i + 1), goal_s[i].s());
  }
}

}  // namespace

MotorwaySpeedOptimizerForIter::MotorwaySpeedOptimizerForIter() {
  name_ = "MotorwaySpeedOptimizerForIter";

  speed_limit_ =
      DataCenter::Instance()->mutable_behavior_speed_limits()->speed_limit();
}

MotorwaySpeedOptimizerForIter::~MotorwaySpeedOptimizerForIter() {
  motorway_mpc_.reset();
  upper_s_bounds_.clear();
  lower_s_bounds_.clear();
  upper_v_bounds_.clear();
  lower_v_bounds_.clear();
  upper_a_bounds_.clear();
  lower_a_bounds_.clear();
  upper_jerk_bounds_.clear();
  lower_jerk_bounds_.clear();
  goal_s_.clear();
  goal_v_.clear();
  goal_a_.clear();
  opt_variables_.clear();
  Reset();
}

ErrorCode MotorwaySpeedOptimizerForIter::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  if (config::PlanningConfig::Instance()
          ->plan_config()
          .common.only_for_single_cipv) {
    LOG_INFO("only for single cipv, iter cipv not use.");
    return ErrorCode::PLANNING_OK;
  }

  VisObstacles(task_info.current_frame()->planning_data().decision_data());
  auto& frame = task_info.current_frame();
  auto outside_planner_data_ptr =
      task_info.current_frame()->mutable_outside_planner_data();
  if (outside_planner_data_ptr->path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  if (outside_planner_data_ptr->speed_succeed_tasks > 0) {
    return ErrorCode::PLANNING_OK;
  }
  if (outside_planner_data_ptr->speed_slow_down) {
    LOG_INFO("skip task, speed_slow_down");
    return ErrorCode::PLANNING_OK;
  }

  auto ret = Optimize(task_info.current_frame()->inside_planner_data(),
                      task_info.current_frame()
                          ->mutable_planning_data()
                          ->mutable_decision_data(),
                      outside_planner_data_ptr);
  if (ret != ErrorCode::PLANNING_OK) {
    outside_planner_data_ptr->speed_fail_tasks += 1;
    return ret;
  }
  outside_planner_data_ptr->speed_succeed_tasks += 1;

  return ErrorCode::PLANNING_OK;
}

ErrorCode MotorwaySpeedOptimizerForIter::Optimize(
    const InsidePlannerData& inside_data, DecisionData* const decision_data,
    OutsidePlannerData* const outside_data) {
  if (config::PlanningConfig::Instance()
          ->planning_research_config()
          .only_for_backup_speed_planner) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  std::vector<SpeedPoint> speed_vector{};
  if (STGenerator(inside_data, decision_data, outside_data, &speed_vector) !=
      ErrorCode::PLANNING_OK) {
    LOG_ERROR("st generator failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  outside_data->speed_data->set_speed_vector(speed_vector);

  LOG_INFO("speed_optimizer result is:");
  for (size_t index = 0; index < speed_vector.size(); index += 20) {
    LOG_INFO("t, s, v, a, j = {:.3}, {:.3}, {:.3}, {:.3}, {:.3f}",
             speed_vector[index].t(), speed_vector[index].s(),
             speed_vector[index].v(), speed_vector[index].a(),
             speed_vector[index].j());
  }

  return ErrorCode::PLANNING_OK;
}

ErrorCode MotorwaySpeedOptimizerForIter::STGenerator(
    const InsidePlannerData& inside_data, DecisionData* const decision_data,
    OutsidePlannerData* const outside_data,
    std::vector<SpeedPoint>* speed_points) {
  double total_time = 5.0;
  n_ = static_cast<std::size_t>(total_time / delta_t_);

  if (!CalInitStateAndControl(inside_data, decision_data, outside_data)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  // 2. bounds(upper_s,v,a,jerk_bounds_; lower_s,v,a,jerk_bounds_)
  if (!CalBounds(inside_data, decision_data, outside_data)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  // 3. goal: T-S, T-V(goal_upper_s,v_; goal_lower_s,v_)
  if (!CalGoalSV(inside_data, decision_data, outside_data)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  VisOptAfter(delta_t_, upper_s_bounds_, lower_s_bounds_, upper_v_bounds_,
              lower_v_bounds_, upper_a_bounds_, lower_a_bounds_,
              upper_jerk_bounds_, lower_jerk_bounds_, goal_s_, goal_v_,
              goal_a_);
  // 4. solve(result delta_t_{0.4})
  motorway_mpc_ = std::make_shared<MotorwaySpeedMPCForIter>(
      delta_t_, n_, MotorwaySpeedForIter::nx, MotorwaySpeedForIter::nu,
      MotorwaySpeedForIter::npc, MotorwaySpeedForIter::ns, init_state_,
      init_control_, speed_limit_, upper_s_bounds_, lower_s_bounds_,
      upper_v_bounds_, lower_v_bounds_, upper_a_bounds_, lower_a_bounds_,
      upper_jerk_bounds_, lower_jerk_bounds_, goal_s_, goal_v_, goal_a_);
  if (!motorway_mpc_->Process()) {
    LOG_ERROR("Motorway speed opt failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  opt_variables_ = motorway_mpc_->optimal_solution();
  VisOptBefore(opt_variables_);

  // 5. dense: [s, t, v, a, jerk](dense_time_{0.05})
  std::vector<MotorwaySpeedForIter::Control> control_variables{};
  for (const auto& variable : opt_variables_) {
    control_variables.emplace_back(variable.uk);
  }
  MotorwaySpeedForIter::A A_dense_t{};
  MotorwaySpeedForIter::B B_dense_t{};
  MotorwaySpeedForIter::StateVector state_vector{};
  MotorwaySpeedForIter::ControlVector control_vector{};
  for (std::size_t i = 0; i < opt_variables_.size(); ++i) {
    speed_points->emplace_back(
        SpeedPoint(opt_variables_[i].xk.state_s, i * delta_t_,
                   opt_variables_[i].xk.state_v, opt_variables_[i].xk.state_a,
                   opt_variables_[i].uk.control_jerk));
    state_vector = MotorwaySpeedForIter::StateToVector(opt_variables_[i].xk);
    control_vector =
        MotorwaySpeedForIter::ControlToVector(opt_variables_[i].uk);
    for (std::size_t j = 1;
         j < static_cast<std::size_t>(delta_t_ / dense_time_); ++j) {
      if (i * delta_t_ + j * dense_time_ > total_time) {
        break;
      }
      A_dense_t.row(0) << 1, j * dense_time_,
          0.5 * (j * dense_time_ * j * dense_time_);
      A_dense_t.row(1) << 0, 1, j * dense_time_;
      A_dense_t.row(2) << 0, 0, 1;
      B_dense_t << 1. / 6 *
                       (j * dense_time_ * j * dense_time_ * j * dense_time_),
          1. / 2 * (j * dense_time_ * j * dense_time_), j * dense_time_;

      MotorwaySpeedForIter::State state_dense_t =
          MotorwaySpeedForIter::VectorToState(A_dense_t * state_vector +
                                              B_dense_t * control_vector);
      speed_points->emplace_back(
          SpeedPoint(state_dense_t.state_s, i * delta_t_ + j * dense_time_,
                     state_dense_t.state_v, state_dense_t.state_a,
                     opt_variables_[i].uk.control_jerk));
    }
  }

  // 6. check
  if (!speed_planner_common::SpeedDataSanityCheck(speed_points)) {
    LOG_ERROR("Fail to smooth st points by third order spline.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  const double station_error =
      data_center_->control_command_msg.ptr->contrl_context()
          .long_ctrl_ctx()
          .station_error();
  if (!speed_planner_common::FinalObstacleCollisionSanityCheck(
          inside_data.init_point, *(outside_data->path_data),
          outside_data->motorway_speed_obstacle_context,
          outside_data->motorway_speed_obstacle_context.ignore_dynamic_obs_id,
          outside_data->motorway_speed_obstacle_context.ignore_static_obs_id,
          station_error, *speed_points)) {
    LOG_ERROR("Final_obstacle_collision_sannty_check failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  return ErrorCode::PLANNING_OK;
}

//{0.0, v0, a0}, {j0}
bool MotorwaySpeedOptimizerForIter::CalInitStateAndControl(
    const InsidePlannerData& inside_data, DecisionData* const decision_data,
    OutsidePlannerData* const outside_data) {
  init_state_.state_s = 0.0;
  init_state_.state_v = inside_data.init_point.velocity() < 0.0
                            ? 0.0
                            : inside_data.init_point.velocity();
  init_state_.state_a = inside_data.init_point.acceleration();
  init_control_.control_jerk = inside_data.init_point.jerk();

  // TEST
  LOG_INFO("init_state: {:.3}, {:.3}, {:.3}", init_state_.state_s,
           init_state_.state_v, init_state_.state_a);
  LOG_INFO("init_control: {:.3}", init_control_.control_jerk);

  return true;
}

// generate upper_s,v,a,j_bounds_ and lower_s,v,a,j_bounds_
bool MotorwaySpeedOptimizerForIter::CalBounds(
    const InsidePlannerData& inside_data, DecisionData* const decision_data,
    OutsidePlannerData* const outside_data) {
  const auto& upper_s_bounds =
      outside_data->motorway_speed_context.tunnel_decision.upper_s_tunnel;
  const auto& lower_s_bounds =
      outside_data->motorway_speed_context.tunnel_decision.lower_s_tunnel;
  const auto& lower_v_bounds =
      outside_data->motorway_speed_context.tunnel_decision.lower_v_tunnel;
  const auto& upper_v_bounds =
      outside_data->motorway_speed_context.tunnel_decision.upper_v_tunnel;
  if (upper_s_bounds.size() < 4) {
    LOG_WARN("s bounds size < 4, must check multi cipv search.");
    return false;
  }
  tk::spline upper_s_bounds_spline, lower_s_bounds_spline;
  std::vector<double> upper_bounds_t_vec{}, upper_bounds_s_vec{},
      lower_bounds_t_vec{}, lower_bounds_s_vec{};
  for (const auto& bound : upper_s_bounds) {
    upper_bounds_t_vec.emplace_back(bound.t());
    upper_bounds_s_vec.emplace_back(bound.s());
  }
  for (const auto& bound : lower_s_bounds) {
    lower_bounds_t_vec.emplace_back(bound.t());
    lower_bounds_s_vec.emplace_back(bound.s());
  }
  upper_s_bounds_spline.set_points(upper_bounds_t_vec, upper_bounds_s_vec,
                                   tk::spline::spline_type::linear);
  lower_s_bounds_spline.set_points(lower_bounds_t_vec, lower_bounds_s_vec,
                                   tk::spline::spline_type::linear);

  upper_s_bounds_.resize(n_ + 1);
  lower_s_bounds_.resize(n_ + 1);
  for (std::size_t i = 0; i <= n_; ++i) {
    upper_s_bounds_[i] = upper_s_bounds_spline(i * delta_t_);
    lower_s_bounds_[i] = lower_s_bounds_spline(i * delta_t_);
  }

  tk::spline upper_v_bounds_spline, lower_v_bounds_spline;
  std::vector<double> vt_upper_bounds_t_vec{}, vt_upper_bounds_v_vec{},
      vt_lower_bounds_t_vec{}, vt_lower_bounds_v_vec{};

  for (const auto& bound : upper_v_bounds) {
    vt_upper_bounds_t_vec.emplace_back(bound.t());
    vt_upper_bounds_v_vec.emplace_back(bound.s());
  }

  for (const auto& bound : lower_v_bounds) {
    vt_lower_bounds_t_vec.emplace_back(bound.t());
    vt_lower_bounds_v_vec.emplace_back(0.0);
  }
  upper_v_bounds_spline.set_points(vt_upper_bounds_t_vec, vt_upper_bounds_v_vec,
                                   tk::spline::spline_type::linear);

  lower_v_bounds_spline.set_points(vt_lower_bounds_t_vec, vt_lower_bounds_v_vec,
                                   tk::spline::spline_type::linear);

  upper_v_bounds_.resize(n_ + 1,
                         DataCenter::Instance()->drive_strategy_max_speed());
  lower_v_bounds_.resize(n_ + 1, 0.0);

  for (std::size_t i = 0; i <= n_; ++i) {
    upper_v_bounds_[i] = upper_v_bounds_spline(i * delta_t_) + 2;
    lower_v_bounds_[i] = lower_v_bounds_spline(i * delta_t_);
  }

  upper_a_bounds_ = std::vector<double>(
      n_ + 1,
      config::PlanningConfig::Instance()->plan_config().common.max_av_plus);
  lower_a_bounds_ = std::vector<double>(n_ + 1, k_min_acceleration);
  VisBoundVT(upper_v_bounds_);
  upper_jerk_bounds_ = std::vector<double>(n_ + 1, k_max_jerk);
  lower_jerk_bounds_ = std::vector<double>(n_ + 1, k_min_jerk);

  return true;
}

bool MotorwaySpeedOptimizerForIter::CalGoalSV(
    const InsidePlannerData& inside_data, DecisionData* const decision_data,
    OutsidePlannerData* const outside_data) {
  const auto& goal_s = outside_data->motorway_speed_context.final_goal_s;
  const auto& goal_v = outside_data->motorway_speed_context.final_goal_v;
  const auto& goal_a = outside_data->motorway_speed_context.final_goal_a;

  tk::spline goal_s_spline;
  std::vector<double> goal_s_t_vec{}, goal_s_s_vec{};
  for (int i = 0; i < goal_s.size(); i++) {
    goal_s_s_vec.emplace_back(goal_s[i].s());
    goal_s_t_vec.emplace_back(goal_s[i].t());
  }
  goal_s_spline.set_points(goal_s_t_vec, goal_s_s_vec,
                           tk::spline::spline_type::linear);

  tk::spline goal_v_spline;
  std::vector<double> goal_v_t_vec{}, goal_v_v_vec{};
  for (int i = 0; i < goal_v.size(); i++) {
    goal_v_v_vec.emplace_back(goal_v[i].s());
    goal_v_t_vec.emplace_back(goal_v[i].t());
  }
  goal_v_spline.set_points(goal_v_t_vec, goal_v_v_vec,
                           tk::spline::spline_type::linear);

  tk::spline goal_a_spline;
  std::vector<double> goal_a_t_vec{}, goal_a_a_vec{};
  for (int i = 0; i < goal_a.size(); i++) {
    goal_a_a_vec.emplace_back(goal_a[i].s());
    goal_a_t_vec.emplace_back(goal_a[i].t());
  }
  goal_a_spline.set_points(goal_a_t_vec, goal_a_a_vec,
                           tk::spline::spline_type::linear);

  goal_s_.resize(n_ + 1);
  goal_v_.resize(n_ + 1);
  goal_a_.resize(n_ + 1);
  for (std::size_t i = 0; i <= n_; ++i) {
    double curr_t = i * delta_t_;
    goal_s_[i] = std::move(STPoint(goal_s_spline(curr_t), curr_t));
    goal_v_[i] = std::move(STPoint(goal_v_spline(curr_t), curr_t));
    goal_a_[i] = std::move(STPoint(goal_a_spline(curr_t), curr_t));
  }

  VisGoalTV(goal_v_, goal_s_);

  return true;
}

}  // namespace planning
}  // namespace neodrive
