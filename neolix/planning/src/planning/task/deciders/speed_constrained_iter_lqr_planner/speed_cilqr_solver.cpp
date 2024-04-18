#include "speed_cilqr_solver.h"

#include "common/visualizer_event/visualizer_event.h"
#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {
namespace {
void VisilqrDeductionResult(
    const std::vector<CilqrModel::CilqrState>& ego_deduction) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto e_iter_result_st =
      vis::EventSender::Instance()->GetEvent("lqr_result_st");
  e_iter_result_st->set_type(visualizer::Event::k2D);
  e_iter_result_st->mutable_color()->set_r(0.8);
  e_iter_result_st->mutable_color()->set_g(0.0);
  e_iter_result_st->mutable_color()->set_b(0.0);
  e_iter_result_st->mutable_color()->set_a(0.6);
  auto iter_result_st = e_iter_result_st->add_polyline();

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x), ans->set_y(y), ans->set_z(0);
  };
  for (int i = 0; i < ego_deduction.size(); i++) {
    set_pt(iter_result_st->add_point(), i * 0.1, ego_deduction[i].state_s);
  }
  auto e_iter_result_vt =
      vis::EventSender::Instance()->GetEvent("lqr_result_vt");
  e_iter_result_vt->set_type(visualizer::Event::k2D);
  e_iter_result_vt->mutable_color()->set_r(0.8);
  e_iter_result_vt->mutable_color()->set_g(0.0);
  e_iter_result_vt->mutable_color()->set_b(0.0);
  e_iter_result_vt->mutable_color()->set_a(0.6);
  auto iter_result_vt = e_iter_result_vt->add_polyline();

  for (int i = 0; i < ego_deduction.size(); i++) {
    set_pt(iter_result_vt->add_point(), i * 0.1, ego_deduction[i].state_v);
  }
}

void VisilqrDeductionResultF(
    const std::vector<CilqrModel::CilqrState>& ego_deduction) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto e_iter_result_st =
      vis::EventSender::Instance()->GetEvent("lqr_result_st_final");
  e_iter_result_st->set_type(visualizer::Event::k2D);
  e_iter_result_st->mutable_color()->set_r(0.12);
  e_iter_result_st->mutable_color()->set_g(0.82);
  e_iter_result_st->mutable_color()->set_b(0.0);
  e_iter_result_st->mutable_color()->set_a(0.6);
  auto iter_result_st = e_iter_result_st->add_polyline();

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x), ans->set_y(y), ans->set_z(0);
  };
  for (int i = 0; i < ego_deduction.size(); i++) {
    set_pt(iter_result_st->add_point(), i * 0.1, ego_deduction[i].state_s);
  }
  auto e_iter_result_vt =
      vis::EventSender::Instance()->GetEvent("lqr_result_vt_final");
  e_iter_result_vt->set_type(visualizer::Event::k2D);
  e_iter_result_vt->mutable_color()->set_r(0.0);
  e_iter_result_vt->mutable_color()->set_g(0.56);
  e_iter_result_vt->mutable_color()->set_b(1.0);
  e_iter_result_vt->mutable_color()->set_a(0.6);
  auto iter_result_vt = e_iter_result_vt->add_polyline();

  for (int i = 0; i < ego_deduction.size(); i++) {
    set_pt(iter_result_vt->add_point(), i * 0.1, ego_deduction[i].state_v);
  }
}

}  // namespace

std::shared_ptr<SpeedCilqrModel> RunStepCilqr::cilqr_model_ =
    std::make_shared<SpeedCilqrModel>("cilqr_speed_model",
                                      config::PlanningConfig::Instance()
                                          ->planning_research_config()
                                          .speed_cilqr.step_time);

RunStepCilqr::RunStepCilqr(
    const CilqrModel::CilqrState& ego_state,
    const std::vector<std::shared_ptr<SpeedCilqrObsProcess>>& sorted_obs_list,
    const RefSpeedPlan& ref_speed_plan, const size_t& steps) {
  const auto& speed_cilqr_conf = config::PlanningConfig::Instance()
                                     ->planning_research_config()
                                     .speed_cilqr;
  reset();
  ego_state_ = ego_state;

  obs_list_ = sorted_obs_list;
  ref_speed_plan_ = ref_speed_plan;
  delta_t_ = speed_cilqr_conf.step_time;
  steps_ = steps;
  max_iters_ = speed_cilqr_conf.max_iters;
}

void RunStepCilqr::reset() {
  obs_list_.clear();
  ego_state_.SetZero();
  is_illness_ = false;
}

std::vector<CilqrModel::CilqrState> RunStepCilqr::Optimize() {
  auto result = GetOptimalControlSeq();
  ClampErrorData(result);
  if (CheckIllness(result)) {
    is_illness_ = true;
  }
  VisilqrDeductionResultF(result);
  for (size_t i = 0; i < result.size(); i++) {
    if ((i + 1) % 3 == 0 || i == 0) {
      LOG_INFO(
          "cilqr -- t : {:.6f} , v : {:.3f}, a : {:.3f}, s : "
          "{:.6f}",
          i * 0.1, result[i].state_v, result[i].state_a, result[i].state_s);
    }
  }
  return result;
}

bool RunStepCilqr::CheckIllness(
    std::vector<CilqrModel::CilqrState>& ego_deduction_data) {
  int error_cnt = 0;
  const auto& speed_cilqr_conf = config::PlanningConfig::Instance()
                                     ->planning_research_config()
                                     .speed_cilqr;
  for (int i = 1; i < ego_deduction_data.size(); i++) {
    if (math::Sign((ego_deduction_data[i].state_a -
                    ego_deduction_data[i - 1].state_a)) &&
        math::Sign((ego_deduction_data[i].state_a -
                    ego_deduction_data[i - 1].state_a) /
                       delta_t_,
                   speed_cilqr_conf.min_jerk_limit / 2,
                   speed_cilqr_conf.max_jerk_limit / 2)) {
      error_cnt++;
    }
  }
  LOG_INFO_IF(error_cnt > kNoise, "error_cnt check->{}", error_cnt);
  return error_cnt > kNoise;
}

std::vector<CilqrModel::CilqrState> RunStepCilqr::ClampErrorData(
    std::vector<CilqrModel::CilqrState>& ego_deduction_data) {
  int index = 100000;
  for (int i = 0; i < ego_deduction_data.size(); i++) {
    if (ego_deduction_data[i].state_v <= 0.0) {
      // clamp data
      index = i;
      break;
    }
  }
  if (index >= 1 && index < ego_deduction_data.size()) {
    for (int i = index; i < ego_deduction_data.size(); i++) {
      ego_deduction_data[i].state_v = 0;
      ego_deduction_data[i].state_s = ego_deduction_data[index - 1].state_s;
      ego_deduction_data[i].state_a = -1;
    }
  }
  if (index == 0) {
    LOG_INFO(
        "last frame cal this time stap v < 0 , need check ilqr speed is "
        "reasonable");
    if (ego_deduction_data[5].state_v > 0.0) {
      return ego_deduction_data;
    } else {
      for (int i = 0; i < ego_deduction_data.size(); i++) {
        ego_deduction_data[i].state_v = 0.0;
        ego_deduction_data[i].state_s = 0.0;
        ego_deduction_data[i].state_a = -1.0;
      }
    }
  }
  return ego_deduction_data;
}

void RunStepCilqr::UpdateData(
    std::vector<CilqrModel::CilqrState>& rough_state_seq,
    std::vector<CilqrModel::CilqrControl>& rough_control_seq,
    const std::pair<std::vector<double>, std::vector<CilqrModel::X_state>>&
        forward_result) {
  rough_control_seq.clear();
  rough_state_seq.clear();
  for (int i = 0; i < forward_result.first.size(); i++) {
    CilqrModel::CilqrState ego_forward_state = {forward_result.second[i](0),
                                                forward_result.second[i](1),
                                                forward_result.second[i](2)};
    rough_state_seq.emplace_back(ego_forward_state);

    CilqrModel::CilqrControl control_res{forward_result.first[i]};
    rough_control_seq.emplace_back(control_res);
  }
  CilqrModel::CilqrState ego_forward_state = {forward_result.second.back()(0),
                                              forward_result.second.back()(1),
                                              forward_result.second.back()(2)};
  rough_state_seq.emplace_back(ego_forward_state);
}

std::vector<CilqrModel::CilqrState> RunStepCilqr::GetOptimalControlSeq() {
  std::vector<CilqrModel::CilqrControl> rough_control_seq{};
  rough_control_seq.resize(steps_);
  for (int i = 0; i < steps_; ++i) {
    rough_control_seq[i].control_jerk =
        10 * (ref_speed_plan_.goal_a_[i + 1].goal_t_a.s() -
              ref_speed_plan_.goal_a_[i].goal_t_a.s());
  }
  std::vector<CilqrModel::CilqrState> rough_state_seq =
      GetRoughStateSeq(ego_state_, rough_control_seq);

  VisilqrDeductionResult(rough_state_seq);

  double lambda = 1.0;
  double lambda_factor = 10.0;
  double max_lambda = 1000.0;

  const auto& speed_cilqr_conf = config::PlanningConfig::Instance()
                                     ->planning_research_config()
                                     .speed_cilqr;

  double cost_old = std::numeric_limits<double>::max();

  int max_iter = 0;
  for (int iter = 0; iter < speed_cilqr_conf.max_iters; ++iter) {
    std::pair<std::vector<CilqrModel::K>, std::vector<CilqrModel::k>>
        backward_result =
            BackwardPass(rough_state_seq, rough_control_seq, lambda);

    auto forward_result =
        ForwardPass(rough_state_seq, rough_control_seq, backward_result);

    double cost_new = GetTotalCost(rough_state_seq, rough_control_seq);

    if (cost_new < cost_old) {
      UpdateData(rough_state_seq, rough_control_seq, forward_result);
      lambda = lambda * lambda_factor;
      if (std::abs(cost_new - cost_old) < 0.1) {
        LOG_INFO("Tolerance reached");
        break;
      }
    } else {
      UpdateData(rough_state_seq, rough_control_seq, forward_result);
      lambda = lambda / lambda_factor;
    }

    cost_old = cost_new;
    // VisilqrDeductionResult(rough_state_seq);
    max_iter++;
  }
  LOG_INFO("max iter times : {}", max_iter);
  return rough_state_seq;
}

std::pair<std::vector<double>, std::vector<CilqrModel::X_state>>
RunStepCilqr::ForwardPass(
    const std::vector<CilqrModel::CilqrState>& rough_state_seq,
    const std::vector<CilqrModel::CilqrControl>& rough_control_seq,
    const std::pair<std::vector<CilqrModel::K>, std::vector<CilqrModel::k>>&
        best_control_rate) {
  std::vector<CilqrModel::X_state> ego_deduction_seq;

  std::vector<CilqrModel::X_state> old_state_seq;
  std::vector<double> ego_control_seq{};
  for (int i = 0; i < rough_state_seq.size(); i++) {
    CilqrModel::X_state x_state;
    x_state << rough_state_seq[i].state_s, rough_state_seq[i].state_v,
        rough_state_seq[i].state_a;
    old_state_seq.emplace_back(x_state);
  }

  CilqrModel::X_state x_state_new;
  x_state_new << rough_state_seq.front().state_s,
      rough_state_seq.front().state_v, rough_state_seq.front().state_a;

  ego_deduction_seq.emplace_back(x_state_new);

  for (int i = 0; i < steps_; i++) {
    double best_control =
        rough_control_seq[i].control_jerk + best_control_rate.second[i](0, 0) +
        (best_control_rate.first[i] * (x_state_new - old_state_seq[i]))(0, 0);

    ego_control_seq.emplace_back(best_control);
    x_state_new = ForwardSimulation(x_state_new, best_control);
    if (i > 1) {
      if (x_state_new(0, 0) <= ego_deduction_seq.back()(0, 0))
        x_state_new(0, 0) = ego_deduction_seq[i - 1](0, 0);
      if (x_state_new(1, 0) <= 0) {
        x_state_new(1, 0) = 0;
        x_state_new(2, 0) = 0;
        ego_control_seq.back() = 0;
      }
    }

    ego_deduction_seq.emplace_back(x_state_new);
  }

  return std::make_pair(ego_control_seq, ego_deduction_seq);
}

CilqrModel::X_state RunStepCilqr::ForwardSimulation(
    const CilqrModel::X_state& x_state_old, const double& control_jerk) {
  return cilqr_model_->model_matrix().A * x_state_old +
         cilqr_model_->model_matrix().B * control_jerk;
}

std::vector<CilqrModel::CilqrState> RunStepCilqr::GetRoughStateSeq(
    const CilqrModel::CilqrState& ego_state,
    const std::vector<CilqrModel::CilqrControl>& rough_control_seq) {
  CilqrModel::CilqrState ego_state_next = ego_state;

  std::vector<CilqrModel::CilqrState> rough_state_seq{};
  rough_state_seq.clear();
  for (int i = 0; i <= rough_control_seq.size(); ++i) {
    ego_state_next.state_a = ref_speed_plan_.goal_a_[i].goal_t_a.s();
    ego_state_next.state_v = ref_speed_plan_.goal_v_[i].goal_t_v.s();
    ego_state_next.state_s = ref_speed_plan_.goal_s_[i].goal_t_s.s();
    rough_state_seq.emplace_back(ego_state_next);
  }

  LOG_INFO("checkout rough state size : {}, rough control size : {}",
           rough_state_seq.size(), rough_control_seq.size());
  return rough_state_seq;
}

std::pair<std::vector<CilqrModel::K>, std::vector<CilqrModel::k>>
RunStepCilqr::BackwardPass(
    const std::vector<CilqrModel::CilqrState>& rough_state_seq,
    const std::vector<CilqrModel::CilqrControl>& rough_control_seq,
    const double& lambda) {
  auto total_cost = GetCostDerivatives(rough_state_seq, rough_control_seq);

  CilqrModel::V_x V_x = total_cost.l_x.back();
  CilqrModel::V_xx V_xx = total_cost.l_xx.back();

  std::vector<CilqrModel::k> k_seq;
  k_seq.resize(steps_);
  std::vector<CilqrModel::K> K_seq;
  K_seq.resize(steps_);

  for (int i = (steps_ - 1); i >= 0; i--) {
    // back ward
    CilqrModel::Q_x Q_x =
        total_cost.l_x[i] +
        cilqr_model_->error_model_matrix().A.transpose() * V_x;
    CilqrModel::Q_u Q_u =
        total_cost.l_u[i] +
        cilqr_model_->error_model_matrix().B.transpose() * V_x;
    CilqrModel::Q_xx Q_xx =
        total_cost.l_xx[i] + cilqr_model_->error_model_matrix().A.transpose() *
                                 V_xx * cilqr_model_->error_model_matrix().A;
    CilqrModel::Q_ux Q_ux =
        total_cost.l_ux[i] + cilqr_model_->error_model_matrix().B.transpose() *
                                 V_xx * cilqr_model_->error_model_matrix().A;
    CilqrModel::Q_uu Q_uu =
        total_cost.l_uu[i] + cilqr_model_->error_model_matrix().B.transpose() *
                                 V_xx * cilqr_model_->error_model_matrix().B;
    CilqrModel::Q_uu Q_uu_eval = Q_uu;

    Q_uu(0, 0) = Q_uu(0, 0) + lambda;
    CilqrModel::Q_uu Q_uu_inv = Q_uu.inverse();
    k_seq[i] = -Q_uu_inv * Q_u;
    K_seq[i] = -Q_uu_inv * Q_ux;
    // update V_x V_xx
    V_x = Q_x + K_seq[i].transpose() * Q_uu * k_seq[i] +
          K_seq[i].transpose() * Q_u + Q_ux.transpose() * k_seq[i];
    V_xx = Q_xx + K_seq[i].transpose() * Q_uu * K_seq[i] +
           K_seq[i].transpose() * Q_ux + Q_ux.transpose() * K_seq[i];
  }
  return std::make_pair(K_seq, k_seq);
}

BackwardPerturbationPara RunStepCilqr::GetCostDerivatives(
    const std::vector<CilqrModel::CilqrState>& rough_state_seq,
    const std::vector<CilqrModel::CilqrControl>& rough_control_seq) {
  auto control_derivative =
      GetControlCostDerivatives(rough_state_seq, rough_control_seq);
  auto state_derivative =
      GetStateCostDerivatives(rough_state_seq, rough_control_seq);

  CilqrModel::l_ux l_ux;
  l_ux.setZero();
  std::vector<CilqrModel::l_ux> l_ux_seq(steps_, l_ux);

  BackwardPerturbationPara backward_perturbation_para{
      state_derivative.l_x, state_derivative.l_xx, control_derivative.l_u,
      control_derivative.l_uu, l_ux_seq};
  return backward_perturbation_para;
}

BackwardPerturbationParaState RunStepCilqr::GetStateCostDerivatives(
    const std::vector<CilqrModel::CilqrState>& rough_state_seq,
    const std::vector<CilqrModel::CilqrControl>& rough_control_seq) {
  const auto& speed_cilqr_conf = config::PlanningConfig::Instance()
                                     ->planning_research_config()
                                     .speed_cilqr;
  std::vector<CilqrModel::l_x> l_x_seq{};
  std::vector<CilqrModel::l_xx> l_xx_seq{};

  l_x_seq.clear();
  l_xx_seq.clear();
  l_x_seq.resize(steps_);
  l_xx_seq.resize(steps_);

  CilqrModel::l_x l_x;
  CilqrModel::l_xx l_xx;
  CilqrModel::X_state state_error_vector;
  for (int i = 0; i < steps_; i++) {
    double goal_s = ref_speed_plan_.goal_s_[i + 1].goal_t_s.s();
    double goal_v = ref_speed_plan_.goal_v_[i + 1].goal_t_v.s();
    double goal_a = ref_speed_plan_.goal_a_[i + 1].goal_t_a.s();

    state_error_vector(0, 0) = rough_state_seq[i + 1].state_s - goal_s;
    state_error_vector(1, 0) = rough_state_seq[i + 1].state_v - goal_v;
    state_error_vector(2, 0) = rough_state_seq[i + 1].state_a - goal_a;

    l_x = 2 * cilqr_model_->cost_para().state_cost_matrix * state_error_vector;
    l_xx = 2 * cilqr_model_->cost_para().state_cost_matrix;

    double error_s_min =
        speed_cilqr_conf.min_s_limit - rough_state_seq[i + 1].state_s;

    auto barrier_function_feedback_s_min =
        BarrireFunction(speed_cilqr_conf.q1_s, speed_cilqr_conf.q2_s,
                        error_s_min, -cilqr_model_->cost_para().P_s);

    double error_v_max =
        rough_state_seq[i + 1].state_v - speed_cilqr_conf.max_v_limit;

    double error_v_min =
        speed_cilqr_conf.min_v_limit - rough_state_seq[i + 1].state_v;

    auto barrier_function_feedback_v_max =
        BarrireFunction(8, 8, error_v_max, cilqr_model_->cost_para().P_v);

    auto barrier_function_feedback_v_min =
        BarrireFunction(speed_cilqr_conf.q1_v, speed_cilqr_conf.q2_v,
                        error_v_min, -cilqr_model_->cost_para().P_v);

    double error_a_max =
        rough_state_seq[i + 1].state_a - speed_cilqr_conf.max_a_limit;
    double error_a_min =
        speed_cilqr_conf.min_a_limit - rough_state_seq[i + 1].state_a;

    auto barrier_function_feedback_a_max =
        BarrireFunction(speed_cilqr_conf.q1_a, speed_cilqr_conf.q2_a,
                        error_a_max, cilqr_model_->cost_para().P_a);
    auto barrier_function_feedback_a_min =
        BarrireFunction(speed_cilqr_conf.q1_a, speed_cilqr_conf.q2_a,
                        error_a_min, -cilqr_model_->cost_para().P_a);

    l_x = l_x + barrier_function_feedback_s_min.exponential_cost_d +
          barrier_function_feedback_v_min.exponential_cost_d +
          barrier_function_feedback_v_max.exponential_cost_d +
          barrier_function_feedback_a_max.exponential_cost_d +
          barrier_function_feedback_a_min.exponential_cost_d;

    l_xx = l_xx + barrier_function_feedback_s_min.exponential_cost_dd +
           barrier_function_feedback_v_min.exponential_cost_dd +
           barrier_function_feedback_v_max.exponential_cost_dd +
           barrier_function_feedback_a_max.exponential_cost_dd +
           barrier_function_feedback_a_min.exponential_cost_dd;

    for (auto& obs : obs_list_) {
      std::vector<ObsDecisionBound> obs_low_bound =
          obs->InterpolatePointsForLow();
      std::vector<ObsDecisionBound> obs_up_bound =
          obs->InterpolatePointsForUpper();

      std::pair<double, double> time_range(obs_low_bound.front().time,
                                           obs_low_bound.back().time);
      if (i < 10 * time_range.first || i > 10 * time_range.second) {
        continue;
      }

      int time_index = i - time_range.first * 10;

      double obs_decision_width =
          (obs_up_bound[time_index].obs_s - obs_low_bound[time_index].obs_s) /
          2.0;
      double obs_decision_middle_s = 0.5 * obs_up_bound[time_index].obs_s +
                                     0.5 * obs_low_bound[time_index].obs_s;
      double c = 0.0;
      // 这里需要修改，后期不应该加buffer
      if ((rough_state_seq[i + 1].state_s - obs_decision_middle_s) >= 0) {
        // 本车在障碍物上 s-m>w+2  ----> w+m-s < 0
        c = obs_decision_width + obs_decision_middle_s -
            rough_state_seq[i + 1].state_s;

        auto barrier_function_feedback_s_to_obs =
            BarrireFunction(5, 7, c, -cilqr_model_->cost_para().P_s);
        l_x = l_x + barrier_function_feedback_s_to_obs.exponential_cost_d;
        l_xx = l_xx + barrier_function_feedback_s_to_obs.exponential_cost_dd;
      } else {
        // 本车在障碍物下  s<m-w-1   ---> s-m+w < 0
        c = rough_state_seq[i + 1].state_s - obs_decision_middle_s +
            obs_decision_width;
        auto barrier_function_feedback_s_to_obs =
            BarrireFunction(5, 7, c, cilqr_model_->cost_para().P_s);
        l_x = l_x + barrier_function_feedback_s_to_obs.exponential_cost_d;
        l_xx = l_xx + barrier_function_feedback_s_to_obs.exponential_cost_dd;
      }
    }

    l_x_seq[i] = l_x;
    l_xx_seq[i] = l_xx;
  }

  BackwardPerturbationParaState state_perturbation_result;
  state_perturbation_result.l_x = l_x_seq;
  state_perturbation_result.l_xx = l_xx_seq;
  return state_perturbation_result;
}

double RunStepCilqr::GetTotalCost(
    const std::vector<CilqrModel::CilqrState>& rough_state_seq,
    const std::vector<CilqrModel::CilqrControl>& rough_control_seq) {
  double total_cost = 0.0;
  const auto& speed_cilqr_conf = config::PlanningConfig::Instance()
                                     ->planning_research_config()
                                     .speed_cilqr;

  for (int i = 0; i < steps_; i++) {
    // 这里面i是否需要加1？

    Eigen::Matrix<double, 3, 1> state_diff;
    state_diff(0, 0) =
        rough_state_seq[i].state_s - ref_speed_plan_.goal_s_[i].goal_t_s.s();
    state_diff(1, 0) =
        rough_state_seq[i].state_v - ref_speed_plan_.goal_v_[i].goal_t_v.s();
    state_diff(2, 0) =
        rough_state_seq[i].state_a - ref_speed_plan_.goal_a_[i].goal_t_a.s();
    double c_state = state_diff.transpose() *
                     cilqr_model_->cost_para().state_cost_matrix * state_diff;
    double c_ctrl = std::pow(rough_control_seq[i].control_jerk, 2) *
                    speed_cilqr_conf.control_cost;
    total_cost = total_cost + c_state + c_ctrl;
  }

  return total_cost;
}

BackwardPerturbationParaControl RunStepCilqr::GetControlCostDerivatives(
    const std::vector<CilqrModel::CilqrState>& rough_state_seq,
    const std::vector<CilqrModel::CilqrControl>& rough_control_seq) {
  const auto& speed_cilqr_conf = config::PlanningConfig::Instance()
                                     ->planning_research_config()
                                     .speed_cilqr;
  std::vector<CilqrModel::l_u> l_u_seq{};
  std::vector<CilqrModel::l_uu> l_uu_seq{};
  l_u_seq.resize(steps_);
  l_uu_seq.resize(steps_);
  Eigen::Matrix<double, 1, 1> P;

  P(0, 0) = 1;

  for (int i = 0; i < steps_; i++) {
    double jerk_error_max =
        rough_control_seq[i].control_jerk - speed_cilqr_conf.max_jerk_limit;

    auto barrier_function_feedback_max = BarrireFunction(
        speed_cilqr_conf.q1_jerk, speed_cilqr_conf.q2_jerk, jerk_error_max, P);

    double jerk_error_min =
        speed_cilqr_conf.min_jerk_limit - rough_control_seq[i].control_jerk;

    auto barrier_function_feedback_min = BarrireFunction(
        speed_cilqr_conf.q1_jerk, speed_cilqr_conf.q2_jerk, jerk_error_min, -P);

    CilqrModel::l_u l_u;
    CilqrModel::l_uu l_uu;
    l_u.setZero();
    l_uu.setZero();
    l_u = l_u + barrier_function_feedback_max.exponential_cost_d +
          barrier_function_feedback_min.exponential_cost_d +
          2 * rough_control_seq[i].control_jerk *
              speed_cilqr_conf.control_cost * P;

    l_uu = l_uu + barrier_function_feedback_max.exponential_cost_dd +
           barrier_function_feedback_min.exponential_cost_dd +
           2 * speed_cilqr_conf.control_cost * P;
    l_u_seq[i] = l_u;
    l_uu_seq[i] = l_uu;
  }

  BackwardPerturbationParaControl control_perturbation_result;
  control_perturbation_result.l_u = l_u_seq;
  control_perturbation_result.l_uu = l_uu_seq;
  return control_perturbation_result;
}

BarrierFunctionFeedBack RunStepCilqr::BarrireFunction(
    double q1, double q2, double error, Eigen::MatrixXd error_dot) {
  BarrierFunctionFeedBack result{
      .exponential_cost = std::exp(600.0),
      .exponential_cost_d = error_dot,
      .exponential_cost_dd = error_dot * error_dot.transpose()};
  double q2e = q2 * error;
  if (q2e > 600.0) {
    result.exponential_cost_d.setConstant(std::exp(600.0));
    result.exponential_cost_dd.diagonal().setConstant(std::exp(600.0));
  } else {
    double exp_t = exp(q2e);
    Eigen::MatrixXd q2ed = q2 * error_dot;
    result.exponential_cost = q1 * exp_t;
    result.exponential_cost_d = result.exponential_cost * q2ed;
    result.exponential_cost_dd = result.exponential_cost_d * q2ed.transpose();
  }

  return result;
}

}  // namespace planning
}  // namespace neodrive
