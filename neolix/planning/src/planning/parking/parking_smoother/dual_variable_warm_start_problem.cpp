#include "dual_variable_warm_start_problem.h"

namespace neodrive {
namespace planning {

DualVariableWarmStartProblem::DualVariableWarmStartProblem(
    const ParkingDualVariableConfig& dualvariable_config) {
  dualvariable_config_ = dualvariable_config;
}

bool DualVariableWarmStartProblem::Solve(
    const size_t horizon, const double ts, const Eigen::MatrixXd& ego,
    size_t obstacles_num, const Eigen::MatrixXi& obstacles_edges_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const Eigen::MatrixXd& xWS, Eigen::MatrixXd* l_warm_up,
    Eigen::MatrixXd* n_warm_up, Eigen::MatrixXd* s_warm_up) {
  bool solver_flag = false;

  if (dualvariable_config_.qp_format_ == 0) {
    DualVariableWarmStartOSQPInterface ptop =
        DualVariableWarmStartOSQPInterface(
            horizon, ts, ego, obstacles_edges_num, obstacles_num, obstacles_A,
            obstacles_b, xWS, dualvariable_config_);

    if (ptop.optimize()) {
      LOG_INFO("dual warm up done.");
      ptop.get_optimization_results(l_warm_up, n_warm_up);

      LOG_INFO("Dual variable warm start solving time in second : ");

      solver_flag = true;
    } else {
      LOG_WARN("dual warm up fail.");
      ptop.get_optimization_results(l_warm_up, n_warm_up);
      solver_flag = false;
    }
  } else if (dualvariable_config_.qp_format_ == 1) {
    DualVariableWarmStartSlackOSQPInterface ptop =
        DualVariableWarmStartSlackOSQPInterface(
            horizon, ts, ego, obstacles_edges_num, obstacles_num, obstacles_A,
            obstacles_b, xWS, dualvariable_config_);

    if (ptop.optimize()) {
      LOG_INFO("dual warm up done.");
      ptop.get_optimization_results(l_warm_up, n_warm_up, s_warm_up);

      LOG_INFO("Dual variable warm start solving time in second : ");

      solver_flag = true;
    } else {
      LOG_WARN("dual warm up fail.");
      ptop.get_optimization_results(l_warm_up, n_warm_up, s_warm_up);
      solver_flag = false;
    }
  } else if (dualvariable_config_.qp_format_ == 2) {
    DualVariableWarmStartIPOPTQPInterface* ptop =
        new DualVariableWarmStartIPOPTQPInterface(
            horizon, ts, ego, obstacles_edges_num, obstacles_num, obstacles_A,
            obstacles_b, xWS, dualvariable_config_);

    Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop;
    // Create an instance of the IpoptApplication
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetIntegerValue(
        "print_level", dualvariable_config_.ipopt_config_.ipopt_print_level_);
    app->Options()->SetIntegerValue(
        "mumps_mem_percent",
        dualvariable_config_.ipopt_config_.mumps_mem_percent_);
    app->Options()->SetNumericValue(
        "mumps_pivtol", dualvariable_config_.ipopt_config_.mumps_pivtol_);
    app->Options()->SetIntegerValue(
        "max_iter", dualvariable_config_.ipopt_config_.ipopt_max_iter_);
    app->Options()->SetNumericValue(
        "tol", dualvariable_config_.ipopt_config_.ipopt_tol_);
    app->Options()->SetNumericValue(
        "acceptable_constr_viol_tol",
        dualvariable_config_.ipopt_config_.ipopt_acceptable_constr_viol_tol_);
    app->Options()->SetNumericValue(
        "min_hessian_perturbation",
        dualvariable_config_.ipopt_config_.ipopt_min_hessian_perturbation_);
    app->Options()->SetNumericValue("jacobian_regularization_value",
                                    dualvariable_config_.ipopt_config_
                                        .ipopt_jacobian_regularization_value_);
    app->Options()->SetStringValue(
        "print_timing_statistics",
        dualvariable_config_.ipopt_config_.ipopt_print_timing_statistics_);
    app->Options()->SetStringValue(
        "alpha_for_y", dualvariable_config_.ipopt_config_.ipopt_alpha_for_y_);
    app->Options()->SetStringValue(
        "recalc_y", dualvariable_config_.ipopt_config_.ipopt_recalc_y_);
    // for qp problem speed up
    app->Options()->SetStringValue("mehrotra_algorithm", "yes");

    Ipopt::ApplicationReturnStatus status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
      LOG_ERROR(
          "Dual variable wart start problem error during initialization!");
      return false;
    }

    status = app->OptimizeTNLP(problem);

    if (status == Ipopt::Solve_Succeeded ||
        status == Ipopt::Solved_To_Acceptable_Level) {
      // Retrieve some statistics about the solve
      Ipopt::Index iter_count = app->Statistics()->IterationCount();
      LOG_INFO("The problem solved in {} iterations!", iter_count);

      Ipopt::Number final_obj = app->Statistics()->FinalObjective();
      LOG_INFO("The final value of the objective function is {}", final_obj);

    } else {
      LOG_INFO("Solve not succeeding, return status: {}", int(status));
    }

    ptop->get_optimization_results(l_warm_up, n_warm_up);

    solver_flag = (status == Ipopt::Solve_Succeeded ||
                   status == Ipopt::Solved_To_Acceptable_Level);
  } else if (dualvariable_config_.qp_format_ == 3) {
    DualVariableWarmStartIPOPTInterface* ptop =
        new DualVariableWarmStartIPOPTInterface(
            horizon, ts, ego, obstacles_edges_num, obstacles_num, obstacles_A,
            obstacles_b, xWS, dualvariable_config_);

    Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop;
    // Create an instance of the IpoptApplication
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetIntegerValue(
        "print_level", dualvariable_config_.ipopt_config_.ipopt_print_level_);
    app->Options()->SetIntegerValue(
        "mumps_mem_percent",
        dualvariable_config_.ipopt_config_.mumps_mem_percent_);
    app->Options()->SetNumericValue(
        "mumps_pivtol", dualvariable_config_.ipopt_config_.mumps_pivtol_);
    app->Options()->SetIntegerValue(
        "max_iter", dualvariable_config_.ipopt_config_.ipopt_max_iter_);
    app->Options()->SetNumericValue(
        "tol", dualvariable_config_.ipopt_config_.ipopt_tol_);
    app->Options()->SetNumericValue(
        "acceptable_constr_viol_tol",
        dualvariable_config_.ipopt_config_.ipopt_acceptable_constr_viol_tol_);
    app->Options()->SetNumericValue(
        "min_hessian_perturbation",
        dualvariable_config_.ipopt_config_.ipopt_min_hessian_perturbation_);
    app->Options()->SetNumericValue("jacobian_regularization_value",
                                    dualvariable_config_.ipopt_config_
                                        .ipopt_jacobian_regularization_value_);
    app->Options()->SetStringValue(
        "print_timing_statistics",
        dualvariable_config_.ipopt_config_.ipopt_print_timing_statistics_);
    app->Options()->SetStringValue(
        "alpha_for_y", dualvariable_config_.ipopt_config_.ipopt_alpha_for_y_);
    app->Options()->SetStringValue(
        "recalc_y", dualvariable_config_.ipopt_config_.ipopt_recalc_y_);

    app->Options()->SetStringValue("print_info_string", "yes");

    Ipopt::ApplicationReturnStatus status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
      LOG_ERROR(
          "Dual variable wart start problem error during initialization!");
      return false;
    }

    status = app->OptimizeTNLP(problem);
    if (status == Ipopt::Solve_Succeeded ||
        status == Ipopt::Solved_To_Acceptable_Level) {
      // Retrieve some statistics about the solve
      Ipopt::Index iter_count = app->Statistics()->IterationCount();
      LOG_INFO("The problem solved in {} iterations!", iter_count);

      Ipopt::Number final_obj = app->Statistics()->FinalObjective();
      LOG_INFO("The final value of the objective function is {}", final_obj);

    } else {
      LOG_INFO("Solve not succeeding, return status: {}", int(status));
    }

    ptop->get_optimization_results(l_warm_up, n_warm_up);

    solver_flag = (status == Ipopt::Solve_Succeeded ||
                   status == Ipopt::Solved_To_Acceptable_Level);
  }

  if (solver_flag == false) {
    // if solver fails during dual warm up, insert zeros instead
    for (int r = 0; r < l_warm_up->rows(); ++r) {
      for (int c = 0; c < l_warm_up->cols(); ++c) {
        (*l_warm_up)(r, c) = 0.0;
      }
    }

    for (int r = 0; r < n_warm_up->rows(); ++r) {
      for (int c = 0; c < n_warm_up->cols(); ++c) {
        (*n_warm_up)(r, c) = 0.0;
      }
    }
  }

  return true;
}
}  // namespace planning
}  // namespace neodrive
