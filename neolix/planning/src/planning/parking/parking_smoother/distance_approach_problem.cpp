#include "distance_approach_problem.h"

namespace neodrive {
namespace planning {

DistanceApproachProblem::DistanceApproachProblem(
    const DistanceApproachConfig& distance_approch_config,
    const ParkingCommonConfig common_config) {
  distance_approch_config_ = distance_approch_config;
  common_config_ = common_config;
}

bool DistanceApproachProblem::Solve(
    const Eigen::MatrixXd& x0, const Eigen::MatrixXd& xF,
    const Eigen::MatrixXd& last_time_u, const size_t horizon, const double ts,
    const Eigen::MatrixXd& ego, const Eigen::MatrixXd& xWS,
    const Eigen::MatrixXd& uWS, const Eigen::MatrixXd& l_warm_up,
    const Eigen::MatrixXd& n_warm_up, const Eigen::MatrixXd& s_warm_up,
    const std::vector<double>& XYbounds, const size_t obstacles_num,
    const Eigen::MatrixXi& obstacles_edges_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    Eigen::MatrixXd* state_result, Eigen::MatrixXd* control_result,
    Eigen::MatrixXd* time_result, Eigen::MatrixXd* dual_l_result,
    Eigen::MatrixXd* dual_n_result) {
  // TODO: evaluate whether need to new it everytime

  DistanceApproachInterface* ptop = nullptr;

  if (distance_approch_config_.distance_approach_mode_ == 0) {
    ptop = new DistanceApproachIPOPTInterface(
        horizon, ts, ego, xWS, uWS, l_warm_up, n_warm_up, x0, xF, last_time_u,
        XYbounds, obstacles_edges_num, obstacles_num, obstacles_A, obstacles_b,
        distance_approch_config_, common_config_);
  } else if (distance_approch_config_.distance_approach_mode_ == 2) {
    ptop = new DistanceApproachIPOPTFixedTsInterface(
        horizon, ts, ego, xWS, uWS, l_warm_up, n_warm_up, x0, xF, last_time_u,
        XYbounds, obstacles_edges_num, obstacles_num, obstacles_A, obstacles_b,
        distance_approch_config_, common_config_);
  }
  // else if (distance_approch_config_.distance_approach_mode_ ==
  //  DISTANCE_APPROACH_IPOPT_CUDA)
  //                 {
  //    ptop = new DistanceApproachIPOPTCUDAInterface(
  //        horizon, ts, ego, xWS, uWS, l_warm_up, n_warm_up, x0, xF,
  //        last_time_u, XYbounds, obstacles_edges_num, obstacles_num,
  //        obstacles_A, obstacles_b, distance_approch_config_);
  //  }
  else if (distance_approch_config_.distance_approach_mode_ == 3) {
    ptop = new DistanceApproachIPOPTFixedDualInterface(
        horizon, ts, ego, xWS, uWS, l_warm_up, n_warm_up, x0, xF, last_time_u,
        XYbounds, obstacles_edges_num, obstacles_num, obstacles_A, obstacles_b,
        distance_approch_config_, common_config_);
  } else if (distance_approch_config_.distance_approach_mode_ == 4) {
    ptop = new DistanceApproachIPOPTRelaxEndInterface(
        horizon, ts, ego, xWS, uWS, l_warm_up, n_warm_up, x0, xF, last_time_u,
        XYbounds, obstacles_edges_num, obstacles_num, obstacles_A, obstacles_b,
        distance_approch_config_, common_config_);
  } else if (distance_approch_config_.distance_approach_mode_ == 5) {
    ptop = new DistanceApproachIPOPTRelaxEndSlackInterface(
        horizon, ts, ego, xWS, uWS, l_warm_up, n_warm_up, s_warm_up, x0, xF,
        last_time_u, XYbounds, obstacles_edges_num, obstacles_num, obstacles_A,
        obstacles_b, distance_approch_config_, common_config_);
  }

  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetIntegerValue(
      "print_level", distance_approch_config_.ipopt_config_.ipopt_print_level_);

  app->Options()->SetIntegerValue(
      "mumps_mem_percent",
      distance_approch_config_.ipopt_config_.mumps_mem_percent_);

  app->Options()->SetNumericValue(
      "mumps_pivtol", distance_approch_config_.ipopt_config_.mumps_pivtol_);

  app->Options()->SetIntegerValue(
      "max_iter", distance_approch_config_.ipopt_config_.ipopt_max_iter_);

  app->Options()->SetNumericValue(
      "tol", distance_approch_config_.ipopt_config_.ipopt_tol_);

  app->Options()->SetNumericValue(
      "acceptable_constr_viol_tol",
      distance_approch_config_.ipopt_config_.ipopt_acceptable_constr_viol_tol_);

  app->Options()->SetNumericValue(
      "min_hessian_perturbation",
      distance_approch_config_.ipopt_config_.ipopt_min_hessian_perturbation_);

  app->Options()->SetNumericValue("jacobian_regularization_value",
                                  distance_approch_config_.ipopt_config_
                                      .ipopt_jacobian_regularization_value_);

  app->Options()->SetStringValue(
      "print_timing_statistics",
      distance_approch_config_.ipopt_config_.ipopt_print_timing_statistics_);

  app->Options()->SetStringValue(
      "alpha_for_y", distance_approch_config_.ipopt_config_.ipopt_alpha_for_y_);

  app->Options()->SetStringValue(
      "recalc_y", distance_approch_config_.ipopt_config_.ipopt_recalc_y_);

  app->Options()->SetNumericValue(
      "mu_init", distance_approch_config_.ipopt_config_.ipopt_mu_init_);

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    LOG_ERROR("Distance Approach problem error during initialization!");
    return false;
  }

  status = app->OptimizeTNLP(problem);

  if (status == Ipopt::Solve_Succeeded ||
      status == Ipopt::Solved_To_Acceptable_Level) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    LOG_INFO("The problem solved in {} iterations!", iter_count);

    Ipopt::Number final_obj = app->Statistics()->FinalObjective();
    LOG_INFO("The final value of the objective function is {}.", final_obj);

  } else {
    /*
      return detailed failure information,
      reference resource: Ipopt::ApplicationReturnStatus, https://
      www.coin-or.org/Doxygen/CoinAll/_ip_return_codes__inc_8h-source.html
    */
    std::unordered_map<int, std::string> failure_status = {
        {0, "Solve_Succeeded"},
        {1, "Solved_To_Acceptable_Level"},
        {2, "Infeasible_Problem_Detected"},
        {3, "Search_Direction_Becomes_Too_Small"},
        {4, "Diverging_Iterates"},
        {5, "User_Requested_Stop"},
        {6, "Feasible_Point_Found"},
        {-1, "Maximum_Iterations_Exceeded"},
        {-2, "Restoration_Failed"},
        {-3, "Error_In_Step_Computation"},
        {-10, "Not_Enough_Degrees_Of_Freedom"},
        {-11, "Invalid_Problem_Definition"},
        {-12, "Invalid_Option"},
        {-13, "Invalid_Number_Detected"},
        {-100, "Unrecoverable_Exception"},
        {-101, "NonIpopt_Exception_Thrown"},
        {-102, "Insufficient_Memory"},
        {-199, "Internal_Error"}};

    if (!failure_status.count(static_cast<std::size_t>(status))) {
      LOG_INFO("Solver ends with unknown failure code: {}",
               static_cast<int>(status));
    } else {
      LOG_INFO("Solver failure case: {}",
               failure_status[static_cast<std::size_t>(status)]);
    }
  }

  ptop->get_optimization_results(state_result, control_result, time_result,
                                 dual_l_result, dual_n_result);

  return status == Ipopt::Solve_Succeeded ||
         status == Ipopt::Solved_To_Acceptable_Level;
}

}  // namespace planning
}  // namespace neodrive
