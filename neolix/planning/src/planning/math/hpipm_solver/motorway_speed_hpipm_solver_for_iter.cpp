#include "motorway_speed_hpipm_solver_for_iter.h"

#include <iostream>

namespace neodrive {
namespace planning {

MotorwaySpeedHpipmSolverForIter::MotorwaySpeedHpipmSolverForIter(
    const std::string &solver_name, const std::size_t n, const std::size_t n_x,
    const std::size_t n_u)
    : solver_name_(solver_name), N_(n), N_X_(n_x), N_U_(n_u) {
  nx_.resize(N_ + 1, 0);
  nu_.resize(N_ + 1, 0);
  nbx_.resize(N_ + 1, 0);
  nbu_.resize(N_ + 1, 0);
  ng_.resize(N_ + 1, 0);
  nsbx_.resize(N_ + 1, 0);
  nsbu_.resize(N_ + 1, 0);
  nsg_.resize(N_ + 1, 0);

  hA_.resize(N_, nullptr);
  hB_.resize(N_, nullptr);
  hb_.resize(N_, nullptr);

  hQ_.resize(N_ + 1, nullptr);
  hS_.resize(N_ + 1, nullptr);
  hR_.resize(N_ + 1, nullptr);
  hq_.resize(N_ + 1, nullptr);
  hr_.resize(N_ + 1, nullptr);

  hlg_.resize(N_ + 1, nullptr);
  hug_.resize(N_ + 1, nullptr);
  hC_.resize(N_ + 1, nullptr);
  hD_.resize(N_ + 1, nullptr);

  hidxbx_.resize(N_ + 1, nullptr);
  hlbx_.resize(N_ + 1, nullptr);
  hubx_.resize(N_ + 1, nullptr);

  hidxbu_.resize(N_ + 1, nullptr);
  hlbu_.resize(N_ + 1, nullptr);
  hubu_.resize(N_ + 1, nullptr);

  hZl_.resize(N_ + 1, nullptr);
  hZu_.resize(N_ + 1, nullptr);
  hzl_.resize(N_ + 1, nullptr);
  hzu_.resize(N_ + 1, nullptr);

  hlls_.resize(N_ + 1, nullptr);
  hlus_.resize(N_ + 1, nullptr);

  hidxs_.resize(N_ + 1, nullptr);

  hpipm_bounds_.resize(N_ + 1);
}

void MotorwaySpeedHpipmSolverForIter::SetDynamics(
    const MotorwaySpeedForIter::State &x0,
    std::vector<MotorwaySpeedForIter::Stage> &stages) {
  b0_ = (stages[0].line_model.A * MotorwaySpeedForIter::StateToVector(x0) +
         stages[0].line_model.g);
  for (int i = 0; i < N_; i++) {
    if (i == 0) {
      hA_[i] = nullptr;
      hB_[i] = stages[i].line_model.B.data();
      hb_[i] = b0_.data();

      nx_[i] = 0;
      nu_[i] = N_U_;
    } else {
      hA_[i] = stages[i].line_model.A.data();
      hB_[i] = stages[i].line_model.B.data();
      hb_[i] = stages[i].line_model.g.data();

      nx_[i] = N_X_;
      nu_[i] = N_U_;
    }
  }
  nx_[N_] = N_X_;
  nu_[N_] = 0;
}

void MotorwaySpeedHpipmSolverForIter::SetCost(
    std::vector<MotorwaySpeedForIter::Stage> &stages) {
  for (int i = 0; i <= N_; i++) {
    hQ_[i] = stages[i].cost.Q.data();
    hR_[i] = stages[i].cost.R.data();
    hS_[i] = stages[i].cost.S.data();

    hq_[i] = stages[i].cost.q.data();
    hr_[i] = stages[i].cost.r.data();

    if (stages[i].ns != 0) {
      hZl_[i] = stages[i].cost.Z.data();
      hZu_[i] = stages[i].cost.Z.data();
      hzl_[i] = stages[i].cost.z.data();
      hzu_[i] = stages[i].cost.z.data();
    } else {
      hZl_[i] = nullptr;
      hZu_[i] = nullptr;
      hzl_[i] = nullptr;
      hzu_[i] = nullptr;
    }
  }
}

void MotorwaySpeedHpipmSolverForIter::SetBounds(
    std::vector<MotorwaySpeedForIter::Stage> &stages) {
  nbu_[0] = 0;
  hpipm_bounds_[0].idx_u.resize(0);
  hpipm_bounds_[0].lower_bounds_u.resize(0);
  hpipm_bounds_[0].upper_bounds_u.resize(0);
  for (int j = 0; j < N_U_; j++) {
    if (stages[0].lower_bounds_u(j) > -MotorwaySpeedForIter::INF &&
        stages[0].upper_bounds_u(j) < MotorwaySpeedForIter::INF) {
      nbu_[0]++;
      hpipm_bounds_[0].idx_u.push_back(j);
      hpipm_bounds_[0].lower_bounds_u.push_back(stages[0].lower_bounds_u(j));
      hpipm_bounds_[0].upper_bounds_u.push_back(stages[0].upper_bounds_u(j));
    }
  }
  nbx_[0] = 0;
  hidxbx_[0] = nullptr;
  hidxbu_[0] = hpipm_bounds_[0].idx_u.data();

  hlbx_[0] = nullptr;
  hubx_[0] = nullptr;
  hlbu_[0] = hpipm_bounds_[0].lower_bounds_u.data();
  hubu_[0] = hpipm_bounds_[0].upper_bounds_u.data();

  for (int i = 1; i <= N_; i++) {
    hpipm_bounds_[i].idx_u.resize(0);
    hpipm_bounds_[i].lower_bounds_u.resize(0);
    hpipm_bounds_[i].upper_bounds_u.resize(0);
    nbu_[i] = 0;
    for (int j = 0; j < N_U_; j++) {
      if (stages[i].lower_bounds_u(j) > -MotorwaySpeedForIter::INF &&
          stages[i].upper_bounds_u(j) < MotorwaySpeedForIter::INF) {
        nbu_[i]++;
        hpipm_bounds_[i].idx_u.push_back(j);
        hpipm_bounds_[i].lower_bounds_u.push_back(stages[i].lower_bounds_u(j));
        hpipm_bounds_[i].upper_bounds_u.push_back(stages[i].upper_bounds_u(j));
      }
    }

    hpipm_bounds_[i].idx_x.resize(0);
    hpipm_bounds_[i].lower_bounds_x.resize(0);
    hpipm_bounds_[i].upper_bounds_x.resize(0);
    nbx_[i] = 0;
    for (int j = 0; j < N_X_; j++) {
      if (stages[i].lower_bounds_x(j) > -MotorwaySpeedForIter::INF &&
          stages[i].upper_bounds_x(j) < MotorwaySpeedForIter::INF) {
        nbx_[i]++;
        hpipm_bounds_[i].idx_x.push_back(j);
        hpipm_bounds_[i].lower_bounds_x.push_back(stages[i].lower_bounds_x(j));
        hpipm_bounds_[i].upper_bounds_x.push_back(stages[i].upper_bounds_x(j));
      }
    }

    hidxbx_[i] = hpipm_bounds_[i].idx_x.data();
    hidxbu_[i] = hpipm_bounds_[i].idx_u.data();
    hlbx_[i] = hpipm_bounds_[i].lower_bounds_x.data();
    hubx_[i] = hpipm_bounds_[i].upper_bounds_x.data();
    hlbu_[i] = hpipm_bounds_[i].lower_bounds_u.data();
    hubu_[i] = hpipm_bounds_[i].upper_bounds_u.data();
  }

  nbu_[N_] = 0;
  hidxbu_[N_] = nullptr;
  hlbu_[N_] = nullptr;
  hubu_[N_] = nullptr;
}

void MotorwaySpeedHpipmSolverForIter::SetPolytopicConstraints(
    std::vector<MotorwaySpeedForIter::Stage> &stages) {
  for (int i = 0; i <= N_; i++) {
    ng_[i] = stages[i].ng;
    if (stages[i].ng > 0) {
      hC_[i] = stages[i].constraints.C.data();
      hD_[i] = stages[i].constraints.D.data();

      hlg_[i] = stages[i].constraints.dl.data();
      hug_[i] = stages[i].constraints.du.data();
    } else {
      hC_[i] = nullptr;
      hD_[i] = nullptr;

      hlg_[i] = nullptr;
      hug_[i] = nullptr;
    }
  }
}

void MotorwaySpeedHpipmSolverForIter::SetSoftConstraints(
    std::vector<MotorwaySpeedForIter::Stage> &stages) {
  // Note: soft constraints support for polytopic constraints
  for (int i = 0; i <= N_; i++) {
    hpipm_bounds_[i].idx_s.resize(0);
    if (stages[i].ns != 0) {
      nsbx_[i] = 0;
      nsbu_[i] = 0;
      nsg_[i] = stages[i].ns;

      for (int j = 0; j < stages[i].ns; j++) {
        hpipm_bounds_[i].idx_s.push_back(j + nbx_[i] + nbu_[i]);
      }

      hidxs_[i] = hpipm_bounds_[i].idx_s.data();
      hlls_[i] = stages[i].lower_bounds_s.data();
      hlus_[i] = stages[i].upper_bounds_s.data();
    } else {
      nsbx_[i] = 0;
      nsbu_[i] = 0;
      nsg_[i] = 0;
      hidxs_[i] = nullptr;
      hlls_[i] = nullptr;
      hlus_[i] = nullptr;
    }
  }
}

std::vector<MotorwaySpeedForIter::OptVariables>
MotorwaySpeedHpipmSolverForIter::SolveMPC(
    const MotorwaySpeedForIter::State &x0,
    std::vector<MotorwaySpeedForIter::Stage> &stages, int *status) {
  SetDynamics(x0, stages);

  SetCost(stages);

  SetBounds(stages);

  SetPolytopicConstraints(stages);

  SetSoftConstraints(stages);

  // PrintData();

  std::vector<MotorwaySpeedForIter::OptVariables> opt_solution = Solve(status);
  opt_solution[0].xk = x0;

  for (const auto &opt_variable : opt_solution) {
    LOG_DEBUG("s, v, a, jerk: {:.3f}, {:.3f}, {:.3f}, {:.3f}",
              opt_variable.xk.state_s, opt_variable.xk.state_v,
              opt_variable.xk.state_a, opt_variable.uk.control_jerk);
  }

  return opt_solution;
}

std::vector<MotorwaySpeedForIter::OptVariables>
MotorwaySpeedHpipmSolverForIter::Solve(int *status) {
  // ocp qp dim
  int dim_size = d_ocp_qp_dim_memsize(N_);
  void *dim_mem = malloc(dim_size);

  struct d_ocp_qp_dim dim;
  d_ocp_qp_dim_create(N_, &dim, dim_mem);

  d_ocp_qp_dim_set_all(nx_.data(), nu_.data(), nbx_.data(), nbu_.data(),
                       ng_.data(), nsbx_.data(), nsbu_.data(), nsg_.data(),
                       &dim);
  // ocp qp
  int qp_size = d_ocp_qp_memsize(&dim);
  void *qp_mem = malloc(qp_size);

  struct d_ocp_qp qp;
  d_ocp_qp_create(&dim, &qp, qp_mem);
  d_ocp_qp_set_all(hA_.data(), hB_.data(), hb_.data(), hQ_.data(), hS_.data(),
                   hR_.data(), hq_.data(), hr_.data(), hidxbx_.data(),
                   hlbx_.data(), hubx_.data(), hidxbu_.data(), hlbu_.data(),
                   hubu_.data(), hC_.data(), hD_.data(), hlg_.data(),
                   hug_.data(), hZl_.data(), hZu_.data(), hzl_.data(),
                   hzu_.data(), hidxs_.data(), hlls_.data(), hlus_.data(), &qp);

  // ocp qp sol
  int qp_sol_size = d_ocp_qp_sol_memsize(&dim);
  void *qp_sol_mem = malloc(qp_sol_size);

  struct d_ocp_qp_sol qp_sol;
  d_ocp_qp_sol_create(&dim, &qp_sol, qp_sol_mem);

  // ipm arg

  int ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim);
  void *ipm_arg_mem = malloc(ipm_arg_size);

  struct d_ocp_qp_ipm_arg arg;
  d_ocp_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);

  // enum hpipm_mode mode = SPEED_ABS;
  enum hpipm_mode mode = SPEED;
  // enum hpipm_mode mode = ROBUST;
  //    enum hpipm_mode mode = ROBUST;

  //    int mode = 1;
  double mu0 = 1e10;
  int iter_max = 30;
  double tol_stat = 1e-2;
  double tol_eq = 1e-1;
  double tol_ineq = 1e-1;
  double tol_comp = 1e-3;
  double reg_prim = 1e-4;
  int warm_start = 0;
  int pred_corr = 1;
  int ric_alg = 0;

  d_ocp_qp_ipm_arg_set_default(mode, &arg);

  // d_ocp_qp_ipm_arg_set_mu0(&mu0, &arg);
  d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &arg);
  // d_ocp_qp_ipm_arg_set_tol_stat(&tol_stat, &arg);
  d_ocp_qp_ipm_arg_set_tol_eq(&tol_eq, &arg);
  d_ocp_qp_ipm_arg_set_tol_ineq(&tol_ineq, &arg);
  // d_ocp_qp_ipm_arg_set_tol_comp(&tol_comp, &arg);
  // d_ocp_qp_ipm_arg_set_reg_prim(&reg_prim, &arg);
  // d_ocp_qp_ipm_arg_set_warm_start(&warm_start, &arg);
  // d_ocp_qp_ipm_arg_set_pred_corr(&pred_corr, &arg);
  // d_ocp_qp_ipm_arg_set_ric_alg(&ric_alg, &arg);

  // ipm
  int ipm_size = d_ocp_qp_ipm_ws_memsize(&dim, &arg);
  void *ipm_mem = malloc(ipm_size);

  struct d_ocp_qp_ipm_ws workspace;
  d_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);

  int hpipm_return;  // 0 normal; 1 max iter; 2 linesearch issues?

  struct timeval tv0, tv1;

  d_ocp_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
  d_ocp_qp_ipm_get_status(&workspace, &hpipm_return);

  // extract and print solution
  std::vector<MotorwaySpeedForIter::OptVariables> optimal_solution;
  optimal_solution.resize(N_ + 1);
  int ii;

  int nu_max = nu_[0];
  for (ii = 1; ii <= N_; ii++)
    if (nu_[ii] > nu_max) nu_max = nu_[ii];
  double *u = (double *)malloc(nu_max * sizeof(double));
  for (int i = 0; i < N_; i++) {
    d_ocp_qp_sol_get_u(i, &qp_sol, u);
    optimal_solution[i].uk = MotorwaySpeedForIter::ArrayToControl(u);
  }
  optimal_solution[N_].uk.SetZero();

  int nx_max = nx_[0];
  for (ii = 1; ii <= N_; ii++)
    if (nx_[ii] > nx_max) nx_max = nx_[ii];
  double *x = (double *)malloc(nx_max * sizeof(double));
  optimal_solution[0].xk.SetZero();
  for (int i = 1; i <= N_; i++) {
    d_ocp_qp_sol_get_x(i, &qp_sol, x);
    optimal_solution[i].xk = MotorwaySpeedForIter::ArrayToState(x);
  }

  int ns_max = nsg_[0];
  for (ii = 1; ii <= N_; ii++)
    if (nsg_[ii] > ns_max) ns_max = nsg_[ii];
  double *sl = (double *)malloc(ns_max * sizeof(double));
  double *su = (double *)malloc(ns_max * sizeof(double));
  for (int ii = 0; ii <= N_; ii++) {
    d_ocp_qp_sol_get_sl(ii, &qp_sol, sl);
    d_ocp_qp_sol_get_su(ii, &qp_sol, su);
  }
  // for (int ii = 0; ii <= N_; ii++) {
  //   LOG_INFO("soft_value: {:.3f}, {:.3f}", su[ii], sl[ii]);
  // }

  /************************************************
   * print ipm statistics
   ************************************************/
  int iter;
  d_ocp_qp_ipm_get_iter(&workspace, &iter);
  double res_stat;
  d_ocp_qp_ipm_get_max_res_stat(&workspace, &res_stat);
  double res_eq;
  d_ocp_qp_ipm_get_max_res_eq(&workspace, &res_eq);
  double res_ineq;
  d_ocp_qp_ipm_get_max_res_ineq(&workspace, &res_ineq);
  double res_comp;
  d_ocp_qp_ipm_get_max_res_comp(&workspace, &res_comp);
  double *stat;
  d_ocp_qp_ipm_get_stat(&workspace, &stat);
  int stat_m;
  d_ocp_qp_ipm_get_stat_m(&workspace, &stat_m);

  LOG_INFO("hpipm iter: {}", iter);

  free(dim_mem);
  free(qp_mem);
  free(qp_sol_mem);
  free(ipm_arg_mem);
  free(ipm_mem);

  free(u);
  free(x);
  free(sl);
  free(su);

  *status = hpipm_return;

  return optimal_solution;
}

void MotorwaySpeedHpipmSolverForIter::PrintData() {
  for (int k = 0; k <= N_; k++) {
    if (k != N_) {
      std::cout << "A_" << k << " = " << std::endl;
      if (k > 0) {
        for (int i = 0; i < N_X_; i++) {
          for (int j = 0; j < N_X_; j++) {
            std::cout << hA_[k][i + j * N_X_] << " ";
          }
          std::cout << std::endl;
        }
      }

      std::cout << "B_" << k << " = " << std::endl;
      for (int i = 0; i < N_X_; i++) {
        for (int j = 0; j < N_U_; j++) {
          std::cout << hB_[k][i + j * N_X_] << " ";
        }
        std::cout << std::endl;
      }

      std::cout << "b_" << k << " = " << std::endl;
      for (int i = 0; i < N_X_; i++) {
        for (int j = 0; j < 1; j++) {
          std::cout << hb_[k][i + j * N_X_] << " ";
        }
        std::cout << std::endl;
      }
    }

    std::cout << "Q_" << k << " = " << std::endl;
    for (int i = 0; i < N_X_; i++) {
      for (int j = 0; j < N_X_; j++) {
        std::cout << hQ_[k][i + j * N_X_] << " ";
      }
      std::cout << std::endl;
    }

    std::cout << "R_" << k << " = " << std::endl;
    for (int i = 0; i < N_U_; i++) {
      for (int j = 0; j < N_U_; j++) {
        std::cout << hR_[k][i + j * N_U_] << " ";
      }
      std::cout << std::endl;
    }

    std::cout << "q_" << k << " = " << std::endl;
    for (int i = 0; i < N_X_; i++) {
      std::cout << hq_[k][i] << " ";
      std::cout << std::endl;
    }

    std::cout << "r_" << k << " = " << std::endl;
    for (int i = 0; i < N_U_; i++) {
      std::cout << hr_[k][i] << " ";
      std::cout << std::endl;
    }

    std::cout << "sizes" << std::endl;
    std::cout << nx_[k] << std::endl;
    std::cout << nu_[k] << std::endl;
    std::cout << nbx_[k] << std::endl;
    std::cout << nbu_[k] << std::endl;
    std::cout << ng_[k] << std::endl;
  }
}

}  // namespace planning
}  // namespace neodrive