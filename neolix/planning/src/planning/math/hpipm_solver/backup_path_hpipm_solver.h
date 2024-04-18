#pragma once

#include <malloc.h>

#include <Eigen/Dense>
#include <array>

#include "src/planning/task/optimizers/backup_path_optimizer/backup_path_model.h"

extern "C" {
#include <blasfeo_d_aux_ext_dep.h>
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_sol.h>
#include <hpipm_timing.h>
}

namespace neodrive {
namespace planning {

class BackupPathHpipmSolver {
 public:
  BackupPathHpipmSolver(const std::string &solver_name, const std::size_t n,
                        const std::size_t n_x, const std::size_t n_u);

  ~BackupPathHpipmSolver() = default;

  std::vector<BackupPath::OptVariables> SolveMPC(
      const BackupPath::State &x0, std::vector<BackupPath::Stage> &stages,
      int *status);

 private:
  void SetDynamics(const BackupPath::State &x0,
                   std::vector<BackupPath::Stage> &stages);

  void SetCost(std::vector<BackupPath::Stage> &stages);

  void SetBounds(std::vector<BackupPath::Stage> &stages);

  void SetPolytopicConstraints(std::vector<BackupPath::Stage> &stages);

  void SetSoftConstraints(std::vector<BackupPath::Stage> &stages);

  std::vector<BackupPath::OptVariables> Solve(int *status);

 private:
  std::string solver_name_{""};
  std::size_t N_{0};
  std::size_t N_X_{0};
  std::size_t N_U_{0};

  std::vector<int> nx_;    // number of states
  std::vector<int> nu_;    // number of inputs
  std::vector<int> nbx_;   // number of bounds on x
  std::vector<int> nbu_;   // number of bounds on u
  std::vector<int> ng_;    // number of polytopic constratins
  std::vector<int> nsbx_;  // number of slacks variables on x
  std::vector<int> nsbu_;  // number of slacks variables on u
  std::vector<int> nsg_;  // number of slacks variables on polytopic constraints

  // LTV dynamics
  // x_k+1 = A_k x_k + B_k u_k + b_k
  std::vector<double *> hA_;  // hA[k] = A_k
  std::vector<double *> hB_;  // hB[k] = B_k
  std::vector<double *> hb_;  // hb[k] = b_k

  // Cost (without soft constraints)
  // min_x,u sum_k = 0^N_K { 1/2*[x_k;u_k]^T*[Q_k , S_k; S_k^T , R_k]*[x_k;u_k]
  // + [q_k; r_k]^T*[x_k;u_k] }
  std::vector<double *> hQ_;  // hQ[k] = Q_k
  std::vector<double *> hS_;  // hS[k] = S_k
  std::vector<double *> hR_;  // hR[k] = R_k
  std::vector<double *> hq_;  // hq[k] = q_k
  std::vector<double *> hr_;  // hr[k] = r_k

  // Polytopic constraints
  // g_lower,k <= D_k*x_k + C_k*u_k
  // D_k*x_k + C_k*u_k  <= g_upper,k
  std::vector<double *> hlg_;  // hlg[k] =  g_lower,k
  std::vector<double *> hug_;  // hug[k] =  g_upper,k
  std::vector<double *> hC_;   // hC[k] = C_k
  std::vector<double *> hD_;   // hD[k] = D_k

  // General bounds
  // x_lower,k <= x_k <= x_upper,k
  // hidxbx can be used to select bounds on a subset of states
  std::vector<int *> hidxbx_;  // hidxbx[k] = {0,1,2,...,nx} for bounds on all
  // and states
  std::vector<double *> hlbx_;  // x_lower,k
  std::vector<double *> hubx_;  // x_upper,k
  // u_lower,k <= u_k <=  u_upper,k
  // hidxbu can be used to select bounds on a subset of inputs
  std::vector<int *> hidxbu_;  // hidxbuk] = {0,1,2,...,nu} for bounds on all

  // and states
  std::vector<double *> hlbu_;  // u_lower,k
  std::vector<double *> hubu_;  // u_upper,k

  // Cost (only soft constriants)
  // s_lower,k -> slack variable of lower polytopic constraint (3) + lower
  // bounds s_upper,k -> slack variable of upper polytopic constraint (4) +
  // upper bounds min_x,u sum_k=0^N_K
  // 1/2*[s_lower,k;s_upper,k]^T*[Z_lower,k ,
  // 0; 0 , Z_upper,k]*[s_lower,k;s_upper,k] + [z_lower,k;
  // z_upper,k]^T*[s_lower,k;s_upper,k]
  std::vector<double *> hZl_;  // hZl[k] = Z_lower,k
  std::vector<double *> hZu_;  // hZu[k] = Z_upper,k
  std::vector<double *> hzl_;  // hzl[k] = z_lower,k
  std::vector<double *> hzu_;  // hzu[k] = z_upper,k

  // Bounds on the soft constraint multipliers
  // s_lower,k >= s_lower,bound,k
  // s_upper,k >= s_upper,bound,k
  std::vector<double *> hlls_;
  std::vector<double *> hlus_;
  // index of the bounds and constraints that are softened
  // order is not really clear
  //   int **hidxs_;
  std::vector<int *> hidxs_;

  // bounds that are different to stages bounds and need to be stored somewhere
  // such the a pointer can point
  std::vector<BackupPath::HpipmBound> hpipm_bounds_;
  Eigen::MatrixXd b0_;
};

}  // namespace planning
}  // namespace neodrive
