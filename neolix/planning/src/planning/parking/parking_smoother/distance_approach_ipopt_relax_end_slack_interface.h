#pragma once

#include <algorithm>
#include <limits>
#include <vector>

#include "distance_approach_interface.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/parking/parking_config.h"
#include "src/planning/public/planning_lib_header.h"

#define tag_f 1
#define tag_g 2
#define tag_L 3
#define HPOFF 30

namespace neodrive {
namespace planning {

class DistanceApproachIPOPTRelaxEndSlackInterface
    : public DistanceApproachInterface {
 public:
  explicit DistanceApproachIPOPTRelaxEndSlackInterface(
      const size_t horizon, const double ts, const Eigen::MatrixXd& ego,
      const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
      const Eigen::MatrixXd& l_warm_up, const Eigen::MatrixXd& n_warm_up,
      const Eigen::MatrixXd& s_warm_up, const Eigen::MatrixXd& x0,
      const Eigen::MatrixXd& xf, const Eigen::MatrixXd& last_time_u,
      const std::vector<double>& XYbounds,
      const Eigen::MatrixXi& obstacles_edges_num, const size_t obstacles_num,
      const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
      const DistanceApproachConfig& distance_approch_config,
      const ParkingCommonConfig common_config);

  virtual ~DistanceApproachIPOPTRelaxEndSlackInterface() = default;

  /** Method to return some info about the nlp */
  bool GetNlpInfo(int& n, int& m, int& nnz_jac_g, int& nnz_h_lag,  // NOLINT
                  IndexStyleEnum& index_style) override;           // NOLINT

  /** Method to return the bounds for my problem */
  bool GetBoundsInfo(int n, double* x_l, double* x_u, int m, double* g_l,
                     double* g_u) override;

  /** Method to return the starting point for the algorithm */
  bool GetStartingPoint(int n, bool init_x, double* x, bool init_z, double* z_L,
                        double* z_U, int m, bool init_lambda,
                        double* lambda) override;

  /** Method to return the objective value */
  bool EvalF(int n, const double* x, bool new_x, double& obj_value) override;

  /** Method to return the gradient of the objective */
  bool EvalGradF(int n, const double* x, bool new_x, double* grad_f) override;

  /** Method to return the constraint residuals */
  bool EvalG(int n, const double* x, bool new_x, int m, double* g) override;

  /** Check unfeasible constraints for further study**/
  bool CheckG(int n, const double* x, int m, const double* g);

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is nullptr)
   *   2) The values of the jacobian (if "values" is not nullptr)
   */
  bool EvalJacG(int n, const double* x, bool new_x, int m, int nele_jac,
                int* iRow, int* jCol, double* values) override;

  // sequential implementation to jac_g
  bool EvalJacGSer(int n, const double* x, bool new_x, int m, int nele_jac,
                   int* iRow, int* jCol, double* values) override;

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is
   * nullptr) 2) The values of the hessian of the lagrangian (if "values" is not
   * nullptr)
   */
  bool EvalH(int n, const double* x, bool new_x, double obj_factor, int m,
             const double* lambda, bool new_lambda, int nele_hess, int* iRow,
             int* jCol, double* values) override;

  /** @name Solution Methods */
  /** This method is called when the algorithm is complete so the TNLP can
   * store/write the solution */
  void FinalizeSolution(Ipopt::SolverReturn status, int n, const double* x,
                        const double* z_L, const double* z_U, int m,
                        const double* g, const double* lambda, double obj_value,
                        const Ipopt::IpoptData* ip_data,
                        Ipopt::IpoptCalculatedQuantities* ip_cq) override;

  void GetOptimizationResults(Eigen::MatrixXd* state_result,
                              Eigen::MatrixXd* control_result,
                              Eigen::MatrixXd* time_result,
                              Eigen::MatrixXd* dual_l_result,
                              Eigen::MatrixXd* dual_n_result) const override;

  //***************    start ADOL-C part ***********************************
  /** Template to return the objective value */
  template <class T>
  void eval_obj(int n, const T* x, T* obj_value);

  /** Template to compute constraints */
  template <class T>
  void eval_constraints(int n, const T* x, int m, T* g);

  /** Method to generate the required tapes by ADOL-C*/
  void generate_tapes(int n, int m, int* nnz_jac_g, int* nnz_h_lag);
  //***************    end   ADOL-C part ***********************************

 private:
  int num_of_variables_ = 0;
  int num_of_constraints_ = 0;
  int horizon_ = 0;
  int lambda_horizon_ = 0;
  int miu_horizon_ = 0;
  int slack_horizon_ = 0;
  double ts_ = 0.0;

  Eigen::MatrixXd ego_;
  Eigen::MatrixXd xWS_;
  Eigen::MatrixXd uWS_;
  Eigen::MatrixXd l_warm_up_;
  Eigen::MatrixXd n_warm_up_;
  Eigen::MatrixXd slack_warm_up_;
  Eigen::MatrixXd x0_;
  Eigen::MatrixXd xf_;
  Eigen::MatrixXd last_time_u_;
  std::vector<double> XYbounds_;

  // debug flag
  bool enable_constraint_check_;

  // penalty
  double weight_state_x_ = 0.0;
  double weight_state_y_ = 0.0;
  double weight_state_phi_ = 0.0;
  double weight_state_v_ = 0.0;
  double weight_input_steer_ = 0.0;
  double weight_input_a_ = 0.0;
  double weight_rate_steer_ = 0.0;
  double weight_rate_a_ = 0.0;
  double weight_stitching_steer_ = 0.0;
  double weight_stitching_a_ = 0.0;
  double weight_first_order_time_ = 0.0;
  double weight_second_order_time_ = 0.0;
  double weight_end_state_ = 0.0;
  double weight_slack_ = 0.0;

  double w_ev_ = 0.0;
  double l_ev_ = 0.0;
  std::vector<double> g_;
  double offset_ = 0.0;
  Eigen::MatrixXi obstacles_edges_num_;
  int obstacles_num_ = 0;
  int obstacles_edges_sum_ = 0;
  double wheelbase_ = 0.0;

  Eigen::MatrixXd state_result_;
  Eigen::MatrixXd dual_l_result_;
  Eigen::MatrixXd dual_n_result_;
  Eigen::MatrixXd control_result_;
  Eigen::MatrixXd time_result_;
  Eigen::MatrixXd slack_result_;

  // obstacles_A
  Eigen::MatrixXd obstacles_A_;

  // obstacles_b
  Eigen::MatrixXd obstacles_b_;

  // whether to use fix time
  bool use_fix_time_ = false;

  // state start index
  int state_start_index_ = 0;

  // control start index.
  int control_start_index_ = 0;

  // time start index
  int time_start_index_ = 0;

  // lagrangian l start index
  int l_start_index_ = 0;

  // lagrangian n start index
  int n_start_index_ = 0;

  // slack s start index
  int slack_index_ = 0;

  double min_safety_distance_ = 0.0;

  double max_safety_distance_ = 0.0;

  double max_steer_angle_ = 0.0;

  double max_speed_forward_ = 0.0;

  double max_speed_reverse_ = 0.0;

  double max_acceleration_forward_ = 0.0;

  double max_acceleration_reverse_ = 0.0;

  double min_time_sample_scaling_ = 0.0;

  double max_time_sample_scaling_ = 0.0;

  double max_steer_rate_ = 0.0;

  double max_lambda_ = 0.0;

  double max_miu_ = 0.0;

  bool enable_jacobian_ad_ = false;

 private:
  DistanceApproachConfig distance_approach_config_;
  ParkingCommonConfig common_config_;

 private:
  //***************    start ADOL-C part ***********************************
  double* obj_lam;
  //** variables for sparsity exploitation
  unsigned int* rind_g; /* row indices    */
  unsigned int* cind_g; /* column indices */
  double* jacval;       /* values         */
  unsigned int* rind_L; /* row indices    */
  unsigned int* cind_L; /* column indices */
  double* hessval;      /* values */
  int nnz_jac;
  int nnz_L;
  int options_g[4];
  int options_L[4];
  //***************    end   ADOL-C part ***********************************
};

}  // namespace planning
}  // namespace neodrive
