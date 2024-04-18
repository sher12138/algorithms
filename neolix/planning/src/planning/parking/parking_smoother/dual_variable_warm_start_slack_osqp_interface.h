#pragma once

#include <limits>
#include <vector>

#include "src/planning/common/planning_logger.h"
#include "src/planning/parking/parking_config.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

class DualVariableWarmStartSlackOSQPInterface {
 public:
  DualVariableWarmStartSlackOSQPInterface(
      size_t horizon, double ts, const Eigen::MatrixXd& ego,
      const Eigen::MatrixXi& obstacles_edges_num, const size_t obstacles_num,
      const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
      const Eigen::MatrixXd& xWS,
      const ParkingDualVariableConfig& dualvariable_config);

  virtual ~DualVariableWarmStartSlackOSQPInterface() = default;

  void GetOptimizationResults(Eigen::MatrixXd* l_warm_up,
                              Eigen::MatrixXd* n_warm_up,
                              Eigen::MatrixXd* s_warm_up) const;

  bool Optimize();

  void AssembleP(std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
                 std::vector<c_int>* P_indptr);

  void AssembleConstraint(std::vector<c_float>* A_data,
                          std::vector<c_int>* A_indices,
                          std::vector<c_int>* A_indptr);

  void AssembleA(const int r, const int c, const std::vector<c_float>& P_data,
                 const std::vector<c_int>& P_indices,
                 const std::vector<c_int>& P_indptr);

  void PrintMatrix(const int r, const int c, const std::vector<c_float>& P_data,
                   const std::vector<c_int>& P_indices,
                   const std::vector<c_int>& P_indptr);

 private:
  int num_of_variables_;
  int num_of_constraints_;
  int horizon_;
  double ts_;
  Eigen::MatrixXd ego_;
  int lambda_horizon_ = 0;
  int miu_horizon_ = 0;
  int slack_horizon_ = 0;
  double beta_ = 0.0;

  Eigen::MatrixXd l_warm_up_;
  Eigen::MatrixXd n_warm_up_;
  Eigen::MatrixXd slacks_;
  double wheelbase_;

  double w_ev_;
  double l_ev_;
  std::vector<double> g_;
  double offset_;
  Eigen::MatrixXi obstacles_edges_num_;
  int obstacles_num_;
  int obstacles_edges_sum_;

  double min_safety_distance_;

  // lagrangian l start index
  int l_start_index_ = 0;

  // lagrangian n start index
  int n_start_index_ = 0;

  // slack s start index
  int s_start_index_ = 0;

  // obstacles_A
  Eigen::MatrixXd obstacles_A_;

  // obstacles_b
  Eigen::MatrixXd obstacles_b_;

  // states of warm up stage
  Eigen::MatrixXd xWS_;

  // constraint A matrix in eigen format
  Eigen::MatrixXf constraint_A_;

  bool check_mode_;
  ParkingDualVariableConfig dualvariable_config_;
};

}  // namespace planning
}  // namespace neodrive
