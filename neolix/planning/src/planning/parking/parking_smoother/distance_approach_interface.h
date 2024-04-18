#pragma once

//#include <omp.h>
#include "src/planning/public/planning_lib_header.h"

#define tag_f 1
#define tag_g 2
#define tag_L 3
#define HPOFF 30

namespace neodrive {
namespace planning {

class DistanceApproachInterface : public Ipopt::TNLP {
 public:
  virtual ~DistanceApproachInterface() = default;

  /** Method to return some info about the nlp */
  virtual bool GetNlpInfo(int& n, int& m, int& nnz_jac_g,    // NOLINT
                          int& nnz_h_lag,                    // NOLINT
                          IndexStyleEnum& index_style) = 0;  // NOLINT

  /** Method to return the bounds for my problem */
  virtual bool GetBoundsInfo(int n, double* x_l, double* x_u, int m,
                             double* g_l, double* g_u) = 0;

  /** Method to return the starting point for the algorithm */
  virtual bool GetStartingPoint(int n, bool init_x, double* x, bool init_z,
                                double* z_L, double* z_U, int m,
                                bool init_lambda, double* lambda) = 0;

  /** Method to return the objective value */
  virtual bool EvalF(int n, const double* x, bool new_x,  // NOLINT
                     double& obj_value) = 0;              // NOLINT

  /** Method to return the gradient of the objective */
  virtual bool EvalGradF(int n, const double* x, bool new_x,
                         double* grad_f) = 0;

  /** Method to return the constraint residuals */
  virtual bool EvalG(int n, const double* x, bool new_x, int m, double* g) = 0;

  /** Check unfeasible constraints for further study**/
  virtual bool CheckG(int n, const double* x, int m, const double* g) = 0;

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is nullptr)
   *   2) The values of the jacobian (if "values" is not nullptr)
   */
  virtual bool EvalJacG(int n, const double* x, bool new_x, int m, int nele_jac,
                        int* iRow, int* jCol, double* values) = 0;
  // sequential implementation to jac_g
  virtual bool EvalJacGSer(int n, const double* x, bool new_x, int m,
                           int nele_jac, int* iRow, int* jCol,
                           double* values) = 0;

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is
   * nullptr) 2) The values of the hessian of the lagrangian (if "values" is not
   * nullptr)
   */
  virtual bool EvalH(int n, const double* x, bool new_x, double obj_factor,
                     int m, const double* lambda, bool new_lambda,
                     int nele_hess, int* iRow, int* jCol, double* values) = 0;

  /** @name Solution Methods */
  /** This method is called when the algorithm is complete so the TNLP can
   * store/write the solution */
  virtual void FinalizeSolution(Ipopt::SolverReturn status, int n,
                                const double* x, const double* z_L,
                                const double* z_U, int m, const double* g,
                                const double* lambda, double obj_value,
                                const Ipopt::IpoptData* ip_data,
                                Ipopt::IpoptCalculatedQuantities* ip_cq) = 0;

  virtual void GetOptimizationResults(Eigen::MatrixXd* state_result,
                                      Eigen::MatrixXd* control_result,
                                      Eigen::MatrixXd* time_result,
                                      Eigen::MatrixXd* dual_l_result,
                                      Eigen::MatrixXd* dual_n_result) const = 0;
};

}  // namespace planning
}  // namespace neodrive
