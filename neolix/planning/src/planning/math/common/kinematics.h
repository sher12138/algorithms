#pragma once

#include <deque>
#include <functional>
#include <list>
#include <map>
#include <queue>
#include <vector>

#include "Eigen/Core"
#include "Eigen/LU"
#include "cyber/common/macros.h"
//#include "src/planning/math/common/geometry.h"

namespace neodrive {
namespace planning {
namespace math {

using AD2 = std::array<double, 2>;
inline int KSign(const double x, double eps = 1e-3) {
  return x < -eps ? -1 : x > eps ? 1 : 0;
}

class ModelManager {
  DECLARE_SINGLETON(ModelManager);
  ~ModelManager() = default;

 public:
  /* model description */
  void CVM(Eigen::MatrixXd &F, Eigen::MatrixXd &B, Eigen::MatrixXd &W,
           Eigen::MatrixXd &H, Eigen::MatrixXd &V, double dT, double w);
  void CAM(Eigen::MatrixXd &F, Eigen::MatrixXd &B, Eigen::MatrixXd &W,
           Eigen::MatrixXd &H, Eigen::MatrixXd &V, double dT, double w);
  void CTM(Eigen::MatrixXd &F, Eigen::MatrixXd &J, Eigen::MatrixXd &J0,
           Eigen::MatrixXd &B, Eigen::MatrixXd &W, Eigen::MatrixXd &H,
           Eigen::MatrixXd &V, double dT, double w, double alpha);
  /* model calculation */
  void NonlinearSEQUpdate(const std::string name, Eigen::MatrixXd &F,
                            Eigen::VectorXd &x, Eigen::MatrixXd &J,
                            Eigen::MatrixXd &J0, double dT = 0.1);
  void NonlinearMEQUpdate(const std::string name, Eigen::MatrixXd &H,
                            Eigen::VectorXd &x, Eigen::MatrixXd &J,
                            Eigen::MatrixXd &J0, double dT = 0.1);
  int FJacboianCTM(Eigen::MatrixXd &F, Eigen::VectorXd &x, Eigen::MatrixXd &J,
                     Eigen::MatrixXd &J0, double dT = 0.1);

  /* model observation */
  const Eigen::MatrixXd &SEQ_Jacboian(const std::string name,
                                      const Eigen::MatrixXd &J,
                                      const Eigen::MatrixXd &J0,
                                      const Eigen::VectorXd &x) const;
  Eigen::MatrixXd *mutable_SEQ_Jacboian(std::string name, Eigen::MatrixXd *J,
                                        Eigen::MatrixXd *J0,
                                        Eigen::VectorXd &x);

 public:
  std::map<std::string, int> model_name{{{"CVM", 0}, {"CAM", 1}, {"CTM", 2}}};
};

class LinearSystemModel {
 public:
   LinearSystemModel();
   LinearSystemModel(const std::string &model, double dT,
                     double w);
   const Eigen::MatrixXd &J() const {
     return ModelManager::Instance()->SEQ_Jacboian(model_name, J_, J0_, x_);
   }
   Eigen::MatrixXd *mutable_J() {
     return ModelManager::Instance()->mutable_SEQ_Jacboian(model_name, &J_,
                                                           &J0_, x_);
   }
   DEFINE_COMPLEX_TYPE_GET_FUNCTION(Eigen::MatrixXd, F);
   DEFINE_COMPLEX_TYPE_GET_FUNCTION(Eigen::MatrixXd, H);
   DEFINE_COMPLEX_TYPE_GET_FUNCTION(Eigen::MatrixXd, B);
   DEFINE_COMPLEX_TYPE_GET_FUNCTION(Eigen::MatrixXd, W);
   DEFINE_COMPLEX_TYPE_GET_FUNCTION(Eigen::MatrixXd, V);
   DEFINE_COMPLEX_TYPE_GET_FUNCTION(Eigen::VectorXd, x);
   DEFINE_COMPLEX_TYPE_GET_FUNCTION(Eigen::VectorXd, z);
   void Update();
   void Predict(int pstep, std::vector<Eigen::VectorXd> &pstate);
   void Predict(int pstep, std::vector<std::array<double, 2>> &pstate);

 private:
  Eigen::MatrixXd F_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd W_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd V_;

   /* SEQ linearization */
   Eigen::MatrixXd J_;
   Eigen::MatrixXd J0_;

   /* MEQ linearization */
   Eigen::MatrixXd P_;
   Eigen::MatrixXd P0_;

   /* signals  */
   Eigen::VectorXd x_;
   Eigen::VectorXd z_;

  public:
   std::string model_name{""};
   bool is_linear{true};
};

class KalmanFilter {
 public:
  KalmanFilter();
  KalmanFilter(const std::string &model, double dT, double w,
               Eigen::MatrixXd &r, Eigen::MatrixXd &q, bool online = false);
  void Init(Eigen::VectorXd &z);
  void Estimation(const std::array<double, 6> &x0);
  void Predict();
  void Update();
  void Repr();
  const Eigen::VectorXd &Process(const std::array<double, 6> &z);

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(std::unique_ptr<LinearSystemModel>, S);

 private:
  std::unique_ptr<LinearSystemModel> S_{nullptr};
  Eigen::MatrixXd Q_;
  Eigen::Matrix2d q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd M_;
  Eigen::MatrixXd Z_;
  Eigen::MatrixXd K_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd FM_;
  Eigen::MatrixXd MRM_;
  Eigen::MatrixXd V_;
  Eigen::MatrixXd X_bar;
  Eigen::MatrixXd P_bar;
  int k_{0};
  double step{0.1};
  bool online_{false};
  bool is_init{false};
};

class TrackingDiff{
 public:
   TrackingDiff() = default;
   TrackingDiff(double Ts, double r, double h)
       : Ts_{Ts}, r_{r}, h_{h} {
     d_ = r * h;
     d0_ = d_ * h;
     kSign[0] = 8 * r;
     kSign[1] = d_ * d_;
   }
   void Diff(math::AD2 &x, double v);
   double Fhan(math::AD2 &x, double v);

  private:
   double Ts_{0.01};
   double r_{100};
   double h_{0.01};
   double d_{1};
   double d0_{0.01};
   math::AD2 kSign{800, 1};
};

std::unique_ptr<KalmanFilter> CVM(const std::array<double, 6> &x0, double dT, double w);

std::unique_ptr<KalmanFilter> CAM(const std::array<double, 6> &x0, double dT,
                                  double w);
std::unique_ptr<KalmanFilter> CTM(const std::array<double, 6> &x0, double dT, double w);

void MatrixPrint(const Eigen::MatrixXd &M, std::string name);

void MatrixPrint(const Eigen::VectorXd &M, std::string name);

Eigen::MatrixXd PseudoInverse(const Eigen::MatrixXd &m, double eps = 1.0e-5);

}  // namespace math
}  // namespace planning
}  // namespace neodrive