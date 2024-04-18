#include "kinematics.h"

#include <Eigen/SVD>
#include <sstream>
#include "neolix_log.h"
#include "src/planning/config/planning_config.h"
namespace neodrive {
namespace planning {
namespace math {

ModelManager::ModelManager() {}

void ModelManager::CVM(Eigen::MatrixXd &F, Eigen::MatrixXd &B,
                       Eigen::MatrixXd &W, Eigen::MatrixXd &H,
                       Eigen::MatrixXd &V, double dT, double w) {
  double dT_2 = dT * dT / 2;
  F.setIdentity(6, 6);
  B.setZero(6, 1);
  W.setZero(6, 2);
  H.setIdentity(4, 6);
  V.setIdentity(6, 6);

  F.bottomRightCorner(2, 2) = Eigen::Matrix2d::Zero();
  F.block<2, 2>(0, 2) = Eigen::Matrix2d::Identity() * dT;
  W.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity() * dT_2;
  W.block<2, 2>(2, 0) = Eigen::Matrix2d::Identity() * dT;
}

void ModelManager::CAM(Eigen::MatrixXd &F, Eigen::MatrixXd &B,
                       Eigen::MatrixXd &W, Eigen::MatrixXd &H,
                       Eigen::MatrixXd &V, double dT, double w) {
  double dT_2 = dT * dT / 2;
  double dT_3 = dT_2 * dT / 3;
  F.setIdentity(6, 6);
  B.setZero(6, 1);
  W.setZero(6, 2);
  H.setIdentity(6, 6);
  V.setIdentity(6, 6);

  F.block<2, 2>(0, 2) = Eigen::Matrix2d::Identity() * dT;
  F.block<2, 2>(2, 4) = Eigen::Matrix2d::Identity() * dT;
  F.topRightCorner(2, 2) = Eigen::Matrix2d::Identity() * dT_2;
  W.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity() * dT_3;
  W.block<2, 2>(2, 0) = Eigen::Matrix2d::Identity() * dT_2;
  W.block<2, 2>(4, 0) = Eigen::Matrix2d::Identity() * dT;
}

void ModelManager::CTM(Eigen::MatrixXd &F, Eigen::MatrixXd &J,
                       Eigen::MatrixXd &J0, Eigen::MatrixXd &B,
                       Eigen::MatrixXd &W, Eigen::MatrixXd &H,
                       Eigen::MatrixXd &V, double dT, double w, double alpha) {
  double dT_2 = dT * dT / 2;
  double wt = w * dT;
  double sinwt = std::sin(wt);
  double coswt = std::cos(wt) - 1;
  double sinDw = math::KSign(w, 1e-5) == 0 ? dT : sinwt / w;
  double cosDw = math::KSign(w, 1e-5) == 0 ? 0.0 : coswt / w;
  double cosDw2 = math::KSign(w, 1e-5) == 0 ? 0.0 : (coswt + 1) / w;
  F.setZero(5, 5);
  B.setZero(5, 1);
  W.setZero(5, 3);
  H.setIdentity(5, 5);
  V.setIdentity(5, 5);
  J.setZero(5, 5);
  J0.setIdentity(5, 5);

  F.block<2, 2>(0, 0).setIdentity();
  F.block<2, 2>(0, 2) << sinDw, cosDw, -cosDw, sinDw;
  F.block<2, 2>(2, 2) << coswt + 1, -sinwt, sinwt, coswt + 1;
  F(4, 4) = alpha;

  J = F;
  J0.block<2, 2>(0, 2) = Eigen::Matrix2d::Identity() * dT;
  J0(4, 4) = alpha;

  W.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity() * dT_2;
  W.block<2, 2>(2, 0) = Eigen::Matrix2d::Identity() * dT;
  W(4, 2) = 1;
}

void ModelManager::NonlinearSEQUpdate(const std::string name,
                                        Eigen::MatrixXd &F, Eigen::VectorXd &x,
                                        Eigen::MatrixXd &J, Eigen::MatrixXd &J0,
                                        double dT) {
  if (model_name.count(name) == 0) return;
  switch (model_name.at(name)) {
    case 2:
      FJacboianCTM(F, x, J, J0, dT);
    default:
      return;
  }
  return;
}

void ModelManager::NonlinearMEQUpdate(const std::string name,
                                        Eigen::MatrixXd &F, Eigen::VectorXd &x,
                                        Eigen::MatrixXd &J, Eigen::MatrixXd &J0,
                                        double dT) {
  if (model_name.count(name) == 0) return;
  switch (model_name.at(name)) {
    default:
      return;
  }
  return;
}

int ModelManager::FJacboianCTM(Eigen::MatrixXd &F, Eigen::VectorXd &x,
                                 Eigen::MatrixXd &J, Eigen::MatrixXd &J0,
                                 double dT) {
  double dT_2 = dT * dT / 2;
  double wt = x(4, 0) * dT;
  double sinwt = std::sin(wt);
  double coswt = std::cos(wt) - 1;
  double sinDw = math::KSign(x(4, 0), 1e-5) == 0 ? dT : sinwt / x(4, 0);
  double cosDw = math::KSign(x(4, 0), 1e-5) == 0 ? 0.0 : coswt / x(4, 0);
  double cosDw2 = math::KSign(x(4, 0), 1e-5) == 0 ? 0.0 : (coswt + 1) / x(4, 0);

  F.block<2, 2>(0, 2) << sinDw, cosDw, -cosDw, sinDw;
  F.block<2, 2>(2, 2) << coswt + 1, -sinwt, sinwt, coswt + 1;

  if (math::KSign(x(4, 0), 1e-5)) {
    J.block<2, 2>(0, 2) = F.block<2, 2>(0, 2);
    J.block<2, 2>(2, 2) = F.block<2, 2>(2, 2);
    J.block<4, 1>(0, 4) << (cosDw2 * x(2, 0) - sinDw * x(3, 0)) * dT -
                               (sinDw * x(2, 0) + cosDw * x(3, 0)) / x(4, 0),
        (sinDw * x(2, 0) + cosDw2 * x(3, 0)) * dT +
            (cosDw * x(2, 0) - sinDw * x(3, 0)) / x(4, 0),
        -dT * (sinwt * x(2, 0) + (coswt + 1) * x(3, 0)),
        dT * ((coswt + 1) * x(2, 0) - sinwt * x(3, 0));
  } else {
    J.block<4, 1>(0, 4).setZero();
  }
  return math::KSign(x(4, 0), 1e-5) != 0;
}

const Eigen::MatrixXd &ModelManager::SEQ_Jacboian(
    const std::string name, const Eigen::MatrixXd &J, const Eigen::MatrixXd &J0,
    const Eigen::VectorXd &x) const {
  if (model_name.count(name) == 0) return J;
  switch (model_name.at(name)) {
    case 2:
      return math::KSign(x(4, 0), 1e-5) == 0 ? J0 : J;
    default:
      return J;
  }
}

Eigen::MatrixXd *ModelManager::mutable_SEQ_Jacboian(std::string name,
                                                    Eigen::MatrixXd *J,
                                                    Eigen::MatrixXd *J0,
                                                    Eigen::VectorXd &x) {
  if (model_name.count(name) == 0) return J;
  switch (model_name.at(name)) {
    case 2:
      return math::KSign(x(4, 0), 1e-5) == 0 ? J0 : J;
    default:
      return J;
  }
}

LinearSystemModel::LinearSystemModel(const std::string &model, double dT,
                                     double w) {
  auto &model_names = ModelManager::Instance()->model_name;
  if (model_names.count(model) == 0) return;
  int model_id = model_names[model];
  model_name = model;
  switch (model_id) {
    case 0:
      ModelManager::Instance()->CVM(F_, B_, W_, H_, V_, dT, w);
      break;
    case 1:
      ModelManager::Instance()->CAM(F_, B_, W_, H_, V_, dT, w);
      break;
    case 2:
      is_linear = false;
      ModelManager::Instance()->CTM(F_, J_, J0_, B_, W_, H_, V_, dT, w, 0.95);
      break;
    default:
      break;
  }
  x_.setZero(F_.rows(), 1);
  z_.setZero(H_.rows(), 1);
}

void LinearSystemModel::Update() {
  ModelManager::Instance()->NonlinearSEQUpdate(model_name, F_, x_, J_, J0_,
                                                 0.1);
  ModelManager::Instance()->NonlinearMEQUpdate(model_name, H_, x_, P_, P0_,
                                                 0.1);
}

void LinearSystemModel::Predict(int pstep,
                                std::vector<Eigen::VectorXd> &pstate) {
  Eigen::MatrixXd tmp_F(F_);
  Eigen::MatrixXd tmp_J(J_);
  Eigen::MatrixXd tmp_J0(J0_);
  Eigen::VectorXd tmp_x(x_);

  pstate.clear();
  while (pstep--) {
    ModelManager::Instance()->NonlinearSEQUpdate(model_name, tmp_F, tmp_x,
                                                   tmp_J, tmp_J0, 0.1);
    tmp_x = tmp_F * tmp_x;
    pstate.push_back(tmp_x);
  }
}

void LinearSystemModel::Predict(int pstep,
                                std::vector<std::array<double, 2>> &pstate) {
  Eigen::MatrixXd tmp_F(F_);
  Eigen::MatrixXd tmp_J(J_);
  Eigen::MatrixXd tmp_J0(J0_);
  Eigen::VectorXd tmp_x(x_);
  tmp_F(4, 4) = 1.05;

  pstate.clear();
  while (pstep--) {
    ModelManager::Instance()->NonlinearSEQUpdate(model_name, tmp_F, tmp_x,
                                                   tmp_J, tmp_J0, 0.1);
    tmp_x = tmp_F * tmp_x;
    pstate.push_back({tmp_x[0], tmp_x[1]});
  }
}

KalmanFilter::KalmanFilter(const std::string &model, double dT, double w,
                           Eigen::MatrixXd &r, Eigen::MatrixXd &q, bool online)
    : S_{std::make_unique<LinearSystemModel>(model, dT, w)},
      R_{r},
      online_{online},
      is_init{false},
      k_{0},
      step{dT} {
  Q_.setZero(S_.get()->W().rows(), S_.get()->W().cols());
  Q_ = S_.get()->W() * q * S_.get()->W().transpose();
  K_.setZero(S_.get()->F().rows(), S_.get()->H().rows());
  P_.setZero(S_.get()->F().rows(), S_.get()->F().cols());
  Z_.setZero(S_.get()->F().rows(), S_.get()->F().cols());
  M_ = PseudoInverse(S_.get()->H().transpose() * S_.get()->H()) *
       S_.get()->H().transpose();
}

void MatrixPrint(const Eigen::MatrixXd &M, std::string name) {
  std::ostringstream sout;
  sout << name << ": ";
  for (int i = 0; i < M.size(); i++) {
    sout << *(M.data() + i) << " ";
  }
  LOG_DEBUG("{}", sout.str());
}

void MatrixPrint(const Eigen::VectorXd &M, std::string name) {
  std::ostringstream sout;
  sout << name << ": ";
  for (int i = 0; i < M.size(); i++) {
    sout << M[i] << " ";
  }
  LOG_DEBUG("{}", sout.str());
};

void KalmanFilter::Init(Eigen::VectorXd &z) {
  if (online_ && !is_init) {
    FM_ = S_.get()->F() * M_;         // constant
    MRM_ = M_ * R_ * M_.transpose();  // constant
    V_ = MRM_ - S_.get()->F() * MRM_ * S_.get()->F().transpose();
    Z_ = V_ + Q_;
    *S_.get()->mutable_z() = z;
    is_init = true;
  } else {
    return;
  }
}

void KalmanFilter::Estimation(const std::array<double, 6> &az) {
  Eigen::VectorXd z(6);
  z << az[0], az[1], az[2], az[3], az[4], az[5];
  k_++;
  if (!online_) {
    *S_.get()->mutable_z() = z;
    return;
  } else {
    Init(z);
    auto &x = *S_.get()->mutable_x();
    x(0, 0) = z(0, 0);
    x(1, 0) = z(1, 0);
  }

  Eigen::VectorXd zeta = M_ * z - FM_ * S_.get()->z();
  Z_ -= (Z_ - zeta * zeta.transpose()) / k_;
  Q_ = Z_ - V_;
  *S_.get()->mutable_z() = z;
}

void KalmanFilter::Predict() {
  P_bar = S_.get()->F() * P_ * S_.get()->F().transpose() + Q_;
  X_bar = S_.get()->F() * S_.get()->x();
}

void KalmanFilter::Update() {
  Eigen::MatrixXd O = S_.get()->H() * P_bar * S_.get()->H().transpose() + R_;
  K_ = P_bar * S_.get()->H().transpose() * PseudoInverse(O);
  *S_.get()->mutable_x() = X_bar + K_ * (S_.get()->z() - S_.get()->H() * X_bar);
  P_ = (Eigen::MatrixXd::Identity(K_.rows(), K_.rows()) - K_ * S_.get()->H()) *
       P_bar;
}

const Eigen::VectorXd &KalmanFilter::Process(const std::array<double, 6> &az) {
  Estimation(az);
  Predict();
  Update();
  S_.get()->Update();
  return S_.get()->x();
}

void KalmanFilter::Repr() {
  std::stringstream ss;
  std::vector<std::pair<std::string, std::string>> ans = {};

  ss << std::setprecision(4) << k_;
  ans.push_back(std::make_pair("matrix K", ss.str()));
  ss.str("");
  ans.push_back(std::make_pair("is_online", online_ ? "true" : "false"));
  ss << std::setprecision(4) << S_.get()->x();
  ans.push_back(std::make_pair("vector X", ss.str()));

  for (const auto &it : ans) {
    LOG_DEBUG("{}: {}", it.first, it.second);
  }
}

void TrackingDiff::Diff(math::AD2 &x, double v) {
  double fu = Fhan(x, v);
  x[0] += Ts_ * x[1];
  x[1] += Ts_ * fu;
}

double TrackingDiff::Fhan(math::AD2 &x, double v) {
  double y = x[0] + x[1] * h_ - v;
  double a = std::sqrt(kSign[0] * std::abs(y) + kSign[1]);
  a = std::abs(y) <= d0_ ? y / h_ + x[1]
                         : x[1] + (a - d_) * math::KSign(y, 1e-10) * 0.5;
  return std::abs(a) <= d_ ? -r_ * a / d_ : -r_ * math::KSign(a, 1e-10);
}

std::unique_ptr<KalmanFilter> CVM(const std::array<double, 6> &x0, double dT,
                                  double w) {
  Eigen::MatrixXd R;
  const auto &kf_R = config::PlanningConfig::Instance()
                         ->planning_research_config()
                         .multimodel_kalman_config.general.lm_cov_r;
  const auto &kf_Q = config::PlanningConfig::Instance()
                         ->planning_research_config()
                         .multimodel_kalman_config.general.cv_cov_q;
  R.setZero(4, 4);
  for (int i = 0; i < R.rows(); i++) R(i, i) = kf_R[i];

  Eigen::MatrixXd Q(2, 2);
  for (int i = 0; i < kf_Q.size(); i++) Q(i) = kf_Q[i];
  Q.transposeInPlace();
  return std::make_unique<KalmanFilter>("CVM", dT, w, R, Q, true);
}

std::unique_ptr<KalmanFilter> CTM(const std::array<double, 6> &x0, double dT,
                                  double w) {
  Eigen::MatrixXd R;
  const auto &kf_R = config::PlanningConfig::Instance()
                         ->planning_research_config()
                         .multimodel_kalman_config.general.nlm_cov_r;
  const auto &kf_Q = config::PlanningConfig::Instance()
                         ->planning_research_config()
                         .multimodel_kalman_config.general.act_cov_q;
  R.setZero(5, 5);
  for (int i = 0; i < R.rows(); i++) R(i, i) = kf_R[i];

  Eigen::MatrixXd Q(3, 3);
  for (int i = 0; i < kf_Q.size(); i++) Q(i) = kf_Q[i];
  Q.transposeInPlace();
  return std::make_unique<KalmanFilter>("CTM", dT, w, R, Q, true);
}

std::unique_ptr<KalmanFilter> CAM(const std::array<double, 6> &x0, double dT,
                                  double w) {
  Eigen::MatrixXd R;
  const auto &kf_R = config::PlanningConfig::Instance()
                         ->planning_research_config()
                         .multimodel_kalman_config.general.lm_cov_r;
  const auto &kf_Q = config::PlanningConfig::Instance()
                         ->planning_research_config()
                         .multimodel_kalman_config.general.ca_cov_q;
  R.setZero(6, 6);
  for (int i = 0; i < R.rows(); i++) R(i, i) = kf_R[i];

  Eigen::MatrixXd Q(2, 2);
  for (int i = 0; i < kf_Q.size(); i++) Q(i) = kf_Q[i];
  Q.transposeInPlace();
  return std::make_unique<KalmanFilter>("CAM", dT, w, R, Q, true);
}

Eigen::MatrixXd PseudoInverse(const Eigen::MatrixXd &m, double eps) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      m, Eigen::ComputeThinU | Eigen::ComputeThinV);
  return svd.matrixV() *
         (svd.singularValues().array().abs() > eps)
             .select(svd.singularValues().array().inverse(), 0)
             .matrix()
             .asDiagonal() *
         svd.matrixU().adjoint();
}

}  // namespace math
}  // namespace planning
}  // namespace neodrive