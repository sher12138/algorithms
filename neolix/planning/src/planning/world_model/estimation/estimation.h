#pragma once

#include <memory>

#include "common/util/time_logger.h"
#include "cyber.h"
#include "src/planning/math/common/kinematics.h"
#include "util/thread_safe_object.h"
#include "world_model/coordinator_transform/coordinator_transform.h"

namespace neodrive {
namespace world_model {

class Estimation {
  DECLARE_SINGLETON(Estimation);

 public:
  bool Init();
  void Estimate();

 private:
  struct State {
    int frame_id{-1};
    std::array<double, 6> x{0.0};
    double dt{0.0};
    double w{0.0};
    State(const int f, const std::array<double, 6>& x0, double dT, double omega)
        : frame_id{f}, x{x0}, dt{dT}, w{omega} {}
  };
  struct GlobalObsInfo {
    std::deque<State> trajectory{};
    std::deque<std::unordered_map<std::string, Eigen::VectorXd>> prediction{{}};
    int id{0};
    int cycle{0};
    int birth{0};
    int last_time{0};

    GlobalObsInfo(const std::array<double, 10>& states) {
      trajectory.push_front(State(
          0, {states[8], states[9], states[0], states[1], states[2], states[3]},
          states[7], states[5]));
    }
    void Update(const std::array<double, 10>& states, bool do_prediect = true) {
      if (trajectory.size() >= Estimation::Instance()->max_prediction_length_) {
        trajectory.pop_back();
        prediction.pop_back();
      }
      trajectory.push_front(State(
          0, {states[8], states[9], states[0], states[1], states[2], states[3]},
          states[7], states[5]));
      if (do_prediect) Predict();
      last_time = 0;
    }
    void Predict() {
      auto& kinematics = Estimation::Instance()->ego_kinematics_;
      std::array<double, 6> ctm_state{
          trajectory.front().x[0], trajectory.front().x[1],
          trajectory.front().x[2], trajectory.front().x[3],
          trajectory.front().w,    0.0};

      prediction.push_front({});
      prediction.front().emplace(
          "cam", kinematics.ca_kf->Process(trajectory.front().x));
      prediction.front().emplace(
          "cvm", kinematics.cv_kf->Process(trajectory.front().x));
      prediction.front().emplace("ctm", kinematics.ct_kf->Process(ctm_state));
      auto& pred_cam = prediction.front().at("cam");
      auto& pred_cvm = prediction.front().at("cvm");
      auto& pred_ctm = prediction.front().at("ctm");
      pred_cam.conservativeResize(8);
      pred_cvm.conservativeResize(8);
      pred_ctm.conservativeResize(8);
    }
  };
  struct EgoState {
    std::array<double, 10> ego_state{0.0};
    planning::math::AD2 imu_accleration{0, 0};
    planning::math::AD2 dr_velocity{0, 0};
    planning::math::AD2 dr_accleration{0, 0};
    std::array<planning::math::AD2, 2> dr_est_velocity{{{0, 0}, {0, 0}}};
    bool init{false};
    EgoState() = default;
  };
  struct ObsKinematicsInfo {
    std::weak_ptr<GlobalObsInfo> obs_info{};
    std::shared_ptr<planning::math::KalmanFilter> cv_kf{};
    std::shared_ptr<planning::math::KalmanFilter> ca_kf{};
    std::shared_ptr<planning::math::KalmanFilter> ct_kf{};
    ObsKinematicsInfo() = default;
    ObsKinematicsInfo(std::weak_ptr<GlobalObsInfo> obs_info_,
                      std::shared_ptr<planning::math::KalmanFilter> cv_kf_,
                      std::shared_ptr<planning::math::KalmanFilter> ca_kf_,
                      std::shared_ptr<planning::math::KalmanFilter> ct_kf_)
        : obs_info{obs_info_}, cv_kf{cv_kf_}, ca_kf{ca_kf_}, ct_kf{ct_kf_} {}
  };

 private:
  bool initialized_{false};
  common::util::MessageUnit<TwistStamped> kinematics_estimation_base_link_msg;
  planning::math::TrackingDiff td_1D_1e1_{0.01, 300, 0.01};
  std::shared_ptr<GlobalObsInfo> ego_{};
  EgoState ego_state_{};
  ObsKinematicsInfo ego_kinematics_{};
  int max_prediction_length_{31};
};

}  // namespace world_model
}  // namespace neodrive
