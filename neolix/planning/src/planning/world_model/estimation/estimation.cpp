#include "estimation.h"

#include <time.h>

#include "common/coordinate/coodrdinate_convertion.h"
#include "common_config/config/common_config.h"
#include "reader_manager/reader_manager.h"
#include "src/planning/world_model/common/world_model_context.h"
#include "world_model/config/world_model_config.h"

namespace neodrive {
namespace world_model {

Estimation::Estimation() {}

bool Estimation::Init() {
  if (initialized_) {
    return true;
  }
  LOG_INFO("Init");

  ego_state_.dr_accleration = {0, 0};
  ego_state_.dr_est_velocity[0][0] = ego_state_.dr_velocity[0];
  ego_state_.dr_est_velocity[1][0] = ego_state_.dr_velocity[1];
  ego_state_.ego_state = {ego_state_.dr_velocity[0],
                          ego_state_.dr_velocity[1],
                          ego_state_.dr_accleration[0],
                          ego_state_.dr_accleration[1],
                          0,
                          0,
                          0,
                          0.1,
                          0,
                          0};
  initialized_ = true;
  return true;
}

void Estimation::Estimate() {
  const auto ego_kinematics_estimation = config::WorldModelConfig::Instance()
                                             ->world_model_config()
                                             .ego_kinematics_estimation;
  if (!ego_kinematics_estimation) {
    return;
  } else {
    auto &unit = WorldModelContxt::Instance()->kinematics_estimate_msg;
    unit.is_updated = false;
    unit.reader->Observe();
    auto msg_ptr = unit.reader->GetLatestObserved();
    if (msg_ptr != nullptr) {
      unit.ptr = msg_ptr;
      unit.is_updated = true;
      unit.is_available = true;

      LOG_INFO(
          "original kinematics msg is {}update.",
          WorldModelContxt::Instance()->kinematics_estimate_msg.is_available
              ? ""
              : "not ");
      if (WorldModelContxt::Instance()->kinematics_estimate_msg.is_available !=
          false) {
        auto &twist_odom_msg =
            WorldModelContxt::Instance()->kinematics_estimate_msg.ptr->twist();
        auto drresult_predict =
            WorldModelContxt::Instance()->dr_result_msg.ptr->odometry_predict();
        auto msf_status = WorldModelContxt::Instance()
                              ->localization_estimate_msg.ptr->msf_status();
        auto chaissis_speed =
            WorldModelContxt::Instance()->chassis_msg.ptr->speed_mps();
        auto &loc_msg =
            WorldModelContxt::Instance()->dr_result_msg.ptr->odometry_pose();

        ego_state_.imu_accleration = {twist_odom_msg.linear_acc().x(),
                                      twist_odom_msg.linear_acc().y()};
        ego_state_.ego_state[5] = twist_odom_msg.angular().z();
        ego_state_.ego_state[4] =
            common::GetYawFromQuaternion(loc_msg.orientation()) - M_PI_2 * 3;
        ego_state_.ego_state[8] = loc_msg.position().x();
        ego_state_.ego_state[9] = loc_msg.position().y();

        if (!drresult_predict) {
          ego_state_.dr_velocity = {twist_odom_msg.linear().x(),
                                    twist_odom_msg.linear().y()};
        } else {
          ego_state_.dr_velocity = {
              chaissis_speed * cos(ego_state_.ego_state[4]),
              chaissis_speed * sin(ego_state_.ego_state[4])};
        }

        if (!ego_state_.init) {
          for (int i = 0; i < 2; i++) {
            ego_state_.dr_est_velocity[i][0] = ego_state_.dr_velocity[i];
            ego_state_.dr_est_velocity[i][1] = ego_state_.imu_accleration[i];
          }
          ego_state_.ego_state[0] = ego_state_.dr_velocity[0];
          ego_state_.ego_state[1] = ego_state_.dr_velocity[1];
          ego_state_.ego_state[2] = ego_state_.imu_accleration[0];
          ego_state_.ego_state[3] = ego_state_.imu_accleration[1];
          ego_ = std::make_shared<GlobalObsInfo>(ego_state_.ego_state);
          std::array<std::shared_ptr<planning::math::KalmanFilter>, 3> kf{
              planning::math::CVM(ego_->trajectory.front().x, 0.1, 0.0),
              planning::math::CAM(ego_->trajectory.front().x, 0.1, 0.0),
              planning::math::CTM(ego_->trajectory.front().x, 0.1, 0.0)};
          ego_kinematics_ =
              ObsKinematicsInfo(std::weak_ptr(ego_), kf[0], kf[1], kf[2]);
        }
      } else {
        return;
      }
    } else {
      LOG_INFO("msg ptr is null");
      return;
    }
  }

  if (ego_state_.init) {
    for (int i = 0; i < 2; i++) {
      td_1D_1e1_.Diff(ego_state_.dr_est_velocity[i], ego_state_.dr_velocity[i]);
      ego_state_.dr_accleration[i] = ego_state_.dr_est_velocity[i][1];
    }
    std::array<double, 2> w_vel{0.05, 0.95}, w_acc{0.5, 0.5};
    ego_state_.ego_state[0] = w_vel[0] * ego_state_.dr_velocity[0] +
                              w_vel[1] * ego_state_.dr_est_velocity[0][0];
    ego_state_.ego_state[1] = w_vel[0] * ego_state_.dr_velocity[1] +
                              w_vel[1] * ego_state_.dr_est_velocity[1][0];
    ego_state_.ego_state[2] = w_acc[0] * ego_state_.dr_accleration[0] +
                              w_acc[1] * ego_state_.imu_accleration[0];
    ego_state_.ego_state[3] = w_acc[0] * ego_state_.dr_accleration[1] +
                              w_acc[1] * ego_state_.imu_accleration[1];

    ego_->Update(ego_state_.ego_state, true);
    LOG_INFO("ego state is updated.");
    LOG_INFO("[{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}]",
             ego_state_.ego_state[8], ego_state_.ego_state[9],
             ego_state_.dr_velocity[0], ego_state_.dr_velocity[1],
             ego_state_.imu_accleration[0], ego_state_.imu_accleration[1],
             ego_->prediction.front()["cam"](0, 0),
             ego_->prediction.front()["cam"](1, 0),
             ego_->prediction.front()["cam"](2, 0),
             ego_->prediction.front()["cam"](3, 0),
             ego_->prediction.front()["cam"](4, 0),
             ego_->prediction.front()["cam"](5, 0),
             ego_state_.dr_est_velocity[0][0], ego_state_.dr_est_velocity[1][0],
             ego_state_.dr_est_velocity[0][1], ego_state_.dr_est_velocity[1][1],
             ego_state_.ego_state[4]);
  } else {
    ego_state_.init = true;
    ego_->Predict();
  }
}

}  // namespace world_model
}  // namespace neodrive
