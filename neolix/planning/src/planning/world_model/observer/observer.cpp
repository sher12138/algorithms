#include "observer.h"

#include <time.h>

#include "common_config/config/common_config.h"
#include "reader_manager/reader_manager.h"

namespace neodrive {
namespace world_model {

Observer::Observer() {}

bool Observer::Init(const std::shared_ptr<neodrive::cyber::Node> &node) {
  if (initialized_) {
    return true;
  }
  LOG_INFO("Init");
  auto &topics_config = neodrive::common::config::CommonConfig::Instance()
                            ->topics_config()
                            .topics;
  if (node == nullptr) {
    LOG_ERROR("node is nullptr!!!");
    return false;
  }
  node_ = node;
  auto reader_manager = neodrive::common::ReaderManager::Instance();
  reader_manager->Init(node_);
  neodrive::cyber::ReaderConfig config;
  std::string config_file_path =
      "/home/caros/cyberrt/conf/planning_localization_reader.conf";
  neodrive::cyber::common::GetProtoFromFile(config_file_path,
                                            &(config.qos_profile));

  config.channel_name = topics_config.at("localization_error_code").topic;
  config.pending_queue_size =
      topics_config.at("localization_error_code").queue_size;
  wm_context_->loc_error_code_msg.reader =
      reader_manager->CreateReader<LocalizationErrorCode>(config);

  config.channel_name = topics_config.at("localization_pose").topic;
  config.pending_queue_size = topics_config.at("localization_pose").queue_size;
  wm_context_->localization_estimate_msg.reader =
      reader_manager->CreateReader<LocalizationEstimate>(config);

  config.channel_name = topics_config.at("localization_odometry").topic;
  config.pending_queue_size =
      topics_config.at("localization_odometry").queue_size;
  wm_context_->dr_result_msg.reader =
      reader_manager->CreateReader<DRResult>(config);

  config.channel_name = topics_config.at("novatel_imu").topic;
  config.pending_queue_size = topics_config.at("novatel_imu").queue_size;
  wm_context_->imu_msg.reader = reader_manager->CreateReader<Imu>(config);

  config.channel_name =
      topics_config.at("kinematics_estimation_base_link").topic;
  config.pending_queue_size =
      topics_config.at("kinematics_estimation_base_link").queue_size;
  wm_context_->kinematics_estimate_msg.reader =
      reader_manager->CreateReader<TwistStamped>(config);

  config.channel_name = topics_config.at("planning_chassis").topic;
  config.pending_queue_size = topics_config.at("planning_chassis").queue_size;
  wm_context_->chassis_msg.reader =
      reader_manager->CreateReader<Chassis>(config);

  if (!coordinate_tranform_->Init(node_)) {
    LOG_ERROR("CoordinateTransform Init failed");
    return false;
  }

  initialized_ = true;
  return true;
}

void Observer::Observe() {
  WORLD_MODEL_OBSERVE_FUNCTION(wm_context_->loc_error_code_msg);
  WORLD_MODEL_OBSERVE_FUNCTION(wm_context_->imu_msg);
  WORLD_MODEL_OBSERVE_FUNCTION(wm_context_->chassis_msg);
  if (false == wm_context_->imu_msg.is_available) {
    return;
  }
  auto &imu_msg = *(wm_context_->imu_msg.ptr);
  auto &loc_error_code_msg = *(wm_context_->loc_error_code_msg.ptr);

  auto process_dr_func = [&](const std::shared_ptr<DRResult> &msg) {
    if (!CheckProto(*msg)) {
      return;
    }
    coordinate_tranform_->UpdateBaselinkPoseInOdom(*msg, imu_msg);
  };
  PROCESS_NEW_MESSAGES(wm_context_->dr_result_msg, process_dr_func);

  auto &dr_msg = *(wm_context_->dr_result_msg.ptr);

  auto process_utm_func =
      [&](const std::shared_ptr<LocalizationEstimate> &msg) {
        if (!CheckProto(*msg)) {
          return;
        }
        coordinate_tranform_->UpdateBaselinkPoseInUtm(
            *msg, imu_msg, loc_error_code_msg, dr_msg);
      };
  PROCESS_NEW_MESSAGES(wm_context_->localization_estimate_msg,
                       process_utm_func);
}

}  // namespace world_model
}  // namespace neodrive
