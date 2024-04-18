#pragma once
#include <deque>

#include "common/reference_line/reference_line.h"
#include "common/reference_line/reference_point.h"
#include "common/utils/planning_code_define.h"
#include "common/utils/time_logger.h"
#include "config/planning_rl_config.h"
#include "container/container_manager.h"
#include "cyber.h"
#include "frame/agent.h"
#include "frame/ego.h"
#include "frame/frame.h"
#include "frame/frames.h"
#include "localization_dead_reckoning.pb.h"
#include "localization_pose.pb.h"
#include "neolix_log.h"
#include "perception_obstacle.pb.h"
#include "planning.pb.h"
// obs_representation
#include "common/feature/infer.h"
#include "common/feature/obs_representation_trt.h"
#include "common/feature/post_predict.h"
#include "common/feature/train_features.h"
#include "common/feature/train_features_v2.h"
#include "common/feature/ud_model_predict.h"
#include "common/utils/data_struct_convert.h"
#include "common/utils/obs_utils.h"
#include "cyber/common/memory_pool.h"

namespace neodrive {
namespace planning_rl {

using planning_rl::ErrorCode;

class PlanningRLManager {
  DECLARE_SINGLETON(PlanningRLManager);

 public:
  bool Init(const std::shared_ptr<neodrive::cyber::Node>& node);
  ErrorCode Process(ContainerMessageShrPtr& container_msg_);
  ErrorCode ProcessUD(ContainerMessageShrPtr& container_msg_);

 private:
  void PublishFakeTrajectory();
  bool ProcessEstopPlanning();
  void PublishRLFakeTrajectory();
  FeaturePreparation DataTransform(
      const ContainerMessageShrPtr& container_msg_);
  FeaturePreparationV2 DataTransformUD(
      const ContainerMessageShrPtr& container_msg_);
  void PredictInterface(FeaturePreparation& fp,
                        ContainerMessageShrPtr& container_msg_);
  void UDPredictInterface(FeaturePreparationV2& fp,
                          ContainerMessageShrPtr& container_msg_);
  void TransUtm2Odom(std::vector<TrajectoryPoint2D>& pred_traj_opt,
                     const ContainerMessageShrPtr& container_msg_);
  void ModelOutputFix(std::vector<double>& output, EgoInfo ego_info,
                      ContainerMessageShrPtr& container_msg_);
  void ModelOutputFixUD(std::vector<std::vector<double>>& output,
                        EgoInfo ego_info,
                        ContainerMessageShrPtr& container_msg_,
                        bool use_step_gap);
  int CheckSceneState(ContainerMessageShrPtr& container_msg_,
                      const FeaturePreparationV2& fp);
  bool PublishPlanningResult(
      const std::vector<TrajectoryPoint2D>& pred_traj_opt);
  bool PublishPlanningRLResult(
      const std::vector<TrajectoryPoint2D>& pred_traj_opt);
  bool PublishPlanningRLUDResult(const std::vector<PredState>& pred_traj);
  void FixPredResult(const std::vector<PredState>& predicts_odom,
                     const ContainerMessageShrPtr& container_msg_);
  void SaveUDResultText(const ContainerMessageShrPtr& container_msg_,
                        const FeaturePreparationV2& fp,
                        const std::vector<std::vector<double>>& output,
                        const std::vector<PredState>& predicts_odom);

 private:
  bool initialized_{false};
  ContainerMessageShrPtr container_msg_{std::make_shared<ContainerMessage>()};
  std::shared_ptr<neodrive::cyber::Node> node_{nullptr};
  std::shared_ptr<ADCTrajectory> fake_trajectory_{nullptr};

  uint32_t sequence_num_{0};
  std::shared_ptr<neodrive::cyber::Writer<ADCTrajectory>> planning_pub_;
  std::shared_ptr<neodrive::cyber::Writer<ADCTrajectory>> planning_rl_pub_;
  bool planning_trajectory_published_;
  bool planning_rl_trajectory_published_;
  double scene_start_time_{0.0};
  std::deque<std::vector<PredState>> pred_trajs_;
  std::deque<EgoFrame> ego_frames_;
  size_t pred_trajs_size_{10};
  double last_pred_time_{0.0};
  double last_reset_scene_time_{0.0};

  utils::TimeLogger manager_time_log_{"planning_rl_manager"};

  neodrive::cyber::common::MemoryPool<ADCTrajectory> fake_trajectory_msg_pool_;
  neodrive::cyber::common::MemoryPool<ADCTrajectory> planning_rl_msg_pool_;
  neodrive::cyber::common::MemoryPool<ADCTrajectory> estop_trajectory_msg_pool_;
};

}  // namespace planning_rl
}  // namespace neodrive
