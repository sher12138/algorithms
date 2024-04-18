/// This is the component class derived from cyber::Component
#include <json.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include "common/utils/time_logger.h"
#include "config/planning_rl_config.h"
#include "container/container_manager.h"
#include "cyber.h"
#include "model_trt/feature_net/featurenet_evaluator.h"
#include "model_trt/imitation_net/imitation_evaluator.h"
#include "neolix_log.h"
#include "perception_obstacle.pb.h"
#include "planning.pb.h"
#include "planning_rl_manager/planning_rl_manager.h"
#include "prediction_obstacles.pb.h"
/// neodrive::planning_rl
namespace neodrive {
namespace planning_rl {

using neodrive::global::localization::LocalizationEstimate;
using neodrive::global::perception::PerceptionObstacles;
using neodrive::global::planning::ADCTrajectory;

/// PlanningRLComponent, to define the main process of prediction
class PlanningRLComponent : public neodrive::cyber::Component<> {
 public:
  /// Destructor
  ~PlanningRLComponent() = default;

  /// @brief Initialize the node
  /// @return If initialized
  bool Init() override;

 private:
  /// @brief Main process for each planning_rl frame
  bool Proc();
  void RunOnce();

  bool CheckData();

  bool WriteJson();

  bool DelayDetect();

 private:
  /// Writer for output channel
  std::shared_ptr<neodrive::cyber::Writer<ADCTrajectory>> planning_writer_;
  std::shared_ptr<std::thread> proc_thread_{};
  ContainerManager* container_manager_{nullptr};
  PlanningRLManager* planning_rl_manager_{nullptr};
  ContainerMessageShrPtr container_msg_{std::make_shared<ContainerMessage>()};
  std::shared_ptr<ADCTrajectory> out_msg_{std::make_shared<ADCTrajectory>()};
  double period_duration_{0.01};
  size_t max_process_msgs_per_interval_{10};
  double prev_send_msg_time_{0};

  config::PlanningRLConfig* planning_rl_config_ =
      config::PlanningRLConfig::Instance();
  utils::TimeLogger time_log_{"planning_rl_pro"};
};

CYBER_REGISTER_COMPONENT(PlanningRLComponent)

}  // namespace planning_rl
}  // namespace neodrive
