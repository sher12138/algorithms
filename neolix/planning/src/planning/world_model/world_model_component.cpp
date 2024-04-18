#include "world_model_component.h"

#include "common/environment.h"
#include "common_config/config/common_config.h"
#include "reader_manager/reader_manager.h"
#include "world_model/config/world_model_config.h"
#include "world_model/observer/observer.h"
#include "world_model/world_model.h"

namespace neodrive {
namespace world_model {

using neodrive::cyber::Node;

WorldModelComponent::~WorldModelComponent() { proc_thread_->detach(); }

bool WorldModelComponent::Init() {
  if (initialized_) {
    return true;
  }
  LOG_INFO("WorldModelComponent init started.");
  if (!WorldModel::Instance()->Init(node_)) {
    LOG_ERROR("WorldModel init failed!");
    return false;
  }

  proc_thread_ = std::make_unique<std::thread>(
      std::bind(&WorldModelComponent::Proc, this));
  neodrive::cyber::scheduler::Instance()->SetInnerThreadAttr(
      "world_model_proc_thread", proc_thread_.get());
  LOG_INFO("WorldModelComponent init finished.");
  initialized_ = true;
  return true;
}

std::string WorldModelComponent::Name() const {
  return std::string("WorldModelComponent");
}

void WorldModelComponent::Proc() {
  LOG_WARN("[thread_name]world_model_component");
  uint64_t frequency = std::max(config::WorldModelConfig::Instance()
                                    ->world_model_config()
                                    .odom_pose_frequency,
                                1);
  uint64_t period_time = 1000000000UL / frequency;
  uint64_t start_time{0}, end_time{0}, delta_time{0};
  auto world_model = WorldModel::Instance();

  while (!neodrive::cyber::IsShutdown()) {
    start_time = cyber::Time::Now().ToNanosecond();
    world_model->RunOnce();
    end_time = cyber::Time::Now().ToNanosecond();
    if (end_time < start_time) {
      LOG_ERROR("end_time < start time, error");
      continue;
    }
    delta_time = end_time - start_time;

    if (delta_time < period_time) {
      std::this_thread::sleep_for(
          std::chrono::nanoseconds(period_time - delta_time));
    } else {
      std::this_thread::yield();
    }
  }
}

}  // namespace world_model
}  // namespace neodrive
