#include "planning_component.h"

#include "common_config/config/common_config.h"
#include "neolix_common/util/performance_test.h"
#include "reader_manager/reader_manager.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/planning_manager/planning_manager.h"

namespace neodrive {
namespace planning {

PERFORMANCE_TEST_DEFINE(planning_component);

using neodrive::cyber::Node;

PlanningComponent::~PlanningComponent() { proc_thread_->detach(); }

bool PlanningComponent::Init() {
  LOG_INFO("PlanningComponent init started.");
  if (!common::ReaderManager::Instance()->Init(node_)) {
    LOG_ERROR("ReaderManager init failed!");
    return false;
  }

  if (!PlanningManager::Instance()->Init(node_)) {
    LOG_ERROR("PlanningManager init failed!");
    return false;
  }

  proc_thread_ =
      std::make_unique<std::thread>(std::bind(&PlanningComponent::Proc, this));
  neodrive::cyber::scheduler::Instance()->SetInnerThreadAttr(
      "planning_proc_thread", proc_thread_.get());

  LOG_INFO("PlanningComponent init finished.");
  return true;
}

void PlanningComponent::Proc() {
  LOG_WARN("[thread_name]planning_component");
  uint64_t period_time = 1000 / std::max(FLAGS_planning_period_frequence, 1);

  uint64_t start_time = 0;
  uint64_t end_time = 0;
  uint64_t delta_time = 0;

  while (!neodrive::cyber::IsShutdown()) {
    start_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
    if (HaveToProcessNow(period_time, start_time)) {
      LOG_INFO("PlanningComponent Process started.");
      std::lock_guard<std::mutex> lock(mutex_);
      PERFORMANCE_TEST_START(planning_component, "proc");
      neodrive::common::config::CommonConfig::Instance()
          ->ReLoadConfigFromJson();
      config::PlanningConfig::Instance()->ReLoadConfigFromJson();
      PlanningManager::Instance()->RunOnce();
      PERFORMANCE_TEST_END(planning_component, "proc");
      LOG_INFO("PlanningComponent Process ended.");
    }

    end_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
    if (end_time < start_time) {
      LOG_ERROR("end_time < start time, error");
      continue;
    }
    delta_time = end_time - start_time;

    if (delta_time < period_time) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(period_time - delta_time));
    } else {
      std::this_thread::yield();
    }
  }
}

std::string PlanningComponent::Name() const { return std::string("planning"); }

bool PlanningComponent::HaveToProcessNow(const uint64_t period_time,
                                         const uint64_t curr_time) {
  observer_->ObservePerceptionObstacleMsg();
  if (data_center_->perception_obstacles_msg.is_updated ||
      curr_time - prev_start_time_ > 10 * period_time) {
    prev_start_time_ = curr_time;
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace neodrive
