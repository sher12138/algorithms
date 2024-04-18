#pragma once
#include <thread>

#include "common/data_center/data_center.h"
#include "cyber.h"
#include "src/planning/observer/observer.h"

namespace neodrive {
namespace planning {

class PlanningComponent final : public neodrive::cyber::Component<> {
 public:
  PlanningComponent() = default;
  ~PlanningComponent();

  bool Init() override;

  std::string Name() const;

 private:
  void Proc();
  bool HaveToProcessNow(const uint64_t period_time, const uint64_t curr_time);

 private:
  std::mutex mutex_;
  std::unique_ptr<std::thread> proc_thread_;
  Observer *observer_{Observer::Instance()};
  DataCenter *data_center_{DataCenter::Instance()};
  uint64_t prev_start_time_{0};
};

CYBER_REGISTER_COMPONENT(PlanningComponent);

}  // namespace planning
}  // namespace neodrive
