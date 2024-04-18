#pragma once

#include <thread>

#include "aeb.h"
#include "common/aeb_type.h"
#include "common/util/time_util.h"
#include "collision_check/collision_check.h"
#include "cyber.h"
#include "cyber/common/memory_pool.h"

namespace neodrive {
namespace aeb {
class AebComponent final : public neodrive::cyber::Component<> {
 public:
  ~AebComponent();

  bool Init() override;

 private:
  bool Proc();
  void UpdateEnvironment();
  void PublishAebCmd();
  void ReportEvent();
  void ReportFcwWarning();
  void ReportCollision();
  std::shared_ptr<Writer<AebCmd>> aeb_cmd_pub_;
  std::shared_ptr<Writer<EventReport>> ivi_event_report_pub_;
  std::shared_ptr<Writer<EventOfInterest>> event_reporter_pub_;
  std::shared_ptr<std::thread> proc_thread_;
  std::shared_ptr<Aeb> aeb_ptr_;
  std::shared_ptr<CollisionCheck> collision_ptr_;
 private:
  void Observe();
  void CheckTimeout();

 private:
  static constexpr double kAebBrakePercent = 100.0;
  static constexpr int32_t kEventRecordTimeEpsilon = 60;
  static constexpr double kMinActiveInterval = 1.0;
  bool is_send_event = false;
  double pre_active_t_ = 0.0;
  neodrive::cyber::common::MemoryPool<AebCmd> aeb_cmd_msg_pool_;
  neodrive::cyber::common::MemoryPool<EventOfInterest> event_msg_pool_;
  neodrive::cyber::common::MemoryPool<EventReport> event_report_msg_pool_;
  double loop_start_time_ = 0.0; //unit: second, debug only, for time cost performance statistic.
};

CYBER_REGISTER_COMPONENT(AebComponent);

}  // namespace aeb
}  // namespace neodrive
