#pragma once

#include <memory>

#include "common/data_center/proto_check_util.h"
#include "common/macros.h"
#include "common/planning_types.h"
#include "common/util/time_logger.h"
#include "cyber.h"
#include "proto/global_adc_status.pb.h"
#include "src/planning/common/planning_macros.h"
#include "util/thread_safe_object.h"

namespace neodrive {
namespace planning {
class DataCenter;

#define PLANNING_OBSERVE_FUNCTION(unit)                                \
  {                                                                    \
    unit.is_updated = false;                                           \
    if (unit.reader->latest_recv_time_sec() >                          \
        unit.latest_recv_time_sec + 1.0e-6) {                          \
      unit.latest_recv_time_sec = unit.reader->latest_recv_time_sec(); \
      unit.reader->Observe();                                          \
      auto msg_ptr = unit.reader->GetLatestObserved();                 \
      if (msg_ptr != nullptr) {                                        \
        if (CheckProto(*msg_ptr)) {                                    \
          unit.ptr = msg_ptr;                                          \
          unit.is_updated = true;                                      \
          unit.is_available = true;                                    \
        } else {                                                       \
          LOG_ERROR("CheckProto failed: {}", msg_ptr->DebugString());  \
        }                                                              \
      }                                                                \
    }                                                                  \
  }

class Observer {
  DECLARE_SINGLETON(Observer);
  friend class ObserverTest;
  friend class PlanningManagerTest;

 public:
  bool Init(const std::shared_ptr<Node> &node);
  void Observe();
  void ObservePerceptionObstacleMsg();

 protected:
  void ObserveAebMsg();

 private:
  static constexpr double kAebActiveDelayShowTime = 1.0;
  bool initialized_{false};
  std::mutex global_state_mutex_;
  std::shared_ptr<Node> node_;
  DataCenter *data_center_{nullptr};

  std::shared_ptr<AebCmd> prev_aeb_cmd_msg_{nullptr},
      prev_prev_aeb_cmd_msg_{nullptr};
  bool prev_is_auto_driving_{false};
};

}  // namespace planning
}  // namespace neodrive
