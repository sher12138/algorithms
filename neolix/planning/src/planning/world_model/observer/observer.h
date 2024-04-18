#pragma once

#include <memory>

#include "common/util/time_logger.h"
#include "cyber.h"
#include "util/thread_safe_object.h"
#include "world_model/common/world_model_context.h"
#include "world_model/coordinator_transform/coordinator_transform.h"
#include "world_model/observer/proto_check_util.h"

namespace neodrive {
namespace world_model {
#define WORLD_MODEL_OBSERVE_FUNCTION(unit)                             \
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

 public:
  bool Init(const std::shared_ptr<neodrive::cyber::Node> &node);
  void Observe();

 private:
  bool initialized_{false};
  std::shared_ptr<neodrive::cyber::Node> node_;
  WorldModelContxt *wm_context_{WorldModelContxt::Instance()};
  CoordinateTransform *coordinate_tranform_{CoordinateTransform::Instance()};
};

}  // namespace world_model
}  // namespace neodrive
