#pragma once

#include <mutex>
#include <string>

#include "config_server_type.h"
#include "cyber/common/macros.h"
#include "cyber/common/memory_pool.h"
#include "cyber/common/message_util.h"

namespace neodrive {
namespace config_server {
class ConfigServerContext {
  DECLARE_SINGLETON(ConfigServerContext);

 public:
  std::string data;
  common::util::MessageUnit<TrafficLightDetection> pnc_traffic_light_req;
  common::util::MessageUnit<Chassis> chassis_msg;
  common::util::MessageUnit<CyberverseTrigger> cyberverse_trigger;
  uint64_t seq{1};
  std::shared_ptr<Writer<TrafficLightDetection>> cloud_traffic_light_pub;
  std::atomic<bool> is_param_updated{false};
  std::mutex mtx;
  std::atomic<bool> is_ping_miss{false};
  double pre_recv_data_t{0.0};

 public:
  void WriteMessage();
  void SendEmptyMessage();

 private:
  neodrive::cyber::common::MemoryPool<TrafficLightDetection> traffic_msg_pool_;
  neodrive::cyber::common::MemoryPool<TrafficLightDetection>
      empty_traffic_msg_pool_;
};

}  // namespace config_server
}  // namespace neodrive