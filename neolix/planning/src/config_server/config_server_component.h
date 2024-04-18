#pragma once

#include <thread>

#include "common/config_server_context.h"
#include "common/config_server_type.h"
#include "config/config_server_config.h"
#include "hdmap/hdmap.h"

namespace neodrive {
namespace config_server {
class ConfigServerComponent final : public neodrive::cyber::Component<> {
 public:
  ~ConfigServerComponent();

  bool Init() override;

 private:
  bool Process();
  void InitSharedMemory();
  void ProcessRequest();
  void SendHeartBeatPing();
  void ResetConnection();
  void ConnectToServer();
  void ReconnectToServer();
  std::shared_ptr<std::thread> proc_thread_;
  cyberverse::HDMap* hdmap_;
  int heart_beat_count_{0};
  uint64_t seq_{1};
  uint64_t ping_miss_count_;
  bool is_first_legal_request_come_{false};
};

CYBER_REGISTER_COMPONENT(ConfigServerComponent);

}  // namespace config_server
}  // namespace neodrive
