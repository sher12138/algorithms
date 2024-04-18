#include "config_server_component.h"

#include "civetweb.h"
#include "common/shm_manager.h"
#include "neolix_common/reader_manager/reader_manager.h"
#include "src/common/util/hash_util.h"

namespace neodrive {
namespace config_server {
namespace {
struct mg_connection *client_conn_;
char err_buf[1024];
int ReceiveDataCallback(struct mg_connection *conn, int flags, char *data,
                        size_t data_len, void *user_data) {
  bool is_text = ((flags & 0xf) == MG_WEBSOCKET_OPCODE_TEXT);
  bool is_bin = ((flags & 0xf) == MG_WEBSOCKET_OPCODE_BINARY);
  bool is_ping = ((flags & 0xf) == MG_WEBSOCKET_OPCODE_PING);
  bool is_pong = ((flags & 0xf) == MG_WEBSOCKET_OPCODE_PONG);
  bool is_close = ((flags & 0xf) == MG_WEBSOCKET_OPCODE_CONNECTION_CLOSE);
  if (is_ping) {
    mg_websocket_client_write(conn, MG_WEBSOCKET_OPCODE_PONG, data, data_len);
  }
  if (is_pong) {
    LOG_INFO("got a pong");
    ConfigServerContext::Instance()->is_ping_miss = false;
  }
  if (is_text) {
    ConfigServerContext::Instance()->is_param_updated = true;
    std::lock_guard<std::mutex> lck(ConfigServerContext::Instance()->mtx);
    ConfigServerContext::Instance()->data = std::string(data, data_len);
    ConfigServerContext::Instance()->WriteMessage();
    ConfigServerContext::Instance()->is_ping_miss = false;
    ConfigServerContext::Instance()->pre_recv_data_t =
        cyber::Time::Now().ToSecond();
    LOG_INFO("receive data:{}", data);
  }
  return 1;
}
}  // namespace
ConfigServerComponent::~ConfigServerComponent() {}

void ConfigServerComponent::InitSharedMemory() {
  if (hdmap_->IsInited()) return;
  uint32_t total_size = 1000 * 1024 * 1024 * 1;
  char *buffer =
      (char *)cyberverse::ShmManager::Instance()->OpenOrCreate(total_size);
  uint32_t read_size = hdmap_->LoadMapInfosFromMemory(buffer);
  LOG_INFO("load map from shared memory success:used:{} ,max:{}", read_size,
           total_size);
}

bool ConfigServerComponent::Init() {
  hdmap_ = cyberverse::HDMap::Instance();
  auto reader_manager = common::ReaderManager::Instance();
  if (!reader_manager->Init(node_)) {
    LOG_ERROR("ReaderManager init failed!");
    return false;
  }
  // coroutine
  auto config_server_config =
      config::ConfigServerConfig::Instance()->config_server_config();
  ConfigServerContext::Instance()->pnc_traffic_light_req.reader =
      reader_manager->CreateReader<TrafficLightDetection>("/pnc/traffic_light");
  ConfigServerContext::Instance()->chassis_msg.reader =
      reader_manager->CreateReader<Chassis>("/planning/proxy/DuDriveChassis");
  ConfigServerContext::Instance()->cyberverse_trigger.reader =
      reader_manager->CreateReader<CyberverseTrigger>("/cyberverse/trigger");
  ConfigServerContext::Instance()->cloud_traffic_light_pub =
      node_->CreateWriter<TrafficLightDetection>("/cloud/traffic_light");
  mg_init_library(config_server_config.connection_security_level);
  proc_thread_ = std::make_unique<std::thread>(
      std::bind(&ConfigServerComponent::Process, this));
  neodrive::cyber::scheduler::Instance()->SetInnerThreadAttr(
      "config_server_proc_thread", proc_thread_.get());
  LOG_INFO("ConfigServerComponent init finished.");
  return true;
}

bool ConfigServerComponent::Process() {
  LOG_WARN("[thread_name]config_server_component");
  auto frequency = config::ConfigServerConfig::Instance()
                       ->config_server_config()
                       .config_server_period_frequency;
  int connection_loss_threshold = config::ConfigServerConfig::Instance()
                                      ->config_server_config()
                                      .connection_loss_threshold;
  uint64_t period_time = 1000 / std::max(frequency, 1);
  uint64_t start_time{0}, end_time{0}, delta_time{0}, heart_beat_count{0};

  while (!neodrive::cyber::IsShutdown()) {
    start_time = cyber::Time::Now().ToMillsecond();
    if (ConfigServerContext::Instance()->is_ping_miss)
      ping_miss_count_++;
    else
      ping_miss_count_ = 0;
    if (ping_miss_count_ > connection_loss_threshold) {
      ReconnectToServer();
      ping_miss_count_ = 0;
    }
    OBSERVE_FUNCTION(ConfigServerContext::Instance()->pnc_traffic_light_req);
    OBSERVE_FUNCTION(ConfigServerContext::Instance()->chassis_msg);
    OBSERVE_FUNCTION(ConfigServerContext::Instance()->cyberverse_trigger);
    ProcessRequest();
    if (++heart_beat_count >= 10) {
      double gap_t = cyber::Time::Now().ToSecond() -
                     ConfigServerContext::Instance()->pre_recv_data_t;
      double threshold = config::ConfigServerConfig::Instance()
                             ->config_server_config()
                             .send_empty_data_time_threshold;
      if (is_first_legal_request_come_)
        (gap_t > threshold) ? ResetConnection() : SendHeartBeatPing();
      heart_beat_count = 0;
    }
    end_time = cyber::Time::Now().ToMillsecond();
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
  return true;
}

void ConfigServerComponent::ProcessRequest() {
  if (ConfigServerContext::Instance()->cyberverse_trigger.is_updated)
    InitSharedMemory();
  if (!hdmap_->IsInited()) return;
  auto &traffic_light_req =
      ConfigServerContext::Instance()->pnc_traffic_light_req;
  auto chassis = ConfigServerContext::Instance()->chassis_msg.ptr;
  if (!traffic_light_req.is_updated) return;
  nlohmann::json cloud_traffic_light_req;
  cloud_traffic_light_req["vin"] = chassis->vin();
  cloud_traffic_light_req["type"] = "realtime_signal";
  cloud_traffic_light_req["command"] = "pull";
  for (auto &each_traffic : traffic_light_req.ptr->traffic_light()) {
    auto signal = hdmap_->GetSignalById(common::HashString(each_traffic.id()));
    if (signal == nullptr) {
      LOG_ERROR("got no signal by id:{}", each_traffic.id());
    }
    if (signal->NodeId() > 999) {
      LOG_ERROR("signal_id :{},node_id:{}", each_traffic.id(),
                signal->NodeId());
      continue;
    }
    nlohmann::json data_unit;
    data_unit["signal_id"] = each_traffic.id();
    data_unit["node_id"] = signal->NodeId();
    auto &phase_ids = signal->PhaseIds();
    for (uint32_t i = 0; i < phase_ids.size(); ++i) {
      data_unit["phase_ids"].emplace_back(phase_ids[i]);
    }
    cloud_traffic_light_req["data"].emplace_back(data_unit);
  }
  if (cloud_traffic_light_req["data"].size() < 1) return;
  if (!is_first_legal_request_come_) {
    ConnectToServer();
    ConfigServerContext::Instance()->pre_recv_data_t =
        cyber::Time::Now().ToSecond();
    is_first_legal_request_come_ = true;
  }
  cloud_traffic_light_req["seq"] = seq_++;
  std::string send_str = cloud_traffic_light_req.dump();
  mg_websocket_client_write(client_conn_, MG_WEBSOCKET_OPCODE_TEXT,
                            send_str.c_str(), send_str.size());
  LOG_INFO("msg send:{}", send_str);
}

void ConfigServerComponent::SendHeartBeatPing() {
  LOG_INFO("send heart beat ping id:{}", heart_beat_count_);
  mg_websocket_client_write(client_conn_, MG_WEBSOCKET_OPCODE_PING,
                            (const char *)&heart_beat_count_, sizeof(int));
  ConfigServerContext::Instance()->is_ping_miss = true;
  heart_beat_count_++;
}

void ConfigServerComponent::ResetConnection() {
  ConfigServerContext::Instance()->SendEmptyMessage();
  is_first_legal_request_come_ = false;
  mg_close_connection(client_conn_);
}

void ConfigServerComponent::ConnectToServer() {
  auto config_server_config =
      config::ConfigServerConfig::Instance()->config_server_config();
  client_conn_ = mg_connect_websocket_client(
      config_server_config.server_url.c_str(), config_server_config.server_port,
      config_server_config.connection_security_level, err_buf, sizeof(err_buf),
      config_server_config.server_path.c_str(), NULL, ReceiveDataCallback,
      nullptr, nullptr);
}

void ConfigServerComponent::ReconnectToServer() {
  auto config_server_config =
      config::ConfigServerConfig::Instance()->config_server_config();
  mg_close_connection(client_conn_);
  LOG_INFO("Reconnect to server.");
  usleep(10 * 1e3);
  client_conn_ = mg_connect_websocket_client(
      config_server_config.server_url.c_str(), config_server_config.server_port,
      config_server_config.connection_security_level, err_buf, sizeof(err_buf),
      config_server_config.server_path.c_str(), NULL, ReceiveDataCallback,
      nullptr, nullptr);
  if (client_conn_ == NULL) {
    LOG_INFO("Reconnect failed.");
  } else {
    LOG_INFO("Reconnect succeed.");
  };
}
}  // namespace config_server
}  // namespace neodrive
