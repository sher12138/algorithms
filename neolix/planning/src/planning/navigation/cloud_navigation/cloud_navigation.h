#pragma once
#include <curl/curl.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "a_star_navigation.h"
#include "breadth_first_navigation.h"
#include "civetweb.h"
#include "common/shm_manager.h"
#include "common_config/config/get_drive_strategy.h"
#include "config/navigation_config.h"
#include "cyber.h"
#include "navigation/common/navigation_types.h"
#include "navigation_base.h"
#include "nonmotorway_navigation.h"
#include "third_party/json/json.hpp"

using neodrive::planning::NavigationContext;

int websocket_connect_handler(const struct mg_connection *conn, void *userdata);
void websocket_ready_handler(struct mg_connection *conn, void *userdata);
int websocket_data_handler(struct mg_connection *conn, int flags, char *data,
                           size_t data_len, void *userdata);
void websocket_close_handler(const struct mg_connection *conn, void *userdata);
void *ws_server_thread(void *parm);
std::string request_handler(nlohmann::json &json_obj);

class RequestHandler {
 public:
  RequestHandler(std::string &version, std::string &url);
  ~RequestHandler() {
    if (buffer_) delete[] buffer_;
  }
  bool Init();
  bool Process(std::shared_ptr<neodrive::global::routing::RoutingRequest>
                   &routing_request,
               std::shared_ptr<neodrive::global::routing::RoutingResult>
                   &routing_response,
               nlohmann::json &response);
  void GetRoutingPath(
      const neodrive::global::routing::RoutingResult &routing_response,
      nlohmann::json &response);
  NavigationContext *GetContext() { return &ctx_; }
  bool GetPointFloor(double x, double y, double h_threshold, int &floor_num,
                     std::vector<double> &floor_heights);

 private:
  bool InitCyberverse(std::string map_folder);
  int IsMotorwayLane(uint64_t id);
  bool IsNonmotorwayMap();

 private:
  std::shared_ptr<neodrive::planning::NavigationProcessBase> navigation_ptr_{
      nullptr};
  std::shared_ptr<neodrive::global::routing::RoutingRequest> request_{nullptr};
  bool is_inited_{false};
  std::mutex mtx_;
  neodrive::common::config::DriveStrategyType drive_stratedy_type_;
  std::string map_folder_;
  neodrive::cyberverse::HDMapCloud *hdmap_{nullptr};
  NavigationContext ctx_;
  char *buffer_{nullptr};
};

class HandlerPool {
 public:
  static HandlerPool &Instacne() {
    static HandlerPool instance;
    return instance;
  }
  std::shared_ptr<RequestHandler> GetHandler(std::string &version,
                                             std::string &url) {
    std::lock_guard<std::mutex> lck(mtx_);
    auto iter = handler_pool_.find(version);
    if (iter == handler_pool_.end()) {
      handler_pool_[version] = std::make_shared<RequestHandler>(version, url);
    }
    return handler_pool_[version];
  }

 private:
  std::unordered_map<std::string, std::shared_ptr<RequestHandler>>
      handler_pool_;
  std::mutex mtx_;
};
