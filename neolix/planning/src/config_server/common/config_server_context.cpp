#include "config_server_context.h"

namespace neodrive {
namespace config_server {
ConfigServerContext::ConfigServerContext() {}

void ConfigServerContext::WriteMessage() {
  nlohmann::json response = nlohmann::json::parse(data);
  if (response["errmsg"] == "success") {
    LOG_INFO(data);
    auto msg = traffic_msg_pool_.GetSharedPtr();
    CHECK_NOTNULL(msg);
    auto header = msg->mutable_header();
    header->set_module_name("cloud_traffic_light");
    header->set_timestamp_sec(cyber::Time::Now().ToSecond());
    header->set_sequence_num(seq++);
    auto results = response["data"];
    msg->clear_traffic_light();
    msg->mutable_traffic_light()->Reserve(results["res"].size());
    for (auto &each_res : results["res"]) {
      auto traffic_light = msg->add_traffic_light();
      auto status = each_res["status"];
      if (status == "YELLOW") {
        traffic_light->set_color(TrafficLight::YELLOW);
      } else if (status == "RED") {
        traffic_light->set_color(TrafficLight::RED);
      } else if (status == "GREEN") {
        traffic_light->set_color(TrafficLight::GREEN);
      } else {
        traffic_light->set_color(TrafficLight::UNKNOWN);
      }
      double remaing_time = each_res["countdown"];
      traffic_light->set_remaining_time(remaing_time / 1000.0);
      traffic_light->set_id(each_res["signalId"]);
    }
    cloud_traffic_light_pub->Write(msg);
  }
}

void ConfigServerContext::SendEmptyMessage() {
  auto msg = empty_traffic_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(msg);
  auto header = msg->mutable_header();
  header->set_module_name("cloud_traffic_light");
  header->set_timestamp_sec(cyber::Time::Now().ToSecond());
  header->set_sequence_num(seq++);
  cloud_traffic_light_pub->Write(msg);
  LOG_WARN("send empty message");
}
}  // namespace config_server
}  // namespace neodrive