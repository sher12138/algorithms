#pragma once

#include <cmath>
#include <deque>
#include <unordered_map>
#include <utility>

#include "common/macros.h"
#include "src/planning/reference_line/reference_point.h"
#include "traffic_light_detection.pb.h"

using TrafficLightColor =
    neodrive::global::perception::traffic_light::TrafficLight_Color;

namespace neodrive {
namespace planning {

struct RouteSignal {
  /// from hdmap
  uint64_t id{0};
  double start_route_s{0.};
  double end_route_s{0.};
  TurnSignalType turn_type{TurnSignalType::TST_STRAIGHT};

  /// from perception traffic light
  double time_stamp{0.};
  TrafficLightColor signal_color =
      TrafficLightColor::TrafficLight_Color_UNKNOWN;
  bool enable_left_time{false};
  bool blink{false};
  double left_time{0.};
  double tracking_time{0.};
  double stop_line_s{-1.0};
  /// need stop status
  bool need_stop{false};
  bool active_protect{false};

  /// traffic light id
  std::string light_id{""};
};

using TrafficLightTable = std::multimap<uint64_t, RouteSignal>;

struct ActiveProtectTimeStamp {
  double start_time{-1.0};
  double curr_time{-1.0};
  void Reset() {
    start_time = -1.0;
    curr_time = -1.0;
  }
};

class TrafficLightLawContext {
 public:
  struct light_info {
    bool last_need_stop{false};
    bool yellow_red_stop{false};
    TrafficLightColor last_color;
    double yellow_blink_time{-1.0};
    double yellow_red_stop_timestamp{0.0};
  };
  using UnorderedMapStringAndDouble = std::unordered_map<uint64_t, double>;
  using UnorderedMapUintAndLightinfo = std::unordered_map<uint64_t, light_info>;
  TrafficLightLawContext() = default;
  ~TrafficLightLawContext() = default;

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(UnorderedMapStringAndDouble,
                                   map_id_and_2nd_stop_line);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(ActiveProtectTimeStamp,
                                   behind_active_protect_time_stamp);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(ActiveProtectTimeStamp,
                                   forward_active_protect_time_stamp);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(UnorderedMapUintAndLightinfo,
                                   traffic_light_infos);

  void set_map_value(uint64_t map_key, double map_value) {
    map_id_and_2nd_stop_line_[map_key] = map_value;
  }

  bool get_map_value(uint64_t map_key, double& map_value) {
    auto iter = map_id_and_2nd_stop_line_.find(map_key);
    if (iter != map_id_and_2nd_stop_line_.end()) {
      map_value = map_id_and_2nd_stop_line_[map_key];
      return true;
    }
    map_value = 0.0;
    return false;
  }

 private:
  UnorderedMapStringAndDouble map_id_and_2nd_stop_line_{};
  ActiveProtectTimeStamp behind_active_protect_time_stamp_{};
  ActiveProtectTimeStamp forward_active_protect_time_stamp_{};

  UnorderedMapUintAndLightinfo traffic_light_infos_{};
};

}  // namespace planning
}  // namespace neodrive
