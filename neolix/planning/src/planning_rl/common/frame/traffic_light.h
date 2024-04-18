#pragma once
#include <iostream>
#include <vector>
#include "common_geometry.pb.h"
#include "neolix_log.h"
#include "perception_obstacle.pb.h"
#include "traffic_light_detection.pb.h"
#include "utils/planning_rl_macros.h"

namespace neodrive {
namespace planning_rl {

using neodrive::global::perception::traffic_light::TrafficLight;
using neodrive::global::perception::traffic_light::TrafficLight_Color;
using neodrive::global::perception::traffic_light::TrafficLightDetection;
using TrafficLightDetectionShrPtr = std::shared_ptr<TrafficLightDetection>;
using TrafficLightShrPtr = std::shared_ptr<TrafficLight>;

// enum TLcolor {
//   TLUNKNOWN = 0,
//   RED = 1,
//   YELLOW = 2,
//   GREEN = 3,
//   BLACK = 4
// };

// enum TLType {
//   STRAIGHT = 0,
//   LEFT = 1,
//   RIGHT = 2,
//   UTURN = 3,
//   UNKNOWN = 4
// };

// enum TrafficLight_Color : int {
//   TrafficLight_Color_UNKNOWN = 0,
//   TrafficLight_Color_RED = 1,
//   TrafficLight_Color_YELLOW = 2,
//   TrafficLight_Color_GREEN = 3,
//   TrafficLight_Color_BLACK = 4
// };

class TraLight {
  public:
    TraLight() = default;
    TraLight(int tl_color, int tl_type, std::string tl_id);
    ~TraLight() = default;
    //TraLight(const )
    // DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(int, id);
    DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::string, id);
    DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(int, color);
    DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(int, type);
    DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, confidence);
    DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, blink);
    DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, remaining_time);
  
  private:
    int color_ = 3;
    int type_ = 4;
    std::string id_ = "";
    double confidence_ = 1.0;
    bool blink_ = false;
    double remaining_time_ = -999.0;

};


class TrafficLights {
  public:
    TrafficLights() = default;
    TrafficLights(TrafficLightDetectionShrPtr traffic_light_dete);
    ~TrafficLights() = default;

    DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<TraLight>, traffic_lights);
  private:
    std::vector<TraLight> traffic_lights_;
};

using TrafficLightsShrPtr = std::shared_ptr<TrafficLights>;

}  // namespace planning_rl
}  // namespace neodrive