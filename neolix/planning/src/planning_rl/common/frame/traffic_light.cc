#include "traffic_light.h"

namespace neodrive {
namespace planning_rl {

TraLight::TraLight(int tl_color, int tl_type, std::string tl_id) {
    color_ = tl_color;
    type_ = tl_type;
    id_ = tl_id;
}

TrafficLights::TrafficLights(TrafficLightDetectionShrPtr traffic_light_dete) {
    auto traffic_light = traffic_light_dete->traffic_light();
    traffic_lights_.clear();
    int index = 0;
    for (auto &tl: traffic_light) {
        int color = tl.color();
        if (color == TrafficLight_Color::TrafficLight_Color_RED) {
            std::cout << "TrafficLight_Color_RED" << std::endl;
        }
        if (color == TrafficLight_Color::TrafficLight_Color_GREEN) {
            std::cout << "TrafficLight_Color_GREEN" << std::endl;
        }
        int type = tl.type();
        std::string id = tl.id();
        if (color > 0) {
            auto tls = TraLight(color, type, id);
            traffic_lights_.emplace_back(tls);
        } else {
            auto tls = TraLight(3, 4, id);
            traffic_lights_.emplace_back(tls);
        }
        
        // std::cout << index << std::endl;
        // index++;
        // std::cout << tl.has_color() << std::endl;
        // std::cout << tl.has_id() << std::endl;
        // std::cout << tl.id() << std::endl;
        // std::cout << tl.color() << std::endl;
        // std::cout << tl.type() << std::endl;
        // std::cout << tl.blink() << std::endl;
        // std::cout << tl.confidence() << std::endl;
        // std::cout << color << std::endl;
        // std::cout << type << std::endl;
    }
    // auto color = traffic_light->color();
    // auto id = traffic_light->id();
    // std::cout << "trals" << std::endl;
    // std::cout << color << std::endl;
    // std::cout << id << std::endl;
}

}  // namespace planning_rl
}  // namespace neodrive