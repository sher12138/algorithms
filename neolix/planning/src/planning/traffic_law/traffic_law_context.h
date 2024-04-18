#pragma once

#include "crosswalk_law_context.h"
#include "stop_sign_law_context.h"
#include "traffic_light_law_context.h"

namespace neodrive {
namespace planning {

class TrafficLawContext {
 public:
  const CrosswalkLawContext& Crosswalklawcontext() const {
    return crosswalk_law_context_;
  }
  CrosswalkLawContext* MutableCrosswalklawcontext() {
    return &crosswalk_law_context_;
  }
  const StopsignLawContext& Stopsignlawcontext() const {
    return stop_sign_law_context_;
  }
  class StopsignLawContext* MutableStopsignlawcontext() {
    return &stop_sign_law_context_;
  }
  const TrafficLightLawContext& TrafficLightlawcontext() const {
    return traffic_light_law_context_;
  }
  TrafficLightLawContext* MutableTrafficLightlawcontext() {
    return &traffic_light_law_context_;
  }

 private:
  CrosswalkLawContext crosswalk_law_context_;
  StopsignLawContext stop_sign_law_context_;
  TrafficLightLawContext traffic_light_law_context_;
};

}  // namespace planning
}  // namespace neodrive
