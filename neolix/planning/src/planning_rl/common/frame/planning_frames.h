/**
 * @file frames.h
 * data struct define
 **/

#pragma once
#include <json/json.h>
#include <deque>
#include "frames.h"
#include "neolix_log.h"
namespace neodrive {
namespace planning_rl {

class PlanningFrames : public Frames {
 public:
  PlanningFrames() = default;
  ~PlanningFrames() = default;
  PlanningFrames(Frames& frames);

 private:
  uint32_t sequence_num_{0};
};

}  // namespace planning_rl
}  // namespace neodrive
