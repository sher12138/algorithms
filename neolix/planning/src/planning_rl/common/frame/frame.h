/**
 * @file frame.h
 * data struct define
 **/

#pragma once
#include <json/json.h>
#include <cmath>
#include <deque>
#include "agent.h"
#include "ego.h"
#include "neolix_log.h"
#include "utils/planning_rl_macros.h"

namespace neodrive {
namespace planning_rl {

class Frame {
 public:
  Frame() = default;
  ~Frame() = default;
  Frame(const Agents& agents, const Ego& ego);
  Frame(const Agents& agents, const Ego& ego, const double& timestamp_sec,
        const double& step_time);

  Json::Value to_json() const;
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(Agents, agents);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(Ego, ego);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, timestamp_sec);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, step_time);

 private:
  Agents agents_;
  Ego ego_;
  double timestamp_sec_;
  double step_time_;
};
// alias for interface using
using Frame2d = std::vector<Frame>;

}  // namespace planning_rl
}  // namespace neodrive
