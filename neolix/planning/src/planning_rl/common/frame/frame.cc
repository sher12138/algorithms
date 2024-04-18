/**
 * @file frame.cpp
 **/

#include "frame.h"

namespace neodrive {
namespace planning_rl {

Frame::Frame(const Agents& agents, const Ego& ego) {
  agents_ = agents;
  ego_ = ego;
}
Frame::Frame(const Agents& agents, const Ego& ego, const double& timestamp_sec,
             const double& step_time) {
  agents_ = agents;
  ego_ = ego;
  timestamp_sec_ = timestamp_sec;
  step_time_ = step_time;
}

Json::Value Frame::to_json() const {
  Json::Value root;
  root["timestamp_sec"] = timestamp_sec_;
  root["step_time"] = step_time_;
  root["ego"] = ego_.to_json();
  root["agents"] = agents_.to_json();
  return root;
}

}  // namespace planning_rl
}  // namespace neodrive
