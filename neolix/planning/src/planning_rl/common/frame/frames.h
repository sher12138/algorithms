/**
 * @file frames.h
 * data struct define
 **/

#pragma once
#include <json/json.h>
#include <deque>
#include "frame.h"
#include "neolix_log.h"
namespace neodrive {
namespace planning_rl {

class Frames {
 public:
  Frames() = default;
  ~Frames() = default;

  Json::Value to_json() const;
  void append_frame(const Frame& frame);
  void reset();
  std::deque<Frame>& frames();
  size_t size();

 protected:
  std::deque<Frame> frames_;
};

}  // namespace planning_rl
}  // namespace neodrive
