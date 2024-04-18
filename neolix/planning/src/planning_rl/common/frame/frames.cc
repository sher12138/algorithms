/**
 * @file frames.cpp
 **/

#include "frames.h"

namespace neodrive {
namespace planning_rl {

void Frames::append_frame(const Frame& frame) {
  // keep deque only has 5
  while (frames_.size() >= 5) {
    frames_.pop_front();
  }
  frames_.push_back(frame);
}
size_t Frames::size() { return frames_.size(); }

void Frames::reset() { frames_.clear(); }

std::deque<Frame>& Frames::frames() { return frames_; }

Json::Value Frames::to_json() const {
  Json::Value root;
  for (std::size_t i = 0; i < frames_.size(); ++i) {
    root.append(frames_[i].to_json());
  }
  return root;
}

}  // namespace planning_rl
}  // namespace neodrive
