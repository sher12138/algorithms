/**
 * @file planning_frames.cpp
 **/

#include "planning_frames.h"

namespace neodrive {
namespace planning_rl {
PlanningFrames::PlanningFrames(Frames& frames) { frames_ = frames.frames(); }

}  // namespace planning_rl
}  // namespace neodrive
