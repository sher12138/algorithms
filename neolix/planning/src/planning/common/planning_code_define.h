#pragma once
#include <string>

namespace {
// Reference line name
const std::string& RL_CRUISE = "RL_CRUISE";
const std::string& RL_REVERSE = "RL_REVERSE";
const std::string& RL_INLANE_UTURN = "RL_INLANE_UTURN";
const std::string& RL_OUTLANE_UTURN = "RL_OUTLANE_UTURN";
const std::string& RL_FREE_SPACE = "RL_FREE_SPACE";
const std::string& RL_PARKING_IN_SLOT = "RL_PARKING_IN_SLOT";
const std::string& RL_PARKING_OUT_SLOT = "RL_PARKING_OUT_SLOT";
const std::string& RL_LANE_BORROW = "RL_LANE_BORROW";
const std::string& RL_POSE_ADJUST = "RL_POSE_ADJUST";
const std::string& RL_CONTROL_LAT_TUNING = "RL_CONTROL_LAT_TUNING";

// error code, need to remove
const std::string& GET_REFERENCE_LINE_FAILED = "034372";

const std::string& DP_OPTIMIZE_FAILED = "034383";
}  // namespace

namespace neodrive {
namespace planning {

enum class ErrorCode {
  PLANNING_SKIP = 1,  // skip current task
  PLANNING_OK = 0,
  PLANNING_ERROR_NOT_FOUND = -1,
  PLANNING_ERROR_OUT_OF_INDEX = -2,
  PLANNING_ERROR_SELF_LOOP = -3,
  PLANNING_ERROR_DUPLICATE = -4,
  PLANNING_ERROR_NULL_POINTER = -5,
  PLANNING_ERROR_NAN = -6,
  PLANNING_ERROR_TIMEOUT = -7,
  PLANNING_ERROR_FAILED = -8,
  PLANNING_ERROR_COLLISION_DETECTED = -9,
  PLANNING_SKIP_REST_TASKS = -10
};

enum MultiLevelMode {
  MLM_HIGHSPEED = 0,
  MLM_NORMAL = 1,
  MLM_PASSINGBY = 2,
  MLM_PASSINGTHROUGH = 3,
  MLM_STRONGPASSING = 4,
};

enum PassByScenario {
  PBC_NORMAL = 0,
  PBC_BACK_OBS_OVERTAKE = 1,
  PBC_SIDE_OBS_OVERTAKE = 2,
  PBC_LEFT_FRONT_OBS_REVERSE = 3,
  PBC_RIGHT_FRONT_OBS_REVERSE = 4,
};

}  // namespace planning
}  // namespace neodrive
