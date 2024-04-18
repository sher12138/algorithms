/**
 * @brief this file is auto-generated by scripts/json_to_cpp.py. Do not edit!!!
 */

#include "config/auto_post_predict.h"

namespace neodrive {
namespace planning_rl {
namespace config {

void InitAutoPostPredict(const Json::Value &input_json, AutoPostPredict &dest) {
  dest.rescale_action_paras.max_acc =
      input_json["rescale_action_paras"]["MAX_ACC"].asDouble();
  dest.rescale_action_paras.min_brake =
      input_json["rescale_action_paras"]["MIN_BRAKE"].asDouble();
  dest.rescale_action_paras.max_deta_x =
      input_json["rescale_action_paras"]["MAX_deta_X"].asDouble();
  dest.rescale_action_paras.max_deta_y =
      input_json["rescale_action_paras"]["MAX_deta_Y"].asDouble();
  dest.rescale_action_paras.min_deta_x =
      input_json["rescale_action_paras"]["MIN_deta_X"].asDouble();
  dest.rescale_action_paras.min_deta_y =
      input_json["rescale_action_paras"]["MIN_deta_Y"].asDouble();
  dest.rescale_action_paras.max_theta =
      input_json["rescale_action_paras"]["MAX_THETA"].asDouble();
  dest.rescale_action_paras.min_theta =
      input_json["rescale_action_paras"]["MIN_THETA"].asDouble();
  dest.rescale_action_paras.max_time =
      input_json["rescale_action_paras"]["MAX_TIME"].asDouble();
  dest.rescale_action_paras.max_speed =
      input_json["rescale_action_paras"]["MAX_SPEED"].asDouble();
  dest.rescale_action_paras.max_curvature =
      input_json["rescale_action_paras"]["MAX_CURVATURE"].asDouble();
}

}  // namespace config
}  // namespace planning_rl
}  // namespace neodrive
