#pragma once
#include <jsoncpp/json/json.h>
#include <stdio.h>

#include <iostream>
#include <string>
#include <vector>

#include "common/feature/train_features.h"
#include "common/opt_traj/include/poly_traj_structs.h"
#include "common/utils/cout_sequence.h"
#include "common/utils/data_reader.h"
#include "common/utils/data_struct_convert.h"
#include "common/utils/obs_utils.h"
#include "config/planning_rl_config.h"
#include "feature_data_struct.h"
#include "neolix_log.h"
#include "optmize_trajectory.h"
#include "post_predict.h"
// #include "model/torch_load_model.h"
#include "common/model_trt/imitation_net/imitation_evaluator.h"

namespace neodrive {
namespace planning_rl {

void Infer();

void UnitInfer();

std::vector<double> ModelPredictTrt(
    const std::vector<std::vector<std::vector<double>>> &target_before);

int WriteTrajJson(const std::vector<TrajectoryPoint2D> &pred_traj_opt);

}  // namespace planning_rl
}  // namespace neodrive
