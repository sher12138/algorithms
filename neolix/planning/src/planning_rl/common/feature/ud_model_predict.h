#pragma once
#include <stdio.h>

#include <iostream>
#include <string>
#include <vector>

#include "common/feature/train_features_v2.h"
#include "common/utils/cout_sequence.h"
#include "common/utils/obs_utils.h"
#include "common/utils/time_logger.h"
#include "common/model_trt/urban_driver_net/urban_driver_evaluator.h"

namespace neodrive {
namespace planning_rl {

std::vector<std::vector<double>> ud_inference(const FeaturePreparationV2 &fp);

}  // namespace planning_rl
}  // namespace neodrive