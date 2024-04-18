#pragma once
#include <jsoncpp/json/json.h>
#include "common/feature/train_features.h"
#include "common/utils/obs_utils.h"
#include "neolix_log.h"

namespace neodrive {
namespace planning_rl {

int DataJsonReader(Json::Value &root);

int UnitTestDataJsonReader(Json::Value &root);

int ModelConfReader(Json::Value &root);

int PostPredictConfReader(Json::Value &root);

FeaturePreparation JsonDataParser(Json::Value &root);

FeaturePreparation UnitJsonDataParser(Json::Value &root);

std::tuple<std::vector<double>, std::vector<double>> CalCurvatureFromOriRefline(
    const std::vector<std::vector<double>> &refline_points);

AgentFrame TransAgentFrame(const Json::Value &agent_data);

EgoFrame TransEgoFrame(const Json::Value &ego_data);

}  // namespace planning_rl
}  // namespace neodrive