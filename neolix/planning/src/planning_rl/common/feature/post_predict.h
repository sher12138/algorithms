#pragma once
#include <math.h>
#include <vector>
#include "common/opt_traj/include/poly_traj_structs.h"
#include "common/utils/data_reader.h"
#include "common/utils/obs_utils.h"
#include "config/planning_rl_config.h"
#include "feature_data_struct.h"
#include "optmize_trajectory.h"
#include "train_features.h"

namespace neodrive {
namespace planning_rl {

std::vector<PredState> PostPredict(
    const std::vector<std::vector<std::vector<double>>> &input,
    std::vector<double> &prediction, const EgoInfo &egoinfo);

std::vector<PredState> TransPredictsFrenet2Odom(
    std::vector<std::vector<double>> &prediction, const EgoInfo &egoinfo, const std::vector<ReferencePoint> &reference_line_odom);

std::vector<PredState> TransPredictsFrenet2OdomStepGap(
    std::vector<std::vector<double>> &prediction, const EgoInfo &egoinfo, const std::vector<ReferencePoint> &reference_line_odom);

std::vector<double> RescaleAction2Unnormalized(std::vector<double> &action);

std::vector<double> RescaleAction(std::vector<double> &action);

std::vector<double> RescaleActionStepGap(std::vector<double> &action);

std::vector<PredState> ConvertPredWorldCor(std::vector<PredState> &pred_states,
                                           const EgoInfo &egoinfo);

std::vector<TrajectoryPoint2D> OptimalTrajectoryRefpoint(
    const std::vector<PredState> &predict_traj,
    const std::vector<RefPoint> &reference_line_points);

std::vector<std::vector<double>> FormatPredicts(std::vector<std::vector<double>> &prediction);

}  // namespace planning_rl
}  // namespace neodrive