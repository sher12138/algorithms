#include "dcp_tree.h"

namespace neodrive {
namespace planning {
namespace sim_planner {

DcpTree::DcpTree(const int& tree_height, const double& layer_time)
    : tree_height_(tree_height), layer_time_(layer_time) {
  last_layer_time_ = layer_time_;
  generateActionScript();
}

DcpTree::DcpTree(const int& tree_height, const double& layer_time,
                 const double& last_layer_time)
    : tree_height_(tree_height),
      layer_time_(layer_time),
      last_layer_time_(last_layer_time) {
  generateActionScript();
}

bool DcpTree::updateScript() { return generateActionScript(); }

std::vector<DcpTree::DcpAction> DcpTree::appendActionSequence(
    const std::vector<DcpAction>& seq_in, const DcpAction& a,
    const int& n) const {
  std::vector<DcpAction> seq = seq_in;
  for (int i = 0; i < n; ++i) {
    seq.emplace_back(a);
  }
  return seq;
}

bool DcpTree::generateActionScript() {
  action_script_.clear();
  std::vector<DcpAction> ongoing_action_seq;
  // for (int lon = 0; lon < static_cast<int>(DcpLonAction::MAX_COUNT); lon++) {
  for (int lon = 1; lon < 2; lon++) {
    ongoing_action_seq.clear();
    ongoing_action_seq.emplace_back(
        DcpAction(DcpLonAction(lon), ongoing_action_.lat, ongoing_action_.t));

    for (int h = 1; h < tree_height_; ++h) {
      for (int lat = 0; lat < static_cast<int>(DcpLatAction::MAX_COUNT);
           lat++) {
        if (lat != static_cast<int>(ongoing_action_.lat)) {
          auto actions = appendActionSequence(
              ongoing_action_seq,
              DcpAction(DcpLonAction(lon), DcpLatAction(lat), layer_time_),
              tree_height_ - h);
          action_script_.emplace_back(std::move(actions));
        }
      }
      ongoing_action_seq.emplace_back(
          DcpAction(DcpLonAction(lon), ongoing_action_.lat, layer_time_));
    }
    action_script_.emplace_back(ongoing_action_seq);
  }
  // override the last layer time
  for (auto& action_seq : action_script_) {
    action_seq.back().t = last_layer_time_;
  }

  return true;
}

double DcpTree::planning_horizon() const {
  if (action_script_.empty()) return 0.0;
  double planning_horizon = 0.0;
  for (const auto& a : action_script_[0]) {
    planning_horizon += a.t;
  }
  return planning_horizon;
}

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive
