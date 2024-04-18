#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace neodrive {
namespace planning {
namespace sim_planner {

class DcpTree {
 public:
  enum class DcpLonAction {
    kMaintain = 0,
    kAccelerate,
    kDecelerate,
    MAX_COUNT = 3
  };
  enum class DcpLatAction {
    kLaneKeeping = 0,
    kLaneChangeLeft,
    kLaneChangeRight,
    MAX_COUNT = 3
  };
  struct DcpAction {
    DcpLonAction lon = DcpLonAction::kMaintain;
    DcpLatAction lat = DcpLatAction::kLaneKeeping;

    double t = 0.0;

    friend std::ostream& operator<<(std::ostream& os, const DcpAction& action) {
      os << "(lon: " << static_cast<int>(action.lon)
         << ", lat: " << static_cast<int>(action.lat) << ", t: " << action.t
         << ")";
      return os;
    }
    DcpAction() {}
    DcpAction(const DcpLonAction& lon_, const DcpLatAction& lat_,
              const double& t_)
        : lon(lon_), lat(lat_), t(t_) {}
  };
  DcpTree(const int& tree_height, const double& layer_time);
  DcpTree(const int& tree_height, const double& layer_time,
          const double& last_layer_time);
  ~DcpTree() = default;

  void set_ongoing_action(const DcpAction& a) { ongoing_action_ = a; }

  std::vector<std::vector<DcpAction>> action_script() const {
    return action_script_;
  }

  double planning_horizon() const;

  int tree_height() const { return tree_height_; }

  double sim_time_per_layer() const { return layer_time_; }

  bool updateScript();

  static std::string retLonActionName(const DcpLonAction a) {
    std::string a_str;
    switch (a) {
      case DcpLonAction::kMaintain: {
        a_str = std::string("M");
        break;
      }
      case DcpLonAction::kAccelerate: {
        a_str = std::string("A");
        break;
      }
      case DcpLonAction::kDecelerate: {
        a_str = std::string("D");
        break;
      }
      default: {
        a_str = std::string("Null");
        break;
      }
    }
    return a_str;
  }

  static std::string retLatActionName(const DcpLatAction a) {
    std::string a_str;
    switch (a) {
      case DcpLatAction::kLaneKeeping: {
        a_str = std::string("K");
        break;
      }
      case DcpLatAction::kLaneChangeLeft: {
        a_str = std::string("L");
        break;
      }
      case DcpLatAction::kLaneChangeRight: {
        a_str = std::string("R");
        break;
      }
      default: {
        a_str = std::string("Null");
        break;
      }
    }
    return a_str;
  }

 private:
  bool generateActionScript();

  std::vector<DcpAction> appendActionSequence(
      const std::vector<DcpAction>& seq_in, const DcpAction& a,
      const int& n) const;

  int tree_height_ = 5;
  double layer_time_ = 1.0;
  double last_layer_time_ = 1.0;
  DcpAction ongoing_action_;
  std::vector<std::vector<DcpAction>> action_script_;
};

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive
