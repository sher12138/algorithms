#pragma once

#include <array>
#include <cmath>
#include <vector>

#include "src/planning/common/visualizer_event/visualizer_event.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/math/common/occupy_map.h"
namespace neodrive {
namespace planning {

class HybridAStar {
 public:
  struct Config {
    double veh_max_steer{35. / 180. * M_PI};
    double veh_wheel_base{1.73};
    double veh_half_width{0.54};

    double grid_step{0.1};
    int node_steer_segs{6};

    double node_arc_len{1};
    int node_arc_segs{5};
    int node_one_shot_gap{5};
    double tar_s_bias{1.5};  //define and modified in plan_config.json
  };
  using AD2 = std::array<double, 2>;
  using AD3 = std::array<double, 3>;
  using AD4 = std::array<double, 4>;
  using AI2 = std::array<int, 2>;
  struct Node {
    int idx{-1};

    std::vector<double> xs{0};
    std::vector<double> ys{0};
    std::vector<double> thetas{0};

    bool is_forward{true};
    double steer{0};

    double go_cost{0};
    double heu_cost{0};

    int parent{-1};
  };

  struct PiecePath {
    bool is_forward{true};
    std::vector<double> xs{};
    std::vector<double> ys{};
    std::vector<double> thetas{};
    std::vector<double> lens{};
  };

 public:
  /// Must construct with config
  /// Usage: HybridAStar has{{.grid_x_step = 0.2, .grid_y_step = 0.2}};
  HybridAStar() = delete;
  explicit HybridAStar(Config&&);

  /// Generate pieces of free-collision path in freespace
  /// @param om Occupy map
  /// @param from Start pose(x, y, theta) of the search
  /// @param to End pose(x, y, theta) of the search
  /// @param task_info task information
  /// @return A list of path, each path has multiple pose(x, y, theta, s)
  std::vector<PiecePath> GeneratePath(const OccupyMap &om, 
                                      const AD3 &from, const AD3 &to,
                                      TaskInfo& task_info);
  const std::vector<Node> &GetNodes() { return nodes_; }

  void Vis2dGrid() {
    if (!FLAGS_planning_enable_vis_event) return;
    if (heu_map_.empty()) {
      LOG_INFO("heu_map_ is empty");
      return;
    }
    auto event = vis::EventSender::Instance()->GetEvent("HeuristicMap");
    event->set_type(neodrive::visualizer::Event::k3D);
    event->add_attribute(visualizer::Event::kOdom);
    LOG_INFO("grid size: ({}, {})", size_[0], size_[1]);
    for (int i = 0; i < size_[0]; ++i) {
      for (int j = 0; j < size_[1]; ++j) {
        double x = minx_ + (i + 0.5) * conf_.grid_step,
               y = miny_ + (j + 0.5) * conf_.grid_step;
        auto txt = event->add_text();
        txt->mutable_position()->set_x(x);
        txt->mutable_position()->set_y(y);
        txt->mutable_position()->set_z(0);
        txt->set_text(std::to_string(int(heu_map_[i][j])));
      }
    }
  }

 private:
  const Config conf_{};
  std::vector<Node> nodes_{};
  AI2 size_;
  double minx_{0.0};
  double miny_{0.0};
  std::vector<std::vector<double>> heu_map_;
};

}  // namespace planning
}  // namespace neodrive
