#pragma once

#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "node2d.h"
#include "common/math/segment2d.h"
#include "src/planning/parking/parking_config.h"

namespace neodrive {
namespace planning {

struct GridAStartResult {
  std::vector<double> x;
  std::vector<double> y;
  double path_cost = 0.0;
};

class GridSearch {
 public:
  explicit GridSearch(const ParkingHybridAStarConfig& config);
  ~GridSearch() = default;

  bool GenerateAStarPath(
      const double sx, const double sy, const double ex, const double ey,
      const std::vector<double>& XYbounds,
      const std::vector<std::vector<Segment2d>>& obstacles_linesegments_vec,
      GridAStartResult* result);

  bool GenerateDpMap(
      const double ex, const double ey, const std::vector<double>& XYbounds,
      const std::vector<std::vector<Segment2d>>& obstacles_linesegments_vec);

  double CheckDpMap(const double sx, const double sy);

 private:
  double EuclidDistance(const double x1, const double y1, const double x2,
                        const double y2);

  void LoadGridAStarResult(GridAStartResult* result);

  std::vector<std::shared_ptr<Node2d>> GenerateNextNodes(
      std::shared_ptr<Node2d> node);

  bool CheckConstraints(std::shared_ptr<Node2d> node);

 private:
  double xy_grid_resolution_ = 0.0;
  double node_radius_ = 0.0;
  double step_length_ = 1.0;
  std::vector<double> XYbounds_;
  double max_grid_x_ = 0.0;
  double max_grid_y_ = 0.0;
  std::shared_ptr<Node2d> start_node_;
  std::shared_ptr<Node2d> end_node_;
  std::shared_ptr<Node2d> final_node_;
  std::vector<std::vector<Segment2d>> obstacles_linesegments_vec_;

  struct cmp {
    bool operator()(const std::pair<std::string, double>& left,
                    const std::pair<std::string, double>& right) const {
      return left.second >= right.second;
    }
  };
  std::unordered_map<std::string, std::shared_ptr<Node2d>> dp_map_;
};

}  // namespace planning
}  // namespace neodrive
