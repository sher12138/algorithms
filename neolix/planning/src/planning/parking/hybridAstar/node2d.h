#pragma once

#include <memory>
#include <string>
#include <vector>

namespace neodrive {
namespace planning {

class Node2d {
 public:
  // XYbounds[xmin, xmax, ymin, ymax]
  Node2d(const double x, const double y, const double xy_resolution,
         const std::vector<double>& XYbounds);
  Node2d(const int grid_x, const int grid_y,
         const std::vector<double>& XYbounds);

  void SetPathCost(const double path_cost);
  void SetHeuristic(const double heuristic);
  void SetCost(const double cost);
  void SetPreNode(std::shared_ptr<Node2d> pre_node);

  double GetGridX() const;
  double GetGridY() const;
  double GetPathCost() const;
  double GetHeuCost() const;
  double GetCost() const;
  const std::string& GetIndex() const;
  std::shared_ptr<Node2d> GetPreNode() const;
  static std::string CalcIndex(const double x, const double y,
                               const double xy_resolution,
                               const std::vector<double>& XYbounds);
  bool operator==(const Node2d& right) const;

 private:
  static std::string ComputeStringIndex(int x_grid, int y_grid);

 private:
  int grid_x_ = 0;
  int grid_y_ = 0;
  double path_cost_ = 0.0;
  double heuristic_ = 0.0;
  double cost_ = 0.0;
  std::string index_;
  std::shared_ptr<Node2d> pre_node_ = nullptr;
};

}  // namespace planning
}  // namespace neodrive
