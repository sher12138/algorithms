#include "node2d.h"

namespace neodrive {
namespace planning {

Node2d::Node2d(const double x, const double y, const double xy_resolution,
               const std::vector<double>& XYbounds) {
  // XYbounds with xmin, xmax, ymin, ymax
  grid_x_ = static_cast<int>((x - XYbounds[0]) / xy_resolution);
  grid_y_ = static_cast<int>((y - XYbounds[2]) / xy_resolution);
  index_ = ComputeStringIndex(grid_x_, grid_y_);
}
Node2d::Node2d(const int grid_x, const int grid_y,
               const std::vector<double>& XYbounds) {
  grid_x_ = grid_x;
  grid_y_ = grid_y;
  index_ = ComputeStringIndex(grid_x_, grid_y_);
}
void Node2d::SetPathCost(const double path_cost) {
  path_cost_ = path_cost;
  cost_ = path_cost_ + heuristic_;
}
void Node2d::SetHeuristic(const double heuristic) {
  heuristic_ = heuristic;
  cost_ = path_cost_ + heuristic_;
}
void Node2d::SetCost(const double cost) { cost_ = cost; }
void Node2d::SetPreNode(std::shared_ptr<Node2d> pre_node) {
  pre_node_ = pre_node;
}
double Node2d::GetGridX() const { return grid_x_; }
double Node2d::GetGridY() const { return grid_y_; }
double Node2d::GetPathCost() const { return path_cost_; }
double Node2d::GetHeuCost() const { return heuristic_; }
double Node2d::GetCost() const { return cost_; }
const std::string& Node2d::GetIndex() const { return index_; }
std::shared_ptr<Node2d> Node2d::GetPreNode() const { return pre_node_; }
std::string Node2d::CalcIndex(const double x, const double y,
                              const double xy_resolution,
                              const std::vector<double>& XYbounds) {
  // XYbounds with xmin, xmax, ymin, ymax
  int grid_x = static_cast<int>((x - XYbounds[0]) / xy_resolution);
  int grid_y = static_cast<int>((y - XYbounds[2]) / xy_resolution);
  return ComputeStringIndex(grid_x, grid_y);
}
bool Node2d::operator==(const Node2d& right) const {
  return right.GetIndex() == index_;
}
std::string Node2d::ComputeStringIndex(int x_grid, int y_grid) {
  std::string tmp;
  tmp = std::to_string(x_grid) + "_" + std::to_string(y_grid);
  return tmp;
}

}  // namespace planning
}  // namespace neodrive
