#pragma once

#include <memory>
#include <string>
#include <vector>

#include "src/planning/common/planning_logger.h"
#include "src/planning/parking/parking_config.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

class Node3d {
 public:
  Node3d(const double x, const double y, const double phi);
  Node3d(const double x, const double y, const double phi,
         const std::vector<double>& XYbounds,
         const ParkingHybridAStarConfig& open_space_conf);
  Node3d(const std::vector<double>& traversed_x,
         const std::vector<double>& traversed_y,
         const std::vector<double>& traversed_phi,
         const std::vector<double>& XYbounds,
         const ParkingHybridAStarConfig& open_space_conf);
  virtual ~Node3d() = default;
  bool operator==(const Node3d& right) const;

  double GetX() const { return x_; }
  double GetY() const { return y_; }
  double GetPhi() const { return phi_; }
  std::size_t GetStepSize() const { return step_size_; }
  const std::vector<double>& GetXs() const { return traversed_x_; }
  const std::vector<double>& GetYs() const { return traversed_y_; }
  const std::vector<double>& GetPhis() const { return traversed_phi_; }
  int GetGridX() const { return x_grid_; }
  int GetGridY() const { return y_grid_; }
  int GetGridPhi() const { return phi_grid_; }
  const std::string& GetIndex() const { return index_; }

  double GetTrajCost() const { return traj_cost_; }
  double GetHeuCost() const { return heuristic_cost_; }
  double GetCost() const { return traj_cost_ + heuristic_cost_; }
  std::shared_ptr<Node3d> GetPreNode() const { return pre_node_; }
  double GetSteer() const { return steering_; }
  bool GetDirec() const { return direction_; }

  void SetTrajCost(double cost) { traj_cost_ = cost; }
  void SetHeuCost(double cost) { heuristic_cost_ = cost; }
  void SetPre(std::shared_ptr<Node3d> pre_node) { pre_node_ = pre_node; }
  void SetSteer(double steering) { steering_ = steering; }
  void SetDirec(bool direction) { direction_ = direction; }

 private:
  static std::string ComputeStringIndex(int x_grid, int y_grid, int phi_grid);

 private:
  double x_ = 0.0;
  double y_ = 0.0;
  double phi_ = 0.0;
  std::size_t step_size_ = 1;
  std::vector<double> traversed_x_;
  std::vector<double> traversed_y_;
  std::vector<double> traversed_phi_;
  int x_grid_ = 0;
  int y_grid_ = 0;
  int phi_grid_ = 0;
  std::string index_;
  double traj_cost_ = 0.0;
  double heuristic_cost_ = 0.0;
  double cost_ = 0.0;
  std::shared_ptr<Node3d> pre_node_ = nullptr;
  double steering_ = 0.0;
  // true for moving forward and false for moving backward
  bool direction_ = true;
};

}  // namespace planning
}  // namespace neodrive
