#pragma once

#include "src/planning/common/speed/st_point.h"

namespace neodrive {
namespace planning {

class STGraphPoint {
 public:
  ~STGraphPoint() = default;

  void init(const std::size_t index_t, const std::size_t index_s,
            const STPoint& st_point);

  const STPoint& point() const;
  STPoint* mutable_point();
  const STGraphPoint* pre_point() const;

  std::size_t index_s() const;
  std::size_t index_t() const;
  double edge_v() const;
  double edge_a() const;
  bool is_feasible() const;
  double reference_cost() const;
  double obstacle_cost() const;
  double total_cost() const;
  double GetOptimalSpeed() const;

  void set_pre_point(const STGraphPoint& pre_point);
  void set_edge_v(const double v);
  void set_edge_a(const double a);
  void set_feasibility(const bool feasible);
  void set_reference_cost(const double reference_cost);
  void set_obstacle_cost(const double obs_cost);
  void set_total_cost(const double total_cost);
  void SetOptimalSpeed(const double optimal_speed);

 private:
  STPoint point_;
  const STGraphPoint* pre_point_ = nullptr;
  std::size_t index_s_ = 0;
  std::size_t index_t_ = 0;
  double edge_v_ = 0.0;
  double edge_a_ = 0.0;

  bool feasible_ = true;
  double reference_cost_ = 0.0;
  double obstacle_cost_ = 0.0;
  double total_cost_ = 0.0;
  double optimal_speed_{0.0};
};

}  // namespace planning
}  // namespace neodrive
