#include "st_graph_point.h"

namespace neodrive {
namespace planning {

void STGraphPoint::init(const std::size_t index_t, const std::size_t index_s,
                        const STPoint& st_point) {
  index_t_ = index_t;
  index_s_ = index_s;
  point_ = st_point;
}
const STPoint& STGraphPoint::point() const { return point_; }
STPoint* STGraphPoint::mutable_point() { return &point_; }
const STGraphPoint* STGraphPoint::pre_point() const { return pre_point_; }

std::size_t STGraphPoint::index_s() const { return index_s_; }

std::size_t STGraphPoint::index_t() const { return index_t_; }

double STGraphPoint::edge_v() const { return edge_v_; }

double STGraphPoint::edge_a() const { return edge_a_; }
bool STGraphPoint::is_feasible() const { return feasible_; }
double STGraphPoint::reference_cost() const { return reference_cost_; }

double STGraphPoint::obstacle_cost() const { return obstacle_cost_; }

double STGraphPoint::total_cost() const { return total_cost_; }

void STGraphPoint::set_pre_point(const STGraphPoint& pre_point) {
  pre_point_ = &pre_point;
}
void STGraphPoint::set_edge_v(const double v) { edge_v_ = v; }

void STGraphPoint::set_edge_a(const double a) { edge_a_ = a; }

void STGraphPoint::set_feasibility(const bool feasible) {
  feasible_ = feasible;
}

void STGraphPoint::set_reference_cost(const double reference_cost) {
  reference_cost_ = reference_cost;
}

void STGraphPoint::set_obstacle_cost(const double obs_cost) {
  obstacle_cost_ = obs_cost;
}

void STGraphPoint::set_total_cost(const double total_cost) {
  total_cost_ = total_cost;
}

double STGraphPoint::GetOptimalSpeed() const { return optimal_speed_; }

void STGraphPoint::SetOptimalSpeed(const double optimal_speed) {
  optimal_speed_ = optimal_speed;
}

}  // namespace planning
}  // namespace neodrive
