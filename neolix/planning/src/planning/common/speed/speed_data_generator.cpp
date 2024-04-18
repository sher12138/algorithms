#include "speed_data_generator.h"

namespace neodrive {
namespace planning {

bool SpeedDataGenerator::add_point_sequence(
    const std::vector<STPoint>& st_profile, const double init_speed,
    const double init_acc, const double init_jerk) {
  if (st_profile.size() < 2) return false;

  points_ = st_profile;
  init_point_ = st_profile.front();
  init_speed_ = init_speed;
  init_acc_ = init_acc / 2;
  init_jerk_ = init_jerk / 6;
  left_matrix_.resize(st_profile.size() + 1, 2);
  b_.resize(st_profile.size() + 1);

  for (std::size_t i = 0; i < st_profile.size(); ++i) {
    double delta_t = st_profile[i].t() - init_point_.t();
    left_matrix_(i, 0) = std::pow(delta_t, 4);
    left_matrix_(i, 1) = std::pow(delta_t, 5);
    b_(i) = st_profile[i].s() - init_point_.s() - init_speed_ * delta_t -
            init_acc_ * delta_t * delta_t - init_jerk_ * pow(delta_t, 3);
  }

  double end_t = st_profile.back().t() - init_point_.t();
  left_matrix_(st_profile.size(), 0) = 12 * end_t * end_t * 0.1;
  left_matrix_(st_profile.size(), 1) = 20 * pow(end_t, 3) * 0.1;
  b_(st_profile.size()) = (-2 * init_acc_ - 6 * init_jerk_ * end_t) * 0.1;

  return left_matrix_.rows() == b_.rows();
}

void SpeedDataGenerator::solve() {
  Eigen::VectorXd param =
      left_matrix_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
          .solve(b_);
  derk_ = param(0);
  ferk_ = param(1);
}

bool SpeedDataGenerator::get_speed_point_at_time(
    const double time, SpeedPoint* const speed_point) {
  if (speed_point == nullptr) return false;

  double delta_t = time - init_point_.t();
  if (delta_t < -1e-3 || delta_t > points_.back().t() - init_point_.t())
    return false;

  double delta_t_sqr = delta_t * delta_t;
  double delta_t_pow_3 = delta_t_sqr * delta_t;
  double delta_t_pow_4 = delta_t_sqr * delta_t_sqr;
  double s = init_point_.s() + init_speed_ * delta_t + init_acc_ * delta_t_sqr +
             init_jerk_ * delta_t_pow_3 + derk_ * delta_t_pow_4 +
             ferk_ * delta_t_pow_4 * delta_t;
  double v = init_speed_ + 2 * init_acc_ * delta_t +
             3 * init_jerk_ * delta_t_sqr + 4 * derk_ * delta_t_pow_3 +
             5 * ferk_ * delta_t_pow_4;
  double a = 2 * init_acc_ + 6 * init_jerk_ * delta_t +
             12 * derk_ * delta_t_sqr + 20 * ferk_ * delta_t_pow_3;
  double j = 6 * init_jerk_ + 24 * derk_ * delta_t + 60 * ferk_ * delta_t_sqr;
  *speed_point = SpeedPoint(s, time, v, a, j);
  return true;
}

}  // namespace planning
}  // namespace neodrive
