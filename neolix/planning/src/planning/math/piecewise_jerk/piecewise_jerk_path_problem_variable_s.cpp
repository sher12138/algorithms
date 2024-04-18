#include "piecewise_jerk_path_problem_variable_s.h"

namespace neodrive {
namespace planning {

PiecewiseJerkPathProblemVariableS::PiecewiseJerkPathProblemVariableS(
    const std::size_t num_of_knots, const int max_iter, const double delta_s,
    const double time_limit, const std::array<double, 3>& x_init,
    const std::vector<double>& delta_s_vec)
    : PiecewiseJerkProblem(num_of_knots, max_iter, delta_s, time_limit,
                           x_init) {
  set_delta_s_vec(delta_s_vec);
}

void PiecewiseJerkPathProblemVariableS::CalculateKernel(
    std::vector<c_float>& P_data, std::vector<c_int>& P_indices,
    std::vector<c_int>& P_ind) {
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * n;
  const int kNumValue = kNumParam + (n - 1);
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(kNumParam);
  int value_index = 0;

  // x(i)^2 * (w_x + w_x_ref + w_x_obs_dis)
  for (int i = 0; i < n - 1; ++i) {
    columns[i].emplace_back(
        i, weight_x_ + weight_x_ref_vec_[i] + weight_x_obs_dis_vec_[i]);
    ++value_index;
  }
  columns[n - 1].emplace_back(n - 1, weight_x_ + weight_x_ref_vec_[n - 1] +
                                         weight_end_state_[0] +
                                         weight_x_obs_dis_vec_[n - 1]);
  ++value_index;

  // x(i)'^2 * w_dx
  for (int i = 0; i < n - 1; ++i) {
    columns[n + i].emplace_back(n + i, weight_dx_);
    ++value_index;
  }
  // x(n-1)'^2 * (w_dx + w_end_dx)
  columns[2 * n - 1].emplace_back(2 * n - 1, weight_dx_ + weight_end_state_[1]);
  ++value_index;

  auto delta_s_square = delta_s_vec_.front() * delta_s_vec_.front();
  // x(i)''^2 * (w_ddx + 2 * w_dddx / delta_s^2)
  columns[2 * n].emplace_back(2 * n,
                              weight_ddx_ + weight_dddx_ / delta_s_square);
  ++value_index;
  for (int i = 1; i < n - 1; ++i) {
    delta_s_square = delta_s_vec_[i + 1] * delta_s_vec_[i + 1];
    columns[2 * n + i].emplace_back(
        2 * n + i, weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square);
    ++value_index;
  }
  delta_s_square = delta_s_vec_.back() * delta_s_vec_.back();
  columns[3 * n - 1].emplace_back(
      3 * n - 1,
      weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]);
  ++value_index;

  // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
  for (int i = 0; i < n - 1; ++i) {
    delta_s_square = delta_s_vec_[i + 1] * delta_s_vec_[i + 1];
    columns[2 * n + i].emplace_back(2 * n + i + 1,
                                    -2.0 * weight_dddx_ / delta_s_square);
    ++value_index;
  }

  if (value_index != kNumValue) {
    LOG_ERROR("value_index != kNumValue");
    return;
  }

  int ind_p = 0;
  for (int i = 0; i < kNumParam; ++i) {
    P_ind.push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      P_data.push_back(row_data_pair.second * 2.0);
      P_indices.push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_ind.push_back(ind_p);
  LOG_DEBUG(
      "delta_s_ {}, weight_x_ref_ {}, "
      "weight_x_ {}, weight_dx_ {}, weight_ddx_ {}, weight_dddx_ {}, ",
      delta_s_, weight_x_ref_, weight_x_, weight_dx_, weight_ddx_,
      weight_dddx_);
  LOG_DEBUG("weight_end_state_[0-2] ({}, {}, {})", weight_end_state_[0],
            weight_end_state_[1], weight_end_state_[2]);
}

void PiecewiseJerkPathProblemVariableS::CalculateOffset(
    std::vector<c_float>& q) {
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * n;
  q.resize(kNumParam, 0.0);

  if (has_x_ref_) {
    for (int i = 0; i < n; ++i) {
      q.at(i) += -2.0 * weight_x_ref_vec_[i] * x_ref_[i];
    }
  }

  if (has_x_obs_dis_) {
    for (int i = 0; i < n; ++i) {
      q.at(i) += -2.0 * weight_x_obs_dis_vec_[i] * x_obs_dis_[i];
      LOG_DEBUG(
          "i {}, weight_x_obs_dis_vec_[i] {}, x_obs_dis_[i] {}, q.at(i) {}", i,
          weight_x_obs_dis_vec_[i], x_obs_dis_[i], q.at(i));
    }
  }

  if (has_end_state_ref_) {
    q.at(n - 1) += -2.0 * weight_end_state_[0] * end_state_ref_[0];
    q.at(2 * n - 1) += -2.0 * weight_end_state_[1] * end_state_ref_[1];
    q.at(3 * n - 1) += -2.0 * weight_end_state_[2] * end_state_ref_[2];
  }
}

}  // namespace planning
}  // namespace neodrive
