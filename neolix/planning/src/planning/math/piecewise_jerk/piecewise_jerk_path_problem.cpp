#include "piecewise_jerk_path_problem.h"

namespace neodrive {
namespace planning {

PiecewiseJerkPathProblem::PiecewiseJerkPathProblem(
    const std::size_t num_of_knots, const int max_iter, const double delta_s,
    const double time_limit, const std::array<double, 3> &x_init)
    : PiecewiseJerkProblem(num_of_knots, max_iter, delta_s, time_limit,
                           x_init) {}

void PiecewiseJerkPathProblem::CalculateKernel(std::vector<c_float> &P_data,
                                               std::vector<c_int> &P_indices,
                                               std::vector<c_int> &P_ind) {
  const int n = static_cast<int>(num_of_knots_);
  int kNumParam = 3 * n;
  if (FLAGS_planning_piecewise_use_soft_constraints) {
    kNumParam = 4 * n;
  }
  const int kNumValue = kNumParam;
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(kNumParam);
  int value_index = 0;

  // x(i)^2 * (w_x + w_x_ref + w_x_obs_dis)
  for (int i = 0; i < n - 1; ++i) {
    // columns[i].emplace_back(i, weight_x_ + weight_x_ref_ +
    // weight_x_obs_dis_);
    // TEST
    columns[i].emplace_back(
        i, weight_x_ + weight_x_ref_vec_[i] + weight_x_obs_dis_vec_[i]);
    ++value_index;
  }
  // x(n-1)^2 * (w_x + w_x_ref + w_end_x + w_x_obs_dis)
  // columns[n - 1].emplace_back(
  //     n - 1,
  //     weight_x_ + weight_x_ref_ + weight_end_state_[0] + weight_x_obs_dis_);
  // TEST
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

  auto delta_s_square = delta_s_ * delta_s_;
  // x(i)''^2 * (w_ddx + 2 * w_dddx / delta_s^2)
  columns[2 * n].emplace_back(2 * n,
                              weight_ddx_ + weight_dddx_ / delta_s_square);
  ++value_index;
  for (int i = 1; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(
        2 * n + i, weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square);
    // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
    columns[2 * n + i - 1].emplace_back(2 * n + i,
                                        -2.0 * weight_dddx_ / delta_s_square);
    ++value_index;
  }
  columns[3 * n - 1].emplace_back(
      3 * n - 1,
      weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]);
  // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
  columns[3 * n - 2].emplace_back(3 * n - 1,
                                  -2.0 * weight_dddx_ / delta_s_square);
  ++value_index;

  // // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
  // for (int i = 0; i < n - 1; ++i) {
  //   columns[2 * n + i].emplace_back(2 * n + i + 1,
  //                                   -2.0 * weight_dddx_ / delta_s_square);
  //   ++value_index;
  // }

  // soft constraints on x
  if (FLAGS_planning_piecewise_use_soft_constraints) {
    for (int i = 0; i < n; ++i) {
      columns[3 * n + i].emplace_back(3 * n + i, weight_soft_x_);
      ++value_index;
    }
  }

  if (value_index != kNumValue) {
    LOG_ERROR("value_index != kNumValue");
    return;
  }

  int ind_p = 0;
  for (int i = 0; i < kNumParam; ++i) {
    P_ind.push_back(ind_p);
    for (const auto &row_data_pair : columns[i]) {
      P_data.push_back(row_data_pair.second * 2.0);
      P_indices.push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_ind.push_back(ind_p);
  LOG_DEBUG(
      "delta_s_ {}, weight_x_ref_ {}, weight_x_ {}, weight_dx_ {}, weight_ddx_ "
      "{}, weight_dddx_ {}, weight_soft_x_ {}",
      delta_s_, weight_x_ref_, weight_x_, weight_dx_, weight_ddx_, weight_dddx_,
      weight_soft_x_);
  LOG_DEBUG("weight_end_state_[0-2] ({}, {}, {})", weight_end_state_[0],
            weight_end_state_[1], weight_end_state_[2]);
  // for (std::size_t i = 0; i < columns.size(); ++i) {
  //   for (std::size_t j = 0; j < columns[i].size(); ++j) {
  //     LOG_DEBUG("[{}][{}], ({}, {})"
  //              , i, j, columns[i][j].first, columns[i][j].second);
  //   }
  // }
  // LOG_DEBUG("P_data.size() {}", P_data.size());
  // for (std::size_t i = 0; i < P_data.size(); ++i) {
  //   LOG_DEBUG("{}, *P_data[i] {}", i, (*P_data)[i]);
  // }
  // LOG_DEBUG("P_indices.size() {}", P_indices.size());
  // for (std::size_t i = 0; i < P_indices.size(); ++i) {
  //   LOG_DEBUG("{}, *P_indices[i] {}", i, (*P_indices)[i]);
  // }
  // LOG_DEBUG("P_ind.size() {}", P_ind.size());
  // for (std::size_t i = 0; i < P_ind.size(); ++i) {
  //   LOG_DEBUG("{}, *P_ind[i] {}", i, (*P_ind)[i]);
  // }
}

void PiecewiseJerkPathProblem::CalculateOffset(std::vector<c_float> &q) {
  const int n = static_cast<int>(num_of_knots_);
  int kNumParam = 3 * n;
  if (FLAGS_planning_piecewise_use_soft_constraints) {
    kNumParam = 4 * n;
  }
  q.resize(kNumParam, 0.0);

  if (has_x_ref_) {
    for (int i = 0; i < n; ++i) {
      // q.at(i) += -2.0 * weight_x_ref_ * x_ref_[i];
      // TEST
      q.at(i) += -2.0 * weight_x_ref_vec_[i] * x_ref_[i];
      // LOG_DEBUG("i {}, weight_x_ref_ {}, x_ref_[i] {}, q.at(i) {}"
      //          , i, weight_x_ref_, x_ref_[i], q.at(i));
    }
  }

  // if (has_x_obs_dis_) {
  //   for (int i = 0; i < n; ++i) {
  //     q.at(i) += -2.0 * weight_x_obs_dis_ * x_obs_dis_[i];
  //     LOG_DEBUG("i {}, weight_x_obs_dis_ {}, x_obs_dis_[i] {}, q.at(i) {}",
  //     i,
  //               weight_x_obs_dis_, x_obs_dis_[i], q.at(i));
  //   }
  // }
  if (has_x_obs_dis_) {
    for (int i = 0; i < n; ++i) {
      q.at(i) += -2.0 * weight_x_obs_dis_vec_[i] * x_obs_dis_[i];
      LOG_DEBUG(
          "i {}, weight_x_obs_dis_vec_[i] {}, x_obs_dis_[i] {}, q.at(i) {}", i,
          weight_x_obs_dis_vec_[i], x_obs_dis_[i], q.at(i));
    }
  }

  // for (std::size_t i = 0; i < q.size(); ++i) {
  //   LOG_DEBUG("q[{}] {}", i, (*q)[i]);
  // }

  if (has_end_state_ref_) {
    q.at(n - 1) += -2.0 * weight_end_state_[0] * end_state_ref_[0];
    q.at(2 * n - 1) += -2.0 * weight_end_state_[1] * end_state_ref_[1];
    q.at(3 * n - 1) += -2.0 * weight_end_state_[2] * end_state_ref_[2];
  }
}

}  // namespace planning
}  // namespace neodrive
