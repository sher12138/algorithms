#include "src/planning/task/optimizers/freespace_path_optimizer/freespace_smoother.h"

#include <iostream>
#include <tuple>

#include "osqp.h"

namespace neodrive {
namespace planning {

namespace {

using AD2 = std::array<double, 2>;
using AD4 = std::array<double, 4>;

std::tuple<std::vector<c_float>, std::vector<c_int>, std::vector<c_int>>
BuildCscMatrixP(const std::size_t val_cnt, const double ws, const double wl,
                const double wr) {
  const size_t n = val_cnt * 2;
  std::vector<c_float> data(3 * n - 6, 0.);
  // smooth(upper triangular)
  // [[ 1  0 -2  0  1  0  0  0  0  0  0  0]
  //  [ 0  1  0 -2  0  1  0  0  0  0  0  0]
  //  [-2  0  5  0 -4  0  1  0  0  0  0  0]
  //  [ 0 -2  0  5  0 -4  0  1  0  0  0  0]
  //  [ 1  0 -4  0  6  0 -4  0  1  0  0  0]
  //  [ 0  1  0 -4  0  6  0 -4  0  1  0  0]
  //  [ 0  0  1  0 -4  0  6  0 -4  0  1  0]
  //  [ 0  0  0  1  0 -4  0  6  0 -4  0  1]
  //  [ 0  0  0  0  1  0 -4  0  5  0 -2  0]
  //  [ 0  0  0  0  0  1  0 -4  0  5  0 -2]
  //  [ 0  0  0  0  0  0  1  0 -2  0  1  0]
  //  [ 0  0  0  0  0  0  0  1  0 -2  0  1]]
  data[0] += ws;  // col 0
  data[1] += ws;  // col 1
  data[2] += -2 * ws;
  data[3] += 5 * ws;  // col 2
  data[4] += -2 * ws;
  data[5] += 5 * ws;                                  // col 3
  for (size_t i = 6; i < data.size() - 12; i += 3) {  // col [4, n - 4)
    data[i] += ws;
    data[i + 1] += -4 * ws;
    data[i + 2] += 6 * ws;
  }
  // col [n - 4, n - 2)
  for (size_t i = data.size() - 12; i < data.size() - 6; i += 3) {
    data[i] += ws;
    data[i + 1] += -4 * ws;
    data[i + 2] += 5 * ws;
  }
  for (size_t i = data.size() - 6; i < data.size(); i += 3) {  // col [n - 2, n)
    data[i] += ws;
    data[i + 1] += -2 * ws;
    data[i + 2] += ws;
  }
  // length
  // [[ 1  0 -1  0  0  0  0  0  0  0  0  0]
  //  [ 0  1  0 -1  0  0  0  0  0  0  0  0]
  //  [-1  0  2  0 -1  0  0  0  0  0  0  0]
  //  [ 0 -1  0  2  0 -1  0  0  0  0  0  0]
  //  [ 0  0 -1  0  2  0 -1  0  0  0  0  0]
  //  [ 0  0  0 -1  0  2  0 -1  0  0  0  0]
  //  [ 0  0  0  0 -1  0  2  0 -1  0  0  0]
  //  [ 0  0  0  0  0 -1  0  2  0 -1  0  0]
  //  [ 0  0  0  0  0  0 -1  0  2  0 -1  0]
  //  [ 0  0  0  0  0  0  0 -1  0  2  0 -1]
  //  [ 0  0  0  0  0  0  0  0 -1  0  1  0]
  //  [ 0  0  0  0  0  0  0  0  0 -1  0  1]]
  data[0] += wl;
  data[1] += wl;
  data[2] += -1 * wl;
  data[3] += 2 * wl;
  data[4] += -1 * wl;
  data[5] += 2 * wl;
  for (size_t i = 6; i < data.size() - 12; i += 3) {
    data[i + 1] += -wl;
    data[i + 2] += 2 * wl;
  }
  for (size_t i = data.size() - 12; i < data.size() - 6; i += 3) {
    data[i + 1] += -wl;
    data[i + 2] += 2 * wl;
  }
  for (size_t i = data.size() - 6; i < data.size(); i += 3) {
    data[i + 1] += -1 * wl;
    data[i + 2] += wl;
  }
  // reference
  // [[1 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 1 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 1 0 0 0 0 0 0 0 0 0]
  //  [0 0 0 1 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 1 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 1 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 1 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 1 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 1 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 1 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 1 0]
  //  [0 0 0 0 0 0 0 0 0 0 0 1]]
  data[0] += wr;                                      // col 0
  data[1] += wr;                                      // col 1
  data[3] += wr;                                      // col 2
  data[5] += wr;                                      // col 3
  for (size_t i = 6; i < data.size() - 12; i += 3) {  // col [4, n - 4)
    data[i + 2] += wr;
  }
  // col [n - 4, n - 2)
  for (size_t i = data.size() - 12; i < data.size() - 6; i += 3) {
    data[i + 2] += wr;
  }
  for (size_t i = data.size() - 6; i < data.size(); i += 3) {  // col [n - 2, n)
    data[i + 2] += wr;
  }
  for (auto& n : data) n *= 2;

  std::vector<c_int> row_idx(3 * n - 6, 0);
  row_idx[0] = 0;  // col 0
  row_idx[1] = 1;  // col 1
  row_idx[2] = 0;
  row_idx[3] = 2;  // col 2
  row_idx[4] = 1;
  row_idx[5] = 3;                       // col 3
  for (size_t i = 0; i < n - 4; ++i) {  // col [4, n)
    row_idx[6 + i * 3 + 0] = i;
    row_idx[6 + i * 3 + 1] = i + 2;
    row_idx[6 + i * 3 + 2] = i + 4;
  }

  std::vector<c_int> cnt_cols{0, 1, 2, 4, 6};  // col [0, 4)
  for (size_t i = 4; i < n; ++i) cnt_cols.push_back(cnt_cols.back() + 3);

  return {data, row_idx, cnt_cols};
}

std::vector<c_float> CalcCscLowerBound(
    const std::vector<AD2>& xys,
    const std::vector<AD4>& bounds) {
  std::vector<c_float> ans(xys.size() * 2, 0);
  for (size_t i = 0; i < bounds.size(); ++i) {
    ans[i * 2] = xys[i][0] - bounds[i][0];
    ans[i * 2 + 1] = xys[i][1] - bounds[i][2];
  }
  return ans;
}

std::vector<c_float> CalcCscUpperBound(
    const std::vector<AD2>& xys,
    const std::vector<AD4>& bounds) {
  std::vector<double> ans(xys.size() * 2, 0);
  for (size_t i = 0; i < bounds.size(); ++i) {
    ans[i * 2] = xys[i][0] + bounds[i][1];
    ans[i * 2 + 1] = xys[i][1] + bounds[i][3];
  }
  return ans;
}

std::tuple<std::vector<c_float>, std::vector<c_int>, std::vector<c_int>>
BuildCscMatrixA(const std::size_t val_cnt) {
  const auto n = val_cnt * 2;
  std::vector<double> data(n, 1);
  std::vector<c_int> col_idx(n, 0);
  for (size_t i = 0; i < n; ++i) col_idx[i] = i;
  std::vector<c_int> cnt_col{0};
  for (size_t i = 0; i < n; ++i) cnt_col.push_back(cnt_col.back() + 1);
  return {data, col_idx, cnt_col};
}

std::vector<c_float> CalcWarmStart(
    const std::vector<AD2>& pts) {
  std::vector<c_float> ans{};
  for (auto [x, y] : pts) {
    ans.push_back(x);
    ans.push_back(y);
  }
  return ans;
}

}  // namespace

FreespaceSmoother::FreespaceSmoother(FreespaceSmoother::Config&& conf)
    : conf_{conf} {}

std::vector<AD2> FreespaceSmoother::Smooth(
    const std::vector<AD2>& ref_points,
    const std::vector<AD4>& bounds) const {
  if (ref_points.size() < 3) return ref_points;
  size_t val_cnt = ref_points.size();

  auto [p_data, p_row_idx, p_cnt_col] =
      BuildCscMatrixP(val_cnt, conf_.weight_smooth, conf_.weight_length,
                      conf_.weight_reference);

  std::vector<c_float> q(ref_points.size() * 2, 0);
  for (size_t i = 0; i < ref_points.size(); ++i) {
    q[i * 2] = -2 * conf_.weight_reference * ref_points[i][0];
    q[i * 2 + 1] = -2 * conf_.weight_reference * ref_points[i][1];
  }

  auto [a_data, a_row_idx, a_cnt_col] = BuildCscMatrixA(val_cnt);

  auto lb = CalcCscLowerBound(ref_points, bounds);
  auto ub = CalcCscUpperBound(ref_points, bounds);

  auto warm = CalcWarmStart(ref_points);

  // Workspace structures
  OSQPWorkspace* work;
  OSQPSettings* settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
  OSQPData* data = (OSQPData*)c_malloc(sizeof(OSQPData));

  // Populate data
  data->n = val_cnt * 2;  // number of variables
  data->m = val_cnt * 2;  // number of constraints
  data->P = csc_matrix(data->n, data->n, p_data.size(), p_data.data(),
                       p_row_idx.data(), p_cnt_col.data());

  data->q = q.data();
  data->A = csc_matrix(data->n, data->n, a_data.size(), a_data.data(),
                       a_row_idx.data(), a_cnt_col.data());
  data->l = lb.data();
  data->u = ub.data();

  // Define solver settings as default
  osqp_set_default_settings(settings);
  settings->alpha = 1.0;  // Change alpha parameter
  settings->max_iter = conf_.max_iteration;
  settings->time_limit = conf_.max_time;
  // settings->verbose = conf_.is_verbose;
  settings->verbose = false;
  settings->scaled_termination = conf_.is_scaled_termination;
  settings->warm_start = conf_.is_warm_start;

  work = osqp_setup(data, settings);

  osqp_warm_start_x(work, warm.data());

  // Solve Problem
  std::vector<AD2> ans{};
  if (auto f = osqp_solve(work); f == 0 || f == 2) {
    for (size_t i = 0; i < val_cnt; ++i) {
      ans.push_back({work->solution->x[i * 2], work->solution->x[i * 2 + 1]});
    }
  }
  // print_pts(ans, "mine ans\n");

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return ans;
}

}  // namespace planning
}  // namespace neodrive
