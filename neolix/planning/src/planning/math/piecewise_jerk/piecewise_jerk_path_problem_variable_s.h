#pragma once

#include "piecewise_jerk_problem.h"

namespace neodrive {
namespace planning {

class PiecewiseJerkPathProblemVariableS : public PiecewiseJerkProblem {
 public:
  PiecewiseJerkPathProblemVariableS(const std::size_t num_of_knots,
                                    const int max_iter, const double delta_s,
                                    const double time_limit,
                                    const std::array<double, 3>& x_init,
                                    const std::vector<double>& delta_s_vec);

  virtual ~PiecewiseJerkPathProblemVariableS() = default;

 protected:
  void CalculateKernel(std::vector<c_float>& P_data,
                       std::vector<c_int>& P_indices,
                       std::vector<c_int>& P_ind) override;

  void CalculateOffset(std::vector<c_float>& q) override;
};

}  // namespace planning
}  // namespace neodrive