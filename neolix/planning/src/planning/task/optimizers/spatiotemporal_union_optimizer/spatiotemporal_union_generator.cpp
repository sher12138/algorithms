#include "spatiotemporal_union_generator.h"

namespace neodrive {
namespace planning {

SpatiotemporalUnionGenerator::SpatiotemporalUnionGenerator(
    const std::string &name, const SpatiotemporalUnion::State &init_state,
    const double speed_limit,
    const SpatiotemporalUnion::TunnelInfos &tunnel_infos,
    const std::vector<double> corridor_flytime)
    : name_(name),
      init_state_(init_state),
      speed_limit_(speed_limit),
      tunnel_infos_(tunnel_infos),
      corridor_flytime_(corridor_flytime) {}

ErrorCode SpatiotemporalUnionGenerator::Generator() {
  spatiotemporal_union_mpc_ = std::make_shared<SpatiotemporalUnionMPC>(
      init_state_, speed_limit_, tunnel_infos_, corridor_flytime_);
  if (!spatiotemporal_union_mpc_->Process()) {
    LOG_ERROR("Spatiotemporal_Union_Optimizer failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  opt_solution_ = spatiotemporal_union_mpc_->optimal_solution();

  DenseSLTPoints();

  return ErrorCode::PLANNING_OK;
}

void SpatiotemporalUnionGenerator::DenseSLTPoints() {
  for (const auto &variable : opt_solution_) {
    std::vector<double> para_s_by_row;
    para_s_by_row.emplace_back(variable.xk.state_p_s);
    para_s_by_row.emplace_back(variable.xk.state_v_s);
    para_s_by_row.emplace_back(variable.xk.state_a_s / 2);
    para_s_by_row.emplace_back(variable.xk.state_j_s / 6);
    para_s_by_row.emplace_back(variable.uk.control_jerk_d_s / 24);
    para_for_bezier_s_.emplace_back(std::move(para_s_by_row));

    std::vector<double> para_l_by_row;
    para_l_by_row.emplace_back(variable.xk.state_p_l);
    para_l_by_row.emplace_back(variable.xk.state_v_l);
    para_l_by_row.emplace_back(variable.xk.state_a_l / 2);
    para_l_by_row.emplace_back(variable.xk.state_j_l / 6);
    para_l_by_row.emplace_back(variable.uk.control_jerk_d_l / 24);
    para_for_bezier_l_.emplace_back(std::move(para_l_by_row));
  }

  double dense_time{0.05};
  double pass_time{0.0};
  for (int j = 0; j < corridor_flytime_.size(); j++) {
    for (std::size_t k = 0;
         k < static_cast<std::size_t>(corridor_flytime_[j] / dense_time); ++k) {
      SpatiotemporalUnion::SLTPoint slt_point;
      double time_point_sum = k * dense_time + pass_time;
      double time_point = k * dense_time;
      slt_point.t = time_point_sum;
      slt_point.s = para_for_bezier_s_[j][0] +
                    para_for_bezier_s_[j][1] * time_point +
                    para_for_bezier_s_[j][2] * std::pow(time_point, 2) +
                    para_for_bezier_s_[j][3] * std::pow(time_point, 3) +
                    para_for_bezier_s_[j][4] * std::pow(time_point, 4);
      slt_point.v_s = para_for_bezier_s_[j][1] + 2 * para_for_bezier_s_[j][2] +
                      3 * para_for_bezier_s_[j][3] * std::pow(time_point, 2) +
                      4 * para_for_bezier_s_[j][4] * std::pow(time_point, 3);
      slt_point.a_s = 2 * para_for_bezier_s_[j][2] +
                      6 * para_for_bezier_s_[j][3] * time_point +
                      12 * para_for_bezier_s_[j][4] * std::pow(time_point, 2);
      slt_point.l = para_for_bezier_l_[j][0] +
                    para_for_bezier_l_[j][1] * time_point +
                    para_for_bezier_l_[j][2] * std::pow(time_point, 2) +
                    para_for_bezier_l_[j][3] * std::pow(time_point, 3) +
                    para_for_bezier_l_[j][4] * std::pow(time_point, 4);
      slt_point.v_l = para_for_bezier_l_[j][1] + 2 * para_for_bezier_l_[j][2] +
                      3 * para_for_bezier_l_[j][3] * std::pow(time_point, 2) +
                      4 * para_for_bezier_l_[j][4] * std::pow(time_point, 3);
      slt_point.a_l = 2 * para_for_bezier_l_[j][2] +
                      6 * para_for_bezier_l_[j][3] * time_point +
                      12 * para_for_bezier_l_[j][4] * std::pow(time_point, 2);
      dense_t_s_l_.emplace_back(std::move(slt_point));
    }
    pass_time = pass_time + corridor_flytime_[j];
  }
}

}  // namespace planning
}  // namespace neodrive
