#pragma once

#include <Eigen/Dense>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>

#include "spatiotemporal_union_model.h"
#include "spatiotemporal_union_mpc.h"
#include "src/planning/common/planning_code_define.h"

namespace neodrive {
namespace planning {

class SpatiotemporalUnionGenerator {
 public:
  SpatiotemporalUnionGenerator(
      const std::string &name, const SpatiotemporalUnion::State &init_state,
      const double speed_limit,
      const SpatiotemporalUnion::TunnelInfos &tunnel_infos,
      const std::vector<double> corridor_flytime);

  ~SpatiotemporalUnionGenerator() = default;

  ErrorCode Generator();

  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(
      std::vector<SpatiotemporalUnion::SLTPoint>, dense_t_s_l);

 private:
  void DenseSLTPoints();

 private:
  std::string name_{""};
  SpatiotemporalUnion::State init_state_{};
  double speed_limit_{};
  SpatiotemporalUnion::TunnelInfos tunnel_infos_{};
  std::vector<double> corridor_flytime_{};

  std::vector<SpatiotemporalUnion::OptVariables> opt_solution_{};
  std::vector<SpatiotemporalUnion::SLTPoint> dense_t_s_l_{};
  std::shared_ptr<SpatiotemporalUnionMPC> spatiotemporal_union_mpc_{};

  std::vector<std::vector<double>> para_for_bezier_s_{};
  std::vector<std::vector<double>> para_for_bezier_l_{};
};
}  // namespace planning
}  // namespace neodrive