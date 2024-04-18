#pragma once

#include "spatiotemporal_union_model.h"
#include "src/planning/math/hpipm_solver/spatiotemporal_union_hpipm_solver.h"

namespace neodrive {
namespace planning {

class SpatiotemporalUnionMPC {
 public:
  SpatiotemporalUnionMPC() = delete;
  SpatiotemporalUnionMPC(const SpatiotemporalUnion::State &init_state,
                         const double speed_limit,
                         const SpatiotemporalUnion::TunnelInfos &tunnel_infos,
                         const std::vector<double> corridor_flytime);
  ~SpatiotemporalUnionMPC();

  bool Process();

  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(
      std::vector<SpatiotemporalUnion::OptVariables>, optimal_solution);

 private:
  bool SetStage(size_t step);

  bool SetMPCProblem();

  SpatiotemporalUnion::LineModelMatrix CalLinemodelAtStep(
      const std::size_t step);

  SpatiotemporalUnion::PolytopicConstraints CalConstraintsAtStep(
      const std::size_t step);

 private:
  SpatiotemporalUnion::State init_state_{};
  double speed_limit_{};
  SpatiotemporalUnion::TunnelInfos tunnel_infos_{};
  std::vector<double> corridor_flytime_{};

  int n_stage_size_{};
  int nx_{};
  int nu_{};
  int npc_{};
  int ns_{};
  std::shared_ptr<SpatiotemporalUnionModel> vehicle_model_{};
  std::vector<SpatiotemporalUnion::Stage> stages_{};

  std::vector<SpatiotemporalUnion::OptVariables> optimal_solution_{};
  std::vector<SpatiotemporalUnion::LineModelMatrix> line_model_vector_{};
  SpatiotemporalUnion::CostMatrix cost_{};
  SpatiotemporalUnion::BoxConstraints box_constraints_{};
  std::vector<SpatiotemporalUnion::PolytopicConstraints> constraints_{};
  std::shared_ptr<SpatiotemporalUnionHpipmSolver> mpc_solver_{};
};

}  // namespace planning
}  // namespace neodrive
