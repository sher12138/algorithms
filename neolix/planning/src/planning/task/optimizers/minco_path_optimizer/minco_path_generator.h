#pragma once

#include <Eigen/Eigen>
#include <fstream>
#include <string>
#include <vector>

#include "minco_generator.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/reference_line/reference_line.h"

namespace neodrive {
namespace planning {

class MincoPathGenerator {
 public:
  explicit MincoPathGenerator(const std::string& name) : name_(name) {}

  ~MincoPathGenerator() = default;

  ErrorCode Optimize(const ReferenceLinePtr& reference_line,
                     const InsidePlannerData& inside_data,
                     OutsidePlannerData* const outside_data);

 private:
  bool Init(const ReferenceLinePtr& reference_line,
            const InsidePlannerData& inside_data,
            OutsidePlannerData* const outside_data);

  bool RunMinco(const ReferenceLinePtr& reference_line,
                const InsidePlannerData& inside_data,
                OutsidePlannerData* const outside_data);

  bool FillPathData(const ReferenceLinePtr ref_ptr,
                    const minco::Trajectory& minco_traj,
                    PathData* const path_data);

 private:
  std::string name_{""};

  Eigen::MatrixXd ini_state_;
  Eigen::MatrixXd fini_state_;
};

}  // namespace planning
}  // namespace neodrive
