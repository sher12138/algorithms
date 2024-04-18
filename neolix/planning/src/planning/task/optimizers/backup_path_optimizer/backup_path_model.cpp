#include "backup_path_model.h"

#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

BackupPathModel::BackupPathModel(const std::string& name,
                                 const BackupPath::State& init_state,
                                 const BackupPath::Control& init_control,
                                 const double delta_s)
    : name_(name),
      init_state_(init_state),
      init_control_(init_control),
      delta_s_(delta_s) {
  line_model_matrix_.A = BackupPath::A::Zero();
  line_model_matrix_.B = BackupPath::B::Zero();
  line_model_matrix_.g = BackupPath::g::Zero();
  line_model_matrix_.A.row(0) << 1, delta_s_, 1 / 2. * std::pow(delta_s_, 2);
  line_model_matrix_.A.row(1) << 0, 1, delta_s_;
  line_model_matrix_.A.row(2) << 0, 0, 1;
  line_model_matrix_.B << 1 / 6. * std::pow(delta_s_, 3),
      1 / 2. * std::pow(delta_s_, 2), delta_s_;

  polytopic_constraint_.C = BackupPath::C::Zero();
  polytopic_constraint_.D = BackupPath::D::Zero();
  polytopic_constraint_.dl = BackupPath::d::Zero();
  polytopic_constraint_.du = BackupPath::d::Zero();
}

}  // namespace planning
}  // namespace neodrive
