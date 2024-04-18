#include "spatiotemporal_union_model.h"

namespace neodrive {
namespace planning {

SpatiotemporalUnionModel::SpatiotemporalUnionModel(
    const std::string &name, const std::vector<double> &flytime,
    const SpatiotemporalUnion::TunnelInfos &TunnelInfos,
    const double speed_limit) {
  name_ = name;
  flytime_ = flytime;
  TunnelInfos_ = TunnelInfos;
  speed_limit_ = speed_limit;

  for (size_t i = 0; i <= flytime_.size(); i++) {
    SpatiotemporalUnion::LineModelMatrix test;
    test.A = SpatiotemporalUnion::A::Zero();
    test.B = SpatiotemporalUnion::B::Zero();
    test.g = SpatiotemporalUnion::g::Zero();
    line_model_matrix_.push_back(test);

    SpatiotemporalUnion::PolytopicConstraints poly_stage_;
    poly_stage_.C = SpatiotemporalUnion::C::Zero();
    poly_stage_.D = SpatiotemporalUnion::D::Zero();
    poly_stage_.dl = SpatiotemporalUnion::dl::Zero();
    poly_stage_.du = SpatiotemporalUnion::du::Zero();
    polytopic_constraints_.push_back(poly_stage_);
  }
  cost_matrix_.Q = SpatiotemporalUnion::Q::Zero();
  cost_matrix_.R = SpatiotemporalUnion::R::Zero();
  cost_matrix_.S = SpatiotemporalUnion::S::Zero();
  cost_matrix_.q = SpatiotemporalUnion::q::Zero();
  cost_matrix_.r = SpatiotemporalUnion::r::Zero();
  cost_matrix_.Z = SpatiotemporalUnion::Z::Zero();
  cost_matrix_.z = SpatiotemporalUnion::z::Zero();

  box_constraints_.uu = SpatiotemporalUnion::Bounds_u::Ones();
  box_constraints_.lu = SpatiotemporalUnion::Bounds_u::Ones();
  box_constraints_.lx = SpatiotemporalUnion::Bounds_x::Ones();
  box_constraints_.ux = SpatiotemporalUnion::Bounds_x::Ones();
  box_constraints_.us = SpatiotemporalUnion::Bounds_s::Zero();
  box_constraints_.ls = SpatiotemporalUnion::Bounds_s::Zero();
}

bool SpatiotemporalUnionModel::Process() {
  // line model
  SpatiotemporalUnionConfig::SpatiotemporalUnionModelConfig config_;
  for (size_t i = 0; i < flytime_.size(); ++i) {
    double h = flytime_[i];

    line_model_matrix_[i].A(0, 0) = 1;
    line_model_matrix_[i].A(1, 1) = 1;
    line_model_matrix_[i].A(2, 2) = 1;
    line_model_matrix_[i].A(3, 3) = 1;

    line_model_matrix_[i].A(0, 1) = h;
    line_model_matrix_[i].A(0, 2) = 0.5 * h * h;
    line_model_matrix_[i].A(0, 3) = h * h * h / 6;
    line_model_matrix_[i].A(1, 2) = h;
    line_model_matrix_[i].A(1, 3) = h * h / 2;
    line_model_matrix_[i].A(2, 3) = h;
    line_model_matrix_[i].A(4, 4) = 1;
    line_model_matrix_[i].A(5, 5) = 1;
    line_model_matrix_[i].A(6, 6) = 1;
    line_model_matrix_[i].A(7, 7) = 1;
    line_model_matrix_[i].A(4, 5) = h;
    line_model_matrix_[i].A(4, 6) = 0.5 * h * h;
    line_model_matrix_[i].A(4, 7) = h * h * h / 6;
    line_model_matrix_[i].A(5, 6) = h;
    line_model_matrix_[i].A(5, 7) = h * h / 2;
    line_model_matrix_[i].A(6, 7) = h;

    line_model_matrix_[i].B(0, 0) = h * h * h * h / 24;
    line_model_matrix_[i].B(1, 0) = h * h * h / 6;
    line_model_matrix_[i].B(2, 0) = h * h / 2;
    line_model_matrix_[i].B(3, 0) = h;
    line_model_matrix_[i].B(4, 1) = h * h * h * h / 24;
    line_model_matrix_[i].B(5, 1) = h * h * h / 6;
    line_model_matrix_[i].B(6, 1) = h * h / 2;
    line_model_matrix_[i].B(7, 1) = h;

    // DEFINE Fai,HM-1,t
    Eigen::MatrixXd Fai;
    Fai = Eigen::MatrixXd::Zero(SpatiotemporalUnion::npc,
                                (SpatiotemporalUnion::nx + 2));
    Eigen::MatrixXd HM((SpatiotemporalUnion::nx / 2 + 1),
                       (SpatiotemporalUnion::nx / 2 + 1));
    Eigen::MatrixXd HM_a((SpatiotemporalUnion::nx / 2 + 1),
                         (SpatiotemporalUnion::nx / 2 + 1));
    Eigen::MatrixXd HM_b((SpatiotemporalUnion::nx / 2 + 1),
                         (SpatiotemporalUnion::nx / 2 + 1));
    Eigen::MatrixXd HM_c((SpatiotemporalUnion::nx / 2 + 1),
                         (SpatiotemporalUnion::nx / 2 + 1));
    Eigen::MatrixXd Tao((SpatiotemporalUnion::nx / 2 + 1),
                        (SpatiotemporalUnion::nx / 2));

    Fai.row(0) << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Fai.row(1) << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
    Fai.row(2) << 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
    Fai.row(3) << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
    Fai.row(4) << 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
    Fai.row(5) << 4. / h, -4. / h, 0, 0, 0, 0, 0, 0, 0, 0;
    Fai.row(6) << 0, 4. / h, -4. / h, 0, 0, 0, 0, 0, 0, 0;
    Fai.row(7) << 0, 0, 4. / h, -4. / h, 0, 0, 0, 0, 0, 0;
    Fai.row(8) << 0, 0, 0, 4. / h, -4. / h, 0, 0, 0, 0, 0;

    Fai.row(9) << 12. / (h * h), -24. / (h * h), 12. / (h * h), 0, 0, 0, 0, 0,
        0, 0;
    Fai.row(10) << 0, 12. / (h * h), -24. / (h * h), 12. / (h * h), 0, 0, 0, 0,
        0, 0;
    Fai.row(11) << 0, 0, 12. / (h * h), -24. / (h * h), 12. / (h * h), 0, 0, 0,
        0, 0;
    Fai.row(12) << 24. / (h * h * h), -72. / (h * h * h), 72. / (h * h * h),
        -24. / (h * h * h), 0, 0, 0, 0, 0, 0;
    Fai.row(13) << 0, 24. / (h * h * h), -72. / (h * h * h), 72. / (h * h * h),
        -24. / (h * h * h), 0, 0, 0, 0, 0;

    Fai.row(14) << 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
    Fai.row(15) << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    Fai.row(16) << 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
    Fai.row(17) << 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
    Fai.row(18) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    Fai.row(19) << 0, 0, 0, 0, 0, 4. / h, -4. / h, 0, 0, 0;
    Fai.row(20) << 0, 0, 0, 0, 0, 0, 4. / h, -4. / h, 0, 0;
    Fai.row(21) << 0, 0, 0, 0, 0, 0, 0, 4. / h, -4. / h, 0;
    Fai.row(22) << 0, 0, 0, 0, 0, 0, 0, 0, 4. / h, -4. / h;

    Fai.row(23) << 0, 0, 0, 0, 0, 12. / (h * h), -24. / (h * h), 12. / (h * h),
        0, 0;
    Fai.row(24) << 0, 0, 0, 0, 0, 0, 12. / (h * h), -24. / (h * h),
        12. / (h * h), 0;
    Fai.row(25) << 0, 0, 0, 0, 0, 0, 0, 12. / (h * h), -24. / (h * h),
        12. / (h * h);
    Fai.row(26) << 0, 0, 0, 0, 0, 24. / (h * h * h), -72. / (h * h * h),
        72. / (h * h * h), -24. / (h * h * h), 0;
    Fai.row(27) << 0, 0, 0, 0, 0, 0, 24. / (h * h * h), -72. / (h * h * h),
        72. / (h * h * h), -24. / (h * h * h);

    HM_a.row(0) << 1, 0, 0, 0, 0;
    HM_a.row(1) << 0, 1, 0, 0, 0;
    HM_a.row(2) << 0, 0, 2, 0, 0;
    HM_a.row(3) << 0, 0, 0, 6, 0;
    HM_a.row(4) << 0, 0, 0, 0, 24;

    HM_b.row(0) << 1, 0, 0, 0, 0;
    HM_b.row(1) << 0, 1. / h, 0, 0, 0;
    HM_b.row(2) << 0, 0, 1. / (h * h), 0, 0;
    HM_b.row(3) << 0, 0, 0, 1. / (h * h * h), 0;
    HM_b.row(4) << 0, 0, 0, 0, 1. / (h * h * h * h);

    HM_c.row(0) << 1, 0, 0, 0, 0;
    HM_c.row(1) << -4, 4, 0, 0, 0;
    HM_c.row(2) << 6, -12, 6, 0, 0;
    HM_c.row(3) << -4, 12, -12, 4, 0;
    HM_c.row(4) << 1, -4, 6, -4, 1;

    Tao.row(0) << 1, 0, 0, 0;
    Tao.row(1) << 0, 1, 0, 0;
    Tao.row(2) << 0, 0, 1, 0;
    Tao.row(3) << 0, 0, 0, 1;
    Tao.row(4) << 0, 0, 0, 0;
    HM = HM_a * HM_b * HM_c;
    Eigen::MatrixXd HMM((SpatiotemporalUnion::nx / 2 + 1),
                        (SpatiotemporalUnion::nx / 2 + 1));
    HMM = HM;
    HM = HM.inverse();

    Eigen::MatrixXd para_a_;
    para_a_ = Eigen::MatrixXd::Zero((SpatiotemporalUnion::nx / 2 + 1),
                                    (SpatiotemporalUnion::nx / 2));
    para_a_ = HM * Tao;
    Eigen::MatrixXd para_b_;
    para_b_ = Eigen::MatrixXd::Zero((SpatiotemporalUnion::nx + 2),
                                    SpatiotemporalUnion::nx);
    para_b_.row(0) << para_a_(0, 0), para_a_(0, 1), para_a_(0, 2),
        para_a_(0, 3), 0, 0, 0, 0;
    para_b_.row(1) << para_a_(1, 0), para_a_(1, 1), para_a_(1, 2),
        para_a_(1, 3), 0, 0, 0, 0;
    para_b_.row(2) << para_a_(2, 0), para_a_(2, 1), para_a_(2, 2),
        para_a_(2, 3), 0, 0, 0, 0;
    para_b_.row(3) << para_a_(3, 0), para_a_(3, 1), para_a_(3, 2),
        para_a_(3, 3), 0, 0, 0, 0;
    para_b_.row(4) << para_a_(4, 0), para_a_(4, 1), para_a_(4, 2),
        para_a_(4, 3), 0, 0, 0, 0;
    para_b_.row(5) << 0, 0, 0, 0, para_a_(0, 0), para_a_(0, 1), para_a_(0, 2),
        para_a_(0, 3);
    para_b_.row(6) << 0, 0, 0, 0, para_a_(1, 0), para_a_(1, 1), para_a_(1, 2),
        para_a_(1, 3);
    para_b_.row(7) << 0, 0, 0, 0, para_a_(2, 0), para_a_(2, 1), para_a_(2, 2),
        para_a_(2, 3);
    para_b_.row(8) << 0, 0, 0, 0, para_a_(3, 0), para_a_(3, 1), para_a_(3, 2),
        para_a_(3, 3);
    para_b_.row(9) << 0, 0, 0, 0, para_a_(4, 0), para_a_(4, 1), para_a_(4, 2),
        para_a_(4, 3);

    polytopic_constraints_[i].C = Fai * para_b_;

    Eigen::MatrixXd e((SpatiotemporalUnion::nx / 2 + 1), 1);
    e.row(0) << 0;
    e.row(1) << 0;
    e.row(2) << 0;
    e.row(3) << 0;
    e.row(4) << 1;
    Eigen::MatrixXd para_c_;
    para_c_ = Eigen::MatrixXd::Zero((SpatiotemporalUnion::nx / 2 + 1), 1);
    para_c_ = HM * e;
    Eigen::MatrixXd para_d_;
    para_d_ = Eigen::MatrixXd::Zero((SpatiotemporalUnion::nx + 2), 2);

    para_d_.row(0) << para_c_(0, 0), 0;
    para_d_.row(1) << para_c_(1, 0), 0;
    para_d_.row(2) << para_c_(2, 0), 0;
    para_d_.row(3) << para_c_(3, 0), 0;
    para_d_.row(4) << para_c_(4, 0), 0;

    para_d_.row(5) << 0, para_c_(0, 0);
    para_d_.row(6) << 0, para_c_(1, 0);
    para_d_.row(7) << 0, para_c_(2, 0);
    para_d_.row(8) << 0, para_c_(3, 0);
    para_d_.row(9) << 0, para_c_(4, 0);

    polytopic_constraints_[i].D = Fai * para_d_;

    polytopic_constraints_[i].du << TunnelInfos_.s_boundary[i].second,
        TunnelInfos_.s_boundary[i].second, TunnelInfos_.s_boundary[i].second,
        TunnelInfos_.s_boundary[i].second, TunnelInfos_.s_boundary[i].second,
        config_.MaxV(), config_.MaxV(), config_.MaxV(), config_.MaxV(),
        config_.MaxA(), config_.MaxA(), config_.MaxA(), config_.MaxJ(),
        config_.MaxJ(), TunnelInfos_.l_boundary[i].second,
        TunnelInfos_.l_boundary[i].second, TunnelInfos_.l_boundary[i].second,
        TunnelInfos_.l_boundary[i].second, TunnelInfos_.l_boundary[i].second,
        config_.MaxV(), config_.MaxV(), config_.MaxV(), config_.MaxV(),
        config_.MaxA(), config_.MaxA(), config_.MaxA(), config_.MaxJ(),
        config_.MaxJ();

    polytopic_constraints_[i].dl << TunnelInfos_.s_boundary[i].first,
        TunnelInfos_.s_boundary[i].first, TunnelInfos_.s_boundary[i].first,
        TunnelInfos_.s_boundary[i].first, TunnelInfos_.s_boundary[i].first,
        config_.MinV(), config_.MinV(), config_.MinV(), config_.MinV(),
        config_.MinA(), config_.MinA(), config_.MinA(), config_.MinJ(),
        config_.MinJ(), TunnelInfos_.l_boundary[i].first,
        TunnelInfos_.l_boundary[i].first, TunnelInfos_.l_boundary[i].first,
        TunnelInfos_.l_boundary[i].first, TunnelInfos_.l_boundary[i].first,
        config_.MinV(), config_.MinV(), config_.MinV(), config_.MinV(),
        config_.MinA(), config_.MinA(), config_.MinA(), config_.MinJ(),
        config_.MinJ();
  }
  // 最后一个stage随便填充
  line_model_matrix_[flytime_.size()].A =
      line_model_matrix_[flytime_.size() - 1].A;
  line_model_matrix_[flytime_.size()].B =
      line_model_matrix_[flytime_.size() - 1].B;

  polytopic_constraints_[flytime_.size()].du =
      1000 * Eigen::MatrixXd::Ones(SpatiotemporalUnion::npc, 1);
  polytopic_constraints_[flytime_.size()].dl =
      -1000 * Eigen::MatrixXd::Ones(SpatiotemporalUnion::npc, 1);

  polytopic_constraints_[flytime_.size()].C = SpatiotemporalUnion::C::Zero();
  polytopic_constraints_[flytime_.size()].D = SpatiotemporalUnion::D::Zero();

  cost_matrix_.Q(2, 2) = config_.WeightA();
  cost_matrix_.Q(3, 3) = config_.Weightj();
  cost_matrix_.Q(6, 6) = config_.WeightA();
  cost_matrix_.Q(7, 7) = config_.Weightj();
  cost_matrix_.R(0, 0) = 1;
  cost_matrix_.R(1, 1) = 1;

  box_constraints_.uu << config_.BoundU(), config_.BoundU();
  box_constraints_.lu << -config_.BoundU(), -config_.BoundU();
  box_constraints_.ux << config_.BoundP(), speed_limit_, config_.BoundA(),
      config_.BoundJ(), config_.BoundP(), config_.BoundV(), config_.BoundA(),
      config_.BoundJ();
  box_constraints_.lx << -config_.BoundP(), 0, -config_.BoundA(),
      -config_.BoundJ(), -config_.BoundP(), -config_.BoundV(),
      -config_.BoundA(), -config_.BoundJ();
  LOG_INFO("finish model establisheds.");

  return true;
}

}  // namespace planning
}  // namespace neodrive
