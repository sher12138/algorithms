#include "minco_generator.h"

#include <cyber/time/time.h>

#include "src/planning/common/math/angle.h"
#include "src/planning/common/visualizer_event/visualizer_event.h"
#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {
namespace {

constexpr double smooth_eps = 1e-4;

void PositiveSmoothedL1(const double x, double& f, double& df) {
  const double pe = smooth_eps;
  const double half = 0.5 * pe;
  const double f3c = 1.0 / (pe * pe);
  const double f4c = -0.5 * f3c / pe;
  const double d2c = 3.0 * f3c;
  const double d3c = 4.0 * f4c;
  if (x < pe) {
    f = (f4c * x + f3c) * x * x * x;
    df = (d3c * x + d2c) * x * x;
  } else {
    f = x - half;
    df = 1.0;
  }
}

void PositiveSmoothedLn(const double& x, const int& n, double& f, double& df) {
  f = std::pow(x, n);
  df = n * std::pow(x, n - 1);
}

template <typename EIGENVEC>
void VirtualT2RealT(const EIGENVEC& VT, Eigen::VectorXd& RT) {
  for (int i = 0; i < VT.size(); ++i) {
    RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                        : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
  }
}

template <typename EIGENVEC>
void RealT2VirtualT(const Eigen::VectorXd& RT, EIGENVEC& VT) {
  for (int i = 0; i < RT.size(); ++i) {
    VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                        : (1.0 - sqrt(2.0 / RT(i) - 1.0));
  }
}

double VirtualTimeGradCost(const double time_weight, const double real_time,
                           const double virtual_time,
                           const double grad_real_time,
                           double& grad_virtual_time) {
  double grad_vt_2_rt;
  if (virtual_time > 0) {
    grad_vt_2_rt = virtual_time + 1.0;
  } else {
    double den_sqrt = (0.5 * virtual_time - 1.0) * virtual_time + 1.0;
    grad_vt_2_rt = (1.0 - virtual_time) / (den_sqrt * den_sqrt);
  }

  grad_virtual_time = (grad_real_time + time_weight) * grad_vt_2_rt;

  return real_time * time_weight;
}

}  // namespace

MincoGenerator::MincoGenerator(
    const Eigen::MatrixXd& ini_state, const Eigen::MatrixXd& fin_state,
    const Eigen::MatrixXd& inner_pts, const Eigen::VectorXd& duration_time,
    const std::vector<Eigen::Vector3d>& state_list,
    const std::vector<Eigen::MatrixXd>& origin_h_polys,
    const std::vector<Eigen::MatrixXd>& shrink_h_polys,
    const std::vector<std::array<double, 2>>& origin_freespace_data,
    const std::vector<std::array<double, 2>>& shrink_freespace_data,
    const int traj_resolution, const int destraj_resolution,
    OccupyMap&& origin_om, OccupyMap&& shrink_om)
    : ini_state_(ini_state),
      fin_state_(fin_state),
      inner_pts_(inner_pts),
      duration_time_(duration_time),
      state_list_(state_list),
      origin_h_polys_(origin_h_polys),
      shrink_h_polys_(shrink_h_polys),
      origin_freespace_data_(origin_freespace_data),
      shrink_freespace_data_(shrink_freespace_data),
      traj_resolution_(traj_resolution),
      destraj_resolution_(destraj_resolution),
      origin_om_(origin_om),
      shrink_om_(shrink_om) {}

bool MincoGenerator::OptimizeTrajectory() {
  if (!BuildOptProblem()) {
    return false;
  }

  return Solve();
}

bool MincoGenerator::BuildOptProblem() {
  if (inner_pts_.cols() == 0 || duration_time_.size() != 1) return false;

  piece_nums_ = inner_pts_.cols() + 1;

  /// 2 * inner_pts.size() +  1 (time) + fin_state
  variable_num_ = 2 * (piece_nums_ - 1) + 1;
  opt_x_.resize(variable_num_);
  opt_x_.setZero();
  min_jerk_opt_.reset(ini_state_, fin_state_, piece_nums_);

  int offset = 0;
  memcpy(opt_x_.data() + offset, inner_pts_.data(),
         inner_pts_.size() * sizeof(opt_x_[0]));
  offset += inner_pts_.size();
  Eigen::Map<Eigen::VectorXd> Vt(opt_x_.data() + offset, 1);
  RealT2VirtualT(duration_time_, Vt);
  offset += 1;

  if ((origin_h_polys_.size() !=
       (piece_nums_ - 2) * traj_resolution_ + 2 * destraj_resolution_ - 1) ||
      (origin_h_polys_.size() != shrink_h_polys_.size())) {
    LOG_ERROR("h_polys size error.");
    return false;
  }

  const auto& config = config::PlanningConfig::Instance()
                           ->planning_research_config()
                           .minco_path_optimizer_config;
  wei_obs_ = config.wei_obs;
  wei_shrink_ = config.wei_shrink;
  wei_match_ = config.wei_match;

  wei_vel_ = config.wei_vel;
  wei_acc_ = config.wei_acc;
  wei_latacc_ = config.wei_latacc;
  wei_cur_ = config.wei_cur;
  wei_phidot_ = config.wei_phidot;
  wei_time_ = config.wei_time;

  max_vel_ = config.max_vel;
  max_acc_ = config.max_acc;
  max_latacc_ = config.max_latacc;
  max_cur_ = config.max_cur;
  max_phidot_ = config.max_phidot;

  // /// Occupy map for collision check
  // origin_om_ = std::move(
  //     OccupyMap{{.grid_step = config.grid_step}, origin_freespace_data_,
  //     {}});
  // // origin_om_.VisMap();
  // shrink_om_ = std::move(
  //     OccupyMap{{.grid_step = config.grid_step}, shrink_freespace_data_,
  //     {}});

  /// Vertexs of the ego car in the body frame
  const auto& veh_params = VehicleParam::Instance();
  const auto lateral_safe_dis = config.lateral_safe_dis;
  const auto longi_safe_dis = config.longi_safe_dis;
  const auto interval = config.virtual_interval;
  veh_le_.clear();
  double start_x = -veh_params->back_edge_to_center() - 0.2;
  double end_x = veh_params->front_edge_to_center() + longi_safe_dis;
  double start_y = -veh_params->width() / 2.0 - lateral_safe_dis;
  double end_y = veh_params->width() / 2.0 + lateral_safe_dis;
  Eigen::Vector2d le;
  for (double x = start_x; x <= end_x; x += interval) {
    le << x, start_y;
    veh_le_.push_back(le);
    le << x, end_y;
    veh_le_.push_back(le);
  }
  if (!veh_le_.empty() && veh_le_.back()[0] < end_x) {
    le << end_x, start_y;
    veh_le_.push_back(le);
    le << end_x, end_y;
    veh_le_.push_back(le);
  }
  for (double y = start_y + interval; y <= end_y; y += interval) {
    le << start_x, y;
    veh_le_.push_back(le);
    le << end_x, y;
    veh_le_.push_back(le);
  }

  return true;
}

bool MincoGenerator::Solve() {
  auto time = cyber::Time::Now().ToSecond();

  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs_params.mem_size = 256;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 1.0e-9;
  lbfgs_params.min_step = 1.0e-20;
  lbfgs_params.delta = 0.0001;
  lbfgs_params.max_linesearch = 30;

  bool flag_success = false;
  auto result = lbfgs::lbfgs_optimize(opt_x_, final_cost_,
                                      MincoGenerator::CostFunctionCallback,
                                      nullptr, nullptr, this, lbfgs_params);

  if (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_CANCELED ||
      result == lbfgs::LBFGS_STOP ||
      result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
      result == lbfgs::LBFGSERR_MAXIMUMITERATION) {
    flag_success = true;
  } else {
    LOG_WARN("lbfgs solver error:{} - {} !!! skip this planning.", result,
             lbfgs::lbfgs_strerror(result));
  }
  LOG_INFO("Minco success: {}, {}, final_cost: {:.3f}, iter: {}, time: {}",
           flag_success, lbfgs::lbfgs_strerror(result), final_cost_, iter_cnt_,
           cyber::Time::Now().ToSecond() - time);

  auto vis = [](const std::vector<std::vector<math::AD2>>& pts,
                const std::string& name) {
    if (!FLAGS_planning_enable_vis_event) return;

    auto event = vis::EventSender::Instance()->GetEvent(name);
    event->set_type(visualizer::Event::k3D);
    event->add_attribute(visualizer::Event::kOdom);

    auto set_pt = [](auto ans, auto& p) {
      ans->set_x(p.x());
      ans->set_y(p.y());
      ans->set_z(0);
    };

    Vec2d pt;
    for (const auto& pose : pts) {
      for (const auto& p : pose) {
        pt.set_x(p[0]), pt.set_y(p[1]);

        auto sphere = event->mutable_sphere()->Add();
        set_pt(sphere->mutable_center(), pt);
        sphere->set_radius(0.01);
      }
    }
  };

  vis(car_pts_, "car_pts");

  return flag_success;
}

double MincoGenerator::CostFunctionCallback(void* func_data,
                                            const Eigen::VectorXd& x,
                                            Eigen::VectorXd& grad) {
  MincoGenerator* opt = reinterpret_cast<MincoGenerator*>(func_data);

  int offset = 0;
  /// pts and its grad
  Eigen::Map<const Eigen::MatrixXd> P(x.data() + offset, 2,
                                      opt->piece_nums_ - 1);
  Eigen::Map<Eigen::MatrixXd> gradP(grad.data() + offset, 2,
                                    opt->piece_nums_ - 1);
  gradP.setZero();
  offset += 2 * (opt->piece_nums_ - 1);

  /// t -> virtual time, T -> real time
  Eigen::Map<const Eigen::VectorXd> t(x.data() + offset, 1);
  Eigen::Map<Eigen::VectorXd> gradt(grad.data() + offset, 1);
  Eigen::VectorXd T(1);
  VirtualT2RealT(t, T);
  offset += 1;

  Eigen::VectorXd arrayT(opt->piece_nums_);
  arrayT.setConstant(T[0] / opt->piece_nums_);
  Eigen::VectorXd arraygradT(opt->piece_nums_);
  arraygradT.setZero();

  /// generate
  opt->min_jerk_opt_.generate(P, arrayT);

  /// smooth cost
  double smooth_cost{0.};
  opt->min_jerk_opt_.initGradCost(arraygradT, smooth_cost);

  /// add pva grad cost to CT
  double penalty_cost = opt->AddPVAGradCost2CT(arraygradT, opt->min_jerk_opt_);

  /// time cost
  opt->min_jerk_opt_.getGrad2TP(arraygradT, gradP);
  double gradsumT{0.}, gradsumt{0.};
  gradsumT = arraygradT.sum() / arraygradT.size();

  double time_cost{0.};
  time_cost =
      VirtualTimeGradCost(opt->wei_time_, T[0], t[0], gradsumT, gradsumt);

  gradt << gradsumt;
  // gradP = gradP;
  // arraygradT = arraygradT;

  opt->iter_cnt_++;

  return smooth_cost + penalty_cost + time_cost;
}

double MincoGenerator::AddPVAGradCost2CT(Eigen::VectorXd& gdTs,
                                         minco::MinJerkOpt& traj) {
  int N = gdTs.size();
  minco::Trajectory path = traj.getTraj(1);

  double collision_cost{0.}, shrink_cost{0.}, match_cost{0.}, vel_cost{0.},
      acc_cost{0.}, latacc_cost{0.}, cur_cost{0.}, phidot_cost{0.};

  double s1{0.}, s2{0.}, s3{0.}, s4{0.}, s5{0.};
  Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
  beta0.setZero(), beta1.setZero(), beta2.setZero();
  beta3.setZero(), beta4.setZero();
  Eigen::Vector2d sigma, dsigma, ddsigma, dddsigma, ddddsigma;
  sigma.setZero(), dsigma.setZero(), ddsigma.setZero();
  dddsigma.setZero(), ddddsigma.setZero();

  double z_h0{0.}, z_h1{0.}, z_h2{0.}, z_h3{0.}, z_h4{0.};
  double n1{0.}, n2{0.}, n3{0.}, n4{0.}, n5{0.}, n6{0.};
  double z1{0.}, z2{0.}, z3{0.};

  double vel2_reci{0.}, vel2_reci_e{0.}, vel3_2_reci_e{0.};
  double acc2{0.}, cur2{0.}, cur{0.}, phi_dot{0.}, latacc2{0.};

  double violaPos{0.}, violaPosShrink{0.}, violaMatchPos{0.}, violaVel{0.},
      violaAcc{0.}, violaLatAcc{0.}, violaCur{0.}, violaCurL{0.}, violaCurR{0.},
      violaPhidotL{0.}, violaPhidotR{0.};
  double violaPosPena{0.}, violaPosShrinkPena{0.}, violaMatchPosPena{0.},
      violaVelPena{0.}, violaAccPena{0.}, violaLatAccPena{0.}, violaCurPena{0.},
      violaCurPenaL{0.}, violaCurPenaR{0.}, violaPhidotPenaL{0.},
      violaPhidotPenaR{0.};
  double violaPosPenaD{0.}, violaPosShrinkPenaD{0.}, violaMatchPosPenaD{0.},
      violaVelPenaD{0.}, violaAccPenaD{0.}, violaLatAccPenaD{0.},
      violaCurPenaD{0.}, violaCurPenaDL{0.}, violaCurPenaDR{0.},
      violaPhidotPenaDL{0.}, violaPhidotPenaDR{0.};

  Eigen::Matrix<double, 6, 2> gradViolaPc, gradViolaPSc, gradViolaMPc,
      gradViolaVc, gradViolaAc, gradViolaLatAc, gradViolaKc, gradViolaKLc,
      gradViolaKRc, gradViolaPhidotLc, gradViolaPhidotRc;
  gradViolaPc.setZero(), gradViolaPSc.setZero(), gradViolaMPc.setZero();
  gradViolaVc.setZero(), gradViolaAc.setZero(), gradViolaLatAc.setZero();
  gradViolaKc.setZero(), gradViolaKLc.setZero(), gradViolaKRc.setZero();
  gradViolaPhidotLc.setZero(), gradViolaPhidotRc.setZero();
  double gradViolaPt{0.}, gradViolaPSt{0.}, gradViolaMPt{0.}, gradViolaVt{0.},
      gradViolaAt{0.}, gradViolaLatAt{0.}, gradViolaKt{0.}, gradViolaKLt{0.},
      gradViolaKRt{0.}, gradViolaPhidotLt{0.}, gradViolaPhidotRt{0.};

  double phidot_denominator{0.}, phidot_nominator{0.};

  Eigen::Matrix<double, 2, 2> B_h;
  B_h << 0, -1, 1, 0;
  double L = VehicleParam::Instance()->wheel_base();

  Eigen::Matrix2d ego_R, help_R;
  Eigen::Vector2d outerNormal;
  ego_R.setZero(), help_R.setZero(), outerNormal.setZero();

  car_pts_.clear();
  std::vector<math::AD2> pts;

  double step{0.}, alpha{0.}, omg{0.}, t{0.};
  int K = 0, point_id = -1;
  for (int i = 0; i < N; ++i) {
    if (i == 0 || i + 1 == N) {
      K = destraj_resolution_ - 1;
    } else {
      K = traj_resolution_ - 1;
    }
    const Eigen::Matrix<double, 6, 2>& c = traj.get_b().block<6, 2>(i * 6, 0);
    step = traj.get_T1()(i) / K;  // T_i /k
    s1 = 0.0;

    for (int j = 0; j <= K; ++j) {
      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
      beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
      beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * s1;

      sigma = c.transpose() * beta0;
      dsigma = c.transpose() * beta1;
      ddsigma = c.transpose() * beta2;
      dddsigma = c.transpose() * beta3;
      ddddsigma = c.transpose() * beta4;

      point_id++;
      s1 += step;
      alpha = 1.0 / K * j;
      omg = 1.0;

      z_h0 = dsigma.norm();
      z_h1 = ddsigma.transpose() * dsigma;
      z_h2 = dddsigma.transpose() * dsigma;
      z_h3 = ddsigma.transpose() * B_h * dsigma;

      n1 = z_h0;
      n2 = n1 * n1;
      n3 = n2 * n1;
      n4 = n2 * n2;
      n5 = n3 * n2;
      n6 = n3 * n3;

      z1 = dddsigma.transpose() * B_h * dsigma;
      z2 = ddsigma.transpose() * B_h * dsigma;
      z3 = dsigma.transpose() * ddsigma;

      if (z_h0 < 1e-4 || (j == 0 && i == 0) || (i == N - 1 && j == K)) {
        continue;
      }

      vel2_reci = 1.0 / (z_h0 * z_h0);
      vel2_reci_e = 1.0 / (z_h0 * z_h0 + kMathEpsilon);
      vel3_2_reci_e = vel2_reci_e * sqrt(vel2_reci_e);
      z_h0 = 1.0 / z_h0;

      z_h4 = z_h1 * vel2_reci;
      violaVel = 1.0 / vel2_reci - max_vel_ * max_vel_;
      acc2 = z_h1 * z_h1 * vel2_reci;
      latacc2 = z_h3 * z_h3 * vel2_reci;
      cur2 = z_h3 * z_h3 * (vel2_reci_e * vel2_reci_e * vel2_reci_e);
      cur = z_h3 * vel3_2_reci_e;
      violaAcc = acc2 - max_acc_ * max_acc_;
      violaLatAcc = latacc2 - max_latacc_ * max_latacc_;

      phidot_denominator = n6 + L * L * z2 * z2;
      phidot_nominator = L * (n3 * z1 - 3 * z2 * z3 * n1);
      phi_dot = phidot_nominator / phidot_denominator;  // S/M

      violaCur = cur2 - max_cur_ * max_cur_;
      violaCurL = cur - max_cur_;
      violaCurR = -cur - max_cur_;
      violaPhidotL = phi_dot - max_phidot_;
      violaPhidotR = -phi_dot - max_phidot_;

      violaMatchPos = std::hypot(sigma(0) - state_list_[point_id](0),
                                 sigma(1) - state_list_[point_id](1));

      ego_R << dsigma(0), -dsigma(1), dsigma(1), dsigma(0);
      ego_R = ego_R * z_h0;

      Eigen::Matrix2d temp_a, temp_v;
      temp_a << ddsigma(0), -ddsigma(1), ddsigma(1), ddsigma(0);
      temp_v << dsigma(0), -dsigma(1), dsigma(1), dsigma(0);
      Eigen::Matrix2d R_dot =
          (temp_a * z_h0 - temp_v * vel2_reci * z_h0 * z_h1);

      double angle = -normalize_angle(path.getAngle(t + s1));
      if (FLAGS_planning_enable_vis_event) pts.clear();
      for (std::size_t id = 0; id < veh_le_.size(); ++id) {
        auto le = veh_le_[id];

        double xx =
            le(0) * std::cos(angle) + le(1) * std::sin(angle) + sigma(0);
        double yy =
            -le(0) * std::sin(angle) + le(1) * std::cos(angle) + sigma(1);
        if (FLAGS_planning_enable_vis_event) pts.push_back({xx, yy});

        if (!origin_om_.IsPointOccupied({xx, yy})) {
          continue;
        }

        Eigen::Vector2d bpt = sigma + ego_R * le;
        Eigen::Matrix2d temp_l_Bl;
        temp_l_Bl << le(0), -le(1), le(1), le(0);

        int corr_k = origin_h_polys_[point_id].cols();
        for (int k = 0; k < corr_k; k++) {
          outerNormal = origin_h_polys_[point_id].col(k).head<2>();
          violaPos =
              outerNormal.dot(bpt - origin_h_polys_[point_id].col(k).tail<2>());

          if (violaPos > 0) {
            PositiveSmoothedL1(violaPos, violaPosPena, violaPosPenaD);
            gradViolaPc = beta0 * outerNormal.transpose() +
                          beta1 * outerNormal.transpose() *
                              (temp_l_Bl * z_h0 -
                               ego_R * le * dsigma.transpose() * vel2_reci);
            gradViolaPt =
                alpha * outerNormal.transpose() * (dsigma + R_dot * le);
            traj.get_gdC().block<6, 2>(i * 6, 0) +=
                omg * step * wei_obs_ * violaPosPenaD * gradViolaPc;
            gdTs(i) += omg * wei_obs_ *
                       (violaPosPenaD * gradViolaPt * step + violaPosPena / K);

            collision_cost += omg * step * wei_obs_ * violaPosPena;
          }
        }
      }
      if (FLAGS_planning_enable_vis_event) car_pts_.emplace_back(pts);

      /// shrink corridors
      if (shrink_om_.IsPointOccupied({sigma(0), sigma(1)})) {
        Eigen::Vector2d le;
        le << 0., 0.;
        Eigen::Vector2d bpt = sigma + ego_R * le;
        Eigen::Matrix2d temp_l_Bl;
        temp_l_Bl << le(0), -le(1), le(1), le(0);

        int corr_k = shrink_h_polys_[point_id].cols();
        for (int k = 0; k < corr_k; k++) {
          outerNormal = shrink_h_polys_[point_id].col(k).head<2>();
          violaPosShrink =
              outerNormal.dot(bpt - shrink_h_polys_[point_id].col(k).tail<2>());

          if (violaPosShrink > 0) {
            PositiveSmoothedL1(violaPosShrink, violaPosShrinkPena,
                               violaPosShrinkPenaD);
            gradViolaPSc = beta0 * outerNormal.transpose() +
                           beta1 * outerNormal.transpose() *
                               (temp_l_Bl * z_h0 -
                                ego_R * le * dsigma.transpose() * vel2_reci);
            gradViolaPSt =
                alpha * outerNormal.transpose() * (dsigma + R_dot * le);
            traj.get_gdC().block<6, 2>(i * 6, 0) +=
                omg * step * wei_shrink_ * violaPosShrinkPenaD * gradViolaPSc;
            gdTs(i) += omg * wei_shrink_ *
                       (violaPosShrinkPenaD * gradViolaPSt * step +
                        violaPosShrinkPena / K);

            shrink_cost += omg * step * wei_shrink_ * violaPosShrinkPena;
          }
        }
      }

      if (violaMatchPos > 0.0) {
        PositiveSmoothedL1(violaMatchPos, violaMatchPosPena,
                           violaMatchPosPenaD);
        Eigen::Vector2d vec;
        vec << (sigma(0) - state_list_[point_id](0)) / violaMatchPos,
            (sigma(1) - state_list_[point_id](1)) / violaMatchPos;
        gradViolaMPc = beta0 * vec.transpose();
        gradViolaMPt = alpha * vec.transpose() * dsigma;
        traj.get_gdC().block<6, 2>(i * 6, 0) +=
            omg * step * wei_match_ * violaMatchPosPenaD * gradViolaMPc;
        gdTs(i) +=
            omg * wei_match_ *
            (violaMatchPosPenaD * gradViolaMPt * step + violaMatchPosPena / K);

        match_cost += omg * step * wei_match_ * violaMatchPosPena;
      }
      if (violaVel > 0.0) {
        PositiveSmoothedL1(violaVel, violaVelPena, violaVelPenaD);
        gradViolaVc = 2.0 * beta1 * dsigma.transpose();  // 6*2
        gradViolaVt = 2.0 * alpha * z_h1;                // 1*1
        traj.get_gdC().block<6, 2>(i * 6, 0) +=
            omg * step * wei_vel_ * violaVelPenaD * gradViolaVc;
        gdTs(i) += omg * wei_vel_ *
                   (violaVelPenaD * gradViolaVt * step + violaVelPena / K);

        vel_cost += omg * step * wei_vel_ * violaVelPena;
      }
      if (violaAcc > 0.0) {
        PositiveSmoothedL1(violaAcc, violaAccPena, violaAccPenaD);
        gradViolaAc = 2.0 * beta1 *
                          (z_h4 * ddsigma.transpose() -
                           z_h4 * z_h4 * dsigma.transpose()) +
                      2.0 * beta2 * z_h4 * dsigma.transpose();  // 6*2
        gradViolaAt =
            2.0 * alpha *
            (z_h4 * (ddsigma.squaredNorm() + z_h2) - z_h4 * z_h4 * z_h1);
        traj.get_gdC().block<6, 2>(i * 6, 0) +=
            omg * step * wei_acc_ * violaAccPenaD * gradViolaAc;
        gdTs(i) += omg * wei_acc_ *
                   (violaAccPenaD * gradViolaAt * step + violaAccPena / K);

        acc_cost += omg * step * wei_acc_ * violaAccPena;
      }
      if (violaLatAcc > 0.0) {
        PositiveSmoothedL1(violaLatAcc, violaLatAccPena, violaLatAccPenaD);
        gradViolaLatAc =
            2.0 * beta1 *
                (z_h3 * vel2_reci * ddsigma.transpose() * B_h -
                 z_h3 * vel2_reci * z_h3 * vel2_reci * dsigma.transpose()) +
            2.0 * beta2 * z_h3 * vel2_reci * dsigma.transpose() *
                B_h.transpose();  // 6*2
        gradViolaLatAt =
            2.0 * alpha *
            (z_h3 * vel2_reci * z1 - z_h3 * vel2_reci * z_h3 * vel2_reci *
                                         dddsigma.transpose() * B_h * dsigma);
        traj.get_gdC().block<6, 2>(i * 6, 0) +=
            omg * step * wei_latacc_ * violaLatAccPenaD * gradViolaLatAc;
        gdTs(i) +=
            omg * wei_latacc_ *
            (violaLatAccPenaD * gradViolaLatAt * step + violaLatAccPena / K);

        latacc_cost += omg * step * wei_latacc_ * violaLatAccPena;
      }
      if (violaCurL > 0.0) {
        PositiveSmoothedL1(violaCurL, violaCurPenaL, violaCurPenaDL);
        gradViolaKLc = beta1 * (vel3_2_reci_e * ddsigma.transpose() * B_h -
                                3 * vel3_2_reci_e * vel2_reci_e * z_h3 *
                                    dsigma.transpose()) +
                       beta2 * vel3_2_reci_e * dsigma.transpose() *
                           B_h.transpose();  // 6*2
        gradViolaKLt = alpha * vel3_2_reci_e *
                       (dddsigma.transpose() * B_h * dsigma -
                        3 * vel2_reci_e * z_h3 * z_h1);
        traj.get_gdC().block<6, 2>(i * 6, 0) +=
            omg * step * wei_cur_ * violaCurPenaDL * gradViolaKLc;
        gdTs(i) += omg * wei_cur_ *
                   (violaCurPenaDL * gradViolaKLt * step + violaCurPenaL / K);

        cur_cost += omg * step * wei_cur_ * violaCurPenaL;
      }
      if (violaCurR > 0.0) {
        PositiveSmoothedL1(violaCurR, violaCurPenaR, violaCurPenaDR);
        gradViolaKRc = -(beta1 * (vel3_2_reci_e * ddsigma.transpose() * B_h -
                                  3 * vel3_2_reci_e * vel2_reci_e * z_h3 *
                                      dsigma.transpose()) +
                         beta2 * vel3_2_reci_e * dsigma.transpose() *
                             B_h.transpose());  // 6*2
        gradViolaKRt = -(alpha * vel3_2_reci_e *
                         (dddsigma.transpose() * B_h * dsigma -
                          3 * vel2_reci_e * z_h3 * z_h1));
        traj.get_gdC().block<6, 2>(i * 6, 0) +=
            omg * step * wei_cur_ * violaCurPenaDR * gradViolaKRc;
        gdTs(i) += omg * wei_cur_ *
                   (violaCurPenaDR * gradViolaKRt * step + violaCurPenaR / K);

        cur_cost += omg * step * wei_cur_ * violaCurPenaR;
      }
      if (violaPhidotL > 0.0) {
        PositiveSmoothedL1(violaPhidotL, violaPhidotPenaL, violaPhidotPenaDL);
        Eigen::Vector2d partial_S_over_partial_dsigma =
            L * (n3 * B_h.transpose() * dddsigma + 3 * z1 * n1 * dsigma -
                 3 * B_h.transpose() * ddsigma * z3 * n1 - 3 * z2 * ddsigma -
                 3 * z2 * z3 * dsigma / z1);
        Eigen::Vector2d partial_M_over_partial_dsigma =
            6 * n4 * dsigma + 2 * L * L * z2 * B_h.transpose() * ddsigma;
        Eigen::Vector2d partial_S_over_partial_ddsigma =
            -3 * L * n1 * (B_h * dsigma * z3 + z2 * dsigma);
        Eigen::Vector2d partial_M_over_partial_ddsigma =
            2 * L * L * z2 * B_h * dsigma;
        Eigen::Vector2d partial_S_over_partial_dddsigma = L * n3 * B_h * dsigma;
        Eigen::Vector2d partial_phi_dot_over_partial_dsigma =
            (partial_S_over_partial_dsigma * phidot_denominator -
             partial_M_over_partial_dsigma * phidot_nominator) /
            pow(phidot_denominator, 2);
        Eigen::Vector2d partial_phi_dot_over_partial_ddsigma =
            (partial_S_over_partial_ddsigma * phidot_denominator -
             partial_M_over_partial_ddsigma * phidot_nominator) /
            pow(phidot_denominator, 2);
        Eigen::Vector2d partial_phi_dot_over_partial_dddsigma =
            partial_S_over_partial_dddsigma / phidot_denominator;

        gradViolaPhidotLc =
            beta1 * partial_phi_dot_over_partial_dsigma.transpose() +
            beta2 * partial_phi_dot_over_partial_ddsigma.transpose() +
            beta3 * partial_phi_dot_over_partial_dddsigma.transpose();
        gradViolaPhidotLt =
            alpha *
            (partial_phi_dot_over_partial_dsigma.transpose() * ddsigma +
             partial_phi_dot_over_partial_ddsigma.transpose() * dddsigma +
             partial_phi_dot_over_partial_dddsigma.transpose() * ddddsigma)(0,
                                                                            0);

        traj.get_gdC().block<6, 2>(i * 6, 0) +=
            omg * step * wei_phidot_ * violaPhidotPenaDL * gradViolaPhidotLc;
        gdTs(i) += omg * wei_phidot_ *
                   (violaPhidotPenaDL * gradViolaPhidotLt * step +
                    violaPhidotPenaL / K);

        phidot_cost += omg * step * wei_phidot_ * violaPhidotPenaL;
      }
      if (violaPhidotR > 0.0) {
        PositiveSmoothedL1(violaPhidotR, violaPhidotPenaR, violaPhidotPenaDR);
        Eigen::Vector2d partial_S_over_partial_dsigma =
            L * (n3 * B_h.transpose() * dddsigma + 3 * z1 * n1 * dsigma -
                 3 * B_h.transpose() * ddsigma * z3 * n1 -
                 3 * z2 * ddsigma * n1 - 3 * z2 * z3 * dsigma / n1);
        Eigen::Vector2d partial_M_over_partial_dsigma =
            6 * n4 * dsigma + 2 * L * L * z2 * B_h.transpose() * ddsigma;
        Eigen::Vector2d partial_S_over_partial_ddsigma =
            -3 * L * n1 * (B_h * dsigma * z3 + z2 * dsigma);
        Eigen::Vector2d partial_M_over_partial_ddsigma =
            2 * L * L * z2 * B_h * dsigma;
        Eigen::Vector2d partial_S_over_partial_dddsigma = L * n3 * B_h * dsigma;

        Eigen::Vector2d partial_phi_dot_over_partial_dsigma =
            (partial_S_over_partial_dsigma * phidot_denominator -
             partial_M_over_partial_dsigma * phidot_nominator) /
            pow(phidot_denominator, 2);
        Eigen::Vector2d partial_phi_dot_over_partial_ddsigma =
            (partial_S_over_partial_ddsigma * phidot_denominator -
             partial_M_over_partial_ddsigma * phidot_nominator) /
            pow(phidot_denominator, 2);
        Eigen::Vector2d partial_phi_dot_over_partial_dddsigma =
            partial_S_over_partial_dddsigma / phidot_denominator;

        gradViolaPhidotRc =
            -beta1 * partial_phi_dot_over_partial_dsigma.transpose() -
            beta2 * partial_phi_dot_over_partial_ddsigma.transpose() -
            beta3 * partial_phi_dot_over_partial_dddsigma.transpose();
        gradViolaPhidotRt =
            -alpha *
            (partial_phi_dot_over_partial_dsigma.transpose() * ddsigma +
             partial_phi_dot_over_partial_ddsigma.transpose() * dddsigma +
             partial_phi_dot_over_partial_dddsigma.transpose() * ddddsigma)(0,
                                                                            0);

        traj.get_gdC().block<6, 2>(i * 6, 0) +=
            omg * step * wei_phidot_ * violaPhidotPenaDR * gradViolaPhidotRc;
        gdTs(i) += omg * wei_phidot_ *
                   (violaPhidotPenaDR * gradViolaPhidotRt * step +
                    violaPhidotPenaR / K);

        phidot_cost += omg * step * wei_phidot_ * violaPhidotPenaR;
      }
    }

    t += traj.get_T1()(i);
  }

  LOG_INFO(
      "cnt: {}, c: {:.3f}, s: {:.3f}, m: {:.3f}, v: {:.3f}, a: {:.3f}, l: "
      "{:.3f}, k: {:.3f}, pd: {:.3f}",
      iter_cnt_, collision_cost, shrink_cost, match_cost, vel_cost, acc_cost,
      latacc_cost, cur_cost, phidot_cost);

  return collision_cost + shrink_cost + match_cost + vel_cost + acc_cost +
         latacc_cost + cur_cost + phidot_cost;
}

}  // namespace planning
}  // namespace neodrive
