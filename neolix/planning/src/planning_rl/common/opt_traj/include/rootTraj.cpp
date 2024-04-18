#include <float.h>
#include <algorithm>
#include <iostream>

#include "vlrException.h"

#include <math.h>
#include "rootTraj.h"

RootTraj::RootTraj(const std::vector<CurvePoint>& center_line) {
  center_line_ = center_line;
}

void RootTraj::calculateRootTrajectory() {
  if (discrete_traj_points_.size() == 0) {
    throw printf(
        "Cannot calc root trajectory as there are no discrete_traj_points_ "
        "sets!");
  }
  if (discrete_traj_points_.size() == 0) {
    throw printf(
        "Cannot calc root trajectory as there are no points in first set of "
        "the discrete_traj_points_!");
  }
  std::vector<trajectory_point_1D>::const_iterator it_traj_point_1D;
  for (it_traj_point_1D = discrete_traj_points_[0].begin();
       it_traj_point_1D != discrete_traj_points_[0].end(); it_traj_point_1D++) {
    CurvePoint cp_temp;

    try {
      evalCenterlineAtS(it_traj_point_1D->x, cp_temp);
    } catch (Ex<>& e) {
      throw printf("Calculate root trajectory error!");
    }
    foot_curve_points_.push_back(cp_temp);
  }
  return;
}

void RootTraj::evalCenterlineAtS(const double& s_interpol,
                                 CurvePoint& cp_interpol) const {
  evalCenterlineAtS_static(center_line_, s_interpol, cp_interpol);
  return;
}

void RootTraj::evalCenterlineAtS_static(
    const std::vector<CurvePoint>& center_line, double s_interpol,
    CurvePoint& cp_interpol) {
  // Find closest index
  if (center_line.size() < 2) {
    // throw printf("Center line doesn't have entries!");
    std::cout << "Center line too short" << std::endl;
  }
  if (s_interpol < center_line.begin()->s) {
    std::cout << "Center line too short at the beginning!" << std::endl;
    // throw printf("Center line too short at the beginning!");
  }
  if (s_interpol > center_line.rbegin()->s) {
    s_interpol = center_line.rbegin()->s;
    std::cout << "Center line too short at the end!" << std::endl;
    // throw printf("Center line too short at the end!");
  }
  // if (s_interpol == center_line.begin()->s) {
  //   std::cout << "Center line right equal the beginning!!!" << std::endl;
  // }

  CurvePoint cp_tmp;
  cp_tmp.s = s_interpol;
  std::vector<CurvePoint>::const_iterator it_upper = std::lower_bound(
      center_line.begin(), center_line.end(), cp_tmp, cp_comp_);
  if (it_upper == center_line.end()) {
    std::cout << "Current s not contained in center_line" << std::endl;
    // throw printf("Current s not contained in center_line");
  }

  const CurvePoint& cp2 = *it_upper;
  double s2 = cp2.s;
  it_upper--;  // it_lower now
  const CurvePoint& cp1 = *it_upper;
  double s1 = cp1.s;
  // for (auto& clp: center_line) {
  //   std::cout << clp.s << std::endl;
  // }
  // std::cout << "center_line_len:" << center_line.size() << std::endl;
  // for (int i = 0; i < center_line.size(); i++) {
  //   std::cout << i << std::endl;
  //   std::cout << center_line[i].x << std::endl;
  //   std::cout << center_line[i].y << std::endl;
  //   std::cout << center_line[i].s << std::endl;
  // }
  // std::cout << "compute_x" << std::endl;
  // std::cout << s2 << std::endl;
  // std::cout << s1 << std::endl;

  double lambda = 0.0;
  if (((s2 - s1) > DBL_EPSILON) || ((s2 - s1) < -DBL_EPSILON)) {
    lambda = (s_interpol - s1) / (s2 - s1);
  } else {
    std::cout << "Center line points too close for interpolation!" << std::endl;
    // throw printf("Center line points too close for interpolation!");
    // std::cout << "center_line_len:" << center_line.size() << std::endl;
    // for (int i = 0; i < center_line.size(); i++) {
    //   std::cout << i << std::endl;
    //   std::cout << center_line[i].x << std::endl;
    //   std::cout << center_line[i].y << std::endl;
    //   std::cout << center_line[i].s << std::endl;
    // }
    // std::cout << cp1.x << std::endl;
    // std::cout << cp2.x << std::endl;
    // std::cout << cp1.s << std::endl;
    // std::cout << cp2.s << std::endl;
  }
  // std::cout << "lambda: " << lambda << std::endl;
  if (s_interpol == center_line.begin()->s) {
    std::cout << "Center line right equal the beginning!!!" << std::endl;
    lambda = 1;
  }

  // Linear interpolation between points
  double x_interpol = cp1.x + lambda * (cp2.x - cp1.x);
  // std::cout << cp1.x << std::endl;
  // std::cout << cp2.x << std::endl;
  // std::cout << cp1.s << std::endl;
  // std::cout << cp2.s << std::endl;
  // std::cout << x_interpol << std::endl;
  double y_interpol = cp1.y + lambda * (cp2.y - cp1.y);

  double delta_theta = cp2.theta - cp1.theta;

  delta_theta = normalize_angle(delta_theta);
  double theta_interpol = cp1.theta + lambda * delta_theta;
  theta_interpol = normalize_angle(theta_interpol);
  double kappa_interpol = cp1.kappa + lambda * (cp2.kappa - cp1.kappa);
  double kappa_prime_interpol =
      cp1.kappa_prime + lambda * (cp2.kappa_prime - cp1.kappa_prime);

  cp_interpol.x = x_interpol;
  cp_interpol.y = y_interpol;
  cp_interpol.theta = theta_interpol;
  cp_interpol.kappa = kappa_interpol;
  cp_interpol.kappa_prime = kappa_prime_interpol;
  // std::cout << "cp_interpol:" << std::endl;
  // std::cout << cp_interpol.x << std::endl;
  // std::cout << cp_interpol.y << std::endl;
  // std::cout << cp_interpol.theta << std::endl;
  // std::cout << cp_interpol.kappa << std::endl;
  // std::cout << cp_interpol.kappa_prime << std::endl;
}
