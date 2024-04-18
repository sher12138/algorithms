#include "src/planning/task/optimizers/freespace_path_optimizer/reeds_shepp_path.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "src/common/angles/angles.h"

namespace neodrive {
namespace planning {

namespace {

using RspPath = ReedsSheppPath::Path;
using RspParam = ReedsSheppPath::Param;
using Point3d = ReedsSheppPath::Point3d;
using Config = ReedsSheppPath::Config;

using namespace neodrive::common::angles;

std::pair<double, double> Cartesian2Polar(double x, double y) {
  double r = std::sqrt(x * x + y * y);
  double theta = std::atan2(y, x);
  return {r, theta};
}

std::pair<double, double> calc_tau_omega(const double u, const double v,
                                         const double xi, const double eta,
                                         const double phi) {
  double delta = normalize_angle(u - v);
  double A = std::sin(u) - std::sin(delta);
  double B = std::cos(u) - std::cos(delta) - 1.0;

  double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
  double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
  double tau = 0.0;
  if (t2 < 0) {
    tau = normalize_angle(t1 + M_PI);
  } else {
    tau = normalize_angle(t1);
  }
  double omega = normalize_angle(tau - u + v - phi);
  return std::make_pair(tau, omega);
}

bool SetRSP(const int size, const double* lengths, const char* types,
            std::vector<RspPath>* all_possible_paths) {
  RspPath path;
  std::vector<double> length_vec(lengths, lengths + size);
  std::vector<char> type_vec(types, types + size);
  path.segs_lengths = length_vec;
  path.segs_types = type_vec;
  for (auto l : length_vec) path.total_length += std::abs(l);
  all_possible_paths->emplace_back(path);
  return true;
}

void Interpolation(const double max_kappa, const int index, const double pd,
                   const char m, const double ox, const double oy,
                   const double ophi, std::vector<double>* px,
                   std ::vector<double>* py, std::vector<double>* pphi,
                   std::vector<bool>* pgear) {
  double ldx = 0.0;
  double ldy = 0.0;
  double gdx = 0.0;
  double gdy = 0.0;
  if (m == 'S') {
    px->at(index) = ox + pd / max_kappa * std::cos(ophi);
    py->at(index) = oy + pd / max_kappa * std::sin(ophi);
    pphi->at(index) = ophi;
  } else {
    ldx = std::sin(pd) / max_kappa;
    if (m == 'L') {
      ldy = (1.0 - std::cos(pd)) / max_kappa;
    } else if (m == 'R') {
      ldy = (1.0 - std::cos(pd)) / -max_kappa;
    }
    gdx = std::cos(-ophi) * ldx + std::sin(-ophi) * ldy;
    gdy = -std::sin(-ophi) * ldx + std::cos(-ophi) * ldy;
    px->at(index) = ox + gdx;
    py->at(index) = oy + gdy;
  }

  if (pd > 0.0) {
    pgear->at(index) = true;
  } else {
    pgear->at(index) = false;
  }

  if (m == 'L') {
    pphi->at(index) = ophi + pd;
  } else if (m == 'R') {
    pphi->at(index) = ophi - pd;
  }
}

void LSL(const double x, const double y, const double phi, RspParam* param) {
  std::pair<double, double> polar =
      Cartesian2Polar(x - std::sin(phi), y - 1.0 + std::cos(phi));
  double u = polar.first;
  double t = polar.second;
  double v = 0.0;
  if (t >= 0.0) {
    v = normalize_angle(phi - t);
    if (v >= 0.0) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

void LSR(const double x, const double y, const double phi, RspParam* param) {
  std::pair<double, double> polar =
      Cartesian2Polar(x + std::sin(phi), y - 1.0 - std::cos(phi));
  double u1 = polar.first * polar.first;
  double t1 = polar.second;
  double u = 0.0;
  double theta = 0.0;
  double t = 0.0;
  double v = 0.0;
  if (u1 >= 4.0) {
    u = std::sqrt(u1 - 4.0);
    theta = std::atan2(2.0, u);
    t = normalize_angle(t1 + theta);
    v = normalize_angle(t - phi);
    if (t >= 0.0 && v >= 0.0) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

void LRL(const double x, const double y, const double phi, RspParam* param) {
  std::pair<double, double> polar =
      Cartesian2Polar(x - std::sin(phi), y - 1.0 + std::cos(phi));
  double u1 = polar.first;
  double t1 = polar.second;
  double u = 0.0;
  double t = 0.0;
  double v = 0.0;
  if (u1 <= 4.0) {
    u = -2.0 * std::asin(0.25 * u1);
    t = normalize_angle(t1 + 0.5 * u + M_PI);
    v = normalize_angle(phi - t + u);
    if (t >= 0.0 && u <= 0.0) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

void SLS(const double x, const double y, const double phi, RspParam* param) {
  double phi_mod = normalize_angle(phi);
  double xd = 0.0;
  double u = 0.0;
  double t = 0.0;
  double v = 0.0;
  double epsilon = 1e-1;
  if (y > 0.0 && phi_mod > epsilon && phi_mod < M_PI) {
    xd = -y / std::tan(phi_mod) + x;
    t = xd - std::tan(phi_mod / 2.0);
    u = phi_mod;
    v = std::sqrt((x - xd) * (x - xd) + y * y) - tan(phi_mod / 2.0);
    param->flag = true;
    param->u = u;
    param->t = t;
    param->v = v;
  } else if (y < 0.0 && phi_mod > epsilon && phi_mod < M_PI) {
    xd = -y / std::tan(phi_mod) + x;
    t = xd - std::tan(phi_mod / 2.0);
    u = phi_mod;
    v = -std::sqrt((x - xd) * (x - xd) + y * y) - std::tan(phi_mod / 2.0);
    param->flag = true;
    param->u = u;
    param->t = t;
    param->v = v;
  }
}

void LRLRn(const double x, const double y, const double phi, RspParam* param) {
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
  double u = 0.0;
  if (rho <= 1.0 && rho >= 0.0) {
    u = std::acos(rho);
    if (u >= 0 && u <= 0.5 * M_PI) {
      std::pair<double, double> tau_omega = calc_tau_omega(u, -u, xi, eta, phi);
      if (tau_omega.first >= 0.0 && tau_omega.second <= 0.0) {
        param->flag = true;
        param->u = u;
        param->t = tau_omega.first;
        param->v = tau_omega.second;
      }
    }
  }
}

void LRLRp(const double x, const double y, const double phi, RspParam* param) {
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  double u = 0.0;
  if (rho <= 1.0 && rho >= 0.0) {
    u = -std::acos(rho);
    if (u >= 0 && u <= 0.5 * M_PI) {
      std::pair<double, double> tau_omega = calc_tau_omega(u, u, xi, eta, phi);
      if (tau_omega.first >= 0.0 && tau_omega.second >= 0.0) {
        param->flag = true;
        param->u = u;
        param->t = tau_omega.first;
        param->v = tau_omega.second;
      }
    }
  }
}

void LRSR(const double x, const double y, const double phi, RspParam* param) {
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  std::pair<double, double> polar = Cartesian2Polar(-eta, xi);
  double rho = polar.first;
  double theta = polar.second;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  if (rho >= 2.0) {
    t = theta;
    u = 2.0 - rho;
    v = normalize_angle(t + 0.5 * M_PI - phi);
    if (t >= 0.0 && u <= 0.0 && v <= 0.0) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

void LRSL(const double x, const double y, const double phi, RspParam* param) {
  double xi = x - std::sin(phi);
  double eta = y - 1.0 + std::cos(phi);
  std::pair<double, double> polar = Cartesian2Polar(xi, eta);
  double rho = polar.first;
  double theta = polar.second;
  double r = 0.0;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;

  if (rho >= 2.0) {
    r = std::sqrt(rho * rho - 4.0);
    u = 2.0 - r;
    t = normalize_angle(theta + std::atan2(r, -2.0));
    v = normalize_angle(phi - 0.5 * M_PI - t);
    if (t >= 0.0 && u <= 0.0 && v <= 0.0) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

void LRSLR(const double x, const double y, const double phi, RspParam* param) {
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  std::pair<double, double> polar = Cartesian2Polar(xi, eta);
  double rho = polar.first;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  if (rho >= 2.0) {
    u = 4.0 - std::sqrt(rho * rho - 4.0);
    if (u <= 0.0) {
      t = normalize_angle(
          atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
      v = normalize_angle(t - phi);

      if (t >= 0.0 && v >= 0.0) {
        param->flag = true;
        param->u = u;
        param->t = t;
        param->v = v;
      }
    }
  }
}

bool SCS(const double x, const double y, const double phi,
         std::vector<RspPath>* all_possible_paths) {
  RspParam SLS_param;
  SLS(x, y, phi, &SLS_param);
  double SLS_lengths[3] = {SLS_param.t, SLS_param.u, SLS_param.v};
  char SLS_types[] = "SLS";
  if (SLS_param.flag &&
      !SetRSP(3, SLS_lengths, SLS_types, all_possible_paths)) {
    printf("Fail at SetRSP with SLS_param\n");
    return false;
  }

  RspParam SRS_param;
  SLS(x, -y, -phi, &SRS_param);
  double SRS_lengths[3] = {SRS_param.t, SRS_param.u, SRS_param.v};
  char SRS_types[] = "SRS";
  if (SRS_param.flag &&
      !SetRSP(3, SRS_lengths, SRS_types, all_possible_paths)) {
    printf("Fail at SetRSP with SRS_param\n");
    return false;
  }
  return true;
}

bool CSC(const double x, const double y, const double phi,
         std::vector<RspPath>* all_possible_paths) {
  RspParam LSL1_param;
  LSL(x, y, phi, &LSL1_param);
  double LSL1_lengths[3] = {LSL1_param.t, LSL1_param.u, LSL1_param.v};
  char LSL1_types[] = "LSL";
  if (LSL1_param.flag &&
      !SetRSP(3, LSL1_lengths, LSL1_types, all_possible_paths)) {
    printf("Fail at SetRSP with LSL1_param\n");
    return false;
  }

  RspParam LSL2_param;
  LSL(-x, y, -phi, &LSL2_param);
  double LSL2_lengths[3] = {-LSL2_param.t, -LSL2_param.u, -LSL2_param.v};
  char LSL2_types[] = "LSL";
  if (LSL2_param.flag &&
      !SetRSP(3, LSL2_lengths, LSL2_types, all_possible_paths)) {
    printf("Fail at SetRSP with LSL2_param\n");
    return false;
  }

  RspParam LSL3_param;
  LSL(x, -y, -phi, &LSL3_param);
  double LSL3_lengths[3] = {LSL3_param.t, LSL3_param.u, LSL3_param.v};
  char LSL3_types[] = "RSR";
  if (LSL3_param.flag &&
      !SetRSP(3, LSL3_lengths, LSL3_types, all_possible_paths)) {
    printf("Fail at SetRSP with LSL3_param\n");
    return false;
  }

  RspParam LSL4_param;
  LSL(-x, -y, phi, &LSL4_param);
  double LSL4_lengths[3] = {-LSL4_param.t, -LSL4_param.u, -LSL4_param.v};
  char LSL4_types[] = "RSR";
  if (LSL4_param.flag &&
      !SetRSP(3, LSL4_lengths, LSL4_types, all_possible_paths)) {
    printf("Fail at SetRSP with LSL4_param\n");
    return false;
  }

  RspParam LSR1_param;
  LSR(x, y, phi, &LSR1_param);
  double LSR1_lengths[3] = {LSR1_param.t, LSR1_param.u, LSR1_param.v};
  char LSR1_types[] = "LSR";
  if (LSR1_param.flag &&
      !SetRSP(3, LSR1_lengths, LSR1_types, all_possible_paths)) {
    printf("Fail at SetRSP with LSR1_param\n");
    return false;
  }

  RspParam LSR2_param;
  LSR(-x, y, -phi, &LSR2_param);
  double LSR2_lengths[3] = {-LSR2_param.t, -LSR2_param.u, -LSR2_param.v};
  char LSR2_types[] = "LSR";
  if (LSR2_param.flag &&
      !SetRSP(3, LSR2_lengths, LSR2_types, all_possible_paths)) {
    printf("Fail at SetRSP with LSR2_param\n");
    return false;
  }

  RspParam LSR3_param;
  LSR(x, -y, -phi, &LSR3_param);
  double LSR3_lengths[3] = {LSR3_param.t, LSR3_param.u, LSR3_param.v};
  char LSR3_types[] = "RSL";
  if (LSR3_param.flag &&
      !SetRSP(3, LSR3_lengths, LSR3_types, all_possible_paths)) {
    printf("Fail at SetRSP with LSR3_param\n");
    return false;
  }

  RspParam LSR4_param;
  LSR(-x, -y, phi, &LSR4_param);
  double LSR4_lengths[3] = {-LSR4_param.t, -LSR4_param.u, -LSR4_param.v};
  char LSR4_types[] = "RSL";
  if (LSR4_param.flag &&
      !SetRSP(3, LSR4_lengths, LSR4_types, all_possible_paths)) {
    printf("Fail at SetRSP with LSR4_param\n");
    return false;
  }
  return true;
}

bool CCC(const double x, const double y, const double phi,
         std::vector<RspPath>* all_possible_paths) {
  RspParam LRL1_param;
  LRL(x, y, phi, &LRL1_param);
  double LRL1_lengths[3] = {LRL1_param.t, LRL1_param.u, LRL1_param.v};
  char LRL1_types[] = "LRL";
  if (LRL1_param.flag &&
      !SetRSP(3, LRL1_lengths, LRL1_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRL_param\n");
    return false;
  }

  RspParam LRL2_param;
  LRL(-x, y, -phi, &LRL2_param);
  double LRL2_lengths[3] = {-LRL2_param.t, -LRL2_param.u, -LRL2_param.v};
  char LRL2_types[] = "LRL";
  if (LRL2_param.flag &&
      !SetRSP(3, LRL2_lengths, LRL2_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRL2_param\n");
    return false;
  }

  RspParam LRL3_param;
  LRL(x, -y, -phi, &LRL3_param);
  double LRL3_lengths[3] = {LRL3_param.t, LRL3_param.u, LRL3_param.v};
  char LRL3_types[] = "RLR";
  if (LRL3_param.flag &&
      !SetRSP(3, LRL3_lengths, LRL3_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRL3_param\n");
    return false;
  }

  RspParam LRL4_param;
  LRL(-x, -y, phi, &LRL4_param);
  double LRL4_lengths[3] = {-LRL4_param.t, -LRL4_param.u, -LRL4_param.v};
  char LRL4_types[] = "RLR";
  if (LRL4_param.flag &&
      !SetRSP(3, LRL4_lengths, LRL4_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRL4_param\n");
    return false;
  }

  // backward
  double xb = x * std::cos(phi) + y * std::sin(phi);
  double yb = x * std::sin(phi) - y * std::cos(phi);

  RspParam LRL5_param;
  LRL(xb, yb, phi, &LRL5_param);
  double LRL5_lengths[3] = {LRL5_param.v, LRL5_param.u, LRL5_param.t};
  char LRL5_types[] = "LRL";
  if (LRL5_param.flag &&
      !SetRSP(3, LRL5_lengths, LRL5_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRL5_param\n");
    return false;
  }

  RspParam LRL6_param;
  LRL(-xb, yb, -phi, &LRL6_param);
  double LRL6_lengths[3] = {-LRL6_param.v, -LRL6_param.u, -LRL6_param.t};
  char LRL6_types[] = "LRL";
  if (LRL6_param.flag &&
      !SetRSP(3, LRL6_lengths, LRL6_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRL6_param\n");
    return false;
  }

  RspParam LRL7_param;
  LRL(xb, -yb, -phi, &LRL7_param);
  double LRL7_lengths[3] = {LRL7_param.v, LRL7_param.u, LRL7_param.t};
  char LRL7_types[] = "RLR";
  if (LRL7_param.flag &&
      !SetRSP(3, LRL7_lengths, LRL7_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRL7_param\n");
    return false;
  }

  RspParam LRL8_param;
  LRL(-xb, -yb, phi, &LRL8_param);
  double LRL8_lengths[3] = {-LRL8_param.v, -LRL8_param.u, -LRL8_param.t};
  char LRL8_types[] = "RLR";
  if (LRL8_param.flag &&
      !SetRSP(3, LRL8_lengths, LRL8_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRL8_param\n");
    return false;
  }
  return true;
}

bool CCCC(const double x, const double y, const double phi,
          std::vector<RspPath>* all_possible_paths) {
  RspParam LRLRn1_param;
  LRLRn(x, y, phi, &LRLRn1_param);
  double LRLRn1_lengths[4] = {LRLRn1_param.t, LRLRn1_param.u, -LRLRn1_param.u,
                              LRLRn1_param.v};
  char LRLRn1_types[] = "LRLR";
  if (LRLRn1_param.flag &&
      !SetRSP(4, LRLRn1_lengths, LRLRn1_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRLRn_param\n");
    return false;
  }

  RspParam LRLRn2_param;
  LRLRn(-x, y, -phi, &LRLRn2_param);
  double LRLRn2_lengths[4] = {-LRLRn2_param.t, -LRLRn2_param.u, LRLRn2_param.u,
                              -LRLRn2_param.v};
  char LRLRn2_types[] = "LRLR";
  if (LRLRn2_param.flag &&
      !SetRSP(4, LRLRn2_lengths, LRLRn2_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRLRn2_param\n");
    return false;
  }

  RspParam LRLRn3_param;
  LRLRn(x, -y, -phi, &LRLRn3_param);
  double LRLRn3_lengths[4] = {LRLRn3_param.t, LRLRn3_param.u, -LRLRn3_param.u,
                              LRLRn3_param.v};
  char LRLRn3_types[] = "RLRL";
  if (LRLRn3_param.flag &&
      !SetRSP(4, LRLRn3_lengths, LRLRn3_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRLRn3_param\n");
    return false;
  }

  RspParam LRLRn4_param;
  LRLRn(-x, -y, phi, &LRLRn4_param);
  double LRLRn4_lengths[4] = {-LRLRn4_param.t, -LRLRn4_param.u, LRLRn4_param.u,
                              -LRLRn4_param.v};
  char LRLRn4_types[] = "RLRL";
  if (LRLRn4_param.flag &&
      !SetRSP(4, LRLRn4_lengths, LRLRn4_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRLRn4_param\n");
    return false;
  }

  RspParam LRLRp1_param;
  LRLRp(x, y, phi, &LRLRp1_param);
  double LRLRp1_lengths[4] = {LRLRp1_param.t, LRLRp1_param.u, LRLRp1_param.u,
                              LRLRp1_param.v};
  char LRLRp1_types[] = "LRLR";
  if (LRLRp1_param.flag &&
      !SetRSP(4, LRLRp1_lengths, LRLRp1_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRLRp1_param\n");
    return false;
  }

  RspParam LRLRp2_param;
  LRLRp(-x, y, -phi, &LRLRp2_param);
  double LRLRp2_lengths[4] = {-LRLRp2_param.t, -LRLRp2_param.u, -LRLRp2_param.u,
                              -LRLRp2_param.v};
  char LRLRp2_types[] = "LRLR";
  if (LRLRp2_param.flag &&
      !SetRSP(4, LRLRp2_lengths, LRLRp2_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRLRp2_param\n");
    return false;
  }

  RspParam LRLRp3_param;
  LRLRp(x, -y, -phi, &LRLRp3_param);
  double LRLRp3_lengths[4] = {LRLRp3_param.t, LRLRp3_param.u, LRLRp3_param.u,
                              LRLRp3_param.v};
  char LRLRp3_types[] = "RLRL";
  if (LRLRp3_param.flag &&
      !SetRSP(4, LRLRp3_lengths, LRLRp3_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRLRp3_param\n");
    return false;
  }

  RspParam LRLRp4_param;
  LRLRp(-x, -y, phi, &LRLRp4_param);
  double LRLRp4_lengths[4] = {-LRLRp4_param.t, -LRLRp4_param.u, -LRLRp4_param.u,
                              -LRLRp4_param.v};
  char LRLRp4_types[] = "RLRL";
  if (LRLRp4_param.flag &&
      !SetRSP(4, LRLRp4_lengths, LRLRp4_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRLRp4_param\n");
    return false;
  }
  return true;
}

bool CCSC(const double x, const double y, const double phi,
          std::vector<RspPath>* all_possible_paths) {
  RspParam LRSL1_param;
  LRSL(x, y, phi, &LRSL1_param);
  double LRSL1_lengths[4] = {LRSL1_param.t, -0.5 * M_PI, LRSL1_param.u,
                             LRSL1_param.v};
  char LRSL1_types[] = "LRSL";
  if (LRSL1_param.flag &&
      !SetRSP(4, LRSL1_lengths, LRSL1_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSL1_param\n");
    return false;
  }

  RspParam LRSL2_param;
  LRSL(-x, y, -phi, &LRSL2_param);
  double LRSL2_lengths[4] = {-LRSL2_param.t, 0.5 * M_PI, -LRSL2_param.u,
                             -LRSL2_param.v};
  char LRSL2_types[] = "LRSL";
  if (LRSL2_param.flag &&
      !SetRSP(4, LRSL2_lengths, LRSL2_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSL2_param\n");
    return false;
  }

  RspParam LRSL3_param;
  LRSL(x, -y, -phi, &LRSL3_param);
  double LRSL3_lengths[4] = {LRSL3_param.t, -0.5 * M_PI, LRSL3_param.u,
                             LRSL3_param.v};
  char LRSL3_types[] = "RLSR";
  if (LRSL3_param.flag &&
      !SetRSP(4, LRSL3_lengths, LRSL3_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSL3_param\n");
    return false;
  }

  RspParam LRSL4_param;
  LRSL(-x, -y, phi, &LRSL4_param);
  double LRSL4_lengths[4] = {-LRSL4_param.t, 0.5 * M_PI, -LRSL4_param.u,
                             -LRSL4_param.v};
  char LRSL4_types[] = "RLSR";
  if (LRSL4_param.flag &&
      !SetRSP(4, LRSL4_lengths, LRSL4_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSL4_param\n");
    return false;
  }

  RspParam LRSR1_param;
  LRSR(x, y, phi, &LRSR1_param);
  double LRSR1_lengths[4] = {LRSR1_param.t, -0.5 * M_PI, LRSR1_param.u,
                             LRSR1_param.v};
  char LRSR1_types[] = "LRSR";
  if (LRSR1_param.flag &&
      !SetRSP(4, LRSR1_lengths, LRSR1_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSR1_param\n");
    return false;
  }

  RspParam LRSR2_param;
  LRSR(-x, y, -phi, &LRSR2_param);
  double LRSR2_lengths[4] = {-LRSR2_param.t, 0.5 * M_PI, -LRSR2_param.u,
                             -LRSR2_param.v};
  char LRSR2_types[] = "LRSR";
  if (LRSR2_param.flag &&
      !SetRSP(4, LRSR2_lengths, LRSR2_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSR2_param\n");
    return false;
  }

  RspParam LRSR3_param;
  LRSR(x, -y, -phi, &LRSR3_param);
  double LRSR3_lengths[4] = {LRSR3_param.t, -0.5 * M_PI, LRSR3_param.u,
                             LRSR3_param.v};
  char LRSR3_types[] = "RLSL";
  if (LRSR3_param.flag &&
      !SetRSP(4, LRSR3_lengths, LRSR3_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSR3_param\n");
    return false;
  }

  RspParam LRSR4_param;
  LRSR(-x, -y, phi, &LRSR4_param);
  double LRSR4_lengths[4] = {-LRSR4_param.t, 0.5 * M_PI, -LRSR4_param.u,
                             -LRSR4_param.v};
  char LRSR4_types[] = "RLSL";
  if (LRSR4_param.flag &&
      !SetRSP(4, LRSR4_lengths, LRSR4_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSR4_param\n");
    return false;
  }

  // backward
  double xb = x * std::cos(phi) + y * std::sin(phi);
  double yb = x * std::sin(phi) - y * std::cos(phi);

  RspParam LRSL5_param;
  LRSL(xb, yb, phi, &LRSL5_param);
  double LRSL5_lengths[4] = {LRSL5_param.v, LRSL5_param.u, -0.5 * M_PI,
                             LRSL5_param.t};
  char LRSL5_types[] = "LSRL";
  if (LRSL5_param.flag &&
      !SetRSP(4, LRSL5_lengths, LRSL5_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRLRn_param\n");
    return false;
  }

  RspParam LRSL6_param;
  LRSL(-xb, yb, -phi, &LRSL6_param);
  double LRSL6_lengths[4] = {-LRSL6_param.v, -LRSL6_param.u, 0.5 * M_PI,
                             -LRSL6_param.t};
  char LRSL6_types[] = "LSRL";
  if (LRSL6_param.flag &&
      !SetRSP(4, LRSL6_lengths, LRSL6_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSL6_param\n");
    return false;
  }

  RspParam LRSL7_param;
  LRSL(xb, -yb, -phi, &LRSL7_param);
  double LRSL7_lengths[4] = {LRSL7_param.v, LRSL7_param.u, -0.5 * M_PI,
                             LRSL7_param.t};
  char LRSL7_types[] = "RSLR";
  if (LRSL7_param.flag &&
      !SetRSP(4, LRSL7_lengths, LRSL7_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSL7_param\n");
    return false;
  }

  RspParam LRSL8_param;
  LRSL(-xb, -yb, phi, &LRSL8_param);
  double LRSL8_lengths[4] = {-LRSL8_param.v, -LRSL8_param.u, 0.5 * M_PI,
                             -LRSL8_param.t};
  char LRSL8_types[] = "RSLR";
  if (LRSL8_param.flag &&
      !SetRSP(4, LRSL8_lengths, LRSL8_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSL8_param\n");
    return false;
  }

  RspParam LRSR5_param;
  LRSR(xb, yb, phi, &LRSR5_param);
  double LRSR5_lengths[4] = {LRSR5_param.v, LRSR5_param.u, -0.5 * M_PI,
                             LRSR5_param.t};
  char LRSR5_types[] = "RSRL";
  if (LRSR5_param.flag &&
      !SetRSP(4, LRSR5_lengths, LRSR5_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSR5_param\n");
    return false;
  }

  RspParam LRSR6_param;
  LRSR(-xb, yb, -phi, &LRSR6_param);
  double LRSR6_lengths[4] = {-LRSR6_param.v, -LRSR6_param.u, 0.5 * M_PI,
                             -LRSR6_param.t};
  char LRSR6_types[] = "RSRL";
  if (LRSR6_param.flag &&
      !SetRSP(4, LRSR6_lengths, LRSR6_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSR6_param\n");
    return false;
  }

  RspParam LRSR7_param;
  LRSR(xb, -yb, -phi, &LRSR7_param);
  double LRSR7_lengths[4] = {LRSR7_param.v, LRSR7_param.u, -0.5 * M_PI,
                             LRSR7_param.t};
  char LRSR7_types[] = "LSLR";
  if (LRSR7_param.flag &&
      !SetRSP(4, LRSR7_lengths, LRSR7_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSR7_param\n");
    return false;
  }

  RspParam LRSR8_param;
  LRSR(-xb, -yb, phi, &LRSR8_param);
  double LRSR8_lengths[4] = {-LRSR8_param.v, -LRSR8_param.u, 0.5 * M_PI,
                             -LRSR8_param.t};
  char LRSR8_types[] = "LSLR";
  if (LRSR8_param.flag &&
      !SetRSP(4, LRSR8_lengths, LRSR8_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSR8_param\n");
    return false;
  }
  return true;
}

bool CCSCC(const double x, const double y, const double phi,
           std::vector<RspPath>* all_possible_paths) {
  RspParam LRSLR1_param;
  LRSLR(x, y, phi, &LRSLR1_param);
  double LRSLR1_lengths[5] = {LRSLR1_param.t, -0.5 * M_PI, LRSLR1_param.u,
                              -0.5 * M_PI, LRSLR1_param.v};
  char LRSLR1_types[] = "LRSLR";
  if (LRSLR1_param.flag &&
      !SetRSP(5, LRSLR1_lengths, LRSLR1_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSLR1_param\n");
    return false;
  }

  RspParam LRSLR2_param;
  LRSLR(-x, y, -phi, &LRSLR2_param);
  double LRSLR2_lengths[5] = {-LRSLR2_param.t, 0.5 * M_PI, -LRSLR2_param.u,
                              0.5 * M_PI, -LRSLR2_param.v};
  char LRSLR2_types[] = "LRSLR";
  if (LRSLR2_param.flag &&
      !SetRSP(5, LRSLR2_lengths, LRSLR2_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSLR2_param\n");
    return false;
  }

  RspParam LRSLR3_param;
  LRSLR(x, -y, -phi, &LRSLR3_param);
  double LRSLR3_lengths[5] = {LRSLR3_param.t, -0.5 * M_PI, LRSLR3_param.u,
                              -0.5 * M_PI, LRSLR3_param.v};
  char LRSLR3_types[] = "RLSRL";
  if (LRSLR3_param.flag &&
      !SetRSP(5, LRSLR3_lengths, LRSLR3_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSLR3_param\n");
    return false;
  }

  RspParam LRSLR4_param;
  LRSLR(-x, -y, phi, &LRSLR4_param);
  double LRSLR4_lengths[5] = {-LRSLR4_param.t, 0.5 * M_PI, -LRSLR4_param.u,
                              0.5 * M_PI, -LRSLR4_param.v};
  char LRSLR4_types[] = "RLSRL";
  if (LRSLR4_param.flag &&
      !SetRSP(5, LRSLR4_lengths, LRSLR4_types, all_possible_paths)) {
    printf("Fail at SetRSP with LRSLR4_param\n");
    return false;
  }
  return true;
}

bool GenerateAllPrimitives(const ReedsSheppPath::Config& conf,
                           const Point3d& start_node, const Point3d& end_node,
                           std::vector<RspPath>* all_possible_paths) {
  double dx = end_node.x - start_node.x;
  double dy = end_node.y - start_node.y;
  double dphi = end_node.phi - start_node.phi;
  double c = std::cos(start_node.phi);
  double s = std::sin(start_node.phi);

  // normalize the initial point to (0,0,0)
  double x = (c * dx + s * dy) * conf.max_kappa;
  double y = (-s * dx + c * dy) * conf.max_kappa;
  if (!SCS(x, y, dphi, all_possible_paths)) printf("Failed at SCS\n");
  if (!CSC(x, y, dphi, all_possible_paths)) printf("Failed at CSC\n");
  if (!CCC(x, y, dphi, all_possible_paths)) printf("Fail at CCC\n");
  if (!CCCC(x, y, dphi, all_possible_paths)) printf("Fail at CCCC\n");
  if (!CCSC(x, y, dphi, all_possible_paths)) printf("Fail at CCSC\n");
  if (!CCSCC(x, y, dphi, all_possible_paths)) printf("Fail at CCSCC\n");

  return all_possible_paths->size();
}

bool GenerateLocalConfigurations(const ReedsSheppPath::Config& conf,
                                 const Point3d& start_node,
                                 const Point3d& end_node,
                                 RspPath* shortest_path) {
  double step_scaled = conf.step_size * conf.max_kappa;

  size_t point_num = static_cast<size_t>(
      std::floor(shortest_path->total_length / step_scaled +
                 static_cast<double>(shortest_path->segs_lengths.size()) + 4));
  std::vector<double> px(point_num, 0.0);
  std::vector<double> py(point_num, 0.0);
  std::vector<double> pphi(point_num, 0.0);
  std::vector<bool> pgear(point_num, true);
  int index = 1;
  double d = 0.0;
  double pd = 0.0;
  double ll = 0.0;

  if (shortest_path->segs_lengths.at(0) > 0.0) {
    pgear.at(0) = true;
    d = step_scaled;
  } else {
    pgear.at(0) = false;
    d = -step_scaled;
  }
  pd = d;
  for (size_t i = 0; i < shortest_path->segs_types.size(); ++i) {
    char m = shortest_path->segs_types.at(i);
    double l = shortest_path->segs_lengths.at(i);
    if (l > 0.0) {
      d = step_scaled;
    } else {
      d = -step_scaled;
    }
    double ox = px.at(index);
    double oy = py.at(index);
    double ophi = pphi.at(index);
    index--;
    if (i >= 1 && shortest_path->segs_lengths.at(i - 1) *
                          shortest_path->segs_lengths.at(i) >
                      0) {
      pd = -d - ll;
    } else {
      pd = d - ll;
    }
    while (std::abs(pd) <= std::abs(l)) {
      index++;
      Interpolation(conf.max_kappa, index, pd, m, ox, oy, ophi, &px, &py, &pphi,
                    &pgear);
      pd += d;
    }
    ll = l - pd - d;
    index++;
    Interpolation(conf.max_kappa, index, l, m, ox, oy, ophi, &px, &py, &pphi,
                  &pgear);
  }
  double epsilon = 1e-15;
  while (std::fabs(px.back()) < epsilon && std::fabs(py.back()) < epsilon &&
         std::fabs(pphi.back()) < epsilon && pgear.back()) {
    px.pop_back();
    py.pop_back();
    pphi.pop_back();
    pgear.pop_back();
  }
  for (size_t i = 0; i < px.size(); ++i) {
    shortest_path->x.push_back(std::cos(-start_node.phi) * px.at(i) +
                               std::sin(-start_node.phi) * py.at(i) +
                               start_node.x);
    shortest_path->y.push_back(-std::sin(-start_node.phi) * px.at(i) +
                               std::cos(-start_node.phi) * py.at(i) +
                               start_node.y);
    shortest_path->phi.push_back(normalize_angle(pphi.at(i) + start_node.phi));
  }
  shortest_path->gear = pgear;
  for (size_t i = 0; i < shortest_path->segs_lengths.size(); ++i) {
    shortest_path->segs_lengths.at(i) =
        shortest_path->segs_lengths.at(i) / conf.max_kappa;
  }
  shortest_path->total_length = shortest_path->total_length / conf.max_kappa;
  return true;
}

std::vector<double> GenPrimitivesCost(
    const Config& conf, const std::vector<RspPath>& paths) {
  std::vector<double> ans(paths.size(), 1e7);
  for (size_t i = 0; i < paths.size(); ++i) {
    auto& [lens, types, tl, x, y, p, g] = paths[i];
    // reverse
    for (auto l : lens) if (l < 0) ans[i] += conf.cost_reverse * std::abs(l);
    // direction change
    for (size_t j = 1; j < lens.size(); ++j) {
      if ((lens[j] < 0) != (lens[j - 1] < 0)) ans[i] += conf.cost_dir_change;
    }
    // steer change
    ans[i] += conf.cost_str_change * types.size();
    // traversal
    for (auto l : lens) ans[i] += conf.cost_traversal * std::abs(l);
  }

  return ans;
}

}  // namespace

ReedsSheppPath::ReedsSheppPath(Config&& conf) : conf_{std::move(conf)} {}

bool ReedsSheppPath::ShortestRsp(const Point3d& start_node,
                                 const Point3d& end_node, RspPath* ans) {
  std::vector<RspPath> all_possible_paths;
  if (!GenerateAllPrimitives(conf_, start_node, end_node,
                             &all_possible_paths)) {
    printf("Fail to generate different combination of Reed Shepp paths\n");
    return false;
  }

  auto costs = GenPrimitivesCost(conf_, all_possible_paths);
  auto min_idx = std::distance(costs.begin(),
                               std::min_element(costs.begin(), costs.end()));
  *ans = all_possible_paths[min_idx];

  if (!GenerateLocalConfigurations(conf_, start_node, end_node, ans)) {
    printf("Fail to generate local configurations(x, y, phi) in SetRSP\n");
    return false;
  }

  if (std::abs(ans->x.back() - end_node.x) > 1e-3 ||
      std::abs(ans->y.back() - end_node.y) > 1e-3 ||
      std::abs(ans->phi.back() - end_node.phi) > 1e-3) {
    printf("RSP end position not right\n");
    std::string types(ans->segs_types.begin(), ans->segs_types.end());
    printf("%s\n", types.c_str());
    printf("x, y, phi of rs: %lf, %lf, %lf\n", ans->x.back(), ans->y.back(),
           ans->phi.back());
    printf("x, y, phi of end: %lf, %lf, %lf\n", end_node.x, end_node.y,
           end_node.phi);
    return false;
  }

  return true;
}

}  // namespace planning
}  // namespace neodrive
