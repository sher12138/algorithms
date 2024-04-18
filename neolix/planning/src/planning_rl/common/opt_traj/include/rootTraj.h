#ifndef ROOTTRAJ_H_
#define ROOTTRAJ_H_

#include <map>
#include <vector>
#include "poly_traj_structs.h"

#include <math.h>
#include "polyTraj.h"

class RootTraj : public PolyTraj {
 public:
  std::vector<CurvePoint>
      foot_curve_points_;  // sampled at the required positions
  std::vector<CurvePoint>
      center_line_;  // discrete representation of the center curve

  RootTraj(const std::vector<CurvePoint>& center_line);
  void calculateRootTrajectory();
  void evalCenterlineAtS(const double& s_interpol,
                         CurvePoint& cp_interpol) const;
  static void evalCenterlineAtS_static(
      const std::vector<CurvePoint>& center_line, double s_interpol,
      CurvePoint& cp_interpol);
  static inline double normalize_angle_positive(const double& angle) {
    return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
  }
  static inline double normalize_angle(const double& angle) {
    double a = normalize_angle_positive(angle);
    if (a > M_PI) a -= 2.0 * M_PI;
    return a;
  }

 private:
  struct CompareCurvePoints {
    bool operator()(const CurvePoint& cp1, const CurvePoint& cp2) {
      return cp1.s < cp2.s;
    }
  };
  static CompareCurvePoints cp_comp_;
};

#endif  // ROOTTRAJ_H_
