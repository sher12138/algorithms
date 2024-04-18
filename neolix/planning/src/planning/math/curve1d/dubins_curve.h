#pragma once

#include <math.h>

#include <limits>

#include "common/math/math_utils.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/reference_line/reference_point.h"

namespace neodrive {
namespace planning {

typedef enum {
  LSL = 0,
  LSR = 1,
  RSL = 2,
  RSR = 3,
  RLR = 4,
  LRL = 5
} DubinsPathType;

typedef struct {
  /* the initial configuration */
  double qi[3];
  /* the lengths of the three segments */
  double param[3];
  /* model forward velocity / model angular velocity */
  double rho;
  /* the path type described */
  DubinsPathType type;
} DubinsPath;

class Dubins_Curve {
 public:
  Dubins_Curve() = default;
  ~Dubins_Curve() = default;

  bool get_dubins_trajectory(const ReferencePoint& start_pt,
                             const ReferencePoint& end_pt,
                             ReferencePointVec1d& dubin_traj,
                             ReferencePointVec1d& dubin_traj_key_pts,
                             const double& point_precision = 0.2);

  /**
   * Generate a path from an initial configuration to
   * a target configuration, with a specified maximum turning
   * radius
   *
   * A configuration is (x, y, theta), where theta is in radians, with zero
   * along the line x = 0, and counter-clockwise is positive
   *
   * @param path  - the resultant path
   * @param q0    - a configuration specified as an array of x, y, theta
   * @param q1    - a configuration specified as an array of x, y, theta
   * @param rho   - turning radius of the vehicle (forward velocity divided by
   * maximum angular velocity)
   * @return      - non-zero on error
   */
  bool dubins_shortest_path(DubinsPath* path, double q0[3], double q1[3],
                            const double& rho);

 private:
  typedef struct {
    double alpha;
    double beta;
    double d;
    double sa;
    double sb;
    double ca;
    double cb;
    double c_ab;
    double d_sq;
  } DubinsIntermediateResults;

  typedef enum { L_SEG = 0, S_SEG = 1, R_SEG = 2 } SegmentType;

  // given a type, build path success or not
  bool dubins_path(DubinsPath* path, double q0[3], double q1[3],
                   const double& rho, const DubinsPathType& pathType);
  // length of path
  double dubins_path_length(const DubinsPath* path);

  void dubins_segment(const double& t, double qi[3], double qt[3],
                      const SegmentType& type);
  // i- the length of segment, [0<= i <=2]
  bool dubins_segment_length(DubinsPath* path, const int& i, double& length);
  bool dubins_segment_length_normalized(DubinsPath* path, const int& i,
                                        double& length);

  // one of LSL, LSR, RSL, RSR, RLR or LRL
  bool dubins_path_type(const DubinsPath* path, DubinsPathType& type);
  /*
   * using the parameter t, calculate the configuration along the path,
   * t: length measure, 0 <= t < dubins_path_length(path)
   * q: configuration result
   * non-zero if 't' is not in the correct range
   */
  bool dubins_path_sample(const DubinsPath* path, const double& t, double q[3]);
  /*
   * function to identify the endpoint of a path
   * path: an initialised path
   * q: the configuration result
   */
  bool dubins_path_endpoint(const DubinsPath* path, double q[3]);
  /*
   * function to extract a subset of a path
   * path: an initialised path
   * t: length measure, where 0 < t < dubins_path_length(path)
   * newpath: the resultant path
   */
  bool dubins_extract_subpath(const DubinsPath* path, const double& t,
                              DubinsPath* newpath);

  bool dubins_intermediate_results(DubinsIntermediateResults* in, double q0[3],
                                   double q1[3], const double& rho);

  bool dubins_word(const DubinsIntermediateResults* in,
                   const DubinsPathType& pathType, double out[3]);
  bool dubins_LRL(const DubinsIntermediateResults* in, double out[3]);
  bool dubins_RLR(const DubinsIntermediateResults* in, double out[3]);
  bool dubins_RSL(const DubinsIntermediateResults* in, double out[3]);
  bool dubins_LSR(const DubinsIntermediateResults* in, double out[3]);
  bool dubins_RSR(const DubinsIntermediateResults* in, double out[3]);
  bool dubins_LSL(const DubinsIntermediateResults* in, double out[3]);

  /*
   * Floating point modulus suitable for rings
   * fmod doesn't behave correctly for angular quantities, this function does
   */
  double fmodr(const double& x, const double& y);
  double mod2pi(const double& theta);

  bool CalcTurnPoints(const ReferencePoint& start_pos, const double& radius,
                      const double& end_heading, const bool& is_forward,
                      const bool& left_right_turn,
                      const double& point_precision,
                      ReferencePointVec1d& pts_vec);

  bool CalcLinePoints(const ReferencePoint& start_pos, const double& extend_dis,
                      const double& point_precision,
                      ReferencePointVec1d& pts_vec);

  bool CalcDestinationOnArc(const ReferencePoint& start_pos,
                            const double radius, const double end_heading,
                            const bool is_forward, const bool left_right_turn,
                            ReferencePoint* const end_pos);

 private:
  const double epsilon = 10e-10;
};

}  // namespace planning
}  // namespace neodrive
