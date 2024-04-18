#ifndef POLY_TRAJ_STRUCTS_H_
#define POLY_TRAJ_STRUCTS_H_

#include <inttypes.h>
//#include <driving_common/Trajectory2D.h>

struct movement_state {
  double x;
  double x_der;
  double x_dder;
};

struct trajectory_point_1D {
  double arg;      // argument (e.g. time t or arc length s)
  double x;        // value
  double x_der;    // first derivative with respect to arg
  double x_dder;   // second derivative
  double x_ddder;  // third
};

struct TrajectoryPoint2D {
  double t;
  double x;
  double y;
  double theta;
  double kappa;
  double kappa_dot;
  double v;
  double a;
  double jerk;
  double d;
  double delta_theta;
  double a_lat;
};

struct velocity_params {
  double t_horizon;
  double a_max;
  double a_min;
  double time_res;
  double time_max;
  double arclength_res;
  double v_res;
  double v_offset_max;
  double v_offset_min;
  double k_t;
  double k_j;
  double k_v;
  double time_sample_res;
};

struct tracking_params {
  double t_horizon;
  double s_res;
  double s_offset_max;
  double s_offset_min;
  double a_max;
  double a_min;
  double time_res;
  double time_max;
  double k_t;
  double k_j;
  double k_s;
  double time_sample_res;
};

struct lanekeeping_params {
  double t_horizon;
  double d_res;
  double d_offset_max;
  double d_offset_min;

  struct {
    double d_ddot_max;
    double d_ddot_min;
    double time_res;
    double time_max;
    double k_t;
    double k_j;
    double k_d;
    double time_sample_res;
  } holon;
  struct {
    double d_pprime_max;
    double d_pprime_min;
    double s_res;
    double s_max;
    double k_s;
    double k_j;
    double k_d;
    double arclength_sample_res;
  } nonhol;
};

struct PolyTraj2D_params {
  double max_curvature;
  double max_center_line_offset;
  double max_center_line_angular_offset;
  double max_lat_acceleration;
  double max_lon_acceleration;
  double min_lon_acceleration;
  double time_delay;
};

struct CurvePoint {
  double s;
  double x;
  double y;
  double theta;
  double kappa;
  double kappa_prime;
  double v;
  double t;
};

class Circle {
 public:
  Circle() {}
  Circle(double x_, double y_, double r_) : x(x_), y(y_), r(r_) {}
  virtual ~Circle() {}

  double x, y, r;
};

struct MovingBox {
  double width;
  double length;
  double ref_offset;
  double t;
  double x;
  double y;
  double psi;

  static const uint32_t num_circles_ = 4;
  Circle circles_[num_circles_];
  Circle circum_circle_;
};

struct InitialStateFakePerception {
  double phi;
  double v;
  double r;
  double x;  // center of circle
  double y;  // center of circle
};

typedef enum { time_based = 1, arclength_based = 2 } GenerationMode;

#endif  // POLY_TRAJ_STRUCTS_H_
