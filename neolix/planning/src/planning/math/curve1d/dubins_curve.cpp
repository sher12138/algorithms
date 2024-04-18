#include "dubins_curve.h"

namespace neodrive {
namespace planning {

bool Dubins_Curve::get_dubins_trajectory(
    const ReferencePoint& start_pt, const ReferencePoint& end_pt,
    ReferencePointVec1d& dubin_traj, ReferencePointVec1d& dubin_traj_key_pts,
    const double& point_precision) {
  double turning_radius = FLAGS_planning_inlane_uturn_min_radius;
  double q0[3];
  double q1[3];

  q0[0] = start_pt.x();
  q0[1] = start_pt.y();
  q0[2] = start_pt.heading();

  q1[0] = end_pt.x();
  q1[1] = end_pt.y();
  q1[2] = end_pt.heading();

  DubinsPath path;

  if (!dubins_shortest_path(&path, q0, q1, turning_radius)) {
    return false;
  }
  dubin_traj.clear();
  dubin_traj_key_pts.clear();
  dubin_traj.push_back(start_pt);
  dubin_traj_key_pts.push_back(start_pt);
  // extract sparse points
  // first segment
  ReferencePointVec1d first_seg_pts;
  switch (path.type) {
    case DubinsPathType::LSL:
    case DubinsPathType::LSR:
    case DubinsPathType::LRL:
      if (!CalcTurnPoints(start_pt, turning_radius,
                          start_pt.heading() + path.param[0], true, true,
                          point_precision, first_seg_pts)) {
        return false;
      }
      break;
    case DubinsPathType::RSL:
    case DubinsPathType::RSR:
    case DubinsPathType::RLR:
      if (!CalcTurnPoints(start_pt, turning_radius,
                          start_pt.heading() - path.param[0], true, false,
                          point_precision, first_seg_pts)) {
        return false;
      }
      break;
    default:
      break;
  }
  if (first_seg_pts.empty()) {
    return false;
  }
  dubin_traj.insert(dubin_traj.end(), first_seg_pts.begin(),
                    first_seg_pts.end());
  dubin_traj_key_pts.push_back(dubin_traj.back());
  // second segment
  first_seg_pts.clear();
  switch (path.type) {
    case DubinsPathType::LSL:
    case DubinsPathType::LSR:
    case DubinsPathType::RSL:
    case DubinsPathType::RSR:
      if (!CalcLinePoints(dubin_traj.back(), path.param[1] * turning_radius,
                          point_precision, first_seg_pts)) {
        return false;
      }
      break;
    case DubinsPathType::RLR:
      if (!CalcTurnPoints(dubin_traj.back(), turning_radius,
                          dubin_traj.back().heading() + path.param[1], true,
                          true, point_precision, first_seg_pts)) {
        return false;
      }
      break;
    case DubinsPathType::LRL:
      if (!CalcTurnPoints(dubin_traj.back(), turning_radius,
                          dubin_traj.back().heading() - path.param[1], true,
                          false, point_precision, first_seg_pts)) {
        return false;
      }
      break;
    default:
      break;
  }
  if (first_seg_pts.empty()) {
    return false;
  }
  dubin_traj.insert(dubin_traj.end(), first_seg_pts.begin(),
                    first_seg_pts.end());
  dubin_traj_key_pts.push_back(dubin_traj.back());

  // third segment
  first_seg_pts.clear();
  switch (path.type) {
    case DubinsPathType::LSL:
    case DubinsPathType::RSL:
    case DubinsPathType::LRL:
      if (!CalcTurnPoints(dubin_traj.back(), turning_radius,
                          dubin_traj.back().heading() + path.param[2], true,
                          true, point_precision, first_seg_pts)) {
        return false;
      }
      break;
    case DubinsPathType::LSR:
    case DubinsPathType::RSR:
    case DubinsPathType::RLR:
      if (!CalcTurnPoints(dubin_traj.back(), turning_radius,
                          dubin_traj.back().heading() - path.param[2], true,
                          false, point_precision, first_seg_pts)) {
        return false;
      }
      break;
    default:
      break;
  }
  if (first_seg_pts.empty()) {
    return false;
  }
  dubin_traj.insert(dubin_traj.end(), first_seg_pts.begin(),
                    first_seg_pts.end());
  dubin_traj.push_back(end_pt);
  dubin_traj_key_pts.push_back(dubin_traj.back());

  for (std::size_t i = 0; i < dubin_traj.size(); ++i) {
    if (i == 0) {
      dubin_traj[i].set_s(0.0);
    } else {
      dubin_traj[i].set_s(
          dubin_traj[i - 1].s() +
          std::sqrt(std::pow(dubin_traj[i].x() - dubin_traj[i - 1].x(), 2) +
                    std::pow(dubin_traj[i].y() - dubin_traj[i - 1].y(), 2)));
    }
  }
  return true;
}

double Dubins_Curve::fmodr(const double& x, const double& y) {
  return x - y * floor(x / y);
}

double Dubins_Curve::mod2pi(const double& theta) {
  return fmodr(theta, 2 * M_PI);
}

bool Dubins_Curve::dubins_shortest_path(DubinsPath* path, double q0[3],
                                        double q1[3], const double& rho) {
  bool errcode = false;
  if (path == nullptr) return errcode;

  DubinsIntermediateResults in;
  double params[3];
  double cost;
  double best_cost = std::numeric_limits<double>::max();
  int best_word = -1;
  errcode = dubins_intermediate_results(&in, q0, q1, rho);
  if (!errcode) {
    return errcode;
  }

  path->qi[0] = q0[0];
  path->qi[1] = q0[1];
  path->qi[2] = q0[2];
  path->rho = rho;

  for (int i = 0; i < 6; ++i) {
    DubinsPathType pathType = (DubinsPathType)i;
    errcode = dubins_word(&in, pathType, params);
    if (errcode) {
      cost = params[0] + params[1] + params[2];
      if (cost < best_cost) {
        best_word = i;
        best_cost = cost;
        path->param[0] = params[0];
        path->param[1] = params[1];
        path->param[2] = params[2];
        path->type = pathType;
      }
    }
  }
  if (best_word == -1) {
    return false;
  }
  return true;
}

bool Dubins_Curve::dubins_path(DubinsPath* path, double q0[3], double q1[3],
                               const double& rho,
                               const DubinsPathType& pathType) {
  bool errcode = false;
  if (path == nullptr) return errcode;
  DubinsIntermediateResults in;
  errcode = dubins_intermediate_results(&in, q0, q1, rho);
  if (errcode) {
    double params[3];
    errcode = dubins_word(&in, pathType, params);
    if (errcode) {
      path->param[0] = params[0];
      path->param[1] = params[1];
      path->param[2] = params[2];
      path->qi[0] = q0[0];
      path->qi[1] = q0[1];
      path->qi[2] = q0[2];
      path->rho = rho;
      path->type = pathType;
    }
  }
  return errcode;
}

double Dubins_Curve::dubins_path_length(const DubinsPath* path) {
  double length = 0.;
  if (path == nullptr) return length;
  length += path->param[0];
  length += path->param[1];
  length += path->param[2];
  length = length * path->rho;
  return length;
}

void Dubins_Curve::dubins_segment(const double& t, double qi[3], double qt[3],
                                  const SegmentType& type) {
  double st = sin(qi[2]);
  double ct = cos(qi[2]);
  if (type == L_SEG) {
    qt[0] = +sin(qi[2] + t) - st;
    qt[1] = -cos(qi[2] + t) + ct;
    qt[2] = t;
  } else if (type == R_SEG) {
    qt[0] = -sin(qi[2] - t) + st;
    qt[1] = +cos(qi[2] - t) - ct;
    qt[2] = -t;
  } else if (type == S_SEG) {
    qt[0] = ct * t;
    qt[1] = st * t;
    qt[2] = 0.0;
  }
  qt[0] += qi[0];
  qt[1] += qi[1];
  qt[2] += qi[2];
}

bool Dubins_Curve::dubins_segment_length(DubinsPath* path, const int& i,
                                         double& length) {
  if (path == nullptr || (i < 0) || (i > 2)) {
    return false;
  }
  length = path->param[i] * path->rho;
  return true;
}

bool Dubins_Curve::dubins_segment_length_normalized(DubinsPath* path,
                                                    const int& i,
                                                    double& length) {
  if (path == nullptr || (i < 0) || (i > 2)) {
    return false;
  }
  length = path->param[i];
  return true;
}

bool Dubins_Curve::dubins_path_type(const DubinsPath* path,
                                    DubinsPathType& type) {
  if (path == nullptr) {
    return false;
  }
  type = path->type;
  return true;
}

bool Dubins_Curve::dubins_path_sample(const DubinsPath* path, const double& t,
                                      double q[3]) {
  if (path == nullptr || path->rho <= 1.0e-3) {
    // invalid parameter
    return false;
  }
  /* tprime is the normalised variant of the parameter t */
  double tprime = t / path->rho;
  double qi[3]; /* The translated initial configuration */
  double q1[3]; /* end-of segment 1 */
  double q2[3]; /* end-of segment 2 */

  /* The segment types for each of the Path types */
  const SegmentType DIRDATA[][3] = {
      {L_SEG, S_SEG, L_SEG}, {L_SEG, S_SEG, R_SEG}, {R_SEG, S_SEG, L_SEG},
      {R_SEG, S_SEG, R_SEG}, {R_SEG, L_SEG, R_SEG}, {L_SEG, R_SEG, L_SEG}};
  const SegmentType* types = DIRDATA[path->type];
  double p1, p2;

  if (t < 0 || t > dubins_path_length(path)) {
    // invalid parameter
    return false;
  }

  /* initial configuration */
  qi[0] = 0.0;
  qi[1] = 0.0;
  qi[2] = path->qi[2];

  /* generate the target configuration */
  p1 = path->param[0];
  p2 = path->param[1];
  dubins_segment(p1, qi, q1, types[0]);
  dubins_segment(p2, q1, q2, types[1]);
  if (tprime < p1) {
    dubins_segment(tprime, qi, q, types[0]);
  } else if (tprime < (p1 + p2)) {
    dubins_segment(tprime - p1, q1, q, types[1]);
  } else {
    dubins_segment(tprime - p1 - p2, q2, q, types[2]);
  }

  /* scale the target configuration, translate back to the original starting
   * point */
  q[0] = q[0] * path->rho + path->qi[0];
  q[1] = q[1] * path->rho + path->qi[1];
  q[2] = mod2pi(q[2]);

  return true;
}

bool Dubins_Curve::dubins_path_endpoint(const DubinsPath* path, double q[3]) {
  if (path == nullptr) return false;
  return dubins_path_sample(path, dubins_path_length(path) - epsilon, q);
}

bool Dubins_Curve::dubins_extract_subpath(const DubinsPath* path,
                                          const double& t,
                                          DubinsPath* newpath) {
  /* calculate the true parameter */
  if (path == nullptr || newpath == nullptr || path->rho <= 1.0e-3) {
    // parameter invalid
    return false;
  }
  double tprime = t / path->rho;

  if ((t < 0) || (t > dubins_path_length(path))) {
    // parameter invalid
    return false;
  }

  /* copy most of the data */
  newpath->qi[0] = path->qi[0];
  newpath->qi[1] = path->qi[1];
  newpath->qi[2] = path->qi[2];
  newpath->rho = path->rho;
  newpath->type = path->type;

  /* fix the parameters */
  newpath->param[0] = fmin(path->param[0], tprime);
  newpath->param[1] = fmin(path->param[1], tprime - newpath->param[0]);
  newpath->param[2] =
      fmin(path->param[2], tprime - newpath->param[0] - newpath->param[1]);
  return true;
}

bool Dubins_Curve::dubins_intermediate_results(DubinsIntermediateResults* in,
                                               double q0[3], double q1[3],
                                               const double& rho) {
  if (in == nullptr) return false;
  double dx, dy, D, d, theta, alpha, beta;
  if (rho <= 0.0) {
    // the rho value is invalid
    return false;
  }

  dx = q1[0] - q0[0];
  dy = q1[1] - q0[1];
  D = std::sqrt(dx * dx + dy * dy);
  d = D / rho;
  theta = 0;

  /* test required to prevent domain errors if dx=0 and dy=0 */
  if (d > 0) {
    theta = mod2pi(atan2(dy, dx));
  }
  alpha = mod2pi(q0[2] - theta);
  beta = mod2pi(q1[2] - theta);

  in->alpha = alpha;
  in->beta = beta;
  in->d = d;
  in->sa = sin(alpha);
  in->sb = sin(beta);
  in->ca = cos(alpha);
  in->cb = cos(beta);
  in->c_ab = cos(alpha - beta);
  in->d_sq = d * d;

  return true;
}

bool Dubins_Curve::dubins_word(const DubinsIntermediateResults* in,
                               const DubinsPathType& pathType, double out[3]) {
  if (in == nullptr) return false;
  bool result = false;
  switch (pathType) {
    case LSL:
      result = dubins_LSL(in, out);
      break;
    case RSL:
      result = dubins_RSL(in, out);
      break;
    case LSR:
      result = dubins_LSR(in, out);
      break;
    case RSR:
      result = dubins_RSR(in, out);
      break;
    case LRL:
      result = dubins_LRL(in, out);
      break;
    case RLR:
      result = dubins_RLR(in, out);
      break;
    default:
      // no connection between configurations with this word
      result = false;
  }
  return result;
}

bool Dubins_Curve::dubins_LSL(const DubinsIntermediateResults* in,
                              double out[3]) {
  if (in == nullptr) {
    return false;
  }
  double tmp0, tmp1, p_sq;

  tmp0 = in->d + in->sa - in->sb;
  p_sq = 2 + in->d_sq - (2 * in->c_ab) + (2 * in->d * (in->sa - in->sb));

  if (p_sq >= 0) {
    tmp1 = atan2((in->cb - in->ca), tmp0);
    out[0] = mod2pi(tmp1 - in->alpha);
    out[1] = sqrt(p_sq);
    out[2] = mod2pi(in->beta - tmp1);
    return true;
  }
  // no connection between configurations with this word
  return false;
}

bool Dubins_Curve::dubins_RSR(const DubinsIntermediateResults* in,
                              double out[3]) {
  if (in == nullptr) {
    return false;
  }
  double tmp0 = in->d - in->sa + in->sb;
  double p_sq = 2 + in->d_sq - (2 * in->c_ab) + (2 * in->d * (in->sb - in->sa));
  if (p_sq >= 0) {
    double tmp1 = atan2((in->ca - in->cb), tmp0);
    out[0] = mod2pi(in->alpha - tmp1);
    out[1] = sqrt(p_sq);
    out[2] = mod2pi(tmp1 - in->beta);
    return true;
  }
  // no connection between configurations with this word
  return false;
}

bool Dubins_Curve::dubins_LSR(const DubinsIntermediateResults* in,
                              double out[3]) {
  if (in == nullptr) {
    return false;
  }
  double p_sq =
      -2 + (in->d_sq) + (2 * in->c_ab) + (2 * in->d * (in->sa + in->sb));
  if (p_sq >= 0) {
    double p = sqrt(p_sq);
    double tmp0 =
        atan2((-in->ca - in->cb), (in->d + in->sa + in->sb)) - atan2(-2.0, p);
    out[0] = mod2pi(tmp0 - in->alpha);
    out[1] = p;
    out[2] = mod2pi(tmp0 - mod2pi(in->beta));
    return true;
  }
  // no connection between configurations with this word
  return false;
}

bool Dubins_Curve::dubins_RSL(const DubinsIntermediateResults* in,
                              double out[3]) {
  if (in == nullptr) {
    return false;
  }
  double p_sq =
      -2 + in->d_sq + (2 * in->c_ab) - (2 * in->d * (in->sa + in->sb));
  if (p_sq >= 0) {
    double p = sqrt(p_sq);
    double tmp0 =
        atan2((in->ca + in->cb), (in->d - in->sa - in->sb)) - atan2(2.0, p);
    out[0] = mod2pi(in->alpha - tmp0);
    out[1] = p;
    out[2] = mod2pi(in->beta - tmp0);
    return true;
  }
  // no connection between configurations with this word
  return false;
}

bool Dubins_Curve::dubins_RLR(const DubinsIntermediateResults* in,
                              double out[3]) {
  if (in == nullptr) {
    return false;
  }
  double tmp0 =
      (6. - in->d_sq + 2 * in->c_ab + 2 * in->d * (in->sa - in->sb)) / 8.;
  double phi = atan2(in->ca - in->cb, in->d - in->sa + in->sb);
  if (fabs(tmp0) <= 1) {
    double p = mod2pi((2 * M_PI) - acos(tmp0));
    double t = mod2pi(in->alpha - phi + mod2pi(p / 2.));
    out[0] = t;
    out[1] = p;
    out[2] = mod2pi(in->alpha - in->beta - t + mod2pi(p));
    return true;
  }
  // no connection between configurations with this word
  return false;
}

bool Dubins_Curve::dubins_LRL(const DubinsIntermediateResults* in,
                              double out[3]) {
  if (in == nullptr) {
    return false;
  }
  double tmp0 =
      (6. - in->d_sq + 2 * in->c_ab + 2 * in->d * (in->sb - in->sa)) / 8.;
  double phi = atan2(in->ca - in->cb, in->d + in->sa - in->sb);
  if (fabs(tmp0) <= 1) {
    double p = mod2pi(2 * M_PI - acos(tmp0));
    double t = mod2pi(-in->alpha - phi + p / 2.);
    out[0] = t;
    out[1] = p;
    out[2] = mod2pi(mod2pi(in->beta) - in->alpha - t + mod2pi(p));
    return true;
  }
  // no connection between configurations with this word
  return false;
}

bool Dubins_Curve::CalcTurnPoints(const ReferencePoint& start_pos,
                                  const double& radius,
                                  const double& end_heading,
                                  const bool& is_forward,
                                  const bool& left_right_turn,
                                  const double& point_precision,
                                  ReferencePointVec1d& pts_vec) {
  pts_vec.clear();
  if (radius < 1.0e-1 || point_precision < 1.e-3) return false;

  ReferencePoint tmp_pt;
  double p = point_precision / radius;
  p = std::min(0.1, p);

  if (left_right_turn) {  // left
    for (double heading1 = start_pos.heading() + p; heading1 < end_heading;
         heading1 += p) {
      if (!CalcDestinationOnArc(start_pos, radius, heading1, is_forward,
                                left_right_turn, &tmp_pt)) {
        return false;
      }
      pts_vec.push_back(tmp_pt);
    }
  } else {  // right
    for (double heading1 = start_pos.heading() - p; heading1 > end_heading;
         heading1 -= p) {
      if (!CalcDestinationOnArc(start_pos, radius, heading1, is_forward,
                                left_right_turn, &tmp_pt)) {
        return false;
      }
      pts_vec.push_back(tmp_pt);
    }
  }

  return true;
}

// is_forward:  false: backward; left_right_turn: true- left turn; false-right
// turn;
bool Dubins_Curve::CalcDestinationOnArc(const ReferencePoint& start_pos,
                                        const double radius,
                                        const double end_heading,
                                        const bool is_forward,
                                        const bool left_right_turn,
                                        ReferencePoint* const end_pos) {
  if (end_pos == nullptr) {
    return false;
  }
  Vec2d start_vec(start_pos.x(), start_pos.y());

  const double heading_diff =
      normalize_angle(end_heading - start_pos.heading());
  // 8 conditions
  /* heading diff==left+/right-==forward+/backward-===
   *   + - - O
   *  + - + X
   * + + - X
   * + + + O
   * - - - X
   * - - + O
   * - + - O
   * - + + X
   */
  if (heading_diff > 0.0 && !left_right_turn && !is_forward) {
    // right/backward
    const double normal_heading =
        normalize_angle(start_pos.heading() - M_PI / 2);
    Vec2d arc_center =
        Vec2d::create_unit_vec(normal_heading) * radius + start_vec;
    const double heading =
        normalize_angle(normal_heading + M_PI + heading_diff);
    Vec2d dest_pos = Vec2d::create_unit_vec(heading) * radius + arc_center;
    end_pos->set_x(dest_pos.x());
    end_pos->set_y(dest_pos.y());
    end_pos->set_heading(end_heading);
  } else if (heading_diff > 0.0 && !left_right_turn && is_forward) {
    // right/forward
    // LOG_ERROR("donot support this type");
    return false;
  } else if (heading_diff > 0.0 && left_right_turn && !is_forward) {
    // left/backward
    // LOG_ERROR("donot support this type");
    return false;
  } else if (heading_diff > 0.0 && left_right_turn && is_forward) {
    // left/forward
    const double normal_heading =
        normalize_angle(start_pos.heading() + M_PI / 2);
    Vec2d arc_center =
        Vec2d::create_unit_vec(normal_heading) * radius + start_vec;
    const double heading =
        normalize_angle(normal_heading + M_PI + heading_diff);
    Vec2d dest_pos = Vec2d::create_unit_vec(heading) * radius + arc_center;
    end_pos->set_x(dest_pos.x());
    end_pos->set_y(dest_pos.y());
    end_pos->set_heading(end_heading);
  } else if (heading_diff < 0.0 && !left_right_turn && !is_forward) {
    // right/backward
    // LOG_ERROR("donot support this type");
    return false;
  } else if (heading_diff < 0.0 && !left_right_turn && is_forward) {
    // right/forward
    const double normal_heading =
        normalize_angle(start_pos.heading() - M_PI / 2);
    Vec2d arc_center =
        Vec2d::create_unit_vec(normal_heading) * radius + start_vec;
    const double heading =
        normalize_angle(normal_heading + M_PI + heading_diff);
    Vec2d dest_pos = Vec2d::create_unit_vec(heading) * radius + arc_center;
    end_pos->set_x(dest_pos.x());
    end_pos->set_y(dest_pos.y());
    end_pos->set_heading(end_heading);
  } else if (heading_diff < 0.0 && left_right_turn && !is_forward) {
    // left/forward
    const double normal_heading =
        normalize_angle(start_pos.heading() + M_PI / 2);
    Vec2d arc_center =
        Vec2d::create_unit_vec(normal_heading) * radius + start_vec;
    const double heading =
        normalize_angle(normal_heading + M_PI + heading_diff);
    Vec2d dest_pos = Vec2d::create_unit_vec(heading) * radius + arc_center;
    end_pos->set_x(dest_pos.x());
    end_pos->set_y(dest_pos.y());
    end_pos->set_heading(end_heading);
  } else if (heading_diff < 0.0 && left_right_turn && is_forward) {
    // left/backward
    // LOG_ERROR("donot support this type");
    return false;
  }

  return true;
}

bool Dubins_Curve::CalcLinePoints(const ReferencePoint& start_pos,
                                  const double& extend_dis,
                                  const double& point_precision,
                                  ReferencePointVec1d& pts_vec) {
  pts_vec.clear();
  Vec2d end_pos(start_pos.x(), start_pos.y());
  const double end_heading = start_pos.heading();
  const auto unit_vec = Vec2d::create_unit_vec(end_heading);
  ReferencePoint ref_point = start_pos;

  for (double s = 0.0; s < extend_dis; s += point_precision) {
    Vec2d step_pos = unit_vec * 1 * point_precision + end_pos;  // 1: forward
    ref_point.set_x(step_pos.x());
    ref_point.set_y(step_pos.y());
    ref_point.set_heading(end_heading);
    pts_vec.push_back(ref_point);
    end_pos = step_pos;
  }

  return true;
}

}  // namespace planning
}  // namespace neodrive
