#include "obs_utils.h"
#include <math.h>
#include <algorithm>
#include <string>
#include <tuple>
#include <vector>

namespace neodrive {
namespace planning_rl {

const double pi = M_PI;

Near100SRet FindNearest100(
    const std::vector<std::vector<double>> &refline_points,
    const std::vector<double> &refline_points_curvature,
    const std::vector<double> &thetas, const std::vector<double> &adc_point) {
  // 取子车方向 ，前70个点 + 后30个点

  int nearest_index = 0;
  double min_dis = std::numeric_limits<double>::max();
  int ref_length = refline_points.size();
  for (int i = 0; i < ref_length; i++) {
    double diff_x = adc_point[0] - refline_points[i][0];
    double diff_y = adc_point[1] - refline_points[i][1];

    double dis = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
    if (dis <= min_dis) {
      nearest_index = i;
      min_dis = dis;
    }
  }
  std::vector<std::vector<double>> nearest_100_refpoint;
  std::vector<std::vector<double>> back_refpoints;
  std::vector<std::vector<double>> front_refpoints;

  std::vector<double> nearest_100_refpoint_curvature;
  std::vector<double> back_refpoints_curvature;
  std::vector<double> front_refpoints_curvature;
  double curvature = 0.0;

  std::vector<double> nearest_100_refpoint_theta;
  std::vector<double> back_refpoints_theta;
  std::vector<double> front_refpoints_theta;

  if (ref_length > 1) {
    curvature = refline_points_curvature[nearest_index];
    // 截取后向
    if (nearest_index >= 30) {
      back_refpoints.insert(back_refpoints.end(),
                            refline_points.begin() + nearest_index - 30,
                            refline_points.begin() + nearest_index);
      back_refpoints_curvature.insert(
          back_refpoints_curvature.end(),
          refline_points_curvature.begin() + nearest_index - 30,
          refline_points_curvature.begin() + nearest_index);
      back_refpoints_theta.insert(back_refpoints_theta.end(),
                                  thetas.begin() + nearest_index - 30,
                                  thetas.begin() + nearest_index);
    } else {
      // 补refline曲线
      auto start_point = refline_points[0];
      double theta = thetas[1] + pi;
      int point_num = 30 - nearest_index;
      curvature = refline_points_curvature[1];
      std::vector<std::vector<double>> support_refline_points =
          GetSupportPoint(curvature, start_point, theta, point_num);

      std::reverse(support_refline_points.begin(),
                   support_refline_points.end());
      back_refpoints.insert(back_refpoints.end(),
                            support_refline_points.begin(),
                            support_refline_points.end());
      back_refpoints.insert(back_refpoints.end(), refline_points.begin(),
                            refline_points.begin() + nearest_index);
      int length_srp = support_refline_points.size();

      std::vector<double> tmp_curvature(length_srp, curvature);
      back_refpoints_curvature.insert(back_refpoints_curvature.end(),
                                      tmp_curvature.begin(),
                                      tmp_curvature.end());
      back_refpoints_curvature.insert(
          back_refpoints_curvature.end(), refline_points_curvature.begin(),
          refline_points_curvature.begin() + nearest_index);

      std::vector<double> tmp_theta(length_srp, theta);
      back_refpoints_theta.insert(back_refpoints_theta.end(), tmp_theta.begin(),
                                  tmp_theta.end());
      back_refpoints_theta.insert(back_refpoints_theta.end(), thetas.begin(),
                                  thetas.begin() + nearest_index);
    }
    // 截取前向
    if ((ref_length - nearest_index) >= 70) {
      front_refpoints.insert(front_refpoints.end(),
                             refline_points.begin() + nearest_index,
                             refline_points.begin() + nearest_index + 70);
      front_refpoints_curvature.insert(
          front_refpoints_curvature.end(),
          refline_points_curvature.begin() + nearest_index,
          refline_points_curvature.begin() + nearest_index + 70);
      front_refpoints_theta.insert(front_refpoints_theta.end(),
                                   thetas.begin() + nearest_index,
                                   thetas.begin() + nearest_index + 70);
    } else {
      // 补refline曲线
      auto start_point = refline_points.back();
      double theta = thetas[thetas.size() - 2];
      int point_num = 70 - (ref_length - nearest_index);
      curvature = refline_points_curvature[refline_points_curvature.size() - 2];
      std::vector<std::vector<double>> support_refline_points =
          GetSupportPoint(curvature, start_point, theta, point_num);

      front_refpoints.insert(front_refpoints.end(),
                             refline_points.begin() + nearest_index,
                             refline_points.end());
      front_refpoints.insert(front_refpoints.end(),
                             support_refline_points.begin(),
                             support_refline_points.end());
      int length_srp = support_refline_points.size();

      front_refpoints_curvature.insert(
          front_refpoints_curvature.end(),
          refline_points_curvature.begin() + nearest_index,
          refline_points_curvature.end());
      std::vector<double> tmp_curvature(length_srp, curvature);
      front_refpoints_curvature.insert(front_refpoints_curvature.end(),
                                       tmp_curvature.begin(),
                                       tmp_curvature.end());

      front_refpoints_theta.insert(front_refpoints_theta.end(),
                                   thetas.begin() + nearest_index,
                                   thetas.end());
      std::vector<double> tmp_theta(length_srp, theta);
      front_refpoints_theta.insert(front_refpoints_theta.end(),
                                   tmp_theta.begin(), tmp_theta.end());
    }

    nearest_100_refpoint.insert(nearest_100_refpoint.end(),
                                back_refpoints.begin(), back_refpoints.end());
    nearest_100_refpoint.insert(nearest_100_refpoint.end(),
                                front_refpoints.begin(), front_refpoints.end());

    nearest_100_refpoint_curvature.insert(nearest_100_refpoint_curvature.end(),
                                          back_refpoints_curvature.begin(),
                                          back_refpoints_curvature.end());
    nearest_100_refpoint_curvature.insert(nearest_100_refpoint_curvature.end(),
                                          front_refpoints_curvature.begin(),
                                          front_refpoints_curvature.end());

    nearest_100_refpoint_theta.insert(nearest_100_refpoint_theta.end(),
                                      back_refpoints_theta.begin(),
                                      back_refpoints_theta.end());
    nearest_100_refpoint_theta.insert(nearest_100_refpoint_theta.end(),
                                      front_refpoints_theta.begin(),
                                      front_refpoints_theta.end());
  }

  Near100SRet ret = Near100SRet(
      nearest_100_refpoint, nearest_100_refpoint_curvature,
      nearest_100_refpoint_theta, curvature, refline_points[nearest_index]);

  return ret;
}

std::vector<std::vector<double>> FindNearest100Meter(
    const std::vector<std::vector<double>> &refline_points,
    const std::vector<double> &refline_points_curvature,
    const std::vector<double> &thetas, const std::vector<double> &adc_point) {
  // 取子车方向 ，前70个点 + 后30个点

  int nearest_index = 0;
  double min_dis = std::numeric_limits<double>::max();
  int ref_length = refline_points.size();
  for (int i = 0; i < ref_length; i++) {
    double diff_x = adc_point[0] - refline_points[i][0];
    double diff_y = adc_point[1] - refline_points[i][1];

    double dis = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
    if (dis <= min_dis) {
      nearest_index = i;
      min_dis = dis;
    }
  }
  std::vector<std::vector<double>> nearest_100m_refpoint;
  std::vector<std::vector<double>> back_refpoints;
  std::vector<std::vector<double>> front_refpoints;

  if (ref_length <= 0) {
    return nearest_100m_refpoint;
  }

  double curvature = refline_points_curvature[nearest_index];

  // 截取后向
  if (nearest_index >= 150) {
    back_refpoints.insert(back_refpoints.end(),
                          refline_points.begin() + nearest_index - 150,
                          refline_points.begin() + nearest_index);
  } else {
    // 补refline曲线
    auto start_point = refline_points[0];
    double theta = thetas[1] + pi;
    int point_num = 150 - nearest_index;
    curvature = refline_points_curvature[1];
    std::vector<std::vector<double>> support_refline_points =
        GetSupportPoint(curvature, start_point, theta, point_num);

    std::reverse(support_refline_points.begin(), support_refline_points.end());
    back_refpoints.insert(back_refpoints.end(), support_refline_points.begin(),
                          support_refline_points.end());
    back_refpoints.insert(back_refpoints.end(), refline_points.begin(),
                          refline_points.begin() + nearest_index);
  }
  // 截取前向
  if ((ref_length - nearest_index) >= 350) {
    front_refpoints.insert(front_refpoints.end(),
                           refline_points.begin() + nearest_index,
                           refline_points.begin() + nearest_index + 350);
  } else {
    // 补refline曲线
    auto start_point = refline_points.back();
    double theta = thetas[thetas.size() - 2];
    int point_num = 350 - (ref_length - nearest_index);
    curvature = refline_points_curvature[refline_points_curvature.size() - 2];
    std::vector<std::vector<double>> support_refline_points =
        GetSupportPoint(curvature, start_point, theta, point_num);

    front_refpoints.insert(front_refpoints.end(),
                           refline_points.begin() + nearest_index,
                           refline_points.end());
    front_refpoints.insert(front_refpoints.end(),
                           support_refline_points.begin(),
                           support_refline_points.end());
  }
  nearest_100m_refpoint.insert(nearest_100m_refpoint.end(),
                               back_refpoints.begin(), back_refpoints.end());
  nearest_100m_refpoint.insert(nearest_100m_refpoint.end(),
                               front_refpoints.begin(), front_refpoints.end());

  return nearest_100m_refpoint;
}

std::tuple<std::vector<ReferencePoint>, std::vector<ReferencePoint>> FindNearest110MeterSample(
    const std::vector<ReferencePoint> &refline_points,
    const std::vector<double> &adc_point, int sample_num) {
  bool is_use_strength = false;
  int nearest_index = 0;
  double min_dis = std::numeric_limits<double>::max();
  int ref_length = refline_points.size();
  for (int i = 0; i < ref_length; i++) {
    double diff_x = adc_point[0] - refline_points[i].x();
    double diff_y = adc_point[1] - refline_points[i].y();

    double dis = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
    if (dis <= min_dis) {
      nearest_index = i;
      min_dis = dis;
    }
  }
  // for (int i = 0; i < ref_length; i++) {
  //   std::cout << refline_points[i].x() << std::endl;
  //   std::cout << refline_points[i].y() << std::endl;
  //   std::cout << refline_points[i].heading() << std::endl;
  // }
  // std::cout << refline_points[0].x() << std::endl;
  // std::cout << refline_points[0].y() << std::endl;
  // std::cout << refline_points[0].s() << std::endl;
  // std::cout << refline_points[0].heading() << std::endl;
  // std::cout << refline_points[0].kappa() << std::endl;
  // std::cout << refline_points[0].left_bound_point().x() << std::endl;
  // std::cout << refline_points[0].left_bound_point().y() << std::endl;
  // std::cout << refline_points.back().x() << std::endl;
  // std::cout << refline_points.back().y() << std::endl;
  // std::cout << refline_points.back().s() << std::endl;
  // std::cout << refline_points.back().heading() << std::endl;
  // std::cout << refline_points.back().kappa() << std::endl;
  // std::cout << refline_points.back().left_bound_point().x() << std::endl;
  // std::cout << refline_points.back().left_bound_point().y() << std::endl;
  std::vector<ReferencePoint> nearest_100m_refpoint;
  std::vector<ReferencePoint> back_refpoints;
  std::vector<ReferencePoint> front_refpoints;
  std::vector<ReferencePoint> nearest_100m_refpoint_sampled;

  if (ref_length <= 0) {
    return std::make_tuple(nearest_100m_refpoint_sampled, nearest_100m_refpoint);
  }

  // 截取后向
  if (nearest_index >= 150) {
    back_refpoints.insert(back_refpoints.end(),
                          refline_points.begin() + nearest_index - 150,
                          refline_points.begin() + nearest_index);
  } else {
    int point_num = 150 - nearest_index;
    if (is_use_strength) {
      // 补refline曲线-延长
      auto start_point = refline_points[0];
      double theta = refline_points[0].heading() + pi;
      
      double curvature = refline_points[0].kappa();
      std::vector<ReferencePoint> support_refline_points =
          GetSupportRefPoint(start_point, theta, point_num);

      std::reverse(support_refline_points.begin(), support_refline_points.end());
      back_refpoints.insert(back_refpoints.end(), support_refline_points.begin(),
                            support_refline_points.end());
      back_refpoints.insert(back_refpoints.end(), refline_points.begin(),
                            refline_points.begin() + nearest_index);
    } else {
      std::vector<ReferencePoint> support_rps(point_num, refline_points[0]);
      back_refpoints.insert(back_refpoints.end(), support_rps.begin(), support_rps.end());
      back_refpoints.insert(back_refpoints.end(), refline_points.begin(),
                            refline_points.begin() + nearest_index);
    }
  }

  // 截取前向 每0.2m 1个点 80m = 400个点
  if ((ref_length - nearest_index) >= 400) {
    front_refpoints.insert(front_refpoints.end(),
                           refline_points.begin() + nearest_index,
                           refline_points.begin() + nearest_index + 400);
  } else {
    int point_num = 400 - (ref_length - nearest_index);
    if (is_use_strength) {
      // 补refline曲线-延长
      auto start_point = refline_points.back();
      double theta = refline_points[ref_length - 2].heading();
      double curvature = refline_points[ref_length - 2].kappa();
      std::vector<ReferencePoint> support_refline_points =
          GetSupportRefPoint(start_point, theta, point_num);

      front_refpoints.insert(front_refpoints.end(),
                            refline_points.begin() + nearest_index,
                            refline_points.end());
      front_refpoints.insert(front_refpoints.end(),
                            support_refline_points.begin(),
                            support_refline_points.end());
    } else {
      front_refpoints.insert(front_refpoints.end(),
                            refline_points.begin() + nearest_index,
                            refline_points.end());
      std::vector<ReferencePoint> support_rps(point_num, refline_points.back());
      front_refpoints.insert(front_refpoints.end(), support_rps.begin(), support_rps.end());
    }
  }
  nearest_100m_refpoint.insert(nearest_100m_refpoint.end(),
                               back_refpoints.begin(), back_refpoints.end());
  nearest_100m_refpoint.insert(nearest_100m_refpoint.end(),
                               front_refpoints.begin(), front_refpoints.end());

  int gap = int(550 / sample_num);
  int near100ref_length = nearest_100m_refpoint.size();
  
  for (int i = 0; i < near100ref_length; i = i + gap) {
    nearest_100m_refpoint_sampled.emplace_back(nearest_100m_refpoint[i]);
  }
  int sample_near100ref_length = nearest_100m_refpoint_sampled.size();
  if (sample_near100ref_length >= 100) {
    nearest_100m_refpoint_sampled.assign(nearest_100m_refpoint_sampled.end() - 100, nearest_100m_refpoint_sampled.end());
  } else {
    for (int i = 0; i < (100 - sample_near100ref_length); i++) {
      nearest_100m_refpoint_sampled.emplace_back(nearest_100m_refpoint_sampled.back());
    }
  }
  // std::cout << "nearest_100m_refpoint_sampled" << std::endl;
  // for (int i = 0; i < nearest_100m_refpoint_sampled.size(); i++) {
  //   std::cout << nearest_100m_refpoint_sampled[i].x() << std::endl;
  //   std::cout << nearest_100m_refpoint_sampled[i].y() << std::endl;
  //   std::cout << nearest_100m_refpoint_sampled[i].heading() << std::endl;
  // }
  // std::cout << "nearest_100m_refpoint" << std::endl;
  // for (int i = 0; i < nearest_100m_refpoint.size() + 1; i++) {
  //   std::cout << nearest_100m_refpoint[i].x() << std::endl;
  //   std::cout << nearest_100m_refpoint[i].y() << std::endl;
  //   std::cout << nearest_100m_refpoint[i].heading() << std::endl;
  // }
  return std::make_tuple(nearest_100m_refpoint_sampled, nearest_100m_refpoint);
}

std::vector<std::vector<double>> FindNearest100MeterRefpoints(
    const std::vector<std::vector<double>> &refline_points,
    const std::vector<double> &adc_point) {
  int nearest_index = 0;
  double min_dis = std::numeric_limits<double>::max();
  int ref_length = refline_points.size();
  for (int i = 0; i < ref_length; i++) {
    double diff_x = adc_point[0] - refline_points[i][0];
    double diff_y = adc_point[1] - refline_points[i][1];

    double dis = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
    if (dis <= min_dis) {
      nearest_index = i;
      min_dis = dis;
    }
  }
  std::vector<std::vector<double>> refpoints_100meter;
  double behind_dis = refline_points[nearest_index][3];
  for (int i = 0; i < nearest_index; i++) {
    if (behind_dis - refline_points[i][3] <= 30) {
      refpoints_100meter.emplace_back(refline_points[i]);
    }
  }
  double ahead_dis = refline_points[nearest_index][3];
  for (int i = nearest_index; i < ref_length; i++) {
    if (refline_points[i][3] - ahead_dis <= 70) {
      refpoints_100meter.emplace_back(refline_points[i]);
    }
  }
  return refpoints_100meter;
}

std::vector<std::vector<double>> GetSupportPoint(
    const double &curvature, const std::vector<double> &start_point,
    double theta, const int &point_num) {
  std::vector<std::vector<double>> support_points;
  for (int i = 0; i < point_num; i++) {
    theta = NormalizeAngle(theta);
    double new_point_x = start_point[0] + i * 0.2 * cos(theta);
    double new_point_y = start_point[1] + i * 0.2 * sin(theta);
    std::vector<double> tmp{new_point_x, new_point_y};
    support_points.emplace_back(tmp);
  }
  return support_points;
}

std::vector<ReferencePoint> GetSupportRefPoint(
    const ReferencePoint &start_point,
    double theta, const int &point_num) {
  std::vector<ReferencePoint> support_points;
  for (int i = 0; i < point_num; i++) {
    theta = NormalizeAngle(theta);
    double new_point_x = start_point.x() + i * 0.2 * cos(theta);
    double new_point_y = start_point.y() + i * 0.2 * sin(theta);
    //调用拷贝构造函数，浅拷贝，python版本用的深拷贝！！
    ReferencePoint tmp = start_point;
    tmp.set_x(new_point_x);
    tmp.set_y(new_point_y);
    tmp.set_heading(start_point.heading());
    tmp.set_kappa(start_point.kappa());
    support_points.emplace_back(tmp);
  }
  return support_points;
}

int FindNearestReflinePoint(const std::vector<ReferencePoint> &refline_points, const std::vector<double> &ADCpoint) {
  int nearest_index = 0;
  double min_dis = std::numeric_limits<double>::max();
  std::vector<double> future_dis_list;
  future_dis_list.clear();
  for (int i = 0; i < int(refline_points.size()); i++) {
    double diff_x = refline_points[i].x() - ADCpoint[0];
    double diff_y = refline_points[i].y() - ADCpoint[1];
    double dis = pow(diff_x, 2) + pow(diff_y, 2);
    if (dis <= min_dis) {
      nearest_index = i;
      min_dis = dis;
      future_dis_list.clear();
    } else {
      future_dis_list.emplace_back(dis);
    }
    // 提前终止
    if (int(future_dis_list.size()) > 3) {
      break;
    }
  }
  return nearest_index;
}

int FindNearestReflinePointS(const std::vector<ReferencePoint> &refline_points, const int start_index, const double predict_s) {
  int refline_length = refline_points.size();
  for (int i = 0; i < refline_length; i++) {
    double s = refline_points[i].s() - refline_points[start_index].s();
    if (s - predict_s >= 0) {
      return i;
    }
  }
  return refline_length - 1;
}

int trans_bool2int(bool input) {
  if (input) {
    return 1;
  } else {
    return 0;
  }
}

double NormalizeAngle(const double &angle) {
  double new_angle = fmod(angle + pi, 2 * pi);
  if (new_angle < 0.0) {
    new_angle = new_angle + pi;
  } else {
    new_angle = new_angle - pi;
  }
  return new_angle;
}

std::vector<double> V1dGradient(const std::vector<double> &input) {
  int length = input.size();
  if (length <= 1) return input;
  std::vector<double> ret;
  for (int i = 0; i < length; i++) {
    int left = i - 1;
    int right = i + 1;
    int step = 2;
    if (left < 0) {
      left = 0;
      right = 1;
      step = 1;
    }
    if (right >= length) {
      left = length - 2;
      right = length - 1;
      step = 1;
    }
    double grad = (input[right] - input[left]) / step;
    ret.emplace_back(grad);
  }
  return ret;
}

std::vector<double> V1dAcrtan2(const std::vector<double> &inputy,
                               const std::vector<double> &inputx) {
  int length = inputy.size();
  std::vector<double> ret;
  for (int i = 0; i < length; i++) {
    double theta = atan2(inputy[i], inputx[i]);
    ret.emplace_back(theta);
  }
  return ret;
}

std::vector<double> CalCurvatureWithTheta(const std::vector<double> &x,
                                          const std::vector<double> &y,
                                          const std::vector<double> &theta) {
  std::vector<double> x_gradient = V1dGradient(x);
  std::vector<double> y_gradient = V1dGradient(y);
  std::vector<double> theta_gradient = V1dGradient(theta);
  std::vector<double> curvature_val;
  int v_size = x.size();
  for (int i = 0; i < v_size; i++) {
    double c = theta_gradient[i] / sqrt(pow(x_gradient[i], 2) +
                                        pow(y_gradient[i], 2) + 0.000000001);
    curvature_val.emplace_back(c);
  }
  return curvature_val;
}

std::tuple<std::vector<double>, std::vector<double>> CalCurvature(
    const std::vector<double> &x, const std::vector<double> &y) {
  std::vector<double> x_gradient = V1dGradient(x);
  std::vector<double> y_gradient = V1dGradient(y);
  std::vector<double> theta = V1dAcrtan2(y_gradient, x_gradient);
  std::vector<double> theta_gradient = V1dGradient(theta);
  std::vector<double> curvature_val;
  int v_size = x.size();
  for (int i = 0; i < v_size; i++) {
    double c =
        theta_gradient[i] / sqrt(pow(x_gradient[i], 2) + pow(y_gradient[i], 2));
    curvature_val.emplace_back(c);
  }
  return std::make_tuple(curvature_val, theta);
}

std::vector<double> CalDKappa(const std::vector<double> &x,
                              const std::vector<double> &y,
                              const std::vector<double> &kappa) {
  std::vector<double> x_gradient = V1dGradient(x);
  std::vector<double> y_gradient = V1dGradient(y);
  std::vector<double> kappa_gradient = V1dGradient(kappa);
  std::vector<double> d_kappa;
  int v_size = x.size();
  for (int i = 0; i < v_size; i++) {
    double c = kappa_gradient[i] / sqrt(pow(x_gradient[i], 2) +
                                        pow(y_gradient[i], 2) + 0.000000001);
    d_kappa.emplace_back(c);
  }
  return d_kappa;
}

bool CmpTotalCost(const std::vector<double> &x, const std::vector<double> &y) {
  int l1 = x.size();
  int l2 = y.size();
  if (l1 > 11 && l2 > 11) {
    return x[11] < y[11];
  }
  return false;
}

std::vector<double> ConvertToRelativeCoordinate2D(
    const std::vector<double> &pt_in_world,
    const std::vector<double> &origin_in_world) {
  double diff_x = pt_in_world[0] - origin_in_world[0];
  double diff_y = pt_in_world[1] - origin_in_world[1];
  double origin_yaw = origin_in_world[2];

  double cosr = cos(origin_yaw);
  double sinr = sin(origin_yaw);

  std::vector<double> pt_in_relative = {0.0, 0.0};

  pt_in_relative[0] = (diff_x * cosr + diff_y * sinr);
  pt_in_relative[1] = (diff_y * cosr - diff_x * sinr);

  return pt_in_relative;
}

std::vector<double> ConvertToRelativeCoordinate(
    const std::vector<double> &pt_in_world,
    const std::vector<double> &origin_in_world) {
  double diff_x = pt_in_world[0] - origin_in_world[0];
  double diff_y = pt_in_world[1] - origin_in_world[1];
  double origin_yaw = origin_in_world[2];

  double cosr = cos(origin_yaw);
  double sinr = sin(origin_yaw);

  std::vector<double> pt_in_relative = {0.0, 0.0, 0.0};

  pt_in_relative[0] = (diff_x * cosr + diff_y * sinr);
  pt_in_relative[1] = (diff_y * cosr - diff_x * sinr);
  pt_in_relative[2] = NormalizeAngle(pt_in_world[2] - origin_yaw);

  return pt_in_relative;
}

std::vector<double> ConvertToRelativeCoordinate2D5input(const double &pt_x,
                                                        const double &pt_y,
                                                        const double &ori_x,
                                                        const double &ori_y,
                                                        const double &ori_yaw) {
  std::vector<double> pt_in_world{pt_x, pt_y};
  std::vector<double> origin_in_world{ori_x, ori_y, ori_yaw};
  return ConvertToRelativeCoordinate2D(pt_in_world, origin_in_world);
}

std::vector<double> ConvertToRelativeCoordinate6input(const double &pt_x,
                                                        const double &pt_y,
                                                        const double &pt_yaw,
                                                        const double &ori_x,
                                                        const double &ori_y,
                                                        const double &ori_yaw) {
  std::vector<double> pt_in_world{pt_x, pt_y, pt_yaw};
  std::vector<double> origin_in_world{ori_x, ori_y, ori_yaw};
  return ConvertToRelativeCoordinate(pt_in_world, origin_in_world);
}

std::vector<double> ConvertToWorldCoordinate(
    const std::vector<double> &pt_in_relative,
    const std::vector<double> &origin_in_world) {
  double origin_yaw = origin_in_world[2];
  origin_yaw = NormalizeAngle(origin_yaw);
  double cosr = cos(origin_yaw);
  double sinr = sin(origin_yaw);

  double diff_x = pt_in_relative[0] * cosr - pt_in_relative[1] * sinr;
  double diff_y = pt_in_relative[0] * sinr + pt_in_relative[1] * cosr;
  std::vector<double> pt_in_world = {0.0, 0.0, 0.0};

  pt_in_world[0] = diff_x + origin_in_world[0];
  pt_in_world[1] = diff_y + origin_in_world[1];
  pt_in_world[2] = NormalizeAngle(pt_in_relative[2] + origin_yaw);

  return pt_in_world;
}

std::vector<double> ConvertToWorldCoordinate6input(
    const double &pt_x, const double &pt_y, const double &pt_yaw,
    const double &ori_x, const double &ori_y, const double &ori_yaw) {
  std::vector<double> pt_in_relative{pt_x, pt_y, pt_yaw};
  std::vector<double> origin_in_world{ori_x, ori_y, ori_yaw};
  return ConvertToWorldCoordinate(pt_in_relative, origin_in_world);
}

void WorldCoordToVehicleCoord(const double ego_world_x,
                              const double ego_world_y,
                              const double ego_world_theta,
                              const double obs_world_x,
                              const double obs_world_y,
                              const double obs_world_theta, double &obs_ego_x,
                              double &obs_ego_y, double &obs_ego_theta) {
  obs_ego_x = (obs_world_x - ego_world_x) * std::cos(ego_world_theta) +
              (obs_world_y - ego_world_y) * std::sin(ego_world_theta);
  obs_ego_y = (ego_world_x - obs_world_x) * std::sin(ego_world_theta) +
              (obs_world_y - ego_world_y) * std::cos(ego_world_theta);
  obs_ego_theta = obs_world_theta - ego_world_theta;
}

void VehicleCoordToOdometryCoord(const double ego_odo_x, const double ego_odo_y,
                                 const double ego_odo_theta,
                                 const double obs_ego_x, const double obs_ego_y,
                                 const double obs_ego_theta, double &obs_odo_x,
                                 double &obs_odo_y, double &obs_odo_theta) {
  obs_odo_x = obs_ego_x * std::cos(ego_odo_theta) -
              obs_ego_y * std::sin(ego_odo_theta) + ego_odo_x;
  obs_odo_y = obs_ego_x * std::sin(ego_odo_theta) +
              obs_ego_y * std::cos(ego_odo_theta) + ego_odo_y;
  obs_odo_theta = ego_odo_theta + obs_ego_theta;
}

}  // namespace planning_rl
}  // namespace neodrive