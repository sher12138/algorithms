#include "post_predict.h"

namespace neodrive {
namespace planning_rl {

const double pi = M_PI;
const double eps = 1e-8;

std::vector<PredState> PostPredict(
    const std::vector<std::vector<std::vector<double>>> &input,
    std::vector<double> &prediction, const EgoInfo &egoinfo) {
  // Json::Value post_predict_conf;
  // PostPredictConfReader(post_predict_conf);
  auto post_predict_conf = config::PlanningRLConfig::Instance()->post_predict();
  // std::cout << "prediction" << std::endl;
  // std::cout << input.size() << std::endl;
  // std::cout << input[0].size() << std::endl;
  // std::cout << prediction << std::endl;
  RescaleAction2Unnormalized(prediction);
  // std::cout << prediction << std::endl;
  int frame_num = input.size() - 1;
  // 此处与python版本取值位置有区别，python先进行了view降维，这里vector直接取值避免类型转换。
  prediction[0] = prediction[0] + input[frame_num][0][0];
  prediction[1] = prediction[1] + input[frame_num][0][1];
  // std::cout << "prediction2" << std::endl;
  // std::cout << prediction << std::endl;
  // std::cout << input[frame_num][0] << std::endl;

  std::vector<PredState> res_pred;
  // // 绘制螺旋曲线生成点
  // EgoInfo ego = EgoInfo();
  // ego.x = input[frame_num][0];
  // ego.y = input[frame_num][1];
  // ego.heading = input[frame_num][2];
  // ego.speed = sqrt(pow(input[frame_num][4], 2) + pow(input[frame_num][5],
  // 2)); ego.acc = sqrt(pow(input[frame_num][6], 2) + pow(input[frame_num][7],
  // 2)); ego.curvature = input[frame_num][3]; ego.step_time = 2.0;

  // std::vector<double> steptimes (20, 0.1);
  PredState start_state = PredState();
  start_state.x = input[frame_num][0][0];
  start_state.y = input[frame_num][0][1];
  start_state.heading = input[frame_num][0][2];
  start_state.speed =
      sqrt(pow(input[frame_num][0][4], 2) + pow(input[frame_num][0][5], 2));
  start_state.acc =
      sqrt(pow(input[frame_num][0][6], 2) + pow(input[frame_num][0][7], 2));
  start_state.curvature = input[frame_num][0][3];
  PredState end_state = PredState();
  end_state.x = prediction[0];
  end_state.y = prediction[1];
  end_state.heading = prediction[2];
  end_state.speed = prediction[3];
  end_state.acc = prediction[4];
  end_state.curvature = prediction[5];

  res_pred.emplace_back(start_state);
  res_pred.emplace_back(end_state);

  ConvertPredWorldCor(res_pred, egoinfo);

  return res_pred;
}

std::vector<double> RescaleAction2Unnormalized(std::vector<double> &action) {
  // double MAX_ACC = post_predict_conf["MAC_ACC"].asDouble();
  // double MIN_BRAKE = post_predict_conf["MIN_BRAKE"].asDouble();
  // double MAX_deta_X = post_predict_conf["MAX_deta_X"].asDouble();
  // double MAX_deta_Y = post_predict_conf["MAX_deta_Y"].asDouble();
  // double MIN_deta_X = post_predict_conf["MIN_deta_X"].asDouble();
  // double MIN_deta_Y = post_predict_conf["MIN_deta_Y"].asDouble();
  auto &post_predict_conf =
      config::PlanningRLConfig::Instance()->post_predict();
  double MAX_ACC = post_predict_conf.rescale_action_paras.max_acc;
  double MIN_BRAKE = post_predict_conf.rescale_action_paras.min_brake;
  double MAX_deta_X = post_predict_conf.rescale_action_paras.max_deta_x;
  double MAX_deta_Y = post_predict_conf.rescale_action_paras.max_deta_y;
  double MIN_deta_X = post_predict_conf.rescale_action_paras.min_deta_x;
  double MIN_deta_Y = post_predict_conf.rescale_action_paras.min_deta_y;

  double MAX_THETA = pi;
  double MIN_THETA = -pi;

  double MAX_TIME = post_predict_conf.rescale_action_paras.max_time;
  double MAX_SPEED = post_predict_conf.rescale_action_paras.max_speed;
  double MAX_CURVATURE = post_predict_conf.rescale_action_paras.max_curvature;

  // double MAX_TIME = post_predict_conf["MAX_TIME"].asDouble();
  // double MAX_SPEED = post_predict_conf["MAX_SPEED"].asDouble();
  // double MAX_CURVATURE = post_predict_conf["MAX_CURVATURE"].asDouble();

  if (action[0] > 0) {
    action[0] = action[0] * MAX_deta_X;
  } else {
    action[0] = -action[0] * MIN_deta_X;
  }
  if (action[1] > 0) {
    action[1] = action[1] * MAX_deta_Y;
  } else {
    action[1] = -action[1] * MIN_deta_Y;
  }
  if (action[2] < 0) {
    action[2] = -action[2] * MIN_THETA;
  } else {
    action[2] = action[2] * MAX_THETA;
  }
  if (action[3] < 0) {
    action[3] = 0;
  } else {
    action[3] = action[3] * MAX_SPEED;
  }
  if (action[4] < 0) {
    action[4] = -action[4] * MIN_BRAKE;
  } else {
    action[4] = action[4] * MAX_ACC;
  }
  action[5] = action[5] * MAX_CURVATURE;
  action[6] = action[6] * MAX_TIME;
  return action;
}

std::vector<double> RescaleAction(std::vector<double> &action) {
  double MAX_deta_ACC = 6.0;
  double MIN_deta_BRAKE = -5.0;
  double MAX_deta_X = 30.0;
  double MAX_deta_Y = 6.0;
  double MIN_deta_X = -2.0;
  double MIN_deta_Y = -6.0;

  double MAX_deta_THETA = 0.6;
  double MIN_deta_THETA = -0.6;
  double MAX_deta_SPEED = 5.0;
  double MAX_CURVATURE = 0.1;

  if (action[0] > 0) {
    action[0] = action[0] * MAX_deta_X;
  } else {
    action[0] = -action[0] * MIN_deta_X;
  }
  if (action[1] > 0) {
    action[1] = action[1] * MAX_deta_Y;
  } else {
    action[1] = -action[1] * MIN_deta_Y;
  }
  if (action[2] < 0) {
    action[2] = -action[2] * MIN_deta_THETA;
  } else {
    action[2] = action[2] * MAX_deta_THETA;
  }
  if (action[3] < 0) {
    action[3] = 0;
  } else {
    action[3] = action[3] * MAX_deta_SPEED;
  }
  if (action[4] < 0) {
    action[4] = -action[4] * MIN_deta_BRAKE;
  } else {
    action[4] = action[4] * MAX_deta_ACC;
  }
  action[5] = action[5] * MAX_CURVATURE;
  return action;
}

std::vector<double> RescaleActionStepGap(std::vector<double> &action) {
  double MAX_deta_X = 6.0;
  double MIN_deta_X = -6.0;

  double MAX_deta_Y = 0.1;
  double MIN_deta_Y = -0.1;

  double MAX_deta_THETA = 0.7;
  double MIN_deta_THETA = -0.7;

  double MAX_deta_SPEED = 0.5;
  double MIN_deta_SPEED = -0.5;

  double MAX_deta_ACC = 4.0;
  double MIN_deta_ACC = -4.0;

  double MAX_deta_CURVATURE = 0.05;
  double MIN_deta_CURVATURE = -0.05;

  if (action[0] > 0) {
    action[0] = action[0] * MAX_deta_X;
  } else {
    action[0] = -action[0] * MIN_deta_X;
  }
  if (action[1] > 0) {
    action[1] = action[1] * MAX_deta_Y;
  } else {
    action[1] = -action[1] * MIN_deta_Y;
  }
  if (action[2] > 0) {
    action[2] = action[2] * MAX_deta_THETA;
  } else {
    action[2] = -action[2] * MIN_deta_THETA;
  }
  if (action[3] > 0) {
    action[3] = action[3] * MAX_deta_SPEED;
  } else {
    action[3] = -action[3] * MIN_deta_SPEED;
  }
  if (action[4] > 0) {
    action[4] = action[4] * MAX_deta_ACC;
  } else {
    action[4] = -action[4] * MIN_deta_ACC;
  }
  if (action[5] > 0) {
    action[5] = action[5] * MAX_deta_CURVATURE;
  } else {
    action[5] = -action[5] * MIN_deta_CURVATURE;
  }
  
  return action;
}

std::vector<PredState> ConvertPredWorldCor(std::vector<PredState> &pred_states,
                                           const EgoInfo &egoinfo) {
  for (auto &pred : pred_states) {
    auto point_global_coordinate = ConvertToWorldCoordinate6input(
        pred.x, pred.y, pred.heading, egoinfo.x, egoinfo.y, egoinfo.heading);
    pred.x = point_global_coordinate[0];
    pred.y = point_global_coordinate[1];
    pred.heading = point_global_coordinate[2];
  }
  return pred_states;
}

std::vector<TrajectoryPoint2D> OptimalTrajectoryRefpoint(
    const std::vector<PredState> &predict_traj,
    const std::vector<RefPoint> &reference_line_points) {
  std::vector<TrajectoryPoint2D> trajectory2D;
  auto start_state = predict_traj.front();
  auto end_state = predict_traj.back();
  std::cout << "start_end_state" << std::endl;
  std::cout << start_state.x << "   " << start_state.y << std::endl;
  std::cout << start_state.heading << std::endl;
  std::cout << start_state.speed << std::endl;
  std::cout << start_state.acc << std::endl;
  std::cout << start_state.curvature << std::endl;
  std::cout << end_state.x << "   " << end_state.y << std::endl;
  std::cout << end_state.heading << std::endl;
  std::cout << end_state.speed << std::endl;
  std::cout << end_state.acc << std::endl;
  std::cout << end_state.curvature << std::endl;

  double deltaT = 2.0;
  double resolution = 0.1;
  int mode = 1;

  std::vector<double> xs, ys, kappas;
  double closest_dis_start = 10000;
  double closest_dis_end = 10000;
  TrajPoint lon_start_state = TrajPoint();
  TrajPoint lat_start_state = TrajPoint();
  TrajPoint lon_end_state = TrajPoint();
  TrajPoint lat_end_state = TrajPoint();
  auto ref_start = RefPoint();
  auto ref_end = RefPoint();

  for (auto &point : reference_line_points) {
    xs.emplace_back(point.x);
    ys.emplace_back(point.y);
    kappas.emplace_back(point.curvature);

    double start_dis, end_dis;
    start_dis = pow(
        pow((start_state.x - point.x), 2) + pow((start_state.y - point.y), 2),
        0.5);
    end_dis = pow(
        pow((end_state.x - point.x), 2) + pow((end_state.y - point.y), 2), 0.5);

    if (start_dis - closest_dis_start < eps) {
      closest_dis_start = start_dis;
      ref_start = point;
    }
    if (end_dis - closest_dis_end < eps) {
      closest_dis_end = end_dis;
      ref_end = point;
    }
  }
  if (fabs(closest_dis_start - 10000) < eps) {
    ref_start = reference_line_points.front();
    return trajectory2D;
  }
  if (fabs(closest_dis_end - 10000) < eps) {
    ref_end = reference_line_points.back();
    return trajectory2D;
  }
  double end_dis = pow(pow(end_state.x - reference_line_points.back().x, 2) +
                           pow(end_state.y - reference_line_points.back().y, 2),
                       0.5);
  if (end_dis < 1 + eps) {
    return trajectory2D;
  }
  // std::cout << "ref_start" << ref_start.x << ref_start.y << std::endl;
  // std::cout << ref_start.s << std::endl;
  // std::cout << ref_start.theta << std::endl;
  // std::cout << ref_start.curvature << std::endl;
  // std::cout << ref_start.z << std::endl;
  // std::cout << "ref_end" << ref_end.x << ref_end.y << std::endl;
  // std::cout << ref_end.s << std::endl;
  // std::cout << ref_end.theta << std::endl;
  // std::cout << ref_end.curvature << std::endl;
  // std::cout << ref_end.z << std::endl;

  auto start_speed = ConvertToRelativeCoordinate2D5input(
      start_state.speed * cos(start_state.heading),
      start_state.speed * sin(start_state.heading), 0, 0, ref_start.theta);
  auto start_acc = ConvertToRelativeCoordinate2D5input(
      start_state.acc * cos(start_state.heading),
      start_state.acc * sin(start_state.heading), 0, 0, ref_start.theta);
  auto start_xy = ConvertToRelativeCoordinate2D5input(
      start_state.x, start_state.y, ref_start.x, ref_start.y, ref_start.theta);
  auto end_speed = ConvertToRelativeCoordinate2D5input(
      end_state.speed * cos(end_state.heading),
      end_state.speed * sin(end_state.heading), 0, 0, ref_end.theta);
  auto end_acc = ConvertToRelativeCoordinate2D5input(
      end_state.acc * cos(end_state.heading),
      end_state.acc * sin(end_state.heading), 0, 0, ref_end.theta);
  auto end_xy = ConvertToRelativeCoordinate2D5input(
      end_state.x, end_state.y, ref_end.x, ref_end.y, ref_end.theta);

  double start_lon_speed, start_lat_speed, start_lon_acc, start_lat_acc,
      start_lon_x, start_lat_y, end_lon_speed, end_lat_speed, end_lon_acc,
      end_lat_acc, end_lon_x, end_lat_y;
  start_lon_speed = start_speed[0];
  start_lat_speed = start_speed[1];
  start_lon_acc = start_acc[0];
  start_lat_acc = start_acc[1];
  start_lon_x = start_xy[0];
  start_lat_y = start_xy[1];
  end_lon_speed = end_speed[0];
  end_lat_speed = end_speed[1];
  end_lon_acc = end_acc[0];
  end_lat_acc = end_acc[1];
  end_lon_x = end_xy[0];
  end_lat_y = end_xy[1];

  lon_start_state.x = ref_start.s;
  lon_start_state.x_der = start_lon_speed;
  lon_start_state.x_dder = start_lon_acc;
  lon_start_state.x_ddder = 0.0;  // jerk

  // lat_start_state.x = 0.0;
  lat_start_state.x = start_lat_y;
  lat_start_state.x_der = start_lat_speed;
  lat_start_state.x_dder = start_lat_acc;
  lat_start_state.x_dder = 0.0;

  lon_end_state.x = ref_end.s;
  if (end_lon_speed < 0) {
    lon_end_state.x_der = 0.0;
  } else {
    lon_end_state.x_der = end_lon_speed;
  }
  lon_end_state.x_dder = end_lon_acc;
  lon_end_state.x_ddder = 0.0;

  lat_end_state.x = end_lat_y;
  lat_end_state.x_der = end_lat_speed;
  lat_end_state.x_dder = end_lat_acc;
  lat_end_state.x_ddder = 0.0;

  auto d_kappas = CalDKappa(xs, ys, kappas);

  // CurvePoint curvep = CurvePoint();
  std::vector<CurvePoint> center_line;
  int index = 0;
  for (auto &point : reference_line_points) {
    CurvePoint temp_curve_point = CurvePoint();
    temp_curve_point.s = point.s;
    temp_curve_point.x = point.x;
    temp_curve_point.y = point.y;
    temp_curve_point.theta = point.theta;
    temp_curve_point.kappa = point.curvature;
    temp_curve_point.kappa_prime = d_kappas[index];
    temp_curve_point.v = 0;
    temp_curve_point.t = 0;
    center_line.emplace_back(temp_curve_point);
    index += 1;
  }
  // std::cout << "ready for gen2dtraj" << std::endl;
  // std::cout << lon_start_state.x << "  " << lon_start_state.x_der << "  " <<
  // lon_start_state.x_dder << std::endl; std::cout << lon_end_state.x << "  "
  // << lon_end_state.x_der << "  " << lon_end_state.x_dder <<std::endl;
  // std::cout << lat_start_state.x << "  " << lat_start_state.x_der << "  " <<
  // lat_start_state.x_dder << std::endl; std::cout << lat_end_state.x << "  "
  // << lat_end_state.x_der << "  " << lat_end_state.x_dder <<std::endl;
  // std::cout << deltaT << std::endl;
  // std::cout << resolution << std::endl;
  // std::cout << center_line[0].x << "  " << center_line[0].y << std::endl;
  // std::cout << center_line.back().x << "  " << center_line.back().y <<
  // std::endl;
  trajectory2D =
      Generate2DTraj(lon_start_state, lon_end_state, lat_start_state,
                     lat_end_state, deltaT, resolution, center_line, mode);
  // std::cout << "size:" << trajectory2D.size() << std::endl;
  // std::cout << "trajectory2D" << std::endl;
  std::cout << "optj_output:" << std::endl;
  for (int i = 0; i < trajectory2D.size(); i++) {
    std::cout << "trajectory_point: " << i << std::endl;
    auto &pt = trajectory2D[i];
    std::cout << pt.t << std::endl;
    std::cout << pt.x << std::endl;
    std::cout << pt.y << std::endl;
    std::cout << pt.theta << std::endl;
    std::cout << pt.v << std::endl;
    std::cout << pt.a << std::endl;
    std::cout << pt.kappa << std::endl;
  }
  // std::cout << trajectory2D[0].t << "   " << trajectory2D[0].x << "   " <<
  // trajectory2D[0].y << "   " << trajectory2D[0].v << "   " <<
  // trajectory2D[0].a << "   " << trajectory2D[0].kappa << std::endl; std::cout
  // << trajectory2D.back().t << "   " << trajectory2D.back().x << "   " <<
  // trajectory2D.back().y << "   " << trajectory2D.back().v << "   " <<
  // trajectory2D.back().a << "   " << trajectory2D.back().kappa << std::endl;
  return trajectory2D;
}

std::vector<PredState> TransPredictsFrenet2Odom(
    std::vector<std::vector<double>> &prediction, const EgoInfo &egoinfo, const std::vector<ReferencePoint> &reference_line_odom) {
  std::vector<double> ADCPoint{egoinfo.x, egoinfo.y};
  // std::cout << "TransPredictsFrenet2Odom" << std::endl;
  int nearest_index = FindNearestReflinePoint(reference_line_odom, ADCPoint);
  auto refline_points_nearest_start = reference_line_odom[nearest_index];
  // std::cout << egoinfo.x << std::endl;
  // std::cout << egoinfo.y << std::endl;
  // std::cout << egoinfo.heading << std::endl;
  // std::cout << refline_points_nearest_start.x() << std::endl;
  // std::cout << refline_points_nearest_start.y() << std::endl;
  // std::cout << refline_points_nearest_start.heading() << std::endl;
  auto ego_relative = ConvertToRelativeCoordinate6input(egoinfo.x, egoinfo.y, egoinfo.heading, refline_points_nearest_start.x(), refline_points_nearest_start.y(), refline_points_nearest_start.heading());
  
  auto startpoint_speed = egoinfo.speed;
  auto startpoint_lon_speed = startpoint_speed * cos(egoinfo.heading);
  auto startpoint_lat_speed = startpoint_speed * sin(egoinfo.heading);
  auto startpoint_speed_relative = ConvertToRelativeCoordinate6input(startpoint_lon_speed, startpoint_lat_speed, egoinfo.heading, 0, 0, refline_points_nearest_start.heading());
  auto startpoint_lon_speed_frenet = startpoint_speed_relative[0];
  auto startpoint_lat_speed_frenet = startpoint_speed_relative[1];
  auto startpoint_yaw = startpoint_speed_relative[2];

  double startpoint_speed_frenet = 0.0;
  if (sin(startpoint_yaw) < 10e-6 && sin(startpoint_yaw) > -10e-6) {
    startpoint_speed_frenet = startpoint_lon_speed_frenet;
  } else {
    startpoint_speed_frenet = startpoint_lat_speed_frenet / sin(startpoint_yaw);
  }

  auto startpoint_acc = egoinfo.acc;
  auto startpoint_lon_acc = startpoint_acc * cos(egoinfo.heading);
  auto startpoint_lat_acc = startpoint_acc * sin(egoinfo.heading);
  auto startpoint_acc_relative = ConvertToRelativeCoordinate6input(startpoint_lon_acc, startpoint_lat_acc, egoinfo.heading, 0, 0, refline_points_nearest_start.heading());
  auto startpoint_lon_acc_frenet = startpoint_acc_relative[0];
  auto startpoint_lat_acc_frenet = startpoint_acc_relative[1];
  startpoint_yaw = startpoint_acc_relative[2];

  double startpoint_acc_frenet = 0.0;
  if (sin(startpoint_yaw) < 10e-6 && sin(startpoint_yaw) > -10e-6) {
    startpoint_acc_frenet = startpoint_lon_acc_frenet;
  } else {
    startpoint_acc_frenet = startpoint_lat_acc_frenet / sin(startpoint_yaw);
  }
  // std::cout << ego_relative << std::endl;
  // std::cout << startpoint_speed_relative << std::endl;
  // std::cout << startpoint_acc_relative << std::endl;

  std::vector<PredState> predicts2odom;
  for (auto &predict: prediction) {
    predict = RescaleAction(predict);
    int predict_nearest_index = FindNearestReflinePointS(reference_line_odom, nearest_index, predict[0]);
    auto refline_points_nearest = reference_line_odom[predict_nearest_index];
    predict[0] = predict[0] - (refline_points_nearest.s() - refline_points_nearest_start.s());
    predict[1] = predict[1] + ego_relative[1];
    predict[2] = predict[2] + ego_relative[2];
    auto targetpoint_odom = ConvertToWorldCoordinate6input(predict[0], predict[1], predict[2], refline_points_nearest.x(), refline_points_nearest.y(), refline_points_nearest.heading());
    
    auto speed = predict[3] + startpoint_speed_frenet;
    auto lon_speed = speed * cos(predict[2]);
    auto lat_speed = speed * sin(predict[2]);
    auto speed_odom = ConvertToWorldCoordinate6input(lon_speed, lat_speed, predict[2], 0, 0, refline_points_nearest.heading());
    auto lon_speed_odom = speed_odom[0];
    auto lat_speed_odom = speed_odom[1];
    auto yaw = speed_odom[2];
    double speed_odom_cor = 0.0;
    if (sin(yaw) < 10e-6 && sin(yaw) > -10e-6) {
      speed_odom_cor = lon_speed_odom;
    } else {
      speed_odom_cor = lat_speed_odom / sin(yaw);
    }
    
    auto acc = predict[4] + startpoint_acc_frenet;
    auto lon_acc = acc * cos(predict[2]);
    auto lat_acc = acc * sin(predict[2]);
    auto acc_odom = ConvertToWorldCoordinate6input(lon_acc, lat_acc, predict[2], 0, 0, refline_points_nearest.heading());
    auto lon_acc_odom = acc_odom[0];
    auto lat_acc_odom = acc_odom[1];
    yaw = acc_odom[2];
    double acc_odom_cor = 0.0;
    if (sin(yaw) < 10e-6 && sin(yaw) > -10e-6) {
      acc_odom_cor = lon_acc_odom;
    } else {
      acc_odom_cor = lat_acc_odom / sin(yaw);
    }
    double curvature = predict[5] + refline_points_nearest.kappa();
    PredState ps = PredState();
    ps.x = targetpoint_odom[0];
    ps.y = targetpoint_odom[1];
    ps.heading = targetpoint_odom[2];
    ps.speed = speed_odom_cor;
    ps.acc = acc_odom_cor;
    ps.curvature = curvature;
    predicts2odom.emplace_back(ps);
  }
  return predicts2odom;
}

std::vector<PredState> TransPredictsFrenet2OdomStepGap(
    std::vector<std::vector<double>> &prediction, const EgoInfo &egoinfo, const std::vector<ReferencePoint> &reference_line_odom) {
  std::vector<double> ADCPoint{egoinfo.x, egoinfo.y};
  // std::cout << "TransPredictsFrenet2Odom" << std::endl;
  int nearest_index = FindNearestReflinePoint(reference_line_odom, ADCPoint);
  auto refline_points_nearest_start = reference_line_odom[nearest_index];
  auto ego_relative = ConvertToRelativeCoordinate6input(egoinfo.x, egoinfo.y, egoinfo.heading, refline_points_nearest_start.x(), refline_points_nearest_start.y(), refline_points_nearest_start.heading());
  
  auto startpoint_speed = egoinfo.speed;
  auto startpoint_lon_speed = startpoint_speed * cos(egoinfo.heading);
  auto startpoint_lat_speed = startpoint_speed * sin(egoinfo.heading);
  auto startpoint_speed_relative = ConvertToRelativeCoordinate6input(startpoint_lon_speed, startpoint_lat_speed, egoinfo.heading, 0, 0, refline_points_nearest_start.heading());
  auto startpoint_lon_speed_frenet = startpoint_speed_relative[0];
  auto startpoint_lat_speed_frenet = startpoint_speed_relative[1];
  auto startpoint_yaw = startpoint_speed_relative[2];

  double startpoint_speed_frenet = 0.0;
  if (sin(startpoint_yaw) < 10e-6 && sin(startpoint_yaw) > -10e-6) {
    startpoint_speed_frenet = startpoint_lon_speed_frenet;
  } else {
    startpoint_speed_frenet = startpoint_lat_speed_frenet / sin(startpoint_yaw);
  }

  auto startpoint_acc = egoinfo.acc;
  auto startpoint_lon_acc = startpoint_acc * cos(egoinfo.heading);
  auto startpoint_lat_acc = startpoint_acc * sin(egoinfo.heading);
  auto startpoint_acc_relative = ConvertToRelativeCoordinate6input(startpoint_lon_acc, startpoint_lat_acc, egoinfo.heading, 0, 0, refline_points_nearest_start.heading());
  auto startpoint_lon_acc_frenet = startpoint_acc_relative[0];
  auto startpoint_lat_acc_frenet = startpoint_acc_relative[1];
  startpoint_yaw = startpoint_acc_relative[2];

  double startpoint_acc_frenet = 0.0;
  if (sin(startpoint_yaw) < 10e-6 && sin(startpoint_yaw) > -10e-6) {
    startpoint_acc_frenet = startpoint_lon_acc_frenet;
  } else {
    startpoint_acc_frenet = startpoint_lat_acc_frenet / sin(startpoint_yaw);
  }
  // std::cout << "FormatPredicts" << std::endl;
  // std::cout << prediction << std::endl;
  FormatPredicts(prediction);
  // std::cout << prediction << std::endl;

  std::vector<PredState> predicts2odom;
  for (auto &predict: prediction) {
    predict = RescaleActionStepGap(predict);
    int predict_nearest_index = FindNearestReflinePointS(reference_line_odom, nearest_index, predict[0]);
    auto refline_points_nearest = reference_line_odom[predict_nearest_index];
    predict[0] = predict[0] - (refline_points_nearest.s() - refline_points_nearest_start.s());
    predict[1] = predict[1] + ego_relative[1];
    predict[2] = predict[2] + ego_relative[2];
    auto targetpoint_odom = ConvertToWorldCoordinate6input(predict[0], predict[1], predict[2], refline_points_nearest.x(), refline_points_nearest.y(), refline_points_nearest.heading());
    
    auto speed = predict[3] + startpoint_speed_frenet;
    auto lon_speed = speed * cos(predict[2]);
    auto lat_speed = speed * sin(predict[2]);
    auto speed_odom = ConvertToWorldCoordinate6input(lon_speed, lat_speed, predict[2], 0, 0, refline_points_nearest.heading());
    auto lon_speed_odom = speed_odom[0];
    auto lat_speed_odom = speed_odom[1];
    auto yaw = speed_odom[2];
    double speed_odom_cor = 0.0;
    if (sin(yaw) < 10e-6 && sin(yaw) > -10e-6) {
      speed_odom_cor = lon_speed_odom;
    } else {
      speed_odom_cor = lat_speed_odom / sin(yaw);
    }
    
    auto acc = predict[4] + startpoint_acc_frenet;
    auto lon_acc = acc * cos(predict[2]);
    auto lat_acc = acc * sin(predict[2]);
    auto acc_odom = ConvertToWorldCoordinate6input(lon_acc, lat_acc, predict[2], 0, 0, refline_points_nearest.heading());
    auto lon_acc_odom = acc_odom[0];
    auto lat_acc_odom = acc_odom[1];
    yaw = acc_odom[2];
    double acc_odom_cor = 0.0;
    if (sin(yaw) < 10e-6 && sin(yaw) > -10e-6) {
      acc_odom_cor = lon_acc_odom;
    } else {
      acc_odom_cor = lat_acc_odom / sin(yaw);
    }
    double curvature = predict[5] + refline_points_nearest.kappa();
    PredState ps = PredState();
    ps.x = targetpoint_odom[0];
    ps.y = targetpoint_odom[1];
    ps.heading = targetpoint_odom[2];
    ps.speed = speed_odom_cor;
    ps.acc = acc_odom_cor;
    ps.curvature = curvature;
    predicts2odom.emplace_back(ps);
  }
  return predicts2odom;
}

std::vector<std::vector<double>> FormatPredicts(std::vector<std::vector<double>> &prediction) {
  std::vector<double> accumulate_action {0, 0, 0, 0, 0, 0};
  int traj_len = prediction.size();
  for (int i = 0; i < traj_len; i++) {
    for (int j = 0; j < 6; j++) {
      accumulate_action[j] = accumulate_action[j] + prediction[i][j];
      prediction[i][j] = accumulate_action[j];
    }
  }
  return prediction;
}

}  // namespace planning_rl
}  // namespace neodrive