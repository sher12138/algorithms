#include "train_features.h"

namespace neodrive {
namespace planning_rl {

void FeaturePreparation::GetData(
    const std::vector<EgoFrame> &egoframes,
    const std::vector<std::vector<AgentFrame>> &agentsframes,
    const std::vector<std::vector<double>> &refline_input,
    const std::vector<double> &step_times,
    const std::vector<double> &refline_curvatures,
    const std::vector<double> &thetas) {
  ego_lists_ = egoframes;
  agents_lists_ = agentsframes;
  refline_points_ = refline_input;
  step_time_list_ = step_times;
  this->refline_curvatures_ = refline_curvatures;
  this->thetas_ = thetas;
  frame_num_ = egoframes.size();
}

// void FeaturePreparation::GetData(const ContainerMessageShrPtr &container_msg) {
//   container_msg_ = container_msg;
// }

std::vector<std::vector<double>> FeaturePreparation::GetAgentFeature(
    const EgoInfo &ego, const EgoFrame &ego_frame,
    std::vector<AgentFrame> &agents_frame,
    const std::vector<std::vector<double>> &refline_point100_meter) {
  time_log_.ResetStartTime();
  agents_input_.clear();
  std::vector<std::vector<double>> refer_points =
      GetReferPointFrom100Meter(refline_point100_meter);
  time_log_.RegisterTimeAndPrint("GetReferPointFrom100Meter");
  // 空frame会引起tensor计算报错
  if (agents_frame.size() > 0) {
    // input 1、选 64 个最关注的的agent 表征
    std::vector<std::vector<double>> total_result =
        GetAgentRepresentationTrt(ego_frame, agents_frame, refer_points);
    time_log_.RegisterTimeAndPrint("GetAgentRepresentationTrt");
    MakeAgentsInput(total_result, ego);
    time_log_.RegisterTimeAndPrint("MakeAgentsInput");
  }
  SupplyAgentsInput();
  time_log_.RegisterTimeAndPrint("SupplyAgentsInput");
  agents_window_.emplace_back(agents_input_);
  return agents_input_;
}

// std::vector<std::vector<double>> FeaturePreparation::GetAgentFeatureOriginal(std::vector<AgentFrame> &agents_frame) {
//   agents_input_.clear();
//   for (auto &agent : agents_frame) {
//     std::vector<double> frame_agent_feature {agent.centroid[0], agent.centroid[1], agent.yaw, agent.velocity[0], agent.velocity[1], agent.extent[0], agent.extent[1], 0};
//     agents_input_.emplace_back(frame_agent_feature);
//   }
//   return agents_input_;
// }

std::vector<std::vector<double>> FeaturePreparation::GetReferPointFrom100Meter(
    const std::vector<std::vector<double>> &refline_point100_meter) {
  std::vector<double> refline_point100_meter_xs, refline_point100_meter_ys;
  int ref_length = refline_point100_meter.size();
  for (int i = 0; i < ref_length; i++) {
    refline_point100_meter_xs.emplace_back(refline_point100_meter[i][0]);
    refline_point100_meter_ys.emplace_back(refline_point100_meter[i][1]);
  }
  std::vector<double> x_gradient = V1dGradient(refline_point100_meter_xs);
  std::vector<double> y_gradient = V1dGradient(refline_point100_meter_ys);
  std::vector<double> refline_thetas = V1dAcrtan2(y_gradient, x_gradient);
  std::vector<std::vector<double>> refer_points;
  for (int i = 0; i < ref_length; i++) {
    std::vector<double> point = {refline_point100_meter_xs[i],
                                 refline_point100_meter_ys[i],
                                 refline_thetas[i]};
    refer_points.emplace_back(point);
  }
  return refer_points;
}

std::vector<std::vector<double>> FeaturePreparation::MakeAgentsInput(
    std::vector<std::vector<double>> &total_result, const EgoInfo &ego) {
  int result_num = total_result.size();
  if (result_num > 0) {
    if (result_num <= result_limit_) {
      for (int i = 0; i < result_num; i++) {
        auto result = total_result[i];
        std::vector<double> temp(result.begin(), result.begin() + 9);
        // 自车坐标系
        double heading = NormalizeAngle(temp[8] - ego.heading);
        temp[8] = heading;
        temp.emplace_back(0.0);
        temp.emplace_back(0.0);
        agents_input_.emplace_back(temp);
      }
    } else {
      std::sort(total_result.begin(), total_result.end(), CmpTotalCost);
      for (int i = 0; i < result_limit_; i++) {
        auto result = total_result[i];
        std::vector<double> temp(result.begin(), result.begin() + 9);
        double heading = NormalizeAngle(temp[8] - ego.heading);
        temp[8] = heading;
        temp.emplace_back(0.0);
        temp.emplace_back(0.0);
        agents_input_.emplace_back(temp);
      }
    }
  }

  return agents_input_;
}

std::vector<std::vector<double>> FeaturePreparation::SupplyAgentsInput() {
  // 不足64 agent 要补位
  int agents_input_size = agents_input_.size();
  if (agents_input_size < result_limit_) {
    for (int i = 0; i < result_limit_ - agents_input_size; i++) {
      std::vector<double> temp{70, 5, 20, 5, 0, 0, 2.14, 1.06, 0, 0, 0};
      agents_input_.emplace_back(temp);
    }
  }
  return agents_input_;
}

// std::tie AlignAgents(s agents_window) {
//   map<> window_dict;
//   map<> window_dict_availability;
//   int agent_feature_len = 8;
//   for (auto &frame_agents: agents_window) {
//     for (auto &fai: frame_agents) {
//       std::vector<double> it_v, it_v2;
//       window_dict[fai.first] = it_v;
//       window_dict[fai.first] = it_v2;
//     }
//   }
//   for (auto &wdi: window_dict) {
//     auto track_id = wdi.first;
//     for (auto &frame_agents: agents_window) {
//       auto iter = frame_agents.find(track_id);
//       if (iter != frame_agents.end()) {
//         window_dict[track_id].emplace_back(frame_agents[track_id]);
//         window_dict_availability[track_id].emplace_back(1);
//       } else {
//         std::vector<double> pad_feature{0.0, agent_feature_len};
//         window_dict[track_id].emplace_back(pad_feature);
//         window_dict_availability[track_id].emplace_back(0);
//       }
//     }
//   }
//   window_list_availability;
//   window_list_frature {, 5};
//   for (auto &wdi: window_dict) {
//     window_list_feature[0].emplace_back(wdi.second[0]);
//     window_list_feature[1].emplace_back(wdi.second[1]);
//     window_list_feature[2].emplace_back(wdi.second[2]);
//     window_list_feature[3].emplace_back(wdi.second[3]);
//     window_list_feature[4].emplace_back(wdi.second[4]);
//     window_list_availability.emplace_back(window_dict_availability[wdi.first]);
//   }

//   return window_list_feature, window_list_availability;
// }

std::vector<std::vector<double>> FeaturePreparation::GetEgoFeature(
    const EgoInfo &ego) {
  ego_input_.clear();
  std::vector<double> temp{ego.x,
                           ego.y,
                           ego.heading,
                           ego.curvature,
                           ego.lon_speed,
                           ego.lat_speed,
                           ego.lon_acc,
                           ego.lat_acc,
                           ego.length,
                           ego.width,
                           0};
  ego_window_.emplace_back(temp);

  double lon_speed, lat_speed;
  std::vector<double> pt_in_world{ego.lon_speed, ego.lat_speed};
  std::vector<double> origin_in_world{0, 0, ego.heading};
  std::vector<double> respeed =
      ConvertToRelativeCoordinate2D(pt_in_world, origin_in_world);
  lon_speed = respeed[0];
  lat_speed = respeed[1];
  double lon_acc, lat_acc;
  std::vector<double> pt_in_world2{ego.lon_acc, ego.lat_acc};
  std::vector<double> origin_in_world2{0, 0, ego.heading};
  std::vector<double> reacc =
      ConvertToRelativeCoordinate2D(pt_in_world2, origin_in_world2);
  lon_acc = reacc[0];
  lat_acc = reacc[1];

  std::vector<double> temp2{0,          0,         0,       ego.curvature,
                            lon_speed,  lat_speed, lon_acc, lat_acc,
                            ego.length, ego.width, 0};
  ego_input_.emplace_back(temp2);
  return ego_input_;
}

// std::vector<double> FeaturePreparation::GetEgoFeature(
//     const EgoFrame &frame_ego) {
//   // 保留ego 数据为odom坐标系
//   auto speed = frame_ego.velocity[0];
//   auto acc = frame_ego.linearAcceleration[0];
//   auto lon_speed = speed * cos(frame_ego.yaw);
//   auto lat_speed = speed * sin(frame_ego.yaw);
//   auto lon_acc = acc * cos(frame_ego.yaw);
//   auto lat_acc = acc * sin(frame_ego.yaw);

//   std::vector<double> ego_input = {frame_ego.centroid[0], frame_ego.centroid[1], frame_ego.yaw, 0.0, lon_speed, lat_speed, lon_acc, lat_acc, frame_ego.extent[0], frame_ego.extent[1], 0};
//   return ego_input;
// } 

// std::vector<std::vector<double>> FeaturePreparation::GetReflineFeatureStrong(const std::vector<ReferencePoint> &refline_point100) {
//   refline_input_.clear();
//   int ref_length = refline_point100.size();
//   for (int i = 0; i < ref_length; i++) {
//     auto left_lane_bound_point = refline_point100[i].left_bound_point();
//     auto right_lane_bound_point = refline_point100[i].right_bound_point();
//     auto left_bound_point = refline_point100[i].left_road_bound_point();
//     auto right_bound_point = refline_point100[i].right_road_bound_point();
//     auto x = refline_point100[i].x();
//     auto y = refline_point100[i].y();
//     auto s = refline_point100[i].s();
//     auto theta = refline_point100[i].heading();
//     auto curvature = refline_point100[i].kappa();
//     auto is_stop_line = refline_point100[i].is_in_stop_sign();
//     auto is_crossroad = refline_point100[i].is_in_crosswalk();
//     auto is_speed_bump = refline_point100[i].is_in_speed_bump();
//     auto lane_turn = refline_point100[i].signal_type();

//     int is_stop_line_int = trans_bool2int(is_stop_line);
//     int is_crossroad_int = trans_bool2int(is_crossroad);
//     int is_speed_bump_int = trans_bool2int(is_speed_bump);
    
//     std::vector<double> temp{x, y, theta, curvature, s, left_lane_bound_point.x(), left_lane_bound_point.y(), right_lane_bound_point.x(), right_lane_bound_point.y(), left_bound_point.x(), left_bound_point.y(), right_bound_point.x(), right_bound_point.y(), is_stop_line_int, is_crossroad_int, is_speed_bump_int, lane_turn};
//     refline_input_.emplace_back(temp);
//   }
//   return refline_input_;
// }

std::vector<std::vector<double>> FeaturePreparation::GetReflineFeature(
    const EgoInfo &ego, const Near100SRet &n100) {
  std::vector<std::vector<double>> w_refline_input;
  std::vector<double> refline_point100_x, refline_point100_y;
  int ref_length = n100.nearest_100_refpoint.size();
  for (int i = 0; i < ref_length; i++) {
    refline_point100_x.emplace_back(n100.nearest_100_refpoint[i][0]);
    refline_point100_y.emplace_back(n100.nearest_100_refpoint[i][1]);
  }
  // 世界坐标系
  for (int i = 0; i < ref_length; i++) {
    // double curvature = n100.nearest_100_refpoint_curvature[i];
    std::vector<double> temp{refline_point100_x[i],
                             refline_point100_y[i],
                             n100.nearest_100_refpoint_theta[i],
                             n100.nearest_100_refpoint_curvature[i],
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0};
    w_refline_input.emplace_back(temp);
  }
  refline_window_.emplace_back(w_refline_input);
  // 自车坐标系
  refline_input_.clear();
  for (int i = 0; i < ref_length; i++) {
    std::vector<double> pt_in_world{refline_point100_x[i],
                                    refline_point100_y[i],
                                    n100.nearest_100_refpoint_theta[i]};
    std::vector<double> origin_in_world{ego.x, ego.y, ego.heading};
    std::vector<double> refpoint_egoCoordinate =
        ConvertToRelativeCoordinate(pt_in_world, origin_in_world);
    std::vector<double> temp{refpoint_egoCoordinate[0],
                             refpoint_egoCoordinate[1],
                             refpoint_egoCoordinate[2],
                             n100.nearest_100_refpoint_curvature[i],
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0};
    refline_input_.emplace_back(temp);
  }
  return refline_input_;
}

std::vector<std::vector<std::vector<double>>> FeaturePreparation::GetObsBefore(
    const int &index_steptime, const EgoInfo &ego) {
  // std::vector<std::vector<std::vector<double>>> target_before;
  target_before_.clear();
  if (index_steptime - 4 <= 0) {
    for (int i = 0; i < index_steptime + 1; i++) {
      EgoFrame &frame_ego_temp = this->ego_lists_[i];
      double step_time = step_time_list_[i];
      EgoFrame frame_ego_ahead;
      if (i == 0) {
        frame_ego_ahead = this->ego_lists_[0];
      } else {
        frame_ego_ahead = this->ego_lists_[i - 1];
      }

      std::vector<double> xs_point, ys_point, thetas_point;
      // 暂时改成1
      int p_limit = std::min(5, int(ego_lists_.size()));
      for (int p = 0; p < p_limit; p++) {
        xs_point.emplace_back(ego_lists_[p].centroid[0]);
        ys_point.emplace_back(ego_lists_[p].centroid[1]);
        thetas_point.emplace_back(ego_lists_[p].yaw);
      }
      std::vector<double> curvature =
          CalCurvatureWithTheta(xs_point, ys_point, thetas_point);

      EgoInfo ego_temp = EgoInfo();
      ego_temp.x = frame_ego_temp.centroid[0];
      ego_temp.y = frame_ego_temp.centroid[1];
      ego_temp.heading = frame_ego_temp.yaw;
      ego_temp.speed = 0.0;
      ego_temp.acc = 0.0;
      ego_temp.curvature = curvature[4];  //最后一个
      ego_temp.step_time = step_time;

      std::vector<double> pt_in_world{ego_temp.x, ego_temp.y, ego_temp.heading};
      std::vector<double> origin_in_world{ego.x, ego.y, ego.heading};
      std::vector<double> targetpoint_egoCoordinate =
          ConvertToRelativeCoordinate(pt_in_world, origin_in_world);

      std::vector<double> pt_in_world2{frame_ego_temp.velocity[0],
                                       frame_ego_temp.velocity[1]};
      std::vector<double> origin_in_world2{0, 0, frame_ego_temp.yaw};
      std::vector<double> respeed =
          ConvertToRelativeCoordinate2D(pt_in_world2, origin_in_world2);
      double lon_speed, lat_speed;
      lon_speed = respeed[0];
      lat_speed = respeed[1];

      std::vector<double> pt_in_world3{frame_ego_temp.linearAcceleration[0],
                                       frame_ego_temp.linearAcceleration[1]};
      std::vector<double> origin_in_world3{0, 0, frame_ego_temp.yaw};
      std::vector<double> reacc =
          ConvertToRelativeCoordinate2D(pt_in_world3, origin_in_world3);
      double lon_acc, lat_acc;
      lon_acc = reacc[0];
      lat_acc = reacc[1];

      std::vector<double> pt_in_world4{frame_ego_ahead.velocity[0],
                                       frame_ego_ahead.velocity[1]};
      std::vector<double> origin_in_world4{0, 0, frame_ego_ahead.yaw};
      std::vector<double> respeed_ahead =
          ConvertToRelativeCoordinate2D(pt_in_world4, origin_in_world4);
      double lon_speed_ahead, lat_speed_ahead;
      lon_speed_ahead = respeed_ahead[0];
      lat_speed_ahead = respeed_ahead[1];
      double speed = sqrt(pow(lon_speed, 2) + pow(lat_speed, 2));
      double speed_ahead =
          sqrt(pow(lon_speed_ahead, 2) + pow(lat_speed_ahead, 2));
      double acc = (speed - speed_ahead) / (step_time + 0.00001);

      ego_temp.x = targetpoint_egoCoordinate[0];
      ego_temp.y = targetpoint_egoCoordinate[1];
      ego_temp.heading = targetpoint_egoCoordinate[2];
      ego_temp.speed = speed;
      ego_temp.acc = acc;
      ego_temp.curvature = curvature[4];  //最后一个

      std::vector<std::vector<double>> input_ego;
      std::vector<double> input_ego_temp{ego_temp.x,
                                         ego_temp.y,
                                         ego_temp.heading,
                                         ego_temp.curvature,
                                         lon_speed,
                                         lat_speed,
                                         lon_acc,
                                         lat_acc,
                                         ego.length,
                                         ego.width,
                                         0};
      input_ego.emplace_back(input_ego_temp);

      std::vector<std::vector<double>> input_refline;
      int temp_index = i;
      for (int j = 0; j < int(refline_window_[temp_index].size()); j++) {
        std::vector<double> pt_in_world{refline_window_[temp_index][j][0],
                                        refline_window_[temp_index][j][1],
                                        refline_window_[temp_index][j][2]};
        std::vector<double> origin_in_world{ego.x, ego.y, ego.heading};
        std::vector<double> refpoint_egoCoordinate =
            ConvertToRelativeCoordinate(pt_in_world, origin_in_world);
        std::vector<double> temp{refpoint_egoCoordinate[0],
                                 refpoint_egoCoordinate[1],
                                 refpoint_egoCoordinate[2],
                                 refline_window_[temp_index][j][3],
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0};
        input_refline.emplace_back(temp);
      }
      std::vector<std::vector<double>> single_inputdata_temp(input_ego);
      single_inputdata_temp.insert(single_inputdata_temp.end(),
                                   agents_window_[temp_index].begin(),
                                   agents_window_[temp_index].end());
      single_inputdata_temp.insert(single_inputdata_temp.end(),
                                   input_refline.begin(), input_refline.end());
      // py版本中此处转化成1维tensor然后把frame特征拼接，此处是2维特征按照frame连起来
      target_before_.emplace_back(single_inputdata_temp);
    }

    for (int i = 0; i < 4 - index_steptime; i++) {
      target_before_.insert(target_before_.begin(), target_before_[0]);
    }
  } else {
    for (int i = index_steptime - 4; i < index_steptime + 1; i++) {
      EgoFrame &frame_ego_temp = this->ego_lists_[i];
      double step_time = step_time_list_[i];
      EgoFrame frame_ego_ahead;
      if (i == 0) {
        frame_ego_ahead = this->ego_lists_[0];
      } else {
        frame_ego_ahead = this->ego_lists_[index_steptime - 1];
      }

      std::vector<double> xs_point, ys_point, thetas_point;
      for (int p = index_steptime - 4; p < index_steptime + 1; p++) {
        xs_point.emplace_back(ego_lists_[p].centroid[0]);
        ys_point.emplace_back(ego_lists_[p].centroid[1]);
        thetas_point.emplace_back(ego_lists_[p].yaw);
      }
      std::vector<double> curvature =
          CalCurvatureWithTheta(xs_point, ys_point, thetas_point);

      EgoInfo ego_temp = EgoInfo();
      ego_temp.x = frame_ego_temp.centroid[0];
      ego_temp.y = frame_ego_temp.centroid[1];
      ego_temp.heading = frame_ego_temp.yaw;
      ego_temp.speed = 0.0;
      ego_temp.acc = 0.0;
      ego_temp.curvature = curvature[index_steptime];  //最后一个
      ego_temp.step_time = step_time;

      std::vector<double> pt_in_world{ego_temp.x, ego_temp.y, ego_temp.heading};
      std::vector<double> origin_in_world{ego.x, ego.y, ego.heading};
      std::vector<double> targetpoint_egoCoordinate =
          ConvertToRelativeCoordinate(pt_in_world, origin_in_world);

      std::vector<double> pt_in_world2{frame_ego_temp.velocity[0],
                                       frame_ego_temp.velocity[1]};
      std::vector<double> origin_in_world2{0, 0, frame_ego_temp.yaw};
      std::vector<double> respeed =
          ConvertToRelativeCoordinate2D(pt_in_world2, origin_in_world2);
      double lon_speed, lat_speed;
      lon_speed = respeed[0];
      lat_speed = respeed[1];

      std::vector<double> pt_in_world3{frame_ego_temp.linearAcceleration[0],
                                       frame_ego_temp.linearAcceleration[1]};
      std::vector<double> origin_in_world3{0, 0, frame_ego_temp.yaw};
      std::vector<double> reacc =
          ConvertToRelativeCoordinate2D(pt_in_world3, origin_in_world3);
      double lon_acc, lat_acc;
      lon_acc = reacc[0];
      lat_acc = reacc[1];

      std::vector<double> pt_in_world4{frame_ego_ahead.velocity[0],
                                       frame_ego_ahead.velocity[1]};
      std::vector<double> origin_in_world4{0, 0, frame_ego_ahead.yaw};
      std::vector<double> respeed_ahead =
          ConvertToRelativeCoordinate2D(pt_in_world4, origin_in_world4);
      double lon_speed_ahead, lat_speed_ahead;
      lon_speed_ahead = respeed_ahead[0];
      lat_speed_ahead = respeed_ahead[1];
      double speed = sqrt(pow(lon_speed, 2) + pow(lat_speed, 2));
      double speed_ahead =
          sqrt(pow(lon_speed_ahead, 2) + pow(lat_speed_ahead, 2));
      double acc = (speed - speed_ahead) / (step_time + 0.00001);

      ego_temp.x = targetpoint_egoCoordinate[0];
      ego_temp.y = targetpoint_egoCoordinate[1];
      ego_temp.heading = targetpoint_egoCoordinate[2];
      ego_temp.speed = speed;
      ego_temp.acc = acc;
      ego_temp.curvature = curvature[index_steptime];  //最后一个

      std::vector<std::vector<double>> input_ego;
      std::vector<double> input_ego_temp{ego_temp.x,
                                         ego_temp.y,
                                         ego_temp.heading,
                                         ego_temp.curvature,
                                         lon_speed,
                                         lat_speed,
                                         lon_acc,
                                         lat_acc,
                                         ego.length,
                                         ego.width,
                                         0};
      input_ego.emplace_back(input_ego_temp);

      std::vector<std::vector<double>> input_refline;
      int temp_index = i;
      for (int j = 0; j < int(refline_window_[temp_index].size()); j++) {
        std::vector<double> pt_in_world{refline_window_[temp_index][j][0],
                                        refline_window_[temp_index][j][1],
                                        refline_window_[temp_index][j][2]};
        std::vector<double> origin_in_world{ego.x, ego.y, ego.heading};
        std::vector<double> refpoint_egoCoordinate =
            ConvertToRelativeCoordinate(pt_in_world, origin_in_world);
        std::vector<double> temp{refpoint_egoCoordinate[0],
                                 refpoint_egoCoordinate[1],
                                 refpoint_egoCoordinate[2],
                                 refline_window_[temp_index][j][3],
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0};
        input_refline.emplace_back(temp);
      }
      std::vector<std::vector<double>> single_inputdata_temp(input_ego);
      single_inputdata_temp.insert(single_inputdata_temp.end(),
                                   agents_window_[temp_index].begin(),
                                   agents_window_[temp_index].end());
      single_inputdata_temp.insert(single_inputdata_temp.end(),
                                   input_refline.begin(), input_refline.end());
      // py版本中此处转化成1维tensor然后把frame特征拼接，此处是2维特征按照frame连起来
      target_before_.emplace_back(single_inputdata_temp);
    }
  }
  return target_before_;
}

// std::vector<std::vector<std::vector<double>>> FeaturePreparation::GetObsBeforeSafety(
//     const int &index_steptime, const EgoInfo &ego) {
//   target_before_.clear();
//   int frame_num = ego_window_.size();
//   for (int i = 0; i < frame_num; i++) {
//     auto single_ego = ego_window_[i];
//     auto ego_input_loc = ConvertToRelativeCoordinate6input(single_ego[0], single_ego[1], single_ego[2], ego.x, ego.y, ego.heading);
//     auto ego_input_speed = ConvertToRelativeCoordinate6input(single_ego[4], single_ego[5], single_ego[2], ego.lon_speed, ego.lat_speed, ego.heading);
//     auto ego_input_acc = ConvertToRelativeCoordinate6input(single_ego[6], single_ego[7], single_ego[2], ego.lon_acc, ego.lat_acc, ego.heading);
//     std::vector<double> ego_input_temp = {ego_input_loc[0], ego_input_loc[1], ego_input_loc[2], single_ego[3], ego_input_speed[0], ego_input_speed[1], ego_input_acc[0], ego_input_acc[1], single_ego[8], single_ego[9], 0};
//     std::vector<std::vector<double>> input_ego;
//     input_ego.emplace_back(ego_input_temp);

//     std::vector<std::vector<double>> input_refline;
//     auto single_refline = refline_window_[i];
//     int single_refline_length = single_refline.size();
//     for (int j = 0; j < single_refline_length; j++) {
//       auto refpoint_egoCoordinate = ConvertToRelativeCoordinate6input(single_refline[j][0], single_refline[j][1], single_refline[j][2], ego.x, ego.y, ego.heading);
//       auto curvature = single_refline[j][3];
//       std::vector<double> temp{refpoint_egoCoordinate[0], refpoint_egoCoordinate[1], refpoint_egoCoordinate[2], curvature, 0, 0, 0, 0, 0, 0, 0};
//       input_refline.emplace_back(temp);
//     }

//     std::vector<std::vector<double>> single_inputdata_temp(input_ego);
//     single_inputdata_temp.insert(single_inputdata_temp.end(),
//                                  agents_window_[i].begin(),
//                                  agents_window_[i].end());
//     single_inputdata_temp.insert(single_inputdata_temp.end(),
//                                  input_refline.begin(), input_refline.end());
//     target_before_.emplace_back(single_inputdata_temp);
//   }
//   return target_before_;
// }

// std::vector<std::vector<std::vector<double>>> FeaturePreparation::GetObsBeforeOriginalFrenetStrong(std::vector<ReferencePoint> nearest_100m_refpoint) {
//   target_before_.clear();
//   int frame_num = ego_window_.size();
//   for (int i = 0; i < frame_num; i++) {
//     auto single_ego = ego_window_[i];
//     std::vector<double> ADCpoint {single_ego[0], single_ego[1]};
//     int nearest_index = FindNearestReflinePoint(nearest_100m_refpoint, ADCpoint);
//     auto refline_points_nearest_ego = nearest_100m_refpoint[nearest_index];
//     std::vector<std::vector<double>> traffic_light_input;
//     auto single_traffic_light = traffic_light_window_[i];
//     for (auto $trafic_light : single_traffic_light) {
//       std::vector<double> temp_tl{traffic_light.type, traffic_light.color};
//       traffic_light_input.emplace_back(temp_tl);
//     }
//     int tl_input_length = traffic_light_input.size();
//     if (tl_input_length > 4) {
//       traffic_light_input = traffic_light_input.assign(traffic_light_input.begin(), traffic_light_input.begin() + 4);
//     } else if (tl_input_length < 4) {
//       for (int j = 0; j < 4 - tl_input_length; j++) {
//         std::vector<double> temp_tl{4, 3};
//         traffic_light_input.emplace_back(temp_tl);
//       }
//     }
//     auto single_ego = ego_window_[i];
//     auto ego_input_loc = ConvertToRelativeCoordinate6input(single_ego[0], single_ego[1], single_ego[2], refline_points_nearest_ego.x(), refline_points_nearest_ego.y(), refline_points_nearest_ego.heading());
//     auto ego_input_speed = ConvertToRelativeCoordinate6input(single_ego[4], single_ego[5], single_ego[2], 0, 0, refline_points_nearest_ego.heading());
//     auto ego_input_acc = ConvertToRelativeCoordinate6input(single_ego[6], single_ego[7], single_ego[2], 0, 0, refline_points_nearest_ego.heading());
//     std::vector<double> ego_input_temp = {ego_input_loc[0], ego_input_loc[1], ego_input_loc[2], single_ego[3], ego_input_speed[0], ego_input_speed[1], ego_input_acc[0], ego_input_acc[1], single_ego[8], single_ego[9], traffic_light_input[0][0], traffic_light_input[0][1], traffic_light_input[1][0], traffic_light_input[1][1], traffic_light_input[2][0], traffic_light_input[2][1], traffic_light_input[3][0], traffic_light_input[3][1]};
//     std::vector<std::vector<double>> input_ego;
//     input_ego.emplace_back(ego_input_temp);
    
//     std::vector<std::vector<double>> input_refline;
//     auto single_refline = refline_window_[i];
//     int single_refline_length = single_refline.size();
//     for (int j = 0; j < single_refline_length; j++) {
//       auto left_lane_bound_point = ConvertToRelativeCoordinate6input(single_refline[j][5], single_refline[j][6], single_refline[j][2], single_refline[j][0], single_refline[j][1], single_refline[j][2]);
//       auto right_lane_bound_point = ConvertToRelativeCoordinate6input(single_refline[j][7], single_refline[j][8], single_refline[j][2], single_refline[j][0], single_refline[j][1], single_refline[j][2]);
//       auto left_bound_point = ConvertToRelativeCoordinate6input(single_refline[j][9], single_refline[j][10], single_refline[j][2], single_refline[j][0], single_refline[j][1], single_refline[j][2]);
//       auto right_bound_point = ConvertToRelativeCoordinate6input(single_refline[j][11], single_refline[j][12], single_refline[j][2], single_refline[j][0], single_refline[j][1], single_refline[j][2]);
//       auto current_s = single_refline[j][4] - refline_points_nearest_ego.s();
//       left_lane_bound_point[0] = left_lane_bound_point[0] + current_s;
//       right_lane_bound_point[0] = right_lane_bound_point[0] + current_s;
//       left_bound_point[0] = left_bound_point[0] + current_s;
//       right_bound_point[0] = right_bound_point[0] + current_s;

//       std::vector<double> temp{current_s, 0, 0, signle_refline[j][3], left_lane_bound_point[0], left_lane_bound_point[1], right_lane_bound_point[0], right_lane_bound_point[1], left_bound_point[0], left_bound_point[1], right_bound_point[0], right_bound_point[1], single_refline[j][13], single_refline[j][14], single_refline[j][15], single_refline[j][16], 0, 0};
//       input_refline.emplace_back(temp);
//     }
//     std::vector<std::vector<double>> agents_input;
//     auto single_agents = agents_window_[i];
//     for (auto &agent: single_agents) {
//       if (std::any_of(agent.begin(), agent.end(), [](size_t l){return l > 0})) {
//         std::vector<double> agentpoint{agent[0], agent[1]};
//         int nearest_index = FindNearestReflinePoint(nearest_100m_refpoint, agentpoint);
//         auto refline_points_nearest = nearest_100m_refpoint[nearest_index];

//         auto agent_input_loc = ConvertToRelativeCoordinate6input(agent[0], agent[1], agent[2], refline_points_nearest.x(), refline_points_nearest.y(), refline_points_nearest.theta());
//         auto agent_input_speed = ConvertToRelativeCoordinate6input(agent[3], agent[4], agent[2], 0, 0, refline_points_nearest.theta());
//         agent_input_loc[0] = agent_input_loc[0] + refline_points_nearest.s() - refline_points_nearest_ego.s();
//         double dis = sqrt(pow(agent_input_loc[0] - ego_input_loc[0], 2) + pow(agent_input_loc[1] - ego_input_loc[1], 2));
//         std::vector<double> agent_input = {agent_input_loc[0], agent_input_loc[1], agent_input_loc[2], agent_input_speed[0], agent_input_speed[1], agent[5], agent[6], agent[7], dis, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//         agents_input.emplace_back(agent_input);
//       } else {
//         std::vector<double> agent_input = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//         agents_input.emplace_back(agent_input);
//       }
//     }
//     int len_agents_input = int(agents_input.size());
//     if (len_agents_input <= 64) {
//       for (int j = 0; j <= (64 - len_agents_input); j++) {
//         double dis = sqrt(pow(100 - ego_input_loc[0], 2) + pow(5 - ego_input_loc[1], 2));
//         std::vector<double> agent_input{100, 5, ego_input_loc[2], ego_input_speed[0], ego_input_speed[1], single_ego[8], single_ego[9], 1, dis, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//         agents_input.emplace_back(agent_input);
//       }
//     } else {
//       agents_input = agents_input.assign(agents_input.begin(), agents_input.begin() + 64);
//     }
//     std::vector<std::vector<double>> single_inputdata_temp(input_ego);
//     single_inputdata_temp.insert(single_inputdata_temp.end(),
//                                  agents_input.begin(),
//                                  agents_input.end());
//     single_inputdata_temp.insert(single_inputdata_temp.end(),
//                                  input_refline.begin(), input_refline.end());
//     target_before_.emplace_back(single_inputdata_temp);
//   }
//   return target_before_;
// }

// std::vector<std::vector<std::vector<double>>> FeaturePreparation::GetObs(const int &frame_index, const EgoInfo &ego) {
//   bool is_use_safety = false;
//   std::vector<int> index_list;
//   int horizon = 5;
//   int sample_num = 100;
//   if (frame_index >= horizon -1) {
//     for (int i = frame_index - horizon + 1; i < frame_index + 1; i++) {
//       index_list.emplace_back(i);
//     }
//   } else {
//     for (int i = 0; i < horizon -1 -frame_index; i++) {
//       index_list.emplace_back(0);
//     }
//     for (int i = 0; i < frame_index + 1; i++) {
//       index_list.emplace_back(i);
//     }
//   }
//   for (auto &frame_i : index_list) {
//     // 待确认是否为ego_list
//     auto frame_ego = ego_lists_[frame_i];
//     auto frame_agents = agents_lists_[frame_i];
//     auto reference_line_points = refline_points_;
//     std::vector<double> adc_point{frame_ego.centroid[0], frame_ego.centroid[1]};
//     std::vector<ReferencePoint> refline_point100, nearest_100m_refpoint;
//     std::tie(refline_point100, nearest_100m_refpoint) = FindNearest110MeterSample(reference_line_points, frame_ego, sample_num);

//     // input 1、选 64 个最关注的的agent 表征
//     if (is_use_safety) {
//       // agents_input_ = GetAgentFeature(ego, ego_data_, agents_data_, refline_point100_meter);
//     } else {
//       GetAgentFeatureOriginal(frame_agents);
//     }
//     agents_window_.emplace_back(agents_input_);

//     // input 2、ego 表征
//     auto ego_input = GetEgoFeature(frame_ego);
//     ego_window_.emplace_back(ego_input);

//     // input 3、refline 100 个点表征
//     auto refline_input = GetReflineFeatureStrong(refline_point100);
//     refline_window_.emplace_back(refline_input);

//     // input 4 traffic lights表征
//     auto traffic_light = frame_ego.traffic_light();
//     traffic_light_window_.emplace_back(traffic_light);
//   }

//   EgoInfo center_point = EgoInfo(ego_lists_.back());

//   std::vector<std::vector<double>> vision_before;
//   if (is_use_safety) {
//     vision_before = GetObsBeforeSafety(center_point);
//   } else {
//     vision_before = GetObsBeforeOriginalFrenetStrong(nearest_100m_refpoint);
//   }
//   return vision_before;
// }

// std::vector<std::vector<std::vector<double>>> FeaturePreparation::GetObsUrbanDriver(const int &frame_index, const EgoInfo &ego) {
//   bool is_use_safety = false;
//   std::vector<int> index_list;
//   int horizon = 5;
//   int sample_num = 100;
//   if (frame_index >= horizon -1) {
//     for (int i = frame_index - horizon + 1; i < frame_index + 1; i++) {
//       index_list.emplace_back(i);
//     }
//   } else {
//     for (int i = 0; i < horizon -1 -frame_index; i++) {
//       index_list.emplace_back(0);
//     }
//     for (int i = 0; i < frame_index + 1; i++) {
//       index_list.emplace_back(i);
//     }
//   }
//   for (auto &frame_i : index_list) {
//     // 待确认是否为ego_list
//     auto frame_ego = ego_lists_[frame_i];
//     auto frame_agents = agents_lists_[frame_i];
//     auto reference_line_points = refline_points_;
//     std::vector<double> adc_point{frame_ego.centroid[0], frame_ego.centroid[1]};
//     std::vector<ReferencePoint> refline_point100, nearest_100m_refpoint;
//     std::tie(refline_point100, nearest_100m_refpoint) = FindNearest110MeterSample(reference_line_points, frame_ego, sample_num);

//     // input 1、选 64 个最关注的的agent 表征
//     if (is_use_safety) {
//       // agents_input_ = GetAgentFeature(ego, ego_data_, agents_data_, refline_point100_meter);
//       agents_window_.emplace_back(agents_input_);
//     } else {
//       GetAgentFeatureOriginal(frame_agents);
//       map<size_t, std::vector<std::vector<double>>> agents_dict;
//       int agents_input_len = agents_input_.size();
//       for (int agent_index = 0; agent_index < agnets_input_len; agent_index++) {
//         auto frame_agent = frame_agents[agent_index];
//         auto agent_input = agents_input_[agent_index];
//         auto agent_id = frame_agent.track_id;
//         agents_dict[agent_id] = agent_input;
//       }
//       agents_window_dict_.emplace_back(agents_dict);
//     }

//     // input 2、ego 表征
//     auto ego_input = GetEgoFeature(frame_ego);
//     ego_window_.emplace_back(ego_input);

//     // input 3、refline 100 个点表征
//     auto refline_input = GetReflineFeatureStrong(refline_point100);
//     refline_window_.emplace_back(refline_input);

//     // input 4 traffic lights表征
//     auto traffic_light = frame_ego.traffic_light();
//     traffic_light_window_.emplace_back(traffic_light);
//   }

//   std::tie(agents_window, agents_window_availability) = AlignAgents(agents_window_dict);

//   EgoInfo center_point = EgoInfo(ego_lists_.back());

//   std::vector<std::vector<double>> vision_before;
//   if (is_use_safety) {
//     vision_before = GetObsBeforeSafety(center_point);
//   } else {
//     vision_before = GetObsBeforeOriginalFrenetStrong(nearest_100m_refpoint);
//   }
//   return vision_before;
// }

void FeaturePreparation::GetAllData() {
  // GetData();
  int frame_index = 0;
  // time_log_.ResetStartTime();
  for (frame_index = 0; frame_index < 5; frame_index++) {
    ego_data_ = ego_lists_[frame_index];
    agents_data_ = agents_lists_[frame_index];
    step_time_ = step_time_list_[frame_index];
    std::vector<double> adc_point{ego_data_.centroid[0], ego_data_.centroid[1]};
    Near100SRet n100rs = FindNearest100(refline_points_, refline_curvatures_,
                                        thetas_, adc_point);
    // std::vector<std::vector<double>> refline_point100 =
    // n100rs.nearest_100_refpoint; std::vector<double>
    // refline_point100_curvature = n100rs.nearest_100_refpoint_curvature;
    std::vector<std::vector<double>> refline_point100_meter =
        FindNearest100Meter(refline_points_, refline_curvatures_, thetas_,
                            adc_point);
    // std::cout << "ego_data" << std::endl;
    // std::cout << ego_data_.centroid << std::endl;
    // std::cout << ego_data_.yaw << std::endl;
    // std::cout << ego_data_.velocity << std::endl;
    // std::cout << ego_data_.linearAcceleration << std::endl;
    // std::cout << ego_data_.extent << std::endl;
    // std::cout << agents_data_[0].centroid << std::endl;
    // std::cout << step_time_ << std::endl;
    // std::cout << "refline_point100" << std::endl;
    // std::cout << refline_point100_meter << std::endl;
    reference_line_points_optj_ =
        FindNearest100MeterRefpoints(refline_points_, ego_data_.centroid);
    EgoInfo ego = EgoInfo(ego_data_, n100rs.curvature, step_time_);
    ego_info_ = ego;
    // agent特征
    GetAgentFeature(ego, ego_data_, agents_data_, refline_point100_meter);
    // std::cout << "agent_intput:" << agents_input_ << std::endl;
    LOG_DEBUG("agent feature done");
    time_log_.RegisterTimeAndPrint("GetAgentFeature");
    // ego特征
    GetEgoFeature(ego);
    // std::cout << "ego_input:" << ego_input_ << std::endl;
    time_log_.RegisterTimeAndPrint("GetEgoFeature");
    LOG_DEBUG("ego feature done");
    // refline特征
    GetReflineFeature(ego, n100rs);
    // std::cout << "refline_input:" << refline_input_ << std::endl;
    time_log_.RegisterTimeAndPrint("GetReflineFeature");
    LOG_DEBUG("refline feature done");
    GetObsBefore(frame_index, ego);
    time_log_.RegisterTimeAndPrint("GetObsBefore");
    LOG_DEBUG("obs before feature done");
  }
}

std::vector<std::vector<std::vector<double>>>
FeaturePreparation::GetAllDataFrame(int frame_index) {
  ego_data_ = ego_lists_[frame_index];
  agents_data_ = agents_lists_[frame_index];
  step_time_ = step_time_list_[frame_index];
  std::vector<double> adc_point{ego_data_.centroid[0], ego_data_.centroid[1]};
  Near100SRet n100rs =
      FindNearest100(refline_points_, refline_curvatures_, thetas_, adc_point);
  std::vector<std::vector<double>> refline_point100_meter = FindNearest100Meter(
      refline_points_, refline_curvatures_, thetas_, adc_point);

  EgoInfo ego = EgoInfo(ego_data_, n100rs.curvature, step_time_);
  ego_info_ = ego;

  // agent特征
  GetAgentFeature(ego, ego_data_, agents_data_, refline_point100_meter);
  // ego特征
  GetEgoFeature(ego);
  // refline特征
  GetReflineFeature(ego, n100rs);

  std::vector<std::vector<std::vector<double>>> obs_before =
      GetObsBefore(frame_index, ego);
  return obs_before;
}

int Tester() {
  auto fp = FeaturePreparation();
  EgoFrame ef = EgoFrame();
  std::vector<EgoFrame> egoframes{ef};
  AgentFrame af = AgentFrame();
  std::vector<AgentFrame> afl{af};
  std::vector<std::vector<AgentFrame>> agentsframes{afl};
  std::vector<double> rfp{0, 1, 0.3};
  std::vector<std::vector<double>> refline_input{rfp};
  std::vector<double> step_times{0.2, 0.2};
  std::vector<double> refline_curvatures{3};
  std::vector<double> thetas{0.2};
  fp.GetData(egoframes, agentsframes, refline_input, step_times,
             refline_curvatures, thetas);
  auto ret = fp.GetAllDataFrame(0);
  return 0;
}

int Testa() {
  auto fp = FeaturePreparation();
  EgoFrame ef = EgoFrame();
  AgentFrame af = AgentFrame();
  std::vector<AgentFrame> afl{af};
  std::vector<double> rfp{0, 1, 0.3};
  std::vector<std::vector<double>> refline_input{rfp};
  std::vector<std::vector<double>> ret =
      GetAgentRepresentationTrt(ef, afl, refline_input);
  return 0;
}

}  // namespace planning_rl
}  // namespace neodrive