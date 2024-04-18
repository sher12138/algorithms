#include "train_features_v2.h"

namespace neodrive {
namespace planning_rl {

// void FeaturePreparationV2::GetData(
//     const std::vector<EgoFrame> &egoframes,
//     const std::vector<std::vector<AgentFrame>> &agentsframes,
//     const std::vector<std::vector<double>> &refline_input,
//     const std::vector<double> &step_times,
//     const std::vector<double> &refline_curvatures,
//     const std::vector<double> &thetas) {
//   ego_lists_ = egoframes;
//   agents_lists_ = agentsframes;
//   refline_points_ = refline_input;
//   step_time_list_ = step_times;
//   this->refline_curvatures_ = refline_curvatures;
//   this->thetas_ = thetas;
//   frame_num_ = egoframes.size();
// }

void FeaturePreparationV2::GetData(
    const ContainerMessageShrPtr &container_msg,
    const std::vector<EgoFrame> &egoframes,
    const std::vector<std::vector<AgentFrame>> &agentsframes,
    const std::vector<std::vector<double>> &refline_input,
    const std::vector<double> &step_times,
    const std::vector<double> &refline_curvatures,
    const std::vector<double> &thetas) {
  container_msg_ = container_msg;
  ego_lists_ = egoframes;
  agents_lists_ = agentsframes;
  refline_points_ = container_msg->reference_line_ptr->get_reference_points();
  traffic_lights_ = container_msg->traffic_lights_ptr->traffic_lights();
  step_time_list_ = step_times;
  this->refline_curvatures_ = refline_curvatures;
  this->thetas_ = thetas;
  frame_num_ = egoframes.size();
}

std::vector<std::vector<double>> FeaturePreparationV2::GetAgentFeatureOriginal(std::vector<AgentFrame> &agents_frame) {
  agents_input_.clear();
  ConstructTypeMap();
  for (auto &agent : agents_frame) {
    int agent_type_e = 0;
    if (type_map_.count(agent.type) > 0) {
      agent_type_e = type_map_[agent.type];
    }
    std::vector<double> frame_agent_feature {agent.centroid[0], agent.centroid[1], agent.yaw, agent.velocity[0], agent.velocity[1], agent.extent[0], agent.extent[1], agent_type_e};
    // std::cout << "frame_agent_feature" << std::endl;
    // std::cout << frame_agent_feature << std::endl;
    agents_input_.emplace_back(frame_agent_feature);
  }
  return agents_input_;
}

std::tuple<std::vector<std::vector<std::vector<double>>>, std::vector<std::vector<double>>> FeaturePreparationV2::AlignAgents(std::vector<std::map<size_t, std::vector<double>>> agents_window) {
  std::map<size_t, std::vector<std::vector<double>>> window_dict;
  std::map<size_t, std::vector<double>> window_dict_availability;
  int agent_feature_len = 8;
  int agents_window_len = agents_window.size();
  for (auto &frame_agents: agents_window) {
    for (auto &fai: frame_agents) {
      std::vector<std::vector<double>> it_v, it_v2;
      window_dict[fai.first] = it_v;
      window_dict[fai.first] = it_v2;
    }
  }
  for (auto &wdi: window_dict) {
    auto track_id = wdi.first;
    for (auto &frame_agents: agents_window) {
      auto iter = frame_agents.find(track_id);
      if (iter != frame_agents.end()) {
        window_dict[track_id].emplace_back(frame_agents[track_id]);
        window_dict_availability[track_id].emplace_back(1);
      } else {
        std::vector<double> pad_feature{0.0, agent_feature_len};
        window_dict[track_id].emplace_back(pad_feature);
        window_dict_availability[track_id].emplace_back(0);
      }
    }
  }
  std::vector<std::vector<double>> window_list_availability;// = std::vector<std::vector<double>>{agents_window_len, std::vector<double>{}}; //std::vector<double>(0.0, agents_window_len);
  std::vector<std::vector<std::vector<double>>> window_list_feature {5, std::vector<std::vector<double>>{}};
  for (auto &wdi: window_dict) {
    window_list_feature[0].emplace_back(wdi.second[0]);
    window_list_feature[1].emplace_back(wdi.second[1]);
    window_list_feature[2].emplace_back(wdi.second[2]);
    window_list_feature[3].emplace_back(wdi.second[3]);
    window_list_feature[4].emplace_back(wdi.second[4]);
    // std::cout << "window_dict_availability:" << window_dict_availability[wdi.first] << std::endl;
    window_list_availability.emplace_back(window_dict_availability[wdi.first]);
  }
  // std::cout << "window_list_availability" << std::endl;
  // std::cout << window_list_availability << std::endl;

  return std::make_tuple(window_list_feature, window_list_availability);
}

std::vector<double> FeaturePreparationV2::GetEgoFeature(
    const EgoFrame &frame_ego) {
  // 保留ego 数据为odom坐标系
  auto speed = frame_ego.velocity[0];
  auto acc = frame_ego.linearAcceleration[0];
  auto lon_speed = speed * cos(frame_ego.yaw);
  auto lat_speed = speed * sin(frame_ego.yaw);
  auto lon_acc = acc * cos(frame_ego.yaw);
  auto lat_acc = acc * sin(frame_ego.yaw);

  std::vector<double> ego_input = {frame_ego.centroid[0], frame_ego.centroid[1], frame_ego.yaw, 0.0, lon_speed, lat_speed, lon_acc, lat_acc, frame_ego.extent[0], frame_ego.extent[1], 0};
  // std::cout << "ego_input" << std::endl;
  // std::cout << ego_input << std::endl;
  return ego_input;
} 

std::vector<std::vector<double>> FeaturePreparationV2::GetReflineFeatureStrong(const std::vector<ReferencePoint> &refline_point100) {
  refline_input_.clear();
  int ref_length = refline_point100.size();
  for (int i = 0; i < ref_length; i++) {
    auto left_lane_bound_point = refline_point100[i].left_bound_point();
    auto right_lane_bound_point = refline_point100[i].right_bound_point();
    auto left_bound_point = refline_point100[i].left_road_bound_point();
    auto right_bound_point = refline_point100[i].right_road_bound_point();
    auto x = refline_point100[i].x();
    auto y = refline_point100[i].y();
    auto s = refline_point100[i].s();
    auto theta = refline_point100[i].heading();
    auto curvature = refline_point100[i].kappa();
    auto is_stop_line = refline_point100[i].is_in_stop_sign();
    auto is_crossroad = refline_point100[i].is_in_crosswalk();
    auto is_speed_bump = refline_point100[i].is_in_speed_bump();
    auto lane_turn = refline_point100[i].signal_type() + 1; //python版本1-4, c++版本0-3, 需要+1

    int is_stop_line_int = trans_bool2int(is_stop_line);
    int is_crossroad_int = trans_bool2int(is_crossroad);
    int is_speed_bump_int = trans_bool2int(is_speed_bump);
    
    std::vector<double> temp{x, y, theta, curvature, s, left_lane_bound_point.x(), left_lane_bound_point.y(), right_lane_bound_point.x(), right_lane_bound_point.y(), left_bound_point.x(), left_bound_point.y(), right_bound_point.x(), right_bound_point.y(), is_stop_line_int, is_crossroad_int, is_speed_bump_int, lane_turn};
    refline_input_.emplace_back(temp);
  }
  return refline_input_;
}

void FeaturePreparationV2::ConstructTypeMap() {
  type_map_["UNKNOWN"] = 0;
  type_map_["UNKNOWN_MOVABLE"] = 1;
  type_map_["UNKNOWN_UNMOVABLE"] = 2;
  type_map_["PEDESTRIAN"] = 3;
  type_map_["BICYCLE"] = 4;
  type_map_["VEHICLE"] = 5;
}

std::vector<std::vector<std::vector<double>>> FeaturePreparationV2::GetObsBeforeSafety(const EgoInfo &ego) {
  target_before_.clear();
  int frame_num = ego_window_.size();
  for (int i = 0; i < frame_num; i++) {
    auto single_ego = ego_window_[i];
    auto ego_input_loc = ConvertToRelativeCoordinate6input(single_ego[0], single_ego[1], single_ego[2], ego.x, ego.y, ego.heading);
    auto ego_input_speed = ConvertToRelativeCoordinate6input(single_ego[4], single_ego[5], single_ego[2], ego.lon_speed, ego.lat_speed, ego.heading);
    auto ego_input_acc = ConvertToRelativeCoordinate6input(single_ego[6], single_ego[7], single_ego[2], ego.lon_acc, ego.lat_acc, ego.heading);
    std::vector<double> ego_input_temp = {ego_input_loc[0], ego_input_loc[1], ego_input_loc[2], single_ego[3], ego_input_speed[0], ego_input_speed[1], ego_input_acc[0], ego_input_acc[1], single_ego[8], single_ego[9], 0};
    std::vector<std::vector<double>> input_ego;
    input_ego.emplace_back(ego_input_temp);

    std::vector<std::vector<double>> input_refline;
    auto single_refline = refline_window_[i];
    int single_refline_length = single_refline.size();
    for (int j = 0; j < single_refline_length; j++) {
      auto refpoint_egoCoordinate = ConvertToRelativeCoordinate6input(single_refline[j][0], single_refline[j][1], single_refline[j][2], ego.x, ego.y, ego.heading);
      auto curvature = single_refline[j][3];
      std::vector<double> temp{refpoint_egoCoordinate[0], refpoint_egoCoordinate[1], refpoint_egoCoordinate[2], curvature, 0, 0, 0, 0, 0, 0, 0};
      input_refline.emplace_back(temp);
    }

    std::vector<std::vector<double>> single_inputdata_temp(input_ego);
    single_inputdata_temp.insert(single_inputdata_temp.end(),
                                 agents_window_[i].begin(),
                                 agents_window_[i].end());
    single_inputdata_temp.insert(single_inputdata_temp.end(),
                                 input_refline.begin(), input_refline.end());
    target_before_.emplace_back(single_inputdata_temp);
  }
  return target_before_;
}

std::vector<std::vector<std::vector<double>>> FeaturePreparationV2::GetObsBeforeOriginalFrenetStrong(std::vector<ReferencePoint> nearest_100m_refpoint) {
  target_before_.clear();
  int frame_num = ego_window_.size();
  double accumulate_s = 0;
  double current_centerpoint_s = 0;
  for (int i = 0; i < frame_num; i++) {
    auto single_ego = ego_window_[i];
    std::vector<double> ADCpoint {single_ego[0], single_ego[1]};
    int nearest_index = FindNearestReflinePoint(nearest_100m_refpoint, ADCpoint);
    auto refline_points_nearest_ego = nearest_100m_refpoint[nearest_index];
    // std::cout << nearest_index << std::endl;
    // std::cout << refline_points_nearest_ego.x() << std::endl;
    // std::cout << refline_points_nearest_ego.y() << std::endl;
    // std::cout << refline_points_nearest_ego.s() << std::endl;
    // std::cout << refline_points_nearest_ego.heading() << std::endl;
    if (i == 0) {
      current_centerpoint_s = refline_points_nearest_ego.s();
    } else {
      accumulate_s = refline_points_nearest_ego.s() - current_centerpoint_s;
    }
    std::vector<std::vector<double>> traffic_light_input;
    auto single_traffic_light = traffic_light_window_[i];
    for (auto &traffic_light : single_traffic_light) {
      // std::vector<double> temp_tl{traffic_light.type, traffic_light.color}; temp
      // std::vector<double> temp_tl{4,3};
      std::vector<double> temp_tl{traffic_light.type(), traffic_light.color()};
      traffic_light_input.emplace_back(temp_tl);
    }
    int tl_input_length = traffic_light_input.size();
    if (tl_input_length > 4) {
      std::vector<std::vector<double>> temp_tl_input;
      temp_tl_input.assign(traffic_light_input.begin(), traffic_light_input.begin() + 4);
      traffic_light_input = temp_tl_input;
    } else if (tl_input_length < 4) {
      for (int j = 0; j < 4 - tl_input_length; j++) {
        std::vector<double> temp_tl{4, 3};
        traffic_light_input.emplace_back(temp_tl);
      }
    }
    // std::cout << "traffic_light_input" << std::endl;
    // std::cout << traffic_light_input << std::endl;

    // tag1
    auto ego_input_loc = ConvertToRelativeCoordinate6input(single_ego[0], single_ego[1], single_ego[2], refline_points_nearest_ego.x(), refline_points_nearest_ego.y(), refline_points_nearest_ego.heading());
    auto ego_input_speed = ConvertToRelativeCoordinate6input(single_ego[4], single_ego[5], single_ego[2], 0, 0, refline_points_nearest_ego.heading());
    auto ego_input_acc = ConvertToRelativeCoordinate6input(single_ego[6], single_ego[7], single_ego[2], 0, 0, refline_points_nearest_ego.heading());
    ego_input_loc[0] = ego_input_loc[0] + accumulate_s;
    std::vector<double> ego_input_temp = {ego_input_loc[0], ego_input_loc[1], ego_input_loc[2], single_ego[3], ego_input_speed[0], ego_input_speed[1], ego_input_acc[0], ego_input_acc[1], single_ego[8], single_ego[9], traffic_light_input[0][0], traffic_light_input[0][1], traffic_light_input[1][0], traffic_light_input[1][1], traffic_light_input[2][0], traffic_light_input[2][1], traffic_light_input[3][0], traffic_light_input[3][1]};
    std::vector<std::vector<double>> input_ego;
    // std::cout << "ego_input_temp" << std::endl;
    // std::cout << ego_input_temp << std::endl;
    input_ego.emplace_back(ego_input_temp);
    
    std::vector<std::vector<double>> input_refline;
    auto single_refline = refline_window_[i];
    // std::cout << "single_refline" << std::endl;
    // std::cout << single_refline << std::endl;
    int single_refline_length = single_refline.size();
    for (int j = 0; j < single_refline_length; j++) {
      auto left_lane_bound_point = ConvertToRelativeCoordinate6input(single_refline[j][5], single_refline[j][6], single_refline[j][2], single_refline[j][0], single_refline[j][1], single_refline[j][2]);
      auto right_lane_bound_point = ConvertToRelativeCoordinate6input(single_refline[j][7], single_refline[j][8], single_refline[j][2], single_refline[j][0], single_refline[j][1], single_refline[j][2]);
      auto left_bound_point = ConvertToRelativeCoordinate6input(single_refline[j][9], single_refline[j][10], single_refline[j][2], single_refline[j][0], single_refline[j][1], single_refline[j][2]);
      auto right_bound_point = ConvertToRelativeCoordinate6input(single_refline[j][11], single_refline[j][12], single_refline[j][2], single_refline[j][0], single_refline[j][1], single_refline[j][2]);
      auto current_s = single_refline[j][4] - refline_points_nearest_ego.s() + accumulate_s;
      left_lane_bound_point[0] = left_lane_bound_point[0] + current_s;
      right_lane_bound_point[0] = right_lane_bound_point[0] + current_s;
      left_bound_point[0] = left_bound_point[0] + current_s;
      right_bound_point[0] = right_bound_point[0] + current_s;

      std::vector<double> temp{current_s, 0, 0, single_refline[j][3], left_lane_bound_point[0], left_lane_bound_point[1], right_lane_bound_point[0], right_lane_bound_point[1], left_bound_point[0], left_bound_point[1], right_bound_point[0], right_bound_point[1], single_refline[j][13], single_refline[j][14], single_refline[j][15], single_refline[j][16], 0, 0};
      input_refline.emplace_back(temp);
    }
    std::vector<std::vector<double>> agents_input;
    auto single_agents = agents_window_[i];
    for (auto &agent: single_agents) {
      if (std::any_of(agent.begin(), agent.end(), [](size_t l){return l > 0;})) {
        std::vector<double> agentpoint{agent[0], agent[1]};
        int nearest_index = FindNearestReflinePoint(nearest_100m_refpoint, agentpoint);
        auto refline_points_nearest = nearest_100m_refpoint[nearest_index];

        auto agent_input_loc = ConvertToRelativeCoordinate6input(agent[0], agent[1], agent[2], refline_points_nearest.x(), refline_points_nearest.y(), refline_points_nearest.heading());
        auto agent_input_speed = ConvertToRelativeCoordinate6input(agent[3], agent[4], agent[2], 0, 0, refline_points_nearest.heading());
        agent_input_loc[0] = agent_input_loc[0] + refline_points_nearest.s() - refline_points_nearest_ego.s() + accumulate_s;
        double dis = sqrt(pow(agent_input_loc[0] - ego_input_loc[0], 2) + pow(agent_input_loc[1] - ego_input_loc[1], 2));
        std::vector<double> agent_input {agent_input_loc[0], agent_input_loc[1], agent_input_loc[2], agent_input_speed[0], agent_input_speed[1], agent[5], agent[6], agent[7], dis, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        agents_input.emplace_back(agent_input);
      } else {
        std::vector<double> agent_input {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        agents_input.emplace_back(agent_input);
      }
    }
    int len_agents_input = int(agents_input.size());
    if (len_agents_input <= 64) {
      for (int j = 0; j < (64 - len_agents_input); j++) {
        double temp_s = 100 + accumulate_s;
        double dis = sqrt(pow(temp_s - ego_input_loc[0], 2) + pow(5 - ego_input_loc[1], 2));
        std::vector<double> agent_input{temp_s, 5, ego_input_loc[2], ego_input_speed[0], ego_input_speed[1], single_ego[8], single_ego[9], 1, dis, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        agents_input.emplace_back(agent_input);
      }
    } else {
      agents_input.assign(agents_input.begin(), agents_input.begin() + 64); //tag
    }
    std::vector<std::vector<double>> single_inputdata_temp(input_ego);
    single_inputdata_temp.insert(single_inputdata_temp.end(),
                                 agents_input.begin(),
                                 agents_input.end());
    single_inputdata_temp.insert(single_inputdata_temp.end(),
                                 input_refline.begin(), input_refline.end());
    target_before_.emplace_back(single_inputdata_temp);
  }
  return target_before_;
}

std::vector<std::vector<std::vector<double>>> FeaturePreparationV2::GetObs(const int &frame_index, const EgoInfo &ego) {
  bool is_use_safety = false;
  std::vector<int> index_list;
  int horizon = 5;
  int sample_num = 100;
  if (frame_index >= horizon -1) {
    for (int i = frame_index - horizon + 1; i < frame_index + 1; i++) {
      index_list.emplace_back(i);
    }
  } else {
    for (int i = 0; i < horizon -1 -frame_index; i++) {
      index_list.emplace_back(0);
    }
    for (int i = 0; i < frame_index + 1; i++) {
      index_list.emplace_back(i);
    }
  }
  std::vector<ReferencePoint> refline_point100, nearest_100m_refpoint;
  for (auto &frame_i : index_list) {
    // 待确认是否为ego_list
    auto frame_ego = ego_lists_[frame_i];
    auto frame_agents = agents_lists_[frame_i];
    auto reference_line_points = refline_points_;
    std::vector<double> adc_point{frame_ego.centroid[0], frame_ego.centroid[1]};
    
    std::tie(refline_point100, nearest_100m_refpoint) = FindNearest110MeterSample(reference_line_points, adc_point, sample_num);

    // input 1、选 64 个最关注的的agent 表征
    if (is_use_safety) {
      // agents_input_ = GetAgentFeature(ego, ego_data_, agents_data_, refline_point100_meter);
    } else {
      GetAgentFeatureOriginal(frame_agents);
    }
    agents_window_.emplace_back(agents_input_);

    // input 2、ego 表征
    auto ego_input = GetEgoFeature(frame_ego);
    ego_window_.emplace_back(ego_input);

    // input 3、refline 100 个点表征
    auto refline_input = GetReflineFeatureStrong(refline_point100);
    refline_window_.emplace_back(refline_input);

    // input 4 traffic lights表征
    // auto traffic_light = frame_ego.traffic_light();
    traffic_light_window_.emplace_back(traffic_lights_);
  }

  EgoInfo center_point = EgoInfo(ego_lists_.back());
  ego_info_ = center_point;

  std::vector<std::vector<std::vector<double>>> vision_before;
  if (is_use_safety) {
    vision_before = GetObsBeforeSafety(center_point);
  } else {
    vision_before = GetObsBeforeOriginalFrenetStrong(nearest_100m_refpoint);
  }
  return vision_before;
}

std::vector<std::vector<std::vector<double>>> FeaturePreparationV2::GetObsUrbanDriver(const int &frame_index) {
  bool is_use_safety = false;
  std::vector<int> index_list;
  int horizon = 5;
  int sample_num = 100;
  if (frame_index >= horizon -1) {
    for (int i = frame_index - horizon + 1; i < frame_index + 1; i++) {
      index_list.emplace_back(i);
    }
  } else {
    for (int i = 0; i < horizon -1 -frame_index; i++) {
      index_list.emplace_back(0);
    }
    for (int i = 0; i < frame_index + 1; i++) {
      index_list.emplace_back(i);
    }
  }
  LOG_DEBUG("index_list finish");
  std::vector<ReferencePoint> refline_point100, nearest_100m_refpoint;
  ego_window_.clear();
  agents_window_.clear();
  refline_window_.clear();
  traffic_light_window_.clear();
  agents_window_dict_.clear();
  for (auto &frame_i : index_list) {
    // 待确认是否为ego_list
    auto frame_ego = ego_lists_[frame_i];
    auto frame_agents = agents_lists_[frame_i];
    auto reference_line_points = refline_points_;
    std::vector<double> adc_point{frame_ego.centroid[0], frame_ego.centroid[1]};
    
    std::tie(refline_point100, nearest_100m_refpoint) = FindNearest110MeterSample(reference_line_points, adc_point, sample_num);
    LOG_DEBUG("FindNearest110MeterSample");

    // input 1、选 64 个最关注的的agent 表征
    if (is_use_safety) {
      // agents_input_ = GetAgentFeature(ego, ego_data_, agents_data_, refline_point100_meter);
      agents_window_.emplace_back(agents_input_);
    } else {
      agents_input_ = GetAgentFeatureOriginal(frame_agents);
      std::map<size_t, std::vector<double>> agents_dict;
      int agents_input_len = agents_input_.size();
      for (int agent_index = 0; agent_index < agents_input_len; agent_index++) {
        auto frame_agent = frame_agents[agent_index];
        auto agent_input = agents_input_[agent_index];
        auto agent_id = frame_agent.track_id;
        // std::cout << "track_id:" << agent_id << std::endl;
        agents_dict[agent_id] = agent_input;
      }
      agents_window_dict_.emplace_back(agents_dict);
    }
    LOG_DEBUG("agent feature");

    // input 2、ego 表征
    auto ego_input = GetEgoFeature(frame_ego);
    ego_window_.emplace_back(ego_input);
    LOG_DEBUG("ego feature");

    // input 3、refline 100 个点表征
    auto refline_input = GetReflineFeatureStrong(refline_point100);
    refline_window_.emplace_back(refline_input);
    LOG_DEBUG("refline feature");

    // input 4 traffic lights表征
    // auto traffic_light = frame_ego.traffic_light();
    // auto traffic_light = TraLight("GREEN", "unknown", "000000000"); //temp
    // std::vector<TraLight> tls{traffic_light};

    traffic_light_window_.emplace_back(traffic_lights_);
    LOG_DEBUG("tl feature");
  }

  agents_window_.clear();
  agents_window_availability_.clear();
  std::tie(agents_window_, agents_window_availability_) = AlignAgents(agents_window_dict_);
  LOG_DEBUG("AlignAgents");

  EgoInfo center_point = EgoInfo(ego_lists_.back());
  ego_info_ = center_point;

  std::vector<std::vector<std::vector<double>>> vision_before;
  if (is_use_safety) {
    vision_before = GetObsBeforeSafety(center_point);
  } else {
    vision_before = GetObsBeforeOriginalFrenetStrong(nearest_100m_refpoint);
  }
  LOG_DEBUG("GetObsBeforeOriginalFrenetStrong");
  return vision_before;
}

void FeaturePreparationV2::GetAllData() {
  // GetData();
  LOG_DEBUG("GET ALL DATA");
  int frame_index = 0;
  // time_log_.ResetStartTime();
  for (frame_index = 0; frame_index < 5; frame_index++) {
    GetObsUrbanDriver(frame_index);
    // time_log_.RegisterTimeAndPrint("GetObsUrbanDriver");
    LOG_DEBUG("obs urban driver feature done");
  }
}

}  // namespace planning_rl
}  // namespace neodrive