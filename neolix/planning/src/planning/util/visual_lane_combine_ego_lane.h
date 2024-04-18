#pragma once

#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/path/sl_point.h"
#include "src/planning/proxy/proxy_type.h"
#include "src/planning/task/task_info.h"

namespace neodrive {
namespace planning {

class VisualLaneCombineEgoLane {
 public:
  VisualLaneCombineEgoLane();

  bool GetVisCurbBound(SLPoint& ego_sl, ReferenceLinePtr reference_line,
                       std::array<double, 2>& side_lat);
  bool Combine(TaskInfo& task_info);
  void SetCurbLines(TaskInfo& task_info);

 private:
  DataCenter* data_center_{nullptr};

  enum EgoRoadLaneType {
    LEFT_ROAD = 1,
    LEFT_LANE,
    RIGHT_LANE,
    RIGHT_ROAD,
    NONE
  };
  std::vector<CameraLaneLineType> need_match_lanes_type_{};
  std::vector<CameraLaneLinePositionType> need_match_lanes_pos_type_{};
  std::vector<std::vector<SLPoint>> need_match_lanes_sl_{};
  std::vector<double> ref_s_vec_{};
  std::vector<double> ref_left_lane_vec_{};
  std::vector<double> ref_left_road_vec_{};
  std::vector<double> ref_right_lane_vec_{};
  std::vector<double> ref_right_road_vec_{};
  std::vector<std::pair<EgoRoadLaneType, std::size_t>>
      matched_type_index_vec_{};
};

}  // namespace planning
}  // namespace neodrive
