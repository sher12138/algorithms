#pragma once

#include "src/planning/parking/geoparking_in/GeoParkingInit.h"

namespace neodrive {
namespace planning {

class PPP_ParkingOut : public GeoParkingInit {
 public:
  PPP_ParkingOut();
  ~PPP_ParkingOut();

  // head_tail_mode=0, no defined;1:tail in;2:head in;
  bool Generate(TrajectoryPoint end_pos,
                std::vector<TrajectoryPoint>& candidate_pos,
                int head_tail_mode = 0);

 private:
  // check the Left_Right_flag, both input are global coordinate
  bool FirstCarportCheck(const TrajectoryPoint& vehicle_pos,
                         const TrajectoryPoint& carspot_pos);

  bool PerpendicularParkingNormalOut(
      TrajectoryPoint end_pos, std::vector<TrajectoryPoint>& candidate_pos);
  bool PerpendicularParkingTailOut(TrajectoryPoint end_pos,
                                   std::vector<TrajectoryPoint>& candidate_pos);
  /*Head out*/
  // go forward
  void NormalHeadOutStep1(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment);
  // turn&out
  void NormalHeadOutStep2(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher,
                          double final_x_val);
  bool Satisfystep2Check();
  void ShiftYDirection(
      TrajectoryPoint vel_coor, double left_radius,
      double right_radius,  // vel_coor:current pos; tmp_radius: turn radius
      int left_right_flag,
      int forward_back_flag,  // left_right_flag:1:left;2:right;forward_back_flag:1:forward;2:backward;b_direction_flag:1:up;2:down;
      double target_y_val, double x_range,
      std::vector<TrajectoryPoint>& in_forward_log_data,  // log data and index
      int& in_start_segment);  // log and index; abnomous_watcher
  /*Tail out*/
  // go backward
  void NormalTailOutStep1(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);
  // turn&out
  void NormalTailOutStep2(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher,
                          double final_x_val);
  void ShiftYDirectionTailOut(
      TrajectoryPoint vel_coor, double left_radius,
      double right_radius,  // vel_coor:current pos; tmp_radius: turn radius
      int left_right_flag,
      int forward_back_flag,  // left_right_flag:1:left;2:right;forward_back_flag:1:forward;2:backward;b_direction_flag:1:up;2:down;
      double target_y_val, double x_range,
      std::vector<TrajectoryPoint>& in_forward_log_data,  // log data and index
      int& in_start_segment);  // log and index; abnomous_watcher
  bool SatisfyTailOutStep2Check();

  // int Left_Right_flag;  // 1:carport right of vehicle;
  // 2:carport left of vehicle
};

}  // namespace planning
}  // namespace neodrive
