#pragma once
#include "GeoParkingInit.h"
#include "src/planning/parking/geoparking_out/AngularParking_out.h"
namespace neodrive {
namespace planning {
class AP_ParkingIn : public GeoParkingInit {
 public:
  // TODO(bryan): base class and subclass destructor should
  // set into virtual
  AP_ParkingIn();
  ~AP_ParkingIn();
  /*functions*/

  // head_tail_mode=0, no defined;1:tail in;2:head in;

  // TODO(bryan): base class and subclass destructor should
  // set into virtual and override

  // TODO(wyc): const reference as input for user-defined type

  bool Generate(TrajectoryPoint start_pos,
                std::vector<TrajectoryPoint>& candidate_pos,
                int head_tail_mode = 0);

 private:
  bool FirstCarportCheck(TrajectoryPoint& vehicle_pos,
                         TrajectoryPoint& carspot_pos);
  bool AngularParkingNormalTailIn(TrajectoryPoint start_pos,
                                  std::vector<TrajectoryPoint>& candidate_pos);
  bool AngularParkingClassicTailIn(TrajectoryPoint start_pos,
                                   std::vector<TrajectoryPoint>& candidate_pos);
  bool AngularParkingNormalHeadIn(TrajectoryPoint start_pos,
                                  std::vector<TrajectoryPoint>& candidate_pos);
  bool AngularParkingClassicHeadIn(TrajectoryPoint start_pos,
                                   std::vector<TrajectoryPoint>& candidate_pos);
  /*For classic tail in*/
  // perpendicular to the carport
  void ClassicTailInStep1(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);
  void step1ParalleltoYAxis(
      TrajectoryPoint vel_coor, double left_radius,
      double right_radius,  // vel_coor:current pos; tmp_radius: turn radius
      int left_right_flag, int forward_back_flag,
      std::vector<TrajectoryPoint>&
          in_forward_log_data,  // left_right_flag:1:left;2:right;forward_back_flag:0:forward;1:backward;log
                                // data
      int& in_start_segment, int& abnomous_watcher);
  // adjust to constant position
  void ClassicTailInStep2(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);
  // turn&back to a certain angle
  void ClassicTailInStep3(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);
  // turn&shift to x-axis
  void ClassicTailInStep4(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);
  // straight backward
  void ClassicTailInStep5(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);
  void ShiftYDirection(
      TrajectoryPoint vel_coor, double left_radius,
      double right_radius,  // vel_coor:current pos; tmp_radius: turn radius
      int left_right_flag,
      int forward_back_flag,  // left_right_flag:1:left;2:right;forward_back_flag:1:forward;2:backward;b_direction_flag:1:up;2:down;
      double target_y_val, double x_range,
      std::vector<TrajectoryPoint>& in_forward_log_data,  // log data and index
      int& in_start_segment,
      int& abnomous_watcher);  // log and index; abnomous_watcher
  /*For classic head in*/
  // adjust to constant position
  void ClassicHeadInStep2(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);
  // turn&back to a certain angle
  void ClassicHeadInStep3(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);
  // turn&shift to x-axis
  void ClassicHeadInStep4(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);
  // go to final pos
  void ClassicHeadInStep5(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);
  void ShiftYDirectionHeadIn(
      TrajectoryPoint vel_coor, double left_radius,
      double right_radius,  // vel_coor:current pos; tmp_radius: turn radius
      int turn_leri_flag, int left_right_vehicle,
      int forward_back_flag,  // left_right_flag:1:left;2:right;forward_back_flag:1:forward;2:backward;b_direction_flag:1:up;2:down;
      double target_y_val, double x_range,
      std::vector<TrajectoryPoint>& in_forward_log_data,  // log data and index
      int& in_start_segment,
      int& abnomous_watcher);  // log and index; abnomous_watcher
  bool SatisfyHeadinStep3Check();
  void MoveXDirectionHeadIn(TrajectoryPoint start_vel_pos,
                            TrajectoryPoint end_vel_pos,
                            std::vector<TrajectoryPoint>& forward_log_data,
                            int& start_segment, int head_direction);

  /*common*/
  // build available area
  bool BuildRoadCarportLimit(Polygon_RELATION& roadcarport_limit);
  // bool    LogParkingData();
  //  int Left_Right_flag;     // 1:left;2:right;
  int bforward_back_flag;  // 1:forward;2:backward
  AP_ParkingOut ap_parking_out;
};
}  // namespace planning
}  // namespace neodrive
