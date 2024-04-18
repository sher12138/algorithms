#pragma once

#include "GeoParkingInit.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/parking/geoparking_out/PerpendicularParking_out.h"

namespace neodrive {
namespace planning {

class PPP_ParkingIn : public GeoParkingInit {
 public:
  PPP_ParkingIn();
  ~PPP_ParkingIn();

  // head_tail_mode=0, no defined;1:tail in;2:head in;
  bool Generate(TrajectoryPoint start_pos,
                std::vector<TrajectoryPoint>& candidate_pos,
                int head_tail_mode = 0);

 private:
  bool FirstCarportCheck(TrajectoryPoint& vehicle_pos,
                         TrajectoryPoint& carspot_pos);

  bool PerpendicularParkingNormalTailIn(
      TrajectoryPoint start_pos, std::vector<TrajectoryPoint>& candidate_pos);

  bool PerpendicularParkingClassicTailIn(
      TrajectoryPoint start_pos, std::vector<TrajectoryPoint>& candidate_pos);

  bool PerpendicularParkingNormalHeadIn(
      TrajectoryPoint start_pos, std::vector<TrajectoryPoint>& candidate_pos);

  bool PerpendicularParkingClassicHeadIn(
      TrajectoryPoint start_pos, std::vector<TrajectoryPoint>& candidate_pos);

  bool PerpendicularParkingShiftTailIn(
      TrajectoryPoint start_pos, std::vector<TrajectoryPoint>& candidate_pos);

  bool PerpendicularParkingShiftHeadIn(
      TrajectoryPoint start_pos, std::vector<TrajectoryPoint>& candidate_pos);

  /*For classic tail in*/
  // perpendicular to the carport
  void ClassicTailInStep1(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);

  // left_right_flag:1:left;2:right;forward_back_flag:0:forward;1:backward;
  void Step1ParalleltoYAxis(TrajectoryPoint vel_coor, double left_radius,
                            double right_radius, int left_right_flag,
                            int forward_back_flag,
                            std::vector<TrajectoryPoint>& in_forward_log_data,
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

  // left_right_flag:1:left;2:right;forward_back_flag:1:forward;2:backward;b_direction_flag:1:up;2:down;
  void ShiftYDirection(TrajectoryPoint vel_coor, double left_radius,
                       double right_radius, int left_right_flag,
                       int forward_back_flag, double target_y_val,
                       double x_range,
                       std::vector<TrajectoryPoint>& in_forward_log_data,
                       int& in_start_segment, int& abnomous_watcher);
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

  // left_right_flag:1:left;2:right;forward_back_flag:1:forward;2:backward;b_direction_flag:1:up;2:down;
  void ShiftYDirectionHeadIn(TrajectoryPoint vel_coor, double left_radius,
                             double right_radius, int turn_leri_flag,
                             int left_right_vehicle, int forward_back_flag,
                             double target_y_val, double x_range,
                             std::vector<TrajectoryPoint>& in_forward_log_data,
                             int& in_start_segment, int& abnomous_watcher);

  bool SatisfyHeadinStep3Check();

  void MoveXDirectionHeadIn(TrajectoryPoint start_vel_pos,
                            TrajectoryPoint end_vel_pos,
                            std::vector<TrajectoryPoint>& forward_log_data,
                            int& start_segment, int head_direction);
  /*For Shift Tail in*/
  void ShiftTailInStep1(TrajectoryPoint& vel_pos,
                        std::vector<TrajectoryPoint>& forward_log_data,
                        int& start_segment, int& abnomous_watcher);

  // left_right_flag:1:left;2:right;forward_back_flag:0:forward;1:backward;
  void ShiftTailStep1Parallel(TrajectoryPoint vel_coor, double left_radius,
                              double right_radius, int left_right_flag,
                              int forward_back_flag,
                              std::vector<TrajectoryPoint>& in_forward_log_data,
                              int& in_start_segment, int& abnomous_watcher);

  // parameters
  // int Left_Right_flag;     // 1:left;2:right;
 private:
  PPP_ParkingOut ppp_parking_out;
};

}  // namespace planning
}  // namespace neodrive
