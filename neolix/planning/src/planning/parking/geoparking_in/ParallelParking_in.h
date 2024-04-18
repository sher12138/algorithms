#pragma once

#include "GeoParkingInit.h"
#include "src/planning/parking/geoparking_out/ParallelParking_out.h"

namespace neodrive {
namespace planning {

class PP_ParkingIn : public GeoParkingInit {
 public:
  PP_ParkingIn();
  ~PP_ParkingIn();

  bool Generate(TrajectoryPoint start_pos,
                std::vector<TrajectoryPoint>& candidate_pos,
                int head_tail_mode = 0);

 private:
  /*Parallel Parking functions*/
  bool FirstCarportCheck(TrajectoryPoint& vehicle_pos,
                         TrajectoryPoint& carspot_pos);

  bool ParallelParkingNormalTailIn(TrajectoryPoint start_pos,
                                   std::vector<TrajectoryPoint>& candidate_pos);
  bool ParallelParkingClassicTailIn(
      TrajectoryPoint start_pos, std::vector<TrajectoryPoint>& candidate_pos);
  bool ParallelParkingShiftIn(TrajectoryPoint start_pos,
                              std::vector<TrajectoryPoint>& candidate_pos);
  /*For normal tail in*/
  // parallel to the carport
  // TODO(bryan): change into tail_in_step_one()?
  void TailInStep1(TrajectoryPoint& vel_pos,
                   std::vector<TrajectoryPoint>& forward_log_data,
                   int& start_segment, int& abnomous_watcher);
  // left_right_flag:1:left;2:right;forward_back_flag:0:forward;1:backward;
  void ParallelParkingStep1RoadCarportCheck(
      TrajectoryPoint vel_coor, double left_radius, double right_radius,
      int left_right_flag, int forward_back_flag,
      std::vector<TrajectoryPoint>& in_forward_log_data, int& in_start_segment,
      int& abnomous_watcher);
  /*For classical tail in*/
  // adjust to constant position
  void ClassicTailInStep2(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);
  // turn&back to a certain angle
  void ClassicTailInStep3(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);
  // straight backward
  void ClassicTailInStep4(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);
  // turn&back to parallel
  void ClassicTailInStep5(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);
  // adjust y to 0
  void ClassicTailInStep6(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);
  // adjust x to 0
  void ClassicTailInStep7(TrajectoryPoint& vel_pos,
                          std::vector<TrajectoryPoint>& forward_log_data,
                          int& start_segment, int& abnomous_watcher);

  void CalculateCalssicStep3Angle(TrajectoryPoint& vel_pos, double& angle);

  void ShiftInStep1(TrajectoryPoint& vel_pos,
                    std::vector<TrajectoryPoint>& forward_log_data,
                    int& start_segment, int& abnomous_watcher);

  // check whether vhicle inside of current carport
  bool CurrCarportBoundSafety(int forward_back_flag);

  // left_right_flag:1:left;2:right;forward_back_flag:1:forward;2:backward;b_direction_flag:1:up;2:down;
  void ShiftYDirection(TrajectoryPoint vel_coor, double left_radius,
                       double right_radius, int left_right_flag,
                       int forward_back_flag, double target_y_val,
                       double x_range,
                       std::vector<TrajectoryPoint>& in_forward_log_data,
                       int& in_start_segment, int& abnomous_watcher);

 private:
  PP_ParkingOut pp_parking_out;
};

}  // namespace planning
}  // namespace neodrive
