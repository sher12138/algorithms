#pragma once

#include "src/planning/common/planning_logger.h"
#include "src/planning/parking/geoparking_in/GeoParkingInit.h"

namespace neodrive {
namespace planning {

class PP_ParkingOut : public GeoParkingInit {
 public:
  PP_ParkingOut();
  ~PP_ParkingOut();

  // main function for parallel parking
  bool Generate(TrajectoryPoint start_pos,
                std::vector<TrajectoryPoint>& candidate_pos,
                int head_tail_mode = 0);

 private:
  // check the Left_Right_flag, both input are global coordinate
  bool FirstCarportCheck(TrajectoryPoint& vehicle_pos,
                         TrajectoryPoint& carspot_pos);
  /*Parallel Parking functions*/
  bool ParallelParkingNormalOut(TrajectoryPoint end_pos,
                                std::vector<TrajectoryPoint>& candidate_pos);
  // go backward
  void TailOutStep1(TrajectoryPoint& curr_vel_pos,
                    std::vector<TrajectoryPoint>& forward_log_data,
                    int& start_segment);
  // turn&adjust inside of current carport
  void TailOutStep2(TrajectoryPoint& curr_vel_pos,
                    std::vector<TrajectoryPoint>& forward_log_data,
                    int& start_segment, int& abnomous_watcher);
  // go forward/backward
  void TailOutStep3(TrajectoryPoint& curr_vel_pos,
                    std::vector<TrajectoryPoint>& forward_log_data,
                    int& start_segment, int& abnomous_watcher,
                    double final_y_val);
  // turn parallel
  void TailOutStep4(TrajectoryPoint& curr_vel_pos,
                    std::vector<TrajectoryPoint>& forward_log_data,
                    int& start_segment, int& abnomous_watcher,
                    double final_y_val);

  bool Satisfystep2Check();
  bool Satisfystep4Check();

  // check whether vhicle inside of current carport
  bool CurrCarportBoundSafety(int forward_back_flag);
};

}  // namespace planning
}  // namespace neodrive
