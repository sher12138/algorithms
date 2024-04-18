#include "AngularParking_in.h"
namespace neodrive {
namespace planning {
// TODO(wyc): initializer list
AP_ParkingIn::AP_ParkingIn() {
  Left_Right_flag = 0;
  bforward_back_flag = 0;
  ClearParkInit();
}
// TODO(wyc): no need
AP_ParkingIn::~AP_ParkingIn() {
  Left_Right_flag = 0;
  bforward_back_flag = 0;
  ClearParkInit();
}

bool AP_ParkingIn::Generate(TrajectoryPoint start_pos,
                            std::vector<TrajectoryPoint>& candidate_pos,
                            int head_tail_mode) {
  // TODO(wyc):
  // define variables as later as possible
  bool bflag = false;
  int manu_num = 0;
  double time = 0.0;

  double final_x = start_pos.x();
  double xdistoRoad =
      GetMinfromCarport('x', limit_road_oppsite_coor_) - final_x;
  double xdistoCarport =
      final_x - GetMaxfromCarport('x', limit_road_front_coor_);
  if (fabs(xdistoRoad) + fabs(xdistoCarport) < 4.8) return false;
  if (fabs(limit_carport_coor_[0].y()) + fabs(limit_carport_coor_[1].y()) < 3.8)
    return false;
  // if (head_tail_mode == 2) {//head in
  //	bflag = AngularParkingNormalHeadIn(start_pos, candidate_pos);
  //	if (bflag == false)
  //		bflag = AngularParkingClassicHeadIn(start_pos,
  // candidate_pos);
  //}
  // else if (head_tail_mode == 1) {
  //	bflag = AngularParkingNormalTailIn(start_pos, candidate_pos);
  //	if (bflag == false)
  //		bflag = AngularParkingClassicTailIn(start_pos,
  // candidate_pos);
  //}
  // else{
  //	bflag = AngularParkingNormalTailIn(start_pos, candidate_pos);
  //	if (bflag == false)
  //		bflag = AngularParkingClassicTailIn(start_pos,
  // candidate_pos); 	if (bflag == false) 		bflag =
  // AngularParkingNormalHeadIn(start_pos, candidate_pos); 	if (bflag ==
  // false) 		bflag = AngularParkingClassicHeadIn(start_pos,
  // candidate_pos);
  //}
  // TODO(wyc):
  // return AngularParkingNormalTailIn();
  bflag = AngularParkingNormalTailIn(start_pos, candidate_pos);
  if (bflag == false) return bflag;
  return bflag;
}
bool AP_ParkingIn::FirstCarportCheck(TrajectoryPoint& vehicle_pos,
                                     TrajectoryPoint& carspot_pos) {
  // TODO(wyc):
  // one variable each line
  double tmp_x = 0.0, tmp_y = 0.0, tmp_theta = 0.0;
  // tranfer carport_pos to vehicle_pos
  earth2vehicle(vehicle_pos.x(), vehicle_pos.y(), vehicle_pos.theta(),
                carspot_pos.x(), carspot_pos.y(), carspot_pos.theta(), tmp_x,
                tmp_y, tmp_theta);
  if (tmp_y > 0)
    // TODO(wyc): varaible name
    Left_Right_flag = 1;
  else
    Left_Right_flag = 2;
  return true;
}

}  // namespace planning
}  // namespace neodrive
