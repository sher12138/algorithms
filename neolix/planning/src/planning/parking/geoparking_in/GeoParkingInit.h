#pragma once

#include <vector>

#include "CreateVehicleShape.h"
#include "PolygonRelation.h"
#include "src/planning/common/trajectory/trajectory_point.h"
#include "common/math/vec2d.h"
#include "common/math/util.h"
#include "src/planning/parking/parking_config.h"

namespace neodrive {
namespace planning {

class GeoParkingInit {
 public:
  GeoParkingInit();

  ~GeoParkingInit();

  /*****************************************************************
   *                    ---limit_carport-----
   * ----road_rear------                      ---road_front-------
   *           **>
   * ---------road_oppsite----------------------------------
   *
   ***********************************************************************/
  /* all the coordinate are in global coordinate
   * If they are all related to one local coordinate, also acceptable.
   */
  bool Initialize(const TrajectoryPoint& vehicle_pos,
                  const TrajectoryPoint& carspot_pos,
                  const std::vector<Vec2d>& limit_carport_coor_global,
                  const std::vector<Vec2d>& limit_road_oppsite_global,
                  const std::vector<Vec2d>& limit_road_rear_global,
                  const std::vector<Vec2d>& limit_road_front_global);
  /*start_pos is in global coordinate,
   *candidate_pos: generated trajectory
   * head_tail_mode: 0 : not defined;1:tail in; 2:head in;
   * parallel parking only accepate tail in;
   */
  bool Generate(TrajectoryPoint& start_pos,
                std::vector<TrajectoryPoint>& candidate_pos,
                int head_tail_mode = 0);

  void SetPARK_PRECISION(const double precision);

 protected:
  double GetLimitLeftCurv() const;
  double GetLimitRightCurv() const;

  void RegulateYaw(double& yaw_angle);        //[-pi/2,pi/2]
  void RegulateYawPi0(double& yaw_angle);     //[-pi,0]
  void RegulateYaw0Pi(double& yaw_angle);     //[0,pi]
  void RegulateYawPiPi(double& yaw_angle);    //[-pi,pi]
  void RegulateYawHpiHpi(double& yaw_angle);  //[-1.5pi,-0.5pi],for head in

  double GetMaxfromCarport(const char flag, const std::vector<Vec2d>& carport);
  double GetMinfromCarport(const char flag, const std::vector<Vec2d>& carport);

  bool CarportCurrBoundSafety();

  void MoveXDirection(const TrajectoryPoint& start_vel_pos,
                      const TrajectoryPoint& end_vel_pos,
                      std::vector<TrajectoryPoint>& forward_log_data,
                      int& start_segment, const int head_direction);

  void MoveYDirection(const TrajectoryPoint& start_vel_pos,
                      const TrajectoryPoint& end_vel_pos,
                      std::vector<TrajectoryPoint>& forward_log_data,
                      int& start_segment, const int head_direction);

  // vel_coor:current pos; tmp_radius: turn radius
  // leri_flag:1:left;2:right;carport_direc:1:left;2:right to the carport
  // log data and segment, abnomous_watcher
  void ShiftXDirectionOnRoad(TrajectoryPoint& vel_coor,
                             const double left_radius,
                             const double right_radius, const int leri_flag,
                             const int carport_direc, const double target_x_val,
                             std::vector<TrajectoryPoint>& in_forward_log_data,
                             int& in_start_segment, int& abnomous_watcher);

  // vel_coor:current pos; tmp_radius: turn radius
  // left_right_flag:1:left;2:right;forward_back_flag:1:forward;2:backward;b_direction_flag:1:up;2:down;
  // log data and index
  // log keypoint and index; abnomous_watcher
  void ShiftYDirectionOnRoad(TrajectoryPoint& vel_coor,
                             const double left_radius,
                             const double right_radius,
                             const int left_right_flag,
                             const int forward_back_flag,
                             const double target_y_val,
                             std::vector<TrajectoryPoint>& in_forward_log_data,
                             int& in_start_segment, int& abnomous_watcher);

  void SaveDatatoBehavior(std::vector<TrajectoryPoint>& candidate_pos);

  bool LogParkingData();

  void ClearParkInit();

 public:
  double PARK_PRECISION = 0.1;
  double protected_half_width = 1.0;
  // carport is left/right of vehicle; 1:left; 2:right;
  int Left_Right_flag = 0;
  double left_radius = 5.0;
  double right_radius = 5.0;
  ParkingCarportLimit config_limit;

  Polygon_RELATION Available_area_;
  CandiatePoint vehicle_pos_;
  TrajectoryPoint final_carport_pos_;

  std::vector<TrajectoryPoint> pp_planning_trajectory;
  std::vector<TrajectoryPoint> pp_planning_key_trajectory;
  std::vector<TrajectoryPoint> pp_old_trajectory;
  std::vector<TrajectoryPoint> pp_old_earth_trajectory;
  // define carport coordinate
  std::vector<Vec2d> limit_carport_coor_;  // curr carport limitation
  std::vector<Vec2d> limit_road_oppsite_coor_;
  std::vector<Vec2d> limit_road_rear_coor_;   // rear carport
  std::vector<Vec2d> limit_road_front_coor_;  // forward carport

  CreateVehicleShape vehicle_shape_creater_;

 private:
  // according to carport pos and road&carport, build avaliable area
  bool SetCarportCoordinates();

  bool BuildRoadCarportLimit();

 private:
  double Limit_left_curv = 0.23;
  double Limit_right_curv = 0.23;
  double normal_left_curv = 0.2;
  double normal_right_curv = 0.2;
  std::vector<Vec2d> limit_carport_coor_global_;  // curr carport limitation
  std::vector<Vec2d> limit_road_oppsite_global_;
  std::vector<Vec2d> limit_road_rear_global_;   // rear carport
  std::vector<Vec2d> limit_road_front_global_;  // forward carport
};

}  // namespace planning
}  // namespace neodrive
