#pragma once

#include "src/planning/reference_line/reference_line.h"
//#include "src/planning/common/obstacle/obstacle.h"
#include "src/planning/common/trajectory/trajectory_point.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

// TODO(wyc): no need to abstract as a class with getter and setter
class ParkingDataInfo {
 public:
  ParkingDataInfo();  // TODO(bryan): default?
  ~ParkingDataInfo();

  void SetParkingRefLine(const ReferenceLinePtr parking_ref_line);
  void SetSpotVertices(std::vector<Vec2d>& spot_vertices);
  void SetParkSpotType(const int& park_spot_type);
  void SetLeftRightType(const int& left_right_type);
  void SetHeadTailIn(const int& head_tail_in);
  void SetFinalPosFlag(const bool& set_final_pos_flag);
  void SetPreDefinedFinalPos(const TrajectoryPoint& pre_defined_final_pos);

  void SetAnchorPointCoordinate(const ReferencePoint& anchor_point);
  void SetCurrPosOnAnchor(const TrajectoryPoint& curr_pos);
  void SetFinalPosOnAnchor(const TrajectoryPoint& final_pos);

  void SetCurrGps(const Vec2d& curr_gps);
  void SetCurrHeading(const double& curr_heading);

  void SetOriginPoint(const Vec2d& origin_point);
  void SetOriginHeading(const double& origin_heading);
  void SetParkingEndPose(const std::vector<double>& parking_end_pose);

  void SetConsiderObstcleFlag(const bool& consider_obstcle_flag);
  //  void set_obs_list(const std::vector<Obstacle>& obs_list);

  void SetROIXyBoundary(const std::vector<double>& ROI_xy_boundary);
  void SetObstaclesNum(const std::size_t& obstacles_num);
  void SetObstaclesEdgesNum(const Eigen::MatrixXi& obstacles_edges_num);
  void SetObstaclesVerticesVec(
      const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec);
  void SetObstaclesA(const Eigen::MatrixXd& obstacles_A);
  void SetObstaclesB(const Eigen::MatrixXd& obstacles_b);

  void SetLimitCarportCoorGlobal(std::vector<Vec2d>& limit_carport_coor_global);
  void SetLimitRoadOppsiteGlobal(std::vector<Vec2d>& limit_road_oppsite_global);
  void SetLimitRoadRearGlobal(std::vector<Vec2d>& limit_road_rear_global);
  void SetLimitRoadFrontGlobal(std::vector<Vec2d>& limit_road_front_global);

  void SetLimitCarportCoorAnchor(std::vector<Vec2d>& limit_carport_coor_anchor);
  void SetLimitRoadOppsiteAnchor(std::vector<Vec2d>& limit_road_oppsite_anchor);
  void SetLimitRoadRearAnchor(std::vector<Vec2d>& limit_road_rear_anchor);
  void SetLimitRoadFrontAnchor(std::vector<Vec2d>& limit_road_front_anchor);

  //******************************
  ReferenceLinePtr ParkingRefLine() const;
  std::vector<Vec2d> SpotVertices() const;
  int ParkSpotType() const;
  int LeftRightType() const;
  int HeadTailIn() const;
  bool FinalPosFlag() const;
  TrajectoryPoint FinalPosOnAnchor() const;

  ReferencePoint AnchorPointCoordinate() const;
  TrajectoryPoint CurrPosOnAnchor() const;
  TrajectoryPoint PreDefinedFinalPos() const;

  Vec2d CurrGps() const;
  double CurrHeading() const;
  Vec2d OriginPoint() const;
  double OriginHeading() const;

  std::vector<double> ParkingEndPose() const;
  bool ConsiderObstcleFlag() const;
  //  std::vector<Obstacle> obs_list() const;

  std::vector<double> ROIXyBoundary() const;
  std::size_t ObstaclesNum() const;
  Eigen::MatrixXi ObstaclesEdgesNum() const;
  Eigen::MatrixXi* MutableObstaclesEdgesNum();
  std::vector<std::vector<Vec2d>> ObstaclesVerticesVec() const;
  std::vector<std::vector<Vec2d>>* MutableObstaclesVerticesVec();
  Eigen::MatrixXd ObstaclesA() const;
  Eigen::MatrixXd* MutableObstaclesA();
  Eigen::MatrixXd ObstaclesB() const;
  Eigen::MatrixXd* MutableObstaclesB();

  std::vector<Vec2d> LimitCarportCoorGlobal() const;
  std::vector<Vec2d> LimitRoadOppsiteGlobal() const;
  std::vector<Vec2d> LimitRoadRearGlobal() const;
  std::vector<Vec2d> LimitRoadFrontGlobal() const;

  std::vector<Vec2d>* MutableLimitCarportCoorGlobal();
  std::vector<Vec2d>* MutableLimitRoadOppsiteGlobal();
  std::vector<Vec2d>* MutableLimitRoadRearGlobal();
  std::vector<Vec2d>* MutableLimitRoadFrontGlobal();

  std::vector<Vec2d> LimitCarportCoorAnchor() const;
  std::vector<Vec2d> LimitRoadOppsiteAnchor() const;
  std::vector<Vec2d> LimitRoadRearAnchor() const;
  std::vector<Vec2d> LimitRoadFrontAnchor() const;

 private:
  // parking info
  ReferenceLinePtr parking_ref_line_;
  std::vector<Vec2d> spot_vertices_;
  // 0:undefined; 1: perpendicular; 2: parallel; 3: inclined
  int park_spot_type_ = 0;
  // carport is left/right of vehicle; 0: undefined; 1:left; 2:right;
  int left_right_type_ = 0;
  // 0: undefined, 1: tail in; 2: head in
  int head_tail_in_ = 0;
  // set_final_pos_: false calculate inner; true: from outside;
  bool set_final_pos_flag_ = false;
  TrajectoryPoint pre_defined_final_pos_;

  /*
   * this anchor point is used to transform all coordinates to here; (hybrid a*,
   * geo parking) After planning, transorm to UTM back
   * **********************************************
   *
   *    ---->
   *
   *  ****************A          ******************
   *                  *        *
   *                  *        *
   *                  **********
   *  * **********************************************
   *
   *                               <-------
   *
   *  ****************A          ******************
   *                  *        *
   *                  *        *
   *                  **********
   */
  ReferencePoint anchor_point_;

  // the following 3 are based on anchor point
  TrajectoryPoint curr_pos_;
  TrajectoryPoint final_pos_;
  TrajectoryPoint left_top_pos_;
  // curr_gps based on global UTM
  Vec2d curr_gps_;
  double curr_heading_ = 0.0;

  // origin point based on anchor, left_top
  // to scale all coordinate in hybrid a*
  Vec2d origin_point_;
  double origin_heading_ = 0.0;
  // end pose based on anchor, [x, y, heading, speed]
  // v = 0; donot use it.
  std::vector<double> parking_end_pose_;

  // do we use obs in hybrid a*
  bool consider_obstcle_flag_ = false;
  //  std::vector<Obstacle> obs_list_;

  // [x_min, x_max, y_min, y_max];
  std::vector<double> ROI_xy_boundary_;

  // obstacles total num including perception obstacles and parking space
  // boundary
  std::size_t obstacles_num_ = 0;
  // the dimension needed for A and b matrix dimension in H representation
  Eigen::MatrixXi obstacles_edges_num_;

  // vector storing the vertices of obstacles in counter-clock-wise order
  std::vector<std::vector<Vec2d>> obstacles_vertices_vec_;
  // Linear inequality representation of the obstacles Ax>b
  Eigen::MatrixXd obstacles_A_;
  Eigen::MatrixXd obstacles_b_;

  /*****************************************************************
   *                    ---limit_carport-----
   * ----road_rear------                      ---road_front-------
   *           **>
   * ---------road_oppsite----------------------------------
   *
   *
   * ---------road_oppsite----------------------------------
   *          **>
   * ----road_rear------                      ---road_front-------
   *                     ---limit_carport-----
   *
   ***********************************************************************/

  // carport limitaions represented in global and anchor coordinates
  std::vector<Vec2d> limit_carport_coor_global_;  // curr carport limitation
  std::vector<Vec2d> limit_road_oppsite_global_;
  std::vector<Vec2d> limit_road_rear_global_;   // rear carport
  std::vector<Vec2d> limit_road_front_global_;  // forward carport

  std::vector<Vec2d> limit_carport_coor_anchor_;  // curr carport limitation
  std::vector<Vec2d> limit_road_oppsite_anchor_;
  std::vector<Vec2d> limit_road_rear_anchor_;   // rear carport
  std::vector<Vec2d> limit_road_front_anchor_;  // forward carport
};

}  // namespace planning
}  // namespace neodrive
