#pragma once

#include "src/planning/common/path/path.h"
#include "reference_line/reference_line.h"
#include "common/math/box2d.h"
#include "common/math/vec2d.h"
#include "common/math/util.h"
#include "src/planning/parking/parking_config.h"
#include "src/planning/parking/parking_data_info.h"

namespace neodrive {
namespace planning {

class ParkingSpotDecider {
 public:
  ParkingSpotDecider() = default;
  ~ParkingSpotDecider() = default;

  bool Process(ReferenceLinePtr const refer_line,
               std::array<Vec2d, 4> &vertices);

  void SetParkingDataInfo(ParkingDataInfo *const parking_data_info,
                          ParkingConfig *const parking_config);

  ParkingDataInfo *GetParkingDataInfo();

 private:
  bool SetParkingReferenceLine(ReferenceLinePtr const refer_line);

  bool CalculateAnchorPoint(std::array<Vec2d, 4> &vertices,
                            std::array<Vec2d, 4> &new_vertices);

  bool SetParkingSpotType(const std::array<Vec2d, 4> &vertices);

  bool ParkingSpotValid(const std::array<Vec2d, 4> &vertices);  // true: valid

  bool SetParkingSpotEndPose(const std::array<Vec2d, 4> &vertices);

  // false: tail-in; true:head in
  bool SetPerpenSpotEndPose(const std::array<Vec2d, 4> &vertices,
                            const bool &head_tail_in);

  bool SetParallelSpotEndPose(const std::array<Vec2d, 4> &vertices);

  // false: tail-in; true:head in
  bool SetInclinedSpotEndPose(const std::array<Vec2d, 4> &vertices,
                              const bool &head_tail_in);

  bool CalInclinedSpotEndPose();

  bool GetParkingBoundary(
      const std::array<Vec2d, 4> &vertices, const ReferenceLinePtr refer_line,
      std::vector<std::vector<Vec2d>> *const roi_parking_boundary);

  void GetRoadBoundary(const ReferenceLinePtr refer_line,
                       const double center_line_s, const Vec2d &origin_point,
                       const double origin_heading,
                       std::vector<Vec2d> *left_lane_boundary,
                       std::vector<Vec2d> *right_lane_boundary,
                       std::vector<Vec2d> *center_lane_boundary_left,
                       std::vector<Vec2d> *center_lane_boundary_right,
                       std::vector<double> *center_lane_s_left,
                       std::vector<double> *center_lane_s_right,
                       std::vector<double> *left_lane_road_width,
                       std::vector<double> *right_lane_road_width);

  void AddBoundaryKeyPoint(const ReferenceLinePtr refer_line,
                           const double check_point_s, const double start_s,
                           const double end_s, const bool is_anchor_point,
                           const bool is_left_curb,
                           std::vector<Vec2d> *center_lane_boundary,
                           std::vector<Vec2d> *curb_lane_boundary,
                           std::vector<double> *center_lane_s,
                           std::vector<double> *road_width);

  bool FuseLineSegments(std::vector<std::vector<Vec2d>> *line_segments_vec);

  bool FormulateBoundaryConstraints(
      const std::vector<std::vector<Vec2d>> &roi_parking_boundary);

  bool LoadObstacleInVertices(
      const std::vector<std::vector<Vec2d>> &roi_parking_boundary);

  bool LoadObstacleInHyperPlanes();

  bool GetHyperPlanes(
      const size_t &obstacles_num, const Eigen::MatrixXi &obstacles_edges_num,
      const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
      Eigen::MatrixXd *A_all, Eigen::MatrixXd *b_all);

  // reorganize refer lines bound
  bool ReferLineReorganize(ReferenceLinePtr const refer_line,
                           std::array<Vec2d, 4> &vertices);

  bool ReferLineBoundAdapt(ReferenceLinePtr const refer_line,
                           std::vector<Vec2d> &vertice);

  bool GetCarportLimitations(const ReferenceLinePtr refer_line,
                             std::array<Vec2d, 4> &vertices,
                             std::vector<Vec2d> &limit_carport_coor_global,
                             std::vector<Vec2d> &limit_road_oppsite_global,
                             std::vector<Vec2d> &limit_road_rear_global,
                             std::vector<Vec2d> &limit_road_front_global);

  bool CalcParkingRoadboundKeyPts(
      const ReferenceLinePtr reference_line, const int &left_right_flag,
      const std::vector<Vec2d> &not_park_side_road_bound,
      const std::vector<Vec2d> &park_side_road_bound,
      const std::vector<Vec2d> &parking_space,
      std::vector<Vec2d> &limit_road_oppsite_global,
      std::vector<Vec2d> &limit_road_rear_global,
      std::vector<Vec2d> &limit_carport_coor_global,
      std::vector<Vec2d> &limit_road_front_global);

  bool ExtractRoadboundKeyPts(const std::vector<Vec2d> &road_bound,
                              std::vector<Vec2d> &key);

  void LogReferLineBound(const std::vector<Vec2d> &vec_road,
                         const std::vector<Vec2d> &vec_road_oppsite,
                         const std::vector<ReferencePoint> &refer_pts);

  void LogParkingBoundary(const std::vector<Vec2d> &round_pts,
                          const Vec2d &veh_xy, const Vec2d &ref_start,
                          const Vec2d &ref_end);

 private:
  ParkingConfig *p_park_config_;
  ParkingDataInfo *parking_data_info_;

  // convert anchor_vertices to origin_coor
  // used in several functions
  Vec2d left_top_origin_;
  Vec2d left_down_origin_;
  Vec2d right_top_origin_;
  Vec2d right_down_origin_;
};

}  // namespace planning
}  // namespace neodrive
