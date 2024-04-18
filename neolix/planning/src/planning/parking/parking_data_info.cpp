#include "parking_data_info.h"

namespace neodrive {
namespace planning {

ParkingDataInfo::ParkingDataInfo() {}
ParkingDataInfo::~ParkingDataInfo() {}

void ParkingDataInfo::SetParkingRefLine(
    const ReferenceLinePtr parking_ref_line) {
  parking_ref_line_ = parking_ref_line;
}
void ParkingDataInfo::SetSpotVertices(std::vector<Vec2d>& spot_vertices) {
  spot_vertices_ = spot_vertices;
}
void ParkingDataInfo::SetParkSpotType(const int& park_spot_type) {
  park_spot_type_ = park_spot_type;
}
void ParkingDataInfo::SetLeftRightType(const int& left_right_type) {
  left_right_type_ = left_right_type;
}
void ParkingDataInfo::SetHeadTailIn(const int& head_tail_in) {
  head_tail_in_ = head_tail_in;
}

void ParkingDataInfo::SetFinalPosFlag(const bool& set_final_pos_flag) {
  set_final_pos_flag_ = set_final_pos_flag;
}

void ParkingDataInfo::SetPreDefinedFinalPos(
    const TrajectoryPoint& pre_defined_final_pos) {
  pre_defined_final_pos_ = pre_defined_final_pos;
}

void ParkingDataInfo::SetAnchorPointCoordinate(
    const ReferencePoint& anchor_point) {
  anchor_point_ = anchor_point;
}

void ParkingDataInfo::SetCurrPosOnAnchor(const TrajectoryPoint& curr_pos) {
  curr_pos_ = curr_pos;
}
void ParkingDataInfo::SetFinalPosOnAnchor(const TrajectoryPoint& final_pos) {
  final_pos_ = final_pos;
}

void ParkingDataInfo::SetCurrGps(const Vec2d& curr_gps) {
  curr_gps_ = curr_gps;
}
void ParkingDataInfo::SetCurrHeading(const double& curr_heading) {
  curr_heading_ = curr_heading;
}

void ParkingDataInfo::SetOriginPoint(const Vec2d& origin_point) {
  origin_point_ = origin_point;
}
void ParkingDataInfo::SetOriginHeading(const double& origin_heading) {
  origin_heading_ = origin_heading;
}
void ParkingDataInfo::SetParkingEndPose(
    const std::vector<double>& parking_end_pose) {
  parking_end_pose_ = parking_end_pose;
}
void ParkingDataInfo::SetConsiderObstcleFlag(
    const bool& consider_obstcle_flag) {
  consider_obstcle_flag_ = consider_obstcle_flag;
}
// void ParkingDataInfo::set_obs_list(std::vector<Obstacle>& obs_list){
// obs_list_=obs_list;
//}
void ParkingDataInfo::SetROIXyBoundary(
    const std::vector<double>& ROI_xy_boundary) {
  ROI_xy_boundary_ = ROI_xy_boundary;
}
void ParkingDataInfo::SetObstaclesNum(const std::size_t& obstacles_num) {
  obstacles_num_ = obstacles_num;
}
void ParkingDataInfo::SetObstaclesEdgesNum(
    const Eigen::MatrixXi& obstacles_edges_num) {
  obstacles_edges_num_ = obstacles_edges_num;
}
void ParkingDataInfo::SetObstaclesVerticesVec(
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec) {
  obstacles_vertices_vec_ = obstacles_vertices_vec;
}
void ParkingDataInfo::SetObstaclesA(const Eigen::MatrixXd& obstacles_A) {
  obstacles_A_ = obstacles_A;
}
void ParkingDataInfo::SetObstaclesB(const Eigen::MatrixXd& obstacles_b) {
  obstacles_b_ = obstacles_b;
}

void ParkingDataInfo::SetLimitCarportCoorGlobal(
    std::vector<Vec2d>& limit_carport_coor_global) {
  limit_carport_coor_global_ = limit_carport_coor_global;
}
void ParkingDataInfo::SetLimitRoadOppsiteGlobal(
    std::vector<Vec2d>& limit_road_oppsite_global) {
  limit_road_oppsite_global_ = limit_road_oppsite_global;
}
void ParkingDataInfo::SetLimitRoadRearGlobal(
    std::vector<Vec2d>& limit_road_rear_global) {
  limit_road_rear_global_ = limit_road_rear_global;
}
void ParkingDataInfo::SetLimitRoadFrontGlobal(
    std::vector<Vec2d>& limit_road_front_global) {
  limit_road_front_global_ = limit_road_front_global;
}

void ParkingDataInfo::SetLimitCarportCoorAnchor(
    std::vector<Vec2d>& limit_carport_coor_anchor) {
  limit_carport_coor_anchor_ = limit_carport_coor_anchor;
}
void ParkingDataInfo::SetLimitRoadOppsiteAnchor(
    std::vector<Vec2d>& limit_road_oppsite_anchor) {
  limit_road_oppsite_anchor_ = limit_road_oppsite_anchor;
}
void ParkingDataInfo::SetLimitRoadRearAnchor(
    std::vector<Vec2d>& limit_road_rear_anchor) {
  limit_road_rear_anchor_ = limit_road_rear_anchor;
}
void ParkingDataInfo::SetLimitRoadFrontAnchor(
    std::vector<Vec2d>& limit_road_front_anchor) {
  limit_road_front_anchor_ = limit_road_front_anchor;
}

ReferenceLinePtr ParkingDataInfo::ParkingRefLine() const {
  return parking_ref_line_;
}
std::vector<Vec2d> ParkingDataInfo::SpotVertices() const {
  return spot_vertices_;
}
int ParkingDataInfo::ParkSpotType() const { return park_spot_type_; }
int ParkingDataInfo::LeftRightType() const { return left_right_type_; }
int ParkingDataInfo::HeadTailIn() const { return head_tail_in_; }
bool ParkingDataInfo::FinalPosFlag() const { return set_final_pos_flag_; }
TrajectoryPoint ParkingDataInfo::PreDefinedFinalPos() const {
  return pre_defined_final_pos_;
}

ReferencePoint ParkingDataInfo::AnchorPointCoordinate() const {
  return anchor_point_;
}

TrajectoryPoint ParkingDataInfo::CurrPosOnAnchor() const { return curr_pos_; }
TrajectoryPoint ParkingDataInfo::FinalPosOnAnchor() const { return final_pos_; }

Vec2d ParkingDataInfo::CurrGps() const { return curr_gps_; }
double ParkingDataInfo::CurrHeading() const { return curr_heading_; }

Vec2d ParkingDataInfo::OriginPoint() const { return origin_point_; }
double ParkingDataInfo::OriginHeading() const { return origin_heading_; }
std::vector<double> ParkingDataInfo::ParkingEndPose() const {
  return parking_end_pose_;
}
bool ParkingDataInfo::ConsiderObstcleFlag() const {
  return consider_obstcle_flag_;
}
// std::vector<Obstacle> ParkingDataInfo::obs_list() const{
// return obs_list_;
//}
std::vector<double> ParkingDataInfo::ROIXyBoundary() const {
  return ROI_xy_boundary_;
}
std::size_t ParkingDataInfo::ObstaclesNum() const { return obstacles_num_; }
Eigen::MatrixXi ParkingDataInfo::ObstaclesEdgesNum() const {
  return obstacles_edges_num_;
}
Eigen::MatrixXi* ParkingDataInfo::MutableObstaclesEdgesNum() {
  return &obstacles_edges_num_;
}
std::vector<std::vector<Vec2d>> ParkingDataInfo::ObstaclesVerticesVec() const {
  return obstacles_vertices_vec_;
}
std::vector<std::vector<Vec2d>>*
ParkingDataInfo::MutableObstaclesVerticesVec() {
  return &obstacles_vertices_vec_;
}
Eigen::MatrixXd ParkingDataInfo::ObstaclesA() const { return obstacles_A_; }
Eigen::MatrixXd* ParkingDataInfo::MutableObstaclesA() { return &obstacles_A_; }
Eigen::MatrixXd ParkingDataInfo::ObstaclesB() const { return obstacles_b_; }
Eigen::MatrixXd* ParkingDataInfo::MutableObstaclesB() { return &obstacles_b_; }

std::vector<Vec2d> ParkingDataInfo::LimitCarportCoorGlobal() const {
  return limit_carport_coor_global_;
}
std::vector<Vec2d> ParkingDataInfo::LimitRoadOppsiteGlobal() const {
  return limit_road_oppsite_global_;
}
std::vector<Vec2d> ParkingDataInfo::LimitRoadRearGlobal() const {
  return limit_road_rear_global_;
}
std::vector<Vec2d> ParkingDataInfo::LimitRoadFrontGlobal() const {
  return limit_road_front_global_;
}

std::vector<Vec2d>* ParkingDataInfo::MutableLimitCarportCoorGlobal() {
  return &limit_carport_coor_global_;
}
std::vector<Vec2d>* ParkingDataInfo::MutableLimitRoadOppsiteGlobal() {
  return &limit_road_oppsite_global_;
}
std::vector<Vec2d>* ParkingDataInfo::MutableLimitRoadRearGlobal() {
  return &limit_road_rear_global_;
}
std::vector<Vec2d>* ParkingDataInfo::MutableLimitRoadFrontGlobal() {
  return &limit_road_front_global_;
}

std::vector<Vec2d> ParkingDataInfo::LimitCarportCoorAnchor() const {
  return limit_carport_coor_anchor_;
}
std::vector<Vec2d> ParkingDataInfo::LimitRoadOppsiteAnchor() const {
  return limit_road_oppsite_anchor_;
}
std::vector<Vec2d> ParkingDataInfo::LimitRoadRearAnchor() const {
  return limit_road_rear_anchor_;
}
std::vector<Vec2d> ParkingDataInfo::LimitRoadFrontAnchor() const {
  return limit_road_front_anchor_;
}

}  // namespace planning
}  // namespace neodrive
