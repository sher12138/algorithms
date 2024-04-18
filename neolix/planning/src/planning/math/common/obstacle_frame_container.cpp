#include "obstacle_frame_container.h"

namespace neodrive {
namespace planning {

ObsFrameContainer::ObsFrameContainer(int fid, double time)
    : frame_id_{fid}, timestamp_{time} {}

ObsFrameContainer::ObsFrameContainer(const ObsFrameContainer &obsf) {
  obs_id_idx_vec_ = obsf.obs_id_idx_vec_;
  obs_id_ridx_map_ = obsf.obs_id_ridx_map_;
  obs_id_idx_map_ = obsf.obs_id_idx_map_;
  for (auto &poly : obsf.obs_polygons_vec_)
    obs_polygons_vec_.emplace_back(poly);
  GenerateFrame();
}

void ObsFrameContainer::Clear() {
  obs_id_idx_vec_.clear();
  obs_id_ridx_map_.clear();
  obs_id_ridx_map_.clear();
  obs_polygons_vec_.clear();
  obs_polygon_tree_.Clear();
  frame_id_ = -1;
}

void ObsFrameContainer::GenerateFrame() { CreatefromPolygons(); }

ObsFrameContainer::ObsCategory ObsFrameContainer::TypeTransfrom(
    const Obstacle::ObstacleType &type) {
  switch (type) {
    case Obstacle::ObstacleType::UNKNOWN:
      return UnknownType;
    case Obstacle::ObstacleType::VEHICLE:
    case Obstacle::ObstacleType::UNKNOWN_MOVABLE:
      return Dynamic;
    case Obstacle::ObstacleType::UNKNOWN_UNMOVABLE:
      return Static;
    case Obstacle::ObstacleType::PEDESTRIAN:
      return Pedestrian;
    case Obstacle::ObstacleType::BICYCLE:
      return Bicycle;
  }
}

void ObsFrameContainer::AddPolygon(int id, uint64_t matched_lane,
                                   double matched_lane_heading_deviation,
                                   const Obstacle::ObstacleType &type,
                                   const math::Polygon &poly,
                                   const localmap::ObsState &states) {
  if (Update(PolygonType, TypeTransfrom(type), id, matched_lane,
             matched_lane_heading_deviation, states)) {
    obs_polygons_vec_.emplace_back(
        math::Node<math::Polygon>{int(obs_polygons_vec_.size()), poly});
  }
}

void ObsFrameContainer::AddPolygon(int id, uint64_t matched_lane,
                                   double matched_lane_heading_deviation,
                                   const Obstacle::ObstacleType &type,
                                   const Polygon2d &poly,
                                   const localmap::ObsState &states) {
  if (Update(PolygonType, TypeTransfrom(type), id, matched_lane,
             matched_lane_heading_deviation, states)) {
    obs_polygons_vec_.emplace_back(math::Node<math::Polygon>{
        static_cast<int>(obs_polygons_vec_.size()), (math::Polygon)poly});
  }
}

bool ObsFrameContainer::Update(ObsShapeType shapetype, int category, int id,
                               uint64_t matched_lane,
                               double matched_lane_heading_deviation,
                               const localmap::ObsState &states) {
  if (obs_id_ridx_map_.count(id) == 0) {
    obs_id_idx_vec_.push_back(
        std::make_pair(ObsInfo{.shape_type = shapetype,
                               .category = category,
                               .obs_id = id,
                               .matched_lane_id = matched_lane,
                               .matched_lane_heading_deviation =
                                   matched_lane_heading_deviation},
                       states));
    obs_id_ridx_map_.insert({id, obs_id_idx_vec_.size() - 1});
    obs_id_idx_map_.insert({obs_id_idx_vec_.size() - 1, id});
    return true;
  } else {
    LOG_INFO("Duplicate obstacle id: #{}", id);
    return false;
  }
}

}  // namespace planning
}  // namespace neodrive