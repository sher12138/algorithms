#pragma once

#include <deque>
#include <map>

#include "common/macros.h"
#include "common/math/polygon2d.h"
#include "common/math/segment2d.h"
#include "common/math/vec2d.h"
#include "common/obstacle/object_table.h"
#include "cyber/common/macros.h"
#include "math/common/kdtree.h"

namespace neodrive {
namespace planning {
namespace localmap {
using ObsState = std::array<double, 10>;
}
class ObsFrameContainer {
 public:
  ObsFrameContainer(int fid = -1, double time = -1.0);
  ObsFrameContainer(const ObsFrameContainer &obsf);
  ~ObsFrameContainer() {}

 public:
  enum ObsShapeType {
    PointType = 0,
    PolylineType = 1,
    PolygonType = 2,
    obsShapeTypeCount
  };
  enum ObsCategory {
    UnknownType = 0,
    Static = 1,
    Dynamic = 2,
    Pedestrian = 3,
    Bicycle = 4,
    obsCategoryCount
  };

  struct ObsInfo {
    int shape_type{PolygonType};
    int category{UnknownType};
    int obs_id{-1};
    uint64_t matched_lane_id;
    double matched_lane_heading_deviation;
  };

  void Clear();

  inline const bool Empty() const { return frame_id_ == -1; }

  void GenerateFrame();

  void AddPolygon(int id, uint64_t matched_lane,
                  double matched_lane_heading_deviation,
                  const Obstacle::ObstacleType &type, const math::Polygon &poly,
                  const localmap::ObsState &states);

  void AddPolygon(int id, uint64_t matched_lane,
                  double matched_lane_heading_deviation,
                  const Obstacle::ObstacleType &type, const Polygon2d &poly,
                  const localmap::ObsState &states);

  inline ObsInfo &GetObsInfo(int idx) { return obs_id_idx_vec_[idx].first; }

  inline const localmap::ObsState &GetObsState(int idx) {
    return obs_id_idx_vec_[idx].second;
  }

  inline const int GetObsIdx(int id) const {
    return obs_id_ridx_map_.count(id) > 0 ? obs_id_ridx_map_.at(id) : -1;
  }

  inline const int GetObsReverseId(int idx) const {
    return obs_id_idx_map_.count(idx) > 0 ? obs_id_idx_map_.at(idx) : -1;
  }

  DEFINE_SIMPLE_TYPE_GET_FUNCTION(int, frame_id);

  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, timestamp);

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(math::KdTree<math::Node<math::Polygon>>,
                                   obs_polygon_tree);

  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<math::Node<math::Polygon>>,
                                       obs_polygons_vec);

 private:
  ObsFrameContainer::ObsCategory TypeTransfrom(
      const Obstacle::ObstacleType &type);

  void CreatefromPolygons() { obs_polygon_tree_.CreateKDT(obs_polygons_vec_); };

  bool Update(ObsShapeType shapetype, int category, int id,
              uint64_t matched_lane, double matched_lane_heading_deviation,
              const localmap::ObsState &states);

 private:
  std::vector<std::pair<ObsInfo, localmap::ObsState>> obs_id_idx_vec_{};
  std::map<int, int> obs_id_ridx_map_{};
  std::map<int, int> obs_id_idx_map_{};
  math::KdTree<math::Node<math::Polygon>> obs_polygon_tree_{};
  std::vector<math::Node<math::Polygon>> obs_polygons_vec_{};
  Vec2d ego_pos_utm_{0.0, 0.0};
  Vec2d ego_pos_odom_{0.0, 0.0};

  int frame_id_{-1};
  double timestamp_{-1.0};
};

}  // namespace planning
}  // namespace neodrive
