#include "PolygonRelation.h"

namespace neodrive {
namespace planning {

Polygon_RELATION::Polygon_RELATION() { vertex_.clear(); }

Polygon_RELATION::Polygon_RELATION(const Polygon_RELATION& other) {
  vertex_.clear();
  size_t vertex_size = other.vertex_.size();
  for (size_t i = 0; i < vertex_size; ++i) vertex_.push_back(other.vertex_[i]);
}

Polygon_RELATION::~Polygon_RELATION() { vertex_.clear(); }

Polygon_RELATION& Polygon_RELATION::operator=(const Polygon_RELATION& other) {
  if (this != &other) {
    vertex_.clear();
    size_t vertex_size = other.vertex_.size();
    for (size_t i = 0; i < vertex_size; ++i)
      vertex_.push_back(other.vertex_[i]);
  }
  return *this;
}

bool Polygon_RELATION::operator==(const Polygon_RELATION& other) {
  if (vertex_.size() != other.vertex_.size()) return false;
  size_t vertex_size = vertex_.size();
  for (size_t i = 0; i < vertex_size; ++i) {
    if (!(vertex_[i] == other.vertex_[i])) return false;
  }
  return true;
}
// poly algorithm
bool Polygon_RELATION::PointInPolygon(double x, double y) {
  int inum, icount_vertex, quadrant_1, quadrant_2, sum, f;
  icount_vertex = vertex_.size();
  if (icount_vertex <= 1) return false;
  std::vector<Vec2d> tmp_vertex;
  for (int i = 0; i < icount_vertex; ++i) tmp_vertex.push_back(vertex_[i]);
  tmp_vertex.push_back(vertex_[0]);
  // improved riks method
  for (inum = 0; inum <= icount_vertex; ++inum) {
    tmp_vertex[inum].set_x(tmp_vertex[inum].x() - x);
    tmp_vertex[inum].set_y(tmp_vertex[inum].y() - y);
  }
  quadrant_1 = tmp_vertex[0].x() >= 0 ? (tmp_vertex[0].y() >= 0 ? 0 : 3)
                                      : (tmp_vertex[0].y() >= 0 ? 1 : 2);
  for (sum = 0, inum = 1; inum <= icount_vertex; ++inum) {
    if (!tmp_vertex[inum].x() && !tmp_vertex[inum].y()) break;
    f = (int)(tmp_vertex[inum].y() * tmp_vertex[inum - 1].x() -
              tmp_vertex[inum].x() * tmp_vertex[inum - 1].y());
    if (!f && tmp_vertex[inum - 1].x() * tmp_vertex[inum].x() <= 0 &&
        tmp_vertex[inum - 1].y() * tmp_vertex[inum].y() <= 0)
      break;
    quadrant_2 = tmp_vertex[inum].x() >= 0
                     ? (tmp_vertex[inum].y() >= 0 ? 0 : 3)
                     : (tmp_vertex[inum].y() >= 0 ? 1 : 2);
    if (quadrant_2 == (quadrant_1 + 1) % 4)
      sum += 1;
    else if (quadrant_2 == (quadrant_1 + 3) % 4)
      sum -= 1;
    else if (quadrant_2 == (quadrant_1 + 2) % 4) {
      if (f > 0)
        sum += 2;
      else
        sum -= 2;
    }
    quadrant_1 = quadrant_2;
  }
  if (inum <= icount_vertex || sum) return true;
  return false;
}
// collision check, true: collided
bool Polygon_RELATION::PolygonCollision(Polygon_RELATION& other) {
  size_t i_vertex = other.vertex_.size();
  bool bcollide = false;
  for (size_t i = 0; i < i_vertex; ++i) {
    bcollide = PointInPolygon(other.vertex_[i].x(), other.vertex_[i].y());
    if (bcollide) break;
  }
  return bcollide;
}
void Polygon_RELATION::SetRect(double x1, double y1, double x2, double y2,
                               double x3, double y3, double x4, double y4) {
  vertex_.clear();
  Vec2d tmp_vertex;
  tmp_vertex.set_x(x1);
  tmp_vertex.set_y(y1);
  vertex_.push_back(tmp_vertex);
  tmp_vertex.set_x(x2);
  tmp_vertex.set_y(y2);
  vertex_.push_back(tmp_vertex);
  tmp_vertex.set_x(x3);
  tmp_vertex.set_y(y3);
  vertex_.push_back(tmp_vertex);
  tmp_vertex.set_x(x4);
  tmp_vertex.set_y(y4);
  vertex_.push_back(tmp_vertex);
}
void Polygon_RELATION::SetRectvertex(std::vector<Vec2d> vertex) {
  vertex_ = vertex;
}

}  // namespace planning
}  // namespace neodrive
