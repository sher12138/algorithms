#include "occupy_map.h"

#include <deque>
#include <queue>

#include "src/planning/common/math/vec2d.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/visualizer_event/visualizer_event.h"

namespace neodrive {
namespace planning {

namespace {

using AD2 = OccupyMap::AD2;
using PII = std::pair<int, int>;

double WrapAngle(const double angle) {
  const double new_angle = std::fmod(angle, M_PI * 2.0);
  return new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle;
}

/// https://www.cse.chalmers.se/edu/year/2013/course/TDA361/grid.pdf
std::vector<PII> VoxelRayTraversal(const AD2& base, const double grid_step,
                                   const AD2& start, const AD2& end,
                                   const PII& size) {
  auto to_index = [&](auto& pt) -> PII {
    return {std::clamp(static_cast<int>((pt[0] - base[0]) / grid_step), 0,
                       size.first - 1),
            std::clamp(static_cast<int>((pt[1] - base[1]) / grid_step), 0,
                       size.second - 1)};
  };
  PII curr_voxel = to_index(start);
  PII last_voxel = to_index(end);
  AD2 ray{end[0] - start[0], end[1] - start[1]};
  // LOG_INFO("base ({:.4f}, {:.4f})", base[0], base[1]);
  // LOG_INFO("start ({:.4f}, {:.4f})", start[0], start[1]);
  // LOG_INFO("end ({:.4f}, {:.4f})", end[0], end[1]);
  // LOG_INFO("curr voxel ({}, {})\n", curr_voxel.first, curr_voxel.second);
  // LOG_INFO("last voxel ({}, {})\n", last_voxel.first, last_voxel.second);
  // LOG_INFO("ray ({:.4f}, {:.4f})\n", ray[0], ray[1]);

  bool is_horizon = curr_voxel.first == last_voxel.first,
       is_vertial = curr_voxel.second == last_voxel.second;

  PII dir{ray[0] > 0 ? 1 : -1, ray[1] > 0 ? 1 : -1};

  AD2 next_grid{(curr_voxel.first + (dir.first > 0)) * grid_step + base[0],
                (curr_voxel.second + (dir.second > 0)) * grid_step + base[1]};
  AD2 t_max = {std::abs((next_grid[0] - start[0]) / ray[0]),
               std::abs((next_grid[1] - start[1]) / ray[1])};
  AD2 t_step{std::abs(grid_step / ray[0] * dir.first),
             std::abs(grid_step / ray[1] * dir.second)};
  if (is_horizon) t_max[0] = t_step[0] = 1e7;
  if (is_vertial) t_max[1] = t_step[1] = 1e7;

  std::vector<PII> ans{curr_voxel};
  while (curr_voxel.first != last_voxel.first ||
         curr_voxel.second != last_voxel.second) {
    if (t_max[0] < t_max[1]) {
      t_max[0] += t_step[0];
      curr_voxel.first += dir.first;
    } else {
      t_max[1] += t_step[1];
      curr_voxel.second += dir.second;
    }
    ans.push_back(curr_voxel);
    if (ans.size() > size.first * size.second) {
      LOG_INFO("oversize: {}", ans.size());
      return ans;
    }
  }

  return ans;
}

void FillMatrix(std::vector<std::vector<int>>& mat, int from, int to) {
  for (size_t i = 0; i < mat.size(); ++i) {
    for (size_t j = 0; j < mat[0].size(); ++j) {
      if (mat[i][j] == from) mat[i][j] = to;
    }
  }
}

void BfsFillColor(std::vector<std::vector<int>>& mat, int i, int j, int from,
                  int to) {
  if (mat[i][j] != from) return;
  mat[i][j] = to;

  int n = mat.size(), m = mat[0].size();
  std::queue<PII> q{};
  int step[5] = {1, 0, -1, 0, 1};

  q.push({i, j});
  while (q.size()) {
    auto [x, y] = q.front();
    q.pop();

    for (int k = 0; k < 4; ++k) {
      int xx = x + step[k], yy = y + step[k + 1];
      if (xx >= 0 && xx < n && yy >= 0 && yy < m && mat[xx][yy] == from) {
        mat[xx][yy] = to;
        q.push({xx, yy});
      }
    }
  }
}

void DrawMatrixEdge(std::vector<std::vector<int>>& mat, int from, int to) {
  int n = mat.size(), m = mat.front().size();
  for (int i = 0; i < n; ++i) {
    if (mat[i][0] == from) BfsFillColor(mat, i, 0, from, to);
    if (mat[i][m - 1] == from) BfsFillColor(mat, i, m - 1, from, to);
  }

  for (int j = 0; j < m; ++j) {
    if (mat[0][j] == from) BfsFillColor(mat, 0, j, from, to);
    if (mat[n - 1][j] == from) BfsFillColor(mat, n - 1, j, from, to);
  }
}

void InflateObstacles(std::vector<std::vector<int>>& grid_map, int inflationRadius) {
  int n = grid_map.size();
  int m = grid_map[0].size();
  std::vector<std::vector<bool>> visited(n, std::vector<bool>(m, false));

  int inflate_nums = 0;
  // iterate over each vector cell of the raster map
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < m; ++j) {
      // If the current cell is an obstacle
      if (grid_map[i][j] == 1 && !visited[i][j]) {
        // Process elements within a fixed dilation radius
        for (int x = std::max(0, i - inflationRadius);
          x <= std::min(n - 1, i + inflationRadius); ++x) {
          for (int y = std::max(0, j - inflationRadius);
            y <= std::min(m - 1, j + inflationRadius); ++y) {
            // Set all cells within the expansion radius as obstacles
            if(grid_map[x][y] == 2){
              ++inflate_nums;
              grid_map[x][y] = 1;
              visited[x][y] = true;
            }
          }
        }
      }
    }
  }
  LOG_INFO("inflate_nums = {}", inflate_nums);
}
void InflateObstacles(std::vector<std::vector<int>>& grid_map) {
  int n = grid_map.size();
  int m = grid_map[0].size();
  std::vector<std::vector<bool>> visited(n, std::vector<bool>(m, false));
  int go[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  int inflate_nums = 0;
  // iterate over each vector cell of the raster map
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < m; ++j) {
      // If the current cell is an obstacle
      if (grid_map[i][j] == 1 && !visited[i][j]) {
        // Process elements within a fixed dilation radius
        for (int k = 0; k < 4; ++k) {
          int x = i + go[k][0], y = j + go[k][1];
          if(x < 0 || x >= n || y < 0 || y >= m)
            continue;
          if(grid_map[x][y] == 2){
            ++inflate_nums;
            grid_map[x][y] = 1;
            visited[x][y] = true;
          }
        }
      }
    }
  }
  LOG_INFO("inflate_nums = {}", inflate_nums);
}

/// Ray casting algorithm + difference array
std::vector<std::vector<int>> BuildPosGrid(
    const AD2& base, const PII& size, const double grid_step,
    const std::vector<AD2>& polygon,
    const std::vector<std::vector<AD2>>& holes) {
  std::vector grid(size.first, std::vector<int>(size.second, 0));

  /// grid: freespace: 0, invalid area: 1;
  for (int i = 0, n = polygon.size(); i < n; ++i) {
    const AD2 &a = polygon[i], &b = polygon[(i + 1) % n];
    for (auto [x, y] : VoxelRayTraversal(base, grid_step, a, b, size)) {
      grid[x][y] = 1;
    }
  }
  DrawMatrixEdge(grid, 0, 1);
  /// gh: holes: 1, invlaid: 2;
  if (holes.size()) {
    std::vector gh(size.first, std::vector<int>(size.second, 0));
    for (auto& hole : holes) {
      for (int i = 0; i < hole.size(); ++i) {
        // LOG_INFO("gh: {}, {}", size.first, size.second);
        const AD2 &a = hole[i], &b = hole[(i + 1) % hole.size()];
        for (auto [x, y] : VoxelRayTraversal(base, grid_step, a, b, size)) {
          // LOG_INFO("x: {}, y:{}", x, y);
          if (x >= size.first || y >= size.second || x < 0 || y < 0) {
            LOG_ERROR("XX oversize");
            continue;
          }
          gh[x][y] = 1;
        }
      }
    }
    DrawMatrixEdge(gh, 0, 2);
    FillMatrix(gh, 0, 1);
    // Inflate the obstacle (There are two overloaded versions)
    InflateObstacles(gh);

    /// merge holes to grid
    for (size_t i = 0; i < grid.size(); ++i) {
      for (size_t j = 0; j < grid.front().size(); ++j) {
        if (gh[i][j] == 1) grid[i][j] = 1;
      }
    }
  }

  return grid;
}

/// Prefix sum
std::vector<std::vector<int>> BuildRecGrid(
    const std::vector<std::vector<int>>& pos_grid) {
  size_t n = pos_grid.size(), m = pos_grid.front().size();
  std::vector<std::vector<int>> grid(n + 1, std::vector<int>(m + 1, 0));
  for (size_t i = 1; i <= n; ++i) {
    for (size_t j = 1; j <= m; ++j) {
      grid[i][j] = pos_grid[i - 1][j - 1] + grid[i - 1][j] + grid[i][j - 1] -
                   grid[i - 1][j - 1];
    }
  }

  return grid;
}

std::vector<std::vector<double>> BuildEsdfGrid(
    const std::vector<std::vector<int>>& grid, const double grid_step) {
  const int n = grid.size(), m = grid[0].size();
  std::vector esdf(n, std::vector<double>(m, 0.));

  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < m; ++j) {
      if (grid[i][j] == 1) esdf[i][j] = -10;
    }
  }
  int dir[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  double dis = 1e-2;
  std::deque<PII> dq{};
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < m; ++j) {
      if (esdf[i][j] < -5) continue;
      for (int k = 0; k < 4; ++k) {
        int nx = i + dir[k][0], ny = j + dir[k][1];
        if (nx < 0 || nx >= n || ny < 0 || ny >= m || esdf[nx][ny] > -5)
          continue;
        dq.push_back({i, j});
        esdf[i][j] = dis;
      }
    }
  }
  while (dq.size()) {
    int size = dq.size();
    dis += grid_step;
    while (size--) {
      auto [x, y] = dq.front();
      dq.pop_front();

      for (int i = 0; i < 4; ++i) {
        int nx = x + dir[i][0], ny = y + dir[i][1];
        if (nx < 0 || nx >= n || ny < 0 || ny >= m || esdf[nx][ny] < -5 ||
            esdf[nx][ny] > 1e-3)
          continue;
        dq.push_back({nx, ny});
        esdf[nx][ny] = dis;
      }
    }
  }

  return esdf;
}

std::vector<AD2> GenerateRectangleEdge(const OccupyMap::Config& conf,
                                       const AD2& base, const double theta) {
  Vec2d u{std::cos(theta), std::sin(theta)};
  Vec2d pu = u.rotate(M_PI_2);
  Vec2d c{base[0], base[1]};

  auto p0 =
      c + u * conf.rectangle_front_half_length + pu * conf.rectangle_half_width;
  auto p1 =
      c + u * conf.rectangle_front_half_length - pu * conf.rectangle_half_width;
  auto p2 =
      c - u * conf.rectangle_back_half_length - pu * conf.rectangle_half_width;
  auto p3 =
      c - u * conf.rectangle_back_half_length + pu * conf.rectangle_half_width;
  return {AD2{p0.x(), p0.y()}, AD2{p1.x(), p1.y()}, AD2{p2.x(), p2.y()},
          AD2{p3.x(), p3.y()}};
}

std::vector<std::array<int, 3>> GenerateRecRowRange(const std::vector<AD2>& pts,
                                                    const AD2& base,
                                                    const double grid_step) {
  std::vector<PII> xys{};
  for (int i = 0, n = pts.size(); i < n; ++i) {
    auto &a = pts[i], &b = pts[(i + 1) % n];
    auto curr = VoxelRayTraversal(base, grid_step, a, b, {1e7, 1e7});
    xys.insert(xys.end(), curr.begin(), curr.end());
  }

  std::sort(xys.begin(), xys.end(), [](auto& a, auto& b) {
    return a.second == b.second ? a.first < b.first : a.second < b.second;
  });

  std::vector<std::array<int, 3>> ans{};
  for (auto [x, y] : xys) {
    if (ans.empty() || y != ans.back()[2]) {
      ans.push_back({x, x, y});
    } else {
      ans.back()[1] = x;
    }
  }

  int offx = base[0] / grid_step;
  int offy = base[1] / grid_step;
  for (auto& [x1, x2, y] : ans) x1 += offx, x2 += offx, y += offy;

  // for (auto [x1, x2, y] : ans) printf("lib (%d, %d, %d)\n", x1, x2, y);

  return ans;
}

}  // namespace

OccupyMap::OccupyMap(Config&& conf, const std::vector<AD2>& polygon,
                     const std::vector<std::vector<AD2>>& holes)
    : conf_{std::move(conf)} {
  for (auto [x, y] : polygon) {
    x_min_ = std::min(x_min_, x);
    x_max_ = std::max(x_max_, x);
    y_min_ = std::min(y_min_, y);
    y_max_ = std::max(y_max_, y);
  }
  const PII size{(x_max_ - x_min_) / conf_.grid_step + 1,
                 (y_max_ - y_min_) / conf_.grid_step + 1};
  LOG_INFO("occupy map size ({}, {})", size.first, size.second);
  LOG_INFO("occupy map begin");

  auto bin_map =
      BuildPosGrid({x_min_, y_min_}, size, conf_.grid_step, polygon, holes);
  LOG_INFO("occupy map build bin map");

  grid_ = BuildRecGrid(bin_map);
  // esdf_ = BuildEsdfGrid(bin_map, conf_.grid_step);

  LOG_INFO("occupy map grid");
  auto lt = [](auto& a, auto& b) { return a[1] < b[1]; };
  for (double h = 1e-5; h < 2 * M_PI; h += conf_.rectangle_heading_step) {
    auto pts = GenerateRectangleEdge(conf_, {0, 0}, h);
    const double x_min = std::min_element(pts.begin(), pts.end())->at(0);
    const double y_min = std::min_element(pts.begin(), pts.end(), lt)->at(1);

    // printf("box (%d, %d, %d, %d)\n", int(x_min / conf_.grid_step),
    //        int(x_max / conf_.grid_step), int(y_min / conf_.grid_step),
    //        int(y_max / conf_.grid_step));
    RectangleIndex curr{
        .row_range = GenerateRecRowRange(pts, {x_min, y_min}, conf_.grid_step)};
    for (auto [x1, x2, y] : curr.row_range) {
      curr.x_min = std::min({curr.x_min, x1, x2});
      curr.y_min = std::min({curr.y_min, y});
      curr.x_max = std::max({curr.x_max, x1, x2});
      curr.y_max = std::max({curr.y_max, y});
    }
    rec_lib_.push_back(std::move(curr));
  }
  LOG_INFO("Done with occupy map");
}

bool OccupyMap::IsPointOccupied(const AD2& pt) const {
  if (pt[0] < x_min_ || pt[0] > x_max_ || pt[1] < y_min_ || pt[1] > y_max_)
    return true;
  const int x = (pt[0] - x_min_) / conf_.grid_step;
  const int y = (pt[1] - y_min_) / conf_.grid_step;
  return grid_[x + 1][y + 1] - grid_[x][y + 1] - grid_[x + 1][y] + grid_[x][y];
}

double OccupyMap::GetPointMinDistanceToObstacle(const AD2& pt) const {
  if (pt[0] < x_min_ || pt[0] > x_max_ || pt[1] < y_min_ || pt[1] > y_max_)
    return 0;
  const int x = (pt[0] - x_min_) / conf_.grid_step;
  const int y = (pt[1] - y_min_) / conf_.grid_step;
  return esdf_[x][y];
}

std::array<int, 2> OccupyMap::GetPointGradientDirection(const AD2& pt) const {
  if (pt[0] < x_min_ || pt[0] > x_max_ || pt[1] < y_min_ || pt[1] > y_max_)
    return {0, 0};
  const int x = (pt[0] - x_min_) / conf_.grid_step;
  const int y = (pt[1] - y_min_) / conf_.grid_step;
  const double err = conf_.grid_step / 2;
  const int n = esdf_.size(), m = esdf_[0].size();
  const int ux = std::min(x + 1, n - 1), lx = std::max(x - 1, 0);
  const int uy = std::min(y + 1, m - 1), ly = std::max(y - 1, 0);
  const double dx = esdf_[ux][y] - esdf_[lx][y];
  const double dy = esdf_[x][uy] - esdf_[x][ly];
  return {std::abs(dx) < err ? 0 : (dx < 0 ? -1 : 1),
          std::abs(dy) < err ? 0 : (dy < 0 ? -1 : 1)};
}

bool OccupyMap::IsAaBoxOccupied(const AD2& lb, const AD2& rt) const {
  if (IsPointOccupied(lb) || IsPointOccupied(rt)) return true;
  const int x_min = (lb[0] - x_min_) / conf_.grid_step;
  const int y_min = (lb[1] - y_min_) / conf_.grid_step;
  const int x_max = (rt[0] - x_min_) / conf_.grid_step;
  const int y_max = (rt[1] - y_min_) / conf_.grid_step;

  return grid_[x_max + 1][y_max + 1] - grid_[x_max + 1][y_min] -
         grid_[x_min][y_max + 1] + grid_[x_min][y_min];
}

bool OccupyMap::IsVehicleBoxOccupied(const AD3& p3) const {
  if (p3[0] < x_min_ || p3[0] > x_max_ || p3[1] < y_min_ || p3[1] > y_max_)
    return true;

  const int off_x = (p3[0] - x_min_) / conf_.grid_step;
  const int off_y = (p3[1] - y_min_) / conf_.grid_step;
  double theta = WrapAngle(p3[2]);
  int idx = std::clamp<int>(theta / conf_.rectangle_heading_step + 0.5, 0,
                            rec_lib_.size() - 1);
  const auto& rec = rec_lib_[idx];
  const int minx = rec.x_min + off_x, miny = rec.y_min + off_y;
  const int maxx = rec.x_max + off_x, maxy = rec.y_max + off_y;
  if (minx < 0 || maxx >= static_cast<int>(grid_.size() - 1) || miny < 0 ||
      maxy >= static_cast<int>(grid_[0].size() - 1))
    return true;
  if (!(grid_[maxx + 1][maxy + 1] - grid_[maxx + 1][miny] -
        grid_[minx][maxy + 1] + grid_[minx][miny]))
    return false;

  for (auto [x1, x2, y] : rec.row_range) {
    const int xx1 = x1 + off_x, xx2 = x2 + off_x, yy = y + off_y;
    if (xx1 < 0 || xx1 >= static_cast<int>(grid_.size() - 1) || xx2 < 0 ||
        xx2 >= static_cast<int>(grid_.size() - 1) || yy < 0 ||
        yy >= static_cast<int>(grid_.front().size() - 1))
      return true;
    if (grid_[xx2 + 1][yy + 1] - grid_[xx2 + 1][yy] - grid_[xx1][yy + 1] +
        grid_[xx1][yy])
      return true;
  }

  return false;
}

std::array<double, 4> OccupyMap::GetRange() const {
  return {x_min_, y_min_, x_max_, y_max_};
}

double OccupyMap::grid_step() const { return conf_.grid_step; }

void OccupyMap::VisMap() const {
  if (!FLAGS_planning_enable_vis_event) return;
  auto fill_pt = [](auto t, auto x, auto y, auto z) {
    t->set_x(x);
    t->set_y(y);
    t->set_z(z);
  };
  auto event = vis::EventSender::Instance()->GetEvent("OccupyMap");
  event->set_type(neodrive::visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  event->mutable_color()->set_r(0);
  event->mutable_color()->set_g(1);
  event->mutable_color()->set_b(0);
  event->mutable_color()->set_a(0.5);

  auto polyline = event->add_polyline();
  /// row lines
  for (auto y = y_min_; y < y_max_; y += conf_.grid_step) {
    auto polyline = event->add_polyline();
    fill_pt(polyline->add_point(), x_min_, y, 0);
    fill_pt(polyline->add_point(), x_max_, y, 0);
  }
  /// col lines
  for (auto x = x_min_; x < x_max_; x += conf_.grid_step) {
    auto polyline = event->add_polyline();
    fill_pt(polyline->add_point(), x, y_min_, 0);
    fill_pt(polyline->add_point(), x, y_max_, 0);
  }

  /// occupy node
  const double step = conf_.grid_step;
  for (auto x = x_min_ + step / 2; x < x_max_; x += step) {
    for (auto y = y_min_ + step / 2; y < y_max_; y += step) {
      if (IsPointOccupied({x, y})) {
        auto sp = event->add_sphere();
        fill_pt(sp->mutable_center(), x, y, 0);
        sp->set_radius(step / 2);
      }
      // auto txt = event->add_text();
      // fill_pt(txt->mutable_position(), x, y, 0);
      // txt->set_text(std::to_string((GetPointMinDistanceToObstacle({x, y}))));
    }
  }
}

}  // namespace planning
}  // namespace neodrive
