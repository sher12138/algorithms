#pragma once

#include <math.h>

#ifdef MAX_ITER
#undef MAX_ITER
#endif
#ifdef RHO
#undef RHO
#endif

#include <cyber/time/time.h>

#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <random>

#include "src/planning/math/common/geometry.h"
#include "src/planning/math/common/kdtree.h"

namespace neodrive {
namespace planning {
namespace corridor {

void CorridorBuilder2d(const std::vector<Eigen::Vector3d> &state_list,
                       std::vector<math::Node<math::Point>> &obs_data,
                       const double add_dx, const double add_dy,
                       std::vector<Eigen::MatrixXd> &h_polys) {
  double create_kd_tree_time{0.}, search_time{0.}, convex_time{0.};

  auto time = cyber::Time::Now().ToSecond();
  math::KdTree<math::Node<math::Point>> kd_tree(obs_data);
  create_kd_tree_time += cyber::Time::Now().ToSecond() - time;

  double radius = std::sqrt(add_dx * add_dx + add_dy * add_dy);
  h_polys.resize(state_list.size());
  for (std::size_t i = 0; i < state_list.size(); i++) {
    /// Observe window
    std::vector<Eigen::Vector2d> add_obs{};
    Eigen::Matrix2d R;
    Eigen::Vector2d p;
    auto pos = state_list[i].head(2);
    auto yaw = state_list[i][2];
    R << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
    p = R * Eigen::Vector2d(add_dx, add_dy);
    add_obs.emplace_back(pos + p);
    p = R * Eigen::Vector2d(add_dx, -add_dy);
    add_obs.emplace_back(pos + p);
    p = R * Eigen::Vector2d(-add_dx, -add_dy);
    add_obs.emplace_back(pos + p);
    p = R * Eigen::Vector2d(-add_dx, add_dy);
    add_obs.emplace_back(pos + p);

    /// Obs data
    time = cyber::Time::Now().ToSecond();
    auto match_obs_pts =
        kd_tree.GetinRadiusof(math::Point(pos[0], pos[1]), radius);
    search_time += cyber::Time::Now().ToSecond() - time;

    /// Add data: new_data and flip_data
    time = cyber::Time::Now().ToSecond();
    float safe_radius = radius;
    Eigen::Matrix<double, 2, 1> data;
    std::vector<Eigen::Matrix<double, 2, 1>> new_data;
    cv::Point2f point;
    std::vector<cv::Point2f> flip_data;
    for (const auto &pt : match_obs_pts) {
      float dx = pt->shape.x() - pos[0];
      float dy = pt->shape.y() - pos[1];
      float norm2 = std::sqrt(dx * dx + dy * dy);
      safe_radius = std::fmin(safe_radius, norm2);
      if (norm2 < kMathEpsilon) continue;
      data << pt->shape.x(), pt->shape.y();
      new_data.push_back(data);
      point.x = dx + 2 * (radius - norm2) * dx / norm2;
      point.y = dy + 2 * (radius - norm2) * dy / norm2;
      flip_data.push_back(point);
    }
    for (std::size_t j = 0; j < add_obs.size(); j++) {
      float dx = add_obs[j](0) - pos(0);
      float dy = add_obs[j](1) - pos(1);
      float norm2 = std::sqrt(dx * dx + dy * dy);
      safe_radius = std::fmin(safe_radius, norm2);
      if (norm2 < kMathEpsilon) continue;
      point.x = dx + 2 * (radius - norm2) * dx / norm2;
      point.y = dy + 2 * (radius - norm2) * dy / norm2;
      new_data.push_back(add_obs[j]);
      flip_data.push_back(point);
    }

    /// Compute convex_hull
    std::vector<int> vertex_indice;
    cv::convexHull(flip_data, vertex_indice, false, false);

    bool is_origin_vertex = false;
    int origin_index = -1;
    std::vector<cv::Point2f> vertex_data;
    for (std::size_t j = 0; j < vertex_indice.size(); j++) {
      auto v = vertex_indice[j];
      if (v == new_data.size()) {
        is_origin_vertex = true;
        origin_index = j;
        vertex_data.push_back(cv::Point2f(pos[0], pos[1]));
      } else {
        vertex_data.push_back(cv::Point2f(new_data[v](0), new_data[v](1)));
      }
    }
    float interior_x{0.}, interior_y{0.};
    if (is_origin_vertex) {
      int last_index = (origin_index - 1) % vertex_indice.size();
      int next_index = (origin_index + 1) % vertex_indice.size();
      float dx = (new_data[vertex_indice[last_index]](0) + pos[0] +
                  new_data[vertex_indice[next_index]](0)) /
                     3 -
                 pos[0];
      float dy = (new_data[vertex_indice[last_index]](1) + pos[1] +
                  new_data[vertex_indice[next_index]](1)) /
                     3 -
                 pos[1];
      float d = std::sqrt(dx * dx + dy * dy);
      interior_x = 0.99 * safe_radius * dx / d + pos[0];
      interior_y = 0.99 * safe_radius * dy / d + pos[1];
    } else {
      interior_x = pos[0];
      interior_y = pos[1];
    }

    std::vector<int> v_index2;
    cv::convexHull(vertex_data, v_index2, false,
                   false);                     // counterclockwise right-hand
    std::vector<Eigen::Vector3f> constraints;  // (a,b,c) a x + b y <= c
    for (size_t j = 0; j < v_index2.size(); j++) {
      int jplus1 = (j + 1) % v_index2.size();
      cv::Point2f rayV =
          vertex_data[v_index2[jplus1]] - vertex_data[v_index2[j]];
      Eigen::Vector2f normalJ(rayV.y, -rayV.x);  // point to outside
      normalJ.normalize();
      int indexJ = v_index2[j];
      while (indexJ != v_index2[jplus1]) {
        float c = (vertex_data[indexJ].x - interior_x) * normalJ(0) +
                  (vertex_data[indexJ].y - interior_y) * normalJ(1);
        constraints.push_back(Eigen::Vector3f(normalJ(0), normalJ(1), c));
        indexJ = (indexJ + 1) % vertex_data.size();
      }
    }

    std::vector<cv::Point2f> dual_points(constraints.size(), cv::Point2f(0, 0));
    for (size_t j = 0; j < constraints.size(); j++) {
      dual_points[j].x = constraints[j](0) / constraints[j](2);
      dual_points[j].y = constraints[j](1) / constraints[j](2);
    }

    std::vector<cv::Point2f> dual_vertex, final_vertex;
    cv::convexHull(dual_points, dual_vertex, true, false);

    for (size_t j = 0; j < dual_vertex.size(); j++) {
      int iplus1 = (j + 1) % dual_vertex.size();
      cv::Point2f rayi = dual_vertex[iplus1] - dual_vertex[j];
      float c = rayi.y * dual_vertex[j].x - rayi.x * dual_vertex[j].y;
      final_vertex.push_back(
          cv::Point2f(interior_x + rayi.y / c, interior_y - rayi.x / c));
    }

    unsigned int size = final_vertex.size();
    h_polys[i].resize(4, size);
    for (unsigned int j = 0; j < size; j++) {
      int iplus1 = (j + 1) % size;
      cv::Point2f rayi = final_vertex[iplus1] - final_vertex[j];
      h_polys[i].col(j).tail<2>() = Eigen::Vector2d(
          final_vertex[j].x, final_vertex[j].y);  // the points on the plane
      h_polys[i].col(j).head<2>() =
          Eigen::Vector2d(-rayi.y, rayi.x);  // outside
    }

    convex_time += cyber::Time::Now().ToSecond() - time;
  }

  LOG_INFO("create_time, search_time, convex_time: {}, {}, {}",
           create_kd_tree_time, search_time, convex_time);
}

}  // namespace corridor
}  // namespace planning
}  // namespace neodrive
