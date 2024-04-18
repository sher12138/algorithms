#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include "neolix_log.h"
#include "reference_line.h"

namespace neodrive {
namespace planning {
class ReferencelineKdTreeNode {
 public:
  using RefPtPtr = const ReferencePoint *;

  ReferencelineKdTreeNode(const std::vector<RefPtPtr> &objects,
                          double max_leaf_dimension = 5.0) {
    CHECK(!objects.empty());

    compute_boundary(objects);
    compute_partition();

    if (split_to_subnodes(objects, max_leaf_dimension)) {
      std::vector<RefPtPtr> left_subnode_objects;
      std::vector<RefPtPtr> right_subnode_objects;
      partition_objects(objects, &left_subnode_objects, &right_subnode_objects);

      // Split to sub-nodes.
      if (!left_subnode_objects.empty()) {
        _left_subnode.reset(new ReferencelineKdTreeNode(left_subnode_objects));
      }
      if (!right_subnode_objects.empty()) {
        _right_subnode.reset(
            new ReferencelineKdTreeNode(right_subnode_objects));
      }
    } else {
      init_objects(objects);
    }
  }

  RefPtPtr get_nearest_object(const Vec2d &point) const {
    RefPtPtr nearest_object = nullptr;
    double min_distance_sqr = std::numeric_limits<double>::infinity();
    get_nearest_object_internal(point, &min_distance_sqr, &nearest_object);
    return nearest_object;
  }

  std::vector<RefPtPtr> get_objects(const Vec2d &point,
                                    const double distance) const {
    std::vector<RefPtPtr> result_objects;
    get_objects_internal(point, distance, std::pow(distance, 2),
                         &result_objects);
    return result_objects;
  }

  AABox2d get_bounding_box() const {
    return AABox2d({_min_x, _min_y}, {_max_x, _max_y});
  }

 private:
  void init_objects(const std::vector<RefPtPtr> &objects) {
    _num_objects = objects.size();
    _objects_sorted_by_min = objects;
    _objects_sorted_by_max = objects;
    std::sort(_objects_sorted_by_min.begin(), _objects_sorted_by_min.end(),
              [&](RefPtPtr obj1, RefPtPtr obj2) {
                return _partition == PARTITION_X ? obj1->x() < obj2->x()
                                                 : obj1->y() < obj2->y();
              });
    std::sort(_objects_sorted_by_max.begin(), _objects_sorted_by_max.end(),
              [&](RefPtPtr obj1, RefPtPtr obj2) {
                return _partition == PARTITION_X ? obj1->x() > obj2->x()
                                                 : obj1->y() > obj2->y();
              });
    _objects_sorted_by_min_bound.reserve(_num_objects);
    for (RefPtPtr object : _objects_sorted_by_min) {
      _objects_sorted_by_min_bound.push_back(
          _partition == PARTITION_X ? object->x() : object->y());
    }
    _objects_sorted_by_max_bound.reserve(_num_objects);
    for (RefPtPtr object : _objects_sorted_by_max) {
      _objects_sorted_by_max_bound.push_back(
          _partition == PARTITION_X ? object->x() : object->y());
    }
  }

  bool split_to_subnodes(const std::vector<RefPtPtr> &objects,
                         const double max_leaf_dimension) {
    if (max_leaf_dimension >= 0.0 &&
        std::max(_max_x - _min_x, _max_y - _min_y) <= max_leaf_dimension) {
      return false;
    }
    return true;
  }

  double lowerbound_distance_sqr_to_point(const Vec2d &point) const {
    double dx = 0.0;
    if (point.x() < _min_x) {
      dx = _min_x - point.x();
    } else if (point.x() > _max_x) {
      dx = point.x() - _max_x;
    }
    double dy = 0.0;
    if (point.y() < _min_y) {
      dy = _min_y - point.y();
    } else if (point.y() > _max_y) {
      dy = point.y() - _max_y;
    }
    return dx * dx + dy * dy;
  }

  double upperbound_distance_sqr_to_point(const Vec2d &point) const {
    const double dx =
        (point.x() > _mid_x ? (point.x() - _min_x) : (point.x() - _max_x));
    const double dy =
        (point.y() > _mid_y ? (point.y() - _min_y) : (point.y() - _max_y));
    return dx * dx + dy * dy;
  }

  void get_all_objects(std::vector<RefPtPtr> *const result_objects) const {
    result_objects->insert(result_objects->end(),
                           _objects_sorted_by_min.begin(),
                           _objects_sorted_by_min.end());
    if (_left_subnode != nullptr) {
      _left_subnode->get_all_objects(result_objects);
    }
    if (_right_subnode != nullptr) {
      _right_subnode->get_all_objects(result_objects);
    }
  }

  void get_objects_internal(const Vec2d &point, const double distance,
                            const double distance_sqr,
                            std::vector<RefPtPtr> *const result_objects) const {
    if (lowerbound_distance_sqr_to_point(point) > distance_sqr) {
      return;
    }
    if (upperbound_distance_sqr_to_point(point) <= distance_sqr) {
      get_all_objects(result_objects);
      return;
    }
    const double pvalue = (_partition == PARTITION_X ? point.x() : point.y());
    if (pvalue < _partition_position) {
      const double limit = pvalue + distance;
      for (int i = 0; i < _num_objects; ++i) {
        if (_objects_sorted_by_min_bound[i] > limit) {
          break;
        }
        RefPtPtr object = _objects_sorted_by_min[i];
        if (object->distance_sqr_to(point) <= distance_sqr) {
          result_objects->push_back(object);
        }
      }
    } else {
      const double limit = pvalue - distance;
      for (int i = 0; i < _num_objects; ++i) {
        if (_objects_sorted_by_max_bound[i] < limit) {
          break;
        }
        RefPtPtr object = _objects_sorted_by_max[i];
        if (object->distance_sqr_to(point) <= distance_sqr) {
          result_objects->push_back(object);
        }
      }
    }
    if (_left_subnode != nullptr) {
      _left_subnode->get_objects_internal(point, distance, distance_sqr,
                                          result_objects);
    }
    if (_right_subnode != nullptr) {
      _right_subnode->get_objects_internal(point, distance, distance_sqr,
                                           result_objects);
    }
  }

  void get_nearest_object_internal(const Vec2d &point,
                                   double *const min_distance_sqr,
                                   RefPtPtr *const nearest_object) const {
    if (lowerbound_distance_sqr_to_point(point) >=
        *min_distance_sqr - kMathEpsilon) {
      return;
    }
    const double pvalue = (_partition == PARTITION_X ? point.x() : point.y());
    const bool search_left_first = (pvalue < _partition_position);
    if (search_left_first) {
      if (_left_subnode != nullptr) {
        _left_subnode->get_nearest_object_internal(point, min_distance_sqr,
                                                   nearest_object);
      }
    } else {
      if (_right_subnode != nullptr) {
        _right_subnode->get_nearest_object_internal(point, min_distance_sqr,
                                                    nearest_object);
      }
    }
    if (*min_distance_sqr <= kMathEpsilon) {
      return;
    }

    if (search_left_first) {
      for (int i = 0; i < _num_objects; ++i) {
        const double bound = _objects_sorted_by_min_bound[i];
        if (bound > pvalue && std::pow(bound - pvalue, 2) > *min_distance_sqr) {
          break;
        }
        RefPtPtr object = _objects_sorted_by_min[i];
        const double distance_sqr = object->distance_sqr_to(point);
        if (distance_sqr < *min_distance_sqr) {
          *min_distance_sqr = distance_sqr;
          *nearest_object = object;
        }
      }
    } else {
      for (int i = 0; i < _num_objects; ++i) {
        const double bound = _objects_sorted_by_max_bound[i];
        if (bound < pvalue && std::pow(bound - pvalue, 2) > *min_distance_sqr) {
          break;
        }
        RefPtPtr object = _objects_sorted_by_max[i];
        const double distance_sqr = object->distance_sqr_to(point);
        if (distance_sqr < *min_distance_sqr) {
          *min_distance_sqr = distance_sqr;
          *nearest_object = object;
        }
      }
    }
    if (*min_distance_sqr <= kMathEpsilon) {
      return;
    }
    if (search_left_first) {
      if (_right_subnode != nullptr) {
        _right_subnode->get_nearest_object_internal(point, min_distance_sqr,
                                                    nearest_object);
      }
    } else {
      if (_left_subnode != nullptr) {
        _left_subnode->get_nearest_object_internal(point, min_distance_sqr,
                                                   nearest_object);
      }
    }
  }

  void compute_boundary(const std::vector<RefPtPtr> &objects) {
    _min_x = std::numeric_limits<double>::infinity();
    _min_y = std::numeric_limits<double>::infinity();
    _max_x = -std::numeric_limits<double>::infinity();
    _max_y = -std::numeric_limits<double>::infinity();
    for (RefPtPtr object : objects) {
      _min_x = std::min(_min_x, object->x());
      _max_x = std::max(_max_x, object->x());
      _min_y = std::min(_min_y, object->y());
      _max_y = std::max(_max_y, object->y());
    }
    _mid_x = (_min_x + _max_x) / 2.0;
    _mid_y = (_min_y + _max_y) / 2.0;
  }

  void compute_partition() {
    if (_max_x - _min_x >= _max_y - _min_y) {
      _partition = PARTITION_X;
      _partition_position = (_min_x + _max_x) / 2.0;
    } else {
      _partition = PARTITION_Y;
      _partition_position = (_min_y + _max_y) / 2.0;
    }
  }

  void partition_objects(const std::vector<RefPtPtr> &objects,
                         std::vector<RefPtPtr> *const left_subnode_objects,
                         std::vector<RefPtPtr> *const right_subnode_objects) {
    left_subnode_objects->clear();
    right_subnode_objects->clear();
    std::vector<RefPtPtr> other_objects;
    if (_partition == PARTITION_X) {
      for (RefPtPtr object : objects) {
        if (object->x() <= _partition_position) {
          left_subnode_objects->push_back(object);
        } else if (object->x() >= _partition_position) {
          right_subnode_objects->push_back(object);
        } else {
          other_objects.push_back(object);
        }
      }
    } else {
      for (RefPtPtr object : objects) {
        if (object->y() <= _partition_position) {
          left_subnode_objects->push_back(object);
        } else if (object->y() >= _partition_position) {
          right_subnode_objects->push_back(object);
        } else {
          other_objects.push_back(object);
        }
      }
    }
    init_objects(other_objects);
  }

 private:
  int _num_objects = 0;
  std::vector<RefPtPtr> _objects_sorted_by_min;
  std::vector<RefPtPtr> _objects_sorted_by_max;
  std::vector<double> _objects_sorted_by_min_bound;
  std::vector<double> _objects_sorted_by_max_bound;

  // Boundary
  double _min_x = 0.0;
  double _max_x = 0.0;
  double _min_y = 0.0;
  double _max_y = 0.0;
  double _mid_x = 0.0;
  double _mid_y = 0.0;

  enum Partition {
    PARTITION_X = 1,
    PARTITION_Y = 2,
  };
  Partition _partition = PARTITION_X;
  double _partition_position = 0.0;

  std::unique_ptr<ReferencelineKdTreeNode> _left_subnode = nullptr;
  std::unique_ptr<ReferencelineKdTreeNode> _right_subnode = nullptr;
};

class ReferencelineKdTree {
 public:
  using RefPtPtr = const ReferencePoint *;

  ReferencelineKdTree(const std::vector<ReferencePoint> &objects) {
    if (!objects.empty()) {
      firt_addr_ = &objects[0];
      std::vector<RefPtPtr> object_ptrs;
      object_ptrs.reserve(objects.size());
      for (const auto &object : objects) {
        object_ptrs.push_back(&object);
      }
      _root.reset(new ReferencelineKdTreeNode(object_ptrs));
    }
  }

  const ReferencePoint *see_first_ptr() { return firt_addr_; }

  RefPtPtr get_nearest_object(const Vec2d &point) const {
    return _root == nullptr ? nullptr : _root->get_nearest_object(point);
  }

  std::vector<RefPtPtr> get_objects(const Vec2d &point,
                                    const double distance) const {
    if (_root == nullptr) {
      LOG_ERROR("root == nullptr");
      return {};
    }
    return _root->get_objects(point, distance);
  }

 private:
  std::unique_ptr<ReferencelineKdTreeNode> _root = nullptr;
  const ReferencePoint *firt_addr_ = nullptr;
};

}  // namespace planning
}  // namespace neodrive
