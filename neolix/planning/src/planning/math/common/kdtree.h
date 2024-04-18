#pragma once

#include <functional>
#include <queue>
#include <type_traits>
#include <unordered_set>

#include "common/math/polygon2d.h"
#include "common/math/segment2d.h"
#include "common/math/vec2d.h"
#include "geometry.h"
#include "neolix_log.h"

namespace neodrive {
namespace planning {
namespace math {

/// Kd-tree node which is manifested as a pair of id and geometric shapes
/// @param id Id of the node
/// @param shape Geometric shapes including: Point, LineSegment, Polyline,
/// Polygon, Vec2d, Segment2d and Polygon2d
template <typename T>
struct Node final {
  int id{-1};
  T shape{};
};

struct PtrTag{};
struct NormalTag{};
template <typename T>
struct TagDispatchTrait {
  using Tag = T;
};
template <>
struct TagDispatchTrait<Polygon2d> {
  using Tag = Polygon;
};
template <>
struct TagDispatchTrait<Segment2d> {
  using Tag = LineSegment;
};
template <>
struct TagDispatchTrait<Vec2d> {
  using Tag = Point;
};


template <typename T>
struct TagDispatchTrait2 {
  using Tag = NormalTag;
};
template <typename T>
struct TagDispatchTrait2<T*> {
  using Tag = PtrTag;
};

/// @brief Quick method to generate a vector of nodes from a vector of geometric
/// shapes to create KdTree
/// @tparam T Geometric shapes
/// @param obj the vector of geometric shapes for objects
/// @param id the vector of their ids (optional)
/// @return the vector of Nodes to create KdTree
template <typename T>
std::vector<Node<typename TagDispatchTrait<T>::Tag>> GenerateNode(
    std::vector<T>& obj, std::vector<int> id = std::vector<int>(0)) {
  std::vector<Node<typename TagDispatchTrait<T>::Tag>> nodes;
  int len = obj.size();
  if (id.size() != len) {
    id.clear();
    for (auto i = 1; i <= len; i++) id.emplace_back(i);
  }
  GenerateNode(nodes, obj, id);
  return nodes;
}

template <typename T, typename T1>
void GenerateNode(std::vector<Node<T1>>& nodes, std::vector<T>& obj,
                  std::vector<int>& id) {
  int i = 0, len = obj.size();
  for (i = 0; i < len; i++) nodes.emplace_back(Node<T1>{id[i], T1(obj[i])});
}

template <typename T>
void GenerateNode(std::vector<Node<T>>& nodes, std::vector<T>& obj,
                  std::vector<int>& id) {
  int i = 0, len = obj.size();
  for (i = 0; i < len; i++) nodes.emplace_back(Node<T>{id[i], obj[i]});
}

/// @class hydrid KdTree
template <typename T>
class KdTree final {
 private:
  struct KNode {
    T node_{};
    std::shared_ptr<AaBox> boundary_;
    std::shared_ptr<KNode> left_;
    std::shared_ptr<KNode> right_;
    mutable bool vis_;
    mutable int cnt_;
  };

 public:
  KdTree(const KdTree&) = delete;
  KdTree& operator=(const KdTree&) = delete;

  ~KdTree() {
    std::function<void(std::shared_ptr<KNode>)> dfs =
        [&](std::shared_ptr<KNode> node) {
          node->boundary_.reset();
          if (node->left_) dfs(node->left_);
          if (node->right_) dfs(node->right_);
          node.reset();
        };
    if (root_) dfs(root_);
  }

 public:
  /// Constructor
  /// @return node Construct an empty tree
  KdTree() {
    root_ = nullptr;
    length_ = 0;
  }

  /// Constructor
  /// @tparam Node
  /// @param pts the vector of nodes
  KdTree(std::vector<T>& pts)
      : root_{Build(pts, 0, pts.size(), 0).first}, length_{pts.size()} {
    if (root_) root_->cnt_ = length_;
  }

  /// Constructor
  /// @tparam Node
  /// @param arr the array of nodes
  KdTree(const T* arr, const int len) {
    std::vector<T> pts;
    for (int i = 0; i < len; i++) pts.emplace_back(*(arr + i));
    root_ = Build(pts, 0, pts.size(), 0).first;
    if (root_)
      root_->cnt_ = length_ = len;
    else
      length_ = 0;
  }

  /// Rebuild the tree with an existed vector of nodes
  /// @tparam Node
  /// @param pts the vector of nodes
  template <typename T1>
  void CreateKDT(std::vector<T1>& pts) {
    CreateKDT(pts, typename TagDispatchTrait2<T1>::Tag{});
  }

  template <typename T1>
  void CreateKDT(std::vector<T1>& pts, NormalTag) {
    Clear();
    root_ = Build(pts, 0, pts.size(), 0).first;
    if (root_)
      root_->cnt_ = length_ = pts.size();
    else
      length_ = 0;
  }

  template <typename T1>
  void CreateKDT(std::vector<T1>& pts, PtrTag) {
    Clear();
    root_ = BuildWithPts(pts, 0, pts.size(), 0).first;
    if (root_)
      root_->cnt_ = length_ = pts.size();
    else
      length_ = 0;
  }

  /// Clear the KdTree
  void Clear() {
    std::function<void(std::shared_ptr<KNode>)> dfs =
        [&](std::shared_ptr<KNode> node) {
          node->boundary_.reset();
          if (node->left_) dfs(node->left_);
          if (node->right_) dfs(node->right_);
          node.reset();
        };
    if (root_) dfs(root_);
  }

  /// Interface to search nearest node of kdtree
  /// @tparam T1 Geometric shape of query object, i.e. Point, LineSegment,
  /// Polyline and X2d
  /// @param obj Query object with a geometric shape
  /// @return <ans, dis> a pair of the pointer of nearest node and distance
  /// between input query object and it
  template <typename T1>
  std::pair<const T*, double> GetNearestNodeOf(const T1& obj) const {
    const T* ans{nullptr};
    double min_err = std::numeric_limits<double>::infinity();
    if (root_) {
      GetNearestNodeOf(root_, obj, 0, &ans, &min_err);
    }
    return {ans, min_err};
  }

  /// Interface to search nodes in a circular area in kdtree
  /// @tparam T1 Geometric shape of query object, i.e. Point, LineSegment,
  /// Polyline and X2d.
  /// @param obj Query object with a geometric shape
  /// @param radius the radius of the circular search region
  /// @return the vector of query nodes
  template <typename T1>
  inline std::vector<const T*> GetinRadiusof(const T1& obj,
                                             const double radius) const {
    std::vector<const T*> ans{};
    if (root_) {
      if (!(std::is_same<typename std::decay<T>::type, Node<Point>>::value)) {
        Circle nt{AD2{obj.aabox().cen[0], obj.aabox().cen[1]}, radius};
        GetinRadiusof(root_, nt, 0, &ans);

        // LOG_INFO("\n\n");
      } else {
        GetinRadiusof(root_, obj, 0, &ans, radius);
      }
    }
    return ans;
  }

  /// Interface to query overlapping nodes of kdtree
  /// @tparam T1 Geometric shape of query object, i.e. Point, LineSegment,
  /// Polyline and X2d.
  /// @param obj Query object with at geometric shape
  /// @return the vector of overlapping nodes
  template <typename T1>
  std::vector<const T*> GetOverlapNodesOf(const T1& obj) const {
    std::vector<const T*> ans{};
    if (root_) GetOverlapNodesOf(root_, obj, &ans);
    return ans;
  }

  /// Interface to query overlapping nodes in this KdTree between itself and the
  /// other kdtree
  /// @tparam T1 Geometric shape of nodes in the query KdTree, i.e. Point,
  /// LineSegment, Polyline and X2d.
  /// @param tree Query KdTree
  /// @return the vector of overlapping nodes in this KdTree
  template <typename T1>
  std::vector<const T*> GetOverlapNodesWith(const KdTree<T1>& tree) {
    std::unordered_set<const T*> ans{};
    if (root_ && tree.root_) GetOverlapNodesWith(root_, tree.root_, 0, ans);
    return std::vector<const T*>(ans.begin(), ans.end());
  }

  [[deprecated("Use GetOverlapNodesWith(const KdTree<T1>&) instead")]] std::
      vector<const T*>
      GetOverlapNodesWith3(const KdTree& tree) {
    std::unordered_set<const T*> ans{};
    if (root_ && tree.root_) GetOverlapNodesWith2(root_, tree.root_, 0, &ans);
    return std::vector<const T*>(ans.begin(), ans.end());
  }

  const int& Length() { return length_; }

  /// Interface to collision relationship between the query object and this tree
  /// @tparam T1 Geometric shape of the query object, i.e. Point, LineSegment,
  /// Polyline and X2d.
  /// @param obj Query object
  /// @return whether obj collides with this tree
  template <typename T1>
  inline bool HasOverlapNodesOf(const T1& obj) const {
    return HasOverlapNodesOf(root_, obj);
  }

  /// Interface to query overlapping nodes in this KdTree and the other kdtree
  /// @tparam T1 Geometric shape of nodes in first KdTree, i.e. Point,
  /// LineSegment, Polyline and X2d
  /// @tparam T2 Geometric shape of nodes in second KdTree, i.e. Point,
  /// LineSegment, Polyline and X2d
  /// @param u First KdTree
  /// @param t Second KdTree
  /// @return the pair of two vector of overlapping nodes respectively in tree u
  /// and t
  template <typename T1, typename T2>
  friend std::pair<std::vector<const T1*>, std::vector<const T2*>>
  GetOverlapNodesBetween(KdTree<T1>& u, KdTree<T2>& t);

 private:
  template <typename T1>
  void GetNearestNodeOf(const std::shared_ptr<KNode> u, const T1& tar,
                        const size_t dim, const T** ans,
                        double* const min_err) const {
    if (!u || *min_err < 1e-3) {
      if (*min_err < 1e-3) *min_err = 0;
      return;
    }

    const auto& node = u->node_;
    if (auto dis = std::abs(Distance(tar, node.shape)); dis < *min_err) {
      *min_err = dis;
      *ans = &u->node_;
    }

    const double dir = tar.aabox().cen[dim] - node.shape.aabox().cen[dim];
    GetNearestNodeOf(dir > 0 ? u->right_ : u->left_, tar, dim ^ 1, ans,
                     min_err);
    if (std::is_same<typename std::decay<T>::type, Node<Point>>::value &&
        std::abs(dir) > *min_err)
      return;
    GetNearestNodeOf(dir > 0 ? u->left_ : u->right_, tar, dim ^ 1, ans,
                     min_err);
  }

  inline void GetinRadiusof(const std::shared_ptr<KNode> u, const Point& tar,
                            const size_t dim, std::vector<const T*>* const ans,
                            const double radius) const {
    if (!u) return;

    const auto& node = u->node_;
    if (auto dis = Distance(tar, node.shape); dis <= radius + 1E-7) {
      ans->emplace_back(&node);
    }

    const double dir = tar.aabox().cen[dim] - node.shape.aabox().cen[dim];
    GetinRadiusof(dir > 0 ? u->right_ : u->left_, tar, dim ^ 1, ans, radius);
    if (std::abs(dir) > radius) return;
    GetinRadiusof(dir > 0 ? u->left_ : u->right_, tar, dim ^ 1, ans, radius);
  }

  inline void GetinRadiusof(const std::shared_ptr<KNode> u, const Circle& tar,
                            const size_t dim,
                            std::vector<const T*>* const ans) const {
    GetOverlapNodesOf(u, tar, ans);
  }

  template <typename T1>
  inline void GetOverlapNodesOf(const std::shared_ptr<KNode> u, const T1& tar,
                                std::vector<const T*>* const ans) const {
    auto IsLeaf = [](auto& node) -> bool {
      return (!node->left_) && (!node->right_);
    };
    if (!u) return;

    if (IsLeaf(u)) {
      if (IsOverlaped(tar, u->node_.shape)) ans->emplace_back(&u->node_);
    } else {
      if (IsOverlaped(tar, *u->boundary_)) {
        if (IsOverlaped(tar, u->node_.shape)) ans->emplace_back(&u->node_);
        GetOverlapNodesOf(u->left_, tar, ans);
        GetOverlapNodesOf(u->right_, tar, ans);
      }
    }
  }

  template <typename T1, typename T2>
  inline bool HasOverlapNodesOf(const T2 u, const T1& tar) const {
    auto IsLeaf = [](auto& node) -> bool {
      return (!node->left_) && (!node->right_);
    };
    if (!u) return false;

    if (IsLeaf(u)) {
      return IsOverlaped(tar, u->node_.shape);
    } else {
      if (IsOverlaped(tar, *u->boundary_)) {
        if (IsOverlaped(tar, u->node_.shape)) return true;
        if (HasOverlapNodesOf(u->left_, tar)) return true;
        if (HasOverlapNodesOf(u->right_, tar)) return true;
      }
      return false;
    }
  }

  template <typename T1, typename T2, typename T3>
  void GetOverlapNodesWith(const T1 u, const T2 t, int reverse,
                           std::unordered_set<const T3*>& ans) const {
    auto aabox = [](auto p) -> const AaBox& { return p->node_.shape.aabox(); };
    if (!u || !t || !IsOverlaped(u->boundary_, t->boundary_)) return;
    if (HasOverlapNodesOf(t, u->node_.shape)) ans.insert(&u->node_);
    if (u->left_) GetOverlapNodesWith(u->left_, t, 0, ans);
    if (u->right_) GetOverlapNodesWith(u->right_, t, 0, ans);
  }

  void GetOverlapNodesWith2(const std::shared_ptr<KNode> u,
                            const std::shared_ptr<KNode> t, const size_t dim,
                            std::unordered_set<const T*>* const ans) const {
    auto aabox = [](auto p) -> const AaBox& { return p->node_.shape.aabox(); };
    if (!u || !t || !IsOverlaped(u->boundary_, t->boundary_)) return;
    // reverse = std::log2(u->cnt_+1)*t->cnt_ > std::log2(t->cnt_+1) ? 0 : 1; //
    // Identically vanishing
    if (HasOverlapNodesOf(t, u->node_.shape)) ans->insert(&u->node_);
    if (u->left_) GetOverlapNodesWith2(u->left_, t, dim ^ 1, ans);
    if (u->right_) GetOverlapNodesWith2(u->right_, t, dim ^ 1, ans);
  }

  template <typename T1, typename T2, typename T3>
  void GetOverlapNodesWith3(const T1 u, const T2 t, int reverse,
                            std::unordered_set<const T3*>& ans) const {
    auto getbranch = [](auto u, auto t, auto rev) -> std::pair<T1, T2> {
      if (rev)
        return {GetminBranch(t, u), t};
      else
        return {u, GetminBranch(u, t)};
    };

    auto aabox = [](auto p) -> const AaBox& { return p->node_.shape.aabox(); };
    if (!u || !t || !IsOverlaped(u->boundary_, t->boundary_)) return;
    auto [ubranch, tbranch] = getbranch(u, t, reverse);
    reverse = std::log2(ubranch->cnt_ + 1) * tbranch->cnt_ >
                      std::log2(tbranch->cnt_ + 1)
                  ? 0
                  : 1;

    if (!reverse) {
      if (HasOverlapNodesOf(tbranch, ubranch->node_.shape))
        ans.insert(&ubranch->node_);
      if (ubranch->left_) GetOverlapNodesWith(ubranch->left_, tbranch, 0, ans);
      if (ubranch->right_)
        GetOverlapNodesWith(ubranch->right_, tbranch, 0, ans);
    } else {
      std::vector<const T3*> reverseAns{};
      GetOverlapNodesOf(ubranch, tbranch->node_.shape, &reverseAns);
      for (auto rnode : reverseAns) ans.insert(rnode);
      if (tbranch->left_) GetOverlapNodesWith(ubranch, tbranch->left_, 1, ans);
      if (tbranch->right_)
        GetOverlapNodesWith(ubranch, tbranch->right_, 1, ans);
    }
  }

  std::pair<std::shared_ptr<KNode>, int> Build(std::vector<T>& elements,
                                               const size_t l, const size_t r,
                                               const size_t dim) {
    if (l >= r) return {nullptr, 0};

    struct NodeComp {
      int d{0};
      bool operator()(const T& a, const T& b) {
        return a.shape.aabox().cen[d] < b.shape.aabox().cen[d];
      }
    };
    auto mid = l + (r - l) / 2;
    auto it = elements.begin();
    std::nth_element(it + l, it + mid, it + r, NodeComp{dim});
    std::shared_ptr<KNode> ans(new KNode{
        elements[mid],
        std::shared_ptr<AaBox>(new AaBox(elements[mid].shape.aabox())), nullptr,
        nullptr, false, 1});
    node_map_.insert(std::make_pair(elements[mid].id, ans));
    for (auto i = l; i < r; ++i) {
      ans->boundary_->lb[0] =
          std::min(ans->boundary_->lb[0], elements[i].shape.aabox().lb[0]);
      ans->boundary_->lb[1] =
          std::min(ans->boundary_->lb[1], elements[i].shape.aabox().lb[1]);
      ans->boundary_->rt[0] =
          std::max(ans->boundary_->rt[0], elements[i].shape.aabox().rt[0]);
      ans->boundary_->rt[1] =
          std::max(ans->boundary_->rt[1], elements[i].shape.aabox().rt[1]);
    }
    auto [right, cntr] = Build(elements, mid + 1, r, dim ^ 1);
    auto [left, cntl] = Build(elements, l, mid, dim ^ 1);
    ans->right_ = right;
    ans->left_ = left;
    ans->cnt_ = cntr + cntl + 1;
    return {ans, cntr + cntl + 1};
  }

  std::pair<std::shared_ptr<KNode>, int> BuildWithPts(std::vector<Node<Polygon>*>& elements,
                                               const size_t l, const size_t r,
                                               const size_t dim) {
    if (l >= r) return {nullptr, 0};

    struct NodeComp {
      int d{0};
      bool operator()(const Node<Polygon>* a, const Node<Polygon>* b) {
        return a->shape.aabox().cen[d] < b->shape.aabox().cen[d];
      }
    };
    auto mid = l + (r - l) / 2;
    auto it = elements.begin();
    std::nth_element(it + l, it + mid, it + r, NodeComp{dim});
    std::shared_ptr<KNode> ans(new KNode{
        *elements[mid],
        std::shared_ptr<AaBox>(new AaBox(elements[mid]->shape.aabox())), nullptr,
        nullptr, false, 1});
    node_map_.insert(std::make_pair(elements[mid]->id, ans));
    for (auto i = l; i < r; ++i) {
      ans->boundary_->lb[0] =
          std::min(ans->boundary_->lb[0], elements[i]->shape.aabox().lb[0]);
      ans->boundary_->lb[1] =
          std::min(ans->boundary_->lb[1], elements[i]->shape.aabox().lb[1]);
      ans->boundary_->rt[0] =
          std::max(ans->boundary_->rt[0], elements[i]->shape.aabox().rt[0]);
      ans->boundary_->rt[1] =
          std::max(ans->boundary_->rt[1], elements[i]->shape.aabox().rt[1]);
    }
    auto [right, cntr] = BuildWithPts(elements, mid + 1, r, dim ^ 1);
    auto [left, cntl] = BuildWithPts(elements, l, mid, dim ^ 1);
    ans->right_ = right;
    ans->left_ = left;
    ans->cnt_ = cntr + cntl + 1;
    return {ans, cntr + cntl + 1};
  }

  template <typename T1, typename T2>
  friend T2 GetminBranch(const T1 t, const T2 branch);

 public:
  std::map<int, std::weak_ptr<KNode>> node_map_{};
 private:
  std::shared_ptr<KNode> root_{nullptr};
  int length_;
  template <typename>
  friend class KdTree;
};

template <typename T1, typename T2>
std::pair<std::vector<const T1*>, std::vector<const T2*>>
GetOverlapNodesBetween(KdTree<T1>& u, KdTree<T2>& t) {
  std::vector<const T1*> ansT1 = u.GetOverlapNodesWith(t);
  std::vector<const T2*> ansT2 = t.GetOverlapNodesWith(u);

  return {ansT1, ansT2};
}

template <typename T1, typename T2>
T2 GetminBranch(const T1 t, const T2 branch) {
  if (!branch) return nullptr;

  auto& tBoundary = t->boundary_;
  std::queue<T2> que;
  que.push(branch);
  while (!que.empty()) {
    auto currentRoot = que.front();
    que.pop();
    std::array<bool, 3> state{currentRoot->vis_, false, false};
    if (currentRoot->left_ &&
        IsOverlaped(tBoundary, *currentRoot->left_->boundary_))
      state[1] = true;
    if (currentRoot->right_ &&
        IsOverlaped(tBoundary, *currentRoot->right_->boundary_))
      state[2] = true;
    if (state[1] ^ state[2])
      que.push(state[1] > state[2] ? currentRoot->left_ : currentRoot->right_);
    else
      return currentRoot;
  }
}

}  // namespace math
}  // namespace planning
}  // namespace neodrive
