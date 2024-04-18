#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <set>
#include <vector>

#include "common/math/polygon2d.h"
#include "common/math/segment2d.h"
#include "common/math/vec2d.h"

namespace neodrive {
namespace planning {
namespace math {

using AD2 = std::array<double, 2>;
inline AD2 operator-(const AD2& p1, const AD2& p2) {
  return AD2{p1[0] - p2[0], p1[1] - p2[1]};
}
inline AD2 operator+(const AD2& p1, const AD2& p2) {
  return AD2{p1[0] + p2[0], p1[1] + p2[1]};
}
inline double Dot(const AD2& p1, const AD2& p2) {
  return p1[0] * p2[0] + p1[1] * p2[1];
}
inline double Cross(const AD2& p1, const AD2& p2) {
  return p1[0] * p2[1] - p1[1] * p2[0];
}
inline double Length(const AD2& p) { return std::hypot(p[0], p[1]); }

template <typename L, typename R>
std::vector<R> cast(const std::vector<L>& left, std::vector<R>*) {
  std::vector<R> right;
  std::transform(left.begin(), left.end(),
                 std::insert_iterator<std::vector<R>>(right, right.begin()),
                 [](L l) -> R { return (R)l; });
  return std::move(right);
}

struct AaBox {
  AD2 lb{0, 0}, rt{0, 0}, cen{0, 0};
};

static AaBox CreateAaBox(const std::vector<AD2>& pts) {
  if (pts.empty()) return AaBox{};
  auto ltx = [](auto& a, auto& b) { return a[0] < b[0]; };
  auto lty = [](auto& a, auto& b) { return a[1] < b[1]; };
  const double minx = std::min_element(pts.begin(), pts.end(), ltx)->at(0);
  const double maxx = std::max_element(pts.begin(), pts.end(), ltx)->at(0);
  const double miny = std::min_element(pts.begin(), pts.end(), lty)->at(1);
  const double maxy = std::max_element(pts.begin(), pts.end(), lty)->at(1);

  return AaBox{
      {minx, miny}, {maxx, maxy}, {(minx + maxx) / 2, (miny + maxy) / 2}};
};

/// @class Define the point 2d
class Point {
 public:
  explicit Point(const double px, const double py)
      : x_{px}, y_{py}, aabox_{CreateAaBox({{px, py}})} {}

  explicit Point(const AD2& p1)
      : x_{p1[0]}, y_{p1[1]}, aabox_{CreateAaBox({{p1[0], p1[1]}})} {}

  explicit Point(const Vec2d& vec)
      : x_{vec.x()}, y_{vec.y()}, aabox_{CreateAaBox({{x_, y_}})} {}

  inline operator AD2() const { return {x_, y_}; }
  inline const double& x() const { return x_; }
  inline const double& y() const { return y_; }
  inline const AaBox& aabox() const { return aabox_; }

 private:
  double x_{0};
  double y_{0};
  AaBox aabox_{};
};

/// @class Define the 2d line segment
struct LineSegment {
 public:
  explicit LineSegment(const AD2& p1, const AD2& p2)
      : start_{p1}, end_{p2}, aabox_{CreateAaBox({p1, p2})} {}

  explicit LineSegment(const Segment2d& s)
      : start_{{s.start().x(), s.start().y()}},
        end_{{s.end().x(), s.end().y()}},
        aabox_{CreateAaBox({start_, end_})} {}

  inline const AD2& start() const { return start_; }
  inline const AD2& end() const { return end_; }
  inline const AaBox& aabox() const { return aabox_; }

 private:
  AD2 start_{0, 0};
  AD2 end_{0, 0};
  AaBox aabox_{};
};

/// @class Define the 2d polyline
class Polyline {
 public:
  explicit Polyline(const std::vector<AD2>& pts)
      : points_{pts}, aabox_{CreateAaBox(pts)} {
    for (size_t i = 1, len = points_.size(); i < len; ++i) {
      line_segs_.emplace_back(points_[i - 1], points_[i]);
    }
  }

  inline const std::vector<AD2>& points() const { return points_; }
  inline const std::vector<LineSegment>& line_segs() const {
    return line_segs_;
  }
  inline const AaBox& aabox() const { return aabox_; }

 private:
  std::vector<AD2> points_{};
  AaBox aabox_{};
  std::vector<LineSegment> line_segs_{};
};

/// @class Expand the 2d polygon from polygon2d
class Polygon {
 public:
  Polygon() {}
  explicit Polygon(const std::vector<AD2>& pts)
      : points_{pts}, aabox_{CreateAaBox(pts)} {
    for (size_t i = 0, len = points_.size(); i < len; ++i) {
      line_segs_.emplace_back(points_[i], points_[(i + 1) % len]);
    }
  }

  explicit Polygon(const Polygon2d& polygon) {
    size_t i = 0, len = polygon.points().size();
    points_.resize(len);
    for (const auto& pt : polygon.points()) {
      points_[i++] = {pt.x(), pt.y()};
    }
    aabox_ = std::move(CreateAaBox(points_));
    for (i = 0; i < len; i++)
      line_segs_.emplace_back(points_[i], points_[(i + 1) % len]);
  }

  inline const std::vector<AD2>& points() const { return points_; }
  inline const std::vector<LineSegment>& line_segs() const {
    return line_segs_;
  }
  inline const AaBox& aabox() const { return aabox_; }

 private:
  std::vector<AD2> points_{};
  AaBox aabox_{};
  std::vector<LineSegment> line_segs_{};
};

class Circle {
 public:
  explicit Circle(const AD2 c, const double r)
      : center_{c},
        radius_{r},
        aabox_{CreateAaBox({{c[0] - r, c[1] - r}, {c[0] + r, c[1] + r}})} {}

  inline const AD2& center() const { return center_; }
  inline const double& radius() const { return radius_; }
  inline const AaBox& aabox() const { return aabox_; }

 private:
  AD2 center_{0, 0};
  double radius_;
  AaBox aabox_{};
};

/// Interface for overlap check
template <typename T1, typename T2>
bool IsOverlaped(const T1&, const T2&) {
  // assert(0);
  return true;
}

template <>
inline bool IsOverlaped(const AaBox& b1, const AaBox& b2) {
  return b1.rt[0] >= b2.lb[0] - 1e-5 && b2.rt[0] >= b1.lb[0] - 1e-5 &&
         b1.rt[1] >= b2.lb[1] - 1e-5 && b2.rt[1] >= b1.lb[1] - 1e-5;
}

template <>
inline bool IsOverlaped(const Point& p1, const Point& p2) {
  return Length({p1.x() - p2.x(), p1.y() - p2.y()}) < 1e-4;
}

template <>
inline bool IsOverlaped(const AD2& p1, const AD2& p2) {
  return Length(p1 - p2) < 1e-4;
}

inline int Sign(const double x, double eps = 1e-3) {
  return x < -eps ? -1 : x > eps ? 1 : 0;
}

inline int Sign(const double x, double low, double high) {
  return x < low ? -1 : x > high ? 1 : 0;
}

template <>
inline bool IsOverlaped(const Point& p, const LineSegment& l) {
  if (IsOverlaped(p, Point(l.start())) || IsOverlaped(p, Point(l.end())))
    return true;
  return Sign(Cross(l.start() - p, l.end() - p)) == 0 &&
         Sign(Dot(l.start() - p, l.end() - p)) <= 0;
}
template <>
inline bool IsOverlaped(const LineSegment& l, const Point& p) {
  return IsOverlaped(p, l);
}

template <>
inline bool IsOverlaped(const Point& p, const AaBox& box) {
  return p.x() >= box.lb[0] - 1e-5 && box.rt[0] >= p.x() - 1e-5 &&
         p.y() >= box.lb[1] - 1e-5 && box.rt[1] >= p.y() - 1e-5;
}

template <>
inline bool IsOverlaped(const Point& p, const Polyline& pl) {
  if (!IsOverlaped(p, pl.aabox())) return false;

  for (auto& l : pl.line_segs())
    if (IsOverlaped(p, l)) return true;
  return false;
}
template <>
inline bool IsOverlaped(const Polyline& pl, const Point& p) {
  return IsOverlaped(p, pl);
}

template <>
inline bool IsOverlaped(const Point& p, const Polygon& pg) {
  if (!IsOverlaped(p, pg.aabox())) return false;

  for (auto& l : pg.line_segs())
    if (IsOverlaped(p, l)) return true;

  bool f{false};
  for (auto& l : pg.line_segs()) {
    auto p0 = l.start(), p1 = l.end();
    if ((p0[1] > p.y()) != (p1[1] > p.y())) {
      auto side = Cross(p1 - p0, {p.x() - p0[0], p.y() - p0[1]});
      f ^= p0[1] < p1[1] ? side > 0 : side < 0;
    }
  }

  return f;
}

template <>
inline bool IsOverlaped(const Polygon& pg, const Point& p) {
  return IsOverlaped(p, pg);
}

template <>
inline bool IsOverlaped(const LineSegment& l1, const LineSegment& l2) {
  if (!IsOverlaped(l1.aabox(), l2.aabox())) return false;
  if (IsOverlaped(Point(l1.start()), l2) || IsOverlaped(Point(l1.end()), l2) ||
      IsOverlaped(Point(l2.start()), l1) || IsOverlaped(Point(l2.end()), l1))
    return true;

  return Sign(Cross(l1.end() - l1.start(), l2.start() - l1.start())) *
                 Sign(Cross(l1.end() - l1.start(), l2.end() - l1.start())) <
             0 &&
         Sign(Cross(l2.end() - l2.start(), l1.start() - l2.start())) *
                 Sign(Cross(l2.end() - l2.start(), l1.end() - l2.start())) <
             0;
}

template <>
inline bool IsOverlaped(const LineSegment& l, const Polyline& pl) {
  if (!IsOverlaped(l.aabox(), pl.aabox())) return false;

  for (auto& l1 : pl.line_segs())
    if (IsOverlaped(l, l1)) return true;
  return false;
}
template <>
inline bool IsOverlaped(const Polyline& pl, const LineSegment& l) {
  return IsOverlaped(l, pl);
}

template <>
inline bool IsOverlaped(const LineSegment& l, const Polygon& pg) {
  if (!IsOverlaped(l.aabox(), pg.aabox())) return false;

  if (IsOverlaped(Point(l.start()), pg) || IsOverlaped(Point(l.end()), pg))
    return true;

  for (auto& l1 : pg.line_segs())
    if (IsOverlaped(l, l1)) return true;
  return false;
}

template <>
inline bool IsOverlaped(const Polygon& pg, const LineSegment& l) {
  return IsOverlaped(l, pg);
}

template <>
inline bool IsOverlaped(const Polyline& pl1, const Polyline& pl2) {
  if (!IsOverlaped(pl1.aabox(), pl2.aabox())) return false;

  for (auto& l : pl1.line_segs())
    if (IsOverlaped(l, pl2)) return true;
  return false;
}

template <>
inline bool IsOverlaped(const Polyline& pl, const Polygon& pg) {
  if (!IsOverlaped(pl.aabox(), pg.aabox())) return false;

  for (auto& l : pl.line_segs())
    if (IsOverlaped(l, pg)) return true;
  return false;
}
template <>
inline bool IsOverlaped(const Polygon& pg, const Polyline& p) {
  return IsOverlaped(p, pg);
}

template <>
inline bool IsOverlaped(const Polygon& pg1, const Polygon& pg2) {
  if (!IsOverlaped(pg1.aabox(), pg2.aabox())) return false;

  for (auto& l : pg1.line_segs())
    if (IsOverlaped(l, pg2)) return true;
  for (auto& l : pg2.line_segs())
    if (IsOverlaped(l, pg1)) return true;
  return false;
}

/// Interface for Distance
template <typename T1, typename T2>
double Distance(const T1&, const T2&) {
  assert(0);
  return 0;
}

template <>
inline double Distance(const AD2& p1, const AD2& p2) {
  return Length({p1[0] - p2[0], p1[1] - p2[1]});
}

template <>
inline double Distance(const Point& p1, const Point& p2) {
  return Length({p1.x() - p2.x(), p1.y() - p2.y()});
}

template <>
inline double Distance(const Point& p, const LineSegment& l) {
  AD2 p0 = {p.x(), p.y()}, v1 = l.end() - l.start(), v2 = p0 - l.start(),
      v3 = p0 - l.end();
  if (Dot(v1, v2) < 0) return Length(v2);
  if (Dot(v1, v3) > 0) return Length(v3);
  return std::abs(Cross(v1, v2) / Length(v1));
}
template <>
inline double Distance(const LineSegment& l, const Point& p) {
  return Distance(p, l);
}

template <>
inline double Distance(const Point& p, const Polyline& pl) {
  double ans = std::numeric_limits<double>::infinity();
  for (auto& l : pl.line_segs()) ans = std::min(ans, Distance(p, l));
  return ans;
}
template <>
inline double Distance(const Polyline& pl, const Point& p) {
  return Distance(p, pl);
}

template <>
inline double Distance(const Point& p, const Polygon& pg) {
  if (IsOverlaped(p, pg)) return 0;

  double ans = std::numeric_limits<double>::infinity();
  for (auto& l : pg.line_segs()) ans = std::min(ans, Distance(p, l));
  return ans;
}
template <>
inline double Distance(const Polygon& pg, const Point& p) {
  return Distance(p, pg);
}

template <>
inline double Distance(const LineSegment& l1, const LineSegment& l2) {
  if (IsOverlaped(l1, l2)) return 0;

  return std::min(
      {Distance(Point(l1.start()), l2), Distance(Point(l1.end()), l2),
       Distance(Point(l2.start()), l1), Distance(Point(l2.end()), l1)});
}

template <>
inline double Distance(const LineSegment& l, const Polyline& pl) {
  if (IsOverlaped(l, pl)) return 0;

  double ans = std::numeric_limits<double>::infinity();
  for (auto& l1 : pl.line_segs()) ans = std::min(ans, Distance(l, l1));
  return ans;
}
template <>
inline double Distance(const Polyline& pl, const LineSegment& l) {
  return Distance(l, pl);
}

template <>
inline double Distance(const LineSegment& l, const Polygon& pg) {
  if (IsOverlaped(l, pg)) return 0;

  double ans = std::numeric_limits<double>::infinity();
  for (auto& l1 : pg.line_segs()) ans = std::min(ans, Distance(l, l1));
  return ans;
}
template <>
inline double Distance(const Polygon& pg, const LineSegment& l) {
  return Distance(l, pg);
}

template <>
inline double Distance(const Polyline& pl1, const Polyline& pl2) {
  if (IsOverlaped(pl1, pl2)) return 0;

  double ans = std::numeric_limits<double>::infinity();
  for (auto& l : pl1.line_segs()) ans = std::min(ans, Distance(l, pl2));
  return ans;
}

template <>
inline double Distance(const Polyline& pl, const Polygon& pg) {
  if (IsOverlaped(pl, pg)) return 0;

  double ans = std::numeric_limits<double>::infinity();
  for (auto& l : pl.line_segs()) ans = std::min(ans, Distance(l, pg));
  return ans;
}
template <>
inline double Distance(const Polygon& pg, const Polyline& pl) {
  return Distance(pl, pg);
}

template <>
inline double Distance(const Polygon& pg1, const Polygon& pg2) {
  if (IsOverlaped(pg1, pg2)) return 0;

  double ans = std::numeric_limits<double>::infinity();
  for (auto& l : pg1.line_segs()) ans = std::min(ans, Distance(l, pg2));
  return ans;
}

// Additional interface of Overlap check
template <>
inline bool IsOverlaped(const Circle& cr, const Polygon& pg) {
  if (!IsOverlaped(cr.aabox(), pg.aabox())) return false;

  Point cr_cen{cr.center()};
  for (auto& l : pg.line_segs()) {
    auto dis = Distance(cr_cen, l);
    if (Sign(cr.radius() - dis) >= 0) return true;
  }
  if (IsOverlaped(cr_cen, pg)) return true;
  return false;
}

template <>
inline bool IsOverlaped(const Polygon& pg, const Circle& cr) {
  return IsOverlaped(cr, pg);
}

template <>
inline bool IsOverlaped(const Circle& cr, const AaBox& pg) {
  if (!IsOverlaped(cr.aabox(), pg)) return false;
  auto cr_cen = cr.center();
  double vx =
      abs(cr_cen[0] - (pg.lb[0] + pg.rt[0]) / 2) - (pg.rt[0] - pg.lb[0]) / 2;
  double vy =
      abs(cr_cen[1] - (pg.lb[1] + pg.rt[1]) / 2) - (pg.rt[1] - pg.lb[1]) / 2;

  vx = vx > 0 ? vx : 0;
  vy = vy > 0 ? vy : 0;

  return vx * vx + vy * vy <= cr.radius() * cr.radius();
}

template <>
inline bool IsOverlaped(const AaBox& pg, const Circle& cr) {
  return IsOverlaped(cr, pg);
}

template <>
inline bool IsOverlaped(const Circle& cr, const Polyline& pl) {
  if (!IsOverlaped(cr.aabox(), pl.aabox())) return false;

  Point cr_cen{cr.center()};
  for (auto& l : pl.line_segs()) {
    auto dis = Distance(cr_cen, l);
    if (dis <= cr.radius()) return true;
  }
  return false;
}

template <>
inline bool IsOverlaped(const Polyline& pl, const Circle& cr) {
  return IsOverlaped(cr, pl);
}

inline AD2 FindIntersection(const std::vector<AD2>& la,
                            const std::vector<AD2>& lb, bool la_inverse = true,
                            bool lb_inverse = true) {
  if (la.size() > lb.size())
    return FindIntersection(lb, la, lb_inverse, la_inverse);
  else {
    auto Cross3D = [](const AD2& a, const AD2& b, double az, double bz) {
      return std::pair<AD2, double>{
          AD2{a[1] * bz - b[1] * az, b[0] * az - a[0] * bz},
          a[0] * b[1] - b[0] * a[1]};
    };
    Polyline La(la);
    Polyline Lb(lb);
    auto& LaSeg = La.line_segs();
    auto& LbSeg = Lb.line_segs();
    double eps = 1e-5;

    for (auto ita = LaSeg.rbegin(); ita != LaSeg.rend(); ita++) {
      for (auto itb = LbSeg.rbegin(); itb != LbSeg.rend(); itb++) {
        if (IsOverlaped(*ita, *itb)) {
          auto a12 = Cross3D(ita->start(), ita->end(), 1, 1);
          auto a34 = Cross3D(itb->start(), itb->end(), 1, 1);
          auto ans = Cross3D(a12.first, a34.first, a12.second, a34.second);
          return AD2{ans.first[0] / ans.second, ans.first[1] / ans.second};
        }
      }
    }
    return AD2{0.0, 0.0};
  }
}

}  // namespace math
}  // namespace planning
}  // namespace neodrive
