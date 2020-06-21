#pragma once

#include "point.h"

namespace magnus {
namespace geom {

template <typename T, int N>
class LineSegment {
 public:
  LineSegment(const Point<T, N> &p0, const Point<T, N> &p1) : p0_(p0), p1_(p1) {}

  const Point<T, N> p0() const { return p0_; }

  const Point<T, N> p1() const { return p1_; }

  // TODO(bpeele) length and vector (maybe unit?)

 private:
  Point<T, N> p0_, p1_;
};

using LineSegment2d = LineSegment<double, 2>;
using LineSegment2f = LineSegment<float, 2>;
using LineSegment3d = LineSegment<double, 3>;
using LineSegment3f = LineSegment<float, 3>;

}  // namespace geom
}  // namespace magnus
