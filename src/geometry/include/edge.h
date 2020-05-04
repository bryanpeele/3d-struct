#pragma once

#include "point.h"

namespace magnus {
namespace geom {

template <typename T, int N>
class Edge {
 public:
  Edge(const Point<T, N>& point_0, const Point<T, N>& point_1) : point_0_(point_0),
                                                                 point_1_(point_1) {}

  const Point<T, N> &p0() const { return point_0_; }
  const Point<T, N> &p1() const { return point_1_; }

  T length() const { return (point_0_.coordinate() - point_1_.coordinate()).norm(); }

  Eigen::Matrix<T, N, 1> vector() const {
    return point_1_.coordinate() - point_0_.coordinate();
  }

 private:
  const Point<T, N> &point_0_;
  const Point<T, N> &point_1_;
};

using Edge2d = Edge<double, 2>;
using Edge2f = Edge<float, 2>;
using Edge3d = Edge<double, 3>;
using Edge3f = Edge<float, 3>;


}  // namespace geom
}  // namespace magnus
