#pragma once

#include "point.h"

namespace magnus {
namespace geom {

template <typename T, int N>
class Edge {
 public:
  Edge(Point<T, N>& point_a, Point<T, N>& point_b) : point_a_(point_a),
                                                     point_b_(point_b) {}

  T length() {
    return (point_a_.coordinate() - point_b_.coordinate()).norm();
  }

 private:
  Point<T, N>& point_a_, point_b_;
};

}  // namespace geom
}  // namespace magnus
