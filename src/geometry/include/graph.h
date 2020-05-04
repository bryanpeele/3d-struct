#pragma once

#include <list>

#include "edge.h"
#include "point.h"

namespace magnus {
namespace geom {

template <typename T, int N>
class Graph {
 public:
  Graph() {}

  void AddEdge(const Point<T, N>& point_0, const  Point<T, N>& point_1) {
    const auto &temp_point_0 = AddPoint(point_0);
    const auto &temp_point_1 = AddPoint(point_1);
    edges_.emplace_back(temp_point_0, temp_point_1);
  }

  int NumPoints() const { return points_.size(); }

  int NumEdges() const { return edges_.size(); }

  const std::list<Point<T, N>> &points() const { return points_; }

  const std::vector<Edge<T, N>> &edges() const { return edges_; }

 private:
  const Point<T, N> &AddPoint(const Point<T, N> &point) {
    // if (points_.size() == 0 || point < *points_.begin()) {
    //   points_.push_front(point);
    //   return points_.front();
    // }

    // TODO do something smarter than linear search
    // TODO use equality search to eliminate redundant points....

    for (auto it = points_.begin(); it != points_.end(); it++) {
      if (point < *it) {
        return *points_.insert(it, point);
      }
    }
    points_.push_back(point);
    return points_.back();
  }

  std::list<Point<T, N>> points_;

  std::vector<Edge<T, N>> edges_;
};

using Graph2d = Graph<double, 2>;
using Graph2f = Graph<float, 2>;
using Graph3d = Graph<double, 3>;
using Graph3f = Graph<float, 3>;

}  // namespace geom
}  // namespace magnus
