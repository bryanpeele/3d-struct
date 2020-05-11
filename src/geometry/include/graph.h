#pragma once

#include <utility>

#include "point.h"

namespace magnus {
namespace geom {

template <typename T, int N>
class Graph;

class PointHandle {
 public:
  PointHandle(int i) : point_index(i) {}
  int point_index;

  friend bool operator <(const PointHandle &a, const PointHandle &b) {
    return a.point_index < b.point_index;
  }
};

class EdgeHandle {
 public:
  EdgeHandle(int i) : edge_index(i) {}
  int edge_index;
};

template <typename T, int N>
class Graph {
 public:
  Graph() {}

  int num_points() const { return points_.size(); }

  int num_edges() const { return edges_.size(); }

  PointHandle AddPoint(const Point<T, N> &point) {
    point_handles_.emplace_back(points_.size());
    points_.emplace_back(point);
    return point_handles_.back();
  }

  EdgeHandle AddEdge(const PointHandle &point_0, const PointHandle &point_1) {
    edge_handles_.emplace_back(edges_.size());
    edges_.emplace_back(point_0, point_1);
    return edge_handles_.back();
  }

  const Point<T, N> &point(const PointHandle &point_handle) const {
    return points_[point_handle.point_index];
  }

  const std::pair<PointHandle, PointHandle> &point_handles(const EdgeHandle &edge_handle) const {
    return edges_[edge_handle.edge_index];
  }

  using Edge = typename std::pair<Point<T, N>, Point<T, N>>;

  Edge edge(const EdgeHandle &edge_handle) const {
    const auto &edge = edges_[edge_handle.edge_index];
    const auto &h0 = edge.first;
    const auto &h1 = edge.second;
    return {point(h0), point(h1)};
  }

  double edge_length(const EdgeHandle &edge_handle) const {
    const auto this_edge = edge(edge_handle);
    return (this_edge.first.coordinate() - this_edge.second.coordinate()).norm();
  }

  Eigen::Matrix<T, N, 1> edge_vector(const EdgeHandle &edge_handle) const {
    const auto this_edge = edge(edge_handle);
    return this_edge.second.coordinate() - this_edge.first.coordinate();
  }

  const std::vector<PointHandle> &point_handles() const { return point_handles_; }

  const std::vector<EdgeHandle> &edge_handles() const { return edge_handles_; }

 private:
  std::vector<Point<T, N>> points_;
  std::vector<PointHandle> point_handles_;
  std::vector<std::pair<PointHandle, PointHandle>> edges_;
  std::vector<EdgeHandle> edge_handles_;
};

using Graph2d = Graph<double, 2>;
using Graph2f = Graph<float, 2>;
using Graph3d = Graph<double, 3>;
using Graph3f = Graph<float, 3>;

}  // namespace geom
}  // namespace magnus
