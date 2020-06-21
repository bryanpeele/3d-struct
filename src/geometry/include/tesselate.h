#pragma once

#include "graph.h"
#include "line_segement.h"

namespace magnus {
namespace geom {


// TODO(bpeele) move to hyperbolic_util.h
Point2d Reflect(const LineSegment2d &line, const Point2d &point);

class Tessellate {
 public:
  Tessellate(int n = 7, int k = 3, int level = 6);

  Graph3d Graph() const;

  std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector2i>> Lines() const;

 private:
  using Polygon2d = std::vector<Point2d>;

  int GetNumInnerPolygons(int num_levels) const;

  void GeneratePolygons();

  Polygon2d ConstructCenterPolygon();

  Polygon2d ConstructNextPolygon(const Polygon2d polygon, int side);

  int ApplyRule(int i, int j);

  const int n_, k_, level_;

  std::vector<Polygon2d> polygons_;
  std::vector<int> rules_;
  int num_inner_polygons_;
};

}  // namespace geom
}  // namespace magnus