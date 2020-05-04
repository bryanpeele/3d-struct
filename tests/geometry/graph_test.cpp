#include "graph.h"

#include <gtest/gtest.h>

#include "math.h"

TEST(Graph, NonidenticalPoints) {
  const auto point0 = magnus::geom::Point3d({6.0, 0.0, 0.0});
  const auto point1 = magnus::geom::Point3d({2.0, 0.0, 0.0});
  const auto point2 = magnus::geom::Point3d({3.0, 0.0, 0.0});
  const auto point3 = magnus::geom::Point3d({9.0, 0.0, 0.0});
  const auto point4 = magnus::geom::Point3d({1.0, 0.0, 0.0});
  const auto point5 = magnus::geom::Point3d({7.0, 0.0, 0.0});

  auto graph = magnus::geom::Graph3d();
  graph.AddEdge(point0, point1);
  graph.AddEdge(point2, point3);
  graph.AddEdge(point4, point5);

  for (const auto &point : graph.points()) {
    std::cout << point.x() << ", ";
  }
  std::cout << std::endl;

  for (const auto &edge : graph.edges()) {
    std::cout << "p0x: " << edge.p0().x() << ", p1x: " << edge.p1().x() << std::endl;
  }

  std::cout << std::endl;
}
