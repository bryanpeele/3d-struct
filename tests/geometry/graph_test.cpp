#include "graph.h"

#include <gtest/gtest.h>

TEST(Graph, NonidenticalPoints) {
  const auto point0 = magnus::geom::Point3d({6.0, 0.0, 0.0});
  const auto point1 = magnus::geom::Point3d({2.0, 0.0, 0.0});
  const auto point2 = magnus::geom::Point3d({3.0, 0.0, 0.0});
  const auto point3 = magnus::geom::Point3d({9.0, 0.0, 0.0});
  const auto point4 = magnus::geom::Point3d({1.0, 0.0, 0.0});
  const auto point5 = magnus::geom::Point3d({7.0, 0.0, 0.0});

  auto graph = magnus::geom::Graph3d();
  const auto handle0 = graph.AddPoint(point0);
  const auto handle1 = graph.AddPoint(point1);
  const auto handle2 = graph.AddPoint(point2);
  const auto handle3 = graph.AddPoint(point3);
  const auto handle4 = graph.AddPoint(point4);
  const auto handle5 = graph.AddPoint(point5);

  const auto edge0 = graph.AddEdge(handle0, handle1);
  const auto edge1 = graph.AddEdge(handle2, handle3);
  const auto edge2 = graph.AddEdge(handle4, handle5);

  for (const auto &handle : graph.point_handles()) {
    std::cout << graph.point(handle).x() << ", ";
  }
  std::cout << std::endl;

  for (const auto &handle : graph.edge_handles()) {
    std::cout << "p0x: " << graph.edge(handle).first.x()
              << ", p1x: " << graph.edge(handle).second.x() << std::endl;
  }
  std::cout << std::endl;

  std::cout << "edge0: " << graph.edge_length(edge0) << std::endl;
  std::cout << "edge1: " << graph.edge_length(edge1) << std::endl;
  std::cout << "edge2: " << graph.edge_length(edge2) << std::endl;
}
