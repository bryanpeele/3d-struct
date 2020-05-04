#include "Open3D/Open3D.h"

#include "structify.h"

int main() {
  const auto p0 = magnus::geom::Point3d({0.0, 0.0, 0.0});
  const auto p1 = magnus::geom::Point3d({9.0, 0.0, 0.0});
  const auto p2 = magnus::geom::Point3d({9.0, 9.0, 0.0});
  const auto p3 = magnus::geom::Point3d({0.0, 9.0, 0.0});

  auto graph = magnus::geom::Graph3d();
  graph.AddEdge(p0, p1);
  graph.AddEdge(p1, p2);
  graph.AddEdge(p2, p3);
  graph.AddEdge(p3, p0);
  std::cout << "made graph..." << std::endl;

  std::cout << "num points: " << graph.NumPoints() << std::endl;
  std::cout << "num edges: " << graph.NumEdges() << std::endl;

  for (const auto &point : graph.points()) {
    std::cout << point.x() << ", ";
  }
  std::cout << std::endl;

  for (const auto &edge : graph.edges()) {
    std::cout << "p0x: " << edge.p0().x() << ", p1x: " << edge.p1().x() << std::endl;
  }

  std::cout << "num edgesV2: " << graph.NumEdges() << std::endl;

  magnus::geom::StructifyConfig config;
  config.num_polygon_sides = 15;
  config.struct_radius = 0.5;
  config.vertex_radius = 1.0;
  std::cout << "made config..." << std::endl;

  std::cout << "num edgesV3: " << graph.NumEdges() << std::endl;

  const auto structify = magnus::geom::Structify(config, graph.edges());
  const auto points = structify.Points();
  const auto triangles = structify.TriangleIndices();

  std::vector<Eigen::Vector3d> raw_points;
  for (const auto &point : points) raw_points.push_back(point.coordinate());

  auto mesh_ptr = std::make_shared<open3d::geometry::TriangleMesh>(raw_points, triangles);
  mesh_ptr->ComputeVertexNormals();
  mesh_ptr->ComputeTriangleNormals();
  open3d::visualization::DrawGeometries({mesh_ptr});
  return 0;
}
