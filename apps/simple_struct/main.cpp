#include "Open3D/Open3D.h"

#include "structify.h"

int main() {
  const auto p0 = magnus::geom::Point3d({0.0, 0.0, 0.0});
  const auto p1 = magnus::geom::Point3d({9.0, 0.0, 0.0});
  const auto p2 = magnus::geom::Point3d({9.0, 9.0, 0.0});
  const auto p3 = magnus::geom::Point3d({0.0, 9.0, 0.0});
  const auto p4 = magnus::geom::Point3d({4.5, 4.5, 0.0});

  auto graph = magnus::geom::Graph3d();
  graph.AddEdge(p0, p1);
  graph.AddEdge(p1, p2);
  graph.AddEdge(p2, p3);
  graph.AddEdge(p3, p0);
  graph.AddEdge(p0, p4);
  graph.AddEdge(p1, p4);
  graph.AddEdge(p2, p4);
  graph.AddEdge(p3, p4);

  magnus::geom::StructifyConfig config;
  config.num_polygon_sides = 3;
  config.struct_radius = 0.3;
  config.vertex_radius = 2.0;

  const auto structify = magnus::geom::Structify(config, graph.edges());
  const auto points = structify.Points();
  const auto triangles = structify.TriangleIndices();

  std::vector<Eigen::Vector3d> raw_points;
  for (const auto &point : points) raw_points.push_back(point.coordinate());

  auto mesh_ptr = std::make_shared<open3d::geometry::TriangleMesh>(raw_points, triangles);
  mesh_ptr->ComputeVertexNormals();
  mesh_ptr->ComputeTriangleNormals();
  mesh_ptr->PaintUniformColor({1, 0.5, 0});
  open3d::visualization::DrawGeometries({mesh_ptr});
  return 0;
}
