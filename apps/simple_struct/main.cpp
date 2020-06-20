#include "Open3D/Open3D.h"

#include "structify.h"


int main() {
  const auto p0 = magnus::geom::Point3d({0.0, 0.0, 0.0});
  const auto p1 = magnus::geom::Point3d({9.0, 0.0, 0.0});
  const auto p2 = magnus::geom::Point3d({9.0, 9.0, 0.0});
  const auto p3 = magnus::geom::Point3d({0.0, 9.0, 0.0});
  const auto p4 = magnus::geom::Point3d({4.5, 4.5, 0.0});
  const auto p5 = magnus::geom::Point3d({4.5, 4.5, 6.0});
  const auto p6 = magnus::geom::Point3d({4.5, 4.5, 12.0});

  auto graph = magnus::geom::Graph3d();
  const auto h0 = graph.AddPoint(p0);
  const auto h1 = graph.AddPoint(p1);
  const auto h2 = graph.AddPoint(p2);
  const auto h3 = graph.AddPoint(p3);
  const auto h4 = graph.AddPoint(p4);
  const auto h5 = graph.AddPoint(p5);
  const auto h6 = graph.AddPoint(p6);

  // Square
  graph.AddEdge(h0, h1);
  graph.AddEdge(h1, h2);
  graph.AddEdge(h2, h3);
  graph.AddEdge(h3, h0);

  // X
  graph.AddEdge(h0, h4);
  graph.AddEdge(h1, h4);
  graph.AddEdge(h2, h4);
  graph.AddEdge(h3, h4);

  // Pyramid
  graph.AddEdge(h0, h5);
  graph.AddEdge(h1, h5);
  graph.AddEdge(h2, h5);
  graph.AddEdge(h3, h5);
  graph.AddEdge(h4, h5);

  // Post
  graph.AddEdge(h5, h6);

  magnus::geom::StructifyConfig config;
  config.num_polygon_sides = 6;
  config.struct_radius = 0.75;
  config.vertex_radius = 2.9;

  const auto structify = magnus::geom::Structify(config, graph);
  const auto points = structify.Points();
  const auto triangles = structify.TriangleIndices();

  std::vector<Eigen::Vector3d> raw_points;
  for (const auto &point : points) raw_points.push_back(point.coordinate());

  auto mesh_ptr = std::make_shared<open3d::geometry::TriangleMesh>(raw_points, triangles);
  mesh_ptr->PaintUniformColor({1, 0.5, 0});
  mesh_ptr->ComputeVertexNormals();
  mesh_ptr->ComputeTriangleNormals();
  mesh_ptr->ComputeAdjacencyList();
  mesh_ptr = std::make_shared<open3d::geometry::TriangleMesh>(mesh_ptr->MergeCloseVertices(1e-6));

  open3d::visualization::DrawGeometries({mesh_ptr});


  auto subdivided_mesh_ptr = mesh_ptr->SubdivideLoop(2);
  subdivided_mesh_ptr->ComputeVertexNormals();
  open3d::visualization::DrawGeometries({subdivided_mesh_ptr});

  auto smooth_mesh_ptr = subdivided_mesh_ptr->FilterSmoothLaplacian(2, 0.5);
  smooth_mesh_ptr->ComputeVertexNormals();
  open3d::visualization::DrawGeometries({smooth_mesh_ptr});

  auto moar_subdivided_mesh_ptr = smooth_mesh_ptr->SubdivideLoop(1);
  moar_subdivided_mesh_ptr->ComputeVertexNormals();
  open3d::visualization::DrawGeometries({moar_subdivided_mesh_ptr});

  return 0;
}
