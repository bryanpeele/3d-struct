#include <iostream>

#include "Open3D/Open3D.h"

#include "structify.h"
#include "tesselate.h"

int main() {
  const auto tessellate = magnus::geom::Tessellate(7, 3, 5);
  const auto lines = tessellate.Lines();
  auto set = std::make_shared<open3d::geometry::LineSet>(lines.first,
                                                         lines.second);
  std::cout << "num vertices: " << lines.first.size() << std::endl;
  std::cout << "num edges: " << lines.second.size() << std::endl;

  open3d::visualization::DrawGeometries({set});

  const auto graph = tessellate.Graph();

  std::cout << "num graph vertices: " << graph.num_points() << std::endl;
  std::cout << "num graph edges: " << graph.num_edges() << std::endl;

  magnus::geom::StructifyConfig config;
  config.num_polygon_sides = 6;
  config.struct_radius = 0.25;
  config.vertex_radius = 0.35;

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

  std::cout << "raw poly-count: " << mesh_ptr->triangles_.size() << std::endl;
  open3d::visualization::DrawGeometries({mesh_ptr});

  auto subdivided_mesh_ptr = mesh_ptr->SubdivideLoop(2);
  subdivided_mesh_ptr->ComputeVertexNormals();
  std::cout << "subdivided poly-count: " << subdivided_mesh_ptr->triangles_.size() << std::endl;
  open3d::visualization::DrawGeometries({subdivided_mesh_ptr});

  auto smooth_mesh_ptr = subdivided_mesh_ptr->FilterSmoothLaplacian(2, 0.5);
  smooth_mesh_ptr->ComputeVertexNormals();
  std::cout << "smooth poly-count: " << smooth_mesh_ptr->triangles_.size() << std::endl;
  open3d::visualization::DrawGeometries({smooth_mesh_ptr});

  return 0;
}
