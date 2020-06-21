#include <iostream>

#include "Open3D/Open3D.h"

#include "structify.h"
#include "tesselate.h"

int main() {
  const auto tessellate = magnus::geom::Tessellate(7, 3, 6);
  const auto lines = tessellate.Lines();
  auto set = std::make_shared<open3d::geometry::LineSet>(lines.first,
                                                         lines.second);
  open3d::visualization::DrawGeometries({set});

  const auto graph = tessellate.Graph();

  magnus::geom::StructifyConfig config;
  config.num_polygon_sides = 3;
  config.struct_radius = 0.001;
  config.vertex_radius = 0.001;

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
